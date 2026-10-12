"""The viewer process: `python -m naoqi_viewer.viewer_process <config JSON>` (on macOS, under mjpython).

Started by naoqi_viewer.launch(). It loads the world, polls NAOqi, owns the window (or, headless,
renders offscreen only), and answers its caller over one loopback connection (protocol.py). It
exits when that connection closes, when the window is closed, or on `stop`.
"""

import logging
import os
import queue
import socket
import sys
import threading
import time
from collections.abc import Callable
from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as package_version
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from naoqi_viewer import protocol
from naoqi_viewer.config import ConfigError, NaoqiViewerConfig, is_scene_path
from naoqi_viewer.model import load_world, resolve_variant
from naoqi_viewer.source import NaoqiSource, PoseSource, Sample
from naoqi_viewer.viewer import ATTRIBUTION, run, run_headless

_log = logging.getLogger(__name__)

CAMERAS = {"top": "CameraTop", "bottom": "CameraBottom"}
MAX_IMAGE_SIDE = 4096

# Renders `camera` at width x height from the posed model: an (height, width, 3) uint8 RGB image.
RenderFunction = Callable[[mujoco.MjModel, mujoco.MjData, str, int, int], np.ndarray]


def _describe(exc: BaseException) -> str:
    """The exception's text; PyOpenGL errors can fail `str()`, so fall back to its type."""
    try:
        text = str(exc)
    except Exception:  # noqa: BLE001  # describing an error must not raise another
        text = ""
    return f"{type(exc).__name__}: {text}" if text else type(exc).__name__


def _check_offscreen(model: mujoco.MjModel, renderer: "OffscreenRenderer") -> None:
    """Render one frame, so a headless viewer that can't render fails at launch, not later.

    640x480 is a common request size, so the renderer it creates is kept.
    """
    renderer(model, mujoco.MjData(model), CAMERAS["top"], 640, 480)


def _naoqi_viewer_version() -> str:
    try:
        return package_version("naoqi-viewer")
    except PackageNotFoundError:
        return "unknown"


class OffscreenRenderer:
    """One mujoco.Renderer per resolution, created on first use. Use from a single thread."""

    def __init__(self) -> None:
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}

    def __call__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        camera: str,
        width: int,
        height: int,
    ) -> np.ndarray:
        renderer = self._renderers.get((width, height))
        if renderer is None:
            # The offscreen buffer must hold the image; the model's default is 640x480.
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
            model.vis.global_.offheight = max(model.vis.global_.offheight, height)
            renderer = mujoco.Renderer(model, height=height, width=width)
            self._renderers[(width, height)] = renderer
        renderer.update_scene(data, camera=camera)
        return renderer.render()

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()


class RequestServer:
    """Reads requests from the caller's connection and answers them from the render loop."""

    def __init__(
        self,
        listener: socket.socket,
        *,
        mode: str,
        url: str,
        variant: str,
        source: PoseSource,
        render: RenderFunction | None,
        accept_timeout: float,
        announce: Callable[[str], None] = lambda line: print(line, flush=True),
    ) -> None:
        self.stop = threading.Event()
        self.ready = False
        self._listener = listener
        self._mode = mode
        self._url = url
        self._variant = variant
        self._source = source
        self._render = render
        self._accept_timeout = accept_timeout
        self._announce = announce
        self._requests: queue.Queue[protocol.Header] = queue.Queue()
        self._connection: socket.socket | None = None

    @property
    def port(self) -> int:
        return self._listener.getsockname()[1]

    def on_frame(
        self, model: mujoco.MjModel, data: mujoco.MjData, sample: Sample | None
    ) -> None:
        """The viewer's on_frame hook: announce readiness once, then answer queued requests."""
        if not self.ready:
            self.ready = True
            self._announce(protocol.ready_line(self.port))
            threading.Thread(
                target=self._serve, name="viewer requests", daemon=True
            ).start()
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            header, payload = self.handle(model, data, sample, request)
            self._send(header, payload)

    def handle(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        sample: Sample | None,
        request: protocol.Header,
    ) -> tuple[protocol.Header, bytes]:
        op, request_id = request.get("op"), request.get("id")

        def ok(**fields: Any) -> protocol.Header:
            return {"id": request_id, "ok": True, **fields}

        def error(message: str) -> protocol.Header:
            return {"id": request_id, "ok": False, "error": message}

        if op == "hello":
            return ok(
                protocol=protocol.PROTOCOL,
                naoqi_viewer=_naoqi_viewer_version(),
                mode=self._mode,
            ), b""
        if op == "status":
            return ok(**self._status()), b""
        if op == "camera_frame":
            if self._render is None:
                return error("mode"), b""
            camera, width, height = (
                request.get("camera"),
                request.get("width"),
                request.get("height"),
            )
            if camera not in CAMERAS:
                return error(
                    f"unknown camera {camera!r}; expected 'top' or 'bottom'"
                ), b""
            if not all(
                isinstance(v, int) and 0 < v <= MAX_IMAGE_SIDE for v in (width, height)
            ):
                return error(f"bad image size {width}x{height}"), b""
            assert isinstance(width, int) and isinstance(height, int)
            image = np.ascontiguousarray(
                self._render(model, data, CAMERAS[camera], width, height)
            )
            header = ok(
                shape=list(image.shape),
                dtype=str(image.dtype),
                camera=camera,
                pose_seq=sample.seq if sample is not None else 0,
                pose_age=time.monotonic() - sample.received_at
                if sample is not None
                else None,
            )
            return header, image.tobytes()
        if op == "stop":
            self.stop.set()
            return ok(), b""
        return error(f"unknown op {op!r}"), b""

    def close(self) -> None:
        self.stop.set()
        self._listener.close()
        if self._connection is not None:
            self._connection.close()

    def _status(self) -> dict[str, Any]:
        info = self._source.info
        latest = self._source.latest()
        return {
            "mode": self._mode,
            "target": info.target if info is not None else None,
            "naoqi_version": info.naoqi_version if info is not None else None,
            "url": self._url,
            "variant": self._variant,
            "rate": self._source.rate(),
            "pose_seq": latest.seq if latest is not None else 0,
            "data_age": time.monotonic() - latest.received_at
            if latest is not None
            else None,
        }

    def _serve(self) -> None:
        """Accept the caller's one connection, then queue its requests until it closes."""
        self._listener.settimeout(self._accept_timeout)
        try:
            connection, _ = self._listener.accept()
        except TimeoutError:
            _log.error(
                "no caller connected within %.0f s; exiting", self._accept_timeout
            )
            self.stop.set()
            return
        except OSError:
            return  # closed while waiting
        finally:
            self._listener.close()
        connection.settimeout(None)
        self._connection = connection
        try:
            while not self.stop.is_set():
                header, _ = protocol.read_message(connection)
                self._requests.put(header)
        except (EOFError, OSError):
            pass  # the caller closed the connection, or exited: the viewer goes with it
        self.stop.set()

    def _send(self, header: protocol.Header, payload: bytes) -> None:
        if self._connection is None:
            return
        try:
            protocol.write_message(self._connection, header, payload)
        except OSError:
            self.stop.set()


def main(argv: list[str] | None = None) -> int:
    """Run a viewer from its config (`NaoqiViewerConfig.to_dict()` as JSON, the one argument)."""
    args = sys.argv[1:] if argv is None else argv
    logging.basicConfig(
        stream=sys.stderr,
        level=logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    def fail(message: str) -> int:
        print(f"{protocol.ERROR_PREFIX} {' '.join(message.split())}", flush=True)
        return 1

    try:
        config = NaoqiViewerConfig.from_json(args[0])
    except ConfigError as exc:
        return fail(f"invalid viewer config: {exc}")
    mode, url, scene = config.mode, config.naoqi.url, config.world.scene
    if is_scene_path(scene) and not Path(scene).is_file():
        return fail(f"scene file not found: {scene}")
    try:
        variant = resolve_variant(config.world.variant)
        model = load_world(scene, variant, name=f"naoqi-viewer · {mode} · {url}")
    except (ValueError, FileNotFoundError) as exc:
        return fail(str(exc))

    renderer = OffscreenRenderer() if mode == "sim" else None
    if config.headless:
        assert renderer is not None  # the config allows headless in sim mode only
        try:
            _check_offscreen(model, renderer)
        except Exception as exc:  # noqa: BLE001  # GL backends fail in many ways; all mean "can't render"
            renderer.close()
            gl = os.environ.get("MUJOCO_GL")
            backend = f"MUJOCO_GL={gl}" if gl else "MuJoCo's default OpenGL backend"
            return fail(
                f"a headless viewer cannot render offscreen with {backend} "
                f"({_describe(exc)}); on Linux install Mesa's EGL (libegl1 libopengl0 libgl1-mesa-dri on "
                "Debian/Ubuntu), or set MUJOCO_GL=osmesa with libosmesa6"
            )

    listener = socket.create_server(("127.0.0.1", 0))
    source = NaoqiSource(url, rate_hz=config.naoqi.rate_hz, commanded=config.ghost)
    server = RequestServer(
        listener,
        mode=mode,
        url=url,
        variant=variant,
        source=source,
        render=renderer,
        accept_timeout=config.launch_timeout_s,
    )
    try:
        if config.headless:
            run_headless(model, source, on_frame=server.on_frame, stop=server.stop)
        else:
            run(
                model,
                source,
                ghost=config.ghost,
                attribution=ATTRIBUTION if variant == "aldebaran" else None,
                on_frame=server.on_frame,
                stop=server.stop,
            )
    except Exception as exc:
        if not server.ready:  # the window never opened (no display, no mjpython)
            return fail(f"the viewer window could not open: {_describe(exc)}")
        raise
    finally:
        server.close()
        source.close()
        if renderer is not None:
            renderer.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
