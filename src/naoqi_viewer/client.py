"""The caller's side: a `NaoqiViewer`, built from a config, starts and drives a viewer process.

Standard library and numpy only: importing naoqi_viewer loads neither mujoco nor qi.
"""

import json
import logging
import os
import queue
import socket
import subprocess
import sys
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import IO, Any, Literal, Self

import numpy as np

from naoqi_viewer import protocol
from naoqi_viewer.config import NaoqiViewerConfig
from naoqi_viewer.props import (
    DEFAULT_OBJECT_COLOR,
    DEFAULT_OBJECT_SIZE,
    Frame,
    Kind,
    PropError,
    PropState,
    Shape,
)

_process_log = logging.getLogger("naoqi_viewer.viewer_process")
_LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}
_STDERR_TAIL_LINES = 50
_STOP_TIMEOUT = 5.0  # s for the viewer process to exit after `stop`
_KILL_TIMEOUT = 2.0

Camera = Literal["top", "bottom"]


class LaunchError(RuntimeError):
    """The viewer process couldn't start (the message includes the tail of its stderr)."""


class ViewerClosed(Exception):
    """The viewer has exited (window closed, `close()`, or a crash)."""


class ModeError(Exception):
    """The request isn't available in this viewer's mode."""


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray  # (height, width, 3) uint8 RGB, row 0 = top of the image
    camera: str
    pose_seq: int  # Sample.seq the frame was rendered at (0: no pose yet)
    pose_age: float | None  # data age of that pose, s (None: no pose yet)


@dataclass(frozen=True)
class ViewerStatus:
    mode: str  # mirror / sim
    target: str | None  # real / naoqi-sim / virtual; None until NAOqi first connects
    naoqi_version: str | None
    url: str
    variant: str  # placeholder / aldebaran
    rate: float  # pose updates per second
    pose_seq: int
    data_age: float | None  # s; None before the first sample


def _floats(value: Any) -> Any:
    """A position, quaternion, size or colour as JSON: numbers and sequences of them."""
    if value is None or isinstance(value, int | float):
        return value
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return value  # the viewer process refuses it with a PropError naming it


class PropsClient:
    """The props of a sim-mode viewer (props.md): each call is one request, serialized with the
    viewer's other calls. Positions are given in the scene's frame (`frame="world"`, NAOqi's
    FRAME_WORLD) or the robot's (`frame="robot"`, FRAME_ROBOT at the robot's pose when the call
    is answered); the states returned are always in the scene's frame."""

    def __init__(self, viewer: "NaoqiViewer") -> None:
        self._viewer = viewer

    def list(self) -> dict[str, PropState]:
        header, _ = self._viewer._prop_request("props")
        return {s["name"]: PropState.from_dict(s) for s in header["props"]}

    def spawn(
        self,
        kind: Kind,
        pos: Any,
        *,
        name: str | None = None,
        quat: Any = None,
        frame: Frame = "world",
        duration: float = 0.0,
        shape: Shape = "sphere",
        size: Any = DEFAULT_OBJECT_SIZE,
        color: Any = DEFAULT_OBJECT_COLOR,
    ) -> PropState:
        fields: dict[str, Any] = {
            "kind": kind,
            "pos": _floats(pos),
            "frame": frame,
            "duration": duration,
        }
        if name is not None:
            fields["name"] = name
        if quat is not None:
            fields["quat"] = _floats(quat)
        if kind == "object":
            fields.update(shape=shape, size=_floats(size), color=_floats(color))
        header, _ = self._viewer._prop_request("prop_spawn", **fields)
        return PropState.from_dict(header["prop"])

    def place(
        self,
        name: str,
        pos: Any = None,
        quat: Any = None,
        *,
        frame: Frame = "world",
        duration: float = 0.0,
    ) -> PropState:
        fields: dict[str, Any] = {"name": name, "frame": frame, "duration": duration}
        if pos is not None:
            fields["pos"] = _floats(pos)
        if quat is not None:
            fields["quat"] = _floats(quat)
        header, _ = self._viewer._prop_request("prop_place", **fields)
        return PropState.from_dict(header["prop"])

    def despawn(self, name: str) -> PropState:
        header, _ = self._viewer._prop_request("prop_despawn", name=name)
        return PropState.from_dict(header["prop"])

    def clear(self) -> dict[str, PropState]:
        header, _ = self._viewer._prop_request("props_clear")
        return {s["name"]: PropState.from_dict(s) for s in header["props"]}


def _viewer_command(config: NaoqiViewerConfig) -> list[str]:
    """The command running the viewer process (tests replace it with a fake viewer process)."""
    if sys.platform == "darwin" and not config.headless:
        # MuJoCo's window must run on the main thread of an mjpython process on macOS.
        python = Path(sys.executable).with_name("mjpython")
        if not python.exists():
            raise LaunchError(
                f"MuJoCo's window needs mjpython on macOS, and there is none next to {sys.executable}; "
                "it comes with the mujoco package, so install naoqi-viewer into this Python environment"
            )
    else:
        python = Path(sys.executable)
    return [
        str(python),
        "-m",
        "naoqi_viewer.viewer_process",
        json.dumps(config.to_dict()),
    ]


def _viewer_env(config: NaoqiViewerConfig) -> dict[str, str]:
    """The viewer process's environment: ours, with its pipes in UTF-8.

    Its output is read as UTF-8 whatever the locale (Windows' is a legacy code page that can't
    print every path). A headless viewer on Linux renders through EGL unless MUJOCO_GL says
    otherwise: with Mesa's software device it needs no display and no GPU.
    """
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
    if config.headless and sys.platform == "linux" and "MUJOCO_GL" not in os.environ:
        env["MUJOCO_GL"] = "egl"
    return env


class _ProcessOutput:
    """Drains the viewer process's stdout and stderr on threads; relays stderr to logging."""

    def __init__(self, process: subprocess.Popen[str]) -> None:
        self.protocol_line: queue.Queue[str | None] = queue.Queue(maxsize=1)
        self._tail: deque[str] = deque(maxlen=_STDERR_TAIL_LINES)
        assert process.stdout is not None and process.stderr is not None
        self._stdout = threading.Thread(
            target=self._read_stdout, args=(process.stdout,), daemon=True
        )
        self._stderr = threading.Thread(
            target=self._read_stderr, args=(process.stderr,), daemon=True
        )
        self._stdout.start()
        self._stderr.start()

    def stderr_tail(self, wait: float = 0.0) -> str:
        """The last stderr lines, after waiting up to `wait` s for the stream to end."""
        self._stderr.join(wait)
        return "\n".join(self._tail)

    def _read_stdout(self, stream: IO[str]) -> None:
        # libqi writes its own log lines to stdout, so the ready (or error) line isn't
        # necessarily the first one: pick it out and relay everything else.
        announced = False
        for raw in stream:
            line = raw.rstrip("\n")
            if not announced and line.startswith(
                (protocol.READY_PREFIX, protocol.ERROR_PREFIX)
            ):
                self.protocol_line.put(line)
                announced = True
            else:
                _process_log.debug("%s", line)
        if not announced:
            self.protocol_line.put(None)  # exited without a word

    def _read_stderr(self, stream: IO[str]) -> None:
        for raw in stream:
            line = raw.rstrip("\n")
            self._tail.append(line)
            level_name, _, message = line.partition(" ")
            level = _LEVELS.get(level_name)
            if level is None:  # MuJoCo's or qi's own prints, a native crash
                _process_log.debug("%s", line)
            else:
                _process_log.log(level, "%s", message)


def _stop_process(process: subprocess.Popen[str], grace: float) -> None:
    try:
        process.wait(grace)
        return
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(_KILL_TIMEOUT)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def _with_stderr(message: str, output: _ProcessOutput) -> str:
    tail = output.stderr_tail(wait=1.0)
    return f"{message}\n--- viewer process stderr ---\n{tail}" if tail else message


@dataclass
class _Running:
    """A launched viewer process and the connection to it."""

    process: subprocess.Popen[str]
    connection: socket.socket | None
    output: _ProcessOutput


class NaoqiViewer:
    """A viewer on a NAO, built from a `NaoqiViewerConfig`; `launch()` opens its window.

    The constructor has no side effects. One NaoqiViewer is one window: `launch()` again after the
    window was closed opens a new one. Calls from several threads are serialized.
    """

    def __init__(self, config: NaoqiViewerConfig | None = None) -> None:
        self._config = config if config is not None else NaoqiViewerConfig()
        self._running: _Running | None = None
        self._lock = threading.Lock()
        self._next_id = 0

    @classmethod
    def from_dict(cls, data: Any) -> Self:
        return cls(NaoqiViewerConfig.from_dict(data))

    @classmethod
    def from_json(cls, text: str) -> Self:
        return cls(NaoqiViewerConfig.from_json(text))

    @classmethod
    def from_json_file(cls, path: str | Path) -> Self:
        return cls(NaoqiViewerConfig.from_json_file(path))

    @property
    def config(self) -> NaoqiViewerConfig:
        return self._config

    def launch(self) -> None:
        """Start the viewer process; return once its window is up (headless: once it renders).

        It returns even if NAOqi is unreachable: the viewer keeps reconnecting. Raises LaunchError
        if the viewer process can't start (missing scene file or meshes, no display, no offscreen
        OpenGL for a headless viewer, timeout), and RuntimeError if this viewer is already running.
        """
        if self.running:
            raise RuntimeError(
                "this viewer is already running; close() it first, or use another NaoqiViewer"
            )
        self.close()  # drop what's left of a previous run
        timeout = self._config.launch_timeout_s
        process = subprocess.Popen(
            _viewer_command(self._config),
            env=_viewer_env(self._config),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding="utf-8",
            errors="replace",  # libqi's own log lines are not ours to trust
            bufsize=1,
        )
        output = _ProcessOutput(process)
        try:
            line = output.protocol_line.get(timeout=timeout)
        except queue.Empty:
            _stop_process(process, grace=0)
            raise LaunchError(
                _with_stderr(
                    f"the viewer process wasn't ready within {timeout:g} s", output
                )
            ) from None
        if line is None or not line.startswith(protocol.READY_PREFIX):
            _stop_process(process, grace=_KILL_TIMEOUT)
            if line is None:
                message = f"the viewer process exited (code {process.returncode}) before it was ready"
            else:
                message = line.removeprefix(protocol.ERROR_PREFIX).strip()
            raise LaunchError(_with_stderr(message, output))

        fields = dict(field.split("=", 1) for field in line.split()[1:])
        connection = socket.create_connection(
            ("127.0.0.1", int(fields["port"])), timeout=timeout
        )
        connection.settimeout(None)
        with self._lock:
            self._running = _Running(process, connection, output)
        try:
            hello, _ = self._request("hello")
        except (ViewerClosed, RuntimeError) as exc:
            self.close()
            raise LaunchError(
                _with_stderr(f"the viewer process didn't answer hello: {exc}", output)
            ) from exc
        if hello.get("protocol") != protocol.PROTOCOL:
            self.close()
            raise LaunchError(
                f"the viewer process speaks protocol {hello.get('protocol')}, not {protocol.PROTOCOL}"
            )

    def camera_frame(self, camera: Camera, width: int, height: int) -> CameraFrame:
        """Render what a head camera sees in the simulated world (sim mode only)."""
        if self._config.mode != "sim":
            raise ModeError(
                "camera_frame is only available in sim mode; a real robot's cameras are served by ALVideoDevice"
            )
        if camera not in ("top", "bottom"):
            raise ValueError(f"unknown camera {camera!r}; expected 'top' or 'bottom'")
        header, payload = self._request(
            "camera_frame", camera=camera, width=width, height=height
        )
        image = np.frombuffer(payload, dtype=header["dtype"]).reshape(header["shape"])
        return CameraFrame(
            image, header["camera"], header["pose_seq"], header["pose_age"]
        )

    @property
    def props(self) -> PropsClient:
        """The props of the simulated world (sim mode only, props.md)."""
        return PropsClient(self)

    def _prop_request(
        self, op: str, **fields: Any
    ) -> tuple[protocol.Header, bytearray]:
        if self._config.mode != "sim":
            raise ModeError(
                "props are only available in sim mode: a real robot's cameras cannot see them"
            )
        return self._request(op, **fields)

    def status(self) -> ViewerStatus:
        header, _ = self._request("status")
        return ViewerStatus(
            **{name: header[name] for name in ViewerStatus.__dataclass_fields__}
        )

    def wait(self, timeout: float | None = None) -> bool:
        """Wait for the viewer to exit (its window closed); True once it has, or if never launched."""
        running = self._running
        if running is None:
            return True
        try:
            running.process.wait(timeout)
        except subprocess.TimeoutExpired:
            return False
        return True

    @property
    def running(self) -> bool:
        running = self._running
        return running is not None and running.process.poll() is None

    def close(self) -> None:
        """Stop the viewer: ask it to exit, then terminate it if it doesn't within 5 s. Harmless
        when it isn't running."""
        with self._lock:
            running, self._running = self._running, None
        if running is None:
            return
        if running.connection is not None:
            try:
                protocol.write_message(running.connection, {"op": "stop", "id": 0})
                protocol.read_message(running.connection)
            except (OSError, EOFError):
                pass  # already gone
            running.connection.close()
        _stop_process(running.process, grace=_STOP_TIMEOUT)

    def __enter__(self) -> Self:
        if not self.running:
            self.launch()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _request(self, op: str, **fields: Any) -> tuple[protocol.Header, bytearray]:
        with self._lock:
            running = self._running
            if running is None or running.connection is None:
                raise ViewerClosed("the viewer isn't running; launch() it first")
            self._next_id += 1
            try:
                protocol.write_message(
                    running.connection, {"op": op, "id": self._next_id, **fields}
                )
                header, payload = protocol.read_message(running.connection)
            except (OSError, EOFError) as exc:
                running.connection.close()
                running.connection = None
                raise ViewerClosed("the viewer has exited") from exc
        if not header.get("ok"):
            error = header.get("error")
            if error == "mode":
                raise ModeError(f"{op} isn't available in {self._config.mode} mode")
            if isinstance(error, str) and error.startswith("prop: "):
                raise PropError(error.removeprefix("prop: "))
            raise RuntimeError(f"the viewer refused {op}: {error}")
        return header, payload
