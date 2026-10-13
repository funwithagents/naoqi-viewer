import json
import os
import socket
import subprocess
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass
from typing import NoReturn

import mujoco
import numpy as np
import pytest
from mock_naoqi import MockNaoqi

from naoqi_viewer import (
    NaoqiSettings,
    NaoqiViewer,
    NaoqiViewerConfig,
    WorldSettings,
    client,
    protocol,
    viewer_process,
)
from naoqi_viewer.model import NaoPose, PoseWriter, load_world
from naoqi_viewer.source import Sample, TargetInfo
from naoqi_viewer.viewer_process import OffscreenRenderer, RequestServer, main

URL = "tcp://127.0.0.1:9559"


def no_offscreen_gl(reason: str) -> NoReturn:
    """Skip a test that needs offscreen OpenGL, or fail it where the run requires it (CI sets
    NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL=1, so a runner that lost its GL fails instead of passing)."""
    if os.environ.get("NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL"):
        pytest.fail(f"offscreen OpenGL is required on this run, and {reason}")
    pytest.skip(reason)


@dataclass
class FakeSource:
    info: TargetInfo | None = None
    sample: Sample | None = None

    def latest(self) -> Sample | None:
        return self.sample

    def rate(self) -> float:
        return 48.0

    def close(self) -> None:
        pass


def stub_render(
    model: mujoco.MjModel, data: mujoco.MjData, camera: str, width: int, height: int
) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[..., 2] = 1 if camera == "CameraTop" else 2
    return image


@pytest.fixture(scope="module")
def world() -> mujoco.MjModel:
    return load_world("table", variant="placeholder")


class Harness:
    """A RequestServer with its render loop driven by hand, and the caller's socket."""

    def __init__(
        self, world: mujoco.MjModel, mode: str, accept_timeout: float = 5.0
    ) -> None:
        self.model, self.data = world, mujoco.MjData(world)
        self.source = FakeSource()
        self.sample: Sample | None = None
        self.announced: list[str] = []
        self.server = RequestServer(
            socket.create_server(("127.0.0.1", 0)),
            mode=mode,
            url=URL,
            variant="placeholder",
            source=self.source,
            render=stub_render if mode == "sim" else None,
            accept_timeout=accept_timeout,
            announce=self.announced.append,
        )
        self.caller: socket.socket | None = None

    def frame(self) -> None:
        self.server.on_frame(self.model, self.data, self.sample)

    def connect(self) -> socket.socket:
        self.frame()  # the first frame announces readiness and starts accepting
        self.caller = socket.create_connection(("127.0.0.1", self.server.port))
        return self.caller

    def request(self, **header: object) -> tuple[protocol.Header, bytearray]:
        assert self.caller is not None
        protocol.write_message(self.caller, header)
        # Requests are answered on the render loop: run frames until the reply is there.
        self.caller.settimeout(0.01)
        deadline = time.monotonic() + 5
        while True:
            self.frame()
            try:
                self.caller.recv(1, socket.MSG_PEEK)
                break
            except TimeoutError:
                if time.monotonic() > deadline:
                    raise AssertionError("no reply") from None
        self.caller.settimeout(None)
        return protocol.read_message(self.caller)

    def close(self) -> None:
        if self.caller is not None:
            self.caller.close()
        self.server.close()


@pytest.fixture
def sim(world: mujoco.MjModel) -> Iterator[Harness]:
    harness = Harness(world, "sim")
    harness.connect()
    yield harness
    harness.close()


def test_ready_line_on_the_first_frame_only(world: mujoco.MjModel):
    harness = Harness(world, "mirror")
    try:
        assert harness.announced == []
        harness.frame()
        harness.frame()
        assert harness.announced == [
            f"NAOQI_VIEWER_READY port={harness.server.port} protocol=1"
        ]
    finally:
        harness.close()


def test_hello_and_status(sim: Harness):
    header, _ = sim.request(op="hello", id=1)
    assert header["ok"] and header["id"] == 1
    assert (header["protocol"], header["mode"]) == (1, "sim")

    header, _ = sim.request(op="status", id=2)
    assert (
        header["target"],
        header["naoqi_version"],
        header["data_age"],
        header["pose_seq"],
    ) == (None, None, None, 0)

    sim.source.info = TargetInfo(URL, "naoqi-sim", "0.3.0")
    sim.source.sample = Sample(
        9,
        NaoPose({}, (0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0)),
        None,
        time.monotonic() - 0.05,
    )
    header, _ = sim.request(op="status", id=3)
    assert (
        header["mode"],
        header["target"],
        header["naoqi_version"],
        header["url"],
    ) == ("sim", "naoqi-sim", "0.3.0", URL)
    assert (header["variant"], header["rate"], header["pose_seq"]) == (
        "placeholder",
        48.0,
        9,
    )
    assert 0.05 <= header["data_age"] < 1.0


def test_camera_frame_reports_the_pose_it_was_rendered_with(sim: Harness):
    header, payload = sim.request(
        op="camera_frame", id=4, camera="bottom", width=64, height=48
    )
    assert header["ok"] and (header["pose_seq"], header["pose_age"]) == (0, None)
    image = np.frombuffer(payload, dtype=header["dtype"]).reshape(header["shape"])
    assert image.shape == (48, 64, 3) and image[0, 0, 2] == 2  # CameraBottom

    sim.sample = Sample(
        5,
        NaoPose({}, (0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0)),
        None,
        time.monotonic() - 0.03,
    )
    header, _ = sim.request(op="camera_frame", id=5, camera="top", width=64, height=48)
    assert header["pose_seq"] == 5 and 0.03 <= header["pose_age"] < 1.0


def test_bad_camera_requests_are_refused(sim: Harness):
    header, _ = sim.request(op="camera_frame", id=6, camera="left", width=64, height=48)
    assert not header["ok"] and "left" in header["error"]
    header, _ = sim.request(op="camera_frame", id=7, camera="top", width=0, height=48)
    assert not header["ok"] and "size" in header["error"]


def test_mirror_mode_refuses_camera_frames(world: mujoco.MjModel):
    harness = Harness(world, "mirror")
    try:
        harness.connect()
        header, payload = harness.request(
            op="camera_frame", id=1, camera="top", width=64, height=48
        )
        assert (header["ok"], header["error"], payload) == (False, "mode", b"")
    finally:
        harness.close()


def test_unknown_ops_are_refused_without_closing_the_connection(sim: Harness):
    header, _ = sim.request(op="touch", id=8)
    assert not header["ok"] and "touch" in header["error"]
    header, _ = sim.request(op="hello", id=9)
    assert header["ok"]
    assert not sim.server.stop.is_set()


def test_requests_wait_for_the_render_loop(sim: Harness):
    assert sim.caller is not None
    protocol.write_message(sim.caller, {"op": "hello", "id": 10})
    time.sleep(0.1)
    sim.caller.settimeout(0.05)
    with pytest.raises(TimeoutError):
        sim.caller.recv(1, socket.MSG_PEEK)
    sim.caller.settimeout(None)
    sim.frame()
    header, _ = protocol.read_message(sim.caller)
    assert header["id"] == 10


def test_stop_answers_then_stops(sim: Harness):
    header, _ = sim.request(op="stop", id=11)
    assert header["ok"]
    assert sim.server.stop.is_set()


def test_the_caller_closing_its_connection_stops_the_viewer(sim: Harness):
    assert sim.caller is not None
    sim.caller.close()
    sim.caller = None
    assert sim.server.stop.wait(5)


def test_no_caller_within_the_timeout_stops_the_viewer(world: mujoco.MjModel):
    harness = Harness(world, "mirror", accept_timeout=0.2)
    try:
        harness.frame()
        assert harness.server.stop.wait(5)
    finally:
        harness.close()


def test_offscreen_renderer_sees_the_table_from_both_cameras(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    PoseWriter(world).apply(data, NaoPose({}, (0.0, 0.0, 0.3332), (1.0, 0.0, 0.0, 0.0)))
    renderer = OffscreenRenderer()
    try:
        try:
            top = renderer(
                world, data, "CameraTop", 800, 600
            )  # larger than the default buffer
        except Exception as exc:  # noqa: BLE001
            no_offscreen_gl(f"no OpenGL for offscreen rendering here: {exc}")
        bottom = renderer(world, data, "CameraBottom", 320, 240)
    finally:
        renderer.close()
    assert top.shape == (600, 800, 3) and bottom.shape == (240, 320, 3)
    # The bottom camera looks down at the table: much of its image is the table's wood color.
    wood = np.array([0.62, 0.45, 0.3]) * 255
    near_wood = np.linalg.norm(bottom.astype(float) - wood, axis=2) < 60
    assert near_wood.mean() > 0.2
    assert top.std() > 10  # not blank


def test_an_invalid_config_is_a_startup_error(capsys: pytest.CaptureFixture[str]):
    assert main(['{"naoqi": {"rate_hz": 0}}']) == 1
    out = capsys.readouterr().out.strip()
    assert out.startswith("NAOQI_VIEWER_ERROR invalid viewer config: naoqi.rate_hz")


def test_a_missing_scene_file_is_a_startup_error(
    capsys: pytest.CaptureFixture[str], tmp_path
):
    missing = tmp_path / "lab.xml"
    assert main([json.dumps({"world": {"scene": str(missing)}})]) == 1
    assert (
        capsys.readouterr().out.strip()
        == f"NAOQI_VIEWER_ERROR scene file not found: {missing}"
    )


def test_a_headless_viewer_that_cannot_render_is_a_startup_error(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
):
    def no_gl(model: mujoco.MjModel, renderer: OffscreenRenderer) -> None:
        raise RuntimeError("no OpenGL context")

    monkeypatch.setattr(viewer_process, "_check_offscreen", no_gl)
    monkeypatch.setenv("MUJOCO_GL", "egl")
    assert main(['{"mode": "sim", "headless": true}']) == 1
    out = capsys.readouterr().out.strip()
    assert out.startswith(
        "NAOQI_VIEWER_ERROR a headless viewer cannot render offscreen with MUJOCO_GL=egl "
        "(RuntimeError: no OpenGL context)"
    )
    assert "libegl1 libopengl0 libgl1-mesa-dri" in out


# --- A real headless viewer process -------------------------------------------------

_PROBE = """
import mujoco
model = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='.1'/></worldbody></mujoco>")
renderer = mujoco.Renderer(model, 48, 64)
renderer.update_scene(mujoco.MjData(model))
renderer.render()
"""


def headless_config(url: str) -> NaoqiViewerConfig:
    return NaoqiViewerConfig(
        mode="sim",
        headless=True,
        naoqi=NaoqiSettings(url=url),
        world=WorldSettings(scene="table", variant="placeholder"),
    )


@pytest.fixture(scope="module")
def offscreen_gl() -> None:
    """Skip unless this machine renders offscreen with the environment launch() gives a viewer."""
    env = client._viewer_env(headless_config("tcp://127.0.0.1:9559")) or dict(
        os.environ
    )
    result = subprocess.run(
        [sys.executable, "-c", _PROBE],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    if result.returncode != 0:
        tail = result.stderr.strip().splitlines()[-1:] or ["no output"]
        no_offscreen_gl(
            f"no offscreen OpenGL here ({tail[0]}); on Linux install libegl1 libopengl0 libgl1-mesa-dri"
        )


def wait_for_pose(viewer: NaoqiViewer, after: int = 0) -> int:
    deadline = time.monotonic() + 10
    while (seq := viewer.status().pose_seq) <= after:
        assert time.monotonic() < deadline, "no pose from the mock NAOqi"
        time.sleep(0.05)
    return seq


@pytest.mark.usefixtures("offscreen_gl")
def test_a_headless_viewer_serves_frames_that_follow_the_robot(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    mock.set_pose({"HeadYaw": 0.0, "HeadPitch": 0.2})
    with NaoqiViewer(headless_config(url)) as viewer:
        seq = wait_for_pose(viewer)
        status = viewer.status()
        assert (status.mode, status.target, status.variant) == (
            "sim",
            "virtual",
            "placeholder",
        )
        top = viewer.camera_frame("top", 640, 480)
        before = viewer.camera_frame("bottom", 320, 240)
        assert top.image.shape == (480, 640, 3) and before.image.shape == (240, 320, 3)
        assert top.image.std() > 10 and before.image.std() > 10  # not blank
        assert before.pose_seq >= seq

        mock.set_pose({"HeadYaw": 0.8})
        moved = viewer.camera_frame("bottom", 320, 240)
        deadline = time.monotonic() + 5
        while np.abs(moved.image.astype(int) - before.image.astype(int)).mean() <= 5:
            assert time.monotonic() < deadline, "the frame did not turn with the head"
            time.sleep(0.05)
            moved = viewer.camera_frame("bottom", 320, 240)
        assert moved.pose_seq > before.pose_seq
    assert viewer.wait(timeout=5)  # close() ended the headless viewer process
