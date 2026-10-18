import json
import logging
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from naoqi_viewer import (
    LaunchError,
    ModeError,
    NaoqiSettings,
    NaoqiViewer,
    NaoqiViewerConfig,
    PropError,
    PropState,
    ViewerClosed,
    ViewerStatus,
    WorldSettings,
    client,
)

FAKE_VIEWER = Path(__file__).with_name("fake_viewer_process.py")
URL = "tcp://127.0.0.1:9559"
SIM = NaoqiViewerConfig(mode="sim", naoqi=NaoqiSettings(url=URL))
MIRROR = NaoqiViewerConfig(mode="mirror", naoqi=NaoqiSettings(url=URL))


@pytest.fixture(autouse=True)
def fake_viewer_process(monkeypatch: pytest.MonkeyPatch) -> None:
    """launch() runs the fake viewer process instead of the real one."""
    monkeypatch.setattr(
        client,
        "_viewer_command",
        lambda config: [sys.executable, str(FAKE_VIEWER), json.dumps(config.to_dict())],
    )


@pytest.fixture
def ops_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "ops.txt"
    monkeypatch.setenv("NAOQI_FAKE_VIEWER_OPS", str(path))
    return path


def received_ops(path: Path) -> list[str]:
    return path.read_text().split() if path.exists() else []


def test_launch_handshake_and_status(ops_file: Path):
    with NaoqiViewer(SIM) as viewer:
        assert viewer.running
        assert viewer.status() == ViewerStatus(
            mode="sim",
            target="naoqi-sim",
            naoqi_version="0.3.0",
            url=URL,
            variant="placeholder",
            rate=50.0,
            pose_seq=12,
            data_age=0.008,
        )
    assert received_ops(ops_file) == ["hello", "status", "stop"]
    assert not viewer.running


def test_the_config_reaches_the_viewer_process_intact(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    received = tmp_path / "config.json"
    monkeypatch.setenv("NAOQI_FAKE_VIEWER_CONFIG", str(received))
    config = NaoqiViewerConfig(
        mode="sim",
        naoqi=NaoqiSettings(url="tcp://10.0.0.7:9600", rate_hz=30),
        world=WorldSettings(scene="table", variant="placeholder"),
        ghost=True,
        launch_timeout_s=12,
    )
    with NaoqiViewer(config):
        pass
    assert NaoqiViewerConfig.from_dict(json.loads(received.read_text())) == config


def test_building_a_viewer_starts_nothing(ops_file: Path):
    viewer = NaoqiViewer.from_dict({"mode": "sim"})
    assert viewer.config == NaoqiViewerConfig(mode="sim")
    assert not viewer.running
    assert viewer.wait(timeout=0) is True
    with pytest.raises(ViewerClosed, match="launch"):
        viewer.status()
    with pytest.raises(ViewerClosed):
        viewer.camera_frame("top", 64, 48)
    viewer.close()  # harmless
    assert received_ops(ops_file) == []


def test_constructors_from_json_and_file(tmp_path: Path):
    text = '{"mode": "sim", "naoqi": {"url": "nao.local"}}'
    path = tmp_path / "viewer.json"
    path.write_text(text)
    expected = NaoqiViewerConfig(mode="sim", naoqi=NaoqiSettings(url="nao.local"))
    assert NaoqiViewer.from_json(text).config == expected
    assert NaoqiViewer.from_json_file(path).config == expected
    assert NaoqiViewer().config == NaoqiViewerConfig()


def test_camera_frame_returns_the_rendered_image():
    with NaoqiViewer(SIM) as viewer:
        frame = viewer.camera_frame("bottom", 320, 240)
        assert frame.image.shape == (240, 320, 3) and frame.image.dtype == np.uint8
        assert (frame.camera, frame.pose_seq, frame.pose_age) == ("bottom", 12, 0.02)
        # Row 0 is the top of the image, columns run left to right, and it's the bottom camera.
        assert frame.image[5, 7].tolist() == [5, 7, 2]
        assert frame.image[239, 319].tolist() == [239, 319 % 256, 2]
        frame.image[0, 0] = 0  # the caller owns a writable array


def test_camera_frame_in_mirror_mode_raises_without_a_round_trip(ops_file: Path):
    with NaoqiViewer(MIRROR) as viewer, pytest.raises(ModeError):
        viewer.camera_frame("top", 640, 480)
    assert "camera_frame" not in received_ops(ops_file)


def test_camera_frame_rejects_an_unknown_camera():
    with NaoqiViewer(SIM) as viewer, pytest.raises(ValueError, match="left"):
        viewer.camera_frame("left", 640, 480)  # type: ignore[arg-type]


def test_launching_twice_is_an_error_but_relaunching_after_close_works():
    viewer = NaoqiViewer(SIM)
    viewer.launch()
    try:
        with pytest.raises(RuntimeError, match="already running"):
            viewer.launch()
    finally:
        viewer.close()
    viewer.launch()  # a new window, same config
    try:
        assert viewer.status().mode == "sim"
    finally:
        viewer.close()


def test_launch_again_after_the_window_was_closed(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv(
        "NAOQI_FAKE_VIEWER", "close-after-2"
    )  # hello, status, then the window closes
    viewer = NaoqiViewer(SIM)
    viewer.launch()
    viewer.status()
    assert viewer.wait(timeout=5)
    viewer.launch()
    try:
        assert viewer.running
        assert viewer.status().pose_seq == 12
    finally:
        viewer.close()


def test_launch_error_from_the_viewer_process(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "error")
    viewer = NaoqiViewer(SIM)
    with pytest.raises(LaunchError, match="unknown scene 'nope'") as raised:
        viewer.launch()
    assert "loading the scene" in str(raised.value)  # the stderr tail comes along
    assert not viewer.running


def test_launch_error_when_the_viewer_process_exits_early(
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "crash")
    with pytest.raises(LaunchError, match="exited \\(code 3\\)") as raised:
        NaoqiViewer(SIM).launch()
    assert "GLFW could not open a window" in str(raised.value)


def test_launch_times_out_and_stops_the_viewer_process(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "hang")
    processes: list[subprocess.Popen[str]] = []
    real_popen = subprocess.Popen

    def recording_popen(*args, **kwargs):
        process = real_popen(*args, **kwargs)
        processes.append(process)
        return process

    monkeypatch.setattr(subprocess, "Popen", recording_popen)
    started = time.monotonic()
    with pytest.raises(LaunchError, match="wasn't ready within 1 s") as raised:
        NaoqiViewer(NaoqiViewerConfig(launch_timeout_s=1)).launch()
    assert time.monotonic() - started < 5
    assert "never ready" in str(raised.value)
    assert processes[0].poll() is not None  # not left running


def test_protocol_mismatch_is_a_launch_error(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER_PROTOCOL", "1")  # an older viewer process
    with pytest.raises(LaunchError, match="protocol 1"):
        NaoqiViewer(SIM).launch()


def test_calls_after_the_viewer_exits_raise_viewer_closed(
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "close-after-2")
    viewer = NaoqiViewer(SIM)
    viewer.launch()
    viewer.status()
    assert viewer.wait(timeout=5)
    assert not viewer.running
    with pytest.raises(ViewerClosed):
        viewer.status()
    with pytest.raises(ViewerClosed):
        viewer.camera_frame("top", 64, 48)
    viewer.close()  # harmless once gone


def test_wait_times_out_while_the_viewer_runs():
    with NaoqiViewer(MIRROR) as viewer:
        assert viewer.wait(timeout=0.1) is False


def test_calls_after_close_raise_viewer_closed():
    viewer = NaoqiViewer(MIRROR)
    viewer.launch()
    viewer.close()
    assert not viewer.running
    with pytest.raises(ViewerClosed):
        viewer.status()


def test_viewer_process_logs_reach_the_callers_logging(
    caplog: pytest.LogCaptureFixture,
):
    with caplog.at_level(logging.DEBUG, logger="naoqi_viewer.viewer_process"):
        with NaoqiViewer(MIRROR) as viewer:
            viewer.status()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not any(
            "native log line" in r.getMessage() for r in caplog.records
        ):
            time.sleep(0.01)
    records = {
        r.getMessage(): r.levelno
        for r in caplog.records
        if r.name == "naoqi_viewer.viewer_process"
    }
    assert records["naoqi_viewer.source: connected to the fake NAOqi"] == logging.INFO
    assert (
        records["naoqi_viewer.viewer: a warning from the viewer process"]
        == logging.WARNING
    )
    native = [message for message in records if "native log line" in message]
    assert native and records[native[0]] == logging.DEBUG
    qi_stdout = [message for message in records if "qi.path.sdklayout" in message]
    assert (
        qi_stdout and records[qi_stdout[0]] == logging.DEBUG
    )  # stdout chatter before the ready line


def test_concurrent_calls_get_their_own_answers():
    with NaoqiViewer(SIM) as viewer:
        sizes = [(32 + i, 24 + i) for i in range(8)]
        results: dict[int, tuple[int, ...]] = {}

        def fetch(i: int) -> None:
            width, height = sizes[i]
            shapes = {
                viewer.camera_frame("top", width, height).image.shape for _ in range(5)
            }
            results[i] = shapes.pop() if len(shapes) == 1 else (0,)

        threads = [threading.Thread(target=fetch, args=(i,)) for i in range(len(sizes))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        assert results == {i: (h, w, 3) for i, (w, h) in enumerate(sizes)}


def test_missing_mjpython_on_macos_is_a_launch_error(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    monkeypatch.undo()  # the real command, not the fake viewer process
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(sys, "executable", str(tmp_path / "python"))
    with pytest.raises(LaunchError, match="mjpython"):
        NaoqiViewer(MIRROR).launch()


HEADLESS = NaoqiViewerConfig(mode="sim", headless=True, naoqi=NaoqiSettings(url=URL))


def test_a_headless_viewer_needs_no_mjpython_on_macos(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    monkeypatch.undo()
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(
        sys, "executable", str(tmp_path / "python")
    )  # no mjpython beside it
    command = client._viewer_command(HEADLESS)
    assert command[:3] == [
        str(tmp_path / "python"),
        "-m",
        "naoqi_viewer.viewer_process",
    ]
    assert NaoqiViewerConfig.from_json(command[3]) == HEADLESS


def test_a_headless_viewer_on_linux_renders_through_egl_by_default(
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.delenv("MUJOCO_GL", raising=False)
    monkeypatch.setenv("NAO_SOMETHING", "kept")
    env = client._viewer_env(HEADLESS)
    assert env["MUJOCO_GL"] == "egl" and env["NAO_SOMETHING"] == "kept"
    assert "MUJOCO_GL" not in client._viewer_env(SIM)  # a window: MuJoCo's own backend


def test_the_callers_mujoco_gl_wins(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setenv("MUJOCO_GL", "osmesa")
    assert client._viewer_env(HEADLESS)["MUJOCO_GL"] == "osmesa"


@pytest.mark.parametrize("platform", ["darwin", "win32"])
def test_macos_and_windows_keep_mujocos_own_offscreen_backend(
    monkeypatch: pytest.MonkeyPatch, platform: str
):
    monkeypatch.setattr(sys, "platform", platform)
    monkeypatch.delenv("MUJOCO_GL", raising=False)
    assert "MUJOCO_GL" not in client._viewer_env(HEADLESS)


def test_a_non_ascii_path_reaches_the_launch_error_intact(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    # Japanese is outside Windows' legacy code pages: the viewer process must print it in UTF-8,
    # and launch() read it as UTF-8, whatever the locale.
    monkeypatch.undo()  # the real viewer process
    # A Windows code page in the caller's environment, on any machine.
    monkeypatch.setenv("PYTHONIOENCODING", "cp1252")
    scene = tmp_path / "ナオ" / "scene.xml"
    config = NaoqiViewerConfig(
        mode="sim", naoqi=NaoqiSettings(url=URL), world=WorldSettings(scene=str(scene))
    )
    with pytest.raises(LaunchError) as raised:
        NaoqiViewer(config).launch()
    assert f"scene file not found: {scene}" in str(raised.value)


def test_importing_naoqi_viewer_loads_neither_mujoco_nor_qi():
    code = "import sys, naoqi_viewer; print(sorted(m for m in ('mujoco', 'qi') if m in sys.modules))"
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert result.stdout.strip() == "[]"


def test_props_calls_go_over_the_wire_and_come_back_as_states(ops_file: Path):
    with NaoqiViewer(SIM) as viewer:
        ball = viewer.props.spawn(
            "object", np.array([1.5, 0.0, 0.5]), size=0.1, color=(0, 1, 0)
        )
        assert ball == PropState(
            "object_1", "object", True, (1.5, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0), False,
            None, "sphere", (0.1,), (0.0, 1.0, 0.0),
        )  # fmt: skip
        face = viewer.props.spawn("face", (1, 0.3, 0.5), name="face_1")
        assert (face.name, face.image) == ("face_1", "armstrong")
        assert viewer.props.place("face_1", (1, -0.3, 0.5), duration=2).pos == (
            1.0, -0.3, 0.5,
        )  # fmt: skip
        assert viewer.props.despawn("face_1").visible is False
        assert set(viewer.props.list()) == {"face_1", "object_1"}
        assert not any(s.visible for s in viewer.props.clear().values())
        with pytest.raises(PropError, match="no prop 'nope'"):
            viewer.props.despawn("nope")
    assert received_ops(ops_file) == [
        "hello", "prop_spawn", "prop_spawn", "prop_place", "prop_despawn",
        "props", "props_clear", "prop_despawn", "stop",
    ]  # fmt: skip


def test_props_in_mirror_mode_raise_without_a_round_trip(ops_file: Path):
    with NaoqiViewer(MIRROR) as viewer, pytest.raises(ModeError, match="sim mode"):
        viewer.props.list()
    assert received_ops(ops_file) == ["hello", "stop"]
