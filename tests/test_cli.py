import io
import json
import logging
import sys
from collections.abc import Iterator
from pathlib import Path

import pytest
from mesh_archive import install_fake_release
from mock_naoqi import MockNaoqi

from naoqi_viewer import client, meshes
from naoqi_viewer.cli import main

FAKE_VIEWER = Path(__file__).with_name("fake_viewer_process.py")


@pytest.fixture(autouse=True)
def reset_logging() -> Iterator[None]:
    """main() configures the root logger; give each test a clean one."""
    root = logging.getLogger()
    handlers, level = root.handlers[:], root.level
    root.handlers = []
    yield
    root.handlers, root.level = handlers, level


@pytest.fixture
def fake_viewer_process(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """`view` runs the fake viewer process; returns the file the config it received lands in."""
    monkeypatch.setattr(
        client,
        "_viewer_command",
        lambda config: [sys.executable, str(FAKE_VIEWER), json.dumps(config.to_dict())],
    )
    received = tmp_path / "received.json"
    monkeypatch.setenv("NAOQI_FAKE_VIEWER_CONFIG", str(received))
    return received


# --- check-model --------------------------------------------------------------------


def test_check_model_prints_the_report_and_passes(
    mock_naoqi: tuple[str, MockNaoqi], capsys: pytest.CaptureFixture[str]
):
    url, mock = mock_naoqi
    assert main(["check-model", url, "--samples", "3", "--seed", "42"]) == 0
    out = capsys.readouterr().out
    assert (
        f"check-model {url} (virtual, NAOqi unknown version): 3 samples, seed 42" in out
    )
    assert "PASS" in out
    assert len(mock.calls_to("angleInterpolation")) == 3


def test_check_model_fails_over_tolerance(
    mock_naoqi: tuple[str, MockNaoqi], capsys: pytest.CaptureFixture[str]
):
    url, mock = mock_naoqi
    shifted = [1.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.0015, 0, 0, 0, 1]  # 1.5 mm up
    mock.set_effector_error("RArm", shifted)
    argv = ["check-model", url, "--samples", "2", "--seed", "1"]
    assert main(argv) == 0
    assert main([*argv, "--tolerance-mm", "1"]) == 1
    assert "FAIL: RArm over 1 mm / 1°" in capsys.readouterr().out


def test_check_model_refuses_a_real_robot(capsys: pytest.CaptureFixture[str]):
    mock = MockNaoqi(target="real").start()
    try:
        assert main(["check-model", mock.url, "--samples", "1"]) == 1
        assert mock.calls_to("angleInterpolation") == []
        err = capsys.readouterr().err
        assert err.startswith("naoqi-viewer: error: ")
        assert "--allow-real" in err
        assert main(["check-model", mock.url, "--samples", "1", "--allow-real"]) == 0
    finally:
        mock.close()


@pytest.mark.parametrize(
    "option", [["--samples", "0"], ["--samples", "two"], ["--tolerance-deg", "-1"]]
)
def test_check_model_bad_options_are_usage_errors(option: list[str]):
    with pytest.raises(SystemExit) as exit_info:
        main(["check-model", "tcp://127.0.0.1:9559", *option])
    assert exit_info.value.code == 2


def test_a_command_is_required():
    with pytest.raises(SystemExit) as exit_info:
        main([])
    assert exit_info.value.code == 2


# --- view ---------------------------------------------------------------------------


def test_view_launches_the_config_and_returns_when_the_window_closes(
    tmp_path: Path, fake_viewer_process: Path, monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.setenv(
        "NAOQI_FAKE_VIEWER", "close-after-1"
    )  # hello, then the window closes
    config = tmp_path / "viewer.json"
    config.write_text(
        json.dumps(
            {"mode": "sim", "naoqi": {"url": "tcp://10.0.0.7:9600"}, "ghost": True}
        )
    )
    assert main(["view", "--config", str(config)]) == 0
    received = json.loads(fake_viewer_process.read_text())
    assert (received["mode"], received["naoqi"]["url"], received["ghost"]) == (
        "sim",
        "tcp://10.0.0.7:9600",
        True,
    )


def test_view_without_a_config_opens_the_default(
    fake_viewer_process: Path, monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "close-after-1")
    assert main(["view"]) == 0
    received = json.loads(fake_viewer_process.read_text())
    assert (received["mode"], received["naoqi"]["url"], received["world"]["scene"]) == (
        "mirror",
        "tcp://127.0.0.1:9559",
        "empty",
    )


def test_view_with_a_bad_config_names_the_key(
    tmp_path: Path, fake_viewer_process: Path, capsys: pytest.CaptureFixture[str]
):
    config = tmp_path / "viewer.json"
    config.write_text(json.dumps({"naoqi": {"rate_hz": -5}}))
    assert main(["view", "--config", str(config)]) == 2
    assert "naoqi.rate_hz" in capsys.readouterr().err
    assert not fake_viewer_process.exists()  # nothing was launched


def test_view_reports_a_launch_failure(
    fake_viewer_process: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "error")
    assert main(["view"]) == 1
    assert "naoqi-viewer: error: unknown scene 'nope'" in capsys.readouterr().err


def test_view_closes_the_viewer_on_ctrl_c(
    fake_viewer_process: Path, monkeypatch: pytest.MonkeyPatch
):
    closed: list[bool] = []
    real_close = client.NaoqiViewer.close

    def interrupt(self: client.NaoqiViewer, timeout: float | None = None) -> bool:
        raise KeyboardInterrupt

    def close(self: client.NaoqiViewer) -> None:
        closed.append(self.running)
        real_close(self)

    monkeypatch.setattr(client.NaoqiViewer, "wait", interrupt)
    monkeypatch.setattr(client.NaoqiViewer, "close", close)
    assert main(["view"]) == 130
    assert closed[-1] is True  # the viewer was still up and got closed


# --- logging ------------------------------------------------------------------------


def test_logs_go_to_stderr_at_info_by_default(
    fake_viewer_process: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "close-after-1")
    assert main(["view"]) == 0
    err = capsys.readouterr().err
    assert "connected to the fake NAOqi" in err  # INFO, from the viewer process
    assert "a warning from the viewer process" in err


def test_quiet_keeps_warnings_only(
    fake_viewer_process: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    monkeypatch.setenv("NAOQI_FAKE_VIEWER", "close-after-1")
    assert main(["-q", "view"]) == 0
    err = capsys.readouterr().err
    assert "connected to the fake NAOqi" not in err
    assert "a warning from the viewer process" in err


def test_verbose_adds_debug_messages(
    mock_naoqi: tuple[str, MockNaoqi], capsys: pytest.CaptureFixture[str]
):
    url, _ = mock_naoqi
    assert main(["check-model", url, "--samples", "2", "--seed", "3"]) == 0
    assert "sample 2/2 done" not in capsys.readouterr().err
    assert main(["-v", "check-model", url, "--samples", "2", "--seed", "3"]) == 0
    assert "DEBUG naoqi_viewer.check_model: sample 2/2 done" in capsys.readouterr().err


# --- fetch-meshes ----------------------------------------------------------------------


def test_fetch_meshes_refuses_a_piped_stdin(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
):
    install_fake_release(tmp_path, monkeypatch)
    monkeypatch.setattr(sys, "stdin", io.StringIO("yes\n"))
    assert main(["fetch-meshes"]) == 1
    captured = capsys.readouterr()
    assert (
        "naoqi-viewer: error: fetch-meshes needs an interactive terminal"
        in captured.err
    )
    assert meshes.installed() is None


def test_fetch_meshes_remove(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
):
    install_fake_release(tmp_path, monkeypatch)
    assert main(["fetch-meshes", "--remove"]) == 0
    assert capsys.readouterr().out == "nothing installed\n"

    class Terminal(io.StringIO):
        def isatty(self) -> bool:
            return True

    monkeypatch.setattr(sys, "stdin", Terminal("yes\n"))
    assert main(["fetch-meshes"]) == 0
    assert meshes.installed() is not None
    assert main(["fetch-meshes", "--remove"]) == 0
    assert capsys.readouterr().out.endswith(f"removed {meshes.data_dir()}\n")
    assert meshes.installed() is None


def test_fetch_meshes_remove_takes_no_other_option(capsys: pytest.CaptureFixture[str]):
    assert main(["fetch-meshes", "--remove", "--force"]) == 2
    assert "--remove takes neither" in capsys.readouterr().err
