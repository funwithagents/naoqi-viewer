import logging
import threading
import time
from collections.abc import Callable
from contextlib import closing

import pytest
from mock_naoqi import MockNaoqi, Target

from naoqi_viewer import source
from naoqi_viewer.model import JOINT_NAMES
from naoqi_viewer.source import NaoqiSource, Sample, connect, normalize_url

TILTED_TORSO = (1, 0, 0, 0.2, 0, 1, 0, -0.1, 0, 0, 1, 0.3, 0, 0, 0, 1)


def wait_until(condition: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError("condition not met in time")
        time.sleep(0.01)


def latest(src: NaoqiSource) -> Sample:
    sample = src.latest()
    assert sample is not None
    return sample


def first_sample(src: NaoqiSource) -> Sample:
    wait_until(lambda: src.latest() is not None)
    return latest(src)


def free_url() -> str:
    """A loopback URL nothing listens on (a mock's port, released)."""
    mock = MockNaoqi().start()
    mock.close()
    return mock.url


def test_samples_carry_the_measured_pose(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_pose({"HeadYaw": 0.5, "LKneePitch": 1.2}, torso_transform=TILTED_TORSO)
    with closing(NaoqiSource(url)) as src:
        sample = first_sample(src)
        assert sample.seq >= 1
        assert sample.pose.angles["HeadYaw"] == pytest.approx(0.5)
        assert sample.pose.angles["LKneePitch"] == pytest.approx(1.2)
        assert set(sample.pose.angles) == set(JOINT_NAMES)
        assert sample.pose.torso_pos == pytest.approx((0.2, -0.1, 0.3))
        assert sample.commanded is None

        mock.set_pose({"HeadYaw": -0.4})
        wait_until(lambda: latest(src).pose.angles["HeadYaw"] == pytest.approx(-0.4))
        assert latest(src).seq > sample.seq


# Rates any machine reaches, a slow CI runner included (one polled 27 Hz at most): the test
# is that polling follows rate_hz, not how fast a machine can poll.
@pytest.mark.parametrize(("rate_hz", "low", "high"), [(5, 4, 6), (15, 13, 16)])
def test_rate_follows_rate_hz(
    mock_naoqi: tuple[str, MockNaoqi], rate_hz: float, low: float, high: float
):
    url, _ = mock_naoqi
    with closing(NaoqiSource(url, rate_hz=rate_hz)) as src:
        first_sample(src)
        time.sleep(1.2)
        assert low <= src.rate() <= high


def test_commanded_angles_when_requested(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_pose(
        {"RShoulderPitch": 0.3},
        torso_transform=TILTED_TORSO,
        commanded={"RShoulderPitch": 0.9},
    )
    with closing(NaoqiSource(url, commanded=True)) as src:
        sample = first_sample(src)
        assert sample.pose.angles["RShoulderPitch"] == pytest.approx(0.3)
        assert sample.commanded is not None
        assert sample.commanded.angles["RShoulderPitch"] == pytest.approx(0.9)
        assert sample.commanded.torso_pos == sample.pose.torso_pos


def test_unknown_body_names_are_left_out_and_logged_once(
    caplog: pytest.LogCaptureFixture,
):
    mock = MockNaoqi(body_names=(*JOINT_NAMES, "LFoo")).start()
    try:
        with (
            caplog.at_level(logging.INFO, logger="naoqi_viewer.source"),
            closing(NaoqiSource(mock.url)) as src,
        ):
            first_sample(src)
            wait_until(lambda: latest(src).seq >= 5)
            assert "LFoo" not in latest(src).pose.angles
        assert len([r for r in caplog.records if "LFoo" in r.getMessage()]) == 1
    finally:
        mock.close()


@pytest.mark.parametrize(
    ("target", "version", "expected_version"),
    [
        ("virtual", "2.1.4.13", None),
        ("naoqi-sim", "2.8.7.4", "2.8.7.4"),
        ("real", "2.8.6.23", "2.8.6.23"),
    ],
)
def test_target_identification(
    target: Target, version: str, expected_version: str | None
):
    mock = MockNaoqi(target=target, version=version).start()
    try:
        with closing(NaoqiSource(mock.url)) as src:
            wait_until(lambda: src.info is not None)
            assert src.info is not None
            assert (src.info.target, src.info.naoqi_version, src.info.url) == (
                target,
                expected_version,
                mock.url,
            )
    finally:
        mock.close()


def test_nothing_before_the_first_connection():
    with closing(NaoqiSource(free_url())) as src:
        time.sleep(0.3)
        assert src.latest() is None
        assert src.info is None
        assert src.rate() == 0.0


def test_failing_calls_keep_the_last_sample_then_recover(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    mock.set_pose({"HeadPitch": 0.2})
    with closing(NaoqiSource(url)) as src:
        first_sample(src)
        mock.fail_calls("ALMotion is gone")
        time.sleep(0.1)
        stalled = src.latest()
        assert stalled is not None
        time.sleep(0.3)
        assert src.latest() is stalled
        assert time.monotonic() - stalled.received_at >= 0.3
        assert src.rate() < 50

        mock.fail_calls(None)
        mock.set_pose({"HeadPitch": -0.1})
        wait_until(lambda: latest(src).seq > stalled.seq)
        assert latest(src).pose.angles["HeadPitch"] == pytest.approx(-0.1)


def test_reconnects_after_naoqi_restarts_and_reidentifies(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    with closing(NaoqiSource(url)) as src:
        before = first_sample(src)
        assert src.info is not None and src.info.target == "virtual"
        mock.disconnect()
        time.sleep(0.2)
        assert latest(src).seq - before.seq < 15

        mock.target = "naoqi-sim"
        mock.version = "0.4.0"
        mock.restart()
        mock.set_pose({"LElbowRoll": -1.0})
        wait_until(
            lambda: latest(src).pose.angles["LElbowRoll"] == pytest.approx(-1.0),
            timeout=10,
        )
        assert src.info is not None and (src.info.target, src.info.naoqi_version) == (
            "naoqi-sim",
            "0.4.0",
        )


def test_connects_once_naoqi_appears():
    mock = MockNaoqi()
    mock.url = free_url()
    with closing(NaoqiSource(mock.url)) as src:
        time.sleep(0.3)
        assert src.latest() is None
        mock.start()
        try:
            wait_until(lambda: src.latest() is not None, timeout=10)
            assert src.info is not None and src.info.url == mock.url
        finally:
            mock.close()


def test_the_source_never_changes_robot_state(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    with closing(NaoqiSource(url, commanded=True)) as src:
        wait_until(lambda: src.latest() is not None and latest(src).seq >= 5)
    assert mock.calls_to("getAngles")
    assert not mock.calls_to("setStiffnesses")
    assert not mock.calls_to("angleInterpolation")
    assert not mock.calls_to("insertData")


def test_close_is_prompt_while_waiting_to_reconnect():
    src = NaoqiSource(free_url())
    time.sleep(0.5)
    started = time.monotonic()
    src.close()
    assert time.monotonic() - started < 0.5


def test_normalize_url():
    assert normalize_url("nao.local") == "tcp://nao.local:9559"
    assert normalize_url("192.168.1.12:9560") == "tcp://192.168.1.12:9560"
    assert normalize_url("tcp://127.0.0.1:9559") == "tcp://127.0.0.1:9559"


def test_connect_accepts_a_url_without_scheme(mock_naoqi: tuple[str, MockNaoqi]):
    url, _ = mock_naoqi
    session = connect(url.removeprefix("tcp://"))
    try:
        assert session.service("ALMotion").getBodyNames("Body") == list(JOINT_NAMES)
    finally:
        session.close()


def test_connect_retries_five_times_with_backoff_then_raises(
    monkeypatch: pytest.MonkeyPatch,
):
    pauses: list[float] = []

    def record(seconds: float, cancel: threading.Event | None) -> bool:
        pauses.append(seconds)
        return False

    monkeypatch.setattr(source, "_pause", record)
    url = free_url()
    with pytest.raises(ConnectionError, match=url):
        connect(url)
    assert pauses == pytest.approx([0.2, 0.4, 0.8, 1.6])


def test_connect_stops_when_cancelled():
    cancel = threading.Event()
    cancel.set()
    started = time.monotonic()
    with pytest.raises(ConnectionError, match="cancelled"):
        connect(free_url(), cancel=cancel)
    assert time.monotonic() - started < 0.5
