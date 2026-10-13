"""Pose sources: where the robot's pose comes from, read from NAOqi over qi."""

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Literal, Protocol

import qi

from naoqi_viewer.model import JOINT_NAMES, NaoPose

_log = logging.getLogger(__name__)

_DEFAULT_PORT = 9559
_CONNECT_ATTEMPTS = 5
_CONNECT_FIRST_DELAY = 0.2  # s, doubling between attempts
_CANCEL_POLL = 0.05  # s between checks of `cancel` during one connect attempt
_RECONNECT_FIRST_DELAY = 0.5  # s, doubling up to _RECONNECT_MAX_DELAY
_RECONNECT_MAX_DELAY = 5.0
_FRAME_WORLD = 1

Target = Literal["real", "naoqi-sim", "virtual"]


@dataclass(frozen=True)
class Sample:
    seq: int  # increments with each new sample, starting at 1
    pose: NaoPose  # measured pose
    commanded: NaoPose | None  # commanded angles (same torso), when requested
    received_at: float  # time.monotonic() when the sample arrived


@dataclass(frozen=True)
class TargetInfo:
    url: str
    target: Target
    naoqi_version: str | None


class PoseSource(Protocol):
    @property
    def info(self) -> TargetInfo | None: ...  # None until the first connection

    def latest(self) -> Sample | None: ...  # never blocks; None before the first sample

    def rate(self) -> float: ...  # samples per second over the last second

    def close(self) -> None: ...


def normalize_url(url: str) -> str:
    """`tcp://host:port` as given; a bare `host` (or `host:port`) gets `tcp://` and port 9559."""
    if "://" in url:
        return url
    return f"tcp://{url}" if ":" in url else f"tcp://{url}:{_DEFAULT_PORT}"


def _pause(seconds: float, cancel: threading.Event | None) -> bool:
    """Wait `seconds`; True when `cancel` was set meanwhile."""
    if cancel is None:
        time.sleep(seconds)
        return False
    return cancel.wait(seconds)


def _wait_connected(session: Any, url: str, cancel: threading.Event | None) -> None:
    """Connect `session`, raising RuntimeError on failure. One TCP attempt can take seconds (a
    refused port on Windows, an unreachable host anywhere), so a set `cancel` gives it up."""
    future = session.connect(url, _async=True)
    if cancel is not None:
        while not future.isFinished():
            if cancel.wait(_CANCEL_POLL):
                future.cancel()
                session.close(_async=True)  # don't wait out the pending attempt
                raise ConnectionError(f"connecting to NAOqi at {url} was cancelled")
    future.value()


def connect(url: str, *, cancel: threading.Event | None = None) -> Any:
    """Open a qi session to NAOqi, retrying: the libqi 3 wheels fail some connects at once.

    Returns the connected qi.Session. Raises ConnectionError after 5 failed attempts, or as soon
    as `cancel` is set.
    """
    url = normalize_url(url)
    delay = _CONNECT_FIRST_DELAY
    error: Exception | None = None
    for attempt in range(_CONNECT_ATTEMPTS):
        if cancel is not None and cancel.is_set():
            raise ConnectionError(f"connecting to NAOqi at {url} was cancelled")
        session = qi.Session()
        try:
            _wait_connected(session, url, cancel)
            return session
        except RuntimeError as exc:
            error = exc
            session.close()
        if attempt < _CONNECT_ATTEMPTS - 1:
            if _pause(delay, cancel):
                raise ConnectionError(f"connecting to NAOqi at {url} was cancelled")
            delay *= 2
    raise ConnectionError(
        f"could not connect to NAOqi at {url} after {_CONNECT_ATTEMPTS} attempts: {error}"
    )


def identify(session: Any, url: str) -> TargetInfo:
    services = {service["name"] for service in session.services()}
    if "NaoqiSim" in services:
        version = session.service("ALMemory").getData("NaoqiSim/NaoqiVersion")
        return TargetInfo(url, "naoqi-sim", str(version))
    if "ALSystem" in services:
        return TargetInfo(url, "real", str(session.service("ALSystem").systemVersion()))
    # A plain desktop naoqi-bin has neither service, and no way to ask its version.
    return TargetInfo(url, "virtual", None)


class NaoqiSource:
    """Polls a NAOqi's measured pose (and optionally the commanded one) on its own thread.

    Never blocks: connecting happens on the thread too. On any error it keeps the last sample and
    reconnects with backoff until `close()`. It never changes the robot's state.
    """

    def __init__(self, url: str, rate_hz: float = 50, commanded: bool = False) -> None:
        self._url = normalize_url(url)
        self._period = 1.0 / rate_hz
        self._commanded = commanded
        self._stop = threading.Event()
        self._info: TargetInfo | None = None
        self._latest: Sample | None = None
        self._seq = 0
        self._received: deque[float] = deque()
        self._received_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run, name=f"NaoqiSource {self._url}", daemon=True
        )
        self._thread.start()

    @property
    def info(self) -> TargetInfo | None:
        return self._info

    def latest(self) -> Sample | None:
        return self._latest

    def rate(self) -> float:
        now = time.monotonic()
        with self._received_lock:
            while self._received and self._received[0] < now - 1.0:
                self._received.popleft()
            return float(len(self._received))

    def close(self) -> None:
        self._stop.set()
        self._thread.join()

    def _run(self) -> None:
        delay = _RECONNECT_FIRST_DELAY
        while not self._stop.is_set():
            session = None
            try:
                session = connect(self._url, cancel=self._stop)
                self._info = identify(session, self._url)
                _log.info(
                    "connected to %s (%s, NAOqi %s)",
                    self._url,
                    self._info.target,
                    self._info.naoqi_version,
                )
                delay = _RECONNECT_FIRST_DELAY
                self._poll(session)
            # qi raises RuntimeError; a malformed NAOqi reply surfaces as ValueError/IndexError/TypeError.
            except (
                RuntimeError,
                ConnectionError,
                ValueError,
                IndexError,
                TypeError,
            ) as exc:
                if self._stop.is_set():
                    break
                _log.warning(
                    "NAOqi at %s: %s; retrying in %.1f s", self._url, exc, delay
                )
            finally:
                if session is not None:
                    session.close()
            if self._stop.wait(delay):
                break
            delay = min(delay * 2, _RECONNECT_MAX_DELAY)

    def _poll(self, session: Any) -> None:
        """Poll until close() (returns) or a qi error (raises)."""
        motion = session.service("ALMotion")
        names = list(motion.getBodyNames("Body"))
        unknown = [name for name in names if name not in JOINT_NAMES]
        if unknown:
            _log.info(
                "ignoring body names unknown to the model: %s", ", ".join(unknown)
            )
        known = [(i, name) for i, name in enumerate(names) if name in JOINT_NAMES]
        known_names = [name for _, name in known]

        due = time.monotonic()
        while not self._stop.is_set():
            measured = motion.getAngles("Body", True)
            torso = motion.getTransform("Torso", _FRAME_WORLD, True)
            pose = NaoPose.from_naoqi(
                known_names, [measured[i] for i, _ in known], torso
            )
            commanded = None
            if self._commanded:
                angles = motion.getAngles("Body", False)
                commanded = NaoPose.from_naoqi(
                    known_names, [angles[i] for i, _ in known], torso
                )
            self._publish(pose, commanded)
            # Ticks are due on a fixed schedule, so sleep overshoot doesn't lower the rate. After an
            # overrun the next tick starts at once and the schedule restarts from now: no catching up.
            due += self._period
            remaining = due - time.monotonic()
            if remaining <= 0:
                due = time.monotonic()
            elif self._stop.wait(remaining):
                return

    def _publish(self, pose: NaoPose, commanded: NaoPose | None) -> None:
        now = time.monotonic()
        self._seq += 1
        self._latest = Sample(self._seq, pose, commanded, now)
        with self._received_lock:
            self._received.append(now)
