"""People display: the people NAOqi lists under `PeoplePerception/` in ALMemory, read on their own
thread and drawn as markers in the window (specs/people.md).

`PeopleSource` is the camera source's sibling: its own qi session, a fixed polling schedule, the
newest result in a slot the window reads without blocking, reconnection with backoff. The pure
functions (`decode_people`, `person_markers`) are what the tests pin without a NAOqi or a window.
"""

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Protocol

from naoqi_viewer.source import connect, normalize_url

_log = logging.getLogger(__name__)

PEOPLE_RATE_HZ = 10
PEOPLE_LIST = "PeoplePerception/PeopleList"
VISIBLE_LIST = "PeoplePerception/VisiblePeopleList"
POSITION_KEY = "PeoplePerception/Person/{id}/PositionInWorldFrame"
PEOPLE_STALE_AFTER = 1.0  # s: ten polls
PERSON_RADIUS = 0.10  # m: a head
PERSON_VISIBLE_RGBA = (1.0, 0.85, 0.1, 0.6)  # yellow
PERSON_HIDDEN_RGBA = (0.6, 0.6, 0.6, 0.3)  # grey: tracked, not seen

_RECONNECT_FIRST_DELAY = 0.5  # s, doubling up to _RECONNECT_MAX_DELAY (as source.py)
_RECONNECT_MAX_DELAY = 5.0


@dataclass(frozen=True)
class Person:
    id: int
    pos: tuple[float, float, float]  # m, NAOqi's FRAME_WORLD
    visible: bool


@dataclass(frozen=True)
class People:
    persons: tuple[Person, ...]
    seq: int  # increments with each poll that answered
    received_at: float  # time.monotonic()


@dataclass(frozen=True)
class Marker:
    pos: tuple[float, float, float]
    rgba: tuple[float, float, float, float]
    label: str


class PeopleReader(Protocol):
    def latest(self) -> People | None: ...  # never blocks

    def rate(self) -> float: ...

    def close(self) -> None: ...


def _ids(value: Any) -> list[int]:
    if not isinstance(value, list | tuple):
        return []
    return [int(v) for v in value if isinstance(v, int) and not isinstance(v, bool)]


def _position(value: Any) -> tuple[float, float, float] | None:
    if (
        isinstance(value, list | tuple)
        and len(value) == 3
        and all(isinstance(v, int | float) and not isinstance(v, bool) for v in value)
    ):
        return (float(value[0]), float(value[1]), float(value[2]))
    return None


def decode_people(lists: Any, positions: dict[int, Any]) -> tuple[Person, ...]:
    """The persons from `[PeopleList, VisiblePeopleList]` and each id's position value; an id
    whose position is missing or malformed is skipped."""
    if not isinstance(lists, list | tuple) or len(lists) != 2:
        return ()
    listed, visible = _ids(lists[0]), set(_ids(lists[1]))
    persons = []
    for person_id in listed:
        pos = _position(positions.get(person_id))
        if pos is not None:
            persons.append(Person(person_id, pos, person_id in visible))
    return tuple(persons)


def person_markers(people: People | None, now: float) -> list[Marker]:
    """What the window draws for `people` at `now`: nothing before the first answer or when
    stale, else one marker per person."""
    if people is None or now - people.received_at > PEOPLE_STALE_AFTER:
        return []
    return [
        Marker(
            p.pos,
            PERSON_VISIBLE_RGBA if p.visible else PERSON_HIDDEN_RGBA,
            str(p.id),
        )
        for p in people.persons
    ]


def people_status(people: People | None, now: float) -> str:
    if people is None:
        return "NO DATA"
    if now - people.received_at > PEOPLE_STALE_AFTER:
        return "STALE"
    if not people.persons:
        return "none"
    seen = sum(p.visible for p in people.persons)
    return f"{len(people.persons)} ({seen} visible)"


class PeopleSource:
    """Polls NAOqi's people-perception keys on its own thread. Never blocks; on any error it keeps
    the last answer and reconnects with backoff until `close()`."""

    def __init__(self, url: str, *, rate_hz: float = PEOPLE_RATE_HZ) -> None:
        self._url = normalize_url(url)
        self._period = 1.0 / rate_hz
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: People | None = None
        self._seq = 0
        self._received: deque[float] = deque()
        self._thread = threading.Thread(
            target=self._run, name=f"PeopleSource {self._url}", daemon=True
        )
        self._thread.start()

    def latest(self) -> People | None:
        with self._lock:
            return self._latest

    def rate(self) -> float:
        now = time.monotonic()
        with self._lock:
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
                memory = session.service("ALMemory")
                delay = _RECONNECT_FIRST_DELAY
                self._poll(memory)
            except (RuntimeError, ConnectionError, ValueError, TypeError) as exc:
                if self._stop.is_set():
                    break
                _log.warning(
                    "people of NAOqi at %s: %s; retrying in %.1f s",
                    self._url,
                    exc,
                    delay,
                )
            finally:
                if session is not None:
                    session.close()
            if self._stop.wait(delay):
                break
            delay = min(delay * 2, _RECONNECT_MAX_DELAY)

    def read(self, memory: Any) -> tuple[Person, ...]:
        lists = memory.getListData([PEOPLE_LIST, VISIBLE_LIST])
        ids = _ids(lists[0]) if isinstance(lists, list | tuple) and lists else []
        values = (
            memory.getListData([POSITION_KEY.format(id=i) for i in ids]) if ids else []
        )
        return decode_people(lists, dict(zip(ids, values, strict=False)))

    def _poll(self, memory: Any) -> None:
        due = time.monotonic()
        while not self._stop.is_set():
            persons = self.read(memory)
            now = time.monotonic()
            self._seq += 1
            with self._lock:
                self._latest = People(persons, self._seq, now)
                self._received.append(now)
            due += self._period
            remaining = due - time.monotonic()
            if remaining <= 0:
                due = time.monotonic()
            elif self._stop.wait(remaining):
                return
