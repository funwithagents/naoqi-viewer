"""The people display's reader and markers (specs/people.md)."""

import time

import pytest
from mock_naoqi import MockNaoqi

from naoqi_viewer.people import (
    PEOPLE_LIST,
    PEOPLE_STALE_AFTER,
    PERSON_HIDDEN_RGBA,
    PERSON_VISIBLE_RGBA,
    POSITION_KEY,
    VISIBLE_LIST,
    People,
    PeopleSource,
    Person,
    decode_people,
    people_status,
    person_markers,
)


def publish(
    mock: MockNaoqi, people: dict[int, list[float]], visible: list[int]
) -> None:
    mock._insertData(PEOPLE_LIST, list(people))
    mock._insertData(VISIBLE_LIST, visible)
    for person_id, pos in people.items():
        mock._insertData(POSITION_KEY.format(id=person_id), pos)


def wait_for(source: PeopleSource, predicate, timeout: float = 5.0) -> People:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        people = source.latest()
        if people is not None and predicate(people):
            return people
        time.sleep(0.02)
    raise AssertionError(f"no matching people within {timeout} s: {source.latest()}")


def test_decode_skips_ids_without_a_position_and_reads_visibility():
    persons = decode_people(
        [[7, 8, 9], [8]], {7: [1.0, 0.5, 1.6], 8: [2, -1, 1.7], 9: None}
    )
    assert persons == (
        Person(7, (1.0, 0.5, 1.6), False),
        Person(8, (2.0, -1.0, 1.7), True),
    )
    assert decode_people([None, None], {}) == ()
    assert decode_people(None, {}) == ()


def test_markers_colour_by_visibility_and_vanish_when_stale():
    now = 50.0
    people = People(
        (Person(3, (1, 0, 1.6), True), Person(4, (2, 1, 1.5), False)), 1, now
    )
    markers = person_markers(people, now + 0.1)
    assert [(m.label, m.rgba) for m in markers] == [
        ("3", PERSON_VISIBLE_RGBA),
        ("4", PERSON_HIDDEN_RGBA),
    ]
    assert person_markers(people, now + PEOPLE_STALE_AFTER + 0.1) == []
    assert person_markers(None, now) == []


def test_status_readings():
    now = 10.0
    assert people_status(None, now) == "NO DATA"
    assert people_status(People((), 1, now), now) == "none"
    two = People((Person(1, (0, 0, 0), True), Person(2, (0, 0, 0), False)), 1, now)
    assert people_status(two, now) == "2 (1 visible)"
    assert people_status(two, now + PEOPLE_STALE_AFTER + 1) == "STALE"


def test_the_source_reads_what_naoqi_publishes(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    source = PeopleSource(url)
    try:
        nobody = wait_for(source, lambda p: True)
        assert nobody.persons == ()  # no keys at all: nobody, not an error
        publish(mock, {12: [1.0, 0.2, 1.6], 13: [2.0, -0.5, 1.7]}, visible=[12])
        people = wait_for(source, lambda p: len(p.persons) == 2)
        assert people.persons == (
            Person(12, (1.0, 0.2, 1.6), True),
            Person(13, (2.0, -0.5, 1.7), False),
        )
    finally:
        source.close()


def test_the_source_keeps_its_last_answer_through_failures_and_reconnects(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    publish(mock, {5: [1.0, 0.0, 1.6]}, visible=[5])
    source = PeopleSource(url)
    try:
        first = wait_for(source, lambda p: len(p.persons) == 1)
        mock.fail_calls("boom")
        time.sleep(0.3)
        kept = source.latest()
        assert kept is not None and kept.persons == first.persons
        mock.fail_calls(None)
        mock.restart()
        publish(mock, {5: [1.0, 0.0, 1.6], 6: [1.5, 0.3, 1.6]}, visible=[5, 6])
        wait_for(source, lambda p: len(p.persons) == 2, timeout=10)
    finally:
        started = time.monotonic()
        source.close()
        assert time.monotonic() - started < 2


@pytest.mark.parametrize("rate", [5.0, 20.0])
def test_the_source_polls_at_its_rate(mock_naoqi: tuple[str, MockNaoqi], rate: float):
    url, _ = mock_naoqi
    source = PeopleSource(url, rate_hz=rate)
    try:
        wait_for(source, lambda p: True)
        time.sleep(1.2)
        assert rate * 0.6 <= source.rate() <= rate + 1
    finally:
        source.close()
