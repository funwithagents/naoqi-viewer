"""The people source against a live NAOqi (needs NAOQI_URL): a person written into ALMemory's
people-perception keys is read back where it was written. The keys are removed afterwards.

The display subscribes to ALPeoplePerception (specs/people.md, "The subscription"), and the module
it starts owns those keys: it would overwrite the test's person with its own population (empty on
a NAOqi that sees nobody). So the test does what any client publishing its own people does: it
pauses the module first, and resumes it after."""

import time
from contextlib import closing

import pytest
from support import require_env, require_window

from naoqi_viewer import NaoqiViewer, NaoqiViewerConfig
from naoqi_viewer.people import (
    PEOPLE_LIST,
    POSITION_KEY,
    VISIBLE_LIST,
    PeopleSource,
)
from naoqi_viewer.source import connect

PERSON_ID = 424242


@pytest.fixture
def people_paused():
    """ALPeoplePerception paused for the test, so the person the test writes stays as written."""
    session = connect(require_env("NAOQI_URL"))
    try:
        perception = session.service("ALPeoplePerception")
    except RuntimeError:  # a target without the module: nothing to pause
        perception = None
    if perception is not None:
        perception.pause(True)
    try:
        yield session.service("ALMemory")
    finally:
        if perception is not None:
            perception.pause(False)
        session.close()


def write_person(memory, position: list[float]) -> None:
    memory.insertData(POSITION_KEY.format(id=PERSON_ID), position)
    memory.insertData(PEOPLE_LIST, [PERSON_ID])
    memory.insertData(VISIBLE_LIST, [PERSON_ID])


def remove_person(memory) -> None:
    memory.insertData(PEOPLE_LIST, [])
    memory.insertData(VISIBLE_LIST, [])
    try:
        memory.removeData(POSITION_KEY.format(id=PERSON_ID))
    except RuntimeError:  # already gone
        pass


def test_a_person_written_to_almemory_is_read_back(people_paused):
    url = require_env("NAOQI_URL")
    memory = people_paused
    try:
        with closing(PeopleSource(url)) as source:
            time.sleep(
                0.5
            )  # subscribed: anything the module writes as it starts, it has written
            write_person(memory, [1.2, -0.3, 1.6])
            deadline = time.monotonic() + 10
            while True:
                people = source.latest()
                if people is not None and people.persons:
                    break
                assert time.monotonic() < deadline, "the person was never read"
                time.sleep(0.05)
            (person,) = people.persons
            assert (person.id, person.visible) == (PERSON_ID, True)
            assert person.pos == pytest.approx(
                (1.2, -0.3, 1.6)
            )  # ALMemory keeps float32
    finally:
        remove_person(memory)


def test_a_viewer_draws_the_people_display_without_failing(people_paused):
    url = require_env("NAOQI_URL")
    require_window()
    memory = people_paused
    config = NaoqiViewerConfig.from_dict(
        {
            "mode": "sim",
            "naoqi": {"url": url},
            "world": {"variant": "placeholder"},
            "people_display": True,
        }
    )
    try:
        write_person(memory, [1.2, -0.3, 0.5])
        with NaoqiViewer(config) as viewer:
            time.sleep(1.5)  # markers drawn for a while
            assert viewer.running
    finally:
        remove_person(memory)
