"""The people source against a live NAOqi (needs NAOQI_URL): a person written into ALMemory's
people-perception keys is read back where it was written. The keys are removed afterwards."""

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


def test_a_person_written_to_almemory_is_read_back():
    url = require_env("NAOQI_URL")
    session = connect(url)
    memory = session.service("ALMemory")
    position = POSITION_KEY.format(id=PERSON_ID)
    try:
        memory.insertData(position, [1.2, -0.3, 1.6])
        memory.insertData(PEOPLE_LIST, [PERSON_ID])
        memory.insertData(VISIBLE_LIST, [PERSON_ID])
        with closing(PeopleSource(url)) as source:
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
        memory.insertData(PEOPLE_LIST, [])
        memory.insertData(VISIBLE_LIST, [])
        memory.removeData(position)
        session.close()


def test_a_viewer_draws_the_people_display_without_failing():
    url = require_env("NAOQI_URL")
    require_window()
    session = connect(url)
    memory = session.service("ALMemory")
    position = POSITION_KEY.format(id=PERSON_ID)
    config = NaoqiViewerConfig.from_dict(
        {
            "mode": "sim",
            "naoqi": {"url": url},
            "world": {"variant": "placeholder"},
            "people_display": True,
        }
    )
    try:
        memory.insertData(position, [1.2, -0.3, 0.5])
        memory.insertData(PEOPLE_LIST, [PERSON_ID])
        memory.insertData(VISIBLE_LIST, [PERSON_ID])
        with NaoqiViewer(config) as viewer:
            time.sleep(1.5)  # markers drawn for a while
            assert viewer.running
    finally:
        memory.insertData(PEOPLE_LIST, [])
        memory.insertData(VISIBLE_LIST, [])
        memory.removeData(position)
        session.close()
