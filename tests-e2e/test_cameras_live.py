"""The camera source against a live NAOqi (needs NAOQI_URL).

Whether images arrive depends on the target (a real NAO serves its cameras, naoqi-sim what it
injected, a plain naoqi-bin nothing), so the tests assert on the subscription and on the images'
shape only when some arrive.
"""

import logging
import time
from contextlib import closing

import numpy as np
from support import require_env, require_window, window_requested

from naoqi_viewer import NaoqiViewer, NaoqiViewerConfig
from naoqi_viewer.cameras import SUBSCRIBER_NAME, CameraSource
from naoqi_viewer.source import connect


def _ours(video) -> list[str]:
    return [name for name in video.getSubscribers() if name.startswith(SUBSCRIBER_NAME)]


def test_camera_source_subscribes_polls_and_unsubscribes(caplog):
    url = require_env("NAOQI_URL")
    session = connect(url)
    try:
        video = session.service("ALVideoDevice")
        with (
            caplog.at_level(logging.WARNING, logger="naoqi_viewer.cameras"),
            closing(CameraSource(url)) as src,
        ):
            deadline = time.monotonic() + 10
            while not _ours(video):
                assert time.monotonic() < deadline, "the viewer never subscribed"
                time.sleep(0.1)
            assert len(_ours(video)) == 1
            time.sleep(1.5)
            for camera in ("top", "bottom"):
                image = src.latest(camera)
                if image is not None:
                    assert image.image.shape == (240, 320, 3)
                    assert image.image.dtype == np.uint8
        assert _ours(video) == []
        assert not [r for r in caplog.records if r.levelno >= logging.WARNING]
    finally:
        session.close()


def test_a_sim_viewer_with_the_camera_display_launches_and_closes():
    url = require_env("NAOQI_URL")
    require_window()
    config = NaoqiViewerConfig.from_dict(
        {
            "mode": "sim",
            "headless": not window_requested(),
            "naoqi": {"url": url},
            "world": {"scene": "table", "variant": "placeholder"},
            "cameras_display": True,
        }
    )
    with NaoqiViewer(config) as viewer:
        deadline = time.monotonic() + 10
        while viewer.status().data_age is None:
            assert time.monotonic() < deadline, "no pose from NAOqi"
            time.sleep(0.1)
        time.sleep(1.0)  # a few pictures drawn
        assert viewer.running
    assert not viewer.running
