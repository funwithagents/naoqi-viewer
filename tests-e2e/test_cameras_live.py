"""The camera source against a live NAOqi (needs NAOQI_URL).

Whether images arrive depends on the target (a real NAO serves its cameras, naoqi-sim what it
injected, a plain naoqi-bin nothing), so the tests assert on the subscription and on the images'
shape only when some arrive.
"""

import logging
import time
from contextlib import closing

import numpy as np
import pytest
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


def test_the_pictures_have_naoqis_own_colours():
    """The source's RGB, decoded from YUV422, against the RGB NAOqi converts itself for a
    subscription of the test's own on the top camera: within a level on average on the same
    capture (naoqi-sim serves both the same one), each channel's mean within six levels on
    different captures (a robot's). A wrong range or swapped channels miss both."""
    url = require_env("NAOQI_URL")
    session = connect(url)
    try:
        video = session.service("ALVideoDevice")
        name = video.subscribeCamera("naoqi_viewer_live_rgb", 0, 1, 11, 10)
        try:
            with closing(CameraSource(url)) as src:
                deadline = time.monotonic() + 10
                while src.latest("top") is None:
                    if time.monotonic() > deadline:
                        pytest.skip("the target serves no top camera image")
                    time.sleep(0.1)
                reference = video.getImageRemote(name)
                ts = reference[4] + reference[5] / 1e6
                deadline = time.monotonic() + 3
                while (image := src.latest("top")) is None or (
                    image.naoqi_time or 0.0
                ) < ts - 1e-6:
                    assert time.monotonic() < deadline, "no picture since the reference"
                    time.sleep(0.02)
        finally:
            video.unsubscribe(name)
    finally:
        session.close()
    width, height = reference[0], reference[1]
    theirs = np.frombuffer(bytes(reference[6]), np.uint8).reshape(height, width, 3)
    ours = image.image.astype(int)
    assert ours.shape == theirs.shape
    gap = np.abs(ours.mean(axis=(0, 1)) - theirs.mean(axis=(0, 1)))
    if abs((image.naoqi_time or 0.0) - ts) < 1e-6:
        error = np.abs(ours - theirs.astype(int))
        print(f"top camera, YUV422 vs NAOqi's RGB: mean {error.mean():.2f} levels")
        assert error.mean() < 1.0
    assert gap.max() < 6


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
