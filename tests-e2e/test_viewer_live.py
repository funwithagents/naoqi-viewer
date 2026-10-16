"""The real viewer process against a live NAOqi (needs NAOQI_URL).

Viewers open headless unless NAOQI_VIEWER_E2E_WINDOW=1, which opens their windows (needs a display).
"""

import time

import numpy as np
from support import require_env, require_window, window_requested

from naoqi_viewer import NaoqiViewer, NaoqiViewerConfig
from naoqi_viewer.source import connect


def _head_can_move(url: str) -> bool:
    """Moving the head is fine on naoqi-sim or a virtual robot, never on a real one."""
    session = connect(url)
    try:
        return "ALSystem" not in {service["name"] for service in session.services()}
    finally:
        session.close()


def _move_head(url: str, yaw: float) -> None:
    session = connect(url)
    try:
        motion = session.service("ALMotion")
        motion.setStiffnesses("Head", 1.0)
        motion.angleInterpolation("HeadYaw", yaw, 0.5, True)
    finally:
        session.close()


def test_sim_mode_serves_real_camera_frames():
    url = require_env("NAOQI_URL")
    config = NaoqiViewerConfig.from_dict(
        {
            "mode": "sim",
            "headless": not window_requested(),
            "naoqi": {"url": url},
            "world": {"scene": "table", "variant": "placeholder"},
        }
    )
    with NaoqiViewer(config) as viewer:
        deadline = time.monotonic() + 10
        while viewer.status().data_age is None:
            assert time.monotonic() < deadline, "no pose from NAOqi"
            time.sleep(0.1)

        status = viewer.status()
        assert status.mode == "sim" and status.target is not None and status.rate > 0

        top = viewer.camera_frame("top", 640, 480)
        bottom = viewer.camera_frame("bottom", 320, 240)
        assert top.image.shape == (480, 640, 3) and bottom.image.shape == (240, 320, 3)
        assert top.image.std() > 10 and bottom.image.std() > 10  # not blank
        assert top.pose_seq > 0

        if _head_can_move(url):
            _move_head(url, 0.0)
            time.sleep(0.3)
            before = viewer.camera_frame("bottom", 320, 240).image.astype(int)
            _move_head(url, 0.6)
            time.sleep(0.3)
            after = viewer.camera_frame("bottom", 320, 240).image.astype(int)
            assert np.abs(after - before).mean() > 5  # the view turned with the head
    assert not viewer.running


def test_mirror_mode_opens_and_reports_status():
    url = require_env("NAOQI_URL")
    require_window()
    with NaoqiViewer.from_dict(
        {"naoqi": {"url": url}, "cameras_display": True}
    ) as viewer:
        assert viewer.config.mode == "mirror"
        assert viewer.status().url == url
