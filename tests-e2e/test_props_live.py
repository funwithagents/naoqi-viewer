"""Props against a live NAOqi (needs NAOQI_URL): a sim viewer's head camera sees an object spawned
in front of the robot, and loses it once despawned."""

import math
import time

import numpy as np
from support import require_env, window_requested

from naoqi_viewer import NaoqiViewer, NaoqiViewerConfig
from naoqi_viewer.source import connect


def red_columns(image: np.ndarray) -> np.ndarray:
    pixels = image.astype(int)
    mask = (pixels[..., 0] > 150) & (pixels[..., 1] < 80) & (pixels[..., 2] < 80)
    return np.flatnonzero(mask.any(axis=0))


def test_the_head_camera_sees_a_spawned_object_ahead_of_the_robot():
    url = require_env("NAOQI_URL")
    session = connect(url)
    try:
        motion = session.service("ALMotion")
        motion.wakeUp()
        session.service("ALRobotPosture").goToPosture("StandInit", 0.8)
        motion.angleInterpolation(["HeadYaw", "HeadPitch"], [0.0, 0.0], 1.0, True)
        # The viewer places the robot at NAOqi's world pose (odometry): aim from it.
        x, y, theta = motion.getRobotPosition(True)
    finally:
        session.close()
    ahead = (x + 1.5 * math.cos(theta), y + 1.5 * math.sin(theta), 0.45)
    config = NaoqiViewerConfig.from_dict(
        {
            "mode": "sim",
            "headless": not window_requested(),
            "naoqi": {"url": url},
            "world": {"scene": "empty", "variant": "placeholder"},
        }
    )
    with NaoqiViewer(config) as viewer:
        deadline = time.monotonic() + 10
        while viewer.status().data_age is None:
            assert time.monotonic() < deadline, "no pose from NAOqi"
            time.sleep(0.1)
        assert red_columns(viewer.camera_frame("top", 640, 480).image).size == 0
        ball = viewer.props.spawn("object", ahead, size=0.1)
        columns = red_columns(viewer.camera_frame("top", 640, 480).image)
        assert columns.size > 10, "the object is not in the top camera's view"
        assert 200 < columns.mean() < 440, "the object is not straight ahead"
        viewer.props.despawn(ball.name)
        assert red_columns(viewer.camera_frame("top", 640, 480).image).size == 0
