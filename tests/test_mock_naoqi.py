import numpy as np
import pytest
from mock_naoqi import FRAME_TORSO, FRAME_WORLD, MockNaoqi

from naoqi_viewer.source import connect

# The left sole below the torso origin at zero pose, from the URDF: hip offset, thigh, tibia, foot.
LEFT_SOLE_AT_ZERO = (0.0, 0.05, -(0.085 + 0.1 + 0.1029 + 0.04511))
SHIFTED_TORSO = (1, 0, 0, 0.5, 0, 1, 0, 0.2, 0, 0, 1, 0.3, 0, 0, 0, 1)


def transform(motion, name: str, frame: int) -> np.ndarray:
    return np.array(motion.getTransform(name, frame, True)).reshape(4, 4)


def test_effector_transforms_follow_the_model_kinematics(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    session = connect(url)
    try:
        motion = session.service("ALMotion")
        np.testing.assert_allclose(
            transform(motion, "LLeg", FRAME_TORSO)[:3, 3], LEFT_SOLE_AT_ZERO, atol=1e-6
        )

        mock.set_pose(
            {"LHipPitch": -0.6, "LKneePitch": 1.2, "LAnklePitch": -0.6},
            torso_transform=SHIFTED_TORSO,
        )
        bent = transform(motion, "LLeg", FRAME_TORSO)
        assert bent[2, 3] > LEFT_SOLE_AT_ZERO[2] + 0.02  # a bent knee lifts the foot
        np.testing.assert_allclose(
            bent[:3, :3], np.eye(3), atol=1e-9
        )  # hip and ankle pitch cancel out
        world = transform(motion, "LLeg", FRAME_WORLD)
        np.testing.assert_allclose(
            world, np.array(SHIFTED_TORSO).reshape(4, 4) @ bent, atol=1e-9
        )
    finally:
        session.close()


def test_effector_error_offsets_the_reported_transform(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    session = connect(url)
    try:
        motion = session.service("ALMotion")
        mock.set_pose({"RShoulderPitch": 0.4, "RElbowRoll": 0.8})
        true = transform(motion, "RArm", FRAME_TORSO)
        left = transform(motion, "LArm", FRAME_TORSO)
        mock.set_effector_error(
            "RArm", (1, 0, 0, 0.003, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
        )
        off = transform(motion, "RArm", FRAME_TORSO)
        np.testing.assert_allclose(
            off[:3, 3] - true[:3, 3], 0.003 * true[:3, 0], atol=1e-9
        )
        np.testing.assert_allclose(
            transform(motion, "LArm", FRAME_TORSO), left
        )  # other effectors unaffected
    finally:
        session.close()


def test_angle_interpolation_moves_the_robot(mock_naoqi: tuple[str, MockNaoqi]):
    url, _ = mock_naoqi
    session = connect(url)
    try:
        motion = session.service("ALMotion")
        motion.angleInterpolation(["HeadYaw", "LHand"], [0.7, 0.4], 0.5, True)
        assert motion.getAngles(["HeadYaw", "LHand"], True) == pytest.approx([0.7, 0.4])
        assert motion.getAngles("HeadYaw", False) == pytest.approx([0.7])
    finally:
        session.close()
