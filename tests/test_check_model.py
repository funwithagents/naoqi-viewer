import math
from collections.abc import Iterator

import mujoco
import numpy as np
import pytest
from mock_naoqi import MockNaoqi

from naoqi_viewer import check_model
from naoqi_viewer.check_model import EFFECTORS, CheckReport, TargetRefused
from naoqi_viewer.model import JOINT_NAMES, load_world


@pytest.fixture
def real_robot() -> Iterator[MockNaoqi]:
    mock = MockNaoqi(target="real", version="2.8.7.4").start()
    try:
        yield mock
    finally:
        mock.close()


def offset(
    translation=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), degrees=0.0
) -> list[float]:
    """A 4x4 offset, row-major: a rotation of `degrees` about `axis`, then `translation`."""
    quat = np.empty(4)
    mujoco.mju_axisAngle2Quat(
        quat, np.asarray(axis, dtype=float), math.radians(degrees)
    )
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quat)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.reshape(3, 3)
    matrix[:3, 3] = translation
    return matrix.flatten().tolist()


def result(report: CheckReport, effector: str) -> check_model.EffectorResult:
    return next(e for e in report.effectors if e.effector == effector)


def sent_configurations(mock: MockNaoqi) -> list[list[float]]:
    return [list(args[1]) for args in mock.calls_to("angleInterpolation")]


def test_the_models_own_kinematics_pass(mock_naoqi: tuple[str, MockNaoqi]):
    url, _ = mock_naoqi
    report = check_model.run(url, samples=5, seed=1)
    assert report.passed
    assert [e.effector for e in report.effectors] == list(EFFECTORS)
    for e in report.effectors:
        assert e.position_mm < 1e-6
        assert e.rotation_deg < 1e-3
    assert (report.url, report.target, report.samples, report.seed) == (
        url,
        "virtual",
        5,
        1,
    )


def test_an_effector_offset_is_measured_and_fails_the_check(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    mock.set_effector_error("LArm", offset(translation=(0.003, 0.004, 0.0)))
    report = check_model.run(url, samples=3, seed=2)
    assert not report.passed
    assert result(report, "LArm").position_mm == pytest.approx(5.0, abs=1e-6)
    assert result(report, "LArm").rotation_deg < 1e-3
    for effector in ("Head", "RArm", "LLeg", "RLeg"):
        assert result(report, effector).position_mm < 1e-6
    assert "FAIL: LArm over 2 mm / 1°" in report.format()


def test_an_effector_rotation_is_measured(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_effector_error("Head", offset(axis=(0.0, 1.0, 0.0), degrees=2.0))
    report = check_model.run(url, samples=3, seed=3)
    assert result(report, "Head").rotation_deg == pytest.approx(2.0, abs=1e-6)
    assert result(report, "Head").position_mm < 1e-6
    assert not report.passed


def test_a_small_error_within_tolerance_passes(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    mock.set_effector_error("RLeg", offset(translation=(0.0, 0.0, 0.0015), degrees=0.5))
    report = check_model.run(url, samples=2, seed=4)
    assert report.passed
    assert "PASS" in report.format()
    # A tighter tolerance turns the same run into a failure.
    assert not check_model.run(url, samples=2, seed=4, tolerance_mm=1).passed


def test_the_model_is_posed_with_the_angles_read_back(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    # NAOqi stops LElbowRoll short of every request: posing the model with the requested
    # angles would put the left hand centimetres away.
    mock.clip_joint("LElbowRoll", -0.05, -0.04)
    report = check_model.run(url, samples=4, seed=5)
    assert report.passed


def test_configurations_stay_within_the_shrunk_joint_ranges(
    mock_naoqi: tuple[str, MockNaoqi],
):
    url, mock = mock_naoqi
    check_model.run(url, samples=10, seed=6)
    model = load_world(variant="placeholder")
    names = mock.calls_to("angleInterpolation")[0][0]
    assert list(names) == list(JOINT_NAMES)
    for configuration in sent_configurations(mock):
        for name, angle in zip(names, configuration, strict=True):
            low, high = model.jnt_range[model.joint(name).id]
            margin = 0.1 * (high - low)
            assert low + margin <= angle <= high - margin


def test_a_seed_replays_the_same_configurations():
    def configurations(seed: int | None) -> tuple[list[list[float]], int]:
        mock = MockNaoqi().start()
        try:
            report = check_model.run(mock.url, samples=3, seed=seed)
            return sent_configurations(mock), report.seed
        finally:
            mock.close()

    first, seed = configurations(None)
    replayed, _ = configurations(seed)
    other, _ = configurations(seed + 1)
    assert len(first) == 3
    assert replayed == first
    assert other != first


def test_stiffness_is_set_once_before_moving(mock_naoqi: tuple[str, MockNaoqi]):
    url, mock = mock_naoqi
    check_model.run(url, samples=3, seed=7)
    assert mock.calls_to("setStiffnesses") == [("Body", 1.0)]
    motions = [
        m for _, m, _ in mock.calls if m in ("setStiffnesses", "angleInterpolation")
    ]
    assert motions[0] == "setStiffnesses"
    assert all(args[2:] == (0.5, True) for args in mock.calls_to("angleInterpolation"))


def test_a_real_robot_is_refused_before_anything_moves(real_robot: MockNaoqi):
    with pytest.raises(TargetRefused, match="allow_real"):
        check_model.run(real_robot.url, samples=2, seed=8)
    assert real_robot.calls_to("setStiffnesses") == []
    assert real_robot.calls_to("angleInterpolation") == []


def test_a_real_robot_is_checked_with_allow_real(real_robot: MockNaoqi):
    report = check_model.run(real_robot.url, samples=2, seed=9, allow_real=True)
    assert report.passed
    assert (report.target, report.naoqi_version) == ("real", "2.8.7.4")


def test_naoqi_sim_is_accepted():
    mock = MockNaoqi(target="naoqi-sim", version="0.3.0").start()
    try:
        report = check_model.run(mock.url, samples=2, seed=10)
    finally:
        mock.close()
    assert report.passed
    assert report.target == "naoqi-sim"


@pytest.mark.parametrize(
    "kwargs",
    [{"samples": 0}, {"tolerance_mm": 0}, {"tolerance_deg": -1}],
)
def test_bad_arguments_are_rejected_before_connecting(kwargs: dict[str, float]):
    with pytest.raises(ValueError):
        check_model.run("tcp://127.0.0.1:1", **kwargs)  # type: ignore[arg-type]


def test_the_report_names_the_worst_sample(mock_naoqi: tuple[str, MockNaoqi]):
    url, _ = mock_naoqi
    report = check_model.run(url, samples=4, seed=11)
    for e in report.effectors:
        assert 1 <= e.position_sample <= 4
        assert 1 <= e.rotation_sample <= 4
    lines = report.format().splitlines()
    assert (
        lines[0]
        == f"check-model {url} (virtual, NAOqi unknown version): 4 samples, seed 11"
    )
    assert (
        len(
            [
                line
                for line in lines
                if line.split()[:1] and line.split()[0] in EFFECTORS
            ]
        )
        == 5
    )
