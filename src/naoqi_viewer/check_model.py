"""check-model: cross-checks the model's forward kinematics against a NAOqi's, effector by effector."""

import logging
import math
import secrets
from dataclasses import dataclass

import mujoco
import numpy as np

from naoqi_viewer.model import JOINT_NAMES, NaoPose, PoseWriter, load_world
from naoqi_viewer.source import Target, connect, identify, normalize_url

_log = logging.getLogger(__name__)

EFFECTORS = ("Head", "LArm", "RArm", "LLeg", "RLeg")
_FRAME_TORSO = 0
_RANGE_MARGIN = 0.1  # fraction of each joint's range left out at both ends
_MOVE_TIME = 0.5  # s per configuration
# Torso at the world origin: a site's world frame is then its frame in torso.
_TORSO_AT_ORIGIN = tuple(float(v) for v in np.eye(4).flatten())


class TargetRefused(RuntimeError):
    """The target is a real robot and `allow_real` wasn't set: the check moves every joint."""


@dataclass(frozen=True)
class EffectorResult:
    effector: str
    position_mm: float  # worst position error
    position_sample: int  # the sample it occurred at (from 1)
    rotation_deg: float  # worst rotation error
    rotation_sample: int


@dataclass(frozen=True)
class CheckReport:
    url: str
    target: Target
    naoqi_version: str | None
    seed: int
    samples: int
    tolerance_mm: float
    tolerance_deg: float
    effectors: tuple[EffectorResult, ...]  # in EFFECTORS order

    def _failing(self) -> list[str]:
        """The effectors over either tolerance."""
        return [
            e.effector
            for e in self.effectors
            if e.position_mm > self.tolerance_mm or e.rotation_deg > self.tolerance_deg
        ]

    @property
    def passed(self) -> bool:
        return not self._failing()

    def format(self) -> str:
        version = self.naoqi_version or "unknown version"
        lines = [
            (
                f"check-model {self.url} ({self.target}, NAOqi {version}): "
                f"{self.samples} samples, seed {self.seed}"
            ),
            "",
            f"{'effector':<10}{'position (mm)':>15}{'sample':>8}{'rotation (°)':>15}{'sample':>8}",
        ]
        for e in self.effectors:
            lines.append(
                f"{e.effector:<10}{e.position_mm:>15.2f}{e.position_sample:>8}"
                f"{e.rotation_deg:>15.2f}{e.rotation_sample:>8}"
            )
        tolerance = f"{self.tolerance_mm:g} mm / {self.tolerance_deg:g}°"
        failing = self._failing()
        lines.append("")
        lines.append(
            f"FAIL: {', '.join(failing)} over {tolerance}"
            if failing
            else f"PASS: every effector within {tolerance}"
        )
        return "\n".join(lines)


def _rotation_deg(a: np.ndarray, b: np.ndarray) -> float:
    """The angle of the rotation taking `a` to `b`, in degrees."""
    cos = (np.trace(a.T @ b) - 1) / 2
    return math.degrees(math.acos(float(np.clip(cos, -1.0, 1.0))))


def run(
    url: str,
    samples: int = 20,
    seed: int | None = None,
    tolerance_mm: float = 2,
    tolerance_deg: float = 1,
    allow_real: bool = False,
) -> CheckReport:
    """Move the NAOqi robot through random configurations and compare its effectors with the model's.

    Raises TargetRefused for a real robot without `allow_real`, ConnectionError when NAOqi can't
    be reached, ValueError for bad arguments.
    """
    if samples < 1:
        raise ValueError(f"samples must be at least 1, got {samples}")
    if tolerance_mm <= 0 or tolerance_deg <= 0:
        raise ValueError("tolerances must be positive")
    if seed is None:
        seed = secrets.randbits(32)
    url = normalize_url(url)

    session = connect(url)
    try:
        info = identify(session, url)
        if info.target == "real" and not allow_real:
            raise TargetRefused(
                f"{url} is a real robot; check-model moves every joint, pass allow_real (--allow-real) to run it anyway"
            )
        _log.info(
            "checking the model against %s (%s), %d samples, seed %d",
            url,
            info.target,
            samples,
            seed,
        )
        motion = session.service("ALMotion")

        model = load_world(variant="placeholder")
        data = mujoco.MjData(model)
        writer = PoseWriter(model)
        body_names = list(motion.getBodyNames("Body"))
        names = [name for name in body_names if name in JOINT_NAMES]
        ranges = np.array([model.jnt_range[model.joint(n).id] for n in names])
        margin = _RANGE_MARGIN * (ranges[:, 1] - ranges[:, 0])
        low, high = ranges[:, 0] + margin, ranges[:, 1] - margin
        rng = np.random.default_rng(seed)

        worst_pos: dict[str, tuple[float, int]] = dict.fromkeys(EFFECTORS, (0.0, 1))
        worst_rot: dict[str, tuple[float, int]] = dict.fromkeys(EFFECTORS, (0.0, 1))
        motion.setStiffnesses("Body", 1.0)
        for sample in range(1, samples + 1):
            angles = rng.uniform(low, high)
            motion.angleInterpolation(
                names, [float(a) for a in angles], _MOVE_TIME, True
            )
            # NAOqi may clip or couple joints: the model is posed as the robot actually is.
            measured = list(motion.getAngles("Body", True))
            writer.apply(
                data, NaoPose.from_naoqi(body_names, measured, _TORSO_AT_ORIGIN)
            )
            for effector in EFFECTORS:
                naoqi = np.asarray(
                    motion.getTransform(effector, _FRAME_TORSO, True), dtype=float
                ).reshape(4, 4)
                site = data.site(effector)
                position = 1000 * float(np.linalg.norm(site.xpos - naoqi[:3, 3]))
                rotation = _rotation_deg(naoqi[:3, :3], site.xmat.reshape(3, 3))
                if position > worst_pos[effector][0]:
                    worst_pos[effector] = (position, sample)
                if rotation > worst_rot[effector][0]:
                    worst_rot[effector] = (rotation, sample)
            _log.debug("sample %d/%d done", sample, samples)
    finally:
        session.close()

    return CheckReport(
        url=url,
        target=info.target,
        naoqi_version=info.naoqi_version,
        seed=seed,
        samples=samples,
        tolerance_mm=tolerance_mm,
        tolerance_deg=tolerance_deg,
        effectors=tuple(
            EffectorResult(
                effector=e,
                position_mm=worst_pos[e][0],
                position_sample=worst_pos[e][1],
                rotation_deg=worst_rot[e][0],
                rotation_sample=worst_rot[e][1],
            )
            for e in EFFECTORS
        ),
    )
