"""The NAO model: loading it into a scene, and writing NAOqi poses into MuJoCo state."""

import logging
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np

from naoqi_viewer import meshes

_log = logging.getLogger(__name__)

_PACKAGE_DIR = Path(__file__).parent
_MODEL_PATH = _PACKAGE_DIR / "models" / "nao.xml"
_SCENES_DIR = _PACKAGE_DIR / "scenes"
_DEFAULT_SCENE = "empty"

# The 26 H25 body joints, named and ordered as ALMotion.getBodyNames("Body").
JOINT_NAMES: tuple[str, ...] = (
    "HeadYaw", "HeadPitch",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
)  # fmt: skip

Variant = Literal["auto", "placeholder", "aldebaran"]


def resolve_variant(variant: Variant = "auto") -> Literal["placeholder", "aldebaran"]:
    """Pick the robot's visuals: Aldebaran's meshes when installed, the placeholder otherwise."""
    if variant == "placeholder":
        return "placeholder"
    if variant == "auto":
        return "aldebaran" if meshes.installed() is not None else "placeholder"
    if variant == "aldebaran":
        if meshes.installed() is None:
            raise FileNotFoundError(
                "Aldebaran's NAO meshes are not installed; run `naoqi-viewer fetch-meshes` first"
            )
        return "aldebaran"
    raise ValueError(
        f"unknown variant {variant!r}; expected 'auto', 'placeholder' or 'aldebaran'"
    )


def _scene_path(scene: str | Path | None) -> Path:
    if isinstance(scene, Path):
        return scene
    name = _DEFAULT_SCENE if scene is None else scene
    if name.endswith(".xml"):
        return Path(name)
    path = _SCENES_DIR / f"{name}.xml"
    if not path.is_file():
        bundled = sorted(p.stem for p in _SCENES_DIR.glob("*.xml"))
        raise ValueError(
            f"unknown scene {name!r}; bundled scenes: {', '.join(bundled)} (or pass a path to an MJCF file)"
        )
    return path


def load_world(
    scene: str | Path | None = None, variant: Variant = "auto", name: str | None = None
) -> mujoco.MjModel:
    """Compile a scene (bundled name or MJCF path, default "empty") with the NAO attached to it.

    `name` becomes the compiled model's name, which MuJoCo's viewer shows as its window title.
    """
    resolved = resolve_variant(variant)
    world = mujoco.MjSpec.from_file(str(_scene_path(scene)))
    if name is not None:
        world.modelname = name
    robot = mujoco.MjSpec.from_file(str(_MODEL_PATH))
    if resolved == "aldebaran":
        directory = meshes.installed()
        assert directory is not None  # resolve_variant checked it
        meshes.apply_visuals(robot, directory)
    # An empty prefix keeps the robot's names (joints, cameras, sites) as in nao.xml.
    world.worldbody.add_frame().attach_body(robot.body("torso"), "", "")
    return world.compile()


@dataclass(frozen=True)
class NaoPose:
    """A NAOqi pose: body joint angles and the torso's world pose."""

    angles: Mapping[str, float]  # joint name -> rad (LHand/RHand: 0..1 open fraction)
    torso_pos: tuple[float, float, float]  # m, world frame
    torso_quat: tuple[float, float, float, float]  # w, x, y, z

    @classmethod
    def from_naoqi(
        cls,
        names: Sequence[str],
        angles: Sequence[float],
        torso_transform: Sequence[float],
    ) -> "NaoPose":
        """Build a pose from getBodyNames, getAngles and getTransform("Torso", ...) (16 floats, row-major 4x4)."""
        matrix = np.asarray(torso_transform, dtype=float).reshape(4, 4)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, matrix[:3, :3].flatten())
        x, y, z = (float(v) for v in matrix[:3, 3])
        w, qx, qy, qz = (float(v) for v in quat)
        return cls(
            angles={
                name: float(angle) for name, angle in zip(names, angles, strict=True)
            },
            torso_pos=(x, y, z),
            torso_quat=(w, qx, qy, qz),
        )


@dataclass(frozen=True)
class _Follower:
    """A joint driven by another through a joint equality (the fingers following their hand)."""

    qpos_adr: int
    leader_adr: int
    qpos0: float
    leader_qpos0: float
    coefs: tuple[float, ...]


class PoseWriter:
    """Writes NaoPoses into an MjData of a model loaded with load_world."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self._model = model
        self._root_adr = int(model.jnt_qposadr[model.joint("root").id])
        self._joint_adr = {
            name: int(model.jnt_qposadr[model.joint(name).id])
            for name in JOINT_NAMES
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0
        }
        self._followers: list[_Follower] = []
        for i in range(model.neq):
            if model.eq_type[i] != mujoco.mjtEq.mjEQ_JOINT:
                continue
            follower, leader = int(model.eq_obj1id[i]), int(model.eq_obj2id[i])
            # Body joints (RHipYawPitch included) are taken as NAOqi reports them.
            if model.joint(follower).name in JOINT_NAMES:
                continue
            self._followers.append(
                _Follower(
                    qpos_adr=int(model.jnt_qposadr[follower]),
                    leader_adr=int(model.jnt_qposadr[leader]),
                    qpos0=float(model.qpos0[model.jnt_qposadr[follower]]),
                    leader_qpos0=float(model.qpos0[model.jnt_qposadr[leader]]),
                    coefs=tuple(float(c) for c in model.eq_data[i][:5]),
                )
            )
        self._ignored: set[str] = set()

    def apply(self, data: mujoco.MjData, pose: NaoPose) -> None:
        """Write the pose into data.qpos, couple the fingers, and update positions and cameras."""
        qpos = data.qpos
        qpos[self._root_adr : self._root_adr + 3] = pose.torso_pos
        qpos[self._root_adr + 3 : self._root_adr + 7] = pose.torso_quat
        for name, angle in pose.angles.items():
            adr = self._joint_adr.get(name)
            if adr is None:
                if name not in self._ignored:
                    self._ignored.add(name)
                    _log.info("ignoring joint %r, unknown to the model", name)
                continue
            qpos[adr] = angle
        for f in self._followers:
            x = qpos[f.leader_adr] - f.leader_qpos0
            qpos[f.qpos_adr] = f.qpos0 + sum(c * x**k for k, c in enumerate(f.coefs))
        mujoco.mj_kinematics(self._model, data)
        # mj_kinematics leaves cameras alone; the head cameras' renders need them posed too.
        mujoco.mj_camlight(self._model, data)
