---
code:
  - src/naoqi_viewer/model.py
  - src/naoqi_viewer/models/nao.xml
  - src/naoqi_viewer/scenes/empty.xml
  - third_party/nao_description/nao.urdf
tests:
  - tests/test_model.py
---

# Model

**Status:** Implemented

## Purpose

The NAO as a MuJoCo model, and the one place that turns a NAOqi pose into MuJoCo state. Every other part of naoqi-viewer (the viewer, check-model) loads the robot and poses it through this module. The committed model draws the robot with styled placeholder shapes of our own, so it ships under MIT. When the user has installed Aldebaran's meshes ([meshes.md](meshes.md)), the same model is loaded with those meshes in place of the placeholder shapes.

## Decided

### One committed model, maintained by hand

- `src/naoqi_viewer/models/nao.xml` is the NAO model. It sits inside the package so it ships in the wheel. The NAO doesn't change, so there is no build tool: the file is the source of truth and is edited by hand (for example, when `check-model` finds a wrong effector offset).
- **Origin**: it is converted once from the NAO V5 (H25) URDF of [ros-naoqi/nao_robot](https://github.com/ros-naoqi/nao_robot) (BSD-3-Clause), file `nao_description/urdf/naoV50_generated_urdf/nao.urdf` (already expanded from xacro) at commit `67476469a1371b00b17538eb6ea336367ece7d44`, during the model's implementation plan. That conversion (MuJoCo's URDF importer plus manual edits) is a one-off step of the plan, not code kept in the repository.
- **Provenance**: the URDF stays vendored at `third_party/nao_description/nao.urdf`, with its `LICENSE` next to it and a `SOURCE` file recording the upstream repository, commit and path. `THIRD_PARTY_NOTICES.md` carries the copyright notice. Nothing reads the URDF at runtime. Tests use it as a reference (see Tests below).
- The URDF provides the kinematics (link frames, joint axes, limits), masses and inertias. Its `<visual>` and `<collision>` mesh references (`package://nao_meshes/...`) are not used in `nao.xml`. Their file names and origins feed the Aldebaran visual table in [meshes.md](meshes.md).

### Model contents

- **Root**: `torso` body with a free joint named `root`. A world pose maps directly to `qpos[0:7]` as position, then quaternion `(w, x, y, z)`. The URDF's fixed `base_link` is folded into `torso`.
- **Joints**: the 26 H25 body joints, named exactly as NAOqi's `ALMotion.getBodyNames("Body")`, with ranges from the URDF limits. In NAOqi order:
  `HeadYaw HeadPitch LShoulderPitch LShoulderRoll LElbowYaw LElbowRoll LWristYaw LHand LHipYawPitch LHipRoll LHipPitch LKneePitch LAnklePitch LAnkleRoll RHipYawPitch RHipRoll RHipPitch RKneePitch RAnklePitch RAnkleRoll RShoulderPitch RShoulderRoll RElbowYaw RElbowRoll RWristYaw RHand`.
  `model.JOINT_NAMES` exposes this tuple.
- **Frames**: the URDF's massless fixed-joint links (cameras, effectors, sole frames) are not bodies in `nao.xml`; their poses become sites and cameras on the parent body. (MuJoCo's URDF importer merges static bodies by default, so the conversion must keep these frames, for example with `fusestatic="false"` or by reading them from the URDF directly.)
- **Hands**: `LHand` and `RHand` are joints with range `[0, 1]` (NAOqi's open fraction). The URDF finger joints are kept and follow the hand joint through its `mimic` multiplier and offset. In the URDF, `LHand` rotates the `l_gripper` frame itself, which NAOqi's arm effector doesn't do, so nothing but that dummy frame hangs off the hand joint: the effector sites sit on the wrist body (see Effector sites).
- **Couplings**: `LHipYawPitch`/`RHipYawPitch` (one motor) and the finger mimics are joint equality constraints in the model, for any future physics use. Since naoqi-viewer never steps physics, constraints are not enforced at render time. `PoseWriter` applies the finger couplings itself (see below). `RHipYawPitch` is taken as reported by NAOqi.
- **Collision geoms**: hand-tuned primitives: capsules for limbs, boxes for feet and torso, a sphere-capped capsule for the head. They sit in geom group 3, hidden by default, with contacts between adjacent bodies excluded. They are the same whichever visuals are loaded.
- **Visual geoms**: see Placeholder visuals below. Every visual geom is visual-only (`contype=conaffinity=0`) and in group 1.
- **Actuators**: one `position` actuator per joint, named after the joint, all in a default class `nao_joint` that holds `kp`, `forcerange` and `damping` as named parameters. The values are placeholders until system identification, and they are unused by the viewer.
- **Sensors**: `jointpos` per joint; `gyro` and `accelerometer` at the `imu` site in the torso; 4 `touch` sensors per foot on sites at the URDF FSR frames (`LFsrFL LFsrFR LFsrRL LFsrRR`, same for `R`). `touch` rather than `force`: a MuJoCo `force` sensor measures the whole body's interaction with its parent, so four on one foot would all read the same.
- **Cameras**: `CameraTop` and `CameraBottom` at the URDF camera frames, oriented so that MuJoCo's view (−Z forward, +Y up) matches the NAO image (not mirrored, horizon level when the head is level). `fovy` uses the V5 vertical field of view, 47.64°. A head camera's render must not show the robot's own head shell, with either set of visuals.
- **Effector sites**: `Head`, `LArm`, `RArm`, `LLeg`, `RLeg`, at the positions of NAOqi's end effectors for those chains (`Head` at the `Head` link frame, the arms at the URDF `l_gripper`/`r_gripper` offset but fixed to the wrist body, the legs at `l_sole`/`r_sole`), so [check_model.md](check_model.md) can compare `ALMotion.getTransform(name, FRAME_TORSO, True)` with the site frame directly. One deviation from the URDF: its `r_gripper` z offset is −0.01213 m, but `l_gripper`'s is −0.01231 m and NAOqi's `RArm` matches the left value. `check-model` measured 0.18 mm against NAOqi 2.1, so `RArm` uses −0.01231 m.
- **Masses and inertias** come from the URDF.

### Placeholder visuals

The default look of the robot: styled MuJoCo primitives that read as a NAO, authored by us under MIT. They are written directly in `nao.xml`, with no mesh files.

- **Shapes**, sized from the URDF link lengths and NAO's general proportions:
  - head: an ellipsoid shell, two dark eye discs, ear discs on the sides;
  - torso: a rounded body (ellipsoid or capsule);
  - arms: sphere shoulders, capsule upper arms and forearms, a small hand block with finger capsules on the finger bodies, so `LHand`/`RHand` visibly open and close;
  - legs: capsule thighs and shins, with flattened ellipsoid shin covers;
  - feet: flattened rounded boxes.
- **Colors**: named materials `nao_white` (main shell), `nao_grey` (joints and limbs), `nao_accent` (ear discs, one accent color), `nao_eye` (dark eyes).
- **Class**: every placeholder visual geom is in the default class `nao_visual`. That is how [meshes.md](meshes.md) finds and removes them when Aldebaran's meshes are loaded instead.
- **Never derived from Aldebaran's meshes**: no tracing, decimating or fitting shapes to the mesh files. The meshes are CC BY-NC-ND (no derivatives), and the placeholders must stay our own MIT work.

### Visuals: `placeholder` and `aldebaran`

The model has one set of kinematics, and two possible visuals, chosen when the world is loaded:

- `"placeholder"`: `nao.xml` as committed.
- `"aldebaran"`: `nao.xml` with the `nao_visual` geoms replaced by Aldebaran's meshes, from the user's local install ([meshes.md](meshes.md)).
- `"auto"`: `"aldebaran"` when the meshes are installed, `"placeholder"` otherwise.

Kinematics, joints, sensors, cameras and collision geoms are therefore identical whichever visuals are loaded.

### Scenes

The model holds the robot only. A scene is an MJCF file with the world around it (floor, lights, objects). `load_world(scene, variant)` attaches the robot into the scene with `MjSpec.attach`, so any scene works with either variant. Scenes ship in `src/naoqi_viewer/scenes/`. `empty.xml` (checker floor, one light) is the default; the other bundled scenes are listed in [api.md](api.md). `load_world` takes a bundled scene name or a path to a user file. With `props=True` (sim mode) it also appends the props pool to the scene's spec before compiling, hidden mocap bodies a caller shows and moves at runtime ([props.md](props.md)); mirror mode passes `False` and gets the scene as written.

### Runtime API (`model.py`)

```python
JOINT_NAMES: tuple[str, ...]                        # the 26 names above, NAOqi order
Variant = Literal["auto", "placeholder", "aldebaran"]

def resolve_variant(variant: Variant = "auto") -> Literal["placeholder", "aldebaran"]
def load_world(scene: str | Path | None = None, variant: Variant = "auto", name: str | None = None, *, props: bool = False) -> mujoco.MjModel

@dataclass(frozen=True)
class NaoPose:
    angles: Mapping[str, float]                     # joint name -> rad (LHand/RHand: 0..1)
    torso_pos: tuple[float, float, float]           # m, world frame
    torso_quat: tuple[float, float, float, float]   # w, x, y, z

    @classmethod
    def from_naoqi(cls, names, angles, torso_transform) -> NaoPose   # transform: 16 floats, row-major 4x4

class PoseWriter:
    def __init__(self, model: mujoco.MjModel): ...  # resolves joint -> qpos address once
    def apply(self, data: mujoco.MjData, pose: NaoPose) -> None     # writes qpos, finger couplings, runs mj_kinematics + mj_camlight
```

- `resolve_variant("auto")` returns `"aldebaran"` when [meshes.md](meshes.md) reports an installed, accepted mesh set, and `"placeholder"` otherwise. Asking for `"aldebaran"` when the meshes are missing raises an error that names `naoqi-viewer fetch-meshes`.
- `load_world` loads `nao.xml` as an `MjSpec`. For `"aldebaran"` it swaps the visuals ([meshes.md](meshes.md)), then attaches the robot into the scene and compiles. `name`, when given, becomes the compiled model's name, which MuJoCo's viewer shows as its window title ([viewer.md](viewer.md)).
- `PoseWriter.apply` runs `mj_camlight` after `mj_kinematics`, because `mj_kinematics` doesn't pose the cameras and the head-camera renders need them.
- `PoseWriter.apply` ignores names the model doesn't know (logged once) and leaves joints missing from the pose unchanged. Values outside a joint's range are written as given: the viewer shows what NAOqi reports.

### Tests

`tests/test_model.py` checks the committed `nao.xml` against the vendored URDF, so a hand edit can't silently break the kinematics: the joint names and order, each joint's axis, range and parent-to-child frame, and the camera and effector frames. Forward kinematics as a whole is validated against NAOqi by [check_model.md](check_model.md).

## Open questions

1. **Effector offsets**: the exact NAOqi end-effector points (hand and foot effectors are offset from the wrist and ankle frames). These come from the URDF frames where they match, otherwise from the NAOqi documentation. `check-model` validates them. Against NAOqi 2.1 every effector now matches within 0.03 mm and 0.03°. NAOqi 2.8 is still to check.
2. **NAO V6**: v1 is V5 geometry. Whether V6 (NAOqi 2.8) needs its own URDF and camera field of view is open.
