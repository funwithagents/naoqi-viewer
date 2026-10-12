---
code:
  - src/naoqi_viewer/check_model.py
tests:
  - tests/test_check_model.py
  - tests-e2e/test_check_model_live.py
---

# Model check

**Status:** Implemented

## Purpose

Proving the model's kinematics match NAOqi's. `naoqi-viewer check-model` puts a NAOqi robot in random configurations and compares where NAOqi says the head, hands and feet are with where MuJoCo's forward kinematics puts them. It is the acceptance test for [model.md](model.md), and it catches a wrong axis, offset or effector site that a picture would hide.

## Decided

- **Entry points**: `check_model.run(url, samples=20, seed=None, tolerance_mm=2, tolerance_deg=1, allow_real=False) -> CheckReport` from Python, and `naoqi-viewer check-model URL [...]` from the [CLI](cli.md).
- **Targets**: a virtual robot or naoqi-sim. A target identified as `real` ([source.md](source.md)) is refused unless `allow_real` (`--allow-real`) is set, since the check moves every joint.
- **Per sample**:
  1. Draw a configuration uniformly within each joint's range, shrunk by 10% at both ends. The draw is seeded, and the seed is printed so a failure can be replayed.
  2. Make the body stiff with `setStiffnesses("Body", 1.0)` (once, at start), then send the configuration with `ALMotion.angleInterpolation(names, angles, 0.5, True)`, which blocks until reached.
  3. Read back `getAngles("Body", True)`, and `getTransform(effector, 0, True)` (frame 0 = torso) for `Head`, `LArm`, `RArm`, `LLeg`, `RLeg`.
  4. Pose the model with the **angles read back**, not the requested ones. NAOqi may clip or couple joints (arm self-collision avoidance, the shared hip motor), and the check is about kinematics, not control.
  5. Compare each effector with the matching model site, expressed in the `torso` frame: position error (mm) and rotation error (angle of the relative rotation, degrees).
- **Result**: a table of the worst position and rotation error per effector, with the sample where it occurred. Exit code 0 when every sample is under both tolerances, 1 otherwise.
- **Afterwards**: stiffness and posture are left as they are; the target is a simulated robot.

### Details

- **Joints**: the names from `getBodyNames("Body")` that the model knows (`model.JOINT_NAMES`), in NAOqi's order, with ranges from the model. `RHipYawPitch` is sent like any other joint. NAOqi drives it from `LHipYawPitch`, and the read-back angles say what it actually did.
- **Model side**: the model is loaded with the `placeholder` visuals (only the kinematics matter). It is posed with the torso at the world origin, so a site's world frame is its frame in `torso`.
- **Seed**: with `seed=None`, a random 32-bit seed is drawn. It is logged at the start of the run and stored in the report, so the same `seed` and `samples` replay the same configurations. Samples are numbered from 1.
- **Errors**: a refused `real` target raises `TargetRefused` (a `RuntimeError`) before anything moves. A NAOqi that can't be reached raises `ConnectionError` from [source.md](source.md)'s `connect`. `samples < 1` or a tolerance `<= 0` raises `ValueError`.

```python
EFFECTORS = ("Head", "LArm", "RArm", "LLeg", "RLeg")

@dataclass(frozen=True)
class EffectorResult:
    effector: str
    position_mm: float          # worst position error
    position_sample: int        # the sample it occurred at
    rotation_deg: float         # worst rotation error
    rotation_sample: int

@dataclass(frozen=True)
class CheckReport:
    url: str
    target: Target              # source.md's TargetInfo.target
    naoqi_version: str | None
    seed: int
    samples: int
    tolerance_mm: float
    tolerance_deg: float
    effectors: tuple[EffectorResult, ...]   # in EFFECTORS order

    @property
    def passed(self) -> bool: ...           # every effector within both tolerances
    def format(self) -> str: ...            # the table, the seed, and PASS or FAIL naming the effectors over tolerance
```

## Open questions

None currently.
