import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field

import mujoco
import pytest

from naoqi_viewer import NaoqiViewerConfig, client
from naoqi_viewer.model import NaoPose, load_world
from naoqi_viewer.source import Sample, TargetInfo
from naoqi_viewer.viewer import (
    ATTRIBUTION,
    KEY_TOGGLE_STATUS,
    ViewerState,
    run_headless,
)

UPRIGHT = ((0.0, 0.0, 0.3332), (1.0, 0.0, 0.0, 0.0))
RIGHT_ARM_BODIES = {"RShoulder", "RBicep", "RElbow", "RForeArm", "r_wrist"}


@dataclass
class FakeSource:
    """A PoseSource whose samples and target the test sets directly."""

    info: TargetInfo | None = None
    sample: Sample | None = None
    samples_per_second: float = 0.0
    seq: int = field(default=0)

    def latest(self) -> Sample | None:
        return self.sample

    def rate(self) -> float:
        return self.samples_per_second

    def close(self) -> None:
        pass

    def push(
        self,
        angles: dict[str, float],
        commanded: dict[str, float] | None = None,
        age: float = 0.0,
    ) -> Sample:
        self.seq += 1
        pose = NaoPose(angles, *UPRIGHT)
        self.sample = Sample(
            self.seq,
            pose,
            None if commanded is None else NaoPose(commanded, *UPRIGHT),
            time.monotonic() - age,
        )
        return self.sample


@pytest.fixture(scope="module")
def world() -> mujoco.MjModel:
    return load_world(variant="placeholder")


def joint(state: ViewerState, name: str) -> float:
    return float(state.data.qpos[state.model.jnt_qposadr[state.model.joint(name).id]])


def status(state: ViewerState) -> tuple[str, str] | None:
    for _, gridpos, left, right in state.texts(time.monotonic()):
        if gridpos == mujoco.mjtGridPos.mjGRID_TOPLEFT:
            return left or "", right or ""
    return None


def status_lines(state: ViewerState) -> tuple[str, str]:
    lines = status(state)
    assert lines is not None
    return lines


def test_update_poses_the_model_from_new_samples_only(world: mujoco.MjModel):
    source = FakeSource()
    state = ViewerState(world, source)
    assert state.update() is None

    first = source.push({"HeadYaw": 0.6})
    assert state.update() is first
    assert state.sample is first
    assert joint(state, "HeadYaw") == pytest.approx(0.6)

    state.data.qpos[world.jnt_qposadr[world.joint("HeadYaw").id]] = 0.0
    assert state.update() is None  # same seq: the pose is not written again
    assert joint(state, "HeadYaw") == 0.0

    source.push({"HeadYaw": -0.2})
    state.update()
    assert joint(state, "HeadYaw") == pytest.approx(-0.2)


def test_status_overlay_lines(world: mujoco.MjModel):
    source = FakeSource()
    state = ViewerState(world, source)
    assert status(state) == ("connecting\ndata age", "\nNO DATA")

    source.info = TargetInfo("tcp://127.0.0.1:9559", "naoqi-sim", "0.3.0")
    source.samples_per_second = 49.6
    source.push({}, age=0.04)
    left, right = status_lines(state)
    assert left.splitlines() == ["naoqi-sim", "NAOqi", "rate", "data age"]
    url, version, rate, age = right.splitlines()
    assert (url, version, rate) == ("tcp://127.0.0.1:9559", "0.3.0", "50 Hz")
    assert age.endswith(" ms") and 40 <= int(age.removesuffix(" ms")) < 500


def test_status_reads_stale_after_half_a_second_and_unknown_version(
    world: mujoco.MjModel,
):
    source = FakeSource(info=TargetInfo("tcp://nao.local:9559", "virtual", None))
    state = ViewerState(world, source)
    source.push({}, age=0.6)
    _, right = status_lines(state)
    assert right.splitlines()[1] == "unknown"
    assert right.splitlines()[3] == "STALE"


def test_f9_toggles_the_status_overlay_and_letters_do_not(world: mujoco.MjModel):
    state = ViewerState(world, FakeSource())
    state.on_key(ord("O"))
    assert status(state) is not None
    state.on_key(KEY_TOGGLE_STATUS)
    assert status(state) is None
    state.on_key(KEY_TOGGLE_STATUS)
    assert status(state) is not None


def test_attribution_only_when_given(world: mujoco.MjModel):
    def bottom_right(state: ViewerState) -> list[str]:
        texts = state.texts(time.monotonic())
        return [
            left or ""
            for _, pos, left, _ in texts
            if pos == mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT
        ]

    assert bottom_right(ViewerState(world, FakeSource())) == []
    with_meshes = ViewerState(world, FakeSource(), attribution=ATTRIBUTION)
    with_meshes.on_key(KEY_TOGGLE_STATUS)
    assert bottom_right(with_meshes) == [ATTRIBUTION]


def visible_ghost_bodies(state: ViewerState, scene: mujoco.MjvScene) -> set[str]:
    state.draw_ghost(scene)
    bodies = set()
    for i in range(scene.ngeom):
        geom = scene.geoms[i]
        assert (
            state.model.geom_group[geom.objid] == 1
        )  # robot visuals only, never the floor
        if geom.rgba[3] > 0:
            bodies.add(state.model.body(state.model.geom_bodyid[geom.objid]).name)
    return bodies


def test_ghost_shows_only_the_bodies_off_their_commanded_pose(world: mujoco.MjModel):
    source = FakeSource()
    state = ViewerState(world, source, ghost=True)
    scene = mujoco.MjvScene(world, maxgeom=2000)

    source.push({"RShoulderPitch": -0.5}, commanded={"RShoulderPitch": -1.3})
    state.update()
    ghost = visible_ghost_bodies(state, scene)
    assert RIGHT_ARM_BODIES <= ghost
    assert not {"torso", "Head", "LBicep", "l_ankle"} & ghost

    source.push({"RShoulderPitch": -1.3}, commanded={"RShoulderPitch": -1.3})
    state.update()
    assert visible_ghost_bodies(state, scene) == set()


def test_no_ghost_without_commanded_angles_or_when_disabled(world: mujoco.MjModel):
    scene = mujoco.MjvScene(world, maxgeom=2000)
    source = FakeSource()
    state = ViewerState(world, source, ghost=True)
    source.push({"HeadYaw": 0.3})
    state.update()
    state.draw_ghost(scene)
    assert scene.ngeom == 0

    plain = ViewerState(world, source)
    source.push({"HeadYaw": 0.3}, commanded={"HeadYaw": -0.3})
    plain.update()
    plain.draw_ghost(scene)
    assert scene.ngeom == 0


def test_run_headless_poses_the_model_each_frame_until_stopped(world: mujoco.MjModel):
    source = FakeSource()
    stop = threading.Event()
    frames: list[tuple[int, float]] = []
    pushed_second = threading.Event()

    def on_frame(model: mujoco.MjModel, data: mujoco.MjData, sample: Sample | None):
        yaw = float(data.qpos[model.joint("HeadYaw").qposadr[0]])
        frames.append((sample.seq if sample is not None else 0, yaw))
        if len(frames) == 3:
            source.push({"HeadYaw": 0.4})
        if len(frames) == 6:
            source.push({"HeadYaw": -0.2})
            pushed_second.set()
        if len(frames) >= 9:
            stop.set()

    loop = threading.Thread(
        target=run_headless,
        args=(world, source),
        kwargs={"on_frame": on_frame, "stop": stop},
    )
    started = time.monotonic()
    loop.start()
    loop.join(timeout=5)
    assert not loop.is_alive(), "run_headless did not return once stopped"
    assert pushed_second.is_set()
    assert frames[0] == (0, 0.0)  # no pose yet
    assert frames[3] == (1, pytest.approx(0.4)) and frames[6] == (
        2,
        pytest.approx(-0.2),
    )
    assert time.monotonic() - started >= 8 / 60 * 0.9  # paced at the frame rate


@pytest.mark.skipif(
    sys.platform == "win32",
    reason="MuJoCo renders offscreen on Windows through GLFW itself, in a hidden window",
)
def test_a_headless_viewer_process_does_not_load_the_window_stack():
    # With the environment launch() gives a headless viewer: on Linux, MuJoCo's default
    # backend is GLFW itself, which MUJOCO_GL=egl replaces.
    headless = NaoqiViewerConfig(mode="sim", headless=True)
    code = "import sys, naoqi_viewer.viewer_process; print(sorted(m for m in ('glfw', 'mujoco.viewer') if m in sys.modules))"
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=client._viewer_env(headless),
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout.strip() == "[]"
