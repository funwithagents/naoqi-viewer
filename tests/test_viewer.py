import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field

import mujoco
import numpy as np
import pytest

from naoqi_viewer import NaoqiViewerConfig, client
from naoqi_viewer.cameras import CameraImage
from naoqi_viewer.model import NaoPose, load_world
from naoqi_viewer.people import (
    PEOPLE_STALE_AFTER,
    PERSON_HIDDEN_RGBA,
    PERSON_RADIUS,
    PERSON_VISIBLE_RGBA,
    People,
    Person,
)
from naoqi_viewer.source import Sample, TargetInfo
from naoqi_viewer.viewer import (
    ATTRIBUTION,
    KEY_TOGGLE_CAMERAS,
    KEY_TOGGLE_MIRROR,
    KEY_TOGGLE_STATUS,
    ViewerState,
    run_headless,
)

VIEW = (0, 0, 1600, 1000)

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


@dataclass
class FakeCameras:
    """An ImageSource whose images the test sets directly."""

    images: dict[str, CameraImage] = field(default_factory=dict)
    seq: int = 0

    def latest(self, camera: str) -> CameraImage | None:
        return self.images.get(camera)

    def rate(self) -> float:
        return 10.0

    def close(self) -> None:
        pass

    def push(self, camera: str, blue: int, age: float = 0.0) -> CameraImage:
        self.seq += 1
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        image[..., 2] = blue
        self.images[camera] = CameraImage(
            camera,  # type: ignore[arg-type]
            image,
            self.seq,
            time.monotonic() - age,
            None,
        )
        return self.images[camera]


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


def test_camera_pictures_fill_both_slots_top_camera_first(world: mujoco.MjModel):
    cameras = FakeCameras()
    state = ViewerState(world, FakeSource(), cameras=cameras)
    pictures = state.pictures(VIEW)
    assert pictures is not None and len(pictures) == 2
    (top_rect, top), (bottom_rect, bottom) = pictures
    assert top_rect[1] > bottom_rect[1]  # the top camera's picture is the higher one
    assert top.shape == (top_rect[3], top_rect[2], 3) == (300, 400, 3)
    assert top[150, 200].tolist() == [40, 40, 40]  # a placeholder before any image
    assert top[0, 0].tolist() == [220, 220, 220]  # framed

    cameras.push("top", 7)
    pictures = state.pictures(VIEW)
    assert pictures is not None
    (_, top), (_, bottom) = pictures
    assert top[150, 200].tolist() == [0, 0, 7] and top[0, 0].tolist() == [220, 220, 220]
    assert bottom[150, 200].tolist() == [40, 40, 40]  # still waiting

    assert state.pictures((0, 0, 100, 80)) is None  # too small a view


def test_pictures_change_on_new_images_resizes_and_f10_only(world: mujoco.MjModel):
    cameras = FakeCameras()
    state = ViewerState(world, FakeSource(), cameras=cameras)
    assert state.pictures_changed(VIEW)  # the first draw
    assert not state.pictures_changed(VIEW)
    cameras.push("bottom", 1)
    assert state.pictures_changed(VIEW) and not state.pictures_changed(VIEW)
    assert state.pictures_changed((0, 0, 1200, 800))  # resized
    state.on_key(KEY_TOGGLE_CAMERAS)
    assert state.pictures_changed((0, 0, 1200, 800))
    assert state.pictures((0, 0, 1200, 800)) is None  # hidden
    state.on_key(KEY_TOGGLE_CAMERAS)
    assert state.pictures_changed((0, 0, 1200, 800))
    assert state.pictures((0, 0, 1200, 800)) is not None

    plain = ViewerState(world, FakeSource())
    assert plain.pictures(VIEW) is None and not plain.pictures_changed(VIEW)
    plain.on_key(KEY_TOGGLE_CAMERAS)  # harmless without cameras
    assert not plain.pictures_changed(VIEW)


def test_pictures_are_mirrored_unless_turned_off_and_f11_switches(
    world: mujoco.MjModel,
):
    cameras = FakeCameras()
    image = cameras.push("top", 0)
    image.image[:, :160] = (200, 0, 0)  # the left half red, as NAOqi serves it

    def top_picture(state: ViewerState) -> np.ndarray:
        pictures = state.pictures(VIEW)
        assert pictures is not None
        return pictures[0][1]

    def labels(state: ViewerState) -> list[str]:
        left, _ = status_lines(state)
        return left.splitlines()[-2:]

    red, black = [200, 0, 0], [0, 0, 0]
    mirrored = ViewerState(world, FakeSource(), cameras=cameras)
    picture = top_picture(mirrored)
    assert (picture[150, 50].tolist(), picture[150, 350].tolist()) == (black, red)
    assert labels(mirrored) == ["top camera (mirrored)", "bottom camera (mirrored)"]

    plain = ViewerState(world, FakeSource(), cameras=cameras, cameras_mirror=False)
    picture = top_picture(plain)
    assert (picture[150, 50].tolist(), picture[150, 350].tolist()) == (red, black)
    assert labels(plain) == ["top camera", "bottom camera"]

    assert mirrored.pictures_changed(VIEW) and not mirrored.pictures_changed(VIEW)
    mirrored.on_key(KEY_TOGGLE_MIRROR)
    assert mirrored.pictures_changed(VIEW)  # F11 asks for a redraw
    assert top_picture(mirrored)[150, 50].tolist() == red
    assert labels(mirrored) == ["top camera", "bottom camera"]
    assert image.image[0, 0].tolist() == red  # the source's image is never flipped


def test_camera_status_lines(world: mujoco.MjModel):
    cameras = FakeCameras()
    source = FakeSource(info=TargetInfo("tcp://nao.local:9559", "real", "2.1.4.13"))
    state = ViewerState(world, source, cameras=cameras)
    source.push({})
    left, right = status_lines(state)
    assert left.splitlines()[-2:] == [
        "top camera (mirrored)",
        "bottom camera (mirrored)",
    ]
    assert right.splitlines()[-2:] == ["NO IMAGE", "NO IMAGE"]

    cameras.push("top", 1, age=0.08)
    cameras.push("bottom", 2, age=1.2)
    _, right = status_lines(state)
    top, bottom = right.splitlines()[-2:]
    assert top.endswith(" ms") and 80 <= int(top.removesuffix(" ms")) < 1000
    assert bottom == "STALE"

    before_connection = ViewerState(world, FakeSource(), cameras=cameras)
    left, _ = status_lines(before_connection)
    assert left.splitlines() == [
        "connecting",
        "data age",
        "top camera (mirrored)",
        "bottom camera (mirrored)",
    ]

    state.on_key(KEY_TOGGLE_STATUS)
    assert status(state) is None  # F9 hides the camera lines with the rest


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


@dataclass
class FakePeople:
    people: People | None = None

    def latest(self) -> People | None:
        return self.people

    def rate(self) -> float:
        return 10.0

    def close(self) -> None:
        pass


def test_people_markers_are_drawn_after_the_ghost_with_labels_and_colours(
    world: mujoco.MjModel,
):
    people = FakePeople()
    state = ViewerState(world, FakeSource(), people=people)
    scene = mujoco.MjvScene(world, maxgeom=50)
    state.draw_ghost(scene)
    state.draw_people(scene, time.monotonic())
    assert scene.ngeom == 0  # no answer yet
    now = time.monotonic()
    people.people = People(
        (Person(7, (1.0, 0.5, 1.6), True), Person(8, (2.0, -1.0, 1.5), False)), 1, now
    )
    state.draw_people(scene, now)
    assert scene.ngeom == 2
    first, second = scene.geoms[0], scene.geoms[1]
    assert first.type == mujoco.mjtGeom.mjGEOM_SPHERE
    assert tuple(first.pos) == pytest.approx((1.0, 0.5, 1.6))
    assert first.size[0] == pytest.approx(PERSON_RADIUS)
    assert tuple(first.rgba) == pytest.approx(PERSON_VISIBLE_RGBA)
    assert tuple(second.rgba) == pytest.approx(PERSON_HIDDEN_RGBA)
    assert (first.label, second.label) == ("7", "8")
    scene.ngeom = 0
    state.draw_people(scene, now + PEOPLE_STALE_AFTER + 0.1)
    assert scene.ngeom == 0  # stale


def test_no_people_source_draws_nothing_and_adds_no_status_line(world: mujoco.MjModel):
    state = ViewerState(world, FakeSource())
    scene = mujoco.MjvScene(world, maxgeom=10)
    state.draw_people(scene, time.monotonic())
    assert scene.ngeom == 0
    assert "people" not in status_lines(state)[0].splitlines()


def test_the_people_status_line(world: mujoco.MjModel):
    people = FakePeople()
    state = ViewerState(world, FakeSource(), people=people)
    left, right = status_lines(state)
    assert (left.splitlines()[-1], right.splitlines()[-1]) == ("people", "NO DATA")
    people.people = People((Person(1, (1, 0, 1.6), True),), 1, time.monotonic())
    assert status_lines(state)[1].splitlines()[-1] == "1 (1 visible)"
