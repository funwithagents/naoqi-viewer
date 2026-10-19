"""The props pool and its driver, on a real compiled model (specs/props.md)."""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from naoqi_viewer.model import NaoPose, PoseWriter, load_world
from naoqi_viewer.props import (
    FACE_IMAGES,
    OBJECT_POOL_SIZE,
    PARKED_POS,
    POOL_NAMES,
    PropError,
    Props,
    PropState,
    robot_frame,
    robot_to_world,
)


class Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture(scope="module")
def world() -> mujoco.MjModel:
    return load_world("empty", variant="placeholder", props=True)


@pytest.fixture
def clock() -> Clock:
    return Clock()


@pytest.fixture
def scene(
    world: mujoco.MjModel, clock: Clock
) -> tuple[mujoco.MjModel, mujoco.MjData, Props]:
    model = world.__copy__()  # each test changes rgba and geom types
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    props = Props(model, data, clock)
    props.step()
    return model, data, props


def alpha(model: mujoco.MjModel, name: str) -> float:
    return float(model.geom_rgba[model.geom(name).id, 3])


def mocap_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    return data.mocap_pos[model.body(name).mocapid[0]]


@pytest.mark.parametrize("scene_name", ["empty", "table"])
def test_the_pool_is_appended_hidden_and_parked(scene_name: str):
    model = load_world(scene_name, variant="placeholder", props=True)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    assert len(POOL_NAMES) == len(FACE_IMAGES) + OBJECT_POOL_SIZE == 12
    for name in POOL_NAMES:
        body = model.body(name)
        assert body.mocapid[0] >= 0
        assert alpha(model, name) == 0
        assert tuple(data.xpos[body.id]) == pytest.approx(PARKED_POS)
        geom = model.geom(name)
        assert (geom.contype[0], geom.conaffinity[0]) == (0, 0)


def test_without_props_the_scene_is_as_written():
    model = load_world("empty", variant="placeholder")
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "face_1") == -1


def test_a_scene_using_a_reserved_name_fails_to_compile(tmp_path: Path):
    scene = tmp_path / "clash.xml"
    scene.write_text(
        '<mujoco><worldbody><body name="object_1"><geom size="0.1"/></body></worldbody></mujoco>'
    )
    with pytest.raises(ValueError, match="object_1"):
        load_world(str(scene), variant="placeholder", props=True)


def test_spawn_shows_the_lowest_free_prop_at_its_position(scene):
    model, data, props = scene
    state = props.spawn("object", (1.5, 0.2, 0.4))
    assert state.name == "object_1" and state.visible
    props.step()
    assert alpha(model, "object_1") == 1
    assert tuple(mocap_pos(model, data, "object_1")) == pytest.approx((1.5, 0.2, 0.4))
    assert data.xpos[model.body("object_1").id] == pytest.approx((1.5, 0.2, 0.4))
    assert props.spawn("object", (1, 0, 0)).name == "object_2"
    assert props.spawn("face", (1, 0, 0.5)).name == "face_1"


def test_an_objects_shape_size_and_colour_are_set_at_spawn(scene):
    model, _, props = scene
    state = props.spawn(
        "object", (1, 0, 0.2), shape="box", size=[0.05, 0.1, 0.15], color=(0, 0.5, 1)
    )
    props.step()
    geom = model.geom(state.name)
    assert geom.type[0] == mujoco.mjtGeom.mjGEOM_BOX
    assert tuple(geom.size) == pytest.approx((0.05, 0.1, 0.15))
    assert tuple(geom.rgba) == pytest.approx((0, 0.5, 1, 1))
    assert state.shape == "box" and state.size == (0.05, 0.1, 0.15)
    cube = props.spawn("object", (1, 0.5, 0.2), shape="box", size=0.04)
    assert cube.size == (0.04, 0.04, 0.04)


def test_a_named_spawn_picks_that_person(scene):
    _, _, props = scene
    state = props.spawn("face", (1, 0, 0.5), name="face_3")
    assert state.image == "schmid"
    with pytest.raises(PropError, match="already shown"):
        props.spawn("face", (1, 0, 0.5), name="face_3")
    with pytest.raises(PropError, match="no prop"):
        props.spawn("face", (1, 0, 0.5), name="face_9")
    with pytest.raises(PropError, match="is a face"):
        props.spawn("object", (1, 0, 0.5), name="face_2")


def test_no_free_prop_names_the_pool(scene):
    _, _, props = scene
    for _ in FACE_IMAGES:
        props.spawn("face", (1, 0, 0.5))
    with pytest.raises(PropError, match="all 4"):
        props.spawn("face", (1, 0, 0.5))


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"pos": (1, 0)}, "pos"),
        ({"pos": (1, 0, math.nan)}, "pos"),
        ({"quat": (0, 0, 0, 0)}, "zeros"),
        ({"duration": -1}, "duration"),
        ({"shape": "cone"}, "shape"),
        ({"shape": "cylinder", "size": 0.1}, "cylinder"),
        ({"size": -0.1}, "positive"),
        ({"color": (2, 0, 0)}, "color"),
    ],
)
def test_bad_values_are_refused(scene, kwargs: dict, match: str):
    _, _, props = scene
    args = {"pos": (1, 0, 0.5), **kwargs}
    with pytest.raises(PropError, match=match):
        props.spawn("object", args.pop("pos"), **args)


def test_despawn_hides_and_frees_and_clear_hides_all(scene):
    model, _, props = scene
    first = props.spawn("object", (1, 0, 0.5))
    props.spawn("face", (1, 0.3, 0.5))
    props.despawn(first.name)
    props.step()
    assert alpha(model, first.name) == 0
    assert props.spawn("object", (2, 0, 0.5)).name == first.name  # free again
    props.clear()
    props.step()
    assert all(alpha(model, name) == 0 for name in POOL_NAMES)
    assert not any(s.visible for s in props.list().values())


def test_a_timed_move_interpolates_and_retargets_without_a_jump(scene, clock: Clock):
    model, data, props = scene
    props.spawn("object", (1.0, 0.0, 0.5))
    props.step()
    state = props.place("object_1", (2.0, 0.0, 0.5), duration=2.0)
    assert state.moving and state.pos == (2.0, 0.0, 0.5)  # the commanded pose
    clock.now += 1.0
    props.step()
    assert mocap_pos(model, data, "object_1")[0] == pytest.approx(1.5)
    props.place("object_1", (1.5, 1.0, 0.5), duration=1.0)  # from where it is now
    props.step()
    assert tuple(mocap_pos(model, data, "object_1")) == pytest.approx((1.5, 0.0, 0.5))
    clock.now += 0.5
    props.step()
    assert tuple(mocap_pos(model, data, "object_1")) == pytest.approx((1.5, 0.5, 0.5))
    clock.now += 1.0
    props.step()
    assert tuple(mocap_pos(model, data, "object_1")) == pytest.approx((1.5, 1.0, 0.5))
    assert not props.list()["object_1"].moving


def test_a_face_looks_at_the_robot_from_either_side(scene):
    model, data, props = scene
    torso = data.xpos[model.body("torso").id].copy()
    for name, pos in (("face_1", (1.0, 0.6, 0.5)), ("face_2", (-0.5, -1.0, 0.5))):
        props.spawn("face", pos, name=name)
        props.step()
        rotation = data.xmat[model.body(name).id].reshape(3, 3)
        normal = rotation[:, 2]  # the plane's thin axis: where the picture faces
        to_robot = torso[:2] - np.array(pos[:2])
        to_robot /= np.linalg.norm(to_robot)
        assert normal[:2] @ to_robot == pytest.approx(1.0, abs=1e-6)
        assert rotation[:, 1] == pytest.approx((0, 0, 1), abs=1e-6)  # upright


def test_step_writes_nothing_when_idle(scene):
    model, data, props = scene
    props.spawn("object", (1, 0, 0.5))
    assert props.step()
    before = (data.mocap_pos.copy(), model.geom_rgba.copy())
    assert not props.step()
    assert np.array_equal(data.mocap_pos, before[0])
    assert np.array_equal(model.geom_rgba, before[1])


def test_the_state_round_trips_as_json():
    state = PropState(
        "object_2", "object", True, (1.0, 2.0, 3.0), (1.0, 0, 0, 0), False,
        None, "box", (0.1, 0.2, 0.3), (0.0, 1.0, 0.0),
    )  # fmt: skip
    assert PropState.from_dict(state.to_dict()) == state


def test_a_spawned_sphere_is_rendered_and_vanishes_when_hidden(scene):
    model, data, props = scene
    try:
        renderer = mujoco.Renderer(model, height=120, width=160)
    except Exception as exc:  # noqa: BLE001  # no offscreen GL on this machine
        pytest.skip(f"no offscreen OpenGL: {exc}")
    with renderer:

        def red_pixels() -> int:
            renderer.update_scene(data, camera="CameraTop")
            image = renderer.render().astype(int)
            return int(((image[..., 0] > 150) & (image[..., 1] < 80)).sum())

        assert red_pixels() == 0
        props.spawn("object", (1.5, 0.0, 0.45), size=0.1)
        props.step()
        assert red_pixels() > 50
        props.despawn("object_1")
        props.step()
        assert red_pixels() == 0


def test_an_object_stands_upright_whatever_its_place(scene):
    """With no orientation given, an object keeps the world's axes (a face alone turns to the
    robot): a box 1 m tall stands on its foot, its long axis vertical, off to the side too."""
    model, data, props = scene
    pillar = props.spawn("object", (1.0, 0.8, 0.5), shape="box", size=(0.1, 0.1, 0.5))
    props.step()
    assert pillar.quat == (1.0, 0.0, 0.0, 0.0)
    assert data.xmat[model.body(pillar.name).id] == pytest.approx(np.eye(3).flatten())
    geom = model.geom(pillar.name).id
    assert tuple(model.geom_aabb[geom][3:]) == pytest.approx((0.1, 0.1, 0.5))
    assert model.geom_rbound[geom] == pytest.approx(math.sqrt(0.01 + 0.01 + 0.25))


def _yaw_quat(theta: float) -> tuple[float, float, float, float]:
    return (math.cos(theta / 2), 0.0, 0.0, math.sin(theta / 2))


@pytest.mark.parametrize("theta", [0.0, -0.133, 2.0, math.pi - 0.01, -math.pi + 0.01])
def test_the_robot_frame_is_the_middle_of_the_feet_along_their_heading(
    world, theta: float
):
    model = world.__copy__()
    data = mujoco.MjData(model)
    PoseWriter(model).apply(data, NaoPose({}, (0.3, -0.2, 0.3332), _yaw_quat(theta)))
    props = Props(model, data)
    x, y, heading = props.robot()
    # Straight legs: the soles are right under the hips, on either side of the torso's axis.
    assert (x, y) == pytest.approx((0.3, -0.2), abs=0.02)
    assert math.remainder(heading - theta, 2 * math.pi) == pytest.approx(0, abs=1e-6)


def test_robot_to_world_turns_and_moves_by_the_robots_pose():
    pos, quat = robot_to_world((1.0, 2.0, math.pi / 2), (1.5, 0.0, 0.1), None)
    assert pos == pytest.approx((1.0, 3.5, 0.1))  # 1.5 m ahead of a robot facing +y
    assert quat is None
    _, quat = robot_to_world((0.0, 0.0, 0.5), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert quat == pytest.approx(_yaw_quat(0.5))


def test_robot_frame_handles_feet_pointing_either_side_of_pi():
    def foot(yaw: float) -> tuple[list[float], list[list[float]]]:
        c, s = math.cos(yaw), math.sin(yaw)
        return [0.0, 0.0, 0.0], [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    heading = robot_frame([foot(math.pi - 0.1), foot(-math.pi + 0.1)])[2]
    assert math.remainder(heading - math.pi, 2 * math.pi) == pytest.approx(0, abs=1e-9)


def test_spawn_and_place_in_the_robot_frame_land_ahead_of_the_drawn_robot(world):
    model = world.__copy__()
    data = mujoco.MjData(model)
    theta = -0.133  # NAOqi's odometry right after boot
    PoseWriter(model).apply(data, NaoPose({}, (0.0, 0.0, 0.3332), _yaw_quat(theta)))
    props = Props(model, data)
    robot = props.robot()
    ball = props.spawn("object", (1.5, 0.0, 0.1), frame="robot", size=0.1)
    assert ball.pos == pytest.approx(robot_to_world(robot, (1.5, 0.0, 0.1), None)[0])
    assert ball.pos == pytest.approx(
        (1.5 * math.cos(theta), 1.5 * math.sin(theta), 0.1), abs=0.02
    )
    assert ball.quat == pytest.approx(
        (1, 0, 0, 0)
    )  # upright in the world, frame or not
    moved = props.place(ball.name, (1.0, 0.5, 0.1), frame="robot")
    assert moved.pos == pytest.approx(robot_to_world(robot, (1.0, 0.5, 0.1), None)[0])
    with pytest.raises(PropError, match="frame"):
        props.spawn("object", (1, 0, 0), frame="head")  # type: ignore[arg-type]


def _texture(model: mujoco.MjModel, name: str) -> np.ndarray:
    texture = model.texture(f"prop_{name}")
    size = texture.width[0] * texture.height[0] * texture.nchannel[0]
    pixels = model.tex_data[texture.adr[0] : texture.adr[0] + size]
    return pixels.reshape(texture.height[0], texture.width[0], texture.nchannel[0])[
        ..., :3
    ]


def _halves(image: np.ndarray) -> tuple[float, float]:
    """Mean brightness of the left half minus the right, and of the top half minus the
    bottom: their signs flip when a picture is mirrored or upside down."""
    grey = image.astype(float).mean(axis=2)
    height, width = grey.shape
    return (
        grey[:, : width // 2].mean() - grey[:, width // 2 :].mean(),
        grey[: height // 2].mean() - grey[height // 2 :].mean(),
    )


def test_each_portrait_is_rendered_upright_and_unmirrored(scene):
    """Seen by the robot's camera, each face plane shows its portrait as the picture is:
    the render's brightness halves match the texture's, which a mirrored or upside-down
    picture would not (every bundled portrait is lopsided enough to tell)."""
    model, data, props = scene
    try:
        renderer = mujoco.Renderer(model, height=480, width=640)
    except Exception as exc:  # noqa: BLE001  # no offscreen GL on this machine
        pytest.skip(f"no offscreen OpenGL: {exc}")
    with renderer:

        def shot() -> np.ndarray:
            renderer.update_scene(data, camera="CameraTop")
            return renderer.render().astype(int)

        background = shot()
        for index, image in enumerate(FACE_IMAGES):
            name = f"face_{index + 1}"
            state = props.spawn("face", (0.8, 0.0, 0.45), name=name)
            props.step()
            frame = shot()
            props.despawn(name)
            props.step()
            assert state.image == image
            rows, columns = np.nonzero(np.abs(frame - background).sum(axis=2) > 30)
            assert rows.size > 10_000, f"{name} is not in view"
            crop = frame[rows.min() : rows.max() + 1, columns.min() : columns.max() + 1]
            expected = _halves(_texture(model, name))
            assert min(map(abs, expected)) > 5  # lopsided enough to tell a flip
            assert _halves(crop) == pytest.approx(expected, abs=3), name
        assert np.array_equal(shot(), background)  # all gone
