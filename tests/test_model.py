import io
import logging
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
from mesh_archive import install_fake_release

from naoqi_viewer import meshes
from naoqi_viewer import model as nao_model
from naoqi_viewer.model import (
    JOINT_NAMES,
    NaoPose,
    PoseWriter,
    load_world,
    resolve_variant,
)

_REPO_ROOT = Path(__file__).resolve().parent.parent
_URDF = _REPO_ROOT / "third_party" / "nao_description" / "nao.urdf"
_NAO_XML = Path(nao_model.__file__).parent / "models" / "nao.xml"

# nao.xml site or camera -> the URDF link whose frame it reproduces.
_SITE_FRAMES = {
    "Head": "Head",
    "LArm": "l_gripper",
    "RArm": "r_gripper",
    "LLeg": "l_sole",
    "RLeg": "r_sole",
    "imu": "ImuTorsoAccelerometer_frame",
    **{
        f"{s}Fsr{c}": f"{s}Fsr{c}_frame" for s in "LR" for c in ("FL", "FR", "RL", "RR")
    },
}
_CAMERA_FRAMES = {"CameraTop": "CameraTop_frame", "CameraBottom": "CameraBottom_frame"}
# Where nao.xml deliberately departs from the URDF (specs/model.md, Effector sites): site ->
# offset from the URDF frame, in that frame. The URDF's r_gripper z (-0.01213) breaks the arms'
# symmetry; NAOqi's RArm sits at l_gripper's -0.01231, as check-model measured.
_URDF_CORRECTIONS = {"RArm": np.array([0.0, 0.0, -0.00018])}


@pytest.fixture(scope="module")
def urdf_model() -> mujoco.MjModel:
    """The vendored URDF as MuJoCo loads it, frames kept, meshes stripped: the kinematic reference."""
    root = ET.parse(_URDF).getroot()
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    mujoco_ext = ET.SubElement(root, "mujoco")
    ET.SubElement(mujoco_ext, "compiler", fusestatic="false", discardvisual="false")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


@pytest.fixture(scope="module")
def world() -> mujoco.MjModel:
    return load_world(variant="placeholder")


def _hinge_names(model: mujoco.MjModel) -> list[str]:
    return [
        model.joint(j).name
        for j in range(model.njnt)
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
    ]


def _standing(angles: dict[str, float]) -> NaoPose:
    """A pose with the torso upright at the origin, 30 cm up."""
    return NaoPose(angles, (0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))


def _relative_to(
    data: mujoco.MjData, origin_body: int, pos: np.ndarray, mat: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    rot = data.xmat[origin_body].reshape(3, 3)
    return rot.T @ (pos - data.xpos[origin_body]), rot.T @ mat.reshape(3, 3)


def test_body_joints_match_the_urdf_names_axes_and_ranges(
    world: mujoco.MjModel, urdf_model: mujoco.MjModel
):
    assert len(JOINT_NAMES) == 26
    assert set(_hinge_names(world)) == set(_hinge_names(urdf_model))
    for name in JOINT_NAMES:
        ours, theirs = world.joint(name), urdf_model.joint(name)
        np.testing.assert_allclose(ours.range, theirs.range, atol=1e-5, err_msg=name)


def test_forward_kinematics_match_the_urdf(
    world: mujoco.MjModel, urdf_model: mujoco.MjModel
):
    ours, theirs = mujoco.MjData(world), mujoco.MjData(urdf_model)
    rng = np.random.default_rng(7)
    torso_ours, torso_theirs = world.body("torso").id, urdf_model.body("torso").id
    for _ in range(20):
        # Any configuration, fingers included: this checks frames, not couplings.
        for name in _hinge_names(world):
            lo, hi = (
                world.joint(name).range
                if world.jnt_limited[world.joint(name).id]
                else (-1.0, 1.0)
            )
            value = rng.uniform(lo, hi)
            ours.qpos[world.jnt_qposadr[world.joint(name).id]] = value
            theirs.qpos[urdf_model.jnt_qposadr[urdf_model.joint(name).id]] = value
        # LArm/RArm sit at the gripper offset on the wrist: compare them with LHand/RHand at 0.
        for hand in ("LHand", "RHand"):
            ours.qpos[world.jnt_qposadr[world.joint(hand).id]] = 0.0
            theirs.qpos[urdf_model.jnt_qposadr[urdf_model.joint(hand).id]] = 0.0
        mujoco.mj_kinematics(world, ours)
        mujoco.mj_camlight(world, ours)
        mujoco.mj_kinematics(urdf_model, theirs)

        for b in range(world.nbody):
            name = world.body(b).name
            if name in ("world", "torso"):
                continue
            pos_o, rot_o = _relative_to(ours, torso_ours, ours.xpos[b], ours.xmat[b])
            pos_t, rot_t = _relative_to(
                theirs,
                torso_theirs,
                theirs.xpos[urdf_model.body(name).id],
                theirs.xmat[urdf_model.body(name).id],
            )
            np.testing.assert_allclose(pos_o, pos_t, atol=1e-5, err_msg=name)
            np.testing.assert_allclose(rot_o, rot_t, atol=1e-5, err_msg=name)

        for site, frame in _SITE_FRAMES.items():
            frame_id = urdf_model.body(frame).id
            pos_o, _ = _relative_to(
                ours, torso_ours, ours.site(site).xpos, ours.site(site).xmat
            )
            frame_pos = theirs.xpos[frame_id] + theirs.xmat[frame_id].reshape(
                3, 3
            ) @ _URDF_CORRECTIONS.get(site, np.zeros(3))
            pos_t, _ = _relative_to(
                theirs, torso_theirs, frame_pos, theirs.xmat[frame_id]
            )
            np.testing.assert_allclose(pos_o, pos_t, atol=1e-5, err_msg=site)

        for camera, frame in _CAMERA_FRAMES.items():
            frame_id = urdf_model.body(frame).id
            pos_o, rot_o = _relative_to(
                ours, torso_ours, ours.cam(camera).xpos, ours.cam(camera).xmat
            )
            pos_t, rot_t = _relative_to(
                theirs, torso_theirs, theirs.xpos[frame_id], theirs.xmat[frame_id]
            )
            np.testing.assert_allclose(pos_o, pos_t, atol=1e-5, err_msg=camera)
            # MuJoCo cameras look along -Z with +Y up; the URDF camera frame looks along +X with +Z up.
            np.testing.assert_allclose(
                -rot_o[:, 2], rot_t[:, 0], atol=1e-5, err_msg=camera
            )
            np.testing.assert_allclose(
                rot_o[:, 1], rot_t[:, 2], atol=1e-5, err_msg=camera
            )


def test_opening_the_hand_does_not_move_the_arm_effector(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    writer = PoseWriter(world)
    writer.apply(data, _standing({"LHand": 0.0, "LWristYaw": 0.3}))
    before = (data.site("LArm").xpos.copy(), data.site("LArm").xmat.copy())
    writer.apply(data, _standing({"LHand": 1.0, "LWristYaw": 0.3}))
    np.testing.assert_allclose(data.site("LArm").xpos, before[0], atol=1e-12)
    np.testing.assert_allclose(data.site("LArm").xmat, before[1], atol=1e-12)


def test_pose_writer_writes_the_torso_pose_and_named_angles(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    quat = (math.cos(0.25), 0.0, 0.0, math.sin(0.25))  # 0.5 rad about z
    PoseWriter(world).apply(
        data, NaoPose({"HeadYaw": 0.4, "LKneePitch": 1.1}, (1.0, -2.0, 0.31), quat)
    )

    torso = world.body("torso").id
    np.testing.assert_allclose(data.xpos[torso], (1.0, -2.0, 0.31))
    np.testing.assert_allclose(data.xquat[torso], quat, atol=1e-12)
    assert data.qpos[world.jnt_qposadr[world.joint("HeadYaw").id]] == pytest.approx(0.4)
    assert data.qpos[world.jnt_qposadr[world.joint("LKneePitch").id]] == pytest.approx(
        1.1
    )
    # The head turned 0.5 + 0.4 rad about z in the world.
    head_x_axis = data.xmat[world.body("Head").id].reshape(3, 3)[:, 0]
    np.testing.assert_allclose(
        head_x_axis, (math.cos(0.9), math.sin(0.9), 0.0), atol=1e-9
    )


def test_pose_writer_poses_the_head_cameras(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    PoseWriter(world).apply(data, _standing({}))
    top = data.cam("CameraTop")
    # Above the torso, at the front of the head, looking forward (+X) with a level horizon.
    assert top.xpos[2] > 0.3 + 0.15
    np.testing.assert_allclose(
        -top.xmat.reshape(3, 3)[:, 2], (1.0, 0.0, 0.0), atol=0.03
    )
    assert top.xmat.reshape(3, 3)[1, 0] == pytest.approx(
        -1.0, abs=1e-3
    )  # image right = robot's right (-Y)


def test_fingers_follow_their_hand(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    writer = PoseWriter(world)
    for opening in (0.0, 0.6, 1.0):
        writer.apply(data, _standing({"LHand": opening, "RHand": 1.0 - opening}))
        for side, value in (("L", opening), ("R", 1.0 - opening)):
            for finger in (f"{side}Finger11", f"{side}Finger23", f"{side}Thumb2"):
                assert data.qpos[
                    world.jnt_qposadr[world.joint(finger).id]
                ] == pytest.approx(0.999899 * value)


def test_pose_writer_ignores_unknown_joints_once_and_keeps_missing_ones(
    world: mujoco.MjModel, caplog: pytest.LogCaptureFixture
):
    data = mujoco.MjData(world)
    writer = PoseWriter(world)
    writer.apply(data, _standing({"HeadPitch": 0.2, "LFoo": 1.0}))
    with caplog.at_level(logging.INFO, logger="naoqi_viewer.model"):
        writer.apply(data, _standing({"HeadYaw": -0.5, "LFoo": 2.0}))
        writer.apply(data, _standing({"LFoo": 3.0}))
    assert data.qpos[world.jnt_qposadr[world.joint("HeadPitch").id]] == pytest.approx(
        0.2
    )
    assert data.qpos[world.jnt_qposadr[world.joint("HeadYaw").id]] == pytest.approx(
        -0.5
    )
    # Already logged by the first apply, so not again.
    assert not [r for r in caplog.records if "LFoo" in r.getMessage()]

    fresh = PoseWriter(world)
    with caplog.at_level(logging.INFO, logger="naoqi_viewer.model"):
        fresh.apply(data, _standing({"LFoo": 1.0}))
        fresh.apply(data, _standing({"LFoo": 1.0}))
    assert len([r for r in caplog.records if "LFoo" in r.getMessage()]) == 1


def test_pose_writer_writes_out_of_range_values_as_given(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    hi = world.joint("HeadPitch").range[1]
    PoseWriter(world).apply(data, _standing({"HeadPitch": hi + 0.3}))
    assert data.qpos[world.jnt_qposadr[world.joint("HeadPitch").id]] == pytest.approx(
        hi + 0.3
    )


def test_nao_pose_from_naoqi_converts_the_torso_transform():
    angle = 0.7  # about y
    c, s = math.cos(angle), math.sin(angle)
    rows = [
        [c, 0.0, s, 0.1],
        [0.0, 1.0, 0.0, -0.2],
        [-s, 0.0, c, 0.33],
        [0.0, 0.0, 0.0, 1.0],
    ]
    transform = [v for row in rows for v in row]
    pose = NaoPose.from_naoqi(["HeadYaw", "HeadPitch"], [0.1, -0.2], transform)
    assert pose.angles == {"HeadYaw": 0.1, "HeadPitch": -0.2}
    assert pose.torso_pos == pytest.approx((0.1, -0.2, 0.33))
    assert pose.torso_quat == pytest.approx(
        (math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)
    )


def test_nao_pose_from_naoqi_rejects_mismatched_names_and_angles():
    with pytest.raises(ValueError):
        NaoPose.from_naoqi(
            ["HeadYaw", "HeadPitch"],
            [0.1],
            [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        )


def test_default_world_is_the_empty_scene_with_the_robot(world: mujoco.MjModel):
    assert world.geom("floor").type == mujoco.mjtGeom.mjGEOM_PLANE
    assert [n for n in _hinge_names(world) if n in JOINT_NAMES] == list(JOINT_NAMES)
    assert world.nu == 26
    assert {world.sensor(i).name for i in range(world.nsensor)} >= {
        "gyro",
        "accelerometer",
        "LFsrFL",
        "RFsrRR",
        "HeadYaw",
    }


def test_load_world_accepts_a_user_scene_file(tmp_path: Path):
    scene = tmp_path / "my_scene.xml"
    scene.write_text(
        '<mujoco><worldbody><geom name="table" type="box" pos="0.4 0 0.2" size="0.2 0.3 0.01"/></worldbody></mujoco>'
    )
    model = load_world(scene, variant="placeholder")
    assert model.geom("table").size[0] == pytest.approx(0.2)
    assert model.joint("HeadYaw").qposadr >= 0
    assert load_world(str(scene), variant="placeholder").ngeom == model.ngeom


def test_load_world_names_the_model_for_the_window_title():
    model = load_world(
        variant="placeholder", name="naoqi-viewer · mirror · tcp://127.0.0.1:9559"
    )
    assert (
        model.names.split(b"\0")[0].decode()
        == "naoqi-viewer · mirror · tcp://127.0.0.1:9559"
    )


def test_load_world_rejects_an_unknown_scene_name():
    with pytest.raises(ValueError, match="empty"):
        load_world("no-such-scene")


def test_variants_without_installed_meshes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    install_fake_release(tmp_path, monkeypatch)  # pinned, but never fetched
    assert resolve_variant() == "placeholder"
    assert resolve_variant("placeholder") == "placeholder"
    with pytest.raises(FileNotFoundError, match="naoqi-viewer fetch-meshes"):
        resolve_variant("aldebaran")
    with pytest.raises(FileNotFoundError, match="naoqi-viewer fetch-meshes"):
        load_world(variant="aldebaran")


class _Terminal(io.StringIO):
    def isatty(self) -> bool:
        return True


def test_variants_with_installed_meshes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    install_fake_release(tmp_path, monkeypatch)
    meshes.fetch(stdin=_Terminal("yes\n"), stdout=io.StringIO())
    assert resolve_variant() == "aldebaran"
    assert resolve_variant("aldebaran") == "aldebaran"
    assert resolve_variant("placeholder") == "placeholder"

    def visual_types(model: mujoco.MjModel) -> set[int]:
        return {
            int(model.geom_type[g])
            for g in range(model.ngeom)
            if model.geom_group[g] == 1
        }

    assert visual_types(load_world("table", variant="auto")) == {
        mujoco.mjtGeom.mjGEOM_MESH
    }
    assert mujoco.mjtGeom.mjGEOM_MESH not in visual_types(
        load_world(variant="placeholder")
    )


def test_visual_and_collision_geoms_are_separated():
    spec = mujoco.MjSpec.from_file(str(_NAO_XML))
    model = spec.compile()
    visual = [g for g in spec.geoms if g.classname.name == "nao_visual"]
    assert visual
    for geom in spec.geoms:
        compiled = model.geom(geom.name) if geom.name else None
        assert geom.classname.name in ("nao_visual", "nao_collision")
        if compiled is not None and geom.classname.name == "nao_visual":
            assert (compiled.group, compiled.contype, compiled.conaffinity) == (1, 0, 0)
    for g in range(model.ngeom):
        assert model.geom_group[g] in (1, 3)
        if model.geom_group[g] == 1:
            assert model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0
        else:
            assert model.geom_contype[g] != 0


def test_head_cameras_do_not_see_the_robots_own_head(world: mujoco.MjModel):
    data = mujoco.MjData(world)
    PoseWriter(world).apply(data, _standing({}))
    head_bodies = {world.body("Head").id, world.body("Neck").id}
    visible_groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    geom_id = np.zeros(1, dtype=np.int32)
    for camera in ("CameraTop", "CameraBottom"):
        cam = data.cam(camera)
        rot = cam.xmat.reshape(3, 3)
        half_v = math.radians(world.cam(camera).fovy[0]) / 2
        half_h = math.atan(math.tan(half_v) * 4 / 3)  # 4:3 images
        for u in np.linspace(-1, 1, 9):
            for v in np.linspace(-1, 1, 9):
                direction = rot @ np.array(
                    [math.tan(half_h) * u, math.tan(half_v) * v, -1.0]
                )
                mujoco.mj_ray(
                    world, data, cam.xpos, direction, visible_groups, 1, -1, geom_id
                )
                if geom_id[0] >= 0:
                    assert world.geom_bodyid[geom_id[0]] not in head_bodies, (
                        camera,
                        u,
                        v,
                    )
