"""Aldebaran's meshes, from a real `naoqi-viewer fetch-meshes` install, loaded and rendered.

Needs no NAOqi: it skips when the meshes aren't installed (fails with
NAOQI_VIEWER_REQUIRE_MESHES set, as CI's meshes job does).
"""

import os

import mujoco
import numpy as np
import pytest
from support import require_meshes

from naoqi_viewer import meshes
from naoqi_viewer.model import NaoPose, PoseWriter, load_world

STANDING = NaoPose(
    angles={"LShoulderPitch": 1.4, "RShoulderPitch": 1.4, "LHand": 0.5, "RHand": 0.5},
    torso_pos=(0.0, 0.0, 0.3332),
    torso_quat=(1.0, 0.0, 0.0, 0.0),
)


def render(model: mujoco.MjModel) -> np.ndarray:
    """The robot seen from the front and a little above, 320×240."""
    data = mujoco.MjData(model)
    PoseWriter(model).apply(data, STANDING)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (0.0, 0.0, 0.3)
    camera.distance, camera.azimuth, camera.elevation = 1.0, 180.0, -15.0
    try:
        renderer = mujoco.Renderer(model, 240, 320)
    except Exception as exc:  # noqa: BLE001
        if os.environ.get("NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL"):
            pytest.fail(f"offscreen OpenGL is required on this run: {exc}")
        pytest.skip(f"no OpenGL for offscreen rendering here: {exc}")
    try:
        renderer.update_scene(data, camera)
        return renderer.render().copy()
    finally:
        renderer.close()


def test_every_table_body_carries_an_aldebaran_mesh():
    require_meshes()
    model = load_world(variant="aldebaran")
    bodies = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g])
        for g in range(model.ngeom)
        if model.geom_group[g] == 1 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
    }
    assert bodies == {v.body for v in meshes.visual_table()}


def test_the_meshes_render_and_differ_from_the_placeholder():
    require_meshes()
    aldebaran = render(load_world(variant="aldebaran"))
    placeholder = render(load_world(variant="placeholder"))
    assert aldebaran.std() > 10  # not blank
    changed = np.abs(aldebaran.astype(int) - placeholder.astype(int)).sum(axis=2) > 30
    assert changed.mean() > 0.02
