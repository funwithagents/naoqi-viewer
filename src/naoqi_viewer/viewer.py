"""The window: poses the model from a pose source and draws it, with no physics; or the same
posing with no window, for a headless viewer."""

import threading
import time
from collections.abc import Callable
from typing import Any

import mujoco
import numpy as np

from naoqi_viewer.cameras import (
    CAMERA_NAMES,
    IMAGE_STALE_AFTER,
    ImageSource,
    Rect,
    framed,
    picture_rects,
    placeholder,
    resample_nearest,
)
from naoqi_viewer.model import PoseWriter
from naoqi_viewer.people import (
    PERSON_RADIUS,
    PeopleReader,
    people_status,
    person_markers,
)
from naoqi_viewer.source import PoseSource, Sample

STALE_AFTER = 0.5  # s of data age before the overlay reads STALE
FRAME_PERIOD = 1.0 / 60
ATTRIBUTION = "NAO meshes © Aldebaran, CC BY-NC-ND 4.0"
GHOST_RGBA = (0.35, 0.65, 1.0, 0.3)
GHOST_MIN_OFFSET = 0.001  # m: closer bodies get no ghost
GHOST_MIN_TURN = 0.025  # Frobenius distance between rotations, about 0.5°
# GLFW_KEY_F9 to F11: letters and digits are all MuJoCo shortcuts already.
KEY_TOGGLE_STATUS = 298
KEY_TOGGLE_CAMERAS = 299
KEY_TOGGLE_MIRROR = 300

_VISUAL_GROUP = 1

# Font, grid position, left column, right column: the tuples Handle.set_texts takes.
Text = tuple[int | None, int | None, str | None, str | None]
# Called between frames with the model, its data and the sample it is posed with.
FrameHook = Callable[[mujoco.MjModel, mujoco.MjData, Sample | None], None]
# A camera picture: where it goes in the view, and the RGB image of exactly that size.
Picture = tuple[Rect, np.ndarray]


def _robot_visuals_option() -> mujoco.MjvOption:
    option = mujoco.MjvOption()
    option.geomgroup[:] = 0
    option.geomgroup[_VISUAL_GROUP] = 1
    return option


class ViewerState:
    """Everything the window shows, kept apart from the window so it works offscreen."""

    def __init__(
        self,
        model: mujoco.MjModel,
        source: PoseSource,
        *,
        ghost: bool = False,
        attribution: str | None = None,
        cameras: ImageSource | None = None,
        cameras_mirror: bool = True,
        people: PeopleReader | None = None,
    ) -> None:
        self.model = model
        self._people = people
        self.data = mujoco.MjData(model)
        self.show_status = True
        self.show_cameras = True
        self.mirror_cameras = (
            cameras_mirror  # left to right, as the view faces the robot
        )
        self._cameras = cameras
        self._drawn: tuple[Rect, tuple[int, ...], bool, bool] | None = (
            None  # last pictures_changed key
        )
        self.sample: Sample | None = None  # the sample the model is posed with
        self._source = source
        self._writer = PoseWriter(model)
        self._ghost = mujoco.MjData(model) if ghost else None
        self._has_ghost_pose = False
        self._attribution = attribution
        self._seq = 0
        self._ghost_option = _robot_visuals_option()
        self._perturb = mujoco.MjvPerturb()

    def update(self) -> Sample | None:
        """Pose the model from the source's latest sample if it is new; return that sample."""
        sample = self._source.latest()
        if sample is None or sample.seq == self._seq:
            return None
        self._seq = sample.seq
        self.sample = sample
        self._writer.apply(self.data, sample.pose)
        if self._ghost is not None and sample.commanded is not None:
            self._writer.apply(self._ghost, sample.commanded)
            self._has_ghost_pose = True
        return sample

    def draw_ghost(self, scene: mujoco.MjvScene) -> None:
        """Replace `scene`'s geoms with the commanded pose, translucent (nothing without one)."""
        scene.ngeom = 0
        if self._ghost is None or not self._has_ghost_pose:
            return
        mujoco.mjv_addGeoms(
            self.model,
            self._ghost,
            self._ghost_option,
            self._perturb,
            mujoco.mjtCatBit.mjCAT_DYNAMIC,
            scene,
        )
        # Where the commanded pose matches the measured one, a ghost would only tint the robot:
        # draw it only for the bodies that are off.
        apart = self._bodies_apart()
        for i in range(scene.ngeom):
            geom = scene.geoms[i]
            geom.rgba[:] = (
                GHOST_RGBA
                if self.model.geom_bodyid[geom.objid] in apart
                else (0, 0, 0, 0)
            )
            geom.matid = -1

    def draw_people(self, scene: mujoco.MjvScene, now: float) -> None:
        """Append one labelled sphere per person NAOqi lists to `scene`, after what is there (the
        ghost); nothing without a people source, before its first answer or when stale."""
        if self._people is None:
            return
        size = np.array([PERSON_RADIUS, 0.0, 0.0])
        identity = np.eye(3).flatten()
        for marker in person_markers(self._people.latest(), now):
            if scene.ngeom >= scene.maxgeom:
                break
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                size,
                np.array(marker.pos),
                identity,
                np.array(marker.rgba, dtype=np.float32),
            )
            geom.label = marker.label
            scene.ngeom += 1

    def _bodies_apart(self) -> set[int]:
        assert self._ghost is not None
        moved = (
            np.linalg.norm(self._ghost.xpos - self.data.xpos, axis=1) > GHOST_MIN_OFFSET
        )
        # Rotation matrices differing by angle θ have a Frobenius distance of 2·√2·sin(θ/2).
        turned = (
            np.linalg.norm(self._ghost.xmat - self.data.xmat, axis=1) > GHOST_MIN_TURN
        )
        return {int(b) for b in np.flatnonzero(moved | turned)}

    def texts(self, now: float) -> list[Text]:
        """The overlays to show at time `now` (time.monotonic())."""
        texts: list[Text] = []
        if self.show_status:
            labels, values = self._status(now)
            texts.append(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    "\n".join(labels),
                    "\n".join(values),
                )
            )
        if self._attribution is not None:
            texts.append(
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                    self._attribution,
                    "",
                )
            )
        return texts

    def pictures(self, viewport: Rect) -> list[Picture] | None:
        """The camera pictures to draw in `viewport` (left, bottom, width, height): the top
        camera's and the bottom camera's, each flipped left to right when mirrored, resampled to its
        rectangle and framed, a dark placeholder before a camera's first image. None when there are no cameras, they are
        hidden (F10), or the view is too small for them."""
        if self._cameras is None or not self.show_cameras:
            return None
        rects = picture_rects(viewport)
        if rects is None:
            return None
        pictures: list[Picture] = []
        for camera, rect in zip(CAMERA_NAMES, rects, strict=True):
            size = (rect[2], rect[3])
            image = self._cameras.latest(camera)
            if image is None:
                picture = placeholder(size)
            else:
                pixels = image.image[:, ::-1] if self.mirror_cameras else image.image
                picture = framed(resample_nearest(pixels, size))
            pictures.append((rect, picture))
        return pictures

    def pictures_changed(self, viewport: Rect) -> bool:
        """Whether `pictures(viewport)` differs from what was drawn when this last returned True:
        a new image for either camera, a resized view, or the F10 or F11 toggle. Always False without
        cameras."""
        if self._cameras is None:
            return False
        seqs = tuple(
            image.seq if (image := self._cameras.latest(camera)) is not None else 0
            for camera in CAMERA_NAMES
        )
        key = (viewport, seqs, self.show_cameras, self.mirror_cameras)
        if key == self._drawn:
            return False
        self._drawn = key
        return True

    def on_key(self, key: int) -> None:
        if key == KEY_TOGGLE_STATUS:
            self.show_status = not self.show_status
        elif key == KEY_TOGGLE_CAMERAS:
            self.show_cameras = not self.show_cameras
        elif key == KEY_TOGGLE_MIRROR:
            self.mirror_cameras = not self.mirror_cameras

    def _status(self, now: float) -> tuple[list[str], list[str]]:
        info = self._source.info
        sample = self._source.latest()
        age = _age_text(
            None if sample is None else sample.received_at, now, STALE_AFTER, "NO DATA"
        )
        if info is None:
            labels, values = ["connecting", "data age"], ["", age]
        else:
            labels = [info.target, "NAOqi", "rate", "data age"]
            values = [
                info.url,
                info.naoqi_version or "unknown",
                f"{self._source.rate():.0f} Hz",
                age,
            ]
        if self._cameras is not None:
            for camera in CAMERA_NAMES:
                image = self._cameras.latest(camera)
                labels.append(
                    f"{camera} camera (mirrored)"
                    if self.mirror_cameras
                    else f"{camera} camera"
                )
                values.append(
                    _age_text(
                        None if image is None else image.received_at,
                        now,
                        IMAGE_STALE_AFTER,
                        "NO IMAGE",
                    )
                )
        if self._people is not None:
            labels.append("people")
            values.append(people_status(self._people.latest(), now))
        return labels, values


def _age_text(
    received_at: float | None, now: float, stale_after: float, missing: str
) -> str:
    if received_at is None:
        return missing
    if now - received_at > stale_after:
        return "STALE"
    return f"{(now - received_at) * 1000:.0f} ms"


def run(
    model: mujoco.MjModel,
    source: PoseSource,
    *,
    ghost: bool = False,
    attribution: str | None = None,
    cameras: ImageSource | None = None,
    cameras_mirror: bool = True,
    people: PeopleReader | None = None,
    on_frame: FrameHook | None = None,
    stop: threading.Event | None = None,
) -> None:
    """Show the robot in a MuJoCo window until it is closed or `stop` is set. On macOS, needs mjpython.

    `on_frame(model, data, sample)` runs between frames, on this thread, with the sample the model
    is posed with. With `cameras`, the head cameras' pictures are drawn in the view's top-right
    corner, mirrored left to right with `cameras_mirror` (F11 switches it).
    """
    import mujoco.viewer  # the window stack (GLFW), which a headless viewer never loads

    stop = stop or threading.Event()
    state = ViewerState(
        model,
        source,
        ghost=ghost,
        attribution=attribution,
        cameras=cameras,
        cameras_mirror=cameras_mirror,
        people=people,
    )
    with mujoco.viewer.launch_passive(
        model,
        state.data,
        key_callback=state.on_key,
        show_left_ui=False,
        show_right_ui=False,
    ) as handle:
        with handle.lock():
            handle.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            handle.cam.trackbodyid = model.body("torso").id
            handle.cam.distance = 1.3
            handle.cam.azimuth = 150
            handle.cam.elevation = -15
        ticks = _Ticks(stop)
        while handle.is_running() and not stop.is_set():
            with handle.lock():
                state.update()
                state.draw_ghost(handle.user_scn)
                state.draw_people(handle.user_scn, time.monotonic())
            handle.set_texts(state.texts(time.monotonic()))
            if cameras is not None:
                _draw_pictures(handle, state)
            if on_frame is not None:
                on_frame(model, state.data, state.sample)
            handle.sync()
            ticks.wait()


def _draw_pictures(handle: Any, state: ViewerState) -> None:
    """Set the camera pictures on the passive viewer `handle` when they changed. Called outside
    `handle.lock()`: `set_images` waits for the render thread, which needs that lock."""
    viewport = handle.viewport
    if viewport is None:
        return
    rect: Rect = (viewport.left, viewport.bottom, viewport.width, viewport.height)
    if not state.pictures_changed(rect):
        return
    pictures = state.pictures(rect)
    if pictures is None:
        handle.clear_images()
    else:
        handle.set_images(
            [(mujoco.MjrRect(*where), image) for where, image in pictures]
        )


def run_headless(
    model: mujoco.MjModel,
    source: PoseSource,
    *,
    on_frame: FrameHook | None = None,
    stop: threading.Event | None = None,
) -> None:
    """Pose the robot at the frame rate with no window, until `stop` is set.

    The same posing as `run`, without overlay, ghost or keys; `on_frame` is where a headless viewer
    renders its camera frames.
    """
    stop = stop or threading.Event()
    state = ViewerState(model, source)
    ticks = _Ticks(stop)
    while not stop.is_set():
        state.update()
        if on_frame is not None:
            on_frame(model, state.data, state.sample)
        ticks.wait()


class _Ticks:
    """Sleeps to the next 1/60 s tick, on a fixed schedule; `stop` cuts the sleep short."""

    def __init__(self, stop: threading.Event) -> None:
        self._stop = stop
        self._due = time.monotonic()

    def wait(self) -> None:
        self._due += FRAME_PERIOD
        remaining = self._due - time.monotonic()
        if remaining > 0:
            self._stop.wait(remaining)
        else:
            self._due = time.monotonic()
