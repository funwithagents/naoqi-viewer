"""Props: a pool of faces and objects appended to a sim-mode scene, shown, moved and hidden while
the viewer runs (specs/props.md).

The state (`PropState`), the error (`PropError`) and the validation are shared with the caller's
side, so this module imports `mujoco` only inside the functions that build or drive a model:
importing it from `client.py` costs nothing.
"""

import math
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, cast

if TYPE_CHECKING:
    import mujoco

type Kind = Literal["face", "object"]
type Shape = Literal["sphere", "box", "cylinder"]
type Vec3 = tuple[float, float, float]
type Quat = tuple[float, float, float, float]

KINDS: tuple[Kind, ...] = ("face", "object")
SHAPES: tuple[Shape, ...] = ("sphere", "box", "cylinder")
PORTRAITS_DIR = Path(__file__).parent / "portraits"
FACE_IMAGES: tuple[str, ...] = ("armstrong", "chawla", "schmid", "malenchenko")
# width, height, thickness (m): the portrait's face (about half the plane's height) is life-size,
# the box a face detector draws on it about 0.20 m tall, what naoqi-sim's people perception sizes
# a person's distance on (its people-perception.md)
FACE_SIZE: Vec3 = (0.32, 0.40, 0.005)
OBJECT_POOL_SIZE = 8
PARKED_POS: Vec3 = (0.0, 0.0, -1.0)  # below the floor, hidden anyway
DEFAULT_OBJECT_SIZE = 0.05
DEFAULT_OBJECT_COLOR: Vec3 = (1.0, 0.0, 0.0)
IDENTITY: Quat = (1.0, 0.0, 0.0, 0.0)
# A face's box in its body: a face prop's identity rotation shows its picture toward -x,
# upright and unmirrored (the box's thin axis along -x, its height up, its width along -y), so
# a face placed on the +x axis looks back at the origin.
FACE_GEOM_QUAT: Quat = (0.5, 0.5, -0.5, -0.5)
POOL_NAMES: tuple[str, ...] = tuple(
    [f"face_{i + 1}" for i in range(len(FACE_IMAGES))]
    + [f"object_{i + 1}" for i in range(OBJECT_POOL_SIZE)]
)


class PropError(ValueError):
    """A props call refused: an unknown prop, a prop already shown, no free prop, a bad value."""


@dataclass(frozen=True)
class PropState:
    name: str
    kind: Kind
    visible: bool
    pos: Vec3  # m, the scene's world frame: the commanded pose (the end of a move in flight)
    quat: Quat  # w, x, y, z
    moving: bool  # a timed move is in flight
    image: str | None = None  # a face's portrait
    shape: Shape | None = None  # an object's
    size: tuple[float, ...] | None = None  # an object's MuJoCo geom size (m)
    color: Vec3 | None = None  # an object's, RGB in 0..1

    def to_dict(self) -> dict[str, Any]:
        return {
            k: list(v) if isinstance(v, tuple) else v for k, v in asdict(self).items()
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PropState":
        def tup(v: Any) -> Any:
            return tuple(float(x) for x in v) if isinstance(v, list) else v

        return cls(**{k: tup(v) for k, v in data.items()})


def kind_of(name: str) -> Kind:
    return "face" if name.startswith("face_") else "object"


# region Validation (both sides)


def _finite(values: Any, what: str, count: int) -> tuple[float, ...]:
    if (
        isinstance(values, str | bytes)
        or not isinstance(values, Sequence)
        or len(values) != count
        or not all(
            isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v)
            for v in values
        )
    ):
        raise PropError(f"{what} must be {count} finite numbers, got {values!r}")
    return tuple(float(v) for v in values)


def check_pos(pos: Any) -> Vec3:
    return cast(Vec3, _finite(pos, "pos", 3))


def check_quat(quat: Any) -> Quat:
    w, x, y, z = _finite(quat, "quat", 4)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        raise PropError("quat must not be all zeros")
    return (w / norm, x / norm, y / norm, z / norm)


def check_duration(duration: Any) -> float:
    (value,) = _finite([duration], "duration", 1)
    if value < 0:
        raise PropError(f"duration must not be negative, got {duration!r}")
    return value


def check_shape(shape: Any) -> Shape:
    for s in SHAPES:
        if shape == s:
            return s
    raise PropError(f"shape must be one of {', '.join(SHAPES)}, got {shape!r}")


def check_size(shape: Shape, size: Any) -> tuple[float, ...]:
    """MuJoCo's geom size for `shape`: sphere [radius], box [x, y, z] half-sizes, cylinder
    [radius, half_height]; a single number is a sphere's radius or a cube's half-side."""
    count = {"sphere": 1, "box": 3, "cylinder": 2}[shape]
    if isinstance(size, int | float) and not isinstance(size, bool):
        if shape == "cylinder":
            raise PropError("a cylinder's size is [radius, half_height]")
        size = [size] * count
    values = _finite(size, "size", count)
    if any(v <= 0 for v in values):
        raise PropError(f"size must be positive, got {size!r}")
    return values


def check_color(color: Any) -> Vec3:
    values = _finite(color, "color", 3)
    if any(not 0 <= v <= 1 for v in values):
        raise PropError(f"color must be RGB in 0..1, got {color!r}")
    return cast(Vec3, values)


def check_kind(kind: Any) -> Kind:
    for k in KINDS:
        if kind == k:
            return k
    raise PropError(f"kind must be 'face' or 'object', got {kind!r}")


# region The pool, in a spec


def add_pool(spec: "mujoco.MjSpec") -> None:
    """Append the pool to a scene's spec: one hidden, parked mocap body per prop, a face's box
    textured with its portrait, an object's one primitive. A scene already using a pool name
    fails to compile ("repeated name")."""
    import mujoco

    for name, image in zip(POOL_NAMES, FACE_IMAGES, strict=False):
        spec.add_texture(
            name=f"prop_{name}",
            file=str(PORTRAITS_DIR / f"{image}.png"),
            type=mujoco.mjtTexture.mjTEXTURE_2D,
        )
        material = spec.add_material(
            name=f"prop_{name}", emission=1.0, specular=0.0, rgba=[1, 1, 1, 0]
        )
        material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = f"prop_{name}"
        body = spec.worldbody.add_body(name=name, mocap=True, pos=list(PARKED_POS))
        width, height, thickness = FACE_SIZE
        body.add_geom(
            name=name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[width / 2, height / 2, thickness / 2],
            quat=list(FACE_GEOM_QUAT),
            material=f"prop_{name}",
            rgba=[1, 1, 1, 0],
            contype=0,
            conaffinity=0,
        )
    for name in POOL_NAMES[len(FACE_IMAGES) :]:
        body = spec.worldbody.add_body(name=name, mocap=True, pos=list(PARKED_POS))
        body.add_geom(
            name=name,
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[DEFAULT_OBJECT_SIZE, 0, 0],
            rgba=[*DEFAULT_OBJECT_COLOR, 0],
            contype=0,
            conaffinity=0,
        )


# region The driver (render thread)


def _lerp(a: Sequence[float], b: Sequence[float], t: float) -> tuple[float, ...]:
    return tuple(x + (y - x) * t for x, y in zip(a, b, strict=True))


def _nlerp(a: Quat, b: Quat, t: float) -> Quat:
    if sum(x * y for x, y in zip(a, b, strict=True)) < 0:  # the short way round
        b = cast(Quat, tuple(-x for x in b))
    q = _lerp(a, b, t)
    norm = math.sqrt(sum(x * x for x in q))
    return cast(Quat, tuple(x / norm for x in q))


@dataclass
class _Prop:
    name: str
    kind: Kind
    body: int
    mocap: int
    geom: int
    material: int  # a face's, -1 for an object
    image: str | None
    visible: bool = False
    pos: Vec3 = PARKED_POS  # where the current move started (or is)
    quat: Quat = IDENTITY
    target_pos: Vec3 = PARKED_POS
    target_quat: Quat = IDENTITY
    start: float = 0.0
    duration: float = 0.0
    shape: Shape = "sphere"
    size: tuple[float, ...] = (DEFAULT_OBJECT_SIZE,)
    color: Vec3 = DEFAULT_OBJECT_COLOR
    dirty: bool = True

    def at(self, now: float) -> tuple[Vec3, Quat, bool]:
        """The pose at `now`, and whether the move is still in flight."""
        if self.duration <= 0 or now >= self.start + self.duration:
            return self.target_pos, self.target_quat, False
        t = max(0.0, (now - self.start) / self.duration)
        return (
            cast(Vec3, _lerp(self.pos, self.target_pos, t)),
            _nlerp(self.quat, self.target_quat, t),
            True,
        )


_GEOM_TYPES = {"sphere": 2, "cylinder": 5, "box": 6}  # mjtGeom


def _half_extents(shape: Shape, size: tuple[float, ...]) -> Vec3:
    """The half-sizes of a shape's box in its own frame, from MuJoCo's geom size."""
    if shape == "sphere":
        return (size[0], size[0], size[0])
    if shape == "cylinder":
        return (size[0], size[0], size[1])
    return (size[0], size[1], size[2])


class Props:
    """The commanded state of a model's pooled props, written into it by `step` on the render
    thread. The other methods may be called from any thread; they take effect at the next step."""

    def __init__(
        self,
        model: "mujoco.MjModel",
        data: "mujoco.MjData",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        import mujoco

        self._model, self._data, self._clock = model, data, clock
        self._lock = threading.Lock()
        self._props: dict[str, _Prop] = {}
        for index, name in enumerate(POOL_NAMES):
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body < 0:
                raise ValueError(
                    f"the model has no prop {name!r}: load it with props=True"
                )
            kind = kind_of(name)
            self._props[name] = _Prop(
                name=name,
                kind=kind,
                body=body,
                mocap=int(model.body_mocapid[body]),
                geom=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name),
                material=mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_MATERIAL, f"prop_{name}"
                )
                if kind == "face"
                else -1,
                image=FACE_IMAGES[index] if kind == "face" else None,
            )

    def list(self) -> dict[str, PropState]:
        with self._lock:
            return {
                name: self._state(p, self._clock()) for name, p in self._props.items()
            }

    def spawn(
        self,
        kind: Kind,
        pos: Any,
        *,
        name: str | None = None,
        quat: Any = None,
        duration: float = 0.0,
        shape: Shape = "sphere",
        size: Any = DEFAULT_OBJECT_SIZE,
        color: Any = DEFAULT_OBJECT_COLOR,
    ) -> PropState:
        kind = check_kind(kind)
        target = check_pos(pos)
        orientation = None if quat is None else check_quat(quat)
        duration = check_duration(duration)
        dims: tuple[float, ...] = (DEFAULT_OBJECT_SIZE,)
        rgb: Vec3 = DEFAULT_OBJECT_COLOR
        if kind == "object":
            shape = check_shape(shape)
            dims = check_size(shape, size)
            rgb = check_color(color)
        with self._lock:
            if name is not None:
                prop = self._get(name)
                if prop.kind != kind:
                    raise PropError(f"{name!r} is a {prop.kind}, not a {kind}")
                if prop.visible:
                    raise PropError(f"{name!r} is already shown; despawn it first")
            else:
                free = [
                    p for p in self._props.values() if p.kind == kind and not p.visible
                ]
                if not free:
                    pool = sum(1 for p in self._props.values() if p.kind == kind)
                    raise PropError(f"no free {kind}: all {pool} are shown")
                prop = free[0]
            if kind == "object":
                prop.shape, prop.size, prop.color = shape, dims, rgb
            prop.visible = True
            self._move(prop, target, orientation, duration)
            return self._state(prop, self._clock())

    def place(
        self,
        name: str,
        pos: Any = None,
        quat: Any = None,
        *,
        duration: float = 0.0,
    ) -> PropState:
        target = None if pos is None else check_pos(pos)
        orientation = None if quat is None else check_quat(quat)
        duration = check_duration(duration)
        with self._lock:
            prop = self._get(name)
            if target is None:
                target = prop.target_pos
            if orientation is None and prop.kind == "object":
                orientation = prop.target_quat
            self._move(prop, target, orientation, duration)
            return self._state(prop, self._clock())

    def despawn(self, name: str) -> PropState:
        with self._lock:
            prop = self._get(name)
            prop.visible, prop.dirty = False, True
            return self._state(prop, self._clock())

    def clear(self) -> dict[str, PropState]:
        with self._lock:
            for prop in self._props.values():
                prop.visible, prop.dirty = False, True
            return {
                name: self._state(p, self._clock()) for name, p in self._props.items()
            }

    def step(self, now: float | None = None) -> bool:
        """Write the props' poses and visibility into the model at `now`; True when a pose
        changed (and kinematics ran). Writes nothing when idle."""
        import mujoco

        now = self._clock() if now is None else now
        moved = False
        with self._lock:
            for prop in self._props.values():
                pos, quat, moving = prop.at(now)
                if not (prop.dirty or moving):
                    continue
                self._data.mocap_pos[prop.mocap] = pos
                self._data.mocap_quat[prop.mocap] = quat
                alpha = 1.0 if prop.visible else 0.0
                if prop.kind == "face":
                    self._model.mat_rgba[prop.material, 3] = alpha
                    self._model.geom_rgba[prop.geom, 3] = alpha
                else:
                    self._model.geom_type[prop.geom] = _GEOM_TYPES[prop.shape]
                    self._model.geom_size[prop.geom] = [*prop.size, 0, 0][:3]
                    self._model.geom_rgba[prop.geom] = [*prop.color, alpha]
                    # Keep the bounds the renderer and MuJoCo read covering the new shape.
                    half = _half_extents(prop.shape, prop.size)
                    self._model.geom_aabb[prop.geom] = [0, 0, 0, *half]
                    self._model.geom_rbound[prop.geom] = math.sqrt(
                        sum(h * h for h in half)
                    )
                prop.dirty = moving  # the end of a move writes once more
                moved = True
        if moved:
            mujoco.mj_kinematics(self._model, self._data)
            mujoco.mj_camlight(self._model, self._data)
        return moved

    def _get(self, name: str) -> _Prop:
        prop = self._props.get(name)
        if prop is None:
            raise PropError(f"no prop {name!r}; the pool is {', '.join(POOL_NAMES)}")
        return prop

    def _move(self, prop: _Prop, pos: Vec3, quat: Quat | None, duration: float) -> None:
        now = self._clock()
        current_pos, current_quat, _ = prop.at(now)
        if quat is None:  # identity: an object upright, a face looking toward -x
            quat = IDENTITY
        prop.pos, prop.quat = current_pos, current_quat
        prop.target_pos, prop.target_quat = pos, quat
        prop.start, prop.duration, prop.dirty = now, duration, True

    def _state(self, prop: _Prop, now: float) -> PropState:
        moving = prop.at(now)[2]
        is_object = prop.kind == "object"
        return PropState(
            name=prop.name,
            kind=prop.kind,
            visible=prop.visible,
            pos=prop.target_pos,
            quat=prop.target_quat,
            moving=moving,
            image=prop.image,
            shape=prop.shape if is_object else None,
            size=prop.size if is_object else None,
            color=prop.color if is_object else None,
        )
