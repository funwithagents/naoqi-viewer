"""A mock NAOqi: Python qi services served from a standalone qi.Session on loopback.

Code under test connects to `MockNaoqi.url` exactly as it would to a robot, so the qi path is
exercised for real. It implements only what naoqi-viewer calls (specs/testing.md, "Mock NAOqi").
"""

import threading
import time
from collections.abc import Mapping, Sequence
from typing import Any, Literal

import mujoco
import numpy as np
import qi

from naoqi_viewer.model import JOINT_NAMES, NaoPose, PoseWriter, load_world

Target = Literal["virtual", "naoqi-sim", "real"]

FRAME_TORSO = 0
FRAME_WORLD = 1
EFFECTORS = ("Head", "LArm", "RArm", "LLeg", "RLeg")

# Torso upright, standing on the floor (the model's standing height).
STANDING_TORSO: tuple[float, ...] = (
    1,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    1,
    0.3332,
    0,
    0,
    0,
    1,
)


def _transform(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = position
    return matrix


class _Motion:
    def __init__(self, mock: "MockNaoqi") -> None:
        self._mock = mock

    def getBodyNames(self, name: str) -> list[str]:
        return self._mock._call("ALMotion", "getBodyNames", name)

    def getAngles(self, names: Any, useSensors: bool) -> list[float]:
        return self._mock._call("ALMotion", "getAngles", names, useSensors)

    def getTransform(self, name: str, frame: int, useSensors: bool) -> list[float]:
        return self._mock._call("ALMotion", "getTransform", name, frame, useSensors)

    def setStiffnesses(self, names: Any, stiffnesses: Any) -> None:
        self._mock._call("ALMotion", "setStiffnesses", names, stiffnesses)

    def angleInterpolation(
        self, names: Any, angles: Any, times: Any, isAbsolute: bool
    ) -> None:
        self._mock._call(
            "ALMotion", "angleInterpolation", names, angles, times, isAbsolute
        )


class _Memory:
    def __init__(self, mock: "MockNaoqi") -> None:
        self._mock = mock

    def getData(self, key: str) -> Any:
        return self._mock._call("ALMemory", "getData", key)

    def insertData(self, key: str, value: Any) -> None:
        self._mock._call("ALMemory", "insertData", key, value)

    def getListData(self, keys: list[str]) -> list[Any]:
        return self._mock._call("ALMemory", "getListData", keys)

    def removeData(self, key: str) -> None:
        self._mock._call("ALMemory", "removeData", key)


class _VideoDevice:
    """ALVideoDevice as naoqi-viewer's camera display uses it (specs/cameras.md): named
    subscriptions NAOqi suffixes `_0` to `_5` and refuses beyond, and images in NAOqi's list
    layout."""

    def __init__(self, mock: "MockNaoqi") -> None:
        self._mock = mock

    def subscribeCameras(
        self,
        name: str,
        cameraIndexes: Any,
        resolutions: Any,
        colorSpaces: Any,
        fps: int,
    ) -> str:
        return self._mock._call(
            "ALVideoDevice",
            "subscribeCameras",
            name,
            cameraIndexes,
            resolutions,
            colorSpaces,
            fps,
        )

    # Bound with NAOqi's signature, so the pixels travel as a qi raw buffer (a `bytearray` on the
    # client), as they do from a robot: a plain `bytes` inside a dynamic value becomes a string.
    @qi.bind(returnType="[(iiiiiiriffff)]", paramsType=(qi.String,))
    def getImagesRemote(self, name: str) -> list[Any]:
        return self._mock._call("ALVideoDevice", "getImagesRemote", name)

    def unsubscribe(self, name: str) -> bool:
        return self._mock._call("ALVideoDevice", "unsubscribe", name)

    def unsubscribeAllInstances(self, name: str) -> bool:
        return self._mock._call("ALVideoDevice", "unsubscribeAllInstances", name)

    def getSubscribers(self) -> list[str]:
        return self._mock._call("ALVideoDevice", "getSubscribers")


class _NaoqiSim:
    def ping(self) -> bool:
        return True


class _System:
    def __init__(self, version: str) -> None:
        self._version = version

    def systemVersion(self) -> str:
        return self._version


class MockNaoqi:
    """A scripted NAOqi. Set the pose with `set_pose`, break it with `fail_calls`/`disconnect`."""

    def __init__(
        self,
        target: Target = "virtual",
        version: str = "2.1.4.13",
        body_names: Sequence[str] = JOINT_NAMES,
    ) -> None:
        self.target: Target = target
        self.version = version
        self.calls: list[tuple[str, str, tuple[Any, ...]]] = []
        self._body_names = list(body_names)
        self._lock = threading.Lock()
        self._measured: dict[str, float] = dict.fromkeys(self._body_names, 0.0)
        self._commanded: dict[str, float] = dict.fromkeys(self._body_names, 0.0)
        self._torso = np.array(STANDING_TORSO, dtype=float).reshape(4, 4)
        self._effector_errors: dict[str, np.ndarray] = {}
        self._clips: dict[str, tuple[float, float]] = {}
        self._memory: dict[str, Any] = {}
        self._subscriptions: dict[
            str, list[int]
        ] = {}  # subscription name -> camera indexes
        self._images: dict[int, np.ndarray] = {}  # camera index -> RGB image
        self._failure: str | None = None
        self._session: Any = None
        self.url = "tcp://127.0.0.1:0"
        self._model = load_world(variant="placeholder")
        self._data = mujoco.MjData(self._model)
        self._writer = PoseWriter(self._model)

    # --- Lifecycle ----------------------------------------------------------------

    def start(self) -> "MockNaoqi":
        """Serve on a free loopback port (or, after `disconnect`, on the same one again)."""
        session = qi.Session()
        session.listenStandalone(self.url)
        self.url = next(
            e for e in session.endpoints() if e.startswith("tcp://127.0.0.1")
        )
        session.registerService("ALMotion", _Motion(self))
        session.registerService("ALMemory", _Memory(self))
        session.registerService("ALVideoDevice", _VideoDevice(self))
        if self.target == "naoqi-sim":
            with self._lock:
                # naoqi-sim publishes its own version too: identification must not read it.
                self._memory["NaoqiSim/Version"] = "0.0.1"
                self._memory["NaoqiSim/NaoqiVersion"] = self.version
            session.registerService("NaoqiSim", _NaoqiSim())
        elif self.target == "real":
            session.registerService("ALSystem", _System(self.version))
        self._session = session
        return self

    def disconnect(self) -> None:
        """Stop serving: connected clients lose their session, new connections fail."""
        if self._session is not None:
            self._session.close()
            self._session = None

    def restart(self) -> None:
        """Serve again on the same port, as a restarted NAOqi would (with the current `target`,
        and without the camera subscriptions, which a NAOqi loses when it restarts)."""
        self.disconnect()
        with self._lock:
            self._subscriptions.clear()
        self.start()

    def close(self) -> None:
        self.disconnect()

    # --- Test controls --------------------------------------------------------------

    def set_pose(
        self,
        angles: Mapping[str, float],
        torso_transform: Sequence[float] | None = None,
        commanded: Mapping[str, float] | None = None,
    ) -> None:
        """Set measured angles (and the commanded ones, which default to the same)."""
        with self._lock:
            self._measured.update(angles)
            self._commanded.update(angles if commanded is None else commanded)
            if torso_transform is not None:
                self._torso = np.asarray(torso_transform, dtype=float).reshape(4, 4)

    def set_effector_error(self, effector: str, transform: Sequence[float]) -> None:
        """Report `effector` off by `transform` (16 floats, row-major, in the effector's frame)."""
        with self._lock:
            self._effector_errors[effector] = np.asarray(
                transform, dtype=float
            ).reshape(4, 4)

    def clip_joint(self, name: str, low: float, high: float) -> None:
        """Make angleInterpolation stop `name` within [low, high], as NAOqi's own clipping does."""
        with self._lock:
            self._clips[name] = (low, high)

    def set_image(self, camera: str, image: np.ndarray | None) -> None:
        """What `getImagesRemote` serves for `camera` ("top" or "bottom"): an RGB uint8 image,
        or None for a camera with nothing to serve (as a NAOqi without that camera)."""
        index = {"top": 0, "bottom": 1}[camera]
        with self._lock:
            if image is None:
                self._images.pop(index, None)
            else:
                self._images[index] = np.ascontiguousarray(image, dtype=np.uint8)

    def subscribers(self) -> list[str]:
        """The live ALVideoDevice subscription names."""
        with self._lock:
            return list(self._subscriptions)

    def fail_calls(self, message: str | None) -> None:
        """Make every ALMotion and ALVideoDevice call raise `message` (None: back to normal)."""
        with self._lock:
            self._failure = message

    def calls_to(self, method: str) -> list[tuple[Any, ...]]:
        with self._lock:
            return [args for _, name, args in self.calls if name == method]

    # --- Service implementation -------------------------------------------------------

    def _call(self, service: str, method: str, *args: Any) -> Any:
        with self._lock:
            self.calls.append((service, method, args))
            if (
                service in ("ALMotion", "ALVideoDevice", "ALMemory")
                and method != "getData"
                and self._failure is not None
            ):
                raise RuntimeError(self._failure)
            return getattr(self, f"_{method}")(*args)

    def _getBodyNames(self, name: str) -> list[str]:
        return list(self._body_names)

    def _getAngles(self, names: Any, use_sensors: bool) -> list[float]:
        angles = self._measured if use_sensors else self._commanded
        if names == "Body":
            names = self._body_names
        elif isinstance(names, str):
            names = [names]
        return [float(angles[name]) for name in names]

    def _getTransform(self, name: str, frame: int, use_sensors: bool) -> list[float]:
        if name == "Torso":
            matrix = self._torso if frame == FRAME_WORLD else np.eye(4)
        elif name in EFFECTORS:
            matrix = self._effector(name, frame)
        else:
            raise RuntimeError(f"ALMotion.getTransform: unknown name {name!r}")
        return [float(v) for v in matrix.flatten()]

    def _effector(self, name: str, frame: int) -> np.ndarray:
        pose = NaoPose.from_naoqi(
            list(self._measured),
            list(self._measured.values()),
            self._torso.flatten().tolist(),
        )
        self._writer.apply(self._data, pose)
        site = self._data.site(name)
        world = _transform(site.xmat.reshape(3, 3), site.xpos)
        if name in self._effector_errors:
            world = world @ self._effector_errors[name]
        if frame == FRAME_WORLD:
            return world
        return np.linalg.inv(self._torso) @ world

    def _setStiffnesses(self, names: Any, stiffnesses: Any) -> None:
        pass

    def _angleInterpolation(
        self, names: Any, angles: Any, times: Any, is_absolute: bool
    ) -> None:
        if isinstance(names, str):
            names, angles = [names], [angles]
        for name, angle in zip(names, angles, strict=True):
            target = angle[-1] if isinstance(angle, list) else angle
            if name in self._clips:
                target = min(max(target, self._clips[name][0]), self._clips[name][1])
            self._measured[name] = float(target)
            self._commanded[name] = float(target)

    def _getData(self, key: str) -> Any:
        if key not in self._memory:
            raise RuntimeError(f"ALMemory.getData: no key {key!r}")
        return self._memory[key]

    def _insertData(self, key: str, value: Any) -> None:
        self._memory[key] = value

    def _getListData(self, keys: list[str]) -> list[Any]:
        # NAOqi answers a missing key with None in its slot, not an error.
        return [self._memory.get(key) for key in keys]

    def _removeData(self, key: str) -> None:
        self._memory.pop(key, None)

    def _subscribeCameras(
        self, name: str, indexes: Any, resolutions: Any, colorspaces: Any, fps: int
    ) -> str:
        for n in range(6):
            candidate = f"{name}_{n}"
            if candidate not in self._subscriptions:
                self._subscriptions[candidate] = [int(i) for i in indexes]
                return candidate
        raise RuntimeError(
            f"ALVideoDevice.subscribeCameras: too many instances of {name!r}"
        )

    def _getImagesRemote(self, name: str) -> list[Any]:
        if name not in self._subscriptions:
            return []
        now = time.time()
        seconds, micros = int(now), int((now - int(now)) * 1e6)
        reply: list[Any] = []
        for index in self._subscriptions[name]:
            image = self._images.get(index)
            if image is None:  # a camera with nothing to serve: no size, no bytes
                reply.append([0, 0, 0, 11, 0, 0, b"", index, 0.0, 0.0, 0.0, 0.0])
                continue
            height, width = image.shape[:2]
            reply.append(
                [
                    width,
                    height,
                    3,
                    11,
                    seconds,
                    micros,
                    image.tobytes(),
                    index,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ]
            )
        return reply

    def _unsubscribe(self, name: str) -> bool:
        return self._subscriptions.pop(name, None) is not None

    def _unsubscribeAllInstances(self, name: str) -> bool:
        instances = [n for n in self._subscriptions if n.rsplit("_", 1)[0] == name]
        for instance in instances:
            del self._subscriptions[instance]
        return bool(instances)

    def _getSubscribers(self) -> list[str]:
        return list(self._subscriptions)
