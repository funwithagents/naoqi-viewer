"""Viewer configuration: `NaoqiViewerConfig` and its blocks (specs/config.md).

One declarative description of a viewer, built in code or loaded with the `from_dict` /
`from_json` / `from_json_file` trio. Invalid input raises `ConfigError` naming the offending key
path. Each block checks its own ranges on construction; the loaders also check shape, types and
unknown keys. A default is declared once, on the dataclass field: `parse_block` passes only the
keys present and lets the dataclass apply the rest. Follows naoqi-bridge's config pattern; imports
neither mujoco nor qi.
"""

import json
import math
from collections.abc import Callable
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Literal, Self

_SCENES_DIR = Path(__file__).parent / "scenes"

type Mode = Literal["mirror", "sim"]
type Variant = Literal["auto", "placeholder", "aldebaran"]
MODES: tuple[Mode, ...] = ("mirror", "sim")
VARIANTS: tuple[Variant, ...] = ("auto", "placeholder", "aldebaran")

# Turns a raw JSON value found at a key path into a field's value, or raises ConfigError.
type Reader[T] = Callable[[Any, str], T]


class ConfigError(ValueError):
    """A malformed configuration. `key` is the offending key path (empty when the error isn't
    about one key) and `detail` the complaint; `str()` joins them."""

    def __init__(self, detail: str, *, key: str = "") -> None:
        super().__init__(f"{key} {detail}" if key else detail)
        self.key = key
        self.detail = detail


def _key_path(path: str, key: str) -> str:
    return f"{path}.{key}" if path else key


# region Value readers


def _as_bool(value: Any, key: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"must be a boolean, got {value!r}", key=key)
    return value


def _as_number(value: Any, key: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(f"must be a number, got {value!r}", key=key)
    return float(value)


def _as_str(value: Any, key: str) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"must be a string, got {value!r}", key=key)
    return value


def _as_choice[C: str](choices: tuple[C, ...]) -> Reader[C]:
    def read(value: Any, key: str) -> C:
        for choice in choices:
            if value == choice:
                return choice
        expected = ", ".join(repr(c) for c in choices)
        raise ConfigError(f"must be one of {expected}, got {value!r}", key=key)

    return read


def _parse_block[T](
    factory: Callable[..., T], data: Any, path: str, **readers: Reader[Any]
) -> T:
    """Build the block `factory` from the JSON object `data` found at `path`.

    `readers` names the allowed keys and how to read each; an absent key is left to the
    dataclass default, an unknown one is an error. The block's own validation errors
    (`__post_init__`) are prefixed with `path`.
    """
    if not isinstance(data, dict):
        raise ConfigError(
            f"must be an object, got {type(data).__name__}", key=path or "config"
        )
    for key in data:
        if key not in readers:
            raise ConfigError(
                f"unknown key {_key_path(path, key)!r}; expected one of: {', '.join(readers)}"
            )
    kwargs = {
        name: reader(data[name], _key_path(path, name))
        for name, reader in readers.items()
        if name in data
    }
    try:
        return factory(**kwargs)
    except ConfigError as exc:
        raise ConfigError(exc.detail, key=_key_path(path, exc.key)) from None


def _require_positive(value: float, key: str) -> None:
    if not (math.isfinite(value) and value > 0):
        raise ConfigError(f"must be a positive, finite number, got {value}", key=key)


def _require_nonempty(value: str, key: str) -> None:
    if not value:
        raise ConfigError("must not be empty", key=key)


# endregion


def bundled_scenes() -> tuple[str, ...]:
    """The scene names shipped in naoqi_viewer/scenes/."""
    return tuple(sorted(path.stem for path in _SCENES_DIR.glob("*.xml")))


def is_scene_path(scene: str) -> bool:
    """A scene ending in .xml is a path to an MJCF file; anything else is a bundled name."""
    return scene.endswith(".xml")


class JsonConfig:
    """The loader trio shared by every config class: `from_json_file` reads, `from_json` parses,
    `from_dict` validates and builds."""

    @classmethod
    def parse(cls, data: Any, path: str) -> Self:
        """Validate `data` found at key `path` and build the block."""
        raise NotImplementedError

    @classmethod
    def from_dict(cls, data: Any) -> Self:
        return cls.parse(data, "")

    @classmethod
    def from_json(cls, text: str) -> Self:
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ConfigError(f"invalid JSON: {exc}") from exc
        return cls.from_dict(data)

    @classmethod
    def from_json_file(cls, path: str | Path) -> Self:
        file = Path(path)
        try:
            text = file.read_text(encoding="utf-8")
        except OSError as exc:
            raise ConfigError(f"cannot read config file {file}: {exc}") from exc
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ConfigError(f"{file}: invalid JSON: {exc}") from exc
        return cls.from_dict(data)._relative_to(file.parent)

    def _relative_to(self, directory: Path) -> Self:
        """Resolve relative paths in the config against `directory` (the config file's)."""
        return self

    def to_dict(self) -> dict[str, Any]:
        """Every field, as JSON-compatible data; `from_dict` reads it back to an equal config."""
        return asdict(self)  # type: ignore[call-overload]  # every subclass is a dataclass


@dataclass(frozen=True)
class NaoqiSettings(JsonConfig):
    """Which NAOqi to watch, and how often to read its pose."""

    url: str = (
        "tcp://127.0.0.1:9559"  # tcp://host:port, host:port, or a bare host (port 9559)
    )
    rate_hz: float = 50.0

    def __post_init__(self) -> None:
        _require_nonempty(self.url, "url")
        _require_positive(self.rate_hz, "rate_hz")

    @classmethod
    def parse(cls, data: Any, path: str) -> Self:
        return _parse_block(cls, data, path, url=_as_str, rate_hz=_as_number)


@dataclass(frozen=True)
class WorldSettings(JsonConfig):
    """The world around the robot, and how the robot looks."""

    scene: str = "empty"  # a bundled scene name, or a path to an MJCF file (.xml)
    variant: Variant = "auto"

    def __post_init__(self) -> None:
        _require_nonempty(self.scene, "scene")
        if not is_scene_path(self.scene) and self.scene not in bundled_scenes():
            raise ConfigError(
                f"{self.scene!r} is not a bundled scene; bundled scenes: {', '.join(bundled_scenes())} "
                "(or a path to an MJCF file ending in .xml)",
                key="scene",
            )
        if self.variant not in VARIANTS:
            raise ConfigError(
                f"must be one of {', '.join(map(repr, VARIANTS))}, got {self.variant!r}",
                key="variant",
            )

    @classmethod
    def parse(cls, data: Any, path: str) -> Self:
        return _parse_block(
            cls, data, path, scene=_as_str, variant=_as_choice(VARIANTS)
        )

    def _relative_to(self, directory: Path) -> Self:
        if is_scene_path(self.scene) and not Path(self.scene).is_absolute():
            return replace(self, scene=str(directory / self.scene))
        return self


@dataclass(frozen=True)
class NaoqiViewerConfig(JsonConfig):
    """A viewer: its mode, the NAOqi it watches, its world and options (specs/config.md)."""

    mode: Mode = "mirror"
    headless: bool = False  # no window, offscreen rendering only (sim mode)
    naoqi: NaoqiSettings = field(default_factory=NaoqiSettings)
    world: WorldSettings = field(default_factory=WorldSettings)
    ghost: bool = False
    cameras_display: bool = False  # the head cameras from ALVideoDevice, in the window
    cameras_mirror: bool = (
        True  # those pictures mirrored, to match the view facing the robot
    )
    launch_timeout_s: float = 30.0

    def __post_init__(self) -> None:
        if self.mode not in MODES:
            raise ConfigError(
                f"must be one of {', '.join(map(repr, MODES))}, got {self.mode!r}",
                key="mode",
            )
        _require_positive(self.launch_timeout_s, "launch_timeout_s")
        if self.headless and self.mode == "mirror":
            raise ConfigError(
                "requires mode 'sim': a headless mirror has nothing to show",
                key="headless",
            )
        if self.headless and self.ghost:
            raise ConfigError(
                "is drawn only in the window, and a headless viewer has none",
                key="ghost",
            )
        if self.headless and self.cameras_display:
            raise ConfigError(
                "is drawn only in the window, and a headless viewer has none",
                key="cameras_display",
            )

    @classmethod
    def parse(cls, data: Any, path: str) -> Self:
        return _parse_block(
            cls,
            data,
            path,
            mode=_as_choice(MODES),
            headless=_as_bool,
            naoqi=NaoqiSettings.parse,
            world=WorldSettings.parse,
            ghost=_as_bool,
            cameras_display=_as_bool,
            cameras_mirror=_as_bool,
            launch_timeout_s=_as_number,
        )

    def _relative_to(self, directory: Path) -> Self:
        return replace(self, world=self.world._relative_to(directory))
