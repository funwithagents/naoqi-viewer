import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from naoqi_viewer import ConfigError, NaoqiSettings, NaoqiViewerConfig, WorldSettings

EXAMPLES = Path(__file__).resolve().parent.parent / "examples" / "configs"


def config_error(data: Any) -> ConfigError:
    with pytest.raises(ConfigError) as raised:
        NaoqiViewerConfig.from_dict(data)
    return raised.value


def test_defaults():
    config = NaoqiViewerConfig()
    assert NaoqiViewerConfig.from_dict({}) == config
    assert (config.mode, config.headless, config.naoqi.url, config.naoqi.rate_hz) == (
        "mirror",
        False,
        "tcp://127.0.0.1:9559",
        50.0,
    )
    assert (
        config.world.scene,
        config.world.variant,
        config.ghost,
        config.cameras_display,
        config.cameras_mirror,
        config.launch_timeout_s,
    ) == (
        "empty",
        "auto",
        False,
        False,
        True,
        30.0,
    )


def test_a_partial_block_keeps_the_other_defaults():
    config = NaoqiViewerConfig.from_dict(
        {"naoqi": {"url": "nao.local"}, "world": {"scene": "table"}}
    )
    assert config.naoqi == NaoqiSettings(url="nao.local", rate_hz=50.0)
    assert config.world == WorldSettings(scene="table", variant="auto")
    assert config.mode == "mirror"


def test_to_dict_round_trips_and_is_json():
    config = NaoqiViewerConfig(
        mode="sim",
        naoqi=NaoqiSettings(url="tcp://10.0.0.7:9600", rate_hz=30),
        world=WorldSettings(scene="table", variant="placeholder"),
        ghost=True,
        cameras_display=True,
        cameras_mirror=False,
        launch_timeout_s=12,
    )
    data = config.to_dict()
    assert data == {
        "mode": "sim",
        "headless": False,
        "naoqi": {"url": "tcp://10.0.0.7:9600", "rate_hz": 30},
        "world": {"scene": "table", "variant": "placeholder"},
        "ghost": True,
        "cameras_display": True,
        "cameras_mirror": False,
        "launch_timeout_s": 12,
    }
    assert NaoqiViewerConfig.from_json(json.dumps(data)) == config
    headless = NaoqiViewerConfig(mode="sim", headless=True)
    assert NaoqiViewerConfig.from_dict(headless.to_dict()) == headless


@pytest.mark.parametrize(
    ("data", "key", "detail"),
    [
        ({"mode": "replay"}, "mode", "must be one of 'mirror', 'sim'"),
        ({"naoqi": {"rate_hz": 0}}, "naoqi.rate_hz", "positive, finite"),
        ({"naoqi": {"rate_hz": -5}}, "naoqi.rate_hz", "positive, finite"),
        ({"naoqi": {"rate_hz": True}}, "naoqi.rate_hz", "must be a number"),
        ({"naoqi": {"rate_hz": "50"}}, "naoqi.rate_hz", "must be a number"),
        ({"naoqi": {"url": ""}}, "naoqi.url", "must not be empty"),
        ({"naoqi": {"url": 9559}}, "naoqi.url", "must be a string"),
        ({"naoqi": "tcp://nao.local"}, "naoqi", "must be an object"),
        ({"world": {"variant": "meshes"}}, "world.variant", "must be one of"),
        ({"world": {"scene": ""}}, "world.scene", "must not be empty"),
        ({"ghost": "yes"}, "ghost", "must be a boolean"),
        ({"mode": "sim", "headless": 1}, "headless", "must be a boolean"),
        ({"headless": True}, "headless", "requires mode 'sim'"),
        ({"mode": "mirror", "headless": True}, "headless", "requires mode 'sim'"),
        ({"mode": "sim", "headless": True, "ghost": True}, "ghost", "headless"),
        ({"cameras_display": 1}, "cameras_display", "must be a boolean"),
        ({"cameras_mirror": "no"}, "cameras_mirror", "must be a boolean"),
        (
            {"mode": "sim", "headless": True, "cameras_display": True},
            "cameras_display",
            "headless",
        ),
        ({"launch_timeout_s": 0}, "launch_timeout_s", "positive, finite"),
    ],
)
def test_errors_name_the_key(data: Any, key: str, detail: str):
    error = config_error(data)
    assert error.key == key
    assert detail in error.detail
    assert str(error).startswith(f"{key} ")


def test_the_camera_display_is_valid_in_both_modes():
    for mode in ("mirror", "sim"):
        config = NaoqiViewerConfig.from_dict({"mode": mode, "cameras_display": True})
        assert config.cameras_display is True


def test_the_mirror_setting_needs_no_display():
    headless = NaoqiViewerConfig.from_dict(
        {"mode": "sim", "headless": True, "cameras_mirror": False}
    )
    assert headless.cameras_mirror is False


def test_non_finite_numbers_are_rejected():
    error = config_error(json.loads('{"launch_timeout_s": Infinity}'))
    assert error.key == "launch_timeout_s"


def test_unknown_keys_are_errors():
    assert "unknown key 'ghosts'" in str(config_error({"ghosts": True}))
    message = str(config_error({"naoqi": {"port": 9559}}))
    assert "unknown key 'naoqi.port'" in message and "url, rate_hz" in message


def test_an_unknown_bundled_scene_lists_the_bundled_ones():
    error = config_error({"world": {"scene": "kitchen"}})
    assert error.key == "world.scene"
    assert "empty, table" in error.detail


def test_a_scene_path_is_not_checked_at_load():
    config = NaoqiViewerConfig.from_dict({"world": {"scene": "/not/yet/written.xml"}})
    assert config.world.scene == "/not/yet/written.xml"


def test_relative_scene_paths_in_a_file_resolve_against_the_files_directory(
    tmp_path: Path,
):
    folder = tmp_path / "setup"
    folder.mkdir()
    (folder / "viewer.json").write_text('{"world": {"scene": "scenes/lab.xml"}}')
    config = NaoqiViewerConfig.from_json_file(folder / "viewer.json")
    assert config.world.scene == str(folder / "scenes" / "lab.xml")
    absolute = NaoqiViewerConfig.from_dict({"world": {"scene": "/abs/lab.xml"}})
    assert absolute.world.scene == "/abs/lab.xml"


def test_direct_construction_validates_too():
    with pytest.raises(ConfigError, match="rate_hz"):
        NaoqiSettings(rate_hz=0)
    with pytest.raises(ConfigError, match="kitchen"):
        WorldSettings(scene="kitchen")
    with pytest.raises(ConfigError, match="mode"):
        NaoqiViewerConfig(mode="replay")  # type: ignore[arg-type]


def test_invalid_json_and_unreadable_files_name_the_file(tmp_path: Path):
    broken = tmp_path / "broken.json"
    broken.write_text("{mode: sim}")
    with pytest.raises(ConfigError, match="broken.json: invalid JSON"):
        NaoqiViewerConfig.from_json_file(broken)
    with pytest.raises(ConfigError, match="missing.json"):
        NaoqiViewerConfig.from_json_file(tmp_path / "missing.json")
    with pytest.raises(ConfigError, match="invalid JSON"):
        NaoqiViewerConfig.from_json("{")
    with pytest.raises(ConfigError, match="config must be an object"):
        NaoqiViewerConfig.from_json("[]")


@pytest.mark.parametrize("path", sorted(EXAMPLES.glob("*.json")), ids=lambda p: p.name)
def test_example_files_load(path: Path):
    config = NaoqiViewerConfig.from_json_file(path)
    assert config.to_dict() == json.loads(path.read_text())  # every field written out


def test_the_examples_exist():
    assert {p.name for p in EXAMPLES.glob("*.json")} >= {
        "mirror.json",
        "sim-table.json",
        "sim-headless.json",
    }


def test_importing_the_config_loads_neither_mujoco_nor_qi():
    code = "import sys, naoqi_viewer.config; print(sorted(m for m in ('mujoco', 'qi') if m in sys.modules))"
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert result.stdout.strip() == "[]"
