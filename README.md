# naoqi-viewer

A MuJoCo viewer for the NAO robot. It shows any NAOqi robot live in a 3D window, and it can also act as naoqi-sim's simulated world. The package includes the NAO model and a license-gated fetcher for Aldebaran's meshes.

naoqi-viewer is a **kinematic mirror, not a physics simulation**. It reads joint angles and the torso pose from NAOqi over qi at 50 Hz, writes them into the model and renders at 60 Hz. It never calls `mj_step` and never writes to the robot. The target can be a real NAO, naoqi-sim or a desktop `naoqi-bin`, and nothing needs to change on the robot side.

naoqi-viewer is one of the three packages of the NAO toolkit (naoqi-bridge, naoqi-sim, naoqi-viewer). It is unofficial and not affiliated with Aldebaran.

## Two modes

| | `mirror` | `sim` |
| --- | --- | --- |
| For | Watching any NAO, usually a real robot | naoqi-sim's simulated world, or a plain virtual robot |
| Camera frames | Refused: a real robot has its own cameras | RGB renders from `CameraTop` / `CameraBottom` at the robot's current pose |
| Headless | No | Yes: offscreen rendering, with no display or GPU needed |

The window shows a status overlay (target, NAOqi version, update rate, data age, and the camera images' age when they are shown; toggle it with `F9`). It can also show a **ghost**: a translucent copy of the commanded pose drawn over the measured one, which shows limbs that are blocked or have stiffness off.

## Install

Requirements: Python 3.12 or 3.13 on **macOS 11+ arm64**, **Linux x86_64** (glibc 2.28+) or **Windows x64**. These are the platforms the libqi wheels cover.

```sh
git clone git@github.com:funwithagents/naoqi-viewer.git
cd naoqi-viewer
uv sync
```

libqi (`qi>=3.1.8,<4`) comes from the package index of [funwithagents/libqi-python](https://github.com/funwithagents/libqi-python), `https://funwithagents.github.io/libqi-python/simple/`, not from PyPI (whose `qi` is Aldebaran's older 3.1.5). uv resolves it through `pyproject.toml`, also when naoqi-viewer is a git dependency of your project. With pip, add `--extra-index-url https://funwithagents.github.io/libqi-python/simple/`.

On macOS, opening a window needs `mjpython`, which ships with `mujoco`. naoqi-viewer starts it for you. Headless rendering on Linux uses Mesa's EGL. On slim images, install it with `apt install libegl1 libopengl0 libgl1-mesa-dri`. On Windows, rendering needs OpenGL 2 or later from a GPU driver. On a machine without one (a VM), put Mesa's `opengl32.dll` and `libgallium_wgl.dll` ([mesa-dist-win](https://github.com/pal1000/mesa-dist-win), `x64/`) next to the `python.exe` the venv was made from.

## Command line

```sh
uv run naoqi-viewer view                                         # mirror a NAOqi on 127.0.0.1:9559
uv run naoqi-viewer view --config examples/configs/mirror.json   # mirror a robot (edit its URL first)
uv run naoqi-viewer view --config examples/configs/sim-table.json
uv run naoqi-viewer check-model tcp://127.0.0.1:9559             # forward-kinematics check, 2 mm / 1°
uv run naoqi-viewer fetch-meshes                                 # install Aldebaran's meshes (asks for the license)
```

| Command | Does |
| --- | --- |
| `view [--config FILE]` | Opens a viewer from a config file and returns when the window closes. Without a config, it opens mirror mode on a local NAOqi with the empty scene. |
| `check-model URL [--samples N] [--seed N] [--tolerance-mm X] [--tolerance-deg X] [--allow-real]` | Moves a NAOqi through random configurations and compares its effector transforms with the model's sites. Exits 0 when they are within tolerance. **It moves every joint**, so it refuses a real robot unless you pass `--allow-real`. |
| `fetch-meshes [--archive PATH] [--force] [--remove]` | Shows the license and waits for you to type `yes`, then installs Aldebaran's meshes locally. `--remove` deletes them. |

`-v` / `-q` before the command set the log level. Exit codes: 0 for success, 1 for a failure, 2 for a usage or config error.

## Configuration

All settings for `view` come from one JSON file. Every key is optional, and missing keys take the defaults shown here:

```json
{
  "mode": "mirror",
  "headless": false,
  "naoqi": { "url": "tcp://127.0.0.1:9559", "rate_hz": 50 },
  "world": { "scene": "empty", "variant": "auto" },
  "ghost": false,
  "cameras_display": false,
  "launch_timeout_s": 30
}
```

- `world.scene` takes a bundled scene (`empty`, or `table`, which gives the cameras something to see) or a path to your own MJCF file ending in `.xml`. A relative path is resolved against the config file's directory.
- `world.variant` can be `auto`, `placeholder` or `aldebaran`. `auto` uses Aldebaran's meshes when they are installed and the placeholder visuals otherwise.
- `cameras_display: true` shows the two head cameras, read from NAOqi's `ALVideoDevice`, as pictures in the window's top-right corner (toggle them with `F10`). It works in both modes: on a real robot they are its cameras, on naoqi-sim the frames it injected. It costs NAOqi one camera subscription (QVGA at 10 fps), which the viewer removes when it closes.
- `headless: true` works only in `sim` mode, and cannot be combined with `ghost` or `cameras_display`.
- An unknown key or a bad value fails at load time, and the error names the key, for example `naoqi.rate_hz must be a positive, finite number, got 0`.

Ready-made files are in [examples/configs/](examples/configs/): `mirror.json`, `sim-table.json` and `sim-headless.json`.

## Python API

```python
from naoqi_viewer import NaoqiViewer, NaoqiViewerConfig

config = NaoqiViewerConfig.from_json_file("examples/configs/sim-table.json")
with NaoqiViewer(config) as viewer:  # launch() on enter, close() on exit
    frame = viewer.camera_frame("top", 640, 480)  # frame.image: (480, 640, 3) uint8 RGB
    print(viewer.status())  # target, NAOqi version, rate, data age…
    viewer.wait()  # until the window is closed
```

- Each viewer runs in **its own process**. Importing `naoqi_viewer` loads neither `mujoco` nor `qi`. MuJoCo, OpenGL and `mjpython` stay in the viewer process, and the viewer process exits when your process does.
- `launch()` returns once the window is up, even if NAOqi is unreachable: the viewer keeps reconnecting in the background. If the viewer cannot start (a missing scene, meshes requested but not installed, no display), `launch()` raises `LaunchError`.
- Building a viewer from config data also works: `NaoqiViewer.from_dict(...)`, `from_json(...)` and `from_json_file(...)`. A program can embed the config as one block of its own config.
- Exceptions: `ConfigError`, `LaunchError`, `ViewerClosed`, and `ModeError` (raised for `camera_frame` in mirror mode).

## Aldebaran's meshes

The repository and the package ship only placeholder visuals, which are our own work. Aldebaran's NAO meshes are under **CC BY-NC-ND 4.0 (non-commercial)** and are never committed or packaged. `naoqi-viewer fetch-meshes` downloads a pinned, encrypted archive from [funwithagents/naoqi-meshes](https://github.com/funwithagents/naoqi-meshes), shows the license, and unlocks the archive only after you type `yes`. The meshes go into your user data directory (`platformdirs.user_data_dir("naoqi-viewer")`). The model built from them exists only in memory, and the window shows an attribution to Aldebaran whenever the meshes are on screen.

## NAOqi versions

- **NAOqi 2.1 (NAO V5)** is validated: the model matches NAOqi's forward kinematics within 0.03 mm and 0.03°, and CI runs against naoqi-sim's NAOqi 2.1.
- **NAOqi 2.8 (NAO V6)** should work through the same qi calls, but has not been validated yet.

## Development

```sh
uv sync --dev
uv run ruff check . && uv run ruff format .
uv run pyright
uv run pytest                                          # fast tier: in-process mock NAOqi, no network
NAOQI_URL=tcp://127.0.0.1:9559 uv run pytest tests-e2e # live tier against a running NAOqi
```

The project is spec-driven: start with [specs/_overview.md](specs/_overview.md) for the architecture and status, and read [AGENTS.md](AGENTS.md) for the workflow.

## License

The code, the model (`nao.xml`) and the scenes are under the MIT license ([LICENSE](LICENSE)). The vendored NAO URDF is BSD-3-Clause ([THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)). Aldebaran's meshes are not part of this repository (see above).
