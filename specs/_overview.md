# naoqi-viewer — Overview

naoqi-viewer is the MuJoCo window for NAO. It is a **kinematic mirror, not a physics simulation**: it reads joint angles and the torso pose from any NAOqi endpoint over qi, writes them into the NAO model's `qpos`, runs `mj_kinematics` and renders. The same viewer watches a real robot (**mirror** mode) or acts as naoqi-sim's simulated world (**sim** mode), where naoqi-sim chooses a scene and pulls head-camera renders to inject into `ALVideoDevice`. naoqi-viewer also owns the NAO model: one MJCF, converted once from the BSD-3 URDF, committed, and drawn with placeholder visuals of our own.

It is one of the three packages of the NAO toolkit (naoqi-bridge, naoqi-sim, naoqi-viewer), unofficial and MIT-licensed, not affiliated with Aldebaran, and it ships no Aldebaran assets.

This overview is the map of the whole project. Each concept has its own spec (see [_index.md](_index.md) for the list and statuses); the sections here only summarize them and link to them. Where this page and a concept spec disagree, the concept spec wins.

## Goals

- Any NAOqi target (a real NAO, naoqi-sim, a plain desktop `naoqi-bin`) can be watched live in a 3D window, with no change on the robot side: naoqi-viewer is an ordinary qi client that only reads.
- naoqi-sim gets its simulated world from a Python object: `NaoqiViewer(config).launch()`, then `camera_frame(...)` for head-camera renders at the robot's current pose, windowed or headless, with no MuJoCo, OpenGL or `qi` import in naoqi-sim's own process.
- The model's forward kinematics match NAOqi's within 2 mm and 1°, proven by `check-model` against a live NAOqi.
- No Aldebaran mesh, texture or mesh-derived file lands in the repository or a package. Aldebaran's meshes reach a user's screen only through a license-gated local install.

## Scope

| In scope | Out of scope |
| --- | --- |
| The NAO model: MJCF, joints, sensors, cameras, effector sites, placeholder visuals | Physics: the viewer never calls `mj_step`; the pose comes only from NAOqi |
| Mirror mode: watching any NAO, with a status overlay and a ghost of the commanded pose | Writing to NAOqi: the viewer is read-only (only `check-model` moves joints, and refuses a real robot by default) |
| Sim mode: scenes around the robot, props (faces, objects) spawned and moved in them at runtime, head-camera renders on request, windowed or headless | Serving camera images to NAOqi clients (`putImage`, the colorspaces apps are served): naoqi-sim owns it |
| Camera display: the two head cameras read from `ALVideoDevice` as pictures in the window, in both modes | Processing those images (detection, recording): the viewer only shows them |
| People display: the people NAOqi lists in ALMemory, as markers in the window, in both modes | Publishing people to NAOqi (a detector, a fake population): the clients' or naoqi-sim's |
| `check-model`: the forward-kinematics cross-check against NAOqi | A client API for NAO applications: that is naoqi-bridge |
| `fetch-meshes`: license-gated install of Aldebaran's meshes, loaded in place of the placeholders | Any way to export, share or cache converted meshes or a model built from them |
| A Python API (`NaoqiViewer`, `NaoqiViewerConfig`) and a thin `naoqi-viewer` command | Record and replay of joint trajectories, video recording (not in v1) |

## Architecture

Two processes. The **caller's process** (a script, naoqi-sim's host program, the `naoqi-viewer` command) holds a light `NaoqiViewer` object. The **viewer process**, which `launch()` starts, loads MuJoCo, polls NAOqi and owns the window and the renderers. The split keeps MuJoCo's window, OpenGL and the macOS `mjpython` requirement out of the caller.

```
caller's process                       viewer process                              NAOqi
─────────────────                      ─────────────────────────────────           ──────────────────
NaoqiViewer (client.py)  ── loopback ──▶ request server ─┐                           real NAO, naoqi-sim
  launch / status /        TCP         (protocol.py)   │ on_frame                  or naoqi-bin
  camera_frame / close                                 ▼                                 ▲
NaoqiViewerConfig (config.py)            render loop, 60 Hz (viewer.py) ◀── latest() ── NaoqiSource ── qi, 50 Hz
                                         PoseWriter → mj_kinematics (model.py)        (source.py)  getAngles,
                                         window, or headless offscreen renders                     getTransform
```

| Part | Runs where | Role | Spec |
| --- | --- | --- | --- |
| Configuration | Caller | `NaoqiViewerConfig`: mode, headless, NAOqi URL and rate, scene and visuals, ghost; JSON loaders, errors naming the key; embeddable as a block of another program's config | [config.md](config.md) |
| `NaoqiViewer` client | Caller | Starts the viewer process (`mjpython` on macOS, plain Python when headless), waits for its ready line, sends requests, relays its logs, ties its lifetime to the caller's. Standard library and `numpy` only | [api.md](api.md) |
| Protocol | Both | Length-prefixed JSON header plus raw payload over one loopback TCP connection: `hello`, `status`, `camera_frame`, `stop`, the props ops (protocol 2) | [api.md](api.md) |
| Viewer process | Viewer process | Loads the world, builds the pose source, runs the loop, answers requests between frames on the thread that owns the OpenGL contexts | [api.md](api.md) |
| Pose source | Viewer process, its own thread | `connect()` in one cancellable attempt; `NaoqiSource` polls measured (and optionally commanded) angles and the torso pose at 50 Hz, reconnects with backoff, identifies the target (`real`, `naoqi-sim`, `virtual`) | [source.md](source.md) |
| Camera source and display | Viewer process, its own thread; drawn by the window | `CameraSource` subscribes both head cameras in `ALVideoDevice` (QVGA, RGB, 10 fps) and polls them; the window draws the two pictures in its top-right corner, with their age in the status overlay. Off by default (`cameras_display`) | [cameras.md](cameras.md) |
| People source and display | Viewer process, its own thread; drawn by the window | `PeopleSource` reads `PeoplePerception/` from ALMemory (10 Hz); the window draws one labelled sphere per person in its user scene, never in a camera render. Off by default (`people_display`) | [people.md](people.md) |
| Props | Viewer process, render thread | A pool of hidden mocap bodies (4 portrait faces, 8 primitive objects) appended to every sim-mode scene; `viewer.props` spawns, places, despawns and clears them, the window and the camera renders agreeing at every frame | [props.md](props.md) |
| Viewer loop | Viewer process, main thread | `launch_passive` window at 60 Hz with the status overlay, ghost and attribution; or the windowless headless loop | [viewer.md](viewer.md) |
| Model and scenes | Viewer process | `nao.xml`, bundled scenes (`empty`, `table`) or a user MJCF, `load_world`, `NaoPose`, `PoseWriter` | [model.md](model.md) |
| Meshes | User's machine | `fetch-meshes`: typed license acceptance, then the pinned encrypted archive of [naoqi-meshes](https://github.com/funwithagents/naoqi-meshes) (OBJ and PNG) unlocked locally; `variant: "aldebaran"` swaps the placeholder visuals for them | [meshes.md](meshes.md) |
| Model check | Caller's process | `check-model`: random configurations on a NAOqi, effector transforms compared with the model's sites | [check_model.md](check_model.md) |
| `naoqi-viewer` command | Caller's process | `view --config`, `check-model`; arguments only, logic in the modules | [cli.md](cli.md) |

### Two modes

Both run the same loop and the same pose source; the mode only decides what the viewer is for and which requests it accepts ([api.md](api.md)).

| | Mirror | Sim |
| --- | --- | --- |
| For | Watching any NAO, typically a real robot | naoqi-sim's simulated world, or a plain virtual robot |
| Scene | `empty` by default, any scene allowed | `empty` by default; a caller sets `table` or its own MJCF to give the cameras something to see |
| `camera_frame` | Refused (`ModeError`): a real robot has its own cameras | RGB render of `CameraTop`/`CameraBottom` at the current pose |
| Props | Refused (`ModeError`): no pool is appended | Faces and objects spawned, moved and hidden in the scene ([props.md](props.md)) |
| Headless | No (nothing to show) | Yes: offscreen rendering, no display needed (Mesa's EGL on Linux) |
| Camera display | The robot's cameras from `ALVideoDevice` | The same reader: what naoqi-sim injected into `ALVideoDevice`, the viewer's own renders after their round trip ([cameras.md](cameras.md)) |

### What naoqi-viewer relies on from a target

- `ALMotion.getBodyNames("Body")`, `getAngles("Body", True/False)`, `getTransform("Torso", 1, True)`; for `check-model` also `setStiffnesses`, `angleInterpolation` and `getTransform(effector, 0, True)`.
- With the camera display on: `ALVideoDevice.subscribeCameras`, `getImagesRemote`, `unsubscribe` and `unsubscribeAllInstances` ([cameras.md](cameras.md)). In sim mode the images are the ones naoqi-sim put there.
- With the people display on: `ALMemory.getListData` of `PeoplePerception/PeopleList`, `VisiblePeopleList` and each person's `PositionInWorldFrame` ([people.md](people.md)): a real robot's own population, or what a client published on naoqi-sim.
- Target identification: the `NaoqiSim` service marks a naoqi-sim target, whose NAOqi version is the ALMemory key `NaoqiSim/NaoqiVersion`; `ALSystem` and `systemVersion()` mark a real robot; neither means a desktop virtual robot. These names are a contract with naoqi-sim.

### Dependencies

naoqi-viewer depends on libqi (`qi`), `mujoco`, `numpy`, `platformdirs` and `pyyaml` ([project.md](project.md)). It never imports naoqi-bridge or naoqi-sim: naoqi-sim depends on naoqi-viewer (through its optional viewer extra), never the other way round. The tests follow the same rule: the fast tier uses naoqi-viewer's own mock NAOqi, and the live tier reaches naoqi-sim's NAOqi by URL only ([testing.md](testing.md)).

## Status

| Concept | State |
| --- | --- |
| Model, scenes, `PoseWriter` | Built and tested; kinematics checked against NAOqi 2.1 within 0.03 mm and 0.03° ([model.md](model.md)) |
| Pose source | Built and tested, against the mock NAOqi and naoqi-sim's NAOqi 2.1 ([source.md](source.md)) |
| Viewer window and headless loop | Built and tested ([viewer.md](viewer.md)) |
| Config, `NaoqiViewer` API, viewer process, protocol | Built and tested ([config.md](config.md), [api.md](api.md)) |
| Camera display (`cameras_display`, `CameraSource`, the pictures) | Built and tested against the mock NAOqi; the live camera test waits for a run against naoqi-sim ([cameras.md](cameras.md)) |
| `check-model` | Built; passes against NAOqi 2.1 ([check_model.md](check_model.md)) |
| CLI | `view`, `check-model` and `fetch-meshes` built ([cli.md](cli.md)) |
| CI | Lint and types, the fast tier with a real headless viewer, the live tier headless and windowed against naoqi-sim's NAOqi 2.1, and the meshes fetched and rendered ([ci.md](ci.md)) |
| Meshes (`fetch-meshes`, `aldebaran` visuals) | Built; naoqi-meshes `r1` fetched, rendered and tested on macOS ([meshes.md](meshes.md)) |
| NAOqi 2.8 / NAO V6 | Not validated (model, `check-model`, CI) |
| Props (faces and objects in sim-mode scenes, protocol 2) | Built and tested, against the mock NAOqi and naoqi-sim's NAOqi 2.1, headless and windowed ([props.md](props.md)) |
| People display (`people_display`, `PeopleSource`, the markers) | Built and tested, against the mock NAOqi and naoqi-sim's NAOqi 2.1 ([people.md](people.md)) |
| Camera streaming to the caller, touch from clicks | Deferred, each a future protocol op ([api.md](api.md)) |

## Licensing

The code is MIT.

| Asset | Source | License | In the repository or a package? |
| --- | --- | --- | --- |
| Code, `nao.xml` with its placeholder visuals, scenes | this repository | MIT | Yes |
| The four portraits of the props' face pool (`src/naoqi_viewer/portraits/`) | NASA photographs on Wikimedia Commons, cropped; credited in `portraits/ATTRIBUTION.md` | Public domain (works of a U.S. federal agency) | Yes ([props.md](props.md)) |
| NAO URDF (`third_party/nao_description/`) | [ros-naoqi/nao_robot](https://github.com/ros-naoqi/nao_robot) at a pinned commit | BSD-3-Clause | Yes, vendored with its license; notice in `THIRD_PARTY_NOTICES.md`. Nothing reads it at runtime; tests check `nao.xml` against it |
| libqi Python 3 wheels | [funwithagents/libqi-python](https://github.com/funwithagents/libqi-python) GitHub Releases | BSD-3-Clause | A runtime dependency, not vendored |
| NAO meshes and textures | Aldebaran's `ros-naoqi/nao_meshes` installer, converted once to OBJ and PNG and published encrypted by [funwithagents/naoqi-meshes](https://github.com/funwithagents/naoqi-meshes) | CC BY-NC-ND 4.0 (**non-commercial**) | **Never.** Unlocked per user after typing `yes`, kept in `platformdirs.user_data_dir("naoqi-viewer")`; the compiled model with them exists only in memory |

- The placeholder visuals are our own work and are never derived from the meshes (no tracing or fitting), so they stay MIT ([model.md](model.md)).
- `.gitignore` blocks mesh, texture, installer and archive files. Whenever the meshes are visible, the window shows an attribution to Aldebaran with the license name ([viewer.md](viewer.md)).

## NAOqi versions

- **NAOqi 2.1.4.13 (NAO V5)** is the validated target: `check-model` passes against it, and CI's live tier runs against naoqi-sim's 2.1 image.
- **NAOqi 2.8 (NAO V6)** is meant to work through the same qi calls, but is not validated yet. The model is V5 (H25) geometry; whether V6 needs its own URDF and camera field of view is open ([model.md](model.md)).
- Host code uses `qi.Session` and `session.service()` from the libqi 3 wheels. From 3.1.8 their connects succeed at the first attempt (3.1.6's failed some with an instant `disconnected` and needed retries), so `connect()` makes one, which `cancel` can give up ([source.md](source.md)).

## CLI

Specified in [cli.md](cli.md).

| Command | Purpose |
| --- | --- |
| `naoqi-viewer view [--config FILE]` | Open a viewer from a config file (default: mirror mode on a local NAOqi, empty scene) and wait until it closes |
| `naoqi-viewer check-model URL [...]` | Forward-kinematics check against a NAOqi; exit 0 within tolerance |
| `naoqi-viewer fetch-meshes [--archive PATH] [--force] [--remove]` | License prompt and local mesh install, or its removal ([meshes.md](meshes.md)) |

Ready-made config files live in `examples/configs/` (`mirror.json`, `sim-table.json`, `sim-headless.json`).

## Packaging and platforms

- Python 3.12–3.13, packaged with uv, `src/` layout ([project.md](project.md)).
- Platforms are those the libqi wheels cover: macOS arm64, Linux x86_64 and Windows x64, with `qi` from the libqi fork's package index ([project.md](project.md)). On macOS a window needs `mjpython`, which `launch()` picks itself; a headless viewer renders offscreen on all three (CGL on macOS, EGL on Linux, GLFW's hidden window on Windows).
- CI runs the fast tier on all three platforms, and everything else on Linux: GitHub's macOS runners have no OpenGL, so the macOS window and renders are tested locally, and the Windows runner renders through Mesa ([ci.md](ci.md)).

## Open questions

Cross-cutting ones only; each concept spec keeps its own.

1. **NAOqi 2.8 / NAO V6**: validate the model and `check-model` against 2.8, and decide whether CI runs it ([model.md](model.md), [ci.md](ci.md)).
