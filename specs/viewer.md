---
code:
  - src/naoqi_viewer/viewer.py
tests:
  - tests/test_viewer.py
---

# Viewer

**Status:** Implemented

## Purpose

The window: it poses the model from a [pose source](source.md) and draws it, with no physics. It runs inside the viewer process ([api.md](api.md)). The mode ([api.md](api.md)) and the scene are parameters; the loop is the same.

## Decided

### Window stack: `mujoco.viewer.launch_passive`

- The built-in passive viewer gives camera controls, rendering options, text overlays and `user_scn` (used for the ghost) for free.
- On macOS, `launch_passive` must run under `mjpython`. [api.md](api.md) starts the viewer process with it, and `viewer.py` assumes it is already there.
- Both side panels start hidden; the standard MuJoCo key toggles them.
- The default camera tracks the `torso` body, so the robot stays in view when it walks. Double-click or the usual MuJoCo keys switch to a free camera.
- **Window title**: the passive viewer has no title argument; it titles its window `MuJoCo : <model name>`. The title is therefore set when the world is compiled, through `load_world(..., name=...)` ([model.md](model.md)); [api.md](api.md) passes `naoqi-viewer · <mode> · <url>`.
- **Keys**: every letter and digit is already a MuJoCo viewer shortcut, and keys reach both MuJoCo and naoqi-viewer's `key_callback`, so naoqi-viewer only uses function keys MuJoCo leaves free (F8 and up).

### Loop

`run(model, source, *, ghost=False, attribution=None, on_frame=None, stop=None)` runs until the window closes or `stop` (a `threading.Event`) is set:

1. `sample = source.latest()`; if its `seq` is new, `PoseWriter.apply` writes it into `data` (which runs `mj_kinematics` and `mj_camlight`).
2. Update the ghost and the overlay.
3. Call `on_frame(model, data, sample)` if given, where `sample` is the sample the model is posed with (`None` before the first). The viewer process ([api.md](api.md)) uses this hook to serve camera renders between frames, on the thread that owns the window.
4. `handle.sync()`, then sleep to the next 1/60 s tick (ticks on a fixed schedule, as in [source.md](source.md)).

Steps 1 and 2 live in a `ViewerState` class (posing, overlay texts, ghost geoms, key handling) that works without a window: the fast tests drive it offscreen with a fake `PoseSource` and an `MjvScene`. `run` only wires it to `launch_passive`, and is tested in the live tier.

`mj_step` is never called: the pose comes only from the source.

### Headless loop

`run_headless(model, source, *, on_frame=None, stop=None)` is the loop of a headless viewer ([api.md](api.md), Headless). It runs until `stop` is set, which is the only way out because there is no window. Each tick it calls `ViewerState.update()` (step 1), then `on_frame` (step 3), then sleeps to the next 1/60 s tick (step 4, without `sync`). There is no overlay and no ghost (step 2), and no key handling. Everything else is shared with `run` through `ViewerState`, so a pose reaches a camera frame the same way in both loops.

`viewer.py` imports `mujoco.viewer` (and through it GLFW) inside `run` only, so a headless viewer process never loads the window stack.

### Status overlay (top-left)

- Lines: target and URL (`naoqi-sim tcp://127.0.0.1:9559`), NAOqi version, update rate (`source.rate()`, Hz), and data age in ms. Before the first connection (`source.info` is `None`) the first line reads `connecting tcp://…`.
- When the data age passes 500 ms, or before the first sample, the data-age line reads `STALE` (or `NO DATA`). The robot stays in its last pose.
- Shown with `handle.set_texts`, which takes plain text with no per-line color, so the `STALE` text is the stale marker. `F9` toggles the overlay.

### Ghost (`ghost=True`)

**What it is for.** NAOqi reports two sets of joint angles: the *measured* ones, read from the joint sensors (`getAngles(..., True)`), which the viewer always draws, and the *commanded* ones, where the motors were told to go (`getAngles(..., False)`). The ghost is a translucent copy of the robot in the commanded pose, drawn over the measured one. Where the two agree the robot is doing what it was asked, and no ghost shows. Where they differ, the ghost shows where a part is trying to go, and the robot where it actually is. That points at:

- a limb blocked by an obstacle or by the robot's own body;
- joints with stiffness off, which don't follow their commands;
- a joint lagging behind during a fast movement;
- a weak or overheating motor that can't hold its target.

It is off by default: it costs one more NAOqi call per poll, and when nothing is wrong it shows nothing. It is turned on with `"ghost": true` in the viewer's config ([config.md](config.md)), for `NaoqiViewer` ([api.md](api.md)) as for `naoqi-viewer view --config` ([cli.md](cli.md)).

**How it is drawn.**

- Needs a source built with `commanded=True`. A second `MjData` is posed from `sample.commanded` with the same torso pose.
- Its geoms are added to `handle.user_scn` each frame with `mjv_addGeoms`, limited to the robot's visual geoms (group 1, dynamic bodies), recolored translucent (alpha 0.3, one flat color) so the gap between commanded and measured angles shows at a glance. Only bodies whose commanded pose is off from the measured one (by more than 1 mm or about 0.5°) are drawn: a ghost laid exactly over the robot would only tint it. A sample without `commanded` draws no ghost.

### Attribution overlay (bottom-right)

- Text: `NAO meshes © Aldebaran, CC BY-NC-ND 4.0`.
- Shown whenever the world is loaded with Aldebaran's meshes (`variant` `aldebaran`, [model.md](model.md)), passed as `attribution`; never shown with the placeholder visuals. It has no toggle: the license asks for attribution, and it takes little room.
- naoqi-viewer records no video in v1, so there is no recording case to handle.

## Open questions

1. **Touch input**: raising touch events for awareness by clicking on the robot (e.g. the head). That's a future [api.md](api.md) op, deferred.
