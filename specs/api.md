---
code:
  - src/naoqi_viewer/__init__.py
  - src/naoqi_viewer/client.py
  - src/naoqi_viewer/protocol.py
  - src/naoqi_viewer/viewer_process.py
  - src/naoqi_viewer/scenes/table.xml
tests:
  - tests/test_client.py
  - tests/test_protocol.py
  - tests/test_viewer_process.py
  - tests/fake_viewer_process.py
  - tests-e2e/test_viewer_live.py
---

# Viewer API

**Status:** Implemented

## Purpose

The public interface of naoqi-viewer: a `NaoqiViewer` object, built from a configuration ([config.md](config.md)), that opens a viewer on a NAO and controls it. Each viewer runs in its own process. MuJoCo's window, OpenGL and the macOS `mjpython` requirement stay inside that process, so the caller only sees a plain Python object.

## Decided

### Two modes

The mode is the `mode` field of the config, at its top level, because it changes what the viewer does:

| | `"mode": "mirror"` | `"mode": "sim"` |
|---|---|---|
| For | Watching any NAO, typically a real robot | naoqi-sim's simulated world (or a plain virtual robot) |
| Pose | From NAOqi | From NAOqi |
| `camera_frame` | Refused with `ModeError`. A real robot has its own cameras, served by `ALVideoDevice`. | Renders what the head cameras see in the simulated world |
| Props ([props.md](props.md)) | Refused with `ModeError`: a real robot's cameras cannot see a MuJoCo body | A pool of faces and objects appended to the scene, spawned, moved and hidden through `viewer.props` |
| Camera display | The robot's cameras, read from `ALVideoDevice` ([cameras.md](cameras.md)) | The same reader: the frames naoqi-sim injected into `ALVideoDevice`, this viewer's renders after their round trip |
| Window title | `MuJoCo : naoqi-viewer · mirror · <url>` | `MuJoCo : naoqi-viewer · sim · <url>` |

Both modes run the same [viewer loop](viewer.md) and [pose source](source.md). The mode only sets the defaults above and which requests the viewer process accepts. Sim mode does not refuse a real robot as its target, but the frames would show the simulated scene, not the robot's surroundings.

### Headless

`"headless": true` in the config ([config.md](config.md), sim mode only) runs the viewer process without a window. It serves the same requests (`status`, `camera_frame`) from offscreen renders. The point is that a caller does not need a display: naoqi-sim's CI, or anyone else's, runs the simulated world on a generic Linux image with no X server, virtual display or GPU.

- **No window**: the viewer process runs the headless loop ([viewer.md](viewer.md)) instead of the passive viewer: same pose updates at 60 Hz, same request handling, but no overlay, keys or ghost (config.md rejects `ghost` with `headless`). The public API is unchanged: `launch()`, `camera_frame`, `status`, `wait` and `close` behave as for a windowed viewer.
- **Lifetime**: there is no window to close, so a headless viewer exits on `close()`, when the caller's control connection closes (including the caller exiting), on `stop`, or when no caller connects within `launch_timeout_s`.
- **No `mjpython`**: `<python>` is `sys.executable` on every platform. Offscreen rendering does not need the macOS main-thread window rule (macOS renders through CGL).
- **OpenGL backend**: MuJoCo reads `MUJOCO_GL` when it is imported. For a headless viewer on Linux with `MUJOCO_GL` unset, `launch()` sets `MUJOCO_GL=egl` in the viewer process's environment. Without a GPU, Mesa's EGL falls back to its software device (llvmpipe), so it needs no display and no GPU, only three packages on Debian/Ubuntu: `libegl1 libopengl0 libgl1-mesa-dri`. PyOpenGL needs `libopengl0` (or `libgl1`) to load at all. GitHub's runners already have it, but slim container images don't. A caller's own `MUJOCO_GL` wins: `osmesa` (with `libosmesa6`) also works, more slowly. Checked on `ubuntu:24.04` and `python:3.12-slim` (x86_64, no GPU). On macOS, MuJoCo's default backend renders offscreen with no window (CGL). On Windows, MuJoCo's only backend is GLFW, which renders offscreen into a hidden window, so `launch()` leaves `MUJOCO_GL` alone there too. It needs OpenGL 2 or later: a GPU driver gives it; without one (a VM, CI's runners), Windows' own `opengl32.dll` is OpenGL 1.1, and Mesa's llvmpipe takes its place when its `opengl32.dll` and `libgallium_wgl.dll` sit next to the base `python.exe`, as CI does ([ci.md](ci.md)).
- **Ready means it can render**: before its ready line, a headless viewer process renders one frame from `CameraTop` at 640×480 (a common request size, so that renderer is kept). If that fails (Mesa's EGL not installed, a backend the machine lacks), it prints `NAOQI_VIEWER_ERROR` naming the backend and how to fix it on Linux and on Windows. A `MUJOCO_GL` value MuJoCo doesn't know already fails when `mujoco` is imported; that error reaches `LaunchError` through the stderr tail. `launch()` then raises `LaunchError` instead of the first `camera_frame` failing.

### Public API (`naoqi_viewer`)

```python
from naoqi_viewer import NaoqiViewer, NaoqiViewerConfig

config = NaoqiViewerConfig.from_json_file("examples/configs/sim-table.json")
with NaoqiViewer(config) as viewer:                    # launch() on enter, close() on exit
    frame = viewer.camera_frame("top", 640, 480)     # RGB numpy image
    print(viewer.status())
    viewer.wait()                                    # until the window is closed

viewer = NaoqiViewer()                                 # the default config: mirror, local NAOqi, empty scene
viewer.launch()
...
viewer.close()
```

```python
class NaoqiViewer:
    def __init__(self, config: NaoqiViewerConfig | None = None) -> None    # cheap: no process, no window
    @classmethod
    def from_dict(cls, data) -> NaoqiViewer                 # NaoqiViewer(NaoqiViewerConfig.from_dict(data))
    @classmethod
    def from_json(cls, text: str) -> NaoqiViewer
    @classmethod
    def from_json_file(cls, path: str | Path) -> NaoqiViewer
    @property
    def config(self) -> NaoqiViewerConfig                   # read-only

    def launch(self) -> None                              # start the viewer process; returns once the window is up
    def camera_frame(self, camera: Literal["top", "bottom"], width: int, height: int) -> CameraFrame
    @property
    def props(self) -> PropsClient                       # sim only: list, spawn, place, despawn, clear (props.md)
    def status(self) -> ViewerStatus
    def wait(self, timeout: float | None = None) -> bool  # True once the viewer has exited (or was never launched)
    @property
    def running(self) -> bool
    def close(self) -> None
    # context manager: __enter__ launches (unless running), __exit__ closes

@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray        # (height, width, 3) uint8 RGB, row 0 = top of the image
    camera: str
    pose_seq: int            # Sample.seq the frame was rendered at (0: no pose yet)
    pose_age: float | None   # data age of that pose, s (None: no pose yet)

@dataclass(frozen=True)
class ViewerStatus:
    mode: str                # mirror / sim
    target: str | None       # real / naoqi-sim / virtual (source.md); None until NAOqi first connects
    naoqi_version: str | None
    url: str
    variant: str             # placeholder / aldebaran (model.md)
    rate: float              # pose updates per second
    pose_seq: int
    data_age: float | None   # s; None before the first sample

class LaunchError(RuntimeError): ... # the viewer process couldn't start (message + its stderr tail)
class ViewerClosed(Exception): ...   # the viewer isn't running: never launched, closed, or exited
class ModeError(Exception): ...      # the request isn't available in this mode
class PropError(ValueError): ...     # a props call refused: unknown prop, prop in use, no free prop, bad value (props.md)
```

- **Building and launching are separate.** The constructor only keeps the config: it has no side effects and can't fail on the machine (a bad config fails earlier, as a `ConfigError`, when it is built). `launch()` does the work: it starts the viewer process and opens the window, and is the one place that raises `LaunchError`.
- **One `NaoqiViewer` is one window.** `launch()` while the viewer is running raises `RuntimeError`; two windows are two `NaoqiViewer` objects. Once the viewer has exited (window closed, `close()`), `launch()` opens a new window with the same config.
- **Calls need a running viewer.** `camera_frame` and `status` before `launch()`, after `close()`, or after the window was closed raise `ViewerClosed`. `close()` is harmless in any state.
- The scene is `config.world.scene`: a bundled scene name (`"empty"`, `"table"`) or a path to a user MJCF file ([model.md](model.md), Scenes). The default, in both modes, is `"empty"`: NAO alone on a floor. A caller that wants something in front of the cameras sets a scene, for example `"table"`.
- `launch()` returns once the viewer is up, even if NAOqi is unreachable. The viewer keeps reconnecting ([source.md](source.md)), and `status().data_age` stays `None` until poses arrive. Errors only the viewer process can detect (a missing scene file, `variant: "aldebaran"` without installed meshes, no display, no offscreen OpenGL for a headless viewer) are reported by the viewer process and raised from `launch()` as `LaunchError`.
- `camera_frame` is checked on the client side first, so mirror mode raises `ModeError` without a round trip; the viewer process refuses it too. In sim mode it blocks until the frame is rendered, which takes about one frame (17 ms) plus render time. Calls from several threads are serialized.
- `props` ([props.md](props.md)) is a `PropsClient` whose five methods are each one request, answered in the same place as `camera_frame` and serialized with it; mirror mode raises `ModeError` on the client side. The state it returns is the commanded one, drawn by the next frame.
- `naoqi_viewer/__init__.py` exports exactly: `NaoqiViewer`, `NaoqiViewerConfig`, `NaoqiSettings`, `WorldSettings`, `ConfigError` ([config.md](config.md)), `CameraFrame`, `ViewerStatus`, `PropState`, `PropsClient`, `LaunchError`, `ViewerClosed`, `ModeError`, `PropError`. Importing it loads neither `mujoco` nor `qi`: `client.py` and `config.py` use the standard library and `numpy` only, so importing naoqi-viewer costs naoqi-sim nothing.

### Viewer process (`viewer_process.py`)

Two processes are involved. The **caller's process** is the program that calls `NaoqiViewer.launch()`, for example naoqi-sim's host program. It imports only `client.py`, `config.py` and `protocol.py`. The **viewer process** is a separate operating-system process that `launch()` starts. It runs `viewer_process.py`, which loads MuJoCo, connects to NAOqi and owns the window. With a viewer open, `ps` shows both.

- `launch()` starts `<python> -m naoqi_viewer.viewer_process <config JSON>` as a subprocess, where the config JSON is `config.to_dict()` ([config.md](config.md)); the viewer process rebuilds the `NaoqiViewerConfig` with `from_dict`, so both processes read one format:
  - `<python>` is `mjpython` next to `sys.executable` on macOS, and `sys.executable` elsewhere (Linux, Windows) and for a headless viewer (Headless, above).
  - Its environment is the caller's with `PYTHONIOENCODING=utf-8` (and `MUJOCO_GL=egl` for a headless viewer on Linux, above), and `launch()` reads its stdout and stderr as UTF-8, undecodable bytes replaced: the protocol lines and error messages carry paths, which Windows' legacy code pages can't always print, and libqi's own log lines are not ours to trust.
  - If `mjpython` is missing, `launch()` raises `LaunchError` saying why.
- The viewer process loads the world ([model.md](model.md); in sim mode with the props pool, [props.md](props.md)), builds a `NaoqiSource` ([source.md](source.md)) and, with `cameras_display` on, a `CameraSource`, with `people_display` on, a `PeopleSource` ([cameras.md](cameras.md), [people.md](people.md)), all on the same URL, then runs the [viewer](viewer.md) loop: with a window, or the headless loop when the config says `headless`. Every source is closed when the loop ends.
- It binds a loopback port, and once the window has drawn its first frame (headless: once its first offscreen frame has rendered) prints `NAOQI_VIEWER_READY port=<n> protocol=2` on stdout; on an error before that (unknown scene, missing meshes) it prints `NAOQI_VIEWER_ERROR <message>` and exits 1. Waiting for the first frame means `launch()` returns with the window actually open, and a window that can't open (no display) is a launch failure. `launch()` waits for that line up to `launch_timeout_s` (other stdout lines, such as libqi's own log lines, which it writes to stdout, are relayed at `DEBUG`), then connects and sends `hello`. On a timeout, an early exit or `NAOQI_VIEWER_ERROR`, `launch()` kills the viewer process if needed and raises `LaunchError` with the tail of its stderr.
- **Lifetime is tied to the caller**: the viewer process exits when its control connection closes. That covers `close()`, and the caller's process exiting or crashing, so a viewer window is never orphaned. It also exits if no connection arrives within `launch_timeout_s` of its ready line (a caller that gave up). Closing the window (if there is one) also ends the viewer process; the next call raises `ViewerClosed`, `wait()` returns, and `launch()` may open a new window.
- A server thread reads requests and queues them; the render loop answers them all through the viewer's `on_frame` hook ([viewer.md](viewer.md)), between frames. Camera renders must happen there, on the thread that owns the renderers' OpenGL contexts, and answering every op in one place keeps the state consistent; a reply waits at most one frame (17 ms). In sim mode the hook first steps the props ([props.md](props.md)), so a frame rendered in that tick shows them where their moves put them. There is one `mujoco.Renderer` per requested resolution, using the model cameras `CameraTop`/`CameraBottom`, with the model's offscreen buffer enlarged when a request needs it. A frame reports the `seq` and age of the sample the model was posed with. In mirror mode no renderer is created, and `camera_frame` is refused with the error `mode`.
- `close()` sends `stop`, waits up to 5 s, then terminates the viewer process.
- **Viewer-process logs reach the caller's logging**: after the ready line, the viewer process logs to stderr, one record per line, with its level in the line. A reader thread in `client.py` drains stderr and passes each line to the `naoqi_viewer.viewer_process` logger at that level. Lines without a level (a native crash, MuJoCo's own prints) go out at `DEBUG`. A working viewer is quiet in a default log, and a problem inside it shows up in the caller's log without extra setup. The reader also keeps the last 50 lines, which `launch()` includes in its error when the viewer process fails to start.

### Protocol (`protocol.py`)

- One TCP connection on `127.0.0.1`. Every message is `u32 header_len | u32 payload_len | header (UTF-8 JSON) | payload (raw bytes)`, with big-endian lengths, the same style as naoqi-sim's host link.
- Requests are `{"op": ..., "id": n, ...}`. Responses echo `id`, with `"ok": true` and the result fields, or `"ok": false, "error": "..."`.
- Ops in protocol 2 (protocol 1 had the first four):
  - `hello` → `{"protocol": 2, "naoqi_viewer": version, "mode": mode}`
  - `status` → the `ViewerStatus` fields
  - `camera_frame {camera, width, height}` (sim only) → `{shape, dtype, camera, pose_seq, pose_age}` plus RGB bytes
  - `stop` → `{}`, then the viewer process exits
  - `props`, `prop_spawn`, `prop_place`, `prop_despawn`, `props_clear` (sim only) → a prop's state or the list of them, as [props.md](props.md) specifies
- Unknown ops are answered with `ok: false` without closing the connection, so a newer client can probe an older viewer process.
- Encoding and decoding are shared by both sides, and they are the only code `client.py` and `viewer_process.py` have in common.

### Bundled scenes

`src/naoqi_viewer/scenes/` ships with the package:

- `empty.xml` (`"empty"`): floor and light, the default in both modes, described in [model.md](model.md);
- `table.xml` (`"table"`): a low table with a few objects in front of the robot, placed so both head cameras see them; for sim-mode callers who want something to look at.

In sim mode every scene, bundled or a user's file, also carries the props pool ([props.md](props.md)): hidden bodies a caller shows and moves while the viewer runs.

## Open questions

1. **Streaming**: pull (one request per frame) is the v1 decision. If naoqi-sim needs two cameras at 30 fps, per-request latency or the copy cost (0.9 MB per VGA frame over loopback) may matter. Then add a `camera_stream` op that pushes frames at a requested rate, or shared memory for the pixels. To decide with measurements.
2. **Colorspace**: RGB out keeps the viewer generic, and naoqi-sim converts to the NAO format. If that conversion is costly, `camera_frame` could take a target colorspace.
3. **Further ops**: touch events from clicks on the robot (for `ALTouch`/awareness), a new op with the protocol version raised. Human figures in the scene are the props of [props.md](props.md) (protocol 2); publishing them to NAOqi is not the viewer's.
