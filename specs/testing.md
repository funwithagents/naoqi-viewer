---
code:
  - tests/conftest.py
  - tests/mock_naoqi.py
  - tests-e2e/conftest.py
  - tests-e2e/support.py
tests:
  - tests/test_mock_naoqi.py
---

# Testing

**Status:** Implemented

## Purpose

naoqi-viewer's testing strategy — the two-tier structure and what a good test looks like. It's a **cross-cutting practice**, not a runtime concept: nothing here ships in the library. It exists as a spec so the decisions have one honest home that stays in sync with the setup, rather than living half in [project.md](project.md) (the tooling choices) and half in [AGENTS.md](../AGENTS.md) (the operational how-to). The concrete shell commands to run each tier live in [AGENTS.md](../AGENTS.md) "Testing".

## Two tiers, physically separated

Tests split into two directories, and the split is structural — a directory boundary, not a marker or an opt-out flag:

| Tier | Directory | Network | Deterministic | Runs by default |
|---|---|---|---|---|
| Unit / integration | `tests/` | never | yes | **yes** |
| Live / e2e | `tests-e2e/` | a running NAOqi, or a real mesh install | no | **no** |

- **`tests/` is the normal dev loop.** Fast, deterministic, no real network, no credentials. `pyproject.toml`'s `testpaths = ["tests"]` points the default `uv run pytest` here, so this is what runs on every change and what any contributor or CI can run with zero credentials.
- **`tests-e2e/` is opt-in.** It calls a real external service — network, credentials, non-deterministic output — so it is deliberately *not* collected by the default run. Because `testpaths` already excludes it, no pytest marker or `--run-e2e` flag is needed: the physical separation is the whole mechanism. Run it explicitly (`uv run pytest tests-e2e`).

**What each tier talks to.** `tests/` never talks to Aldebaran software. Code that needs NAOqi is tested against a **mock NAOqi**: Python 3 qi services (`ALMotion` returning scripted angles and transforms, and so on) registered in a standalone `qi.Session` that listens on a loopback port inside the test process. That is still deterministic and offline. `tests-e2e/` talks to a **live NAOqi endpoint** given by URL: a real robot, or a naoqi-sim container someone started separately. It also holds the end-to-end test of Aldebaran's meshes, which needs no NAOqi but a real install made by `fetch-meshes` (see "E2e tier: Aldebaran's meshes" below). The e2e tier never imports naoqi-sim, so testing against the container adds no package dependency and no cycle (naoqi-sim → naoqi-viewer stays one-way). The dependency is a running process, not an import.

The `tests/` tier mirrors the `src/naoqi_viewer/` module layout (`test_<module>.py`, plus the `test_project_map.py` drift-guard); `tests-e2e/` is organized around live scenarios rather than modules.

## What a good test asserts

- **Functional, not tautological.** Exercise what a feature actually does — inputs → outputs, state changes, side effects — not that it runs or matches its own signature. A test that would pass against a broken implementation (asserting a constant, that an object isn't `None`, that a mock was called) isn't worth writing.
- **Drive the public API like a real caller.** Prefer exercising the public surface the way a consumer would over reaching into internals; assert on the observable result.
- **In the e2e tier, assert on behavior, not exact output.** Real service responses vary run to run, so a live test asserts a robust property ("a non-empty result came back", "the side effect happened"), never a specific string.

## Mock NAOqi (`tests/mock_naoqi.py`)

naoqi-viewer owns a small mock rather than borrowing one from naoqi-bridge, which sits upstream of naoqi-sim and so of naoqi-viewer. It implements only what naoqi-viewer calls.

- **Transport**: a real `qi.Session` listening standalone on `tcp://127.0.0.1:<free port>` inside the test process, with Python objects registered as services. Code under test connects to it by URL, through the same `connect()` it uses against a robot, so the qi path is exercised for real. No Aldebaran software is involved.
- **Services**:
  - `ALMotion`: `getBodyNames`, `getAngles(names, useSensors)` (measured and commanded kept separately, so the ghost can be tested), `getTransform(name, frame, useSensors)`, `setStiffnesses`, `angleInterpolation`.
  - `ALMemory`: `getData`, `insertData`.
  - Optional `NaoqiSim` and `ALSystem` services, so all three target identifications can be tested.
- **Effector transforms**: `getTransform` for effectors comes from the forward kinematics of the committed model (`nao.xml`). A test can add a fixed error to one effector, so `check-model` is tested both passing and failing.
- **Control from tests**: a pytest fixture (`mock_naoqi`) yields `(url, mock)` for a virtual robot; other targets are built with `MockNaoqi(target=...)`. The test sets the pose (`mock.set_pose(...)`), can make calls fail (`mock.fail_calls(...)`), drop the session (`mock.disconnect()`) and serve again on the same port (`mock.restart()`) to exercise reconnection, and can read the calls received (`mock.calls_to(...)`).
- **Verified transport**: the libqi 3.1.6 fork serves plain Python objects from a standalone session, a missing service raises `RuntimeError`, closing the server disconnects clients, and a restarted server rebinds the same port at once.

## Testing the viewer process

A windowed viewer process (a MuJoCo window, OpenGL, a subprocess) only runs in the live tier. The fast tier tests both sides of the protocol without it, and runs one real headless viewer process:

- **Client side (`tests/`)**: `client.py` is tested against a **fake viewer process** (`tests/fake_viewer_process.py`): a small script speaking the protocol with canned frames and scripted failures, which `NaoqiViewer.launch()` starts as a real subprocess in place of `viewer_process` (tests swap the command it runs). Being a real subprocess, it exercises the ready line, stderr relaying, early exits and timeouts as well as the handshake, message encoding, `ModeError` on the client side, `ViewerClosed`, and calls from several threads.
- **Viewer-process side (`tests/`)**: the viewer process's request handling (queueing, mirror refusing `camera_frame`, `status`) is called directly with the render function stubbed. No subprocess and no OpenGL.
- **Real headless viewer (`tests/`)**: a headless sim `NaoqiViewer` on the mock NAOqi starts the real viewer process (no fake), waits for a pose, and checks that the frames it returns aren't blank and change when the mock's head turns. It needs no display, but it does need an offscreen OpenGL backend (Mesa's EGL on Linux, see [api.md](api.md), Headless). A session fixture checks for one in a subprocess, with the environment `launch()` would give the viewer, and **skips** these tests when there is none. A plain Linux image without `libegl1 libopengl0 libgl1-mesa-dri` therefore skips them instead of failing. With `NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL` set (CI sets it, [ci.md](ci.md)), this skip and the in-process renderer test's skip become failures.
- **Live (`tests-e2e/`)**: a `NaoqiViewer` in sim mode on `NAOQI_URL` starts the real viewer process. The test fetches real frames, checks they aren't blank, and checks that they change when the head moves. Whether the viewers the run opens are headless or windowed is the run's choice, not the test's (see "Live tier: headless or windowed" below). The tests' own qi sessions (to move the head) go through `naoqi_viewer.source.connect`, which retries as the viewer does, because the libqi 3 wheels fail some connects at once.

## Test isolation

If the package holds process-global or singleton state, both tiers carry an identical autouse fixture (in each tier's `conftest.py`) that resets it before and after every test, so no state — or background timers/threads — leaks across tests. The fixture is duplicated rather than shared because `tests-e2e/` isn't a package that imports from `tests/`, and it's only a few lines.

**Aldebaran's meshes.** A developer may have the real meshes installed ([meshes.md](meshes.md)), and they must change no test's result:

- Every test that builds or renders a model names its variant (`variant="placeholder"`, or `"auto"`/`"aldebaran"` where that is what it tests). A viewer started as a subprocess gets it from its config.
- As a safety net, `tests/conftest.py` points the meshes folder at an empty temporary folder for the whole session. A default (`"auto"`) then always resolves to the placeholder, and no test reads or writes the real install. Mesh tests install a test-built release of their own.
- In the e2e tier, only `test_meshes_live.py` uses the real meshes. The live viewer tests name `"placeholder"`.

## Live tier: skip without an endpoint

A live test needs a NAOqi to talk to, and it must **skip, never fail**, when none is configured. That way anyone without a robot or a container, including CI, can run the tier without breaking it. The endpoint comes from the `NAOQI_URL` environment variable (for example `tcp://127.0.0.1:9559` for naoqi-sim, `tcp://<robot>:9559` for a real NAO). `tests-e2e/support.require_env(NAME)` returns the variable or calls `pytest.skip(...)` when it is unset. Nothing about the endpoint is committed.

## Live tier: headless or windowed

One run opens its viewers one way, chosen by an environment variable, as reachy-mini-bridge's live tier does:

| Run | Command | Viewers |
|---|---|---|
| headless (default) | `NAOQI_URL=… uv run pytest tests-e2e -rs` | sim tests run headless and need no display, which is what CI runs. The mirror test **skips**, because mirror mode has no headless form. |
| windowed | `NAOQI_VIEWER_E2E_WINDOW=1 NAOQI_URL=… uv run pytest tests-e2e -rs` | every viewer opens its window (under `mjpython` on macOS), the mirror test included. The run needs a display and an unlocked session. |

- `support.window_requested()` reads `NAOQI_VIEWER_E2E_WINDOW`: `1`, `true`, `yes` or `on` (any case) mean windowed, and anything else, including unset, means headless. `support.require_window()` skips a test that only exists with a window, naming the variable.
- **A windowed run without a display fails, it does not skip.** The run asked for windows, so a window that can't open is a real failure.
- The live tier talks to a real NAOqi, never to the mock. The fast tier already runs a real headless viewer against the mock, and `check-model` against the mock would compare the model with itself: the mock computes its effectors from `nao.xml`. What only the live tier checks is the real NAOqi API, target identification, the model's kinematics, and the window.

## E2e tier: Aldebaran's meshes

`tests-e2e/test_meshes_live.py` loads the world with `variant="aldebaran"` from the meshes `fetch-meshes` installed, and renders it offscreen ([meshes.md](meshes.md), Testing). It needs neither `NAOQI_URL` nor a window, and **skips** when no complete install is found. With `NAOQI_VIEWER_REQUIRE_MESHES` set, the skip becomes a failure, as `NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL` does for the fast tier. CI's `meshes` job sets it ([ci.md](ci.md)). The fast tier never touches an Aldebaran file: its mesh tests generate Collada files of their own at test time.

## Tooling

- **`pytest`** is the runner; **`ruff`** lints/formats; **`pyright`** (`standard` mode) type-checks. All three are the gate after any change — lint, type check, and tests must pass before work is considered done (see [AGENTS.md](../AGENTS.md), "Verification").
- **`pyright` covers test code too:** its `include` is `src`, `tests`, and `tests-e2e`, so tests are type-checked alongside the library rather than being a blind spot.

## Open questions

None currently. CI, which runs both tiers on GitHub's runners, is [ci.md](ci.md).
