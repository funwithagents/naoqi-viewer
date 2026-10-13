# naoqi-viewer

naoqi-viewer is the MuJoCo window for NAO, one of the three packages of the NAO toolkit (naoqi-bridge, naoqi-sim, naoqi-viewer). Its core idea is a kinematic mirror, not a physics simulation: it reads joint angles and the torso pose from any NAOqi endpoint over qi, writes them into the model's `qpos`, runs `mj_kinematics` and renders. It is a Python library: a `NaoqiViewer`, built from a JSON-friendly `NaoqiViewerConfig`, starts the viewer in its own process with `launch()` (where MuJoCo's window and `mjpython` live) and controls it. The same viewer watches a real robot, or acts as naoqi-sim's simulated world, where naoqi-sim passes a scene and pulls head-camera frames to inject into `ALVideoDevice`. naoqi-viewer also owns the NAO model: one MJCF, converted once from the BSD-3 URDF and committed, ships in the package with styled placeholder visuals of our own. Aldebaran's meshes replace those visuals only on the user's machine, after a license-gated `fetch-meshes`, because they must never enter the repository or a package. naoqi-viewer depends on libqi alone, never on naoqi-bridge or naoqi-sim, so the dependency chain stays one-way.

The big picture (goals, scope, architecture, status, licensing) is in [_overview.md](_overview.md).

## Specs

<!-- One row per concept spec. Keep the Status column in sync with each spec's `**Status:**` line. -->

| Spec | Description | Status |
|---|---|---|
| [project.md](project.md) | Project structure and tooling: Python version, packaging with uv, `qi` from the libqi fork's package index, platforms (macOS arm64, Linux x86_64, Windows x64), layout conventions | Updated |
| [testing.md](testing.md) | Testing strategy: two-tier `tests/`/`tests-e2e/` split, in-process mock NAOqi, a real headless viewer in the fast tier, live tier gated on `NAOQI_URL` and headless unless `NAOQI_VIEWER_E2E_WINDOW`; the platform skips | Updated |
| [model.md](model.md) | The NAO MJCF model: committed `nao.xml` from the URDF, joints, couplings, sensors, cameras, effector sites, placeholder visuals; scenes; `NaoPose` and `PoseWriter` | Implemented |
| [meshes.md](meshes.md) | `fetch-meshes`: license prompt, then naoqi-meshes' encrypted OBJ/PNG archive unlocked locally, and its removal; and loading Aldebaran's meshes in place of the placeholder visuals | Updated |
| [source.md](source.md) | Pose sources: `connect()` in one attempt that `cancel` can give up, `NaoqiSource` polling NAOqi at 50 Hz with reconnect and target identification | Updated |
| [viewer.md](viewer.md) | The render loop on `mujoco.viewer.launch_passive`: pose → `mj_kinematics` at 60 Hz, status overlay, ghost, attribution; the windowless headless loop | Implemented |
| [config.md](config.md) | `NaoqiViewerConfig`: mode, headless, NAOqi, world, ghost; `from_dict`/`from_json`/`from_json_file`, `to_dict`, `ConfigError` naming the key; example files | Implemented |
| [api.md](api.md) | Public API: `NaoqiViewer(config)` with `launch()` starting the viewer in its own process (`mjpython` on macOS), `camera_frame`, `status`, `wait`, `close`; headless sim with offscreen rendering (Mesa's EGL on Linux, GLFW on Windows); UTF-8 pipes; loopback protocol; bundled scenes | Updated |
| [check_model.md](check_model.md) | `check-model`: forward-kinematics cross-check against NAOqi, 2 mm / 1° | Implemented |
| [ci.md](ci.md) | Continuous integration: `check`, `fast-tier`, `meshes` and `e2e-naoqi-sim` on GitHub's runners: the fast tier on Linux, macOS and Windows (Mesa's llvmpipe), the live tier headless and windowed against a real NAOqi 2.1 built by naoqi-sim at a pinned commit, its images cached | Updated |
| [cli.md](cli.md) | Thin `naoqi-viewer` command: `view --config`, `check-model`, `fetch-meshes` | Implemented |

Each spec also opens with a YAML **frontmatter** block declaring the `code:` and `tests:` files it governs — the spec → code/tests mapping the spec-drift checks use to scope what they compare. Keep it current when files move, and see [AGENTS.md](../AGENTS.md) ("Spec frontmatter") for the full convention.

### Status legend

- **Not started** — no design decisions made yet
- **Draft** — actively being brainstormed/defined, contains open questions
- **Stable** — design settled, reviewed and validated (open questions are deferrals only), **ready to implement but not necessarily implemented yet**. This is the design-review gate, before code is written.
- **Implemented** — a **Stable** spec that a `Done` plan has built: the code now exists and matches the spec (design and code in sync)
- **Updated** — an **Implemented** spec since edited in a way that needs new code, so the code no longer matches it; a new implementation plan is needed (or in progress) to catch up. Returns to **Implemented** once that plan is `Done`.
