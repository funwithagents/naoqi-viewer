---
code:
  - .github/workflows/ci.yml
  - scripts/ci_fetch_meshes.py
  - pyproject.toml
tests:
---

# Continuous integration

**Status:** Updated

## Purpose

The verification gate of [AGENTS.md](../AGENTS.md) (lint, type check, tests) runs on GitHub's hosted runners for every pull request and every push to `main`, so each change gets a machine's verdict, not only a local command. CI runs both test tiers ([testing.md](testing.md)) and as much of each as a hosted runner can: the fast tier, with a real headless viewer on the mock NAOqi, on each platform naoqi-viewer installs on, and the live tier against a real NAOqi that naoqi-sim builds on the runner, headless and with windows. Like [testing.md](testing.md), this is a cross-cutting practice, not a runtime concept: nothing here ships in the library, and the one file that implements it is the workflow.

## Decided

### The runner

- **GitHub-hosted runners, the free tier.** The repository is public, so minutes cost nothing, and CI covers every path a runner can run reliably. Runners are pinned by name (`ubuntu-24.04`, `macos-15`, `windows-2025`) rather than `*-latest`, so an image only changes when the pin is bumped on purpose. No self-hosted runner.
- **The fast tier on the three platforms, the rest on Linux.** naoqi-viewer installs on Linux x86_64, macOS arm64 and Windows x64 ([project.md](project.md)), each with its own `qi` wheel and MuJoCo build, and the fast tier exercises both for real (the mock NAOqi is a real `qi.Session`). So the fast tier runs on each, which also checks that `uv.lock` installs there. Linux carries everything else: the EGL default of a headless viewer ([api.md](api.md), "Headless"), a windowed viewer under a virtual display, and Docker running naoqi-sim's amd64 NAOqi with no emulation.
- **macOS: the fast tier without OpenGL.** GitHub's macOS runners (Apple Silicon VMs) have no GPU and so no OpenGL: MuJoCo's CGL backend asks for an accelerated pixel format (`CGLPFAAccelerated`) and fails with `invalid pixel format`, measured on a `macos-15` fast-tier run, and GLFW, MuJoCo's other macOS backend, asks for one too. Its rendering tests therefore skip, and the job checks the rest: the macOS `qi` wheel against the mock NAOqi, the lock, and everything platform-neutral. An earlier macOS entry was dropped for covering only what Linux runs; the per-platform `qi` wheels are what make it worth its minutes now. The window, CGL and `mjpython` are still tested locally, on the Macs naoqi-viewer is developed on.
- **Windows: the fast tier on Mesa.** GitHub's Windows runners have no GPU either, and Windows' own `opengl32.dll` is OpenGL 1.1, too old for MuJoCo. A step puts Mesa's llvmpipe in its place (below), so the rendering tests run, as on Linux.
- **The live and meshes jobs stay on Linux.** macOS and Windows runners can't run naoqi-sim's Linux containers (no Docker on macOS, Windows containers only on Windows). The meshes job's platform-specific part (data directory, extraction, paths handed to MuJoCo) is already in the fast tier, through its test-built archive, on all three; the real archive is the same bytes everywhere, and its pseudo-terminal driver is POSIX only ([meshes.md](meshes.md)).
- **One Python**, 3.12: the floor of `requires-python`, and the version `.python-version` names. uv installs it.
- **System packages.** On Linux, Mesa's EGL (`libegl1 libopengl0 libgl1-mesa-dri`) in the three test jobs, for offscreen rendering with no display and no GPU. The windowed live entry adds a virtual X display and Mesa's GLX (`xvfb xauth libgl1 libglx-mesa0`). Nothing else is installed with apt. On Windows, Mesa from its release archive (below); on macOS, nothing.

### The workflow

One file, `.github/workflows/ci.yml`. It triggers on `pull_request`, on `push` to `main`, and on `workflow_dispatch`. Concurrency is one run per ref: a newer push cancels the older run still in flight.

| Job | What |
|---|---|
| `check` | `uv sync --locked`, then `ruff check .`, `ruff format --check .`, `pyright`: the static gate |
| `fast-tier` | Matrix `ubuntu-24.04`, `macos-15`, `windows-2025`: `pytest -rs`, the fast tier (`tests/` only, from `testpaths`); on Linux with Mesa's EGL and `MUJOCO_GL=egl`, on Windows with Mesa's llvmpipe; `NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL=1` on both, not on macOS |
| `meshes` | Mesa's EGL; `naoqi-viewer fetch-meshes` under a pseudo-terminal that types `yes`; `pytest tests-e2e/test_meshes_live.py -rs` with `MUJOCO_GL=egl`, `NAOQI_VIEWER_REQUIRE_MESHES=1` and `NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL=1` |
| `e2e-naoqi-sim` | Matrix `headless`, `window`, on `ubuntu-24.04`: naoqi-sim at a pinned commit builds and starts NAOqi 2.1; `pytest tests-e2e -rs` with `NAOQI_URL=tcp://127.0.0.1:9559`, the `window` entry with `NAOQI_VIEWER_E2E_WINDOW=1` under `xvfb-run` |

The four jobs, the fast tier's three entries and the live job's two (seven runners in all) run side by side, and none waits on another: a run takes as long as its slowest entry (a live one). Neither matrix fails fast, so one entry failing leaves the others running. Every step runs in bash, Windows included (Git's bash on its runners).

- **`--locked`.** The sync fails when `uv.lock` doesn't match `pyproject.toml`, so a dependency edit lands with its relock or not at all.
- **The format check covers the whole repo** (`.`). `specs/` and `plans/` are excluded in `pyproject.toml`, because ruff also formats Python blocks inside Markdown and the specs' blocks are hand-aligned. A local `ruff format .` is therefore the same command, and touches Python files only.
- **uv's cache** is kept by `astral-sh/setup-uv` (`enable-cache`, keyed on the lock files).
- **Timeouts**: `check` 10 minutes, `fast-tier` 15, `meshes` 15, `e2e-naoqi-sim` 30 (a cold run downloads about 800 MB and builds two images). A hung test fails its job in minutes, not hours.

### The fast tier in CI

- **Linux**: `MUJOCO_GL=egl` for the whole run. MuJoCo's default on Linux is GLFW, which needs a display, so without this the in-process renderer test would skip. The headless viewer tests get EGL from `launch()` either way.
- **Windows**: MuJoCo's GLFW, in hidden windows, on Mesa's llvmpipe. Windows loads `opengl32.dll` from the executable's folder before `System32`, so the step copies Mesa's `opengl32.dll` and `libgallium_wgl.dll` (from the `x64/` folder of [pal1000/mesa-dist-win](https://github.com/pal1000/mesa-dist-win)'s MSVC release archive) next to the base interpreter's `python.exe` (`sys.base_prefix`), which the venv's `python.exe` starts. The archive is pinned in the workflow by URL and SHA-256 (`MESA_WIN_URL`, `MESA_WIN_SHA256`; 26.2.3 today), checked before it is unpacked, and bumped on purpose. `GALLIUM_DRIVER=llvmpipe` picks Mesa's software rasterizer over its Direct3D 12 driver, which needs a GPU.
- **macOS**: nothing to install, and nothing to render with.
- `NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL=1` on Linux and Windows turns the "no offscreen OpenGL" skips into failures ([testing.md](testing.md), "Testing the viewer process"). A runner that loses its GL fails the job instead of passing it on skips. macOS doesn't set it.
- **Expected skips** ([testing.md](testing.md), "Platforms"): **Linux, none. Windows**: the two tests of the pseudo-terminal driver and the window-stack test. **macOS**: the tests that render offscreen. Any other skip is a regression.
- **Timing tests ask for rates any runner reaches.** A macOS runner polled the mock NAOqi at 27 Hz at most, so a test that the source follows `rate_hz` uses 5 and 15 Hz, not 50: it checks the behavior, not the machine's speed.

### The live job: a real NAOqi from naoqi-sim

- **NAOqi 2.1.4.13 (NAO V5)**, the version `check-model` was validated against. 2.8 isn't run in CI (open question 1).
- **naoqi-sim's own tooling, at a pinned commit.** The job checks out the public [funwithagents/naoqi-sim](https://github.com/funwithagents/naoqi-sim) at the commit `NAOQI_SIM_REF` names in the workflow, then gets naoqi-sim's NAOqi 2.1 and `tts` images: from the cache below, or else by running `naoqi-sim fetch-and-build-images 2.1` in naoqi-sim's own uv project. That command fetches the vendor files, builds the two images, and checks that they boot. The job then starts them with `docker compose --profile 2.1 up -d --wait --no-build`. The pin moves only when it is bumped on purpose, so a naoqi-sim change can't break naoqi-viewer's CI without a naoqi-viewer commit.
- **No import, one-way dependency.** naoqi-viewer's tests reach naoqi-sim's NAOqi by URL only. naoqi-sim's package lives in its own environment and is never installed into naoqi-viewer's ([testing.md](testing.md)).
- **The images are cached, not Aldebaran's files.** The vendor files (the 2.1 Choregraphe suite, and the `animations` package extracted from the robot image) are fetched only to build naoqi-sim's images, as on a user's machine. The cache holds the result, not the inputs: the two images, written by `docker save` as one file of about 1.6 GB, in this repository's Actions cache, keyed on `NAOQI_SIM_REF`.
  - **Hit**: `docker load`, with no fetch, build or naoqi-sim boot check. `compose up --wait` still waits for NAOqi's healthcheck.
  - **Miss** (a new pin): `fetch-and-build-images`, then `docker save`, then the cache is saved right after the build rather than at the end of the job, so a failing live tier doesn't build again on the next run.
  - **What a pin freezes.** A cached image keeps the base images and the `tts` image's unpinned `pip install` as they were when it was built, until the pin moves. Runs are reproducible, and an upstream change reaches CI only with a bump.
  - **Never pushed.** The images contain Aldebaran's software. They sit in a cache only this repository's workflows restore, and no registry ever receives them.
  - **Measured**: on a miss, `fetch-and-build-images` takes about 2.5 minutes (downloads, build, boot check) and the save 10 s, for a live job of about 3.7 minutes. On a hit, the restore takes 20 s and `docker load` 42 s, for a live job of 2 minutes.
- **Headless and windowed, one entry each.** The live tier opens its viewers as the run says ([testing.md](testing.md), "Live tier: headless or windowed"). The `headless` entry uses the tier's default and needs no display. The `window` entry sets `NAOQI_VIEWER_E2E_WINDOW=1` and runs under `xvfb-run`, so the windowed loop (`launch_passive`, the overlay) and the mirror test run against a real NAOqi. Each entry has its own runner and its own NAOqi. Both read the same image cache: on a new pin both miss and build, and the second save is refused because the key already exists, which is harmless.
- Viewer warnings and errors print live (`--log-cli-level=WARNING`). When an entry fails, a last step prints the NAOqi containers' logs.
- **Expected skips: in `headless`, the mirror test**, which needs a window, **and in both entries the meshes test**, which runs in its own job. Any other skip is a regression. Since `NAOQI_URL` is set, a NAOqi that doesn't answer fails the tests; nothing skips for want of an endpoint.

### The meshes job: Aldebaran's meshes, fetched each run

- **Accepted like a person would.** `fetch-meshes` has no flag or variable that skips the license prompt ([meshes.md](meshes.md)), and CI doesn't get one. A step drives the command through a pseudo-terminal (`scripts/ci_fetch_meshes.py`, on the standard library's `pty`): it waits for the prompt and types `yes`. Running the job is the maintainer accepting the license for CI.
- **Never cached.** The naoqi-meshes archive is downloaded, checked and unlocked on every run. Neither the archive nor the meshes go into the Actions cache or an artifact, and they disappear with the runner.
- **Expected skips: none.** `NAOQI_VIEWER_REQUIRE_MESHES=1` turns a missing install into a failure ([testing.md](testing.md)).

### Secrets and protection

- **No secret is required**, and none is used. naoqi-sim and Mesa's archive are public, so pull requests from forks run the same four jobs.
- **The status checks to require on `main`** are every entry: `lint, format, types`, `fast tier, ubuntu-24.04`, `fast tier, macos-15`, `fast tier, windows-2025`, `meshes`, `live tier, naoqi-sim NAOqi 2.1, headless` and `live tier, naoqi-sim NAOqi 2.1, window`. Requiring them is a repository setting on GitHub, outside the repo.

## Open questions

1. **NAOqi 2.8 (NAO V6).** A matrix entry beside 2.1 would catch a model or API difference between V5 and V6, at about twice the download (1.33 GB suite plus a 732 MB robot image) and a second cached image (about 2.4 GB on disk for 2.8, against the 10 GB per repository). Deferred until 2.8 matters to naoqi-viewer's users.
