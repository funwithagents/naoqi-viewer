# Implementation plans

Implementation plans for naoqi-viewer — each plan turns a settled part of a spec (see [specs/_index.md](../specs/_index.md)) into concrete, buildable steps. Plans are ordered by their date-time filename prefix (`YYYYMMDDHHmm_`).

## Plans

<!-- One row per plan, chronological by filename prefix. Keep the Status column in sync with each plan's `**Status:**` line. -->

| Plan | Description | Status |
|---|---|---|
| [202610090930_nao-model.md](202610090930_nao-model.md) | Vendored URDF, committed `nao.xml` with placeholder visuals, `empty` scene, `model.py` (implements [model.md](../specs/model.md)) | Done |
| [202610090954_pose-source.md](202610090954_pose-source.md) | `connect`, `NaoqiSource` with reconnection and target identification, and the in-process mock NAOqi (implements [source.md](../specs/source.md), [testing.md](../specs/testing.md)) | Done |
| [202610091003_viewer-window.md](202610091003_viewer-window.md) | `ViewerState` (overlay, ghost, attribution, keys) and `run` on MuJoCo's passive viewer; window title via `load_world(name=...)` (implements [viewer.md](../specs/viewer.md)) | Done |
| [202610091018_viewer-api.md](202610091018_viewer-api.md) | `launch` and the `Viewer` handle, the viewer process and its loopback protocol, the `table` scene, live window test (implements [api.md](../specs/api.md)) | Done |
| [202610091043_viewer-config.md](202610091043_viewer-config.md) | `NaoqiViewerConfig` and the `NaoqiViewer` object with `launch()` as a method, replacing the `launch()` function (implements [config.md](../specs/config.md), [api.md](../specs/api.md)) | Done |
| [202610091130_check-model-and-cli.md](202610091130_check-model-and-cli.md) | `check_model.run` and its report, the `naoqi-viewer` command with `view` and `check-model` (implements [check_model.md](../specs/check_model.md), [cli.md](../specs/cli.md)) | Done |
| [202610091137_headless-viewer.md](202610091137_headless-viewer.md) | `headless` config field, headless sim with offscreen rendering (Mesa's EGL on Linux), `run_headless`, a real headless viewer in the fast tier (implements [config.md](../specs/config.md), [api.md](../specs/api.md), [viewer.md](../specs/viewer.md), [testing.md](../specs/testing.md)) | Done |
| [202610091243_live-tier-window-switch.md](202610091243_live-tier-window-switch.md) | `NAOQI_VIEWER_E2E_WINDOW` chooses headless (default) or windowed viewers for a live run, replacing the per-test parametrization (implements [testing.md](../specs/testing.md)) | Done |
| [202610091309_ci.md](202610091309_ci.md) | CI on GitHub's Linux runners: `check`, `fast-tier` (real headless viewer, GL required) and `e2e-naoqi-sim` (naoqi-sim's NAOqi 2.1 at a pinned commit, vendor files cached) (implements [ci.md](../specs/ci.md), [testing.md](../specs/testing.md)) | Done |
| [202610091335_ci-image-cache.md](202610091335_ci-image-cache.md) | The live job caches naoqi-sim's built images (`docker save`/`load`, keyed on the naoqi-sim pin) instead of Aldebaran's vendor files (implements [ci.md](../specs/ci.md)) | Done |
| [202610091402_ci-macos-and-window.md](202610091402_ci-macos-and-window.md) | The live tier headless and windowed (under Xvfb); a macOS fast tier tried and dropped, its runners having no OpenGL (implements [ci.md](../specs/ci.md)) | Done |
| [202610091455_aldebaran-meshes.md](202610091455_aldebaran-meshes.md) | `fetch-meshes` (license prompt, naoqi-meshes' pinned encrypted archive downloaded and unlocked, `--archive`, `--force`, `--remove`), the visual table, `variant="aldebaran"` loading, fast tests on a test-built archive, the e2e meshes test and the `meshes` CI job (implements [meshes.md](../specs/meshes.md), [project.md](../specs/project.md), [cli.md](../specs/cli.md), [testing.md](../specs/testing.md), [ci.md](../specs/ci.md)) | Done |
| [202610121735_rename-to-naoqi-viewer.md](202610121735_rename-to-naoqi-viewer.md) | Rename to naoqi-viewer: package, CLI, `NaoqiViewer*`, `NAOQI_VIEWER_*`, data directory, specs and docs; detect the `NaoqiSim` contract; the republished naoqi-meshes `r1`; two commits around naoqi-sim's rename (analysis in naoqi-sim's `analysis/naoqi-rename.md`) | Done |
| [202610131053_libqi-index-and-windows.md](202610131053_libqi-index-and-windows.md) | `qi` 3.1.8 from the libqi fork's package index (uv and pip, git dependents need nothing), Windows x64, UTF-8 pipes and files, a pure-Python test archive, the fast tier on Linux, macOS and Windows (Mesa's llvmpipe) (implements [project.md](../specs/project.md), [api.md](../specs/api.md), [meshes.md](../specs/meshes.md), [testing.md](../specs/testing.md), [ci.md](../specs/ci.md)) | In progress |

## Status legend

- **Todo** — written, not yet started
- **In progress** — actively being implemented
- **Done** — implemented, verified (lint/type-check/tests pass), and merged
