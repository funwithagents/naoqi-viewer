---
code:
  - pyproject.toml
tests:
  - tests/test_project_map.py
---

# Project

**Status:** Implemented

## Purpose

Structure and tooling for the naoqi-viewer project itself: Python version, dependency/packaging management with `uv`, repo layout conventions, and development tooling.

## Decided

- **Python version:** 3.12 or 3.13 (`>=3.12,<3.14`). The upper bound comes from the libqi wheels, which exist for cp310–cp313 only.
- **libqi (`qi`):** pinned to `qi==3.1.6`. It comes from the GitHub Releases of [funwithagents/libqi-python](https://github.com/funwithagents/libqi-python), not PyPI, through per-platform URL entries in `[tool.uv.sources]`. `[tool.uv] environments` limits resolution to the platforms with wheels: macOS arm64 and Linux x86_64.
- **Runtime dependencies:** `mujoco`, `numpy`, `platformdirs`, `pyyaml`, `qi`. Aldebaran's meshes arrive already converted to OBJ and PNG from naoqi-meshes ([meshes.md](meshes.md)), so naoqi-viewer needs no mesh converter (`trimesh` was dropped). naoqi-viewer never imports naoqi-bridge or naoqi-sim (dependencies go one way: naoqi-sim → naoqi-viewer → libqi).
- **No Aldebaran assets:** `.gitignore` blocks mesh, texture, installer and archive files. Meshes and anything built from them live only in `platformdirs.user_data_dir("naoqi-viewer")`.
- **Package layout:** `src/` layout — `src/naoqi_viewer/...` — not flat, to avoid accidentally importing an uninstalled package from the repo root.
- **Dependency/venv management:** `uv`. Dev tooling lives in the `dev` dependency group (`uv sync --dev`), not in runtime `dependencies`.
- **Linting/formatting:** `ruff`.
- **Testing:** `pytest`, in two physically-separated tiers — a fast, deterministic, no-network default run (`tests/`, the only tier `testpaths` collects) and an opt-in live tier (`tests-e2e/`) that calls real external services. Full strategy is specced in [testing.md](testing.md).
- **Type checking:** `pyright` (`standard` mode), a dev dependency run via `uv run pyright`. Config lives in `[tool.pyright]` in `pyproject.toml`, targeting `src`, `tests`, and `tests-e2e`, pinned to the `.venv`. `mujoco`'s compiled bindings carry no type information, so a partial stub in `typings/mujoco/` (pyright's default stub path) types its top-level names as `Any`; its submodules still resolve to the installed package.
- **Repo shape:**
  - `src/naoqi_viewer/` — the package, one module per core concept.
  - `specs/` — pre-implementation design docs, one per concept (this folder).
  - `plans/` — implementation plans turning settled specs into buildable steps.
  - `tests/` at repo root, mirroring the `src/naoqi_viewer/` module structure.
  - `tests-e2e/` at repo root, for the live tier above — not collected by the default `pytest` run.

## Open questions

1. **libqi for pip users.** `[tool.uv.sources]` only works for uv-based development; a published wheel would declare a bare `qi` dependency. How pip users get the fork's wheels (find-links URL, a GitHub Pages index, or PyPI under a distinct name) is undecided. Intel macOS and Windows have no wheels yet.
