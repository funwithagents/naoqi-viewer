---
code:
  - pyproject.toml
  - .gitattributes
tests:
  - tests/test_project_map.py
---

# Project

**Status:** Implemented

## Purpose

Structure and tooling for the naoqi-viewer project itself: Python version, dependency/packaging management with `uv`, repo layout conventions, and development tooling.

## Decided

- **Python version:** 3.12 or 3.13 (`>=3.12,<3.14`): the versions tested. The libqi wheels now go from cp310 to cp315, so the bound is ours, not libqi's (open question 2).
- **libqi (`qi`):** `qi>=3.1.8,<4`, from the package index of [funwithagents/libqi-python](https://github.com/funwithagents/libqi-python) on GitHub Pages (`https://funwithagents.github.io/libqi-python/simple/`), not PyPI, whose `qi` is Aldebaran's older 3.1.5. `pyproject.toml` declares it as a named index (`[[tool.uv.index]] name = "libqi"`) with `explicit = true`, so only the packages pinned to it come from it, and pins `qi` to it in `[tool.uv.sources]` (`qi = { index = "libqi" }`). `uv.lock` holds the exact version; the range lets naoqi-viewer and the projects depending on it (naoqi-sim) move to a new libqi release one at a time without a resolver conflict. Any 3.x from 3.1.8 is accepted, on the expectation that libqi keeps its Python API within a major version; a 4.0 is adopted on purpose.
  - **A depending project declares nothing.** uv applies naoqi-viewer's index source to a git or path dependency on it: measured with uv 0.11.29, a scratch project depending on naoqi-viewer by git commit and by path locked `qi` 3.1.8 from the index, and ran it.
  - **pip** finds it with `--extra-index-url https://funwithagents.github.io/libqi-python/simple/`: pip takes the highest `qi` across indexes, 3.1.8 over PyPI's 3.1.5 (measured with pip in a fresh venv).
- **Platforms:** those the libqi wheels cover, which `[tool.uv] environments` limits the lock to: **macOS 11+ on arm64, Linux x86_64 (glibc 2.28+), Windows x64**. Elsewhere (Intel macOS, Linux arm64) installation fails instead of producing a naoqi-viewer that can't reach NAOqi. On Windows:
  - files the package reads or writes are opened as UTF-8 (Windows' default is a legacy code page), and the viewer process's pipes are UTF-8 both ways ([api.md](api.md));
  - `.gitattributes` checks every text file out with LF, so a Windows checkout (and the wheel a git install builds from it) holds the same package data as anywhere else;
  - offscreen rendering needs OpenGL 2+ from a GPU driver or Mesa ([api.md](api.md), Headless).
- **Runtime dependencies:** `mujoco`, `numpy`, `platformdirs`, `pyyaml`, `qi` (all with wheels for the three platforms). Aldebaran's meshes arrive already converted to OBJ and PNG from naoqi-meshes ([meshes.md](meshes.md)), so naoqi-viewer needs no mesh converter (`trimesh` was dropped). naoqi-viewer never imports naoqi-bridge or naoqi-sim (dependencies go one way: naoqi-sim → naoqi-viewer → libqi).
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

1. **Publishing on PyPI.** naoqi-viewer itself is installed from git. A published wheel would declare `qi>=3.1.8,<4`, which pip resolves through the libqi index above; whether to publish is undecided.
2. **Python 3.14 and later.** The libqi wheels exist up to cp315; widening `requires-python` waits on MuJoCo and numpy wheels for those versions and on CI running one.
