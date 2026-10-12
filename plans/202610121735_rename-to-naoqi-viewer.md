# Rename to naoqi-viewer

**Status:** In progress

The toolkit's packages become `naoqi-*` (cross-repository analysis: `nao-sim/analysis/naoqi-rename.md`). This plan renames nao-viewer to naoqi-viewer everywhere a name comes from the package: repository, distribution, import package, CLI, `NaoViewer*` classes, `NAO_VIEWER_*` variables, user data directory, the specs and docs; it follows naoqi-sim's renamed contract (the `NaoqiSim` service, `NaoqiSim/*` keys, target id `"naoqi-sim"`) and pins the republished naoqi-meshes `r1`. It changes no behavior and adds no compatibility alias.

**Kept** (robot or upstream names, not the package's): `nao.xml`, `NAO_XML`, `NaoPose`, `NaoH25V50`, every MJCF name (`nao_joint`, `nao_visual`, `nao_grey`, `nao_collision`…), `third_party/nao_description/`, `ros-naoqi/nao_robot`, the underscore `nao_meshes` (upstream installer paths and manifest keys `nao_meshes/meshes/V40/…`), prose about the robot ("the NAO model"). `Done` plans are historical records and stay as written.

## Scope

Names, in replacement order (exact tokens, never a bare `nao`):

| Old | New |
| --- | --- |
| `NaoViewer` (`NaoViewer`, `NaoViewerConfig`, `FakeNaoViewer`) | `NaoqiViewer`… |
| `nao_viewer` | `naoqi_viewer` |
| `nao-viewer` | `naoqi-viewer` |
| `NAO_VIEWER_*` (`NAO_VIEWER_READY`, `NAO_VIEWER_ERROR`, `NAO_VIEWER_E2E_WINDOW`, `NAO_VIEWER_REQUIRE_MESHES`, `NAO_VIEWER_REQUIRE_OFFSCREEN_GL`) | `NAOQI_VIEWER_*` |
| `NAO_FAKE_VIEWER*` (test fake's variables) | `NAOQI_FAKE_VIEWER*` |
| `NaoSim` (service, `NaoSim/NaoqiVersion`, prose) | `NaoqiSim` |
| `nao-sim` (target id, repository, CLI, images, cache keys) | `naoqi-sim` |
| `NAO_SIM_REF` | `NAOQI_SIM_REF` |
| `nao-meshes` (hyphen: our repository, archive stem, password) | `naoqi-meshes` |
| `nao-bridge`, `NaoBridgeConfig` (prose) | `naoqi-bridge`, `NaoqiBridgeConfig` |

Files:

- `src/nao_viewer/` → `src/naoqi_viewer/` (`git mv`, every module and `models/`, `scenes/`, `py.typed`)
- `src/naoqi_viewer/source.py` — `Target = Literal["real", "naoqi-sim", "virtual"]`; detection on the `NaoqiSim` service, version from `NaoqiSim/NaoqiVersion`
- `src/naoqi_viewer/meshes.py` — `_REPOSITORY` `https://github.com/funwithagents/naoqi-meshes`, `_PASSWORD = b"naoqi-meshes:cc-by-nc-nd-4.0:accepted"`, `archive_name()` `naoqi-meshes-{MESH_SET}-{release}`, `RELEASE` stays `"r1"`, `ARCHIVE_SHA256` from naoqi-meshes' `SHA256SUMS` (`LICENSE_SHA256` unchanged: same text); `platformdirs.user_data_dir("naoqi-viewer")`
- `src/naoqi_viewer/protocol.py` — `NAOQI_VIEWER_READY`/`NAOQI_VIEWER_ERROR` prefixes (both ends of the pipe are this package)
- `src/naoqi_viewer/cli.py`, `client.py`, `__init__.py`, `viewer.py`, `viewer_process.py`, `config.py`, `check_model.py` — names, help texts, docstrings
- `pyproject.toml` — `name = "naoqi-viewer"`, version `0.1.0` (unchanged), `naoqi-viewer = "naoqi_viewer.cli:main"`, the description
- `uv.lock` — regenerated
- `tests/` and `tests-e2e/` — imports, fakes (`fake_viewer_process.py`, `mock_naoqi.py` exposing a `NaoqiSim` service and keys), `mesh_archive.py` (archive stem, password), the target id in `test_source.py`, `test_client.py`, `test_viewer_live.py`
- `scripts/ci_fetch_meshes.py` — names
- `.github/workflows/ci.yml` — job `e2e-naoqi-sim`, `repository: funwithagents/naoqi-sim`, `path: naoqi-sim`, `NAOQI_SIM_REF`, cache key and tar `naoqi-sim-images-…`, `naoqi-sim fetch-and-build-images`, images `naoqi-sim/naoqi:2.1.4.13 naoqi-sim/tts:dev`, `naoqi-sim/docker/compose.yaml`
- `examples/configs/*.json` — only if a name appears (paths in prose)
- `specs/` — every spec: names, the `code:`/`tests:` frontmatter paths (`src/naoqi_viewer/…`), `_overview.md` (title, the toolkit section, links `../naoqi-sim`, `../naoqi-bridge`), `source.md` (target id, the `NaoqiSim` contract), `meshes.md` (repository, the republished `r1`, password, archive stem, data directory), `ci.md` (job, pin, cache), `project.md` (data directory, dependency direction naoqi-sim → naoqi-viewer), `api.md`, `config.md`, `cli.md`, `testing.md`, `viewer.md`, `check_model.md`
- `specs/_index.md`, `plans/_index.md` — names in prose and this plan's row
- `AGENTS.md` — the project map (`src/naoqi_viewer/*` rows, which `tests/test_project_map.py` checks), names, the live-tier and assets sections; `README.md` (title, install, commands, compatibility); `THIRD_PARTY_NOTICES.md` (package name only)

Spec statuses: the spec edits land in the same change as the code, so every `Implemented` spec stays `Implemented`. If the spec edits are committed ahead of the code, set the touched specs to `Updated` until this plan is `Done`.

## Steps

1. **Local migration** (once for the four repositories, from a plain terminal, no Claude session open; analysis, "Local migration"): `gh repo rename naoqi-viewer`; `mv nao-viewer naoqi-viewer`; `git remote set-url origin git@github.com:funwithagents/naoqi-viewer.git`; move `~/.claude/projects/-Users-thibault-code-funwithagents-nao-viewer` to `…-naoqi-viewer` (after the backup) and rewrite the paths in it with the analysis's perl pass; `rm -rf .venv .pytest_cache .ruff_cache && uv sync --dev`; reopen VS Code and Claude in the new folder and check `/resume` lists the old conversations.
2. Branch `rename-naoqi` (the repository's memory: no commits on `main`).
3. `git mv src/nao_viewer src/naoqi_viewer`.
4. Replace the tokens in the table, in that order, across tracked files except `plans/2*.md`: `git ls-files -z | xargs -0 perl -pi -e '…'` with one exact substitution per row. Then review `git diff` for the kept names: `git grep -n -E 'naoqi_(joint|visual|grey|collision|finger|fsr|eye|effector|sensor|dcm|model|accent|white|meshes|description|robot)'` must return nothing (each would be an over-replacement).
5. Write the contract by hand, not by replacement: `source.py`'s detection and `Literal`, `mock_naoqi.py`'s service and keys, the tests asserting `"naoqi-sim"`.
6. **Meshes**: once naoqi-meshes has republished `r1`, set `ARCHIVE_SHA256` to the new archive's (`RELEASE` stays `"r1"`); rebuild `tests/mesh_archive.py`'s fixture names. Run `uv run naoqi-viewer fetch-meshes`, type `yes`, check the window shows the meshes, then delete `~/Library/Application Support/nao-viewer/`.
7. `uv lock && rm -rf .venv && uv sync --dev`; lint, type check, tests (Verification).
8. **Commit A** and push the branch. The fast tier is green; the live job is expected red (its pinned sim still exposes `NaoSim`). Hand commit A's SHA to naoqi-sim's plan, which pins it.
9. Once naoqi-sim's rename is merged, **commit B**: `NAOQI_SIM_REF` set to that commit. The live job turns green.
10. Merge **fast-forward** (or rebase, never squash), so commit A, which naoqi-sim pins, stays on `main`. If it was squashed, re-pin naoqi-sim to the merge commit.
11. Claude memory, in `~/.claude/projects/-Users-thibault-code-funwithagents-naoqi-viewer/memory/`: rename `nao-toolkit-naming.md` to `naoqi-toolkit-naming.md` (`name:` slug too) recording nao-local → nao-sim (Oct 8) → naoqi-sim (Oct 12, 2026) and how to detect it (`NaoqiSim` service, `NaoqiSim/Version`, target id `"naoqi-sim"`); update `mesh-legal-question-internal.md`, `specs-self-contained.md` and `MEMORY.md` with the new names and links.

## Verification

- `uv run ruff check .`, `uv run ruff format --check .`, `uv run pyright`, `uv run pytest` all clean.
- `git grep -n -E 'NaoViewer|nao_viewer|nao-viewer|NAO_VIEWER|NAO_FAKE_VIEWER|NaoSim|nao-sim|NAO_SIM|nao-meshes|nao-bridge|NaoBridge' -- ':!plans/2*'` returns nothing.
- `uv run python -c "import naoqi_viewer"` works; `uv pip list` shows `naoqi-viewer` and no `nao-viewer`.
- `uv run naoqi-viewer check-model` passes; `naoqi-viewer fetch-meshes` installs the new `r1` into `~/Library/Application Support/naoqi-viewer/meshes/V40-r1`, and the old `nao-meshes` archive no longer exists at the old URL.
- CI green on commit B, both jobs, including the live tier against naoqi-sim's 2.1 (the target reported as `naoqi-sim`, version `2.1.4.13`).

Mark this plan `Done` here and in [_index.md](_index.md) once commit B is merged and CI is green.
