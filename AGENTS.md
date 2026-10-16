# Agent instructions

Start at [specs/_overview.md](specs/_overview.md) for the big picture of naoqi-viewer (goals, scope, architecture, status), then [specs/_index.md](specs/_index.md) for the list of specs and their status before making design decisions or writing code — it lists each spec and whether it's still open ("Draft"/"Not started"), design-validated ("Stable"), or built ("Implemented"). For what's been (or is being) built, see [plans/_index.md](plans/_index.md), which lists each implementation plan and its status ("Todo"/"In progress"/"Done").

## Project map

Where things live. This is a coarse, module-level map — for the full file inventory use `git ls-files`; for design detail follow the spec links.

### Top-level layout

| Path | What's there |
|---|---|
| `src/naoqi_viewer/` | The library itself — one module per core concept (see below) |
| `specs/` | Pre-implementation design docs, one per concept, each with a `**Status:**` — mapped by [specs/_overview.md](specs/_overview.md), indexed by [specs/_index.md](specs/_index.md) |
| `plans/` | Implementation plans turning settled specs into buildable steps — indexed by [plans/_index.md](plans/_index.md) |
| `tests/` | Fast, deterministic, no-network tests; mirrors the `src/naoqi_viewer/` module structure |
| `tests-e2e/` | Opt-in live tests against a running NAOqi given by `NAOQI_URL` (not collected by default `pytest`) |
| `typings/` | Local type stubs for pyright (a partial `mujoco` stub: its compiled bindings carry no types) |
| `examples/configs/` | Ready-to-use viewer config files, kept in sync with [config.md](specs/config.md) by a test |
| `.github/workflows/` | CI: lint/types, the fast tier and the live tier on naoqi-sim's NAOqi 2.1 — see [ci.md](specs/ci.md) |
| `scripts/` | Repository tooling outside the package (CI's pseudo-terminal driver for `fetch-meshes`) — see [ci.md](specs/ci.md) |
| `third_party/` | Vendored third-party sources with their licenses (the NAO URDF the model was converted from) — see `THIRD_PARTY_NOTICES.md` |

### `src/naoqi_viewer/` modules

<!-- One row per concept module. Keep this in sync with the code (a test enforces it). -->

| Module | Role | Spec |
|---|---|---|
| [`src/naoqi_viewer/__init__.py`](src/naoqi_viewer/__init__.py) | Package glue (exempt from the spec rule) | — |
| [`src/naoqi_viewer/source.py`](src/naoqi_viewer/source.py) | Pose sources: `connect` in one cancellable attempt, `NaoqiSource` polling NAOqi with reconnection and target identification | [source.md](specs/source.md) |
| [`src/naoqi_viewer/cameras.py`](src/naoqi_viewer/cameras.py) | Camera display: `CameraSource` reading both head cameras from NAOqi's `ALVideoDevice` (one subscription, reconnection, no leaked subscriptions) and the layout of their pictures in the window | [cameras.md](specs/cameras.md) |
| [`src/naoqi_viewer/config.py`](src/naoqi_viewer/config.py) | `NaoqiViewerConfig`: the viewer's declarative config, its loaders and `ConfigError` | [config.md](specs/config.md) |
| [`src/naoqi_viewer/client.py`](src/naoqi_viewer/client.py) | The caller's side: `NaoqiViewer`, built from a config, launching and driving a viewer process (standard library + numpy only) | [api.md](specs/api.md) |
| [`src/naoqi_viewer/protocol.py`](src/naoqi_viewer/protocol.py) | Message framing between a caller and its viewer process | [api.md](specs/api.md) |
| [`src/naoqi_viewer/viewer_process.py`](src/naoqi_viewer/viewer_process.py) | The viewer process: world, pose source, request server, window, camera renders | [api.md](specs/api.md) |
| [`src/naoqi_viewer/viewer.py`](src/naoqi_viewer/viewer.py) | The window: `ViewerState` (posing, status overlay, ghost, attribution) and `run` on MuJoCo's passive viewer | [viewer.md](specs/viewer.md) |
| [`src/naoqi_viewer/model.py`](src/naoqi_viewer/model.py) | The NAO model (`models/nao.xml`) and scenes (`scenes/`): `load_world`, `NaoPose`, `PoseWriter` | [model.md](specs/model.md) |
| [`src/naoqi_viewer/meshes.py`](src/naoqi_viewer/meshes.py) | `fetch-meshes`: the license-gated download and unlock of naoqi-meshes' encrypted OBJ/PNG archive, `installed`, `remove`, and `apply_visuals` swapping the placeholder visuals for Aldebaran's meshes | [meshes.md](specs/meshes.md) |
| [`src/naoqi_viewer/check_model.py`](src/naoqi_viewer/check_model.py) | `check-model`: moves a NAOqi through random configurations and compares its effectors with the model's sites | [check_model.md](specs/check_model.md) |
| [`src/naoqi_viewer/cli.py`](src/naoqi_viewer/cli.py) | The `naoqi-viewer` command: `view` and `check-model`, argument parsing only | [cli.md](specs/cli.md) |

**Keep this map current:** when you add, rename, or remove a top-level `src/naoqi_viewer/` module or a root directory, update the map in the same change — same discipline as keeping spec/plan statuses honest (below). A test (`tests/test_project_map.py`) enforces that every `src/naoqi_viewer/*.py` module appears here and vice-versa — and that the spec frontmatter (see below) stays honest too.

## Keeping statuses current

Specs and plans both carry a status, and you are responsible for keeping it honest as work progresses — update it in the same change that does the work, not as an afterthought:

- **Spec status** (`**Status:**` line near the top of each spec, and the Status column in [specs/_index.md](specs/_index.md)) tracks *design maturity* and *whether the code reflects the spec*, as a lifecycle: `Not started` → `Draft` (open questions remain) → `Stable` (design settled, reviewed and validated — open questions are deferrals only — but **not necessarily implemented yet**) → `Implemented` (a `Done` plan has built it and the code matches the spec). Keep the `**Status:**` line and the index row in sync.
  - **`Stable` is the design-review gate, not an implementation claim.** Promote `Draft` → `Stable` once the core design is settled and its remaining open questions are genuine deferrals (not load-bearing unknowns) — this is where the design is validated *before* code is written. No implementation is required to be `Stable`.
  - **`Implemented` means code matches.** Promote `Stable` → `Implemented` only once a plan implementing it is `Done` (lint, type check, tests all pass — see Verification). This is the one transition that asserts design and code are in sync.
  - **When you edit an `Implemented` spec in a way that requires new code, set its status to `Updated` in the same change.** `Updated` means the design is settled but the existing implementation now lags it — a stronger warning than `Stable`, because there is stale code to fix, not just code to write. Then write a new implementation plan for the gap (see below) and, once that plan is `Done`, flip the spec back to `Implemented`. This `Implemented → Updated → Implemented` loop keeps a spec's status an honest signal of whether the code actually matches it — never leave a re-designed spec sitting at `Implemented`.
  - A purely editorial edit to a `Stable` or `Implemented` spec (typos, clarifications, reordering — nothing that changes what the code should do) keeps its status; it does **not** need `Updated`.
- **Plan status** (`**Status:**` line near the top of each plan, and the Status column in [plans/_index.md](plans/_index.md)) tracks *implementation progress*: `Todo` → `In progress` → `Done`. Mark a plan `Done` only once it's implemented and verified (lint, type check, tests all pass — see Verification). Keep the `**Status:**` line and the index row in sync.
- Whenever you add a spec or plan, add its row to the relevant `_index.md`; whenever you change a status, change it in both the file and the index.

## Spec frontmatter

Every spec opens with a YAML frontmatter block naming the code and tests it governs:

```
---
code:
  - src/naoqi_viewer/<module>.py
tests:
  - tests/test_<module>.py
---
```

This is the **spec → code/tests** mapping — the inverse of the module → spec column in the Project map above. Its job is to give the **spec-drift checks** an explicit, version-controlled scope: the exact files to diff a spec against, so a checker never has to guess which code implements a given spec. `code:` names the implementation the spec specifies; `tests:` names the tests that pin its behavior (may be empty/absent).

The mapping is **many-to-many**: a file can be governed by several specs, so the same path legitimately appears in more than one spec's frontmatter.

**Keep it current** (same discipline as statuses): when you move, rename, or delete a file a spec governs — or add a new `src/naoqi_viewer/` module — update the affected spec's `code:`/`tests:` in the same change. Before implementation, a spec's frontmatter names the files its plan will create. `tests/test_project_map.py` enforces three invariants: every path listed by an `Implemented` spec exists (earlier statuses may name files that don't exist yet), every spec declares a non-empty `code:` list, and every concept module in `src/naoqi_viewer/` is named by at least one spec (`__init__.py` is exempt as package glue).

## Testing

- Write functional tests: exercise what a feature/function actually does (inputs → outputs, state changes, side effects), not just that it runs or matches its signature.
- Avoid trivial/tautological tests — e.g. asserting a constant, asserting an object is not `None`, asserting a mock was called. If a test would pass for a broken implementation, it's not worth writing.
- Prefer driving the public API the way a real caller would over asserting on internals.

### Live/e2e tests

Some tests talk to a live NAOqi: a real robot, or a naoqi-sim container started separately. They live in `tests-e2e/`, separate from `tests/`, so the default `uv run pytest` never runs them. The normal dev loop uses the in-process mock NAOqi instead (see [specs/testing.md](specs/testing.md)). Run the live tier explicitly with `NAOQI_URL=tcp://<host>:9559 uv run pytest tests-e2e`. Without `NAOQI_URL`, every live test **skips**. Viewers open headless by default; `NAOQI_VIEWER_E2E_WINDOW=1` opens their windows instead (needs a display), and the mirror test runs only then. Never import naoqi-sim or naoqi-bridge from tests: naoqi-sim depends on naoqi-viewer, not the other way round.

## Implementation plans

- Write implementation plans as files in the [plans](plans/) folder.
- Name each file `YYYYMMDDHHmm_plan-title.md`: a compact date-time prefix, then an underscore, then a kebab-case title (words separated by `-`).
  - Example: `202607201830_world-registry-refactor.md`
- Give each plan a `**Status:**` line just under its title (`Todo`/`In progress`/`Done`) and add a row for it to [plans/_index.md](plans/_index.md). Keep both current as work progresses (see "Keeping statuses current" above).
- Start from [plans/_plan-template.md](plans/_plan-template.md).

## Verification

After any code change, run linting, type checking, and tests, and fix any failures before considering the work done.

## Commands

```
uv sync --dev
uv run ruff check .
uv run ruff format .
uv run pyright
uv run pytest
```
