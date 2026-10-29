# Props and the people display

**Status:** Done

Implements [props.md](../specs/props.md) and [people.md](../specs/people.md), with the changes they bring to [api.md](../specs/api.md) (protocol 2, `viewer.props`), [config.md](../specs/config.md) (`people_display`), [model.md](../specs/model.md) (`load_world(props=True)`) and [viewer.md](../specs/viewer.md) (the people markers in the loop). Leaves out what the specs defer: caller-supplied portraits, runtime texture swaps, configurable pool sizes, a toggle key for the markers.

## Scope

- `src/naoqi_viewer/portraits/` — the four public-domain portraits and `ATTRIBUTION.md`, shipped as package data
- `src/naoqi_viewer/props.py` — the pool (`add_pool` on an `MjSpec`), `PropState`, `PropError`, the validation shared by both sides, and `Props`, the render-thread driver; imports `mujoco` only inside the functions that need it, so `client.py` can import the state and the error
- `src/naoqi_viewer/model.py` — `load_world(..., props=False)` appends the pool
- `src/naoqi_viewer/protocol.py` — `PROTOCOL = 2`
- `src/naoqi_viewer/viewer_process.py` — the pool in sim mode, `Props.step` in `on_frame`, the five ops; a reserved name is a startup error; a `PeopleSource` with `people_display`
- `src/naoqi_viewer/client.py` — `PropsClient` and `NaoqiViewer.props`, `PropError` raised from `prop:` errors
- `src/naoqi_viewer/people.py` — `PeopleSource`, `People`, `Person`, `decode_people`, `person_geoms`
- `src/naoqi_viewer/viewer.py` — `ViewerState` takes `people`, draws the markers into `user_scn` after the ghost, the status line; `run(people=...)`
- `src/naoqi_viewer/config.py` — `people_display`, refused with `headless`
- `src/naoqi_viewer/__init__.py` — `PropState`, `PropsClient`, `PropError`
- `tests/test_props.py`, `tests/test_people.py` — new; `tests/test_viewer_process.py`, `tests/test_client.py`, `tests/test_viewer.py`, `tests/test_config.py`, `tests/test_model.py`, `tests/fake_viewer_process.py`, `tests/mock_naoqi.py` (`getListData`) — extended
- `tests-e2e/test_props_live.py`, `tests-e2e/test_people_live.py` — new
- `AGENTS.md` — the project map gains `props.py` and `people.py`; `THIRD_PARTY_NOTICES.md` the portraits

## Steps

1. Copy the portraits and their attribution into `src/naoqi_viewer/portraits/`.
2. `props.py`: constants (`FACE_IMAGES`, `FACE_SIZE`, `OBJECT_POOL_SIZE`, `PARKED_POS`), `add_pool(spec)`, the state and error, `check_spawn`/`check_place` validation, `Props(model, data, clock)` with `list`, `spawn`, `place`, `despawn`, `clear`, `step`.
3. `load_world(props=True)`, and the viewer process wiring it in sim mode with the five ops; protocol 2.
4. `PropsClient` on `NaoqiViewer`, the fake viewer process answering the ops from a `Props`-free table.
5. `people.py`, the `people_display` config field, the markers in `ViewerState` and `run`, the mock's `getListData`.
6. Live tests; project map; spec statuses.

## Verification

`uv run ruff check .`, `uv run ruff format --check .`, `uv run pyright`, `uv run pytest`, and `uv run pytest tests-e2e` against naoqi-sim's NAOqi 2.1 (`NAOQI_URL`). Then props.md and people.md go to Implemented, api.md, config.md, model.md and viewer.md back to Implemented, and this plan to Done.
