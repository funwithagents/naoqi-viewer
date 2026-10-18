# Mirrored camera pictures

**Status:** Done

Implements the mirroring in `specs/cameras.md` ("The pictures"), with its config field in `specs/config.md` and its key in `specs/viewer.md`. The window draws the head-camera pictures mirrored left to right by default, so they agree with the default view that faces the robot. `cameras_mirror: false` turns that off, F11 switches it while the window runs, and the status lines say `(mirrored)`. `CameraSource` and what NAOqi serves are untouched.

## Scope

- `src/naoqi_viewer/config.py` — `cameras_mirror: bool = True`, read as a boolean, no combination errors
- `src/naoqi_viewer/viewer.py` — `KEY_TOGGLE_MIRROR` (F11), `ViewerState(cameras_mirror=...)`, flipped pictures, redraw on switch, `(mirrored)` labels; `run(cameras_mirror=...)`
- `src/naoqi_viewer/viewer_process.py` — passes `config.cameras_mirror` to `run`
- `examples/configs/*.json` — the new field written out (the examples test requires every field)
- `tests/test_config.py`, `tests/test_viewer.py` — the tests listed in cameras.md's Testing

## Steps

1. Config field, its reader, the examples.
2. `ViewerState`: a `mirror_cameras` attribute from the constructor; `pictures` flips `image[:, ::-1]` before resampling (the placeholder is symmetric); the `pictures_changed` key includes it; `on_key` switches it on F11; `_status` appends ` (mirrored)` to the camera labels while it is on.
3. `run` takes `cameras_mirror` and hands it to `ViewerState`; the viewer process passes the config's.
4. Tests, then the spec statuses back to `Implemented`.

## Verification

`uv run ruff check .`, `uv run ruff format .`, `uv run pyright`, `uv run pytest`. Mark this plan `Done` (here and in [_index.md](_index.md)) only once all pass.
