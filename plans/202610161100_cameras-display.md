# Camera display

**Status:** Done

Implements [specs/cameras.md](../specs/cameras.md): the `cameras_display` config field, `CameraSource` reading both head cameras from NAOqi's `ALVideoDevice`, and the two pictures in the window's top-right corner with their status lines and the F10 toggle, in both modes. It catches up the specs the design updated ([config.md](../specs/config.md), [viewer.md](../specs/viewer.md), [api.md](../specs/api.md), [testing.md](../specs/testing.md)). It deliberately leaves out resolution and rate knobs, camera fields in `ViewerStatus`, and any frame source other than `ALVideoDevice` (the spec's open questions).

Precondition: [cameras.md](../specs/cameras.md) promoted from `Draft` to `Stable` after review.

## Scope

- `src/naoqi_viewer/cameras.py` — new: the constants, `CameraImage`, `ImageSource`, `decode_images`, `CameraSource`, `picture_rects`, `resample_nearest`, the placeholder picture
- `src/naoqi_viewer/config.py` — `cameras_display: bool = False`, its type check, the `ConfigError` with `headless`, `to_dict`
- `src/naoqi_viewer/viewer.py` — `ViewerState(cameras=...)`: `pictures(viewport)`, the two status lines, F10; `run(cameras=...)` calling `set_images`/`clear_images`
- `src/naoqi_viewer/viewer_process.py` — builds a `CameraSource` when the config says so, hands it to `run`, closes it
- `examples/configs/mirror.json`, `sim-table.json`, `sim-headless.json` — the new field (`true`, `true`, `false`)
- `README.md` — the field in the config listing and one sentence on the pictures and F10
- `tests/mock_naoqi.py` — the `ALVideoDevice` service, `set_image`, `subscribers()`
- `tests/test_mock_naoqi.py` — the mock's subscription rules
- `tests/test_cameras.py` — new: `CameraSource` on the mock, `picture_rects`, `resample_nearest`, `decode_images`
- `tests/test_viewer.py` — `ViewerState` with a fake image source
- `tests/test_config.py` — the field, the examples
- `tests/test_viewer_process.py` — no `ALVideoDevice` call with the field off; a `CameraSource` built and closed with it on
- `tests-e2e/test_cameras_live.py` — new: a `CameraSource` against the live NAOqi
- `tests-e2e/test_viewer_live.py` — the mirror test opens with `cameras_display` on
- `AGENTS.md` — `cameras.py` in the project map (only once the module exists: `tests/test_project_map.py` rejects a row without a file)
- `specs/cameras.md`, `config.md`, `viewer.md`, `api.md`, `testing.md`, `specs/_index.md`, `specs/_overview.md`, this plan, `plans/_index.md` — statuses

## Steps

1. **Config.** Add `cameras_display` to `NaoqiViewerConfig` next to `ghost`, read with `_as_bool`, in `to_dict`, with `headless and cameras_display` a `ConfigError` on key `cameras_display` ("drawn only in the window, and a headless viewer has none"). Update the three example files and the README. Tests: default `False`, `from_dict`/`to_dict` round trip, `"yes"` rejected, the headless combination, the examples still parse.
2. **Mock `ALVideoDevice`.** A `_VideoDevice` service on `MockNaoqi`, behind `_call` like the others (so `fail_calls` and the call log cover it):
   - `subscribeCameras(name, indexes, resolutions, colorspaces, fps)` records the subscription and returns `f"{name}_{n}"` with `n` the first free suffix for that base name, raising past `_5` as NAOqi does;
   - `getImagesRemote(name)` returns, for each index of the subscription, NAOqi's list (`[width, height, 3, 11, ts_s, ts_us, bytes, index, 0, 0, 0, 0]`) built from the image set with `mock.set_image("top" | "bottom", array)`, or an empty list `[]` for a camera without one; an unknown name returns `[]`;
   - `unsubscribe(name)` removes one; `unsubscribeAllInstances(name)` removes every `name_<n>`; `getSubscribers()` lists the names; `mock.subscribers()` is the same list from the test's side.
   - Test the mock itself in `tests/test_mock_naoqi.py`: suffixing, the seventh refused, the list layout, `unsubscribeAllInstances`.
3. **`cameras.py`.**
   - Constants as in the spec (`CAMERAS`, `DISPLAY_RESOLUTION`, `DISPLAY_COLORSPACE`, `DISPLAY_FPS`, `SUBSCRIBER_NAME`, `IMAGE_STALE_AFTER`, `PICTURE_*`).
   - `decode_images(reply, received_at, seq) -> dict[str, CameraImage]`: a pure function over NAOqi's reply; skips an entry with no bytes or a byte count that isn't `width * height * 3`; maps camera index to `top`/`bottom`.
   - `CameraSource`: the thread connects through `connect(url, cancel=...)`, calls `unsubscribeAllInstances(SUBSCRIBER_NAME)` the first time only (a flag on the instance), `subscribeCameras`, then polls `getImagesRemote` on a fixed schedule at `fps` with `_Ticks`-style timing; `latest(camera)` reads a per-camera slot under a lock; `rate()` as `NaoqiSource.rate()`; an exception in connect, subscribe or a poll logs once per outage, keeps the slots, sleeps the backoff (0.5 s doubling to 5 s) and starts over from `connect`; `close()` sets the cancel event, joins the thread, then unsubscribes with `_async=True` and waits at most 1 s. Reuse `NaoqiSource`'s backoff and rate helpers rather than copying them where they are already functions; extract them if they aren't.
   - `picture_rects(viewport)` and `resample_nearest(image, size)` as specified; `placeholder(size)` a dark grey image with the frame; `framed(image)` the border.
   - Tests in `tests/test_cameras.py` follow the spec's Testing list, on the mock through a real `CameraSource` (waiting on `latest()` with a deadline, as `test_source.py` waits on samples).
4. **`ViewerState` and `run`.**
   - `ViewerState(..., cameras: ImageSource | None = None)` gains `show_cameras = True`, `pictures(viewport) -> list[tuple[tuple[int, int, int, int], np.ndarray]] | None` returning the rectangles and framed images, and `pictures_changed(viewport) -> bool` (a new `seq` for either camera, a different viewport, or the toggle flipped since the last call). `_status` appends the two camera lines when `cameras` is given. `on_key` handles `KEY_TOGGLE_CAMERAS = 299` (GLFW F10).
   - `run(..., cameras=None)`: after `set_texts`, when `cameras` is given and `pictures_changed(handle.viewport)`, call `handle.set_images([(mujoco.MjrRect(*rect), image) ...])`, or `handle.clear_images()` when hidden or when `pictures` is `None`. Outside `handle.lock()`.
   - Tests in `tests/test_viewer.py` with a fake `ImageSource` and a fixed viewport tuple: the two rectangles and image sizes, the placeholder before an image, `pictures_changed` true once then false, F10, the status lines through `NO IMAGE`, an age, `STALE`.
5. **Viewer process.** With `config.cameras_display`, build `CameraSource(url)` after the pose source and pass it to `run`; close it in the `finally` block. Tests on the mock: with the field off, `mock.calls_to("subscribeCameras")` is empty after a viewer process's setup; with it on, a `CameraSource` is built (the headless path never gets here, by the config). The real headless viewer test is unchanged.
6. **Live tier.** `tests-e2e/test_cameras_live.py`: a `CameraSource` on `NAOQI_URL`; within 5 s a name starting with `naoqi_viewer` is in `ALVideoDevice.getSubscribers()` (through a `connect()` session of the test's own); after 1 s of polling no error was logged (`caplog`); any image that arrived is `(240, 320, 3)` `uint8`; after `close()` the name is gone. In `test_viewer_live.py`, the mirror test's config gets `"cameras_display": True`. Verify against naoqi-sim's NAOqi 2.1 that `subscribeCameras`, `getImagesRemote` and `unsubscribeAllInstances` behave as the mock assumes, and fix the mock where they don't.
7. **Project map and statuses.** Add the `cameras.py` row to `AGENTS.md`. Mark this plan `Done`, `cameras.md` `Implemented`, and `config.md`, `viewer.md`, `api.md`, `testing.md` back to `Implemented`; update the overview's Status table row.

## Progress

- Done: every step. Two things the live NAOqi could not be asked here, so they were settled against the libqi fork's sources and its behavior instead: a qi raw buffer arrives in Python as a `bytearray` (the fork's `visitRaw`), so `decode_images` accepts `bytes`, `bytearray` and `memoryview`; and a plain `bytes` returned inside a dynamic value travels as a string, so the mock binds `getImagesRemote` with NAOqi's signature (`[(iiiiiiriffff)]`) to serve a real raw buffer. One rule was added to the spec while testing: NAOqi answers an unknown subscription name with an empty reply, not an error, so the source checks its name against `getSubscribers()` every 2 s while no image comes, which is what makes two viewers on one NAOqi converge.
- Lint, types and the fast tier pass (215 tests). A windowed mirror viewer on the mock NAOqi, with `cameras_display` on, drew for four seconds without a warning, exited cleanly and left no subscription. The live tier (`tests-e2e/test_cameras_live.py`, the mirror test with the display on) has not been run yet: no NAOqi was reachable from this machine. Step 6's check of `subscribeCameras`, `getImagesRemote` and `unsubscribeAllInstances` against NAOqi 2.1 is therefore still owed to the first live run.

## Verification

- `uv run ruff check .`, `uv run ruff format .`, `uv run pyright`, `uv run pytest`: all pass, with no network.
- `NAOQI_URL=tcp://127.0.0.1:9559 uv run pytest tests-e2e -rs` against naoqi-sim's NAOqi 2.1, headless and with `NAOQI_VIEWER_E2E_WINDOW=1`.
- By hand: `naoqi-viewer view --config examples/configs/sim-table.json` against naoqi-sim shows two pictures (or `NO IMAGE` until naoqi-sim injects), F10 hides them, closing the window leaves `ALVideoDevice.getSubscribers()` without a `naoqi_viewer` entry; `mirror.json` against a real NAO shows its cameras.
