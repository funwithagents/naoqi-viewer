# libqi from its package index, and Windows

**Status:** In progress

Implements [project.md](../specs/project.md) ("libqi", "Platforms"), [source.md](../specs/source.md) ("`connect`"), [api.md](../specs/api.md) ("Headless", "Viewer process"), [meshes.md](../specs/meshes.md) ("Location", "Testing"), [testing.md](../specs/testing.md) ("Platforms") and [ci.md](../specs/ci.md) ("The runner", "The fast tier in CI"). `qi` 3.1.8 comes from the libqi fork's package index on GitHub Pages instead of per-wheel release URLs, Windows x64 joins macOS arm64 and Linux x86_64, and CI runs the fast tier on all three. The live and meshes jobs stay on Linux. naoqi-sim moves to this commit afterwards, in its own change.

## Scope

- `pyproject.toml`: `qi>=3.1.8,<4`, the `libqi` index (`explicit = true`) and `qi = { index = "libqi" }`, Windows x64 in `environments`
- `uv.lock`: relocked (qi 3.1.8 from the index; colorama, pytest's Windows dependency)
- `.gitattributes`: every text file checked out with LF
- `src/naoqi_viewer/client.py`: the viewer process's environment always set (`PYTHONIOENCODING=utf-8`, EGL as before), its pipes read as UTF-8 with replacement
- `src/naoqi_viewer/meshes.py`: the visual table, manifest and acceptance read and written as UTF-8
- `src/naoqi_viewer/source.py`: `connect` in one attempt (libqi 3.1.8 fixed the failed connects 3.1.6 needed retries for), which `cancel` gives up in flight (Windows takes about 2 s to refuse a port)
- `src/naoqi_viewer/viewer_process.py`: the "cannot render offscreen" error says how to fix it on Windows too
- `tests/test_client.py`: the environment tests follow `_viewer_env`'s new shape, and a non-ASCII path reaches `LaunchError` intact under a legacy code page
- `tests/mesh_archive.py`: a ZipCrypto writer in place of the `zip` command
- `tests/test_meshes.py`: the pseudo-terminal driver's tests skip on Windows
- `tests/test_source.py`: cancelling `connect`, and closing a `NaoqiSource`, while an attempt to a port that never speaks qi is in flight; a failed connect is not retried (in place of the retry test)
- `tests-e2e/test_viewer_live.py`: the comments about retrying go
- `AGENTS.md`: the project map's `source.py` row
- `tests/test_viewer_process.py`: the missing-scene test writes its config with `json.dumps` (a Windows path's backslashes)
- `tests/test_viewer.py`: the window-stack test skips on Windows
- `.github/workflows/ci.yml`: `fast-tier` as a matrix over `ubuntu-24.04`, `macos-15`, `windows-2025`, with Mesa's llvmpipe on Windows
- `README.md`, the specs above, `specs/_overview.md`, `specs/_index.md`, this plan, `plans/_index.md`

## Steps

1. Switch `qi` to the index and relock. Measure that a project depending on naoqi-viewer, by git commit and by path, gets `qi` from the index with nothing declared, and that pip finds it with `--extra-index-url`.
2. UTF-8: `_viewer_env` returns the caller's environment plus `PYTHONIOENCODING=utf-8` (and `MUJOCO_GL=egl` for a headless viewer on Linux when unset); `launch()` opens the pipes with `encoding="utf-8", errors="replace"`. `meshes.py` passes `encoding="utf-8"` to every `read_text`/`write_text`.
3. Tests: the ZipCrypto writer (deflated, encrypted files; unencrypted folder entries; the header's last byte the CRC's high byte); the platform skips of [testing.md](../specs/testing.md), "Platforms"; the non-ASCII launch test, with `PYTHONIOENCODING=cp1252` in the caller's environment standing in for Windows.
4. `.gitattributes` with `* text=auto eol=lf`.
5. CI: the fast-tier matrix in bash; apt's EGL on Linux only; on Windows, after the sync, Mesa 26.2.3's MSVC archive downloaded, its SHA-256 checked, and `x64/opengl32.dll`, `x64/libgallium_wgl.dll` unpacked into `sys.base_prefix`; `MUJOCO_GL=egl` on Linux, `GALLIUM_DRIVER=llvmpipe`, `NAOQI_VIEWER_REQUIRE_OFFSCREEN_GL=1` except on macOS.
6. From the first CI run (Windows, 3 failures): `connect` gives up an attempt in flight when `cancel` is set, and checks it before each attempt (the two prompt-cancel tests waited out Windows' 2 s refusal); the missing-scene test writes its JSON with `json.dumps`.
7. No retry: with libqi 3.1.8, 200 plain `session.connect` calls to naoqi-sim's NAOqi 2.1 all succeeded (3.1.6: 12 failed with `disconnected`), so `connect` makes one attempt. `NaoqiSource` keeps its reconnection.
8. Statuses, once a run on GitHub is green with ci.md's expected skips: this plan `Done`; project.md, source.md, api.md, meshes.md, testing.md and ci.md `Implemented`.

## Verification

- Locally on macOS arm64 with qi 3.1.8: lint, format, types, the fast tier (186 passed), and the live tier against naoqi-sim's NAOqi 2.1, headless (4 passed, the mirror test skipped) and windowed (5 passed, the meshes test included).
- Step 1 measured with uv 0.11.29: the git and path dependents locked `qi` 3.1.8 from the index, and the git one imported it; pip 3.12 installed `qi` 3.1.8.
- The non-ASCII launch test fails without the `client.py` change and passes with it. The two in-flight tests fail without the `source.py` change (within their time bound, no hang) and pass with it; the no-retry test counts 5 attempts on the retrying `connect` and 1 on the new one. Info-ZIP's `unzip -t` accepts the test-built archive and refuses a wrong password.
- On GitHub: every job green; fast-tier skips as ci.md expects (Linux none; Windows the driver's two tests and the window-stack test; macOS the offscreen renders).
