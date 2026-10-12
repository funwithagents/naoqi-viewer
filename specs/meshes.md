---
code:
  - src/naoqi_viewer/meshes.py
  - src/naoqi_viewer/model.py
  - src/naoqi_viewer/models/aldebaran_visuals.json
  - src/naoqi_viewer/cli.py
  - .github/workflows/ci.yml
  - scripts/ci_fetch_meshes.py
tests:
  - tests/test_meshes.py
  - tests-e2e/test_meshes_live.py
---

# Meshes

**Status:** Implemented

## Purpose

This spec covers how each user gets Aldebaran's NAO meshes in a way the license allows, and how the viewer loads them in place of the placeholder visuals ([model.md](model.md)). The meshes are CC BY-NC-ND 4.0, not MIT. Aldebaran allows redistribution only where the user explicitly types or clicks "yes". So nothing from the meshes may enter this repository, a package or an image. Without them, naoqi-viewer works fully with its placeholder visuals.

The meshes come from [funwithagents/naoqi-meshes](https://github.com/funwithagents/naoqi-meshes). That repository runs Aldebaran's installer once, converts the meshes to OBJ with PNG textures, and publishes them as an encrypted release archive. naoqi-viewer downloads that archive and unlocks it only after the user types `yes`. naoqi-viewer needs no Collada converter, no Docker and no Linux binary, so `fetch-meshes` works the same on macOS and Linux.

## Decided

### What naoqi-viewer relies on from naoqi-meshes

A release `r<N>` of `funwithagents/naoqi-meshes` provides, for the `V40` mesh set (the meshes the V5 URDF uses; the tag numbers releases of this repository, not robot versions):

- `naoqi-meshes-V40-r<N>.zip`, encrypted with ZipCrypto under a fixed password, which `meshes.py` holds as a constant. Python's `zipfile` reads it, with no added dependency.
- `LICENSE`, the CC BY-NC-ND 4.0 text, identical to `ros-naoqi/nao_meshes`'s (SHA-256 `993af1e3328c63ad28770c756d6133fcf2f76d5e48c4a0bf6720057b3d872310`).
- Inside the archive, under `naoqi-meshes-V40-r<N>/`:
  - `manifest.json` (`format: 1`) maps each URDF visual mesh path (`V40/HeadPitch.dae`) to its parts. Each part is `{"obj", "texture" | null, "rgba" | null}`, with exactly one of `texture` and `rgba` set.
  - The OBJs are in their source DAE's frame and units, unscaled, so the URDF's visual origin and `scale` place them.
  - `meshes/*.obj`, `textures/*.png`, `NOTICE` and `LICENSE`.

Releases are immutable. naoqi-viewer pins one tag and its archive's SHA-256 (`meshes.RELEASE`, `meshes.ARCHIVE_SHA256`), with the mesh set in `meshes.MESH_SET`. A new release reaches users only through a naoqi-viewer change that bumps the pin.

### Location

- Everything lives in `platformdirs.user_data_dir("naoqi-viewer")/meshes/V40-r<N>/`: the archive's contents (`manifest.json`, `meshes/`, `textures/`, `NOTICE`, `LICENSE`) and `ACCEPTANCE.json`.
- Nothing is ever written to the working tree. `.gitignore` and the asset guard (see Testing) back this up.
- `meshes.installed() -> Path | None` returns that directory when `ACCEPTANCE.json` is there, `manifest.json` covers every mesh of the visual table, and every file the manifest names exists. Otherwise it returns `None`. [model.md](model.md)'s `resolve_variant` relies on it.

### `naoqi-viewer fetch-meshes [--archive PATH] [--force]`

1. **Already installed**: if `installed()` finds a complete install of the pinned release, the command says so, prints the path and exits 0 without prompting. `--force` removes that install and runs every step again.
2. **Notice**: the command says what it downloads and from where, that the meshes are Aldebaran's work under CC BY-NC-ND 4.0, that they are **non-commercial**, and that naoqi-viewer's MIT license does not cover them.
3. **License text**: the release's `LICENSE`, downloaded and checked against its SHA-256, then printed in full. On a mismatch the command stops.
4. **Acceptance**: the user must type `yes` in full. No default, no `y`, no flag, no environment variable. Anything else aborts before the archive is downloaded, including EOF or a stdin that is not a terminal. Automation, CI included, goes through a pseudo-terminal that types the answer, like a person would (see Testing).
5. **Download**: the pinned release's archive, checked against `ARCHIVE_SHA256` and refused on mismatch. `--archive PATH` uses a local copy with the same check.
6. **Unlock**: extract with the password into a staging directory under `meshes/`. Only entries under `naoqi-meshes-V40-r<N>/` are written, and no path may leave that directory. Check that the manifest's `format` is 1 and that it covers the visual table.
7. **Record**: write `ACCEPTANCE.json` with the UTC timestamp, the license SHA-256, the archive SHA-256, the release and the naoqi-viewer version.
8. **Atomic result**: only a complete staging directory is moved into place (`os.replace`). An interrupted or failed run leaves no partial install for `installed()` to accept, and no staging directory.

Downloads use `urllib.request` with a timeout. A failed download names the URL.

### `naoqi-viewer fetch-meshes --remove`

`--remove` deletes `meshes/` from the data directory (every release), after printing the path. It asks for no confirmation, since the meshes can be fetched again. It exits 0 when nothing is installed.

### Aldebaran visual table (`models/aldebaran_visuals.json`)

- A committed table, one entry per URDF link that has a visual mesh (39 of them): the body name in `nao.xml`, the mesh path (`V40/HeadPitch.dae`, the manifest key), position, orientation (`w x y z`) and scale.
- It is taken once from the vendored URDF's `<visual>` elements (`<mesh filename="package://nao_meshes/meshes/...">` with its origin and scale), which are BSD-3 like the rest of the URDF. It holds names and placements only, never mesh data.
- Like `nao.xml`, it is maintained by hand. `tests/test_meshes.py` checks it against the URDF and checks that every body it names exists in `nao.xml`.

### Loading (`variant="aldebaran"`)

[model.md](model.md)'s `load_world` calls `meshes.apply_visuals(spec, installed_dir)` on the `nao.xml` spec before compiling. It:

- removes every placeholder geom (class `nao_visual`);
- for each table entry and each of its manifest parts, adds a mesh asset (the OBJ's absolute path, the table's scale) and one visual-only mesh geom (`contype=conaffinity=0`, group 1, class `nao_visual`) on the matching body, at the table's placement;
- gives each geom a material: a MuJoCo 2D texture and material from the part's PNG, or the part's `rgba`.

Kinematics, joints, sensors, cameras and collision geoms are untouched, so both variants behave the same. The ghost ([viewer.md](viewer.md)) draws the group 1 geoms, so it shows the meshes too, recolored translucent.

### Sim mode and camera frames

No special case. With the meshes installed, `"auto"` resolves to `"aldebaran"` in sim mode as in mirror mode, so `camera_frame` renders can show mesh pixels (for example the hands and feet seen by `CameraBottom`). A caller who wants frames without them sets `variant: "placeholder"` ([config.md](config.md)). The window, when there is one, shows the attribution ([viewer.md](viewer.md)).

### What never happens

There is no command to export the meshes or a model that includes them, and no caching to a shared location, CI included. The compiled model with Aldebaran meshes exists only in memory.

### Testing

- **Fast tier (`tests/test_meshes.py`)**: no Aldebaran file and no network. The tests build an archive of their own at test time: small hand-written OBJs, a PNG and a manifest, zipped and encrypted with the `zip` command (skipped if `zip` is missing). The archive is served through a monkeypatched download function, with the pins monkeypatched to its hashes. These tests cover:
  - the acceptance prompt: only `yes` proceeds, while `y`, an empty line, EOF and a non-terminal stdin abort before any archive download;
  - wrong license and archive hashes;
  - unlocking, and a path-escaping entry refused;
  - `installed()`, complete or not;
  - a second fetch being a no-op, `--force`, `--remove`, and the atomic result;
  - `apply_visuals`: no placeholder geom left, a mesh geom per table body, the same body poses as the placeholder;
  - the visual table against the URDF;
  - the **asset guard**: `git ls-files` lists no `.dae`, `.stl`, `.obj`, `.msh`, `.run` or `.zip`, nor a texture image outside the docs, and `src/naoqi_viewer/` holds none.
- **E2e tier (`tests-e2e/test_meshes_live.py`)**: runs against a real install. It **skips** when `installed()` is `None`, unless `NAOQI_VIEWER_REQUIRE_MESHES` is set, which turns the skip into a failure. It needs no NAOqi: it loads `load_world(variant="aldebaran")`, poses it with `PoseWriter`, renders offscreen, and checks that every table body carries a mesh geom and that the render differs from the placeholder one.
- **CI (`meshes` job, [ci.md](ci.md))**: `scripts/ci_fetch_meshes.py` runs `naoqi-viewer fetch-meshes` under a pseudo-terminal (the standard library's `pty`) and types `yes`. Then `pytest tests-e2e/test_meshes_live.py` runs with `NAOQI_VIEWER_REQUIRE_MESHES=1`. The archive is fetched on every run and never cached.

## Open questions

None currently. The pinned release is `r1`: all 39 meshes, each one textured part sharing Aldebaran's texture atlas.
