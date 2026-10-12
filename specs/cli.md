---
code:
  - src/naoqi_viewer/cli.py
  - pyproject.toml
tests:
  - tests/test_cli.py
---

# Command line

**Status:** Implemented

## Purpose

A small `naoqi-viewer` command for one-shot tools that don't fit in application code, plus a convenience command to open a viewer from a terminal. Viewing is done through the Python API ([api.md](api.md)); the CLI is a thin layer on top of the modules.

## Decided

### Commands

| Command | Does | Spec |
|---|---|---|
| `naoqi-viewer view [--config PATH]` | Builds a `NaoqiViewer` from the config file (the default config without one), launches it and waits until the window closes | [api.md](api.md), [config.md](config.md) |
| `naoqi-viewer check-model URL [--samples N] [--seed N] [--tolerance-mm X] [--tolerance-deg X] [--allow-real]` | Forward-kinematics check | [check_model.md](check_model.md) |
| `naoqi-viewer fetch-meshes [--archive PATH] [--force] [--remove]` | License prompt, then download and unlock of the mesh archive (interactive); a no-op when already installed; `--remove` deletes the install | [meshes.md](meshes.md) |

- `view` takes its settings from a config file only, as naoqi-bridge's servers do: the mode, NAOqi URL, scene, variant and ghost live in the file ([config.md](config.md), with ready-made files in `examples/configs/`), not in flags. Without `--config` it opens the default config: mirror mode on a local NAOqi, empty scene.
- `check-model`'s `URL` is `tcp://host:port` or a bare host (port 9559).
- `fetch-meshes` needs an interactive terminal: its license prompt is specified in [meshes.md](meshes.md).
- `check-model` prints its report ([check_model.md](check_model.md), `CheckReport.format()`) to stdout.
- `view` returns when the window closes. A headless config has no window, so `view` runs until the viewer exits or Ctrl-C. Ctrl-C closes the viewer and exits with 130.
- `-v`/`-q` (before the command) set the log level: INFO by default, DEBUG with `-v`, WARNING with `-q`. Logs go to stderr. Errors are one line on stderr, `naoqi-viewer: error: <message>`, with no traceback.
- Exit codes: 0 success, 1 failure (`check-model` over tolerance, a refused or failed fetch, a viewer that couldn't launch), 2 usage error (argparse's own, or a `ConfigError`, whose message names the key).

### Entry point

`[project.scripts] naoqi-viewer = "naoqi_viewer.cli:main"`. `view` goes through the public API (`NaoqiViewer`), so the `mjpython` handling lives in one place ([api.md](api.md), Viewer process) and the CLI process itself never needs `mjpython`.

### Structure

`cli.py` only parses arguments and calls module functions. Logic belongs to the modules, so they stay testable without a subprocess.

## Open questions

None currently.
