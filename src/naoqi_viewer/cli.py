"""The `naoqi-viewer` command: argument parsing on top of the modules, nothing more."""

import argparse
import logging
import sys
from collections.abc import Callable, Sequence
from pathlib import Path

from naoqi_viewer.client import LaunchError, NaoqiViewer
from naoqi_viewer.config import ConfigError

_EXIT_FAILURE = 1
_EXIT_USAGE = 2
_EXIT_INTERRUPTED = 130


def _positive[T: (int, float)](kind: Callable[[str], T]) -> Callable[[str], T]:
    def parse(text: str) -> T:
        try:
            value = kind(text)
        except ValueError:
            raise argparse.ArgumentTypeError(f"invalid number: {text!r}") from None
        if value <= 0:
            raise argparse.ArgumentTypeError(f"must be positive, got {text}")
        return value

    return parse


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="naoqi-viewer", description="MuJoCo viewer for NAO."
    )
    verbosity = parser.add_mutually_exclusive_group()
    verbosity.add_argument(
        "-v", "--verbose", action="store_true", help="log debug messages"
    )
    verbosity.add_argument(
        "-q", "--quiet", action="store_true", help="log warnings and errors only"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    view = commands.add_parser(
        "view", help="open a viewer and wait until its window closes"
    )
    view.add_argument(
        "--config",
        metavar="PATH",
        help="a NaoqiViewerConfig JSON file (default: mirror a local NAOqi, empty scene)",
    )

    check = commands.add_parser(
        "check-model",
        help="check the model's forward kinematics against a NAOqi (moves every joint)",
    )
    check.add_argument("url", metavar="URL", help="tcp://host:port, or a bare host")
    check.add_argument("--samples", type=_positive(int), default=20)
    check.add_argument(
        "--seed", type=int, help="replay a previous run's configurations"
    )
    check.add_argument("--tolerance-mm", type=_positive(float), default=2.0)
    check.add_argument("--tolerance-deg", type=_positive(float), default=1.0)
    check.add_argument(
        "--allow-real", action="store_true", help="run against a real robot too"
    )

    fetch = commands.add_parser(
        "fetch-meshes",
        help="install Aldebaran's NAO meshes (CC BY-NC-ND 4.0, non-commercial) after a license prompt",
    )
    fetch.add_argument(
        "--archive",
        metavar="PATH",
        type=Path,
        help="a local copy of the naoqi-meshes archive (checked like a download)",
    )
    fetch.add_argument(
        "--force", action="store_true", help="install again even if already installed"
    )
    fetch.add_argument(
        "--remove", action="store_true", help="delete the installed meshes instead"
    )
    return parser


def _error(message: object) -> None:
    print(f"naoqi-viewer: error: {message}", file=sys.stderr)


def _view(args: argparse.Namespace) -> int:
    try:
        viewer = (
            NaoqiViewer.from_json_file(args.config) if args.config else NaoqiViewer()
        )
    except ConfigError as exc:
        _error(exc)
        return _EXIT_USAGE
    try:
        viewer.launch()
        viewer.wait()
    except LaunchError as exc:
        _error(exc)
        return _EXIT_FAILURE
    except KeyboardInterrupt:
        return _EXIT_INTERRUPTED
    finally:
        viewer.close()
    return 0


def _check_model(args: argparse.Namespace) -> int:
    # Loads mujoco and qi, which `view` keeps out of this process.
    from naoqi_viewer import check_model

    try:
        report = check_model.run(
            args.url,
            samples=args.samples,
            seed=args.seed,
            tolerance_mm=args.tolerance_mm,
            tolerance_deg=args.tolerance_deg,
            allow_real=args.allow_real,
        )
    except (check_model.TargetRefused, ConnectionError) as exc:
        _error(exc)
        return _EXIT_FAILURE
    print(report.format())
    return 0 if report.passed else _EXIT_FAILURE


def _fetch_meshes(args: argparse.Namespace) -> int:
    from naoqi_viewer import meshes

    if args.remove:
        if args.archive or args.force:
            _error("--remove takes neither --archive nor --force")
            return _EXIT_USAGE
        removed = meshes.remove()
        print(f"removed {removed}" if removed else "nothing installed")
        return 0
    try:
        meshes.fetch(
            archive=args.archive, force=args.force, stdin=sys.stdin, stdout=sys.stdout
        )
    except (meshes.MeshesError, OSError) as exc:
        _error(exc)
        return _EXIT_FAILURE
    except KeyboardInterrupt:
        return _EXIT_INTERRUPTED
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    level = (
        logging.DEBUG
        if args.verbose
        else logging.WARNING
        if args.quiet
        else logging.INFO
    )
    logging.basicConfig(
        level=level,
        stream=sys.stderr,
        format="%(levelname)s %(name)s: %(message)s",
        force=True,
    )
    if args.command == "view":
        return _view(args)
    if args.command == "fetch-meshes":
        return _fetch_meshes(args)
    return _check_model(args)
