"""Helpers for the opt-in live tier.

A live test needs real credentials. Rather than fail when they're absent, a test
calls `require_env(...)` up front so it *skips* cleanly — you only exercise the
services you hold keys for, and a contributor (or CI) with none is never broken.
"""

import os
from pathlib import Path

import pytest


def require_env(name: str) -> str:
    """Return env var `name`, or skip the calling test if it's unset/empty."""
    value = os.environ.get(name)
    if not value:
        pytest.skip(f"{name} not set; skipping live test")
    return value


def window_requested() -> bool:
    """Whether this run opens its viewers with their windows (NAOQI_VIEWER_E2E_WINDOW), not headless."""
    return os.environ.get("NAOQI_VIEWER_E2E_WINDOW", "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def require_window() -> None:
    """Skip the calling test in a headless run: it needs a viewer window."""
    if not window_requested():
        pytest.skip("needs a viewer window; set NAOQI_VIEWER_E2E_WINDOW=1 to run it")


def require_meshes() -> Path:
    """The installed Aldebaran meshes, or skip (fail with NAOQI_VIEWER_REQUIRE_MESHES set)."""
    from naoqi_viewer import meshes

    directory = meshes.installed()
    if directory is None:
        reason = "Aldebaran's meshes are not installed; run `naoqi-viewer fetch-meshes`"
        if os.environ.get("NAOQI_VIEWER_REQUIRE_MESHES"):
            pytest.fail(
                f"this run requires the meshes, and {reason[0].lower()}{reason[1:]}"
            )
        pytest.skip(reason)
    return directory
