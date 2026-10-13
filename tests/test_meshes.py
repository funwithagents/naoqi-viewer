import io
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import mujoco
import numpy as np
import pytest
from mesh_archive import (
    LICENSE_TEXT,
    FakeRelease,
    install_fake_release,
    write_release_tree,
    zip_encrypted,
)

from naoqi_viewer import meshes
from naoqi_viewer.model import JOINT_NAMES, NaoPose, PoseWriter

REPO = Path(__file__).resolve().parent.parent
URDF = REPO / "third_party" / "nao_description" / "nao.urdf"
NAO_XML = REPO / "src" / "naoqi_viewer" / "models" / "nao.xml"


class Terminal(io.StringIO):
    """A stdin that says it is a terminal, with the user's typed lines."""

    def isatty(self) -> bool:
        return True


class Untypeable(io.StringIO):
    """A terminal stdin that fails the test if fetch ever reads it."""

    def isatty(self) -> bool:
        return True

    def readline(self, size: int | None = -1, /) -> str:
        raise AssertionError("fetch prompted, but it should not have")


@pytest.fixture
def release(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> FakeRelease:
    return install_fake_release(tmp_path, monkeypatch)


def fetch(stdin: io.StringIO, **kwargs: object) -> tuple[Path, str]:
    out = io.StringIO()
    path = meshes.fetch(stdin=stdin, stdout=out, **kwargs)  # type: ignore[arg-type]
    return path, out.getvalue()


def staging_dirs() -> list[Path]:
    root = meshes.data_dir()
    return (
        [p for p in root.iterdir() if p.name.startswith(".")] if root.exists() else []
    )


# --- the visual table -----------------------------------------------------------------


def _urdf_quat(rpy: list[float]) -> np.ndarray:
    """URDF rpy (fixed axes X, then Y, then Z) as a w x y z quaternion."""
    r, p, y = (a / 2 for a in rpy)
    return np.array(
        [
            math.cos(r) * math.cos(p) * math.cos(y)
            + math.sin(r) * math.sin(p) * math.sin(y),
            math.sin(r) * math.cos(p) * math.cos(y)
            - math.cos(r) * math.sin(p) * math.sin(y),
            math.cos(r) * math.sin(p) * math.cos(y)
            + math.sin(r) * math.cos(p) * math.sin(y),
            math.cos(r) * math.cos(p) * math.sin(y)
            - math.sin(r) * math.sin(p) * math.cos(y),
        ]
    )


def test_visual_table_matches_the_urdf_visuals():
    expected = []
    for link in ET.parse(URDF).getroot().findall("link"):
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            mesh = visual.find("geometry/mesh")
            assert mesh is not None
            xyz = (
                origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            ).split()
            rpy = (
                origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            ).split()
            expected.append(
                (
                    link.get("name"),
                    mesh.get("filename", "").removeprefix(
                        "package://nao_meshes/meshes/"
                    ),
                    [float(v) for v in xyz],
                    _urdf_quat([float(v) for v in rpy]),
                    [float(v) for v in mesh.get("scale", "1 1 1").split()],
                )
            )
    table = meshes.visual_table()
    assert len(table) == len(expected) == 39
    for entry, (body, mesh, pos, quat, scale) in zip(table, expected, strict=True):
        assert (entry.body, entry.mesh) == (body, mesh)
        np.testing.assert_allclose(entry.pos, pos, atol=1e-6)
        assert (
            abs(abs(float(np.dot(entry.quat, quat))) - 1) < 1e-9
        )  # q and -q are equal
        assert list(entry.scale) == scale


def test_every_visual_table_body_exists_in_nao_xml():
    spec = mujoco.MjSpec.from_file(str(NAO_XML))
    missing = [v.body for v in meshes.visual_table() if spec.body(v.body) is None]
    assert missing == []


# --- fetch: the license gate ----------------------------------------------------------


def test_fetch_installs_after_yes(release: FakeRelease):
    path, out = fetch(Terminal("yes\n"))
    assert path == meshes.data_dir() / f"V40-{release.release}"
    assert meshes.installed() == path
    assert "NON-COMMERCIAL" in out and LICENSE_TEXT.decode().strip() in out
    accepted = json.loads((path / "ACCEPTANCE.json").read_text())
    assert accepted["release"] == release.release
    assert accepted["archive_sha256"] == meshes.ARCHIVE_SHA256
    assert accepted["license_sha256"] == meshes.LICENSE_SHA256
    assert (path / "manifest.json").is_file()
    assert staging_dirs() == []


@pytest.mark.parametrize("answer", ["y\n", "Yes\n", "yes please\n", "\n", ""])
def test_anything_but_yes_aborts_before_the_archive_download(
    release: FakeRelease, answer: str
):
    with pytest.raises(meshes.MeshesError, match="not accepted"):
        fetch(Terminal(answer))
    assert release.downloads == [meshes.license_url(release.release)]
    assert meshes.installed() is None


def test_a_stdin_that_is_not_a_terminal_is_refused(release: FakeRelease):
    with pytest.raises(meshes.MeshesError, match="interactive terminal"):
        fetch(io.StringIO("yes\n"))
    assert meshes.archive_url(release.release) not in release.downloads
    assert meshes.installed() is None


def test_a_wrong_license_text_stops_before_the_prompt(
    release: FakeRelease, monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.setattr(meshes, "LICENSE_SHA256", "0" * 64)
    with pytest.raises(meshes.MeshesError, match="license text"):
        fetch(Untypeable())


def test_a_wrong_archive_is_refused(
    release: FakeRelease, monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.setattr(meshes, "ARCHIVE_SHA256", "0" * 64)
    with pytest.raises(meshes.MeshesError, match="archive doesn't match"):
        fetch(Terminal("yes\n"))
    assert meshes.installed() is None
    assert staging_dirs() == []


def test_a_local_archive_is_used_and_checked(release: FakeRelease):
    path, _ = fetch(Terminal("yes\n"), archive=release.archive_path)
    assert meshes.installed() == path
    assert meshes.archive_url(release.release) not in release.downloads


def test_no_pinned_release_refuses(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(meshes, "RELEASE", None)
    with pytest.raises(meshes.MeshesError, match="no naoqi-meshes release is pinned"):
        fetch(Untypeable())


# --- fetch: install lifecycle ---------------------------------------------------------


def test_a_second_fetch_is_a_no_op_without_a_prompt(release: FakeRelease):
    first, _ = fetch(Terminal("yes\n"))
    downloads = len(release.downloads)
    second, out = fetch(Untypeable())
    assert second == first
    assert "already installed" in out
    assert len(release.downloads) == downloads


def test_force_installs_again(release: FakeRelease):
    first, _ = fetch(Terminal("yes\n"))
    (first / "stale").write_text("from the previous install")
    second, _ = fetch(Terminal("yes\n"), force=True)
    assert second == first
    assert not (second / "stale").exists()
    assert meshes.installed() == second


def test_remove_deletes_every_install(release: FakeRelease):
    fetch(Terminal("yes\n"))
    assert meshes.remove() == meshes.data_dir()
    assert meshes.installed() is None
    assert meshes.remove() is None


def test_installed_needs_every_file_and_the_acceptance(release: FakeRelease):
    path, _ = fetch(Terminal("yes\n"))
    (path / "meshes" / "Torso_0.obj").unlink()
    assert meshes.installed() is None

    path, _ = fetch(Terminal("yes\n"), force=True)
    (path / "ACCEPTANCE.json").unlink()
    assert meshes.installed() is None


def _pin(monkeypatch: pytest.MonkeyPatch, release: FakeRelease, archive: bytes) -> None:
    import hashlib

    monkeypatch.setattr(meshes, "ARCHIVE_SHA256", hashlib.sha256(archive).hexdigest())
    release.archive = archive
    monkeypatch.setattr(
        meshes,
        "_download",
        lambda url: LICENSE_TEXT if url.endswith("/LICENSE") else archive,
    )


def test_an_archive_entry_outside_its_folder_is_refused(
    release: FakeRelease, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    evil = tmp_path / "evil.zip"
    shutil.copy(release.archive_path, evil)
    with zipfile.ZipFile(evil, "a") as archive:
        archive.writestr("escape.txt", "outside naoqi-meshes-V40-r<N>/")
    _pin(monkeypatch, release, evil.read_bytes())
    with pytest.raises(meshes.MeshesError, match="unexpected archive entry"):
        fetch(Terminal("yes\n"))
    assert meshes.installed() is None
    assert staging_dirs() == []
    assert not (tmp_path / "data" / "escape.txt").exists()


def test_an_incomplete_archive_leaves_no_install(
    release: FakeRelease, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    build = tmp_path / "incomplete"
    folder = write_release_tree(build, release.release)
    manifest = json.loads((folder / "manifest.json").read_text())
    del manifest["meshes"]["V40/Torso.dae"]
    (folder / "manifest.json").write_text(json.dumps(manifest))
    archive = zip_encrypted(build, folder.name, tmp_path / "incomplete.zip")
    _pin(monkeypatch, release, archive)
    with pytest.raises(meshes.MeshesError, match="lacks meshes: V40/Torso.dae"):
        fetch(Terminal("yes\n"))
    assert meshes.installed() is None
    assert staging_dirs() == []


# --- loading --------------------------------------------------------------------------


def _posed_bodies(model: mujoco.MjModel) -> np.ndarray:
    data = mujoco.MjData(model)
    pose = NaoPose(
        angles={name: 0.3 for name in JOINT_NAMES},
        torso_pos=(0.0, 0.0, 0.33),
        torso_quat=(1.0, 0.0, 0.0, 0.0),
    )
    PoseWriter(model).apply(data, pose)
    return np.concatenate([data.xpos.copy(), data.xquat.copy()], axis=1)


def test_apply_visuals_swaps_the_placeholders_for_meshes(release: FakeRelease):
    path, _ = fetch(Terminal("yes\n"))
    placeholder = mujoco.MjSpec.from_file(str(NAO_XML))
    spec = mujoco.MjSpec.from_file(str(NAO_XML))
    meshes.apply_visuals(spec, path)
    model = spec.compile()

    visual = [
        g
        for g in range(model.ngeom)
        if model.geom_group[g] == 1 and model.geom_contype[g] == 0
    ]
    assert all(model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH for g in visual)
    bodies = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g])
        for g in visual
    }
    assert bodies == {v.body for v in meshes.visual_table()}
    assert (
        model.ngeom == placeholder.compile().ngeom - _placeholder_count() + 40
    )  # 39 + head's 2nd part

    textured = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "aldebaran_Head_0")
    flat = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "aldebaran_Head_1")
    assert (
        model.mat_texid[model.geom_matid[textured], mujoco.mjtTextureRole.mjTEXROLE_RGB]
        >= 0
    )
    np.testing.assert_allclose(
        model.mat_rgba[model.geom_matid[flat]], [0.2, 0.2, 0.2, 1]
    )

    np.testing.assert_allclose(
        _posed_bodies(model), _posed_bodies(placeholder.compile()), atol=1e-12
    )


def _placeholder_count() -> int:
    spec = mujoco.MjSpec.from_file(str(NAO_XML))
    return sum(g.classname.name == "nao_visual" for g in spec.geoms)


# --- the asset guard ------------------------------------------------------------------

_ASSET_SUFFIXES = {
    ".dae",
    ".stl",
    ".obj",
    ".msh",
    ".mesh",
    ".run",
    ".zip",
    ".png",
    ".jpg",
    ".jpeg",
    ".tga",
}


def test_no_mesh_texture_installer_or_archive_is_tracked():
    tracked = subprocess.run(
        ["git", "ls-files"], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout.split()
    assert [f for f in tracked if Path(f).suffix.lower() in _ASSET_SUFFIXES] == []


def test_the_package_holds_no_mesh_texture_or_archive():
    package = REPO / "src" / "naoqi_viewer"
    found = [p for p in package.rglob("*") if p.suffix.lower() in _ASSET_SUFFIXES]
    assert found == []


# --- CI's pseudo-terminal driver ------------------------------------------------------

_CI_DRIVER = REPO / "scripts" / "ci_fetch_meshes.py"

# The driver runs fetch-meshes on a pseudo-terminal (the standard library's pty), which Windows
# lacks; CI runs it on Linux (specs/ci.md).
posix_terminal = pytest.mark.skipif(
    sys.platform == "win32", reason="the CI driver needs a POSIX pseudo-terminal"
)

# Stands in for fetch-meshes: insists on a terminal, prompts like it, and needs `yes`.
_FAKE_FETCH = """
import sys
if not sys.stdin.isatty():
    sys.exit("not a terminal")
sys.stdout.write("Type yes to accept the license and download the meshes: ")
sys.stdout.flush()
sys.exit(0 if sys.stdin.readline().strip() == "yes" else 3)
"""


@posix_terminal
def test_the_ci_driver_types_yes_on_a_terminal():
    result = subprocess.run(
        [sys.executable, str(_CI_DRIVER), "--", sys.executable, "-c", _FAKE_FETCH],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Type yes to accept the license" in result.stdout


@posix_terminal
def test_the_ci_driver_fails_when_no_prompt_comes():
    result = subprocess.run(
        [
            sys.executable,
            str(_CI_DRIVER),
            "--",
            sys.executable,
            "-c",
            "print('no prompt')",
        ],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 1
    assert "the license prompt never came" in result.stderr
