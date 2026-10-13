"""Aldebaran's NAO meshes: the license-gated fetch of naoqi-meshes' archive, and loading them.

The meshes are CC BY-NC-ND 4.0 and never ship with naoqi-viewer. funwithagents/naoqi-meshes
publishes them converted to OBJ and PNG, in an encrypted zip that `fetch` unlocks only
after the user types `yes`. Everything lands in the user data directory, never in the tree.
"""

import hashlib
import importlib.metadata
import json
import os
import shutil
import tempfile
import urllib.request
import zipfile
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path, PurePosixPath
from typing import Any, TextIO

import mujoco
import platformdirs

# The pinned naoqi-meshes release (tag r<N>) of the V40 meshes, the ones the V5 URDF uses.
# Releases are immutable: a new one reaches users only by bumping these two pins.
MESH_SET = "V40"
RELEASE: str | None = "r1"
ARCHIVE_SHA256: str | None = (
    "905c12effaf6daa83947b7625d89b7bf82558b5052d197e96aec1c4684be8371"
)
# The CC BY-NC-ND 4.0 text, byte for byte ros-naoqi/nao_meshes's LICENSE.
LICENSE_SHA256 = "993af1e3328c63ad28770c756d6133fcf2f76d5e48c4a0bf6720057b3d872310"
# A gate, not a secret: the archive is opened only after the user's typed `yes`.
_PASSWORD = b"naoqi-meshes:cc-by-nc-nd-4.0:accepted"
_REPOSITORY = "https://github.com/funwithagents/naoqi-meshes"
MANIFEST_FORMAT = 1

_TABLE_PATH = Path(__file__).parent / "models" / "aldebaran_visuals.json"
_DOWNLOAD_TIMEOUT_S = 60
_PROMPT = "Type yes to accept the license and download the meshes: "


class MeshesError(Exception):
    """fetch-meshes could not complete: refused, not accepted, or a failed check."""


@dataclass(frozen=True)
class Visual:
    """One entry of the Aldebaran visual table: a URDF visual mesh and where it sits."""

    body: str
    mesh: (
        str  # path under the installer's meshes/, the manifest key (V40/HeadPitch.dae)
    )
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]  # w, x, y, z
    scale: tuple[float, float, float]


def visual_table() -> list[Visual]:
    """The committed table of Aldebaran visuals, in the URDF's link order."""
    entries = json.loads(_TABLE_PATH.read_text(encoding="utf-8"))
    return [
        Visual(
            body=e["body"],
            mesh=e["mesh"],
            pos=tuple(e["pos"]),
            quat=tuple(e["quat"]),
            scale=tuple(e["scale"]),
        )
        for e in entries
    ]


def data_dir() -> Path:
    """Where every release is installed: <user data dir>/meshes."""
    return Path(platformdirs.user_data_dir("naoqi-viewer")) / "meshes"


def archive_name(release: str) -> str:
    """The archive's stem, which is also its top folder: naoqi-meshes-V40-r1."""
    return f"naoqi-meshes-{MESH_SET}-{release}"


def install_dir(release: str) -> Path:
    """Where a release is installed: <user data dir>/meshes/V40-r1."""
    return data_dir() / f"{MESH_SET}-{release}"


def archive_url(release: str) -> str:
    return f"{_REPOSITORY}/releases/download/{release}/{archive_name(release)}.zip"


def license_url(release: str) -> str:
    return f"{_REPOSITORY}/releases/download/{release}/LICENSE"


def load_manifest(directory: Path) -> dict[str, Any]:
    """Read and check an unlocked archive's manifest.json; raises MeshesError."""
    try:
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise MeshesError(f"unreadable manifest in {directory}: {exc}") from None
    if manifest.get("format") != MANIFEST_FORMAT:
        raise MeshesError(
            f"manifest format {manifest.get('format')!r} is not {MANIFEST_FORMAT}; "
            "this naoqi-viewer can't read that naoqi-meshes release"
        )
    missing = sorted({v.mesh for v in visual_table()} - set(manifest["meshes"]))
    if missing:
        raise MeshesError(f"the archive lacks meshes: {', '.join(missing)}")
    for parts in manifest["meshes"].values():
        for part in parts:
            if (part.get("texture") is None) == (part.get("rgba") is None):
                raise MeshesError(
                    f"part {part.get('obj')!r} needs exactly one of texture and rgba"
                )
    return manifest


def _manifest_files(manifest: dict[str, Any]) -> set[str]:
    files: set[str] = set()
    for parts in manifest["meshes"].values():
        for part in parts:
            files.add(part["obj"])
            if part.get("texture"):
                files.add(part["texture"])
    return files


def installed(release: str | None = None) -> Path | None:
    """The install directory of `release` (default: the pinned one) when complete, else None."""
    release = release or RELEASE
    if release is None:
        return None
    directory = install_dir(release)
    try:
        json.loads((directory / "ACCEPTANCE.json").read_text(encoding="utf-8"))
        manifest = load_manifest(directory)
    except (OSError, ValueError, MeshesError):
        return None
    if not all((directory / f).is_file() for f in _manifest_files(manifest)):
        return None
    return directory


def remove() -> Path | None:
    """Delete every installed release; returns the directory removed, or None if none."""
    directory = data_dir()
    if not directory.exists():
        return None
    shutil.rmtree(directory)
    return directory


def _download(url: str) -> bytes:
    try:
        with urllib.request.urlopen(url, timeout=_DOWNLOAD_TIMEOUT_S) as response:
            return response.read()
    except OSError as exc:
        raise MeshesError(f"download failed: {url}: {exc}") from None


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _notice(release: str) -> str:
    return (
        f"naoqi-viewer will download Aldebaran's NAO meshes ({archive_name(release)}) from\n"
        f"  {archive_url(release)}\n"
        "They are Aldebaran's work, licensed under Creative Commons\n"
        "Attribution-NonCommercial-NoDerivatives 4.0 (CC BY-NC-ND 4.0): NON-COMMERCIAL use only.\n"
        "naoqi-viewer's MIT license does not cover them. The license text follows.\n"
    )


def _accept(stdin: TextIO, stdout: TextIO) -> None:
    if not stdin.isatty():
        raise MeshesError(
            "fetch-meshes needs an interactive terminal: the license must be accepted by typing yes"
        )
    stdout.write(_PROMPT)
    stdout.flush()
    answer = stdin.readline()
    if answer.strip() != "yes":
        raise MeshesError("license not accepted; nothing was downloaded")


def _unlock(data: bytes, release: str, into: Path) -> None:
    """Extract the archive's top folder (naoqi-meshes-V40-r1/) into `into`, refusing escapes."""
    root = PurePosixPath(archive_name(release))
    target = into.resolve()
    with tempfile.TemporaryFile() as raw:
        raw.write(data)
        raw.seek(0)
        try:
            archive = zipfile.ZipFile(raw)
        except zipfile.BadZipFile as exc:
            raise MeshesError(f"not a zip archive: {exc}") from None
        with archive:
            for info in archive.infolist():
                name = PurePosixPath(info.filename)
                if name.parts[:1] != root.parts or ".." in name.parts:
                    raise MeshesError(f"unexpected archive entry {info.filename!r}")
                relative = name.relative_to(root)
                path = (target / relative).resolve()
                if not path.is_relative_to(target):
                    raise MeshesError(f"unexpected archive entry {info.filename!r}")
                if info.is_dir():
                    path.mkdir(parents=True, exist_ok=True)
                    continue
                path.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with (
                        archive.open(info, pwd=_PASSWORD) as src,
                        path.open("wb") as dst,
                    ):
                        shutil.copyfileobj(src, dst)
                except RuntimeError as exc:  # a wrong password
                    raise MeshesError(f"cannot unlock the archive: {exc}") from None


def fetch(
    *, archive: Path | None = None, force: bool = False, stdin: TextIO, stdout: TextIO
) -> Path:
    """Run fetch-meshes: license, typed `yes`, download, unlock. Returns the install directory."""
    release = RELEASE
    if release is None or ARCHIVE_SHA256 is None:
        raise MeshesError(
            "no naoqi-meshes release is pinned in this naoqi-viewer yet; the meshes can't be fetched"
        )
    present = installed(release)
    if present is not None and not force:
        stdout.write(f"Aldebaran's meshes are already installed in {present}\n")
        return present

    stdout.write(_notice(release))
    license_text = _download(license_url(release))
    if _sha256(license_text) != LICENSE_SHA256:
        raise MeshesError(
            f"the license text at {license_url(release)} doesn't match the pinned SHA-256"
        )
    stdout.write(license_text.decode("utf-8", errors="replace").rstrip() + "\n\n")
    _accept(stdin, stdout)

    data = archive.read_bytes() if archive else _download(archive_url(release))
    archive_sha256 = _sha256(data)
    if archive_sha256 != ARCHIVE_SHA256:
        raise MeshesError("the mesh archive doesn't match the pinned SHA-256")

    root = data_dir()
    root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{release}-", dir=root))
    try:
        _unlock(data, release, staging)
        load_manifest(staging)
        accepted = {
            "accepted_at": datetime.now(UTC).isoformat(timespec="seconds"),
            "release": release,
            "license_sha256": LICENSE_SHA256,
            "archive_sha256": archive_sha256,
            "naoqi_viewer_version": importlib.metadata.version("naoqi-viewer"),
        }
        (staging / "ACCEPTANCE.json").write_text(
            json.dumps(accepted, indent=2) + "\n", encoding="utf-8"
        )
        final = install_dir(release)
        if final.exists():
            shutil.rmtree(final)
        os.replace(staging, final)
    finally:
        if staging.exists():
            shutil.rmtree(staging)
    stdout.write(f"Installed Aldebaran's meshes in {final}\n")
    return final


def apply_visuals(spec: mujoco.MjSpec, directory: Path) -> None:
    """Swap the placeholder visuals of a nao.xml spec for the installed Aldebaran meshes."""
    manifest = load_manifest(directory)
    for geom in [g for g in spec.geoms if g.classname.name == "nao_visual"]:
        spec.delete(geom)
    visual_class = spec.find_default("nao_visual")
    for visual in visual_table():
        body = spec.body(visual.body)
        for index, part in enumerate(manifest["meshes"][visual.mesh]):
            name = f"aldebaran_{visual.body}_{index}"
            mesh = spec.add_mesh(
                name=name, file=str(directory / part["obj"]), scale=list(visual.scale)
            )
            # Visual only: the bodies' inertia comes from nao.xml, and a thin shell needs
            # no closed volume to compile.
            mesh.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
            material = spec.add_material(name=name)
            if part.get("texture"):
                spec.add_texture(
                    name=name,
                    file=str(directory / part["texture"]),
                    type=mujoco.mjtTexture.mjTEXTURE_2D,
                )
                material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name
            else:
                material.rgba = list(part["rgba"])
            body.add_geom(
                default=visual_class,
                name=name,
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname=name,
                material=name,
                pos=list(visual.pos),
                quat=list(visual.quat),
            )
