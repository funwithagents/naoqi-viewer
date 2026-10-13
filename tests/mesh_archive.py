"""A naoqi-meshes-shaped archive built at test time from shapes of our own (no Aldebaran file).

The archive follows the release contract in specs/meshes.md: an encrypted zip holding
naoqi-meshes-V40-r<N>/ with manifest.json, meshes/*.obj and textures/*.png. Every mesh of
the visual table gets a textured tetrahedron, and the head a second, flat-colored part.
"""

import hashlib
import json
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

import pytest

from naoqi_viewer import meshes

LICENSE_TEXT = b"Test license: CC BY-NC-ND 4.0 stand-in.\n"

# A tetrahedron about 4 cm across before the table's 0.1 scale (DAE units), with UVs.
TEXTURED_OBJ = """\
v 0 0 0
v 0.4 0 0
v 0 0.4 0
v 0 0 0.4
vt 0 0
vt 1 0
vt 0 1
vt 1 1
f 1/1 3/3 2/2
f 1/1 2/2 4/4
f 1/1 4/4 3/3
f 2/2 3/3 4/4
"""

FLAT_OBJ = """\
v 0 0 0.5
v 0.2 0 0.5
v 0 0.2 0.5
v 0 0 0.7
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
"""


def png(width: int, height: int, rgb: tuple[int, int, int]) -> bytes:
    """A plain RGB PNG, written by hand so the tests need no imaging library."""

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    rows = b"".join(b"\x00" + bytes(rgb) * width for _ in range(height))
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


def write_release_tree(root: Path, release: str) -> Path:
    """Write the unencrypted naoqi-meshes-V40-r<N>/ folder under `root`; returns it."""
    folder = root / meshes.archive_name(release)
    (folder / "meshes").mkdir(parents=True)
    (folder / "textures").mkdir()
    (folder / "textures" / "skin.png").write_bytes(png(2, 2, (230, 230, 225)))
    entries: dict[str, list[dict[str, object]]] = {}
    for visual in meshes.visual_table():
        stem = Path(visual.mesh).stem
        (folder / "meshes" / f"{stem}_0.obj").write_text(TEXTURED_OBJ)
        parts: list[dict[str, object]] = [
            {
                "obj": f"meshes/{stem}_0.obj",
                "texture": "textures/skin.png",
                "rgba": None,
            }
        ]
        if stem == "HeadPitch":
            (folder / "meshes" / f"{stem}_1.obj").write_text(FLAT_OBJ)
            parts.append(
                {
                    "obj": f"meshes/{stem}_1.obj",
                    "texture": None,
                    "rgba": [0.2, 0.2, 0.2, 1],
                }
            )
        entries[visual.mesh] = parts
    manifest = {"format": 1, "release": release, "source": {}, "meshes": entries}
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2))
    (folder / "LICENSE").write_bytes(LICENSE_TEXT)
    (folder / "NOTICE").write_text("Test meshes, our own shapes.\n")
    return folder


class _ZipCrypto:
    """PKWARE's traditional encryption (ZipCrypto), the cipher `zip -e` writes and zipfile reads."""

    def __init__(self, password: bytes) -> None:
        self.keys = [0x12345678, 0x23456789, 0x34567890]
        for byte in password:
            self._update(byte)

    @staticmethod
    def _crc(
        crc: int, byte: int
    ) -> int:  # one CRC-32 step, without zlib's pre/post inversion
        return zlib.crc32(bytes([byte]), crc ^ 0xFFFFFFFF) ^ 0xFFFFFFFF

    def _update(self, byte: int) -> None:
        k0, k1, k2 = self.keys
        k0 = self._crc(k0, byte)
        k1 = ((k1 + (k0 & 0xFF)) * 134775813 + 1) & 0xFFFFFFFF
        k2 = self._crc(k2, k1 >> 24)
        self.keys = [k0, k1, k2]

    def encrypt(self, data: bytes) -> bytes:
        out = bytearray()
        for byte in data:
            temp = (self.keys[2] | 2) & 0xFFFF
            out.append(byte ^ (((temp * (temp ^ 1)) >> 8) & 0xFF))
            self._update(byte)
        return bytes(out)


def zip_encrypted(source_root: Path, folder_name: str, out: Path) -> bytes:
    """Zip `source_root/folder_name` with ZipCrypto under naoqi-viewer's password, as `zip -r -e`
    does: deflated files, unencrypted folder entries. Written here because zipfile can only read
    encrypted entries, and a `zip` command isn't on every machine (Windows)."""
    entries, central = bytearray(), bytearray()
    paths = sorted((source_root / folder_name).rglob("*"))
    for path in [source_root / folder_name, *paths]:
        name = path.relative_to(source_root).as_posix() + ("/" if path.is_dir() else "")
        plain = b"" if path.is_dir() else path.read_bytes()
        crc = zlib.crc32(plain)
        if path.is_dir():
            flags, method, data = 0, 0, b""
        else:
            deflate = zlib.compressobj(9, zlib.DEFLATED, -15)
            cipher = _ZipCrypto(meshes._PASSWORD)
            # The 12-byte header ends with the CRC's high byte, which readers check the password by.
            header = bytes(range(11)) + bytes([crc >> 24])
            data = cipher.encrypt(header + deflate.compress(plain) + deflate.flush())
            flags, method = 0x1, 8
        fields = struct.pack(
            "<HHHHHIIIHH",
            20,
            flags,
            method,
            0,
            0x21,
            crc,
            len(data),
            len(plain),
            len(name),
            0,
        )
        central += struct.pack("<IH", 0x02014B50, 20) + fields
        central += struct.pack("<HHHII", 0, 0, 0, 0, len(entries)) + name.encode()
        entries += struct.pack("<I", 0x04034B50) + fields + name.encode() + data
    count = len(paths) + 1
    end = struct.pack(
        "<IHHHHIIH", 0x06054B50, 0, 0, count, count, len(central), len(entries), 0
    )
    out.write_bytes(bytes(entries + central + end))
    return out.read_bytes()


@dataclass
class FakeRelease:
    """A pinned fake release: its archive bytes, and the URLs fetch downloaded."""

    release: str
    archive: bytes
    archive_path: Path
    downloads: list[str]


def install_fake_release(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, release: str = "r0"
) -> FakeRelease:
    """Pin a test-built release, point the data directory into tmp_path, serve downloads."""
    build = tmp_path / "build"
    write_release_tree(build, release)
    archive_path = tmp_path / f"{meshes.archive_name(release)}.zip"
    archive = zip_encrypted(build, meshes.archive_name(release), archive_path)
    downloads: list[str] = []

    def download(url: str) -> bytes:
        downloads.append(url)
        if url == meshes.license_url(release):
            return LICENSE_TEXT
        if url == meshes.archive_url(release):
            return archive
        raise meshes.MeshesError(f"download failed: {url}: not found")

    monkeypatch.setattr(meshes, "RELEASE", release)
    monkeypatch.setattr(meshes, "ARCHIVE_SHA256", hashlib.sha256(archive).hexdigest())
    monkeypatch.setattr(
        meshes, "LICENSE_SHA256", hashlib.sha256(LICENSE_TEXT).hexdigest()
    )
    monkeypatch.setattr(meshes, "_download", download)
    data = tmp_path / "data" / "meshes"
    monkeypatch.setattr(meshes, "data_dir", lambda: data)
    return FakeRelease(release, archive, archive_path, downloads)
