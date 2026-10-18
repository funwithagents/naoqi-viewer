"""The loopback protocol between a caller and its viewer process.

Every message is `u32 header_len | u32 payload_len | header (UTF-8 JSON) | payload`, with
big-endian lengths. Shared by client.py and viewer_process.py; standard library only.
"""

import json
import socket
import struct
from typing import Any

PROTOCOL = 2
READY_PREFIX = "NAOQI_VIEWER_READY"
ERROR_PREFIX = "NAOQI_VIEWER_ERROR"

_LENGTHS = struct.Struct(">II")

Header = dict[str, Any]


def encode(header: Header, payload: bytes = b"") -> bytes:
    body = json.dumps(header, separators=(",", ":")).encode()
    return _LENGTHS.pack(len(body), len(payload)) + body + payload


def write_message(sock: socket.socket, header: Header, payload: bytes = b"") -> None:
    sock.sendall(encode(header, payload))


def _read_exactly(sock: socket.socket, size: int) -> bytearray:
    buffer = bytearray(size)
    view = memoryview(buffer)
    received = 0
    while received < size:
        count = sock.recv_into(view[received:], size - received)
        if count == 0:
            raise EOFError("connection closed")
        received += count
    return buffer


def read_message(sock: socket.socket) -> tuple[Header, bytearray]:
    """Read one message; EOFError when the peer has closed the connection.

    The payload is a fresh, writable buffer, so a frame can wrap it without another copy.
    """
    header_len, payload_len = _LENGTHS.unpack(_read_exactly(sock, _LENGTHS.size))
    header = json.loads(_read_exactly(sock, header_len))
    payload = _read_exactly(sock, payload_len) if payload_len else bytearray()
    return header, payload


def ready_line(port: int) -> str:
    return f"{READY_PREFIX} port={port} protocol={PROTOCOL}"
