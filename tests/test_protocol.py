import socket
import threading

import pytest

from naoqi_viewer import protocol


def test_messages_round_trip_with_and_without_payload():
    left, right = socket.socketpair()
    with left, right:
        protocol.write_message(left, {"op": "status", "id": 1})
        protocol.write_message(
            left, {"op": "frame", "id": 2, "shape": [2, 3]}, b"\x00\x01\x02\x03\x04\x05"
        )
        assert protocol.read_message(right) == ({"op": "status", "id": 1}, bytearray())
        header, payload = protocol.read_message(right)
        assert header == {"op": "frame", "id": 2, "shape": [2, 3]}
        assert payload == b"\x00\x01\x02\x03\x04\x05"


def test_large_payloads_arrive_whole():
    payload = bytes(range(256)) * 4096  # 1 MiB, more than one socket buffer
    left, right = socket.socketpair()
    with left, right:
        sender = threading.Thread(
            target=protocol.write_message, args=(left, {"id": 7}, payload)
        )
        sender.start()
        header, received = protocol.read_message(right)
        sender.join()
    assert header == {"id": 7}
    assert received == payload


def test_framing_is_big_endian_lengths_then_json_then_payload():
    assert (
        protocol.encode({"a": 1}, b"xy")
        == b"\x00\x00\x00\x07\x00\x00\x00\x02" + b'{"a":1}' + b"xy"
    )


def test_a_closed_connection_is_eof_even_mid_message():
    left, right = socket.socketpair()
    with right:
        left.sendall(protocol.encode({"id": 1}, b"abcdef")[:-3])
        left.close()
        with pytest.raises(EOFError):
            protocol.read_message(right)


def test_ready_line():
    assert protocol.ready_line(51234) == "NAOQI_VIEWER_READY port=51234 protocol=1"
