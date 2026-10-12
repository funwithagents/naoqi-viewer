"""A fake viewer process for client tests: speaks the protocol, no MuJoCo, no window, no NAOqi.

`NaoqiViewer.launch()` runs it in place of `python -m naoqi_viewer.viewer_process <config>` (see test_client.py).
NAOQI_FAKE_VIEWER picks a scripted behavior; NAOQI_FAKE_VIEWER_OPS names a file where each received op
is appended, one per line.
"""

import json
import os
import socket
import sys
import time

import numpy as np

from naoqi_viewer import protocol

config = json.loads(sys.argv[1])  # NaoqiViewerConfig.to_dict()
behavior = os.environ.get("NAOQI_FAKE_VIEWER", "")
ops_file = os.environ.get("NAOQI_FAKE_VIEWER_OPS")
config_file = os.environ.get("NAOQI_FAKE_VIEWER_CONFIG")
if config_file:
    with open(config_file, "w") as f:
        json.dump(config, f)


def log(line: str) -> None:
    print(line, file=sys.stderr, flush=True)


if behavior == "error":
    log("INFO naoqi_viewer.model: loading the scene")
    print(
        f"{protocol.ERROR_PREFIX} unknown scene 'nope'; bundled scenes: empty, table",
        flush=True,
    )
    sys.exit(1)
if behavior == "crash":
    log("Traceback (most recent call last):")
    log("RuntimeError: GLFW could not open a window")
    sys.exit(3)
if behavior == "hang":
    log("INFO fake: never ready")
    time.sleep(60)
    sys.exit(0)

listener = socket.create_server(("127.0.0.1", 0))
print(
    "[W] 1791534114.227813 36619 qi.path.sdklayout: No Application was created",
    flush=True,
)
print(protocol.ready_line(listener.getsockname()[1]), flush=True)
log("INFO naoqi_viewer.source: connected to the fake NAOqi")
log("WARNING naoqi_viewer.viewer: a warning from the viewer process")
log("[I] 1791534114.3 qimessaging.session: a native log line")
connection, _ = listener.accept()

answered = 0
try:
    while True:
        request, _ = protocol.read_message(connection)
        op = request.get("op")
        if ops_file:
            with open(ops_file, "a") as f:
                f.write(f"{op}\n")
        reply: protocol.Header = {"id": request.get("id"), "ok": True}
        payload = b""
        if op == "hello":
            reply.update(
                protocol=int(
                    os.environ.get("NAOQI_FAKE_VIEWER_PROTOCOL", protocol.PROTOCOL)
                ),
                naoqi_viewer="0.1.0",
                mode=config["mode"],
            )
        elif op == "status":
            reply.update(
                mode=config["mode"],
                target="naoqi-sim",
                naoqi_version="0.3.0",
                url=config["naoqi"]["url"],
                variant="placeholder",
                rate=50.0,
                pose_seq=12,
                data_age=0.008,
            )
        elif op == "camera_frame" and config["mode"] == "sim":
            width, height = request["width"], request["height"]
            # Each pixel encodes its row in red and its column in green, and the camera in blue.
            image = np.zeros((height, width, 3), dtype=np.uint8)
            image[:, :, 0] = (np.arange(height) % 256)[:, None]
            image[:, :, 1] = (np.arange(width) % 256)[None, :]
            image[:, :, 2] = 1 if request["camera"] == "top" else 2
            reply.update(
                shape=list(image.shape),
                dtype="uint8",
                camera=request["camera"],
                pose_seq=12,
                pose_age=0.02,
            )
            payload = image.tobytes()
        elif op == "camera_frame":
            reply = {"id": request.get("id"), "ok": False, "error": "mode"}
        elif op == "stop":
            protocol.write_message(connection, reply)
            break
        else:
            reply = {
                "id": request.get("id"),
                "ok": False,
                "error": f"unknown op {op!r}",
            }
        protocol.write_message(connection, reply, payload)
        answered += 1
        if behavior.startswith("close-after-") and answered == int(behavior[12:]):
            break  # as if the window had been closed
except EOFError:
    pass
sys.exit(0)
