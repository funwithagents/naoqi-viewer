"""Camera display: the two head cameras read from NAOqi's ALVideoDevice, and the pictures the
window draws of them (specs/cameras.md).

`CameraSource` is the pose source's sibling for images: its own thread connects, subscribes both
cameras once, polls `getImagesRemote` on a fixed schedule, keeps the newest image per camera in a
slot the window reads without blocking, and reconnects with backoff. The pure functions below it
(`decode_images`, `picture_rects`, `resample_nearest`, `framed`, `placeholder`) are what the
tests pin without a NAOqi or a window.
"""

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Literal, Protocol

import numpy as np

from naoqi_viewer.source import connect, normalize_url

_log = logging.getLogger(__name__)

type Camera = Literal["top", "bottom"]
type Rect = tuple[
    int, int, int, int
]  # left, bottom, width, height; origin bottom-left (MjrRect)

CAMERAS: dict[Camera, int] = {"top": 0, "bottom": 1}  # ALVideoDevice camera indexes
CAMERA_NAMES: tuple[Camera, ...] = (
    "top",
    "bottom",
)  # the order of the pictures, top first
DISPLAY_RESOLUTION = 1  # kQVGA, 320x240
DISPLAY_COLORSPACE = 11  # kRGBColorSpace
DISPLAY_FPS = 10
SUBSCRIBER_NAME = "naoqi_viewer"
IMAGE_STALE_AFTER = 1.0  # s: ten frame periods at the display rate
UNSUBSCRIBE_TIMEOUT = (
    1.0  # s: how long close() gives NAOqi to acknowledge the unsubscribe
)

SUBSCRIPTION_CHECK = (
    2.0  # s: while no image comes, how often to check the subscription exists
)

PICTURE_FRACTION = 0.25  # of the view's width
PICTURE_MARGIN = (
    0.02  # of the view's width: the inset from the corner, the gap between pictures
)
PICTURE_MIN_SIDE = 32  # px: smaller than this, nothing is drawn
PICTURE_BORDER = 2  # px
_PICTURE_ASPECT = 3 / 4  # height / width: the cameras' 4:3
_BORDER_VALUE = 220  # light grey
_PLACEHOLDER_VALUE = 40  # dark grey

_RECONNECT_FIRST_DELAY = 0.5  # s, doubling up to _RECONNECT_MAX_DELAY (as source.py)
_RECONNECT_MAX_DELAY = 5.0
_ASYNC_POLL = 0.02

_INDEX_TO_CAMERA: dict[int, Camera] = {
    index: camera for camera, index in CAMERAS.items()
}


@dataclass(frozen=True)
class CameraImage:
    camera: Camera
    image: np.ndarray  # (height, width, 3) uint8 RGB, row 0 = top of the image
    seq: int  # increments with each poll that returned images, starting at 1
    received_at: float  # time.monotonic() when the images arrived
    naoqi_time: float | None  # NAOqi's own timestamp of the image, s (robot clock)


class ImageSource(Protocol):
    def latest(self, camera: Camera) -> CameraImage | None: ...  # never blocks

    def rate(self) -> float: ...  # polls that returned images, per second

    def close(self) -> None: ...


def decode_images(
    reply: Any, *, received_at: float, seq: int
) -> dict[Camera, CameraImage]:
    """The images in a `getImagesRemote` reply, by camera.

    Each image NAOqi returns is a list: width, height, layers, colorspace, timestamp seconds and
    microseconds, the pixel bytes, the camera index, then the field-of-view angles. An entry with
    no bytes, or a byte count that isn't width x height x 3, or an unknown camera, is skipped:
    that is NAOqi's answer for a camera with nothing to serve.
    """
    images: dict[Camera, CameraImage] = {}
    if not isinstance(reply, list | tuple):
        return images
    for entry in reply:
        if not isinstance(entry, list | tuple) or len(entry) < 8:
            continue
        width, height, _layers, _colorspace, seconds, micros, data, index = entry[:8]
        camera = _INDEX_TO_CAMERA.get(index)
        if camera is None or not isinstance(data, bytes | bytearray | memoryview):
            continue
        if not (isinstance(width, int) and isinstance(height, int)):
            continue
        pixels = np.frombuffer(data, dtype=np.uint8)
        if pixels.size != width * height * 3 or pixels.size == 0:
            continue
        naoqi_time = None
        if isinstance(seconds, int | float) and isinstance(micros, int | float):
            naoqi_time = float(seconds) + float(micros) / 1e6
        images[camera] = CameraImage(
            camera,
            pixels.reshape(height, width, 3).copy(),
            seq,
            received_at,
            naoqi_time,
        )
    return images


class CameraSource:
    """Polls both head cameras of a NAOqi's ALVideoDevice on its own thread.

    Never blocks: connecting and subscribing happen on the thread. On any error it keeps the
    last images and reconnects with backoff until `close()`, which also unsubscribes.
    """

    def __init__(self, url: str, *, fps: int = DISPLAY_FPS) -> None:
        self._url = normalize_url(url)
        self._fps = fps
        self._period = 1.0 / fps
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: dict[Camera, CameraImage] = {}
        self._seq = 0
        self._received: deque[float] = deque()
        self._cleared = False  # unsubscribeAllInstances done, once per source
        self._name: str | None = None  # our subscription's name, as NAOqi returned it
        self._thread = threading.Thread(
            target=self._run, name=f"CameraSource {self._url}", daemon=True
        )
        self._thread.start()

    def latest(self, camera: Camera) -> CameraImage | None:
        with self._lock:
            return self._latest.get(camera)

    def rate(self) -> float:
        now = time.monotonic()
        with self._lock:
            while self._received and self._received[0] < now - 1.0:
                self._received.popleft()
            return float(len(self._received))

    def close(self) -> None:
        self._stop.set()
        self._thread.join()

    def _run(self) -> None:
        delay = _RECONNECT_FIRST_DELAY
        while not self._stop.is_set():
            session = None
            video = None
            try:
                session = connect(self._url, cancel=self._stop)
                video = session.service("ALVideoDevice")
                self._subscribe(video)
                delay = _RECONNECT_FIRST_DELAY
                self._poll(video)
            # qi raises RuntimeError; a malformed NAOqi reply surfaces as ValueError/IndexError/TypeError.
            except (
                RuntimeError,
                ConnectionError,
                ValueError,
                IndexError,
                TypeError,
            ) as exc:
                if self._stop.is_set():
                    break
                _log.warning(
                    "cameras of NAOqi at %s: %s; retrying in %.1f s",
                    self._url,
                    exc,
                    delay,
                )
            finally:
                if session is not None:
                    if video is not None:
                        self._unsubscribe(video)
                    session.close()
            if self._stop.wait(delay):
                break
            delay = min(delay * 2, _RECONNECT_MAX_DELAY)

    def _subscribe(self, video: Any) -> None:
        if not self._cleared:
            # What a crashed viewer left behind. Once only: a second viewer on the same NAOqi
            # clears ours when it starts, and we subscribe again without clearing its.
            video.unsubscribeAllInstances(SUBSCRIBER_NAME)
            self._cleared = True
        elif self._name is not None:
            # Our previous subscription, if NAOqi still has it after an outage of ours.
            video.unsubscribe(self._name)
            self._name = None
        indexes = [CAMERAS[camera] for camera in CAMERA_NAMES]
        self._name = str(
            video.subscribeCameras(
                SUBSCRIBER_NAME,
                indexes,
                [DISPLAY_RESOLUTION] * len(indexes),
                [DISPLAY_COLORSPACE] * len(indexes),
                self._fps,
            )
        )
        _log.info(
            "subscribed to the cameras of %s as %s (QVGA, RGB, %d fps)",
            self._url,
            self._name,
            self._fps,
        )

    def _unsubscribe(self, video: Any) -> None:
        """Best effort, bounded: a NAOqi that went away must not hold up a close."""
        name = self._name
        if name is None:
            return
        try:
            future = video.unsubscribe(name, _async=True)
            deadline = time.monotonic() + UNSUBSCRIBE_TIMEOUT
            while not future.isFinished() and time.monotonic() < deadline:
                time.sleep(_ASYNC_POLL)
            if future.isFinished() and not future.hasError():
                self._name = None
        except RuntimeError:
            pass

    def _poll(self, video: Any) -> None:
        """Poll until close() (returns) or a qi error (raises)."""
        assert self._name is not None
        due = time.monotonic()
        checked = due
        while not self._stop.is_set():
            reply = video.getImagesRemote(self._name)
            now = time.monotonic()
            images = decode_images(reply, received_at=now, seq=self._seq + 1)
            if images:
                self._seq += 1
                with self._lock:
                    self._latest.update(images)
                    self._received.append(now)
            elif now - checked >= SUBSCRIPTION_CHECK:
                # NAOqi answers an unknown name with an empty reply, not an error: a camera with
                # nothing to serve looks the same as a subscription another viewer cleared.
                checked = now
                if self._name not in list(video.getSubscribers()):
                    raise RuntimeError(f"subscription {self._name} is gone")
            # Ticks are due on a fixed schedule (as source.py): no catching up after an overrun.
            due += self._period
            remaining = due - time.monotonic()
            if remaining <= 0:
                due = time.monotonic()
            elif self._stop.wait(remaining):
                return


# --- The pictures (pure) --------------------------------------------------------------


def picture_rects(viewport: Rect) -> list[Rect] | None:
    """Where the two pictures go in `viewport` (left, bottom, width, height, framebuffer pixels,
    origin bottom-left): two 4:3 rectangles, each PICTURE_FRACTION of the view's width with even
    sides, stacked in the top-right corner, inset PICTURE_MARGIN of the view's width from the
    corner and from each other, the top camera's first. None when a side would be under
    PICTURE_MIN_SIDE or the two don't fit: nothing is drawn then."""
    left, bottom, view_width, view_height = viewport
    width = int(view_width * PICTURE_FRACTION) // 2 * 2
    height = int(width * _PICTURE_ASPECT) // 2 * 2
    inset = int(view_width * PICTURE_MARGIN)
    if (
        min(width, height) < PICTURE_MIN_SIDE
        or width + inset > view_width
        or 2 * height + 3 * inset > view_height
    ):
        return None
    x = left + view_width - width - inset
    top_y = bottom + view_height - inset - height
    return [(x, top_y, width, height), (x, top_y - inset - height, width, height)]


def resample_nearest(image: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    """`image` (height x width x channels) at `size` (width, height) by nearest neighbour: the
    first and last output pixels take the first and last source pixels. Always a contiguous
    copy."""
    width, height = size
    source_height, source_width = image.shape[:2]
    if (source_width, source_height) == (width, height):
        return np.array(image, order="C")
    rows = np.rint(np.arange(height) * (source_height - 1) / max(height - 1, 1))
    cols = np.rint(np.arange(width) * (source_width - 1) / max(width - 1, 1))
    return np.ascontiguousarray(image[rows.astype(int)[:, None], cols.astype(int)])


def framed(image: np.ndarray) -> np.ndarray:
    """`image` with its PICTURE_BORDER outer pixels light grey; modified in place and returned."""
    b = PICTURE_BORDER
    image[:b], image[-b:] = _BORDER_VALUE, _BORDER_VALUE
    image[:, :b], image[:, -b:] = _BORDER_VALUE, _BORDER_VALUE
    return image


def placeholder(size: tuple[int, int]) -> np.ndarray:
    """A dark grey framed picture of `size` (width, height): a camera's slot before its first
    image."""
    width, height = size
    return framed(np.full((height, width, 3), _PLACEHOLDER_VALUE, dtype=np.uint8))
