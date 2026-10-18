---
code:
  - src/naoqi_viewer/cameras.py
  - src/naoqi_viewer/viewer.py
  - src/naoqi_viewer/viewer_process.py
  - src/naoqi_viewer/config.py
tests:
  - tests/test_cameras.py
  - tests/test_viewer.py
  - tests/mock_naoqi.py
  - tests-e2e/test_cameras_live.py
---

# Camera display

**Status:** Implemented

## Purpose

The window shows what the robot's two head cameras see, as pictures in its top-right corner. The frames come from NAOqi's `ALVideoDevice` in **both modes**, never from the viewer's own renderers: the pictures show *what is in NAOqi*, which is what any NAOqi client would get. On a real robot that is the robot's cameras. In sim mode it is whatever naoqi-sim injected, so the pictures also check naoqi-sim's camera path end to end: the viewer's render travels to naoqi-sim, into `ALVideoDevice`, and back to this window. If that path stalls, the pictures go stale while the 3D view keeps moving, and the gap is visible. The display is a diagnostic for the person watching: it is read-only, costs NAOqi one subscription, and is off by default.

The alternative, drawing the viewer's own offscreen renders in sim mode, was rejected: it would show frames nobody else sees, and it would need a second source of frames for mirror mode. One reader, one picture, both modes.

## Decided

### Two readers of NAOqi

The pose source ([source.md](source.md)) reads `ALMotion`. The camera source, `CameraSource` in `cameras.py`, reads `ALVideoDevice`. They are the same shape, a thread of their own that connects through `connect()`, polls on a fixed schedule, keeps the newest result in a slot the window reads without blocking, and reconnects with backoff, but they are separate objects with separate qi sessions, because each owns its own connection lifecycle and `NaoqiSource` is not touched by this feature. A viewer whose config has `cameras_display: false` (the default) never builds a `CameraSource` and never calls `ALVideoDevice`, so a default viewer leaves the robot's camera subscriptions exactly as it found them.

### `CameraSource(url, *, fps=DISPLAY_FPS)`

```python
CAMERAS = {"top": 0, "bottom": 1}        # ALVideoDevice camera indexes
DISPLAY_RESOLUTION = 1                   # kQVGA, 320x240
DISPLAY_COLORSPACE = 11                  # kRGBColorSpace
DISPLAY_FPS = 10
SUBSCRIBER_NAME = "naoqi_viewer"

@dataclass(frozen=True)
class CameraImage:
    camera: Literal["top", "bottom"]
    image: np.ndarray          # (height, width, 3) uint8 RGB, row 0 = top of the image
    seq: int                   # increments with each poll that returned images, starting at 1
    received_at: float         # time.monotonic() when the images arrived
    naoqi_time: float | None   # NAOqi's own timestamp of the image, s (robot clock)

class ImageSource(Protocol):
    def latest(self, camera: str) -> CameraImage | None   # never blocks; None before the first image
    def rate(self) -> float                               # polls that returned images, per second
    def close(self) -> None

class CameraSource(ImageSource): ...
```

- **Never blocks**: the constructor only starts the thread. `latest()` is read by the window loop every frame and never waits on NAOqi. Older images are dropped, never queued.
- **One subscription for both cameras**: on connect the source calls `ALVideoDevice.subscribeCameras(SUBSCRIBER_NAME, [0, 1], [QVGA, QVGA], [RGB, RGB], fps)` and keeps the name NAOqi returns (`naoqi_viewer_<n>`). Each poll is one `getImagesRemote(name)`, which returns both images from one round trip, so the two pictures are of the same instant and a poll costs one call, not two.
- **Image decoding**: each image NAOqi returns is a list whose items 0, 1, 2 are the width, height and layer count, items 4 and 5 the timestamp in seconds and microseconds, item 6 the pixel bytes and item 7 the camera index. The source reshapes the bytes to `(height, width, 3)`. A reply with no images, an image with no bytes or a size that doesn't match (NAOqi's answer when a camera has nothing to serve: a plain `naoqi-bin`, or naoqi-sim before its first injection) counts as no image for that camera: `latest()` keeps returning the last one, or `None` before any.
- **Polling**: ticks are due on a fixed schedule at `fps`, with the same rule as `NaoqiSource` when a tick overruns. `rate()` counts the polls that returned at least one image, over the last second.
- **Age** = `time.monotonic() - image.received_at`, computed by the caller. Host receive time, as for poses, because the robot's clock isn't the host's. `naoqi_time` is kept for a caller who wants it, and unused by the window.
- **Subscriptions are the trap.** NAOqi keeps a subscription alive after its client disconnects, suffixes each new one (`_0` to `_5`) and refuses the seventh with the same name. So:
  - at its first connection the source calls `unsubscribeAllInstances(SUBSCRIBER_NAME)` once, which clears what a crashed viewer left behind, then subscribes;
  - after an error or a reconnect it subscribes again without clearing (it only unsubscribes its own previous name, in case NAOqi still has it). Two viewers on one NAOqi therefore converge: the second clears the first's subscription when it starts and receives the same suffix back, so the first's polls are answered through the second's subscription until the second closes and unsubscribes it; then the first notices (next point) and subscribes again. Neither clears again;
  - NAOqi answers a poll on a name it no longer knows with an empty reply, not an error, which looks the same as a camera with nothing to serve. So while no image comes, the source checks every `SUBSCRIPTION_CHECK = 2` s that its name is still in `getSubscribers()`, and treats a missing name as an error, which the reconnection path turns into a new subscription;
  - `close()` unsubscribes its name, asynchronously with a one-second bound, so closing a viewer never waits on a NAOqi that went away.
- **Robustness**: on a qi error, a disconnection or a failed `connect()`, the source keeps its last images (their age grows, which the window shows) and reconnects in the background with the pose source's backoff (0.5 s doubling to a 5 s cap) until `close()`, then subscribes again.
- **Read-only**: subscribing is the one thing the source does to NAOqi's state, and it undoes it.
- **The source serves images as NAOqi does**: never flipped, row 0 on top and the robot's left on the left. Mirroring is a choice of the window alone (below).

### The pictures (window only)

The window loop ([viewer.md](viewer.md)) draws the pictures through the passive viewer's `set_images` (MuJoCo 3.3.1 and later; naoqi-viewer requires 3.15, so there is no older case to degrade to). The layout is a pure function the tests can check without a window:

```python
PICTURE_FRACTION = 0.25     # of the view's width
PICTURE_MARGIN = 0.02       # of the view's width: the inset from the corner and the gap between pictures
PICTURE_MIN_SIDE = 32       # px: smaller than this, nothing is drawn
PICTURE_BORDER = 2          # px, light grey

def picture_rects(viewport: tuple[int, int, int, int]) -> list[tuple[int, int, int, int]] | None
def resample_nearest(image: np.ndarray, size: tuple[int, int]) -> np.ndarray
```

- **Where**: two 4:3 pictures (the cameras' aspect) stacked in the top-right corner of the 3D view, the **top camera above the bottom camera**, the order of the cameras on the head. Each is `PICTURE_FRACTION` of the view's width, both sides even, inset `PICTURE_MARGIN` of the view's width from the corner and separated by the same gap. `picture_rects` returns `[top, bottom]` as `(left, bottom, width, height)` in framebuffer pixels with a bottom-left origin, the convention of `MjrRect` and `handle.viewport`; it returns `None`, and nothing is drawn, when a side would be under `PICTURE_MIN_SIDE` or the two pictures don't fit in the view. It is recomputed from `handle.viewport` on every frame, so a window resize (and a Retina display, where the framebuffer is twice the window's points) keeps the pictures in their corner.
- **Mirrored by default**: the window's default view faces the robot (azimuth 150, [viewer.md](viewer.md)), so the robot's left is on the watcher's right. An image as NAOqi serves it has the robot's left on its left, which contradicts the 3D view beside it. So the pictures are drawn mirrored left to right, like a car's mirror: an object on the robot's left shows on the right of the 3D view and on the right of the picture. With a webcam standing in for the robot's eyes (naoqi-sim's webcam mode), a person in front of it sees themselves as in a mirror. The viewer can't tell where the frames come from, so the rule is the same for every source. `cameras_mirror: false` in the config ([config.md](config.md)) draws them as NAOqi serves them, and **F11** switches between the two while the window runs. Mirroring changes only what the window draws: the source, NAOqi and every NAOqi client keep the unflipped images.
- **What**: the latest image of each camera, flipped left to right when mirrored, resampled to its rectangle by nearest neighbour (`resample_nearest`, whose first and last output pixels take the first and last source pixels), with a `PICTURE_BORDER` light frame so a picture stands out from a scene of its own colours. `set_images` takes row 0 as the top of the image and flips it for OpenGL itself. Before a camera's first image, its slot shows a dark grey placeholder with the same frame, so the person sees that the display is on and that nothing has arrived. A camera whose images stopped keeps its last picture, as the robot keeps its last pose.
- **When**: the images are set when a new image arrived for either camera, when the viewport changed, when the display was just turned on, or when F11 switched the mirroring; `set_images` copies, and resetting two unchanged pictures at 60 Hz would be a waste. `set_images` is called from the window loop's thread, outside `handle.lock()`: it waits for the viewer's render thread, which needs that lock, so calling it under the lock would deadlock. The same rule already governs `set_texts`.
- **Status lines**: the status overlay ([viewer.md](viewer.md)) gains two lines, `top camera` and `bottom camera`, each with the image's age in ms, `NO IMAGE` before the first, or `STALE` once the age passes `IMAGE_STALE_AFTER = 1.0` s (ten frame periods at the display rate: a real robot's Wi-Fi jitters more than its joint reads). While the pictures are mirrored the labels read `top camera (mirrored)` and `bottom camera (mirrored)`, so nobody mistakes the window's picture for what NAOqi serves. They are part of the status overlay, so F9 hides them with it.
- **F10** toggles the pictures (`clear_images` when hidden; the source keeps polling, so showing them again is immediate). **F11** switches the mirroring. F9 keeps the status overlay, and letters and digits remain MuJoCo's.
- **No window, no pictures**: a headless viewer has nothing to draw on, so `cameras_display: true` with `headless: true` is a `ConfigError` ([config.md](config.md)), as for `ghost`.

### Config

`"cameras_display": true` at the top level of the config ([config.md](config.md)), a boolean, `false` by default, valid in both modes. Next to it, `"cameras_mirror"`, a boolean, `true` by default: whether the pictures start mirrored (F11 switches them later). It has no effect without the display, so it is not an error with `headless`. The viewer process ([api.md](api.md)) builds a `CameraSource` on the config's URL when it is on, hands it to the window loop, and closes it with the pose source.

### What the pictures show, per target

| Target | Pictures |
|---|---|
| Real NAO (mirror mode) | The robot's cameras, QVGA at 10 fps, about 4.6 MB/s over its link (two 230 kB images, ten times a second) |
| naoqi-sim (sim mode) | The frames naoqi-sim injected into `ALVideoDevice`: this viewer's own renders after the round trip. The pictures lag the 3D view by the injection period plus the display period, a few frames; that lag is what a NAOqi client sees, and the point of showing it |
| Plain `naoqi-bin` (virtual) | `NO IMAGE`: nothing serves the cameras, and the display says so |

### Testing

**Fast tier** (`tests/test_cameras.py`, `tests/test_viewer.py`, the mock NAOqi of [testing.md](testing.md) with its `ALVideoDevice`):

- `CameraSource` on the mock: subscribes both cameras at QVGA, RGB and the display rate, clears its name's instances once at startup and not on a resubscribe, decodes the scripted images into `(240, 320, 3)` RGB arrays with row 0 on top, returns the newest image per camera, reports `None` for a camera the mock serves nothing for, keeps the last image through a failing call and a dropped session, subscribes again after the mock restarts, notices a subscription removed behind its back and subscribes again (two sources on one mock: both get images while the second runs, and the first subscribes again once the second has closed), closes promptly when NAOqi is gone, and unsubscribes on `close()` (the mock's subscription list is empty afterwards). A `CameraSource` is never built with `cameras_display` off (the viewer process's wiring, tested with the mock's call log: no `ALVideoDevice` call).
- `picture_rects`: both pictures 4:3 with even sides, in the corner at the margin, the top one above, `None` on a view too small or too short for two; `resample_nearest` keeps the corners and the size.
- `ViewerState` with a fake `ImageSource`: the pictures it asks for are the two rectangles with the resampled images, the placeholder before an image, a redraw only when something changed, nothing with F10 off, the status lines reading `NO IMAGE`, an age, then `STALE`. Mirroring: with the default the picture of an image whose left half differs from its right has those halves swapped and the labels end in `(mirrored)`; with `cameras_mirror` off, or after F11, the picture keeps NAOqi's orientation and the plain labels; F11 asks for a redraw.
- `config.py`: the fields' defaults, round trip, type checks, the `ConfigError` of `cameras_display` with `headless`, and `cameras_mirror` accepted with `headless`.

**Live tier** (`tests-e2e/test_cameras_live.py`, on `NAOQI_URL`): a `CameraSource` against the real NAOqi subscribes (its name appears in `ALVideoDevice.getSubscribers()`), polls without error for a second, and after `close()` its name is gone. Whether images arrive depends on the target, so the test asserts on the subscription, and on the images' shape only when some arrive. A windowed run also opens a sim viewer with `cameras_display` on and checks it launches and closes cleanly.

## Open questions

1. **Resolution and rate knobs**: QVGA at 10 fps is a guess at what a NAO V5's Wi-Fi sustains next to the pose polling. If a real robot needs less, or a user wants VGA, `cameras_display` would also accept an object (`{"fps": 5, "resolution": "qqvga"}`), with `true` staying the shorthand for the defaults. Deferred until measured on a robot.
2. **Camera fields in `ViewerStatus`**: a caller might want the images' age through `status()` as it gets the pose's. One more protocol field; deferred until a caller asks.
3. **NAOqi 2.8**: `subscribeCameras` and `getImagesRemote` exist from NAOqi 1.14 on, and 2.8 keeps them; validated on 2.1 only, like the rest ([_overview.md](_overview.md)).
