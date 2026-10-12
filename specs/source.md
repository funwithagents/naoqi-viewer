---
code:
  - src/naoqi_viewer/source.py
tests:
  - tests/test_source.py
---

# Pose sources

**Status:** Implemented

## Purpose

Where the robot's pose comes from. The viewer asks a source for the latest sample on every frame and never waits on NAOqi. Everything qi-related on the read side (connecting, retrying, polling, identifying the target) lives here, behind one small interface.

## Decided

### Interface

```python
@dataclass(frozen=True)
class Sample:
    seq: int                       # increments with each new sample, starting at 1
    pose: NaoPose                  # measured pose (model.md)
    commanded: NaoPose | None      # commanded angles (same torso), when requested
    received_at: float             # time.monotonic() when the sample arrived

@dataclass(frozen=True)
class TargetInfo:
    url: str
    target: Literal["real", "naoqi-sim", "virtual"]
    naoqi_version: str | None

class PoseSource(Protocol):
    info: TargetInfo | None                  # None until the first connection
    def latest(self) -> Sample | None        # never blocks; None before the first sample
    def rate(self) -> float                  # samples per second over the last second
    def close(self) -> None
```

- `latest()` is called by the render loop and never blocks. Sources produce samples on their own thread and swap in the newest one; older samples are dropped, never queued.
- Data age = `time.monotonic() - sample.received_at`, computed by the caller. Host receive time is used because NAOqi's `getAngles` carries no timestamp.

### `connect(url, *, cancel=None) -> qi.Session`

- Accepts `tcp://host:port`, `host:port`, or `host` alone (port 9559 implied); `normalize_url(url)` does that mapping.
- Retries the connection up to 5 times, with 0.2 s, 0.4 s, 0.8 s… backoff. The libqi 3 wheels fail about one connect in three against NAOqi 2.1 with an instant `disconnected` error (root cause in the fork unknown), and naoqi-mcp already works around it the same way. After the last attempt it raises `ConnectionError`.
- `cancel` (a `threading.Event`) cuts the retries short with a `ConnectionError`, so `NaoqiSource.close()` doesn't wait out a backoff.
- Shared by `NaoqiSource`, [check_model.md](check_model.md) and the viewer process ([api.md](api.md)).

### `NaoqiSource(url, rate_hz=50, commanded=False)`

- **Never blocks**: the constructor only starts the source's thread, which connects (through `connect()`), then polls. An unreachable NAOqi is handled like a lost connection (see Robustness), so a viewer opens even when NAOqi isn't there yet ([api.md](api.md)).
- **On connect**:
  - reads `ALMotion.getBodyNames("Body")` once and keeps that order;
  - logs names unknown to the model (`model.JOINT_NAMES`) once per connection, and leaves them out of the samples' poses; joints NAOqi doesn't report stay at 0;
  - identifies the target:
    - **`naoqi-sim`** if the `NaoqiSim` service exists, NAOqi version from the ALMemory key `NaoqiSim/NaoqiVersion` (not `NaoqiSim/Version`, which holds naoqi-sim's own version);
    - **`real`** if `ALSystem` exists, version from `ALSystem.systemVersion()`;
    - **`virtual`** otherwise: a plain desktop `naoqi-bin` has neither, and its version is unknown (`None`).
- **Polling thread**: each tick calls `ALMotion.getAngles("Body", True)` and `ALMotion.getTransform("Torso", 1, True)` (frame 1 = world). With `commanded=True` it also calls `getAngles("Body", False)`. Ticks are due on a fixed schedule at `rate_hz` (timing each tick from its actual start would lose the sleep overshoot, about 3 ms per 20 ms on macOS, and run at ~43 Hz); if a tick overruns, the next one starts immediately and the schedule restarts from there, rather than catching up.
- **Robustness**: on a qi error, a disconnection or a failed `connect()`, the source keeps its last sample (the data age grows, which the viewer shows) and reconnects in the background with backoff (0.5 s doubling to a 5 s cap) until `close()`. After reconnecting it reads `getBodyNames` and the target again, because NAOqi may have restarted.
- **Read-only**: the source never calls anything that changes robot state.

## Open questions

1. **Polling cost**: two or three qi calls per tick at 50 Hz. If a real robot can't sustain the 30 Hz exit target, batching reads through `ALMemory.getListData` on the joint sensor keys is the fallback. It would need the per-version key maps that naoqi-bridge owns, so it's deferred until measured.
2. **Virtual-robot sensors**: whether `getAngles(..., True)` on a virtual robot returns commanded values (unconfirmed). It doesn't change this design; the ghost is simply identical to the robot there.
