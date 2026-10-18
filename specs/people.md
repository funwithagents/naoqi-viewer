---
code:
  - src/naoqi_viewer/people.py
  - src/naoqi_viewer/viewer.py
  - src/naoqi_viewer/viewer_process.py
  - src/naoqi_viewer/config.py
tests:
  - tests/test_people.py
  - tests/test_viewer.py
  - tests/mock_naoqi.py
  - tests-e2e/test_people_live.py
---

# People display

**Status:** Implemented

## Purpose

The window shows the people NAOqi knows about, as markers in the 3D scene where NAOqi says they are. NAOqi's people perception publishes its population in ALMemory under `PeoplePerception/`: the list of people, and for each one a position in the robot's frames. On a real robot `ALPeoplePerception` writes them; on naoqi-sim, whose desktop NAOqi has no perception engine, a client that runs its own detector on the camera stream publishes the same keys (naoqi-sim's `_overview.md`, "Perception and speech recognition"), so that `ALBasicAwareness` and `ALTracker` turn the head toward the person as on a NAO. Either way the keys are what NAOqi acts on, and this display draws them: a person watching sees where the robot believes people are, next to the robot's pose and, in sim mode, next to the face props a test placed ([props.md](props.md)), so a detector's error is a gap between a marker and a portrait.

The display is read-only, costs NAOqi a few ALMemory reads a second, works in both modes, is off by default, and is drawn in the window only: a marker is never in a camera render, so a detector never sees one.

## Decided

### A third reader of NAOqi

`PeopleSource` in `people.py` is the same shape as `CameraSource` ([cameras.md](cameras.md)): its own thread and qi session, `connect()` in one attempt, a fixed polling schedule, the newest result in a slot the window reads without blocking, reconnection with the pose source's backoff. A viewer whose config has `people_display: false` (the default) never builds one and never reads `PeoplePerception/`.

```python
PEOPLE_RATE_HZ = 10
PEOPLE_LIST = "PeoplePerception/PeopleList"
VISIBLE_LIST = "PeoplePerception/VisiblePeopleList"
POSITION_KEY = "PeoplePerception/Person/{id}/PositionInWorldFrame"

@dataclass(frozen=True)
class Person:
    id: int
    pos: tuple[float, float, float]   # m, NAOqi's FRAME_WORLD: the head's position
    visible: bool

@dataclass(frozen=True)
class People:
    persons: tuple[Person, ...]
    seq: int                          # increments with each poll that answered
    received_at: float                # time.monotonic()

class PeopleSource:
    def __init__(self, url: str, *, rate_hz: float = PEOPLE_RATE_HZ) -> None
    def latest(self) -> People | None  # never blocks; None before the first answer
    def rate(self) -> float
    def close(self) -> None
```

- **Two reads per poll**: `ALMemory.getListData([PEOPLE_LIST, VISIBLE_LIST])` for the ids, then one `getListData` of the `POSITION_KEY` of every id listed. An id whose position key is missing or malformed (not three numbers) is skipped. An empty or absent `PeopleList` is a `People` with no persons, not an error: a plain `naoqi-bin` and a naoqi-sim with no client publishing both read as nobody.
- **The frame is the viewer's.** `PositionInWorldFrame` is NAOqi's `FRAME_WORLD`, the frame the viewer poses the torso in (`getTransform("Torso", 1, True)`, [source.md](source.md)), so a marker is placed at the key's coordinates with no offset. The sibling keys in the torso and robot frames are not read.
- **Robustness**: a qi error or a disconnection keeps the last `People` (its age grows) and reconnects in the background; `close()` ends the thread and the session.

### The markers (window only)

The window loop ([viewer.md](viewer.md)) draws the people each frame into `handle.user_scn`, after the ghost's geoms, with `mjv_initGeom`: one sphere per person, `PERSON_RADIUS = 0.10` m (a head), at `pos`, labelled with the person's id (the geom's `label`). A visible person is `PERSON_VISIBLE_RGBA` (yellow, alpha 0.6); one in `PeopleList` but not in `VisiblePeopleList` is `PERSON_HIDDEN_RGBA` (grey, alpha 0.3): NAOqi still tracks them but does not see them. `user_scn` is drawn over the model's geoms and is not part of the offscreen renderers' scenes, so the camera frames never contain a marker; the ghost relies on the same property.

- **Stale data vanishes.** A `People` older than `PEOPLE_STALE_AFTER = 1.0` s (ten polls) is not drawn, so a NAOqi that went away leaves no frozen markers. The keys themselves may be old: the display shows what ALMemory holds, as NAOqi's own modules read it.
- **Status line**: `people: 2 (1 visible)`, `people: none`, `NO DATA` before the first answer, `STALE` past the threshold; part of the status overlay (F9).
- **No key.** The config flag alone turns the display on or off for the viewer's lifetime; the window's keys stay as they are ([viewer.md](viewer.md), Keys). A toggle key is a later addition, when someone watching wants one.
- **No window, no markers**: `people_display: true` with `headless: true` is a `ConfigError` on `people_display`, as for `cameras_display` and `ghost`.

### Config

`"people_display": true` at the top level of the config ([config.md](config.md)), a boolean, `false` by default, valid in both modes. The viewer process ([api.md](api.md)) builds a `PeopleSource` on the config's URL when it is on, hands it to the window loop and closes it with the other sources.

### What the markers show, per target

| Target | Markers |
| --- | --- |
| Real NAO (mirror mode) | `ALPeoplePerception`'s population, where the robot sees people |
| naoqi-sim (sim mode) | What a client published: its detector's people, placed from its own estimate of their position. With a face prop in the scene, the marker should sit on the portrait; the gap is the client's error |
| Plain `naoqi-bin`, or naoqi-sim with no client publishing | `people: none` |

### Testing

**Fast tier** (`tests/test_people.py`, `tests/test_viewer.py`, the mock NAOqi of [testing.md](testing.md), which gains the `PeoplePerception/` keys): `PeopleSource` reads two persons with their positions and visibility, skips an id without a position, reports nobody on an absent list, keeps the last `People` through a failing call and reconnects after a restart, closes promptly; `ViewerState` with a fake source draws one labelled sphere per person in the right colours, nothing when stale, and the status line's four readings; `config.py`: the field's default, round trip, type check, the `ConfigError` with `headless`.

**Live tier** (`tests-e2e/test_people_live.py`, on `NAOQI_URL`): the test writes a person into ALMemory (`insertData` of the three keys), and a `PeopleSource` reports them at the written position (ALMemory keeps float32, so approximately); the keys are removed after. A windowed run also opens a sim viewer with `people_display` on and a person published, and checks it draws for a while without failing.

## Open questions

1. **More of the contract.** `PeoplePerception/Person/<id>/` also holds `Distance`, `AnglesYawPitch`, `RealHeight`, `IsFaceDetected`, `NotSeenSince`, `PresentSince`. A marker sized by `RealHeight`, or a stalk from the floor, would read better than a sphere; deferred until the display is used.
2. **Faces without people.** 2.1 and 2.8 robots also publish `FaceDetected` (ALFaceDetection), with angles rather than positions. Not drawn; a client that publishes people is the contract this display reads.
3. **The 2.8 population.** On naoqi-sim, 2.8's `ALPeoplePerception` runs and reports an empty population at times; whether it overwrites a client's keys is naoqi-sim's to measure, and this display shows whatever wins.
