---
code:
  - src/naoqi_viewer/props.py
  - src/naoqi_viewer/client.py
  - src/naoqi_viewer/viewer_process.py
  - src/naoqi_viewer/model.py
  - src/naoqi_viewer/portraits/ATTRIBUTION.md
tests:
  - tests/test_props.py
  - tests/test_client.py
  - tests/test_viewer_process.py
  - tests-e2e/test_props_live.py
---

# Props: objects placed in the simulated world

**Status:** Implemented

## Purpose

Something for the simulated robot to look at, put there and moved while the viewer runs. In sim mode the head cameras render the scene ([api.md](api.md)), but a scene is an MJCF file compiled once at launch: a caller that wants a face in front of the robot, or a red ball to track, has had to write a scene file for it, and nothing could appear, move or disappear afterwards. Props fix that: a pool of bodies the viewer appends to every sim-mode scene, hidden, that the caller **spawns** (shows, at a position), **places** (moves, at once or over a duration) and **despawns** (hides) through the `NaoqiViewer` API, with the window and the rendered head cameras agreeing at every frame, since both draw the same model.

Two kinds, for the two needs a test of a NAO program has:

- **faces**: a portrait photo on a thin plane, so a face detector running on the frames naoqi-sim serves from the render finds a person where the test put one: since Oct 29, 2026 naoqi-sim's own `ALFaceDetection` and `ALPeoplePerception` replacements (its `face-detection.md` and `people-perception.md`), as on a real robot, and a client's own detector as before;
- **objects**: a coloured primitive (a sphere, a box, a cylinder) with a size and a colour, so a test checks what the camera sees where the robot's pose says it should, or gives a colour tracker something to follow.

This is the viewer's half, and it is geometry only: every prop has a pose in the scene, a position and a rotation, and the viewer puts it there, with no notion of NAOqi's frames and no behaviour of its own (a face is not turned toward the robot). naoqi-sim's `sim.props` drives it and manages the frames: positions relative to the robot, through NAOqi (naoqi-sim's `specs/host/props.md`). The mechanism: mocap bodies, a pool, visibility by alpha. naoqi-viewer compiles the scene itself through `MjSpec`, so the pool is appended in memory and no scene file is written, and the props are driven from the render loop rather than from a physics callback, since the viewer runs no physics.

## Decided

### The pool

MuJoCo cannot add bodies to a compiled model, so `load_world` ([model.md](model.md)) appends a fixed pool of **mocap bodies** to the scene's `MjSpec` before compiling it, in sim mode only (`load_world(..., props=True)`): a bundled scene or a user MJCF both get it. Every pooled prop starts **hidden** (its geom's alpha 0) and **parked** at the origin, below the floor (`PARKED_POS = (0, 0, -1)`), so an untouched scene renders exactly as before.

| Kind | Props | Body | Geom |
| --- | --- | --- | --- |
| `face` | `face_1` … `face_4`, one per bundled portrait, in the order below | mocap | one box `FACE_SIZE` = 0.32 m wide, 0.40 m tall, 0.005 m thick (a life-size face, below), textured with the portrait: `emission="1"`, no specular, so the pixels reach the camera unshaded whatever the scene's light |
| `object` | `object_1` … `object_8` (`OBJECT_POOL_SIZE = 8`) | mocap | one primitive, a sphere until spawned; its type, size and colour are set at spawn in `model.geom_type`, `geom_size` and `geom_rgba` (checked: MuJoCo's renderer reads them each frame) |

- **Textures are compiled in**, one per portrait (`spec.add_texture` from the bundled PNG, as the meshes do, [meshes.md](meshes.md)), so a face's picture is fixed for the viewer's lifetime; swapping it at runtime would mean re-uploading the texture to the GL context (open question 2).
- **Collision off** (`contype`/`conaffinity` 0) and no joints: the viewer never calls `mj_step`, and a mocap body's pose is read from `data.mocap_pos`/`mocap_quat` by `mj_kinematics`, which the pose writer already runs ([model.md](model.md)).
- **Names are reserved.** A scene whose own bodies use a pool name (`face_3`, `object_1`) fails to compile ("repeated name"), reported by `launch()` as a `LaunchError` naming the prop, as for a missing scene file.
- **Nothing in mirror mode**: no pool is appended and the props ops are refused with `ModeError`. A real robot's cameras cannot see a MuJoCo body.

### The portraits

Four public-domain portraits in `src/naoqi_viewer/portraits/`, credited in its `ATTRIBUTION.md` (each a crop of a NASA photograph on Wikimedia Commons), one person each, cropped to the same framing (320×400 pixels, the face about half the height): `armstrong` (frontal), `chawla` (frontal), `schmid` (head turned about 30°), `malenchenko` (head tilted down). All four are works of NASA, a U.S. federal agency, so public domain: they may ship in the repository and the package, unlike Aldebaran's meshes ([_overview.md](_overview.md), Licensing). A prop is a person: `face_2` is always Chawla, so a test reads "`face_2` appears, moves, leaves". The two frontal faces come first, so a spawn without a name gets a frontal face.

A face's apparent size is its distance: every face plane has the same `FACE_SIZE`, and a test makes one face bigger than another by spawning it nearer, as people at different distances are. **The pictured face is life-size** (decided Oct 29, 2026, with naoqi-sim's own perception): the plane is 0.32 × 0.40 m so that the box a face detector draws on the portrait (about half the plane's height, measured with OpenCV's YuNet) is about 0.20 m tall, the real height naoqi-sim's `ALPeoplePerception` assumes when it turns a face's angular size into a distance; a prop at 1 m then reads about 1 m, as a person at 1 m in front of a webcam does. At 1.0 m from a NAO head camera (60.97° across VGA, a focal length of 543 pixels) the plane is about 174 pixels wide and 217 tall, the face box about 106 pixels tall: well inside the range of common detectors at VGA, and still about 53 pixels tall at 2 m. Before that date the plane was 0.20 × 0.25 m (the face about 70 pixels tall at 1 m).

### Driving the props: `Props` in `props.py`

`Props(model, data)` attaches to the pooled bodies of a compiled model and holds the **commanded state** of each: visible, position, orientation, a move in flight. It runs on the render thread, in the request server's `on_frame` ([api.md](api.md)): each tick, before the queued requests are answered, `step(now)` advances the moves in flight, writes every changed pose into `data.mocap_pos`/`mocap_quat` and every changed visibility into `model.geom_rgba[..., 3]`, and runs `mj_kinematics` and `mj_camlight` when a pose changed, so a camera frame rendered in that tick shows the props where their moves put them at that instant. Idle, it writes nothing. The ops are plain method calls:

```python
Kind = Literal["face", "object"]
Shape = Literal["sphere", "box", "cylinder"]

@dataclass(frozen=True)
class PropState:
    name: str                                  # "face_1", "object_3"
    kind: Kind
    visible: bool
    pos: tuple[float, float, float]            # m, the scene's world frame: the commanded pose (the end of a move in flight)
    quat: tuple[float, float, float, float]    # w, x, y, z
    moving: bool                               # a timed move is in flight
    image: str | None                          # a face's portrait: "armstrong", "chawla", "schmid", "malenchenko"
    shape: Shape | None                        # an object's
    size: tuple[float, ...] | None             # an object's, MuJoCo geom size for its shape (m)
    color: tuple[float, float, float] | None   # an object's, RGB in 0..1

class Props:
    def list(self) -> dict[str, PropState]
    def spawn(self, kind: Kind, pos, *, name: str | None = None, quat=None, duration: float = 0.0,
              shape: Shape = "sphere", size=0.05, color=(1.0, 0.0, 0.0)) -> PropState
    def place(self, name: str, pos=None, quat=None, *, duration: float = 0.0) -> PropState
    def despawn(self, name: str) -> PropState
    def clear(self) -> None
    def step(self, now: float) -> None          # the render loop's, every tick
```

- **`spawn`** shows a free prop of `kind`: `name` picks a given one, else the lowest-numbered free prop of that kind. It is placed at `pos` (from its parked pose at once when `duration` is 0, else moved there over `duration` seconds from where it is) and shown, in one call, and its state is returned. A named prop already shown, an unknown name and no free prop of that kind (the error names the pool's size) are `PropError`s. **Free is hidden**: `despawn` hides a prop (its pose stays) and makes it free again; `clear` hides every prop.
- **`place`** moves a shown or hidden prop to `pos` and/or `quat`, at once or over `duration`. A **timed move** interpolates linearly on the wall clock (a monotonic clock, injectable for the tests) from where the prop *currently is*, so retargeting a move in flight starts from the interpolated pose, with no jump; the orientation by normalised lerp. `moving` reports a move in flight; a caller that wants the prop still polls `list()` (or `props()` on the client).
- **Poses are in the scene's frame**: `pos` in metres, `quat` a rotation `(w, x, y, z)`, both absolute. The scene's frame is the one the viewer draws the robot in, NAOqi's `FRAME_WORLD` (it poses the torso with `getTransform("Torso", FRAME_WORLD, True)`), so its origin is where NAOqi's odometry started, and the robot is not at it once its pose drifts: placing a prop relative to the robot is the caller's business (naoqi-sim does it).
- **The identity rotation, for every kind**, is what `quat` `None` means at `spawn`: an object keeps the scene's axes (a box's sizes along x, y and z, a cylinder upright); a face shows its picture toward **-x**, upright and unmirrored, so a face placed on the +x axis looks back at the origin. A given `quat` turns the prop from there, about its own centre: a yaw of `+pi/2` about z shows a face's picture toward -y. At `place`, `quat` `None` keeps the current rotation. A given `quat` is normalised (all-zero refused). The face's box carries the fixed rotation that makes this so (`FACE_GEOM_QUAT`, on the geom, not the body), so a prop's own rotation starts at identity.
- **Objects' shape, size and colour** are set at `spawn` only: `shape` picks `model.geom_type`; `size` follows MuJoCo's geom size for the shape (sphere `[radius]`, box `[x, y, z]` half-sizes, cylinder `[radius, half_height]`), with a single number accepted for a sphere's radius or a cube's half-side; `color` is RGB in 0..1. A despawned object keeps them until its next spawn resets them.
- **Validation**: a position that is not three finite numbers, a quaternion that is not four, a negative `duration`, an unknown `kind`, `shape` or `name`, a size that does not fit the shape, a colour outside 0..1 are `PropError(ValueError)`s, raised to the caller and never to the render loop.
- The state reported is the **commanded** one: the destination of a move in flight, and `visible` as asked, even before the frame that draws it (at most one tick, 17 ms, later).

### Ops and client API (protocol 2)

Five ops join [api.md](api.md)'s protocol, which becomes protocol **2**; all are **sim only** (`ModeError` on the client side first, `error: "mode"` from the viewer process) and work headless, which is what CI needs:

| Op | Request fields | Result |
| --- | --- | --- |
| `props` | — | `{"props": [state, …]}` |
| `prop_spawn` | `kind`, `pos`, optional `name`, `quat`, `duration`, `shape`, `size`, `color` | `{"prop": state}` |
| `prop_place` | `name`, optional `pos`, `quat`, `duration` | `{"prop": state}` |
| `prop_despawn` | `name` | `{"prop": state}` |
| `props_clear` | — | `{"props": [state, …]}` |

A state is the `PropState` fields as JSON. A `PropError` in the viewer process is answered `ok: false` with `"error": "prop: <message>"`, which the client raises as `PropError`; other errors stay as they are.

On the client, `NaoqiViewer.props` is a `PropsClient` with the same five methods and signatures as `Props` (`list`, `spawn`, `place`, `despawn`, `clear`), each one request, serialized with the other calls; `PropState` and `PropError` are exported from `naoqi_viewer`. `hello` answers `protocol: 2`.

```python
with NaoqiViewer(config) as viewer:                             # sim mode
    ball = viewer.props.spawn("object", (1.5, 0.0, 0.5), size=0.1)   # a red sphere, radius 10 cm
    face = viewer.props.spawn("face", (1.0, 0.3, 0.5))               # face_1, its picture toward -x
    viewer.props.place(face.name, (1.0, -0.3, 0.5), duration=2.0)   # walks across in 2 s
    viewer.props.despawn(face.name)
    viewer.props.clear()
```

### Measured

Oct 18, 2026, macOS arm64, MuJoCo's default (CGL) offscreen renderer, the `empty` scene with a probe camera (fovy 47.64°):

- A pool appended with `MjSpec` compiles; a sphere prop moved through `mocap_pos` and `mj_kinematics` alone (no `mj_step`) renders at its new place: a 0.3 m move at 1.0 m shifts it 165 pixels at VGA, against 163 for a pinhole; at 2.0 m it is a quarter of the pixels.
- Alpha 0 in `geom_rgba` removes the prop from the render (0 pixels of its colour), alpha 1 brings it back; `geom_type` and `geom_size` changed at runtime render the new shape (a sphere becoming a box seven times its pixel area); `geom_rgba` recolours it.
- A box textured from a bundled portrait PNG through `add_texture` and a material compiles and renders.
- A 320×240 render with two props takes about 2.3 ms on this machine.
- A second body with a pool name makes `compile()` raise `ValueError: repeated name`.

### Testing

**Fast tier** (`tests/test_props.py`, a real headless model as [testing.md](testing.md) runs one): `load_world(props=True)` on `empty`, `table` and a user scene appends 4 faces and 8 objects, hidden, parked, and `props=False` appends nothing; a scene with a reserved name fails to compile with the prop's name. `Props` on that model: `spawn` picks the lowest free prop, shows it at `pos` on the next `step`, and the geom's alpha, `mocap_pos` and `geom_type`/`size`/`rgba` say so; a named spawn, a spawn of a shown prop, no free prop and bad values raise as specified; `despawn` hides and frees; `clear` hides all; a timed move with an injected clock is halfway at half the duration, retargets without a jump and ends `moving: false`; a face spawned without a rotation shows its picture toward -x and upright wherever it is, and a yaw of `pi/2` turns it toward -y; an object spawned without a rotation stands upright; `step` writes nothing when idle (the arrays unchanged). Offscreen renders: a spawned red sphere has red pixels and none once hidden; each of the four portraits, spawned on the +x axis with no rotation and seen by `CameraTop`, has the same brightness halves (left against right, top against bottom) as its texture, which a mirrored or upside-down picture would not, and none is left once despawned. The viewer process (`tests/test_viewer_process.py`): the five ops on a sim-mode server and `error: "mode"` in mirror mode; the client (`tests/test_client.py`): `PropsClient` against the fake viewer process, `ModeError` in mirror mode without a round trip.

**Live tier** (`tests-e2e/test_props_live.py`, on `NAOQI_URL`, headless or windowed as the tier runs): the robot stands (`StandInit`; a fresh virtual robot lies in no known posture), a sim viewer spawns a red sphere 1.5 m ahead of it, the position computed in the scene's frame from `ALMotion.getRobotPosition` read right before (a virtual 2.8 robot keeps turning a little after a stand), and `camera_frame("top", 640, 480)` has the sphere in the middle of the frame, and none after `despawn`: the scene's frame is NAOqi's world frame, on 2.1 and 2.8. The fast tier also drives a real headless viewer process through `viewer.props` against the mock NAOqi: a spawned sphere is in the very next frame asked for.

## Open questions

1. **The robot at the scene's origin.** The viewer places the robot at NAOqi's world pose, which is odometry, so a scene's own objects (the `table` scene's) are not where a scene author expects once the robot's pose drifts. Props placed relative to the robot are naoqi-sim's answer; re-basing the whole scene on the robot's pose at launch is deferred.
2. **Changing a portrait's image at runtime**, or more than four faces: a texture swap on a running model means re-uploading `tex_data` from the render thread. The pool assigns images at launch; a larger or caller-supplied pool (`WorldSettings.props`: portraits from the caller's files) is the cheaper step, when a test needs it.
3. **Pool sizes.** 4 faces and 8 objects are a guess at what a test scene needs at once; a config knob waits for a caller that runs out.
4. **A person's body.** A portrait on a plane is a head; a presence detector or a people detector working on bodies would want a torso too. A `person` kind (a billboard of a standing person, or a face plane over a box) is a new kind beside these two, when a client needs it.
5. **Faces turned toward a point.** A caller that wants a face looking at the robot, or anywhere, computes the rotation; a helper that does it would be the caller's (naoqi-sim's, for the robot), not the viewer's.
