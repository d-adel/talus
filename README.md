# Talus Physics

Talus is a C++23 physics library for integration into game engines. It provides
rigid bodies, compound collision shapes, deformable bodies, plastic deformation,
tearing, material-driven structural fracture and a fixed-step simulation pipeline.

The host engine owns scenes, rendering, input, audio, networking and game logic.
Talus owns physical state and returns motion, contacts and destruction events.
The former standalone game engine, renderer, editor, player and C# runtime are
removed from the current tree.

![Three-storey building collapsing](single-building-collapse.gif)

Three-storey concrete-frame collapse, rendered with the original Talus testbed.

## Features

### Rigid bodies and collision

- Boxes, spheres, compound bodies and sampled signed-distance-field shapes.
- Broad-phase collision detection, persistent contact manifolds, speculative
  contacts and continuous collision handling for moving bodies.
- Friction, restitution, sleeping, impulses and runtime body or shape edits.
- Compound-child removal and splitting, with mass and inertia recomputed for
  the resulting pieces.

### Joints

- Ball, hinge and swing-twist joints between rigid bodies or to the world,
  with rigid angle limits and an elliptical swing cone.
- Drives with physical stiffness and damping on each axis, per-direction
  torque bounds and a torque cap, so a response does not depend on how light
  the joined bodies are.
- Strength-limited spring joints that slip at their force or torque cap, for
  grips and tethers.
- A force-limited planar velocity joint and a pogo spring joint, which drive
  and float a dynamic character body in the style of Box3D's dynamic mover.
- Joined pairs that do not collide, and per-joint force, torque and saturation
  read-back.
- Islands that contain a joint, with every body they touch, step with their
  own substep count; other islands are unaffected. Joints keep their anchors
  through compound splits.

### Deformation and material history

- Finite-element deformation with elastic recovery, plastic flow and hardening.
- Material parameters for density, stiffness, damping, yield and tearing.
- Persistent plastic deformation and mesh tearing, including updated surfaces
  and collision geometry.
- Transitions between rigid and deformable representations, retaining supported
  deformation history when a body returns to rigid motion.
- CPU signed-distance-field construction, material queries and field updates.

### Structural fracture and destruction

- Connected chunks with stress-based compression, tension and shear failure.
- Intact, cracked and broken bonds, including frictional bearing after cracking.
- Concrete, reinforced concrete, brick, timber, glass and steel material presets,
  with support for direction-dependent strength and material variation.
- Chunk removal, material-based subdivision and separation into moving compound
  bodies when connections fail.
- Contact loads, blasts and scheduled destruction actions.

### Simulation and integration

- Fixed-step simulation, with a default timestep of 1/60 second.
- Multithreaded CPU stepping, runtime SIMD selection and optional host task
  callbacks for integration with an existing scheduler.
- Sleeping and freezing/thawing mechanisms, plus a workload controller for
  deferring structural work.
- Ray queries, contact-hit and destruction reports, workload counters and
  per-stage timing data.
- Determinism regression coverage across the tested Windows/MSVC and Linux/GCC
  builds, worker counts and SIMD widths.
- CMake embedding or an installed `Talus::Physics` package, with headless examples.

Current limitations are listed under [Known limits](#known-limits). The
workload controller is not a hard real-time guarantee, and game-engine adapters
remain separate work. Joint usage is described under [Integration](#integration).

## Use from another CMake project

```cmake
add_subdirectory(external/talus)
target_link_libraries(my_adapter PRIVATE Talus::Physics)
```

```cpp
#include <talus/talus.h>

talus::World world;
world.dynamics.workerCount = 8;
// Add bodies, then call once per fixed physics tick.
auto report = world.step();
```

The library requires GLM, Eigen and system threading. CMake can fetch the pinned
header dependencies when they are not installed. It requires no graphics API,
windowing library, ECS, .NET runtime or other physics engine. Tests additionally
fetch Catch2. See [third-party notices](THIRD_PARTY_NOTICES.md).

The initial integration API is C++; it is not a stable binary ABI. Godot, Unity
and Unreal adapters are not included yet. See [Integration](#integration)
for ownership, timing and joint requirements.

## Build and test

Use CMake 3.28+, Ninja and a C++23 compiler on x86-64. On Windows, use an x64
Developer Command Prompt for Visual Studio 2022 so the compiler and SDK are set up.

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
```

`cmake --workflow --preset verify` runs those three stages in sequence.

This builds the static library, physics tests and `talus_headless`, a console
example that drops a box onto a floor. There is no GUI or GPU requirement.
The `dev` preset selects Debug. Windows/MSVC and Linux/GCC 14 x86-64 builds are checked; known
limitations are listed under [Known limits](#known-limits).

When embedded through `add_subdirectory`, tests and examples default to off.
They can also be disabled explicitly with `TALUS_BUILD_TESTS=OFF` and
`TALUS_BUILD_EXAMPLES=OFF`. Set `TALUS_FETCH_DEPENDENCIES=OFF` to require installed
dependencies instead of downloading them.

## Installed package

```sh
cmake --install build-release --prefix /path/to/talus-sdk
```

Consumers need GLM 1.x and Eigen 3.4+ installed as CMake packages:

```cmake
find_package(Talus CONFIG REQUIRED)
target_link_libraries(my_adapter PRIVATE Talus::Physics)
```

`examples/consumer` is a separate application using that installed package.

## Integration

Link `Talus::Physics` and include `<talus/talus.h>`. A `talus::World` runs
scheduled edits, a dynamics step and the destruction update, in that order. The
API is C++ source, not a stable binary ABI. Match the compiler and runtime of
the consuming engine.

### Conventions and ownership

- Units are metres, kilograms, seconds and radians. Gravity points along
  negative Y. Convert the host's units and axes at the adapter boundary.
- The host owns scenes, rendering, input, game rules and networking. Do not let
  Talus and the host physics system both solve the same contact.
- Body IDs and compound-child keys are different identifiers. Creation helpers
  return vector indices, and indices change after removals. Read `RigidBody::id`
  and store that.
- Change bodies only through `<talus/dynamics/edits/edits.h>` (teleports,
  impulses, shape changes, removals). Writing fields such as
  `RigidBody::velocity` directly skips cache updates, waking and joint rules.
  A direct write to `RigidBody::asleep` is invisible to `world.step()`: wake
  bodies with `dynamics::wakeBody` or an edit.
- The world owns its containers. Sampled-field pointers are borrowed: keep
  their storage alive while any body uses them.

### Stepping

- Call `world.step()` on a fixed schedule, 1/60 s by default. Accumulate time
  in the host, never use render frame times, and do not silently drop excess
  time. Interpolate render poses in the host.
- Read poses and the `StepReport` after each step. Structural splits create new
  bodies and can change which body owns a chunk, so update host objects from
  the report.
- Serialize access to a world. Never edit a world while it steps. Each world has
  its own `dynamics.workerCount`. A host scheduler can supply the enqueue and
  finish callbacks in `dynamics::World`.
- Stepping does not read the wall clock. Results are identical across SIMD widths
  and worker counts on the tested Windows/MSVC and Linux/GCC builds.

### Joints

A joint connects two rigid bodies, or a rigid body and the world. Joint types
are in `<talus/dynamics/joints/joints.h>`. Create and change joints only with
the functions in `jointedits.h`, passing `world.dynamics`. `world.joints` is
read-only. Kinds: `Ball`, `Hinge` (limit and one drive axis), `SwingTwist`
(twist range, elliptical swing cone, three drive axes), `Spring` (pulls
frame B to a target pose, with force and torque caps), and `PlanarVelocity`
and `Pogo` for character movers (see [Character movers](#character-movers)).

```cpp
using namespace talus::dynamics;

JointDef elbow;
elbow.kind = JointKind::Hinge;
elbow.bodyA = upperArm;
elbow.bodyB = forearm;
elbow.frameA.position = glm::dvec3(0.0, -0.145, 0.0);
elbow.frameB.position = glm::dvec3(0.0, 0.13, 0.0);
elbow.hinge = HingeLimit{true, 0.0, 2.5};
elbow.drive.axes[0].targetAngle = 1.0;
elbow.drive.axes[0].stiffness = 200.0;
elbow.drive.axes[0].damping = 20.0;
elbow.drive.maxTorque = 80.0;
const uint32_t joint = addJoint(world.dynamics, elbow);
```

- Invalid values are rejected: `addJoint` returns 0 and setters return false,
  and nothing is stored. Check the return value.
- Name bodies by `RigidBody::id`. `bodyA` may be `noBody`, the world, in which
  case frame A is in world coordinates. Use `noBody` for static bodies. `bodyB`
  must be a rigid body. A mass-0 body may be either end.
- A frame is a position and rotation relative to its body's centre of mass.
- Drive stiffness (N·m/rad) and damping (N·m·s/rad) are physical, so the
  response does not depend on body mass. `maxTorque` caps the torque vector.
  Stiff drives ring slower than √(k/I) and are accurate only below about 39 Hz
  between equal bodies and 12 Hz at a 75:1 mass ratio. Raise `jointSubsteps`
  if that matters.
- A swing-twist joint splits frame B's rotation relative to frame A as
  q = q_swing · q_twist, with the twist about frame B's x axis. Its drive axes
  are `axes[0]`, the twist angle, and `axes[1]` and `axes[2]`, the y and z
  components of the swing vector (axis times angle). Targets are in those
  coordinates, not the shortest rotation, so a twist range wider than π works.
- Limits are rigid. A joint past its limit snaps back at up to 30 rad/s.
- A spring slips at its force or torque cap, so a heavy object can drag out of a
  grip. Its damping acts on the bodies' relative velocity. To carry an object
  smoothly, make body A a mass-0 hand and move it with `setRigidPose` and
  `setVelocity`.
- Joined bodies do not collide unless `collideConnected` is true. Planar
  velocity and pogo joints must set it to true.
- `jointReport` returns force, torque, drive torque and saturation flags from
  the last step. A joint in a sleeping island keeps its old report.
- Islands with a joint step `jointSubsteps` times per frame (16 by default).
  Other islands use `softstepSubsteps` (4). A jointed body that rests on a pile
  makes the whole pile step at 16 while they touch. The count is per frame, so
  a 30 Hz host needs 32 for the same joint behaviour.
- `removeBody` and `removeBodies` delete the body's joints and wake the other
  ends. A joint end cannot `promote` to a lattice until the joint is removed.
- When destruction splits a body, each joint end moves to the piece that holds
  its anchor. Joint forces do not load destruction bonds.

### Character movers

`castRay(world.dynamics, from, to, skips)` in `<talus/dynamics/rays/worldray.h>`
returns the nearest hit's side, body id, child, point, normal and fraction,
using the broadphase from the last step plus bodies added since. Skip every
body moved with the character: its own body, kinematic hands and held objects.
Skips do not apply to statics.

| Kind | Effect | Settings |
| --- | --- | --- |
| `PlanarVelocity` | Drives B's velocity relative to A in frame B's xz plane; no torque | `planar.targetVelocity` (world axes), `planar.maxForce` (N; 0 is off) |
| `Pogo` | Spring along the ground normal from a ground point on A (frame A) to a foot anchor on B (frame B); no torque on B | `pogo.normal`, `restLength`, `hertz`, `dampingRatio`, `maxTension`, `maxCompression`; warm start `force`, `speed` |

- Both kinds need `collideConnected = true`. A pogo refuses ground with
  dot(normal, frame B's y) < 0.25. Keep the body upright with an angular-only
  `Spring` to the world: the planar joint's plane tilts with frame B, so a
  tilted character drifts sideways in the air.
- The pogo's ground point (frame A) is in body A's frame, or in world
  coordinates for `noBody`. `pogo.normal` is in world axes.
- Scale pogo caps with the substeps: 100 × weight × `jointSubsteps` / 4. The
  caps also limit the spring's speed, so soften it with `hertz`, not a low cap.
- A contact that holds the body below its rest length (a ceiling, a ledge
  corner) makes the pogo press at its full cap: shorten the rest length when
  the head is blocked, use a capsule, add upright control or lower the tension
  cap.

A host step for each character, before `world.step()`, as in Box3D's sample:

1. Cast down from the foot anchor, reaching `restLength` (twice on the ground).
2. With no walkable hit, remove the pogo.
3. On the same ground body, call `setPogo` with the new point and normal, and
   `jointReport`'s `pogoForce` and `pogoSpeed` as `force` and `speed`.
4. On a new ground body, remove the pogo and add one, with the old pogo's
   `pogoForce` and `pogoSpeed` as `force` and `speed`. Body A is the hit body,
   or `noBody` for static and lattice hits.
5. The planar joint's body A is the ground body when that is a moving mass-0
   platform or deck, so the character moves with it (as in Box3D's sample), and
   the world otherwise; re-create the joint when A changes. Call
   `setPlanarVelocity` with the input velocity relative to A and a force cap.

During a jump, set the pogo's `maxTension` to 0; restore it on landing.

### Failure handling

- `StepReport::nonFinite` is true when a body state became NaN or infinite. The
  host must detect it and reset or remove the body.
- Edit functions return false or 0 when they do nothing.
- Talus guarantees no wall-clock deadline. The workload controller defers
  structural work, but a step can exceed the frame budget.

## Known limits

- Only x86-64 builds are qualified.
- The workload controller's cost constants are unqualified. They underpredict
  in reference checks, and the large district test missed the 4 ms physics
  goal. Rubble can stay awake, and thin-sheet glass does not shatter
  convincingly.
- One ball/soft-face test and two joint tests are expected failures: 680 cases,
  3 expected failures, Windows/MSVC Release, at `4a5ecf6`.
- One 20-body humanoid costs 0.225 ms per step (0.148 ms in joints) on a Ryzen 7
  7840HS laptop, MSVC 19.44, `55d5282`. Four cost 0.765 ms.
- A jointed body touching a lattice steps both at 16 substeps. A five-box hinge
  chain on a 96-cell slab costs 3.0 ms per step, against 1.0 ms at 4 substeps.
- Without collision groups, a jointed character collides with itself. Contacts
  can push a joint past its limit.
- Not measured: the cost of compound splits that move joints, and of waking
  every joint on an anchored building when a chunk breaks off.
- The world ray cast can miss a body for one step after it moved farther than
  predicted or was teleported.
- A ray onto a lattice costs about 130 µs.
- A character on a lattice uses the world as ground and puts no load on it.
- A character's island, and every pile or lattice it touches, steps at 16
  substeps. A soft body that a character touches costs about 4 times as much.
- A character edited every step, and what it stands on, never sleeps.
- When a hold at the pogo's cap ends abruptly, the body pops up at up to 4 m/s.
- Stepping 0.4 m up onto 10 kg crates shoves them down with about 5 kN.
- A character spawned exactly at its rest height misses the ground with its
  first in-air ray.
- `world.islands` can omit islands whose bodies all sleep.
- A step after a body removal, a lattice added or removed, a tier switch, a
  moved or removed static body (`setStaticPose`, `removeStatic`; both also wake
  every body) or a contact-band change visits every body.
  That step costs about 1.4 ms among 20,000 sleeping bodies on the Ryzen 7
  7840HS laptop.
- A world that holds any lattice body still copies every rigid body's velocity
  each step and visits every lattice island. Sleeping rigid bodies are not free
  there. In `talus::dynamics::step`, a sleeping rigid body costs almost nothing
  in all other worlds.
- In `talus::World::step`, the destruction update also visits only awake
  bodies. Among 20,000 sleeping bodies it costs below 1 µs per step on one
  worker of the Ryzen 7 7840HS laptop and 1 to 2 µs on eight, against 0.67 ms
  and 0.26 ms when it visited every body. It still visits every rigid body:
  - in the update after an update that threw;
  - from the point where an update removes a body itself: a scheduled removal
    of a whole piece, a split that empties an anchored piece, or the removal of
    frozen debris or of bodies below `discardBelow`;
  - in its rest tracker and body-id map, in the first update after a body
    removal, a tier switch, a removed lattice or a moved or removed static body;
  - in every update while `discardBelow` is finite.

## Repository layout

- `src/talus/`: engine-neutral physics implementation and headers.
- `tests/`: physics regressions and integration checks; district geometry is a test fixture.
- `examples/`: headless embedding and installed-package examples.
- `cmake/`: source list and package configuration.

The solver kernels are carried over from the current Talus implementation.
Physics data types have been extracted from the old object model; scene and
rendering dependencies have been removed. This reorganization does not resolve
existing performance or physical-behavior limitations. See
[Known limits](#known-limits).

## License

MIT. See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
