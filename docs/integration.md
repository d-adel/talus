# Integrating Talus Physics

Link `Talus::Physics` from CMake and include `<talus/talus.h>`. A `talus::World`
combines the dynamics state with the structural destruction state. It performs
scheduled edits, a dynamics step and the destruction update in that order.

## Host responsibilities

The host owns its scene graph, rendering, input, game rules and networking.
Maintain a mapping between host objects and Talus body IDs or compound-child
keys. Body IDs and child keys are different identifiers. Low-level creation
helpers currently return vector indices; read the body's `id` before storing a
persistent reference. Indices may change after removals.

Call `world.step()` on a fixed schedule. The default dynamics timestep is 1/60 s.
Accumulate elapsed time in the host rather than using rendering frame times as
physics timesteps. Decide how the game handles excess accumulated work; do not
silently discard simulation time. Interpolate render poses in the host if needed.
The examples use metres, kilograms, seconds and gravity along negative Y. Convert
the host's units and coordinate system at the adapter boundary.

Read body poses and `StepReport` after stepping. Structural splits create new
bodies and may change which body owns a chunk. Handle those events when updating
host objects. Use the edit functions in `<talus/dynamics/edits/edits.h>` for
teleports, impulses, shape changes and removals so caches are updated.

The world owns its simulation containers. Meshes use shared ownership. Sampled
field pointers are borrowed: keep their backing storage alive for every body
that uses them. Serialize simulation access; do not mutate a world concurrently
with a step. Each world has its own worker configuration. Hosts with a task
system can supply the existing enqueue/finish callbacks in `dynamics::World`.

## Package consumption

`add_subdirectory` supplies the same `Talus::Physics` target as an installed
package. Installation exports headers and the static archive, not applications
or tests. An installed consumer must provide GLM and Eigen CMake packages; system
threading is resolved by CMake. The supplied target propagates its C++ standard,
GLM configuration and public field constants.

The static archive is position-independent where the platform requires it, so a
native engine plugin can link it. This is a source-level C++ integration API, not
a stable C ABI or a ready-made plugin. Match the compiler, runtime and build
configuration of the consuming engine. An engine adapter still needs shape,
material, query, body and lifecycle mappings.

Avoid having both Talus and the host physics system independently solve the same
contact. A Godot integration could use its custom physics extension interface,
but that adapter and its moving-character interactions require separate work.
This repository restructuring does not implement it.

## Joints

A joint connects two rigid bodies, or a rigid body and the world. The editing
functions are in `<talus/dynamics/joints/jointedits.h>` and the types are in
`<talus/dynamics/joints/joints.h>`. Pass the dynamics world, which is
`world.dynamics` in a `talus::World`. Measured cost and known limits are in
[physics status](physics-status.md#joints).

| Kind | What it holds | Limits | Drive |
| --- | --- | --- | --- |
| `Ball` | The two anchor points together | None | None |
| `Hinge` | The anchors together, and frame B's x axis on frame A's x axis | An angle range about that axis | One axis |
| `SwingTwist` | The anchors together | A twist range about frame B's x axis and an elliptical swing cone | Three axes |
| `Spring` | Pulls frame B toward a target pose on body A, up to a force and a torque cap | None | Its own stiffness and damping |

### Creating a joint

```cpp
#include <talus/dynamics/joints/jointedits.h>

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

- Refer to bodies by `RigidBody::id`, never by vector index. `bodyA` may be
  `noBody`, which means the world. A joint to a static body uses `noBody`,
  because statics never move. `bodyB` must be a rigid body.
- A mass-0 (driven) body may be either end, so a joint can ride a moving
  platform or a kinematic hand.
- Each frame is a position and a rotation relative to its body's
  centre-of-mass frame (`RigidBody::position` and `rotation`). For the world
  end, frame A is in world coordinates.
- `addJoint` returns the joint's id. Ids start at 1 and are never reused. It
  returns 0 and adds nothing when the definition is invalid (see
  [Accepted values](#accepted-values)).
- Creating, editing or removing a joint wakes both bodies.
- With `collideConnected` false, which is the default, the two bodies never
  collide, and their existing contacts are dropped when the joint is created.
- `removeJoint` deletes a joint. When a body is removed, its joints are deleted
  at the start of the next step, and `jointReport` then returns
  `std::nullopt`.

### Angles, limits and drives

- **Hinge.** The axis is frame A's x. The angle is the rotation of frame B
  relative to frame A about that axis, in radians.
- **Swing-twist.** The relative rotation of frame B in frame A is split as
  q = q_swing · q_twist. The twist is about frame B's x axis. The swing is a
  vector (y, z) whose direction is the swing axis and whose length is the swing
  angle. This is Reflex's decomposition, so a Reflex target means the same pose
  in Talus.
  - The cone is the ellipse ((s_y − c_y) / a)² + ((s_z − c_z) / b)² ≤ 1, with
    `cone.swing` = (a, b) and `cone.centre` = (c_y, c_z).
- **Limits are rigid.** A limit row stops the closing velocity completely. A
  joint that is past its limit is pushed back at up to 30 rad/s.
  - After each joint pass, the solver gives two extra passes to the limit rows
    of every joint that is pushing on a limit or is within 0.05 rad of one.
    Joints far from their limits pay nothing for this.
  - The push-back is a hard snap. A joint that starts far outside its limits
    returns within about one frame.
- **Drives.** Each drive axis has a `targetAngle` (rad), a `targetVelocity`
  (rad/s), a `stiffness` k (N·m/rad), a `damping` c (N·m·s/rad), and
  per-direction torque bounds `minTorque` and `maxTorque` (N·m).
  - Hinge drives use `axes[0]`. Swing-twist drives use `axes[0]` for the twist
    and `axes[1]` and `axes[2]` for the two swing components.
  - `JointDrive::maxTorque` caps the length of the world torque vector, after
    the per-axis bounds. `feedForward` is a world-space torque. Each substep
    applies it to body B and its opposite to body A.
  - k and c are physical, so the response does not depend on how light the
    bodies are. A drive deflects by τ/k under a steady torque τ, and rings at
    √(k/I), where I is the inertia it moves.
  - The ringing frequency is accurate within 5 % up to a range that depends on
    the substep length. At the default 1/960 s substep, the range ends at about
    39 Hz between equal bodies, 19 Hz at a 10:1 mass ratio and 12 Hz at 75:1.
    Stiffer drives ring slower than √(k/I). Raise `jointSubsteps` if a stiffer
    drive must ring at its exact frequency.
- `setJointDrive` replaces a joint's drive. It returns false for an unknown
  joint or an invalid drive.

### Spring joints

- The target pose is body A's pose, then `frameA`, then `spring.target`. So
  the default target pulls frame B onto frame A. With body A the world, the
  target is a world pose.
- Linear `linearStiffness` (N/m), `linearDamping` (N·s/m) and `maxForce` (N).
  Angular `angularStiffness` (N·m/rad), `angularDamping` (N·m·s/rad) and
  `maxTorque` (N·m). The caps limit the force and torque vectors in any
  direction.
- When a cap is reached, the spring slips instead of holding, so a heavy
  object drags or falls out of a grip.
- `setSpringTarget` moves the target pose. It returns false for an unknown
  joint or an invalid frame.
- A spring has no target-velocity input, and its damping acts on the relative
  velocity of the two bodies. To carry an object smoothly, make body A a mass-0
  hand and move the hand with `setRigidPose` and `setVelocity`. The grip then
  follows without lag. If instead body A is the world and the host moves the
  target every step, the damping resists the object's motion: in the tests, a
  3 kg box with c = 200 N·s/m lags 0.034 m behind a target that moves at 1 m/s.

### Read-back

`jointReport(world, joint)` returns the joint's state after the last step:

- `force`: the force that the joint applied to body B in the last substep, in
  world axes (N). Body A received the opposite force.
- `torque`: the torque from the alignment and limit rows, or from a spring's
  angular block, in the last substep (N·m). It excludes the drive.
- `driveTorque`: the drive torque on each drive axis (N·m).
- `limitActive`: a limit row pushed in the last substep.
- `driveSaturated`: a per-axis bound or the magnitude cap clipped the drive.
- `forceSaturated` and `torqueSaturated`: a spring cap clipped the spring.

`StepReport::jointRows` counts the joints solved in the step, and
`StepReport::limitPassRows` counts the limit rows that the extra passes
visited. `StepProfile::joints` is the time of the joint stages in softstep, in
milliseconds.

### Accepted values

`addJoint` rejects a definition, and the setters return false, when:

- body A and body B are the same, body B is `noBody`, or a body is unknown or
  not rigid;
- a value is NaN, a frame rotation has zero length, or a position, angle,
  gain, `hertz`, `dampingRatio` or feed-forward is infinite (torque bounds and
  caps may be infinite, which is their default);
- `hertz` is not positive, a stiffness, damping, cap or `dampingRatio` is
  negative, or a `minTorque` is above its `maxTorque`;
- an enabled hinge range has its lower bound above its upper bound, or a bound
  outside ±0.99π;
- an enabled cone has its twist lower bound above its upper bound, a twist
  bound outside ±0.99π, a half-angle outside [0.001, 0.89π], or a centre
  offset plus its largest half-angle above 0.89π;
- a drive target is outside ±π, or a swing target (`axes[1]` and `axes[2]`
  together) is longer than 0.89π.

The swing bound is 0.89π (160°) because near a half turn the twist row and a
swing row point in almost the same direction and fight each other. A 3.0 rad
swing target spun the joint up after 4 s, and 2.8 rad stayed stable for every
stiffness tested.

### Substeps and frame rate

An island that contains a joint is stepped with
`IntegrationParameters::jointSubsteps` substeps, 16 by default. Every other
island keeps `softstepSubsteps`, 4 by default. Islands without joints do
exactly the same work as before.

`jointSubsteps` is a count per frame. A host that steps at 30 Hz (dt 1/30 s)
needs 32 to get the same substep length, and so the same joint behaviour, as
16 at 60 Hz. With the same substep length, joint motion does not depend on the
frame length.

Jointed islands limit each body's rotation to 45° per substep instead of per
frame. A jointed body can therefore turn more than 45° in one frame.
Continuous collision interpolates rotation linearly between a frame's start
and end poses, so it is approximate for such fast-turning bodies.

### Sleep and wake

- A chain of jointed bodies sleeps and wakes as one island.
- `setRigidPose` and `setVelocity` wake every body jointed to the edited body.
- At the end of each step, every body jointed to an awake, moving mass-0 body
  is woken, so it follows the mass-0 body.
- A host that writes `RigidBody::velocity` directly bypasses both rules, as it
  does for contacts. Use `setVelocity` instead.

### Tiers and destruction

- A switchable body stays in the rigid tier while it is a joint end, so a held
  prop does not dent. `promote` returns false for a joint end and changes
  nothing. Remove the joint first to promote the body.
- A jointed island that touches a lattice body is solved by strainstep, at
  `jointSubsteps` or the lattice's own substep count, whichever is larger.
  While they touch, the lattice is stepped at that count too. In the one scene
  measured, this tripled the lattice's strainstep time.
- When destruction re-centres a compound, each joint end on it is re-expressed
  in the new body frame, so its world anchor and frame rotation do not change.
- When a compound splits, or chunks break off an anchored building, each joint
  end moves to the piece whose child box holds the anchor, or is nearest to
  it. On a tie, the piece that keeps the old body id wins, then the new pieces
  in the order the split creates them. The joint's contact exclusion moves to
  the new pair, its warm-start impulses are kept, and both ends are woken.
- When the chunk that holds an anchor is destroyed, the joint moves to the
  piece with the nearest remaining child. It does not break.
- Joint forces do not yet load destruction bonds. Only contacts load
  structures.

### Determinism

Joints are solved one after another in ascending id. Results are identical at
every SIMD width and worker count.
