# Rigid joints, capsules and collision filters

Draft for approval, 2026-10-29. Source revision: `e5c97c3`.

## Why

Two users need Talus to join rigid bodies, and neither can run on Talus today.

- **Reflex**, the active-ragdoll library, needs about 20 capsule bodies per character, joined by hinge and swing-twist joints. Muscle drives pull each joint toward a target pose. Reflex runs on Box3D until Talus has joints.
- **A dynamic character mover**, in the style of Box3D's (pull request 202, October 2026). It is a capsule body driven by a force-limited planar velocity joint and held above the ground by a spring "pogo" joint. A host game also needs strength-limited springs that hold objects in a character's hands.

A learned policy may later correct Reflex's targets. It needs physical per-axis gains and fast headless stepping.

Talus has none of this. There are no joints, no capsule shape and no collision filtering (`RigidShape` is `Sampled`, `Sphere`, `Box` or `Compound`).

## Goals

1. Joints between two rigid bodies, or between a rigid body and the world: ball, hinge, swing-twist, spring, planar velocity and pogo.
2. Drives in physical units, set independently per axis.
3. A capsule shape that collides with every existing shape.
4. Collision filtering: groups, category and mask bits, and joined pairs that never collide. Rays honour the same filter.
5. Per-joint read-back of force, torque and saturation.
6. No change to worlds that use none of this.

## Non-goals

- **Structural connections.** Parts of buildings and structures stay compounds and destruction bonds. Joints are for characters, tools in hands and machines.
- **Joints anchored on lattice or deformable bodies.** Joints attach to rigid bodies only. A rigid body with a joint stays in the rigid tier while it is jointed, because `switchTiers` skips it. So a held prop does not dent while held (**my call**, to revisit after this work).
- **Character logic in Talus.** Ground detection, input and when to jump stay in the host, as `CONTRIBUTING.md` requires. The host casts the pogo ray and recreates the pogo joint each step, as Box3D's sample does.
- **Reflex or a training backend in this repository.**
- **Changes to host bridges.** The Godot bridge is a separate job.

## Anti-goals

The work fails, even if it runs, if any of these happen:

- Results of an existing world change, or differ across worker counts or SIMD widths.
- Ragdolls need the whole world stepped at 16 substeps.
- A drive's stiffness depends on how light the joined bodies are. On Box3D a planted ankle's motor is clamped to about a sixth of the requested stiffness, because the spring is sized by the light foot.
- A ragdoll explodes or creeps at ordinary character mass ratios, such as a 40 kg torso above 1 kg feet.

## Design

### Joint records and the solver

- **Storage.** A new `dynamics/joints` module holds joints in `dynamics::World`.
  - Each joint has a stable id, a kind, body ids A and B, and a local frame on each body.
  - A is a rigid body or the world. Statics are signed-distance fields with no motion, so a joint to a static uses the world.
  - A mass-0 (driven) body may be A, so joints can ride moving platforms.
- **Rows.** Each joint carries softness for its rigid rows (hertz and damping ratio) and a `collideConnected` flag.
  - Defaults follow Box3D: 60 Hz, damping ratio 2, clamped to a quarter of the substep rate.
  - Limits are speculative: when a limit is not yet reached, its row allows the remaining gap to close in one substep.
  - A soft limit's stiffness is its frequency squared times the joined pair's effective inertia. So, like Box3D's motor, it is soft between light bodies, and a strong drive could push a light foot past its ankle limit.
  - **Measured while planning (2026-10-29).** For Reflex's reference humanoid (`humanoid20.json`), a 60 Hz soft limit would be pushed past its stop at the joint's torque cap by between 0.04 and 0.87 rad, depending on the joint and axis. Every joint misses the 0.02 rad check. The worst are the ankles, elbows and wrists, about 0.2 to 0.87 rad.
    - Method: torque cap / (effective inertia × (2π·60)²), using the pair's centre-of-mass inertias from the rig.
    - A per-island substep count would need about 27 substeps for the ankle.
  - **Decision.** Limit rows are therefore **rigid**. They have no softness, so the impulse fully cancels the closing velocity, and they correct position at bias C/h, capped at a joint correction speed of 3 rad/s. This is the stiffest form of "limit stiffness sized from the torque cap". Point and alignment rows keep Box3D's soft 60 Hz rows. If rigid limits jitter or still miss verification 3, the work stops and reports before going further.
- **Destruction.** Until pull request 5, joint forces do not load destruction bonds. Destruction re-centres and splits compounds. Joint anchors are re-expressed in the new body frame. When a compound splits, the joint stays with the piece that holds its anchor.
- **Solve order.** The rows are solved in `solveSoftstep`, using Box3D's joint mathematics.
  - Joints are prepared after body gather.
  - In each substep, joints warm start, solve with bias and relax without bias, each before the contact stage of the same pass.
  - Joints take no part in restitution.
- **Both solvers.** An island that contains a lattice body is solved by strainstep, in double precision. A ragdoll whose hand brushes a dented prop joins that island. So joint rows are written once over the scalar type and used by both softstep and strainstep, as contact rows already are (`rigidrows.h`). Joints are never silently dropped.
- **Determinism.** Joints are solved serially in id order. This keeps results bit-identical across worker counts and SIMD widths. With no joints the new stages do no work, so existing results are unchanged.
  - **My call:** a serial order is cheap at the expected count (fewer than 200 joints per world). Colouring joints for parallel solves is deferred until profiling asks for it.
- **Islands and sleep.**
  - Joints link islands in `build` and add edges to `split`'s union-find. Removal calls `markRemoved`.
  - Creating or editing a joint wakes both bodies.
  - Bodies are stored by id, because vector indices change when bodies are removed.

### Joint kinds

- **Ball:** three point rows.
- **Hinge:** point rows and two angular rows that keep the axis aligned, an optional angle range, and a drive about the axis. The axis is the parent frame's x.
- **Swing-twist:** point rows, a twist range about the child frame's x, and an **elliptical** swing cone. The cone has separate half-angles about y and z and an optional centre offset. It has a three-axis drive about the child frame's x, y and z.
  - The decomposition is q = q_swing · q_twist, Reflex's convention.
  - Box3D's cone is circular and capped at π/2, which forced Reflex's backend to widen its limits.
- **Drives (hinge and swing-twist).** Each axis has:
  - a target angle (for swing-twist, the target relative rotation);
  - a target velocity;
  - a stiffness k in N·m/rad and a damping c in N·m·s/rad.

  Each axis is its own soft row with physical k and c (Catto's soft constraints, GDC 2011). So the response does not depend on the joined bodies' masses.
  - Torque is bounded per axis and per direction, with a lower and an upper bound, and also by a cap on the torque vector's magnitude.
  - A world-space feed-forward torque is applied as equal and opposite body torques in every substep.
- **Spring joint.** Pulls body B's frame toward a target frame on A. The host may move the target every step.
  - It has linear and angular stiffness and damping in physical units, and caps on force and torque.
  - When a cap is reached, the joint slips instead of holding. This serves grips, a ragdoll tethered to its mover capsule, and the role of Box3D's motor joint.
- **Planar velocity joint (Box3D's mover joint).** Two rows in the plane normal to frame B's y. They drive B's velocity relative to A toward a target velocity.
  - The effective mass is 1/(m_A + m_B) with inertia ignored, so it never rotates either body.
  - The impulse magnitude is capped at h × maxForce.
  - It is named so as not to clash with the existing `World::movers` list.
- **Pogo joint.** One row along a ground normal the host supplies. It joins a ground point on A (a body or the world) to a bottom anchor on B.
  - It is an implicit spring-damper on c = dot(axis, d) − restLength.
  - Box3D's custom bias is used, so steps do not feed velocity into the body, and the relax pass has zero bias.
  - The impulse is clamped to [−h·maxTension, h·maxCompression].
  - The initial impulse and internal velocity are inputs, so a pogo recreated each step warm-starts.

### Capsule

- A new `RigidShape::Capsule`: a core segment along the body's y axis, a half-length and a radius.
- Mass and inertia come from the solid capsule. Bounds and ray tests are analytic.
- Contacts are generated against spheres, capsules, boxes, compound child boxes, and static and lattice fields. Against a field, the field is sampled along the core segment and the radius is subtracted.
- Manifolds stay within four points, so capsule pairs fit the wide SIMD lanes. Continuous collision treats a capsule like a sphere of its radius swept along its core.

### Collision filter

- **On each rigid body and static:**
  - a group (int32): bodies in the same negative group never collide, and bodies in the same positive group always collide (Box3D's rule);
  - category and mask bits (uint32).
- **Joined pairs.** A pair joined with `collideConnected` false never forms contacts. Creating such a joint drops the pair's existing manifold.
- **Where it applies.** The filter is applied where the broadphase keeps pairs and in `ManifoldCache::createPairs`.
- **Rays.** Rays take a filter (mask and skip group) as well as `RaySkip`, so the host can ignore a whole character.
- **Defaults.** Default values make everything collide, as today.

### World ray cast

The only ray query today, `rayBlockers` and `firstHit`, serves blast occlusion. It hits only compound children and mass-0 boxes, returns no point or normal, and scans every body. The pogo needs the ground point, normal and body every step.

- **A new world ray cast,** in pull request 2 before the pogo. It hits static fields, spheres, boxes, capsules, compound children, and sampled and lattice bodies.
  - It returns the body, child, point, normal and fraction.
  - It finds candidates through the broadphase and takes the collision filter.
- **The blast occlusion helper stays as it is,** so characters do not start shielding blasts (**my call**).

### Read-back

- **Per joint:** the last substep's constraint force and torque (impulse × substep rate), the drive torque per axis, and flags for each cap that was reached.
- **Per body:** contacts stay as they are (`contactsOf`, `normalForceOn`).

## Verification

"Done" means all of the following, with expectations from physics or published values, never tolerances fitted to a result.

- **No regressions.**
  - The existing suite passes with its one known expected failure.
  - The existing worker and width hash checks pass.
  - A new hash check compares a jointed scene across widths 1, 4 and 8 and workers 1, 4 and 8.
- **Behaviour at the default 4 substeps.**
  1. A hinge pendulum's small-angle period matches 2π√(L/g) within 2 %. Its energy does not grow over 60 s.
  2. A 20-body chain with a 40:1 mass ratio hangs and settles with no explosion and no creep over 60 s.
  3. With every drive of Reflex's reference humanoid pushing into its limit at its torque cap, with that rig's body masses, no hinge or cone-twist limit is exceeded by more than 0.02 rad (about 1°, **my call**).
  4. A drive's static deflection under a known torque is τ/k within 2 %. Its response frequency is √(k/I) within 5 %, for joined-mass ratios from 1 to 75. This is the planted-ankle case.
  5. Per-direction torque bounds and the magnitude cap hold.
  6. A spring joint capped at 50 N holds a 3 kg box and lets a 10 kg box slip and fall.
  7. A planar velocity joint brings a capsule to its target speed within its force cap and never rotates it.
  8. A pogo floats a capsule at its rest length and carries it up a 0.15 m step with no upward speed spike. Its tension and compression caps hold.
  9. A capsule rests and rolls correctly on a box, a sphere, a compound and a static field. Ray hits on capsules are exact.
  10. The group, mask and joined-pair filters work. Default filters reproduce today's contacts exactly.
  11. A jointed chain sleeps and wakes together. Removing a joint splits its island.
  12. A ragdoll touching a dented lattice prop keeps every joint within its tolerances while the shared island runs in strainstep.
  13. The world ray cast hits every shape kind with the exact point and normal, and skips filtered bodies.
  14. A sleeping body touched by a moving mass-0 body wakes. Reading the code suggests it may not today. If this fails, it is fixed separately before the mover work.
- **Cost.** Measured on the reference desktop in Release:
  - Four 20-body ragdolls standing on a floor add at most 0.2 ms per world step at 4 substeps (**my call**).
  - Steps per second for one ragdoll in an otherwise empty world are recorded, to plan training.
- **Outside this repository.** Reflex's Talus backend and its standing gate come after this work. They are not part of "done" here.

## Delivery

There are four pull requests, each useful and tested on its own:

1. Joint core, with ball, hinge, swing-twist, drives, the spring joint and the joined-pair exclusion.
2. World ray cast, then planar velocity and pogo joints.
3. Capsule shape.
4. Collision groups, masks and ray filters.
5. Joint loads on destructible structures (user, 2026-10-29). Joint forces reach the stress solver, so pulling on a part through a grip or hinge can crack and break its bonds. Until this lands, only contacts load structures.

Box3D is MIT-licensed. Ported code keeps its notice in `THIRD_PARTY_NOTICES.md`.

## Open questions

These do not block the work.

- **Substeps for ragdolls.** If ragdolls need more than 4 substeps, add a per-island substep count rather than raising the world's.
- **Colouring joints for parallel solves,** if many characters make the serial order costly.
- **Joints on deformable bodies.**

## Left to the builder

File layout, function names, exact row mathematics (following Box3D), test scenes and fixtures.
