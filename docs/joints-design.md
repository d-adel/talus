# Rigid joints, capsules and collision filters

Approved 2026-10-29. Source revision: `e5c97c3`. Pull request 1 was built on 2026-10-30. Its measured results, and the design changes made while building it, are in [Results of pull request 1](#results-of-pull-request-1).

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
- **Joints anchored on lattice or deformable bodies.** Joints attach to rigid bodies only. A rigid body with a joint stays in the rigid tier while it is jointed, because `switchTiers` skips it and `promote` refuses it. So a held prop does not dent while held (**my call**, to revisit after this work).
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
  - **Measured on 2026-10-30.** A serial joint-by-joint solve does not carry a stop through a chain within 4 substeps: chains overrun their limits by 0.15 to 1.3 rad, and a 20-link 40:1 chain blows up. At 16 substeps, Box3D's row maths hold most chains within about 0.03 rad.
  - **Owner's decision (2026-10-30).** Islands that contain a joint are stepped with their own substep count, 16 by default (`IntegrationParameters::jointSubsteps`); all other islands keep the world's 4. Joint rows stay Box3D's row maths. The limit tolerance is 0.03 rad.
  - **Earlier decision.** Limit rows are therefore **rigid**. They have no softness, so the impulse fully cancels the closing velocity, and they correct position at bias C/h, capped at a joint correction speed. This is the stiffest form of "limit stiffness sized from the torque cap". Point and alignment rows keep Box3D's soft 60 Hz rows. If rigid limits jitter or still miss verification 3, the work stops and reports before going further.
  - **Changed on 2026-10-30 (provisional; the owner reserved this decision).** The correction speed cap is 30 rad/s, not the 3 rad/s first planned. After each joint pass, two extra passes run over the limit rows of every joint that is pushing on a limit or is within 0.05 rad of one. Without both changes, the humanoid of verification 3 overshot its limits by 0.10–0.20 rad. See [Results](#results-of-pull-request-1).
- **Destruction.** Until pull request 5, joint forces do not load destruction bonds. Destruction re-centres and splits compounds. Joint anchors are re-expressed in the new body frame. When a compound splits, the joint stays with the piece that holds its anchor.
  - **As built.** The holder is the piece with a child box that contains the anchor, or else the child box nearest to it. On a tie, the piece that keeps the old body id wins, then the new pieces in creation order. Chunks that break off an anchored building take the same path (`splitAnchored`), because they leave through new compound bodies, not through a compound split. The joint's contact exclusion moves to the new pair, its warm-start impulses are kept, and both ends are woken.
- **Solve order.** The rows are solved in `solveSoftstep`, using Box3D's joint mathematics.
  - Joints are prepared after body gather.
  - In each substep, joints warm start, solve with bias and relax without bias, each before the contact stage of the same pass.
  - Joints take no part in restitution.
- **Both solvers.** An island that contains a lattice body is solved by strainstep, in double precision. A ragdoll whose hand brushes a dented prop joins that island. So joint rows are written once over the scalar type and used by both softstep and strainstep, as contact rows already are (`rigidrows.h`). Joints are never silently dropped.
  - **As built.** A strainstep island with a dynamic joint end steps at `jointSubsteps` or the lattice's own count, whichever is larger. Strainstep solves joints in each of its four passes per substep, and repeats the extra limit passes after each one.
  - Strainstep runs after softstep has moved mass-0 bodies to their end-of-step pose. A mass-0 joint end is therefore rewound to its start-of-step pose and advanced by its velocity, as softstep does. Without the rewind, the joint led a moving hand by one frame of travel.
- **Rotation clamp.** Jointed islands limit each body's turn to π/4 per substep, not per step, in both solvers. With a per-step clamp, a 30 Hz frame broke the hinge limit (0.657 rad against 0.53), because the clamp scaled the spin but not the velocity of the anchor. Now motion depends only on the substep length. The cost: a jointed body may turn more than π/4 in a frame, so continuous collision's linear rotation interpolation is approximate for it (Box3D's `allowFastRotation` makes the same trade).
- **Determinism.** Joints are solved serially in id order. This keeps results bit-identical across worker counts and SIMD widths. With no joints the new stages do no work, so existing results are unchanged.
  - **My call:** a serial order is cheap at the expected count (fewer than 200 joints per world). Colouring joints for parallel solves is deferred until profiling asks for it.
- **Islands and sleep.**
  - Joints link islands in `build` and add edges to `split`'s union-find. Removal calls `markRemoved`.
  - Creating or editing a joint wakes both bodies.
  - Bodies are stored by id, because vector indices change when bodies are removed.
  - **As built.** Removing a joint also wakes both bodies. Removing a body deletes its joints at once and wakes the bodies at their other ends, so no step has to look for joints on removed bodies. `setRigidPose` and `setVelocity` wake every body jointed to the edited body. At the end of each step, every body jointed to an awake, moving mass-0 body is woken, so a sleeping body follows a platform or hand that starts to move. A host that writes a mass-0 body's velocity directly bypasses both rules, as it does for contacts.

### Joint kinds

- **Ball:** three point rows.
- **Hinge:** point rows and two angular rows that keep the axis aligned, an optional angle range, and a drive about the axis. The axis is the parent frame's x.
- **Swing-twist:** point rows, a twist range about the child frame's x, and an **elliptical** swing cone. The cone has separate half-angles about y and z and an optional centre offset. It has a three-axis drive about the child frame's x, y and z.
  - The decomposition is q = q_swing · q_twist, Reflex's convention.
  - Box3D's cone is circular and capped at π/2, which forced Reflex's backend to widen its limits.
  - **As built.** Half-angles are accepted in [10⁻³, 0.89π] rad, and the centre offset plus the largest half-angle must not exceed 0.89π. Swing drive targets must not be longer than 0.89π. Near π, the twist row and a swing row point within about 1° of each other and fight: a 3.0 rad target spun the joint up after 4 s, while 2.8 rad (0.89π) stayed stable for every stiffness up to 10⁴ N·m/rad. Reflex's widest reach is 2.11 rad. The 10⁻³ rad minimum keeps the float cone row well conditioned; it stops working below about 10⁻¹⁰ rad. Hinge and twist limits are accepted within ±0.99π and drive targets within ±π; values outside are rejected, not clamped.
- **Drives (hinge and swing-twist).** Each axis has:
  - a target angle (for swing-twist, the twist angle and the two swing-vector components);
  - a target velocity;
  - a stiffness k in N·m/rad and a damping c in N·m·s/rad.

  Each axis is its own soft row with physical k and c (Catto's soft constraints, GDC 2011). So the response does not depend on the joined bodies' masses.
  - Torque is bounded per axis and per direction, with a lower and an upper bound, and also by a cap on the torque vector's magnitude.
  - A world-space feed-forward torque is applied as equal and opposite body torques in every substep.
  - **As built.** Each axis's bounds must allow zero torque (`minTorque` ≤ 0 ≤ `maxTorque`). A positive lower bound is a constant torque source, which is what the feed-forward torque is for, and a lower bound of +∞ made the first step non-finite.
  - **As built.** A swing-twist drive measures each axis's error in the swing-twist coordinates: the twist angle about frame B's x, and the two components of the swing vector. It does not use the shortest rotation to the target, because Reflex's shoulder twist range spans more than π, and the shortest rotation would drive through the far limit. The row directions, masses and softness are recomputed in every solve. With directions fixed for a whole step, a 1 kg limb escaped a hip cone by 1.73 rad. The magnitude cap limits the world torque vector; the per-axis bounds limit each angle's torque.
- **Spring joint.** Pulls body B's frame toward a target frame on A. The host may move the target every step.
  - It has linear and angular stiffness and damping in physical units, and caps on force and torque.
  - When a cap is reached, the joint slips instead of holding. This serves grips, a ragdoll tethered to its mover capsule, and the role of Box3D's motor joint.
  - **As built.** The target pose is A × frameA × target, so the default target pulls B's frame onto A's frame. Each block (linear and angular) is solved as one 3 × 3 implicit spring-damper with the drives' softness, using the block's effective-mass matrix. That keeps an off-centre grip physical in every direction. The caps limit the force and torque vectors in any direction.
  - `setSpringSettings` changes a spring's gains, caps and target without re-creating the joint, so a grip can weaken as a character tires.
  - The spring has no target-velocity input, and its damping acts on the relative velocity of A and B. A host that carries an object therefore drives a mass-0 body A (a kinematic hand) with `setRigidPose` and `setVelocity`, and the grip follows without lag. A world-anchored target moved by the host each step lags instead: 0.034 m at 1 m/s for a 3 kg box with c = 200 N·s/m.
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

"Done" means all of the following, with expectations from physics or published values, never tolerances fitted to a result. Behaviour checks of jointed bodies run at the jointed islands' substep count (16).

- **No regressions.**
  - The existing suite passes with its one known expected failure.
  - The existing worker and width hash checks pass.
  - A new hash check compares a jointed scene across widths 1, 4 and 8 and workers 1, 4 and 8.
- **Behaviour.**
  1. A hinge pendulum's small-angle period matches 2π√(L/g) within 2 %. Its energy does not grow over 60 s.
  2. A 20-body chain with a 40:1 mass ratio hangs and settles with no explosion and no creep over 60 s.
  3. With every drive of Reflex's reference humanoid pushing into its limit at its torque cap, with that rig's body masses, no hinge or cone-twist limit is exceeded by more than 0.03 rad (about 1.7°, owner's decision 2026-10-30, at the jointed islands' 16 substeps).
  4. A drive's static deflection under a known torque is τ/k within 2 %. Its response frequency is √(k/I) within 5 %, for joined-mass ratios from 1 to 75, up to the measured frequency range (a provisional decision of 2026-10-30, pending the owner). This is the planted-ankle case.
     - At the default 1/960 s substep (1/60 s × 16), the bound holds up to about 39 Hz at a mass ratio of 1, 19 Hz at 10 and 12 Hz at 75. The worst error in that range is 4.3 % slow. Measured on two 0.1 m crates hinged 0.1 m from each centre.
     - Above that range, stiff drives ring slower. At 78, 38 and 24 Hz (k = 200 N·m/rad on the same bodies), the frequency is 10 %, 7 % and 10 % low.
     - Raising `jointSubsteps` shortens the substep and restores the accuracy, at proportional cost. At 64 substeps (1/3840 s), the same 24–78 Hz drives are within 2 %, and 54–174 Hz drives are within 5 %.
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
  - A 20-body ragdoll standing on a floor costs at most about 0.25 ms per world step at its 16 substeps, so four cost about 1 ms (owner's decision 2026-10-30, replacing my 0.2 ms for four at 4 substeps).
  - Steps per second for one ragdoll in an otherwise empty world are recorded, to plan training.
- **Outside this repository.** Reflex's Talus backend and its standing gate come after this work. They are not part of "done" here.

## Results of pull request 1

Pull request 1 delivers ball, hinge, swing-twist and spring joints, physical drives, rigid limits, joined-pair contact exclusion and per-joint read-back. Softstep and strainstep both solve them. Items 7–10, 13 and 14 belong to later pull requests.

Measured on 2026-10-30 on Dilan's laptop (AMD Ryzen 7 7840HS, Windows 11, MSVC 19.44, Release). The runtime source is `55d5282` plus this pull request's `promote` change, which does not touch these cases. Every check below is a test in `tests/dynamics_joint_tests.cc`.

### Verification

| Item | Measured | Bound |
| --- | --- | --- |
| No regressions | 629 cases at the final head; all pass except 3 expected failures: the known ball/soft-face case and the two `[!mayfail]` cases in items 3 and 4. The eight hidden hash tags equal the baseline. | One known expected failure before this work |
| Width × worker hashes | Identical at widths 1, 4 and 8 × workers 1, 4 and 8 for six jointed scenes, in both solvers. One scene has two pinned humanoids pushing into a box pile, a ball chain on the pile, and a spring grip, so it covers all four kinds, limits and drives. | Identical |
| 1. Pendulum | Period within 0.15 % for ball and hinge. The peak swing never grows over 60 s, and the hinge sways at most 7 mm out of its plane. | 2 %; no growth; 10 mm |
| 2. 40:1 chain | Hung at rest, the lowest body moves 4.3 × 10⁻⁸ m over the last 10 s of 60 s. Released at 0.3 rad, it stays finite, peaks at 3.4 m/s and never gains energy. | 5 mm; 5 m/s; 1 % of the swing energy |
| 3. Humanoid limits at the torque caps, without self-contact | Worst 0.0268 rad past a limit with k = 10⁵ N·m/rad (left elbow), 0.0234 rad with Reflex's 10 Hz gains (lower neck) | 0.03 rad |
| 3. The same with self-contact | 0.0634 rad (left wrist cone), marked `[!mayfail]` until pull request 4 | 0.03 rad |
| 4. Drive deflection | Within 2 % of τ/k for the hinge and each swing-twist axis | 2 % |
| 4. Drive frequency, mass ratios 1, 10 and 75 | −0.59 %, −0.94 % and −1.40 % at k = 10 N·m/rad, inside the accurate range | 5 % |
| 4. The same above the range | 10.2 %, 6.7 % and 9.6 % slow at k = 200 N·m/rad, marked `[!mayfail]` | 5 % |
| 5. Torque bounds and cap | Hold | Hold |
| 6. Spring capped at 50 N | 3 kg sags 5.886 mm (m·g/k) and never saturates. 10 kg saturates at 50.000 N and falls 9.74 m in 2 s. | 0.02 m; more than 1 m |
| 11. Sleep and islands | A jointed chain sleeps and wakes together. Removing a joint splits its island. | Pass |
| 12. Joints in strainstep | A five-box hinge chain on a 96-cell lattice slab keeps its anchors within 2.47 mm. A hinge at its drive cap beside a lattice stops within 10⁻⁵ rad of its 0.5 rad limit, the same within 10⁻⁶ rad at 60 and 30 Hz. | 5 mm; 0.53 rad |
| Hinge limit at a 150 N·m cap, softstep | Within 10⁻⁵ rad of the 0.5 rad limit at 60 and 30 Hz | 0.53 rad |
| Elliptical cone and twist range at a 100 N·m cap | Ellipse value at most 1.022; twist within 10⁻⁵ rad of its limit | 1.21; 0.03 rad |
| Compound re-centring and splits | World anchors kept within 3.4 × 10⁻¹⁶ m. After a split, the contact exclusion moves to the piece that holds the anchor. | 10⁻⁹ m |

### Cost

The hidden case `talus_tests "[jointcost]"` stands unpinned humanoids on a floor, awake, with every drive axis at k = 10³ N·m/rad and c = 50 N·m·s/rad holding a standing pose. On the laptop, one humanoid costs 0.148 ms of joint solving and 0.225 ms of whole step per 1/60 s step at 16 substeps, or 4,434 steps per second. Four cost 0.191 ms of whole step each. On the reference desktop (Intel Core i7-10700KF, MSVC 19.50), one humanoid costs 0.167 ms of joint solving and 0.275 ms of whole step, and four cost 0.893 ms in all. Four humanoids fit the 1 ms budget. One humanoid is 10 % over its 0.25 ms budget, but that includes self-contact work that Reflex would not have. The test humanoid collides with itself until pull request 4. [Physics status](physics-status.md#joints) gives the full tables, what self-contact adds, and an estimate for the collision-group configuration.

Two costs are larger:

- **Every limit engaged.** With every drive pushing into its limit at its cap, the extra limit passes add 24 % to the joint solve (0.157 to 0.195 ms per humanoid). In normal motion inside the limits they add 0.9–1.6 %, within the run-to-run spread.
- **A ragdoll touching a lattice.** The shared strainstep island steps at 16 substeps. A five-box chain on a 96-cell slab costs 3.0 ms per step against 1.0 ms at 4 substeps, while they touch.

### Changes from this design

The "As built" notes above give the details. In short:

1. Islands that contain a joint step at `jointSubsteps` (16); others keep 4 (decided by the owner on 2026-10-30).
2. The limit correction cap is 30 rad/s, with two extra limit passes for joints at or near a limit (provisional).
3. Jointed islands clamp rotation per substep, so motion depends only on the substep length.
4. `jointSubsteps` is a count per frame. A 30 Hz host needs 32 for the same substep length, and so the same joint behaviour, as 16 at 60 Hz. Before the extra limit passes, 16 substeps at 30 Hz let the hinge limit test reach 0.539 rad; with them it stops within 10⁻⁵ rad of the limit, but drive accuracy still depends on the substep length.
5. Out-of-range limits and targets are rejected. Swing reach is bounded at 0.89π.
6. Swing-twist drives work in Reflex's swing-twist coordinates, with directions recomputed every solve.
7. The spring target is A × frameA × target. Springs have no target velocity, so hosts drive a mass-0 body A.
8. Strainstep steps jointed islands at `jointSubsteps`, solves joints in each of its four passes, and rewinds mass-0 joint ends.
9. Editing the pose or velocity of a body wakes its joint partners, and a moving mass-0 body wakes its partners at the end of each step.
10. `promote` refuses a joint end. Before, it promoted the body, and its joints were then deleted silently at the next step.
11. Joints move to the piece that holds their anchor through compound splits and through chunks that leave anchored buildings.
12. Removing a body deletes its joints at once and wakes the bodies at their other ends. Before, the next step deleted them and left the other end asleep, so a box hung from a removed beam stayed in the air.
13. Hosts read joint records but change them only through the `jointedits.h` functions. Drive bounds must allow zero torque.

### Open questions for the owner

These are open, not decided:

- **Humanoid self-collision.** The all-limits test with self-contact stays `[!mayfail]`. Contacts are solved after the joints and push the hand past its cone. Should it pass once pull request 4's collision groups let the test humanoid turn self-collision off, as Reflex does?
- **Destroyed anchor chunks.** When the chunk that holds a joint's anchor is destroyed, the joint moves to the piece with the nearest remaining child. Should it break instead?
- **The 30 rad/s snap.** A joint far outside its limits now returns within about one frame. When every humanoid joint starts 0.4 rad outside its limits, the peak body spin is 104 rad/s and the widest anchor gap 0.047 m, against 35 rad/s and 0.026 m at 3 rad/s. The owner reserved this decision.

## Delivery

There are five pull requests, each useful and tested on its own:

1. Joint core, with ball, hinge, swing-twist, drives, the spring joint and the joined-pair exclusion.
2. World ray cast, then planar velocity and pogo joints.
3. Capsule shape.
4. Collision groups, masks and ray filters.
5. Joint loads on destructible structures (user, 2026-10-29). Joint forces reach the stress solver, so pulling on a part through a grip or hinge can crack and break its bonds. Until this lands, only contacts load structures.

Box3D is MIT-licensed. Ported code keeps its notice in `THIRD_PARTY_NOTICES.md`.

## Open questions

These do not block the work.

- **Whole-chain solving.** If 16 substeps still leave ragdolls floppy or unstable at extreme mass ratios, solve a character's joints together (articulation or sparse direct solve). Not planned.
- **Colouring joints for parallel solves,** if many characters make the serial order costly.
- **Joints on deformable bodies.**

## Left to the builder

File layout, function names, exact row mathematics (following Box3D), test scenes and fixtures.
