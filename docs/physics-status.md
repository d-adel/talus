# Physics status

This extraction carries the physics integrated on 2026-10-19, with its existing
limitations. Rigid and deformable stepping, collision, plasticity, tearing and
structural destruction are retained. Rendering, ECS and engine-specific scene
bridges are removed. The old desktop-editor bridge tests remain with that engine;
physics assertions retained here are not relaxed.

The controller's fitted cost constants are preserved. They remain unqualified:
fresh reference checks found underpredictions, and the large district exceeded
the intended 4 ms physics / 16.7 ms rendered frame goals. The controller does not
guarantee a wall-clock deadline. The source description in `budget.h` now points
here instead of to an internal research archive.

One known ball/soft-face contact test can fail as expected. Rubble can remain
awake. The fracture hierarchy and contact-load fracture measure do not establish
convincing thin-sheet glass shattering. These limitations are not repaired by
removing the game engine.

The first extracted build targets x86-64 and retains the runtime SIMD selection.
Other architectures are not qualified. Windows/MSVC Release checks pass: 556 physics cases, including one known
expected failure, and 818,195 assertions. The headless example and a separate
installed-package consumer pass. All 76 public headers compile through the
exported CMake target. A host project also configures with add_subdirectory,
without enabling tests or downloading dependencies. 37 deterministic hash records match the original implementation on Windows.
Linux/GCC 14.2 Release also passes all 556 cases with the same known expected
failure (818,194 assertions). All 37 deterministic records agree with Windows.
The public verification uses the exact runtime source commit 9af39ac3; later
migration documentation changes do not alter the solver. Independent review
found no blocking issues. The private campaign archive is not published with this library.

## Joints

The first joint pull request (2026-10-30) adds ball, hinge, swing-twist and
spring joints, physical drives, rigid limits, joined-pair contact exclusion and
per-joint read-back. Softstep and strainstep both solve them. Usage is in
[integration](integration.md#joints). The design, its measured results and the
decisions made while building it are in [joints design](joints-design.md).

### Checks

- Windows/MSVC Release: 629 physics cases. All pass except 3 expected
  failures: the known ball/soft-face case and two joint cases marked
  `[!mayfail]`, described below. The headless example passes.
- The eight hidden hash tags print the same records as before the joint work,
  so worlds without joints are unchanged.
- Jointed scenes give identical hashes at SIMD widths 1, 4 and 8 and worker
  counts 1, 4 and 8, in both solvers. One of them has two pinned humanoids
  pushing into a box pile, with all four joint kinds, limits and drives.
- Linux/GCC 13 Release with `-Werror`, on the lab's spare machine: the build
  and the full physics suite passed at `55d5282` and at `3a1f382`, before the
  final review fixes. **Pending: the same run at the final head commit.
  [Result goes here.]** The GCC 14 checks named elsewhere predate the joint
  work.

### Cost

The hidden case `talus_tests "[jointcost]"` prints these numbers. It stands
unpinned 20-body humanoids on a static floor, with sleep off. Every drive axis
holds a standing pose with k = 10³ N·m/rad and c = 50 N·m·s/rad at the rig's
torque caps. Each run is 600 steps of 1/60 s, with jointed islands at 16
substeps and one worker. "Joint solve" is `StepProfile::joints`; "whole step"
is `StepProfile::total`. Each value is the mean over 600 steps, and the median
of 3 runs.

Laptop (AMD Ryzen 7 7840HS, Windows 11, MSVC 19.44, Release, 8-wide contacts),
runtime source `55d5282` plus this pull request's `promote` change:

| Scene | Joint solve | Whole step | Steps per second |
| --- | --- | --- | --- |
| 1 humanoid, self-contact | 0.148 ms | 0.225 ms | 4,434 |
| 1 humanoid, inert springs | 0.259 ms | 0.311 ms | 3,215 |
| 4 humanoids, self-contact | 0.589 ms (0.147 each) | 0.765 ms (0.191 each) | 1,306 |
| 4 humanoids, inert springs | 1.027 ms (0.257 each) | 1.161 ms (0.290 each) | 861 |

Single runs varied by up to 17 %, and the medians by about 5 % between
invocations. A re-run after the final review fixes, which only remove
per-step work, matched these medians within 4 %.

Reference desktop: **pending, `[jointcost]` at the final head commit.
[Result goes here.]** The design's budget is about 0.25 ms per world step for
one humanoid there.

How to read the two configurations:

- **Self-contact.** The test humanoid collides with itself, because collision
  groups arrive in pull request 4. Its hands touch its thighs and its upper
  arms start 3.5 cm inside its chest: 64 contact points on 16 body pairs per
  humanoid. Reflex runs its characters without self-collision, so this row
  includes contact work that Reflex would not have.
- **Inert springs.** Today the only way to turn self-contact off is a joint
  between every pair of bodies that should not collide. `excludeSelfContact`
  in the test fixture adds 171 spring joints with zero gains per humanoid.
  They apply no force, but the solver still visits them, which adds about
  0.11 ms of joint solve per humanoid.
- Neither row is the configuration that collision groups will give. Its cost
  should be about the self-contact row's joint solve plus the inert-spring
  row's other work: about 0.20 ms per step for one humanoid on the laptop.
  This is an estimate, not a measurement.
- In this pose the knees and elbows rest on their straight-limb limits. So
  four joints per humanoid get the extra limit passes in every substep (256
  limit rows per step).
- The build pose of the test humanoid balances its centre of mass 5 cm in
  front of its heels. Without self-contact it tips backwards within 3 s.
  While it tips, every joint stays within 0.03 rad of its target, so the whole
  body falls like a statue. The standing pose therefore pitches the ankles 0.02 rad forward.
  Without self-contact, every pitch from 0.01 to 0.06 rad stood, and 0.02 rad
  swayed least.

Other measured costs, on the same laptop:

- **Every limit engaged.** With every drive of the humanoid pushing into its
  limits at its cap, the extra limit passes add 24 % to the joint solve
  (0.157 to 0.195 ms per humanoid). In normal motion inside the limits they add
  0.9–1.6 %, within the run-to-run spread.
- **A jointed body touching a lattice.** The shared island runs in strainstep
  at 16 substeps, and the lattice is stepped at that count too. A five-box
  hinge chain on a 96-cell slab costs 3.0 ms per step, against 1.0 ms at 4
  substeps. This is far above the ragdoll budget while they touch.
- **Joints in a world with many other bodies.** Four sleeping 19-joint
  chains (76 joints) were added after 0, 5,000 or 20,000 other sleeping
  bodies. A step cost 0.004, 0.008 and 0.02 ms more than the same world
  without joints, within the run-to-run spread of the 1.4 ms step at 20,000
  bodies. Removing a body deletes its joints at once, so no step searches the
  world for them. Before this was fixed, that search cost 0.33 ms per step at
  5,000 bodies and 1.8–2.0 ms at 20,000. Measured with a probe outside the
  suite.
- **A jointed island beside a large free pile.** A ball-jointed pair 5 m from
  an awake pile of 405 crates adds about 0.03 ms per step to the substep
  stage, because the jointed solve batches only its own contacts. The whole
  step grows by 0.06–0.16 ms, mostly for the second solve call's serial pass
  over the world's contact pairs.

### Known limits

- **Self-collision.** The case "every humanoid limit holds with its drive at
  the cap" runs with self-contact and is marked `[!mayfail]`. Contacts are
  solved after the joints and push a hand 0.063 rad past its cone, against a
  0.03 rad bound. Without self-contact the same push stays within 0.027 rad.
- **Stiff drives ring slow.** The case "a hinge drive rings at sqrt(k over I)
  for any mass ratio" is marked `[!mayfail]`. At k = 200 N·m/rad, 24–78 Hz,
  the drive rings 7–10 % slower than √(k/I). Within the accurate range (about
  39, 19 and 12 Hz at mass ratios 1, 10 and 75) the error stays below 5 %.
- **Hard snap.** A joint that is outside its limits is pushed back at up to
  30 rad/s. When every humanoid joint starts 0.4 rad outside its limits, the
  peak body spin is 104 rad/s and the widest anchor gap 0.047 m.
- **Two solvers, two iteration counts.** Strainstep solves joints in each of
  its four passes per substep; softstep solves them once. Before the extra
  limit passes, a hinge at its drive cap stopped at 0.50006 rad in strainstep
  and 0.50147 rad in softstep; now both stop within 10⁻⁵ rad of the 0.5 rad
  limit. Drives and springs still get four solves per substep in strainstep,
  so an island that moves between the solvers may change its joint feel
  slightly; this is not measured. Strainstep also repeats the extra limit
  passes after each pass; that cost is not timed.
- **Fast turns.** A jointed body may turn more than 45° in one frame, so
  continuous collision is approximate for it.
- **Touched piles pay too.** A jointed body that touches a dynamic pile joins
  the pile's island, so the whole contact-connected pile steps at 16 substeps
  while they touch. This is not measured yet.
- **Not measured yet:** the first substep after a compound split moves a
  joint's warm-start impulse onto a much lighter piece; the cost of
  re-expressing joints in re-centres and splits; and the wake of every joint
  on an anchored building each time a chunk breaks off it.
- **Owner questions, open:** whether the self-contact case should pass once
  collision groups land, and whether a joint should break when the chunk that
  holds its anchor is destroyed. Today it moves to the nearest remaining
  piece.
