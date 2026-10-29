# Joint Core (Pull Request 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add rigid joints to Talus: ball, hinge, swing-twist and spring joints, physical per-axis drives, rigid limits, joined-pair exclusion and read-back. Both rigid solvers solve them, with no change to worlds without joints.

**Architecture:**
- A `dynamics/joints` module holds joint definitions in `dynamics::World`, with a small editing API.
- Header-only row templates (`jointrows.h`) hold the joint mathematics once. `softstep.cc` instantiates them in float and `strainstep.cc` in double, solving joints serially in id order next to the contact stages.
- Joints link islands like heavy contacts.
- Point and alignment rows are Box3D-style soft rows (60 Hz, damping ratio 2). Limits are rigid rows. Drives are soft rows with physical stiffness and damping.

**Tech Stack:** C++23, glm, Catch2 v3.7.1, CMake with Ninja, MSVC 2022 BuildTools on Windows (GCC 14 on Linux).

**Spec:** `docs/joints-design.md` (approved 2026-10-29). PR 1 of 5. Ray cast, mover and pogo, capsule, filters and structure loads get their own plans.

## Global Constraints

- C++23, the repository's clang-format, MSVC `/W4 /WX`, GCC `-Wall -Wextra -Wpedantic -Werror -ffp-contract=off`. No fast-math.
- New code carries no comments (owner's rule).
- A world without joints does exactly the same work and gives bit-identical results. The hidden hash cases print the same values as the baseline in Task 0.
- Results are bit-identical across SIMD widths 1, 4 and 8 and worker counts 1, 4 and 8. Joints are solved serially in ascending joint id.
- Joint ids start at 1. 0 means invalid. Bodies are referenced by `RigidBody::id`, never by vector index.
- Body A may be `noBody`, which means the world. Statics are always the world.
- Defaults: row hertz 60, damping ratio 2, both clamped to a quarter of the substep rate. Rigid limit correction speed is capped at 3 rad/s.
- Islands that contain a joint step with `IntegrationParameters::jointSubsteps` (default 16); every other island keeps `softstepSubsteps` (4). Joint rows follow Box3D's row maths. Limit tolerance 0.03 rad (owner's decision 2026-10-30).
- Ported Box3D code keeps Box3D's MIT notice in `THIRD_PARTY_NOTICES.md` (source: commit 5d83df83, copy at `C:\Users\dilan\Documents\GitHub\reflex\build-dev\_deps\box3d-src`).
- **BUILD**, in PowerShell: `cmd /c '"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" && cd /d C:\Users\dilan\Documents\GitHub\talus && cmake --preset release && cmake --build --preset release'`
- **TEST tag**: `C:\Users\dilan\Documents\GitHub\talus\build-release\tests\talus_tests.exe "<tag>"`. The full suite is `cmake --workflow --preset verify` inside the same vcvars call.
- New sources are listed in `cmake/sources.cmake`. New test files go in `tests/CMakeLists.txt`.

## Review Focus

1. **A jointed body is removed** by the host or by destruction. The joint disappears at the next step, with no crash and no stale index. Test in Task 1.
2. **Degenerate input:** the same body twice, unknown ids, a zero quaternion, negative stiffness or damping, NaN. `addJoint` returns 0, setters return false, and nothing non-finite reaches the solver. Test in Task 1.
3. **A sleeping jointed island gets a new drive target.** It wakes; it does not silently ignore the drive. Test in Task 2.
4. **A jointed switchable body is hit hard enough to dent.** It stays rigid while jointed. Test in Task 8.
5. **A compound with a joint re-centres or splits.** The joint's world anchor is unchanged, and the joint follows the piece that holds its anchor. Test in Task 9.

---

### Task 0: Baseline

**Files:** none in the repository. Write outputs to the plan workspace.

- [ ] **Step 1:** On branch `joints-design` at `edbef4d` (design docs only), run BUILD, then the full `verify` workflow. Save the output as `baseline-suite.txt`.
  Expected: 556 cases, one known expected failure (the ball/soft-face contact case), everything else passing.
- [ ] **Step 2:** Run each hidden hash case: `[speedhash]`, `[rigidhash]`, `[loosehash]`, `[threadhash]`, `[compoundhash]`, `[stresshash]`, `[destructionhash]`, `[districthash]`. Save the printed values as `baseline-hashes.txt`.
  Expected: every case prints its hash records.

### Task 1: Joint records and the editing API

**Files:**
- Create: `src/talus/dynamics/joints/joints.h`, `joints.cc` (types, `JointSet`)
- Create: `src/talus/dynamics/joints/jointedits.h`, `jointedits.cc` (World API)
- Modify: `src/talus/dynamics/pipeline/pipeline.h` (add `JointSet joints;` to `World`)
- Modify: `src/talus/dynamics/pipeline/pipeline.cc` (prune at step start)
- Modify: `cmake/sources.cmake`, `tests/CMakeLists.txt`
- Test: `tests/dynamics_joint_tests.cc` (new)

**Interfaces (produces), all in `talus::dynamics`:**
- `enum class JointKind : uint8_t { Ball, Hinge, SwingTwist, Spring };`
- `struct JointFrame { glm::dvec3 position{0.0}; glm::dquat rotation{1.0, 0.0, 0.0, 0.0}; };`
  - The frame is relative to the body's centre-of-mass frame (`RigidBody::position` and `rotation`).
- `struct AxisDrive { double targetAngle = 0.0; double targetVelocity = 0.0; double stiffness = 0.0; double damping = 0.0; double minTorque = -inf; double maxTorque = +inf; };`
- `struct JointDrive { std::array<AxisDrive, 3> axes{}; double maxTorque = +inf; glm::dvec3 feedForward{0.0}; };`
  - Hinge uses `axes[0]`, about frame A's x.
  - Swing-twist uses x for twist and y and z for the swing vector, in frame B's axes.
- `struct HingeLimit { bool enabled = false; double lower = 0.0; double upper = 0.0; };`
- `struct ConeLimit { bool enabled = false; double twistLower = 0.0; double twistUpper = 0.0; glm::dvec2 swing{0.0}; glm::dvec2 centre{0.0}; };`
  - `swing` holds the half-angles about y and z.
- `struct SpringSettings { JointFrame target{}; double linearStiffness = 0.0; double linearDamping = 0.0; double maxForce = +inf; double angularStiffness = 0.0; double angularDamping = 0.0; double maxTorque = +inf; };`
- `struct JointDef { JointKind kind = JointKind::Ball; uint32_t bodyA = noBody; uint32_t bodyB = noBody; JointFrame frameA{}; JointFrame frameB{}; double hertz = 60.0; double dampingRatio = 2.0; bool collideConnected = false; HingeLimit hinge{}; ConeLimit cone{}; JointDrive drive{}; SpringSettings spring{}; };`
- `struct JointReport { glm::dvec3 force{0.0}; glm::dvec3 torque{0.0}; glm::dvec3 driveTorque{0.0}; bool limitActive = false; bool driveSaturated = false; bool forceSaturated = false; bool torqueSaturated = false; };`
- `struct Joint { uint32_t id = 0u; JointDef def; JointReport report; JointImpulses impulses; };`
  - `JointImpulses` holds the warm-start impulses of every row, in double.
- `class JointSet`, with:
  - `[[nodiscard]] uint32_t add(const JointDef&)`;
  - `bool remove(uint32_t)`;
  - `Joint* find(uint32_t)` and its const version;
  - `[[nodiscard]] std::span<const Joint> all() const` (ascending id);
  - `[[nodiscard]] bool excludes(uint32_t a, uint32_t b) const`;
  - `[[nodiscard]] bool empty() const`.
- In `jointedits.h`:
  - `[[nodiscard]] uint32_t addJoint(World&, const JointDef&)`;
  - `bool removeJoint(World&, uint32_t joint)`;
  - `bool setJointDrive(World&, uint32_t joint, const JointDrive&)`;
  - `bool setSpringTarget(World&, uint32_t joint, const JointFrame&)`;
  - `[[nodiscard]] std::optional<JointReport> jointReport(const World&, uint32_t joint)`;
  - `void pruneJoints(World&)`.

- [ ] **Step 1: Write the failing tests** (tag `[dynamics][joints]`):
  - `"joint ids are stable and start at 1"`: two joints get ids 1 and 2. After removing 1, a new joint gets 3. `jointReport(world, 1)` is `nullopt`.
  - `"invalid joint definitions are rejected"`: each of these returns 0 and leaves `world.joints.empty()`:
    - bodyA == bodyB;
    - an unknown body id;
    - bodyA == bodyB == `noBody`;
    - a frame rotation of (0,0,0,0);
    - a NaN position;
    - a negative stiffness, damping or `maxForce`;
    - `minTorque > maxTorque`;
    - an enabled hinge limit with `lower > upper`.
  - `"setters reject unknown joints and invalid drives"`: `setJointDrive(world, 99, {})` is false, and so is a drive with a NaN target.
  - `"a removed body's joints disappear at the next step"`: two boxes are joined. One box is removed through the existing removal edit. After one `step`, `jointReport` is `nullopt` and the step report is finite.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: build failure, because `jointedits.h` does not exist.
- [ ] **Step 3: Implement** the types, `JointSet` (a sorted vector plus a count map of excluded pairs keyed by (min id, max id)), the API and validation.
  - `pruneJoints` drops joints whose body ids no longer resolve. It is called at the very start of `dynamics::step` and returns immediately when `joints.empty()`.
  - `addJoint`, `setJointDrive` and `setSpringTarget` wake both bodies through `wakeBody`.
- [ ] **Step 4: Run** BUILD, TEST `[joints]`, then TEST `[dynamics]`.
  Expected: the new tests pass and `[dynamics]` is unchanged against `baseline-suite.txt`.
- [ ] **Step 5: Commit** `Add joint records and a joint editing API`.

### Task 2: Islands, sleep and joined-pair exclusion

**Files:**
- Modify: `src/talus/dynamics/islands/islands.cc` and `islands.h` (joint edges)
- Modify: `src/talus/dynamics/pipeline/pipeline.cc`
- Modify: `src/talus/dynamics/manifoldcache/manifoldcache.cc` and `.h` (exclusion, `forgetPair`)
- Test: `tests/dynamics_joint_tests.cc`

**Interfaces:**
- Consumes `JointSet::all()` and `excludes()`.
- Produces:
  - `IslandStore::build(..., const JointSet& joints, ...)` and `split(..., const JointSet& joints)` as extra trailing parameters;
  - `ManifoldCache::createPairs(..., const JointSet* joints)`;
  - `void ManifoldCache::forgetPair(uint32_t idA, uint32_t idB)`.

- [ ] **Step 1: Write the failing tests:**
  - `"a jointed chain sleeps and wakes together"`: five boxes in a ball-joint chain rest on the floor until all are asleep, which is `REQUIRE`d within 600 steps. An impulse on box 0 then wakes all five within one step.
  - `"removing a joint splits its island"`: two jointed boxes share an island. After `removeJoint` and 5 steps, `islandOf(a) != islandOf(b)`.
  - `"a new drive target wakes a sleeping jointed island"`: a sleeping chain gets `setJointDrive` on its first joint. All bodies are awake after one step.
  - `"joined pairs never collide unless asked"`: two overlapping boxes joined by a ball joint have no `contactsOf` entry between them for 60 steps. The same scene with `collideConnected = true` has contacts within 2 steps.
  - `"adding an excluding joint forgets the existing manifold"`: two touching boxes have a manifold. After `addJoint` and one step, they have none.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: the new cases fail, because islands ignore joints and contacts persist.
- [ ] **Step 3: Implement.**
  - Joints whose two ends are heavy rigid bodies call `link` in `build` and join the union-find in `split`. Removal requests a split of the joint's island.
  - `createPairs` skips pairs that `excludes()` reports.
  - `addJoint` with `collideConnected` false calls `forgetPair`.
  - With no joints, no new code path runs.
- [ ] **Step 4: Run** TEST `[joints]` and TEST `[dynamics]`.
  Expected: pass, with `[dynamics]` unchanged.
- [ ] **Step 5: Commit** `Link jointed bodies into islands and skip joined-pair contacts`.

### Task 3: Shared row maths, ball joint and read-back in softstep

**Files:**
- Create: `src/talus/dynamics/joints/jointrows.h` (templates over the real type and the state type)
- Modify: `src/talus/dynamics/softstep/softstep.cc` and `softstep.h` (stages)
- Modify: `src/talus/dynamics/pipeline/pipeline.cc` (pass `world.joints`)
- Modify: `src/talus/dynamics/pipeline/pipeline.h` (`StepProfile::joints`)
- Test: `tests/dynamics_joint_tests.cc`

**Interfaces:**
- `template <typename Real> struct JointRow;` holds the per-step prepared rows for one joint: state indices, world-frame lever arms, row masses, softness, and the kinematic-end velocities.
- The templates, where `State` is any type with `v, w, dp, dq, invMass, invI`:
  - `template <typename Real, typename State> void prepareJointRows(const JointSet&, const BodySet&, const IslandStore&, std::span<const State>, std::span<const uint32_t> stateOf, Real h, std::vector<JointRow<Real>>&);`
  - `template <typename Real, typename State> void warmStartJointRows(std::span<State>, std::span<const JointRow<Real>>);`
  - `template <typename Real, typename State> void solveJointRows(std::span<State>, std::span<JointRow<Real>>, Real invH, bool useBias);`
  - `template <typename Real> void storeJointRows(JointSet&, std::span<const JointRow<Real>>, Real invH);`
- Inputs: `solveSoftstep` gains `JointSet* joints` (nullptr or empty means no joint work).

**Rules:**
- A joint is solved in a softstep call when at least one end is in that call's state set and the other end is in the set, the world, or a mass-0 body.
- A mass-0 end outside the set is a kinematic end. It uses its body's velocity and angular velocity, and its anchor advances by them every substep.
- Order, after Prepare: `prepareJointRows`. In each substep, before the overflow contacts of the same pass: `warmStartJointRows`, then `solveJointRows(useBias = true)`, then after Advance, `solveJointRows(useBias = false)`. `storeJointRows` runs with StoreMemory.
- The report holds force = last substep's linear impulse × substep rate, and torque likewise.

- [ ] **Step 1: Write the failing tests:**
  - `"a ball pendulum keeps its period and energy"`: a 1 kg box hangs from the world on a 1 m lever and is released at 0.1 rad.
    - The measured period is within 2 % of 2π√(L/g), with L the distance from pivot to centre of mass.
    - Over 60 s the peak swing never exceeds the initial 0.1 rad by more than 1 %.
  - `"a hanging joint reports its load"`: a 3 kg box at rest under the world. `jointReport().force.y` is within 1 % of 3 · 9.81 N.
  - `"a joint rides a moving mass-0 body"`: a box hangs from a mass-0 body moving at 1 m/s along x. After 2 s, the box's horizontal offset from the anchor is below 0.01 m.
  - `"jointed worlds step the same at widths 1, 4 and 8 and workers 1, 4 and 8"`: an 8-box ball chain falling onto a box pile, hashed over 120 steps. It follows the pattern of `"worlds step the same at contact widths 1, 4 and 8"` in `dynamics_tests.cc`.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: the new cases fail, because the pendulum falls freely.
- [ ] **Step 3: Implement** the ball joint's three soft point rows (Box3D `spherical_joint.c` point block) in `jointrows.h`, and wire the stages into softstep.
- [ ] **Step 4: Run** TEST `[joints]`, TEST `[dynamics]` and the eight hidden hash tags.
  Expected: the new tests pass and the hashes equal `baseline-hashes.txt`.
- [ ] **Step 5: Commit** `Solve ball joints in softstep with shared row templates`.

### Task 4a: Per-island substeps for jointed islands

**Files:** modify `src/talus/dynamics/integrationparameters/integrationparameters.h` (`uint32_t jointSubsteps = 16u;`), `src/talus/dynamics/pipeline/pipeline.cc` (softstep islands with a joint are solved in their own `solveSoftstep` call at `jointSubsteps`; the rest as today), tests in `tests/dynamics_joint_tests.cc`.

- [ ] **Step 1: Write the failing tests:** `"jointed islands step at the joint substep count"` (a scene with one ball-jointed pair and one free box: the jointed island's solve reports `jointSubsteps` substeps and the free box's island reports 4, through a counter exposed in `StepReport`); `"worlds without joints step exactly as before"` (the eight hidden hash tags equal the baseline — run, not asserted in code).
- [ ] **Step 2: Run** BUILD and TEST `[joints]`. Expected: the new case fails.
- [ ] **Step 3: Implement.** Islands are split deterministically by whether any member has a joint; the jointed call runs first; with no joints there is one call exactly as before.
- [ ] **Step 4: Run** TEST `[joints]`, the full binary, and the eight hidden hash tags. Expected: pass, hashes equal baseline.
- [ ] **Step 5: Commit** `Step jointed islands at their own substep count`.

### Task 4: Hinge with rigid limits and a physical drive

**Files:** modify `jointrows.h`. Test: `tests/dynamics_joint_tests.cc`.

**Interfaces:**
- Consumes the Task 3 templates. The Hinge branch adds two soft alignment rows, two rigid limit rows (lower and upper, speculative) and one drive row about frame A's x.
- Drive row softness from physical k and c with effective mass m: ω = √(k/m), ζ = c / (2√(k·m)). The `makeSoft` formula is used with no frequency clamp. The impulse is clamped per axis to [h·minTorque, h·maxTorque], then the drive impulse vector to h·maxTorque. Feed-forward adds h·feedForward as equal and opposite angular impulses.
- Rigid limit row: massScale 1, impulseScale 0, one-sided impulse.
  - When the limit is still open (C > 0), the bias is the speculative C/h.
  - When it is violated (C < 0), the correcting bias has magnitude min(|C|/h, 3 rad/s).
  - Bias is used only when `useBias`.

- [ ] **Step 1: Write the failing tests:**
  - `"a hinge pendulum keeps its period"`: the same as the ball test, but swinging about the hinge axis.
  - `"a hinge drive deflects by torque over stiffness"`: a hinge to the world with k = 200 N·m/rad and c = 20. A constant 10 N·m feed-forward in zero gravity settles at 0.05 rad within 2 %.
  - `"a hinge drive rings at sqrt(k over I) for any mass ratio"`: the parent is a dynamic box of mass M, the child 1 kg, with M in {1, 10, 75} kg and c = 0. After a 0.05 rad offset, the oscillation frequency is within 5 % of √(k/I_r)/(2π).
    - I_r is the reduced inertia of the two bodies about the hinge axis through the anchor.
  - `"hinge drive torque respects per-direction bounds and the cap"`: with minTorque −5 and maxTorque 2, a target 1 rad away gives a measured `driveTorque.x` within [−5, 2] N·m, and the cap 1 limits it to [−1, 1].
  - `"a hinge limit holds at the drive cap"`: a 1 kg child, limit [−0.5, 0.5] rad, drive target 2 rad with k = 10⁵ and cap 150 N·m. Over 2 s the angle never exceeds 0.53 rad.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: these cases fail, because the hinge branch is missing.
- [ ] **Step 3: Implement** the hinge rows.
- [ ] **Step 4: Run** TEST `[joints]` and TEST `[dynamics]`.
  Expected: pass.
- [ ] **Step 5: Commit** `Add hinge joints with rigid limits and physical drives`.

### Task 5: Swing-twist with an elliptical cone and a three-axis drive

**Files:** modify `jointrows.h`. Test: `tests/dynamics_joint_tests.cc`.

**Interfaces:**
- Decomposition: q_rel = q_swing · q_twist about frame B's x. The twist angle and the swing vector (y, z) follow Reflex's `core/swingtwist/swingtwist.cc:51-72` exactly, with the same decomposition and the same swing parameterisation, so Reflex's targets mean the same thing in Talus.
- The cone is the ellipse (s_y − c_y)²/a² + (s_z − c_z)²/b² ≤ 1. Its row normal is the ellipse gradient at the current swing. It is rigid, one-sided and speculative.
- The twist range uses two rigid rows.
- The drive has three soft rows about frame B's x, y and z, with Task 4's per-axis softness, bounds, cap and feed-forward.

- [ ] **Step 1: Write the failing tests:**
  - `"a swing-twist cone holds an ellipse"`: half-angles 0.6 and 0.3 rad with centre (0.1, 0). The drive pushes at a 100 N·m cap toward each of 8 directions at 2 rad. The ellipse value never exceeds (1 + 0.03/0.3)².
  - `"a swing-twist twist range holds"`: range [−0.4, 0.2] rad, driven at the cap past each end. The overshoot stays at or below 0.03 rad.
  - `"a swing-twist drive deflects per axis"`: k = (100, 200, 400) N·m/rad. A 10 N·m feed-forward about each axis in turn settles at 10/k within 2 % on that axis and below 0.002 rad on the others.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: these cases fail.
- [ ] **Step 3: Implement** the swing-twist rows.
- [ ] **Step 4: Run** TEST `[joints]`.
  Expected: pass.
- [ ] **Step 5: Commit** `Add swing-twist joints with elliptical cones`.

### Task 6: Reference humanoid checks (verification 2 and 3)

**Files:**
- Create: `tests/fixtures/humanoid.h` and `humanoid.cc`. They hold the 20 bodies and 19 joints of Reflex `assets/rigs/humanoid20.json`, copied as data.
  - Capsules become boxes with the capsule's extents, keeping the rig's mass. Inertia comes from the rig's override or from the solid capsule.
- Test: `tests/dynamics_joint_tests.cc`

**Interfaces:**
- Produces `struct Humanoid { std::vector<uint32_t> bodies; std::vector<uint32_t> joints; };`
- Produces `Humanoid addHumanoid(talus::dynamics::World&, const glm::dvec3& pelvis, bool pinPelvis);`

- [ ] **Step 1: Write the failing tests:**
  - `"every humanoid limit holds with its drive at the cap"`: the pelvis is pinned (mass 0) under gravity. Each joint is driven toward a target 1 rad beyond its upper limit on every limited axis, with k = 10⁵, c = 10³ and the rig's `maxTorqueNm` cap. Over 2 s no hinge, twist or cone limit is exceeded by more than 0.03 rad. Then the same toward the lower limits.
  - `"a humanoid-mass chain hangs without creep"`: 20 boxes alternating 40 kg and 1 kg hang in a ball chain from the world, released 0.3 rad from vertical. Over 60 s:
    - every state stays finite and no body exceeds 5 m/s;
    - after 30 s every body's speed is below 0.05 m/s;
    - the lowest body moves less than 0.005 m over the last 10 s.
  - Print the largest anchor gap as information only; the design sets no gap limit for soft point rows.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: build failure, because the fixture is missing.
- [ ] **Step 3: Implement** the fixture.
  - If the limit test fails, **stop and report** the worst joint, its axis and its overshoot. The design says so; do not loosen 0.02 rad.
- [ ] **Step 4: Run** TEST `[joints]`.
  Expected: pass.
- [ ] **Step 5: Commit** `Check joint limits and chains on the reference humanoid`.

### Task 7: Spring joint

**Files:** modify `jointrows.h` and `jointedits.cc`. Test: `tests/dynamics_joint_tests.cc`.

**Interfaces:**
- The Spring branch has three linear rows and three angular rows. They pull frame B toward `spring.target`, which is expressed in A's frame.
- Each block is soft with physical stiffness and damping (Task 4's formula). The linear block is capped by h·maxForce and the angular block by h·maxTorque.
- `forceSaturated` and `torqueSaturated` are set when the cap clamped in the last substep.

- [ ] **Step 1: Write the failing tests:**
  - `"a capped spring holds a light box and lets a heavy one slip"`: a spring from the world with linear k = 5000 N/m, c = 200 and maxForce 50 N. A 3 kg box stays within 0.02 m of its target over 2 s and `forceSaturated` is false. A 10 kg box falls more than 1 m in 2 s and `forceSaturated` is true.
  - `"a spring target moved every step is followed"`: the target moves at 1 m/s along x for 2 s. The 3 kg box lags by less than 0.05 m at the end.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: these cases fail.
- [ ] **Step 3: Implement** the spring rows.
- [ ] **Step 4: Run** TEST `[joints]`.
  Expected: pass.
- [ ] **Step 5: Commit** `Add strength-limited spring joints`.

### Task 8: Strainstep and tier switching

**Files:**
- Modify: `src/talus/dynamics/strainstep/strainstep.cc`, `strainstep.h`
- Modify: `src/talus/dynamics/tiers/tiers.cc` (skip jointed bodies)
- Modify: `src/talus/dynamics/pipeline/pipeline.cc`
- Test: `tests/dynamics_joint_tests.cc`

**Interfaces:**
- `solveStrainstep(..., JointSet* joints)` instantiates the Task 3 templates with `double` and `RigidState`.
  - It warm-starts joints before manifolds, solves them in each pass before `pushManifold`, and relaxes them before `relaxManifold`.
- A joint is solved by the solver that owns its bodies' island, and by no other.
- `StepReport` gains `uint32_t jointRows`, the number of joints solved this step, summed over both solvers.

- [ ] **Step 1: Write the failing tests:**
  - `"a jointed chain touching a lattice keeps its joints"`: a 5-box hinge chain rests one end on a lattice block, from the existing strainstep fixtures, so their island runs in strainstep (`REQUIRE` the island owner). Over 120 steps no anchor gap exceeds 0.005 m, and every step reports `jointRows == 4`.
  - `"a jointed switchable body stays rigid when struck"`: a switchable box jointed to the world is struck so hard that the same unjointed box promotes in the existing tier test. It reports no promotion over 30 steps.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: the cases fail. The chain separates, or `jointRows` is 0.
- [ ] **Step 3: Implement** the strainstep wiring and the tier skip.
- [ ] **Step 4: Run** TEST `[joints]` and TEST `[strainstep]`.
  Expected: pass, with `[strainstep]` unchanged.
- [ ] **Step 5: Commit** `Solve joints in strainstep islands and keep jointed bodies rigid`.

### Task 9: Anchors through compound re-centring and splits

**Files:**
- Modify: `src/talus/dynamics/edits/edits.cc`, where `reframe` and `splitCompound` report frame changes
- Modify: `jointedits.cc` (re-express anchors)
- Test: `tests/dynamics_joint_tests.cc`

**Interfaces:** produces `void rebaseJoints(World&, uint32_t oldBody, std::span<const BodyRebase> pieces);`. `BodyRebase` holds the new body id, its old-to-new transform and the children it now holds.

- [ ] **Step 1: Write the failing tests:**
  - `"a joint keeps its world anchor when a compound re-centres"`: a ball joint on a compound loses one child, which moves its centre of mass. The joint's world anchor, computed before and after, differs by less than 1e-9 m.
  - `"a joint follows the piece holding its anchor when a compound splits"`: the anchor lies inside child 2. After a split from the destruction fixtures, the joint references the body that now holds child 2, and its world anchor is unchanged within 1e-9 m.
- [ ] **Step 2: Run** BUILD, then TEST `[joints]`.
  Expected: the cases fail.
- [ ] **Step 3: Implement** `rebaseJoints` and call it from both edit paths when joints exist.
- [ ] **Step 4: Run** TEST `[joints]` and TEST `[destruction]`.
  Expected: pass, with `[destruction]` unchanged.
- [ ] **Step 5: Commit** `Keep joint anchors through compound re-centring and splits`.

### Task 10: Final determinism, cost and documentation

**Files:**
- Modify: `docs/physics-status.md` and `docs/integration.md` (joints section), `THIRD_PARTY_NOTICES.md` (Box3D MIT notice), `README.md` (feature list), `docs/joints-design.md` (results)
- Test: `tests/dynamics_joint_tests.cc`

- [ ] **Step 1:** Extend the determinism test to a scene with all four joint kinds, limits and drives, plus two pinned humanoids touching a box pile. Run it.
  Expected: identical hashes at widths 1, 4 and 8 × workers 1, 4 and 8.
- [ ] **Step 2:** Add the hidden profile case `[.][jointcost]`: four unpinned humanoids on a floor, drives holding a standing pose (k = 10³ and c = 50 on every axis), 600 steps at 4 substeps. It prints the mean `StepProfile::joints` time, the mean step time, and steps per second for one humanoid alone.
  - Run it on this laptop. Then run it on the lab desktop through the `talus-lab` skill (worker `desktop`, Release).
  Expected on the desktop: one 20-body ragdoll at or below about 0.25 ms per step at its 16 substeps. Report all numbers with machine, compiler and commit.
- [ ] **Step 3:** Run the full `verify` workflow and the eight hidden hash tags.
  Expected: the baseline's cases plus the new ones pass, the same one expected failure remains, and the hashes equal `baseline-hashes.txt`.
- [ ] **Step 4:** Write the docs, with plain usage in `integration.md` and status and measured cost in `physics-status.md`. Commit `Document joints and record their cost`.
- [ ] **Step 5:** Final whole-branch review on the most capable model. Then ask the owner before pushing and opening the public pull request.
