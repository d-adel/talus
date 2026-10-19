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
