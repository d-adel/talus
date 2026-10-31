# Contributing

Talus is a physics library. Keep rendering, game logic, engine scene graphs,
network sessions and editor UI in host integrations, not the physics core.

Develop the library and review pull requests in this public repository. Discuss
substantial changes before implementation. Use C++23 and the repository's
clang-format configuration. Keep compiler warnings and relevant monitors active.

Preserve numerical behavior during structural refactors. Keep default stepping
independent of wall-clock time, and do not introduce fast-math or contraction
changes without explicit determinism checks. Test material behavior and useful
interactions, not just agreement between two implementations.

Run the physics suite, the headless example and a separate consumer build for
integration or packaging changes. Report existing failures honestly. Do not fit
tolerances merely to obtain a pass. Record platform, compiler and workload when
making performance claims. See the Known limits section of the [README](README.md#known-limits).

Contributions are accepted under the MIT license.
