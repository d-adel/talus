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
