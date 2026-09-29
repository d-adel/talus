# talus

A game engine for deformable worlds, in C++23 on Vulkan and Jolt.

Objects are signed distance fields sampled onto a per-object grid rather than
triangle meshes. The renderer sphere-traces the grid, physics collides against
the same grid, and a body whose material can yield is stepped by finite
elements when an impact would yield it. Triangle meshes are supported on their
own pipeline.

## Status

Work in progress, built by one person. The editor is a separate project and is
not public. The demo project has one world, `main`: rigid field bodies, two
static bodies and a mesh. Deformation runs in the tests rather than in the
demo.

## Features

### Fields

- Analytic SDFs built from primitives and CSG operations
- Sampled fields baked on the GPU onto a grid in the object's own space, and
  checked against the CPU bake
- Per-brick value ranges and Lipschitz bounds, reduced by a compute shader
- Objects with the same shape share one sampled field

### Rendering

- Vulkan 1.4: dynamic rendering, synchronization2, frames in flight
- Sphere tracing of the analytic or the sampled form
- OBJ meshes with textures, depth and diffuse lighting
- One material table for fields and meshes
- Shadow casters binned across the light, so the shadow cost stays flat as
  objects are added
- Golden image tests

### Physics

- Jolt at a fixed 60 Hz step
- A Jolt shape backed by the field: contacts seeded from surface bricks, shape
  casts against the field, and a convex hull for convex fields
- Mass, centre of mass and inertia integrated from the field
- Explicit finite elements for yielding materials: a body stays rigid until an
  impact would yield it, is stepped by 8-point corotated J2 hexahedra at the
  material's stable step inside an impact window, and returns to Jolt with its
  linear and angular momentum kept
- Energy and momentum monitors on every impact window and every transition

### System

- C++23, `/W4 /WX`, zero warnings
- Twenty-three libraries, each depending only on the ones below it
- EnTT object model; projects, worlds and materials are JSON on disk
- Every target declares a tier (Runtime, Developer or Editor), and configure
  fails if one links above its tier
- Game code is C# in the project, loaded over a C ABI

## Layout

| | |
| --- | --- |
| `engine/` | the `talus::` libraries, the C ABI, the player, the packager |
| `projects/demo/` | the demo project: one world, materials, a mesh, C# scripts |
| `shaders/` | GLSL, compiled by `glslc` during the build |
| `tests/` | Catch2 unit tests and the golden image tests |

The libraries, bottom to top:

| | |
| --- | --- |
| `talus::core` | configuration, events, telemetry |
| `talus::field` | primitives, CSG, baking, ray casting |
| `talus::platform` | window and input, currently GLFW |
| `talus::imagecompare` | image load, save and compare (Developer) |
| `talus::gpu` | Vulkan devices, swapchains, buffers, images, pipelines, descriptors |
| `talus::objectmodel` | what exists in a world and where it is |
| `talus::serialize` | the on-disk format for projects, worlds and materials |
| `talus::deformable` | finite element materials, meshes, contact and impact windows |
| `talus::physics` | Jolt and the field shape |
| `talus::systems` | input state and the ordered systems a frame runs |
| `talus::script` | the C ABI game code is written against, and its .NET host |
| `talus::query` | ray casts, overlaps and lookups on a world |
| `talus::view` | cameras and views; a view names the target it draws |
| `talus::renderer` | sphere tracing, mesh drawing, volume residency, shadows |
| `talus::debugdraw` | lines and shapes drawn over a frame |
| `talus::assets` | meshes, textures and materials, loaded once and shared |
| `talus::runtime` | play mode: instantiation, bodies, deformation |
| `talus::engine` | one frame of all of the above |
| `talus::undo`, `talus::gizmos`, `talus::package` | edit history, handles, packaging (Editor) |
| `talus_c` | the C ABI a front end loads (Editor) |

`talus::gpu` does not know what a field is, and `talus::objectmodel` does not
know how anything is drawn. `talus::engine` is the only place they meet; the
player and the editor drive the same one.

## Building

Requirements:

- Windows and a Vulkan 1.4 GPU. The goldens were recorded on an RTX 4060.
- The [LunarG Vulkan SDK](https://vulkan.lunarg.com/), with `VULKAN_SDK` set
- CMake 3.28 or newer, and Ninja
- Visual Studio Build Tools 2022 (MSVC)
- The [.NET 10 SDK](https://dotnet.microsoft.com/en-us/download), for the C#
  scripts

Every other dependency is fetched by CMake.

```sh
git clone https://github.com/d-adel/talus.git
cd talus
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

Run these from a Developer Command Prompt for VS so `cl.exe` and `ninja` are
on `PATH`. `dev` is Debug with the validation layers on; `release` is the
preset for performance numbers.

## Running

```sh
build-dev/TalusPlayer --project projects/demo
```

Left mouse or F shoots a ball, R restarts the world.

| | |
| --- | --- |
| `--project DIR` | project to open, default `projects/demo` |
| `--world NAME` | world to load, default `main` |
| `--frames N` | run N frames, then exit |
| `--fixed-step SECONDS` | step at a fixed timestep |
| `--analytic` | trace the analytic field instead of the sampled one |
| `--screenshot PATH` | render one frame to a file and exit |
| `--golden PATH` | render one frame and compare it against a reference |
| `--capture DIR` | write every frame, at a fixed timestep |
| `--verify-bake` | check the GPU bake against the CPU bake |
| `-- ARGS` | pass the rest to the scripts |

## Why I am building it

I am building talus because I want to make games with worlds that can respond to the player in many different ways and support new types of interaction. I also like the idea that someone much more creative than me could eventually use the same systems to make something I would never have thought of.

Building the engine myself gives me control over how its systems fit together and lets me shape the technology around those kinds of games. A big part of the project is also just learning  (which I've already done an unreasonable amount of) from this thing, and I am nowhere near done.

## How it is built

I set the goals, decide the architecture and review the work. Most of the code,
tests and measurements are written by Claude, Anthropic's model, working as a
coding agent under [doctrine](https://github.com/d-adel/doctrine), the process
I built for it: each change is a task with a frozen design and acceptance
criteria, each claim rests on a check that ran, and each decision records who
made it.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT, see [LICENSE](LICENSE). Third-party code and assets are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
