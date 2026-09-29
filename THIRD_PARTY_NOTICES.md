# Third-party notices

talus is MIT licensed (see `LICENSE`). It builds against the projects below,
which CMake fetches at configure time; none of their source is in this
repository. Each is under its own license.

| Project | Version | License | Source |
| --- | --- | --- | --- |
| GLFW | 3.4 | zlib | https://github.com/glfw/glfw |
| GLM | 0af55cc | MIT | https://github.com/g-truc/glm |
| stb | 31c1ad3 | MIT or public domain | https://github.com/nothings/stb |
| tinyobjloader | e60d333 | MIT | https://github.com/tinyobjloader/tinyobjloader |
| Jolt Physics | v5.6.0 | MIT | https://github.com/jrouwe/JoltPhysics |
| EnTT | v4.0.0 | MIT | https://github.com/skypjack/entt |
| Glaze | v8.3.0 | MIT | https://github.com/stephenberry/glaze |
| Eigen | 3.4.0 | MPL-2.0 | https://gitlab.com/libeigen/eigen |
| Catch2 (tests only) | v3.7.1 | BSL-1.0 | https://github.com/catchorg/Catch2 |

Vulkan headers, the loader and `glslc` come from the LunarG Vulkan SDK
(https://vulkan.lunarg.com/), installed separately.

## Assets

`projects/demo/assets/viking_room.obj` and `viking_room.png` are the
[Viking room](https://sketchfab.com/3d-models/viking-room-a49f1b8e4f5c4ecf9e1fe7d81915ad38)
model by [nigelgoh](https://sketchfab.com/nigelgoh), licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), as distributed with
the [Vulkan Tutorial](https://vulkan-tutorial.com/Loading_models). The texture
is unchanged; the `.obj` has been modified.
