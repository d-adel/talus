# Third party notices

Talus is MIT licensed. CMake uses the following separately licensed dependencies;
their source is not vendored in this repository.

| Project | Pinned source | License |
| --- | --- | --- |
| [GLM](https://github.com/g-truc/glm) | 0af55ccecd98d4e5a8d1fad7de25ba429d60e863 | MIT |
| [Eigen](https://gitlab.com/libeigen/eigen) | 3.4.0 | MPL-2.0 |
| [Catch2](https://github.com/catchorg/Catch2) | v3.7.1, tests only | BSL-1.0 |

No former demo artwork, renderer dependencies or desktop editor assets are
included in the physics-only tree.

## Box3D

The joint row mathematics in `src/talus/dynamics/joints/jointrows.h` is adapted
from [Box3D](https://github.com/erincatto/box3d) by Erin Catto, at commit
5d83df83ab47172c2c745b44315e7538ead02397. It follows Box3D's soft constraint
softness (`b3MakeSoft` in `src/solver.h`) and its joint solvers in
`src/joint.c`, `src/spherical_joint.c`, `src/revolute_joint.c` and
`src/motor_joint.c`.

The planar velocity and pogo joint rows in the same file are adapted from
Box3D's `src/mover_joint.c` and `src/pogo_joint.c`, added in Box3D pull
request 202, at merge commit 336263320469e5e9febfe18697ab562488f7e97a. The
test host in `tests/fixtures/mover.cc` follows that commit's
`samples/dynamic_mover.cpp`.

Box3D is distributed under the following license.

```text
MIT License

Copyright (c) 2026 Erin Catto

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
