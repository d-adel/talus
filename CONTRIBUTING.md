# Contributing

## Issues

Bug reports and proposals are welcome as issues. For a bug, include the GPU,
the driver version, the command that ran and what it printed.

## Pull requests

Open an issue first for anything beyond a small fix, so the change is agreed
before it is written. Keep a pull request to one change.

This repository is published from a private working repository. A pull
request is not merged here; an accepted one is applied there with your
authorship kept, and arrives here with the next export. The pull request is
then closed with a link to that export.

## Build and tests

    cmake --preset dev
    cmake --build --preset dev
    ctest --preset dev

A change passes the whole suite in `build-dev`, including both golden image
tests, with no Vulkan validation-layer messages. A performance claim is
measured in `build-release`.

## Code style

- C++23, `/W4 /WX`, zero warnings
- Formatted with clang-format 20 or newer, using `.clang-format`
- No comments in code, CMake, shaders or C#
- A module links only the tiers its own tier allows; see
  `cmake/TalusModule.cmake`

## License

Contributions are accepted under the repository's MIT license.
