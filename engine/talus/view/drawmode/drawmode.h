#pragma once

namespace talus::view {

enum class DrawMode {
  Nothing,
  Mesh,
  Sdf,
  Both
};

[[nodiscard]] constexpr bool drawsMeshes(DrawMode mode) noexcept {
  return mode == DrawMode::Mesh || mode == DrawMode::Both;
}

[[nodiscard]] constexpr bool drawsSdf(DrawMode mode) noexcept {
  return mode == DrawMode::Sdf || mode == DrawMode::Both;
}

[[nodiscard]] constexpr DrawMode nextDrawMode(DrawMode current) noexcept {
  switch (current) {
    case DrawMode::Nothing:
      return DrawMode::Mesh;
    case DrawMode::Mesh:
      return DrawMode::Sdf;
    case DrawMode::Sdf:
      return DrawMode::Both;
    case DrawMode::Both:
      return DrawMode::Mesh;
  }

  return DrawMode::Both;
}

[[nodiscard]] constexpr const char* drawModeName(DrawMode mode) noexcept {
  switch (mode) {
    case DrawMode::Nothing:
      return "none ";
    case DrawMode::Mesh:
      return "mesh ";
    case DrawMode::Sdf:
      return "field";
    case DrawMode::Both:
      return "both ";
  }

  return "both ";
}

}
