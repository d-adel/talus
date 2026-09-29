#pragma once

#include <cstdint>
#include <span>

namespace talus::view {

enum class Outlined {
  None,
  Visible,
  Occluded
};

[[nodiscard]] Outlined outlined(
  std::span<const float> selectionDepth,
  std::span<const float> sceneDepth,
  uint32_t width,
  uint32_t height,
  uint32_t x,
  uint32_t y,
  uint32_t radius
);

}
