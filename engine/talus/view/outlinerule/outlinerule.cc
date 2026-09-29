#include "outlinerule.ih"

namespace talus::view {

namespace {

constexpr float FAR = 1.0f;

constexpr float BIAS = 1e-4f;

bool chosen(std::span<const float> selectionDepth, size_t at) {
  return selectionDepth[at] < FAR;
}

}

Outlined outlined(
  std::span<const float> selectionDepth,
  std::span<const float> sceneDepth,
  uint32_t width,
  uint32_t height,
  uint32_t x,
  uint32_t y,
  uint32_t radius
) {
  if (x >= width || y >= height) {
    return Outlined::None;
  }

  if (chosen(selectionDepth, static_cast<size_t>(y) * width + x)) {
    return Outlined::None;
  }

  const int64_t reach = static_cast<int64_t>(radius);

  bool found = false;
  bool visible = false;

  for (int64_t dy = -reach; dy <= reach; ++dy) {
    for (int64_t dx = -reach; dx <= reach; ++dx) {
      const int64_t px = std::clamp<int64_t>(
        static_cast<int64_t>(x) + dx,
        0,
        static_cast<int64_t>(width) - 1
      );
      const int64_t py = std::clamp<int64_t>(
        static_cast<int64_t>(y) + dy,
        0,
        static_cast<int64_t>(height) - 1
      );

      const size_t probe =
        static_cast<size_t>(py) * width + static_cast<size_t>(px);

      if (!chosen(selectionDepth, probe)) {
        continue;
      }

      found = true;

      if (sceneDepth[probe] >= selectionDepth[probe] - BIAS) {
        visible = true;
      }
    }
  }

  if (!found) {
    return Outlined::None;
  }

  return visible ? Outlined::Visible : Outlined::Occluded;
}

}
