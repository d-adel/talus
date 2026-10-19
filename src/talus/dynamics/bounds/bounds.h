#pragma once

#include <glm/glm.hpp>

namespace talus::dynamics {

struct Bounds {
  glm::dvec3 low{0.0};
  glm::dvec3 high{0.0};
};

[[nodiscard]] inline bool overlaps(const Bounds& a, const Bounds& b) {
  return a.low.x <= b.high.x && b.low.x <= a.high.x && a.low.y <= b.high.y
         && b.low.y <= a.high.y && a.low.z <= b.high.z && b.low.z <= a.high.z;
}

[[nodiscard]] inline bool contains(const Bounds& outer, const Bounds& inner) {
  return outer.low.x <= inner.low.x && outer.low.y <= inner.low.y
         && outer.low.z <= inner.low.z && inner.high.x <= outer.high.x
         && inner.high.y <= outer.high.y && inner.high.z <= outer.high.z;
}

[[nodiscard]] inline Bounds unite(const Bounds& a, const Bounds& b) {
  return {glm::min(a.low, b.low), glm::max(a.high, b.high)};
}

[[nodiscard]] inline Bounds grown(const Bounds& box, double by) {
  return {box.low - glm::dvec3(by), box.high + glm::dvec3(by)};
}

[[nodiscard]] inline double area(const Bounds& box) {
  const glm::dvec3 size = box.high - box.low;

  return 2.0 * (size.x * size.y + size.y * size.z + size.z * size.x);
}

}
