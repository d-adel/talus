#pragma once

#include <talus/dynamics/manifold/manifold.h>

namespace talus::dynamics {

[[nodiscard]] Manifold collideBoxHulls(
  const BoxPose& a,
  const BoxPose& b,
  double speculative
);

}
