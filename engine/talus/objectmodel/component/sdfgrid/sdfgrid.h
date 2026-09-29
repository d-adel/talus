#pragma once

#include <talus/core/config/config.h>
#include <talus/objectmodel/trait/authored/authored.h>

#include <talus/field/analytic/analytic.h>
#include <talus/field/field.h>

#include <glm/glm.hpp>

#include <optional>
#include <span>

namespace talus::objectmodel {

struct SdfGrid {
  glm::vec3 voxelSize{1.0f};
  glm::uvec3 resolution{0u};
  glm::vec4 origin{0.0f};
  std::optional<float> margin{};
  std::optional<float> shadowCullMargin{};
};

[[nodiscard]] float gridMargin(
  const SdfGrid& grid,
  std::span<const talus::field::Primitive> primitives
);

[[nodiscard]] float fittedMargin(
  std::span<const talus::field::Primitive> primitives,
  const glm::uvec3& resolution
);

[[nodiscard]] float shadowCullMarginOf(
  const SdfGrid& grid,
  std::span<const talus::field::Primitive> primitives
);

talus::field::Aabb gridBox(
  std::span<const talus::field::Primitive> primitives,
  float margin = talus::core::FIELD_GRID_MARGIN
);

[[nodiscard]] talus::field::Aabb casterBox(
  std::span<const talus::field::Primitive> primitives,
  const SdfGrid& grid
);

void fitToPrimitives(
  SdfGrid& grid,
  std::span<const talus::field::Primitive> primitives
);

template<>
inline constexpr bool authored<SdfGrid> = true;

}
