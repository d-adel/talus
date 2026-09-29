#pragma once

#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>

#include <cstdint>
#include <optional>

namespace talus::objectmodel {

struct SdfDeformOutcome {
  talus::field::WriteReport write;
  uint32_t cellsRemoved = 0u;
  float volumeRemoved = 0.0f;
};

[[nodiscard]] std::optional<SdfDeformOutcome> deformSampledSdf(
  World& world,
  Entity entity,
  const talus::field::Primitive& localPrimitive
);

}
