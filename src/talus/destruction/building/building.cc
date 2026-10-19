#include "building.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace talus::destruction {

namespace {

bool finite(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool positive(const glm::dvec3& value) {
  return value.x > 0.0 && value.y > 0.0 && value.z > 0.0;
}

bool notNegative(const glm::dvec3& value) {
  return value.x >= 0.0 && value.y >= 0.0 && value.z >= 0.0;
}

}

void validate(const Building& building, std::span<const StructuralMaterial> materials) {
  for (const StructuralMaterial& material : materials) {
    validate(material);
  }

  for (size_t index = 0u; index != building.members.size(); ++index) {
    const Member& member = building.members[index];

    if (!finite(member.centre) || !finite(member.halfExtents) || !positive(member.halfExtents)) {
      throw std::invalid_argument(
        "Building member " + std::to_string(index) + " needs a finite centre and positive half extents"
      );
    }

    if (member.material >= materials.size()) {
      throw std::out_of_range(
        "Building member " + std::to_string(index) + " names a material the building was not given"
      );
    }
  }

  for (size_t index = 0u; index != building.openings.size(); ++index) {
    const Opening& opening = building.openings[index];

    if (!finite(opening.centre) || !finite(opening.halfExtents) || !notNegative(opening.halfExtents)) {
      throw std::invalid_argument(
        "Building opening " + std::to_string(index) + " needs a finite centre and half extents of at least zero"
      );
    }
  }
}

}
