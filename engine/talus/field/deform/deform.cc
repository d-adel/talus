#include "deform.ih"

namespace talus::field {

namespace {

constexpr uint32_t INTERSECTION = 2u;

uint32_t latticeIndex(const SampledSdf& field, const glm::uvec3& at) {
  return at.x + field.resolution.x * (at.y + field.resolution.y * at.z);
}

}

SampleBox affectedBox(
  const SampledSdf& field,
  const Primitive& primitive,
  uint32_t marginVoxels
) {
  if (glm::any(glm::lessThan(field.resolution, glm::uvec3(1u)))) {
    return {};
  }

  if (primitive.shapeConfig.z == INTERSECTION) {
    return {glm::uvec3(0u), field.resolution};
  }

  const std::optional<Aabb> tight = primitiveBox(primitive);

  const bool usable =
    tight.has_value()
    && glm::all(glm::greaterThan(tight->maximum, tight->minimum));

  if (!usable && primitive.bounds.w <= 0.0f) {
    return {glm::uvec3(0u), field.resolution};
  }

  const glm::vec3 low =
    usable ? tight->minimum
           : glm::vec3(primitive.bounds) - glm::vec3(primitive.bounds.w);

  const glm::vec3 high =
    usable ? tight->maximum
           : glm::vec3(primitive.bounds) + glm::vec3(primitive.bounds.w);

  const glm::vec3 pad(static_cast<float>(marginVoxels));

  const glm::vec3 lowest = (low - field.origin) / field.voxelSize - pad;
  const glm::vec3 highest = (high - field.origin) / field.voxelSize + pad;

  const glm::vec3 last = glm::vec3(field.resolution - glm::uvec3(1u));

  if (
    glm::any(glm::greaterThan(lowest, last))
    || glm::any(glm::lessThan(highest, glm::vec3(0.0f)))
  ) {
    return {};
  }

  const glm::uvec3 first =
    glm::uvec3(glm::max(glm::floor(lowest), glm::vec3(0.0f)));

  const glm::uvec3 beyond =
    glm::uvec3(glm::min(glm::ceil(highest), last)) + glm::uvec3(1u);

  return {first, beyond - first};
}

WriteReport deform(SampledSdf& field, const Primitive& primitive) {
  const SampleBox box = affectedBox(field, primitive, DEFORM_BAND_VOXELS);

  const size_t count =
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z;

  if (count == 0u) {
    return {box, glm::uvec3(0u), glm::uvec3(0u)};
  }

  std::vector<float> distances(count);
  std::vector<uint8_t> materials(count);

  size_t at = 0;

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const glm::uvec3 lattice = box.minimum + glm::uvec3(x, y, z);
        const uint32_t index = latticeIndex(field, lattice);

        const glm::vec3 point =
          field.origin + field.voxelSize * glm::vec3(lattice);

        const AnalyticSample was{
          field.distances[index],
          field.materials[index]
        };

        const AnalyticSample now = combine(was, primitive, point);

        distances[at] = now.distance;
        materials[at] = static_cast<uint8_t>(now.material);

        ++at;
      }
    }
  }

  return write(field, box, distances, materials);
}

DeformReport deformAndRepair(
  SampledSdf& field,
  const Primitive& primitive,
  uint32_t sweeps
) {
  const SampleBox box = affectedBox(field, primitive, DEFORM_BAND_VOXELS);

  const size_t count =
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z;

  if (count == 0u) {
    return {{box, glm::uvec3(0u), glm::uvec3(0u)}, 0.0f};
  }

  std::vector<float> distances(count);
  std::vector<uint8_t> materials(count);
  std::vector<uint8_t> damaged(count, 0u);

  size_t at = 0;

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const glm::uvec3 lattice = box.minimum + glm::uvec3(x, y, z);
        const uint32_t index = latticeIndex(field, lattice);

        const glm::vec3 point =
          field.origin + field.voxelSize * glm::vec3(lattice);

        const AnalyticSample was{
          field.distances[index],
          field.materials[index]
        };

        const AnalyticSample now = combine(was, primitive, point);

        distances[at] = now.distance;
        materials[at] = static_cast<uint8_t>(now.material);
        damaged[at] = now.distance == was.distance ? 0u : 1u;

        ++at;
      }
    }
  }

  const float settled = redistance(field, box, damaged, distances, sweeps);

  uint32_t removed = 0u;

  at = 0;

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const glm::uvec3 lattice = box.minimum + glm::uvec3(x, y, z);
        const uint32_t index = latticeIndex(field, lattice);

        if (field.distances[index] < 0.0f && distances[at] >= 0.0f) {
          ++removed;
        }

        ++at;
      }
    }
  }

  return {write(field, box, distances, materials), settled, removed};
}

}
