#include "deformedfield.ih"

namespace talus::deformable {

namespace {

constexpr float UNCHANGED = 1.0e-7f;

size_t sampleIndex(const glm::uvec3& resolution, uint32_t x, uint32_t y, uint32_t z) {
  return x + static_cast<size_t>(resolution.x) * (y + static_cast<size_t>(resolution.y) * z);
}

}

talus::field::WriteReport bakeDeformedField(
  const talus::field::SampledSdf& rest,
  const talus::PreparedMesh& mesh,
  std::span<const glm::vec3> position,
  const talus::Pose& pose,
  talus::field::SampledSdf& current,
  bool carveBeyondReach
) {
  if (
    current.resolution != rest.resolution || current.origin != rest.origin
    || current.voxelSize != rest.voxelSize
    || current.distances.size() != rest.distances.size()
  ) {
    throw std::invalid_argument("The current field must share the rest field's lattice");
  }

  if (current.materials.size() != current.distances.size()) {
    throw std::invalid_argument("The current field needs one material per sample");
  }

  if (position.size() != mesh.restPositions.size()) {
    throw std::invalid_argument("A deformed field needs one position per lattice node");
  }

  DeformedProbe probe = deformedProbe(mesh, rest);
  const std::vector<glm::dvec3> placed(position.begin(), position.end());

  placeDeformedProbe(probe, placed);

  const glm::dmat4 model(talus::model(pose));
  const glm::uvec3 resolution = rest.resolution;
  std::vector<float> baked(rest.distances.size());
  glm::uvec3 low(std::numeric_limits<uint32_t>::max());
  glm::uvec3 high(0u);
  bool changed = false;

  for (uint32_t z = 0u; z != resolution.z; ++z) {
    for (uint32_t y = 0u; y != resolution.y; ++y) {
      for (uint32_t x = 0u; x != resolution.x; ++x) {
        const size_t index = sampleIndex(resolution, x, y, z);
        const glm::dvec3 local =
          glm::dvec3(rest.origin) + glm::dvec3(rest.voxelSize) * glm::dvec3(glm::uvec3(x, y, z));
        const glm::dvec3 world(model * glm::dvec4(local, 1.0));
        const DeformedDistance found = probeDeformed(probe, world);
        const float unplaced = carveBeyondReach
                                 ? std::max(rest.distances[index], static_cast<float>(probe.pad))
                                 : rest.distances[index];
        const float value = found.found ? static_cast<float>(found.distance) : unplaced;

        if (std::abs(value - current.distances[index]) > UNCHANGED) {
          baked[index] = value;
          low = glm::min(low, glm::uvec3(x, y, z));
          high = glm::max(high, glm::uvec3(x, y, z));
          changed = true;
        } else {
          baked[index] = current.distances[index];
        }
      }
    }
  }

  if (!changed) {
    return {};
  }

  const talus::field::SampleBox box{low, high - low + glm::uvec3(1u)};
  const size_t count = static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z;
  std::vector<float> distances;
  std::vector<uint8_t> materials;

  distances.reserve(count);
  materials.reserve(count);

  for (uint32_t z = box.minimum.z; z != box.minimum.z + box.extent.z; ++z) {
    for (uint32_t y = box.minimum.y; y != box.minimum.y + box.extent.y; ++y) {
      for (uint32_t x = box.minimum.x; x != box.minimum.x + box.extent.x; ++x) {
        const size_t index = sampleIndex(resolution, x, y, z);

        distances.push_back(baked[index]);
        materials.push_back(current.materials[index]);
      }
    }
  }

  return talus::field::write(current, box, distances, materials);
}

}
