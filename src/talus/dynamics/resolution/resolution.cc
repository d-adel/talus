#include "resolution.h"

#include <talus/dynamics/strainstep/element.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_set>
#include <vector>

namespace talus::dynamics {

namespace {

glm::uvec3 cellIndex(const glm::vec3& corner, const glm::vec3& origin, const glm::vec3& voxel) {
  return glm::uvec3(glm::max(glm::round((corner - origin) / voxel), glm::vec3(0.0f)));
}

uint64_t cellKey(const glm::uvec3& cell) {
  return static_cast<uint64_t>(cell.x) | static_cast<uint64_t>(cell.y) << 21u
         | static_cast<uint64_t>(cell.z) << 42u;
}

glm::uvec3 lowestCell(const talus::PreparedMesh& mesh, const field::SampledSdf& field) {
  glm::uvec3 lowest(std::numeric_limits<uint32_t>::max());

  for (const talus::PreparedHex& hex : mesh.hexes) {
    lowest = glm::min(lowest, cellIndex(mesh.restPositions[hex.nodes[0]], field.origin, field.voxelSize));
  }

  return lowest;
}

const talus::PhysicalMaterial& laneOf(
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t lane
) {
  return materials[std::min<size_t>(lane, materials.size() - 1u)];
}

uint32_t rootOf(std::vector<uint32_t>& parent, uint32_t at) {
  while (parent[at] != at) {
    parent[at] = parent[parent[at]];
    at = parent[at];
  }

  return at;
}

constexpr float GROWTH = 1.0e-3f;

bool holds(
  const deformable::SupportVolume& support,
  const glm::mat4& localToWorld,
  const glm::vec3& rest
) {
  const glm::vec3 world = glm::vec3(localToWorld * glm::vec4(rest, 1.0f));
  const glm::vec3 local = glm::vec3(support.worldToSupport * glm::vec4(world, 1.0f));

  return glm::all(glm::lessThanEqual(glm::abs(local), support.halfExtents));
}

std::vector<uint32_t> anchorsOf(
  const talus::PreparedMesh& mesh,
  const glm::mat4& localToWorld,
  std::span<const deformable::SupportVolume> supports
) {
  std::vector<uint32_t> counts(supports.size(), 0u);

  for (const glm::vec3& rest : mesh.restPositions) {
    for (size_t support = 0u; support != supports.size(); ++support) {
      counts[support] += holds(supports[support], localToWorld, rest) ? 1u : 0u;
    }
  }

  return counts;
}

bool keepsSupports(const std::vector<uint32_t>& authored, const std::vector<uint32_t>& candidate) {
  for (size_t support = 0u; support != authored.size(); ++support) {
    if (authored[support] != 0u && candidate[support] == 0u) {
      return false;
    }
  }

  return true;
}

}

field::SampledSdf grownField(const field::SampledSdf& field) {
  field::SampledSdf grown;

  grown.origin = field.origin;
  grown.voxelSize = field.voxelSize;
  grown.resolution = field.resolution;
  grown.distances = field.distances;
  grown.materials = field.materials;

  const float growth = GROWTH * std::min({field.voxelSize.x, field.voxelSize.y, field.voxelSize.z});

  for (float& distance : grown.distances) {
    distance -= growth;
  }

  return grown;
}

field::SampledSdf coarserField(
  const field::SampledSdf& field,
  uint32_t factor,
  const glm::uvec3& phase
) {
  field::SampledSdf coarse;

  coarse.voxelSize = field.voxelSize * static_cast<float>(factor);
  coarse.origin = field.origin + glm::vec3(phase) * field.voxelSize;

  for (glm::length_t axis = 0; axis != 3; ++axis) {
    const uint32_t last = field.resolution[axis] - 1u;
    const uint32_t span = last - std::min(phase[axis], last);

    coarse.resolution[axis] = std::max(2u, (span + factor - 1u) / factor + 1u);
  }

  const size_t count =
    static_cast<size_t>(coarse.resolution.x) * coarse.resolution.y * coarse.resolution.z;

  coarse.distances.resize(count);
  coarse.materials.resize(count, 0u);

  for (uint32_t z = 0u; z != coarse.resolution.z; ++z) {
    for (uint32_t y = 0u; y != coarse.resolution.y; ++y) {
      for (uint32_t x = 0u; x != coarse.resolution.x; ++x) {
        const glm::uvec3 fine = phase + factor * glm::uvec3(x, y, z);
        const glm::uvec3 held = glm::min(fine, field.resolution - glm::uvec3(1u));
        const size_t from =
          held.x + static_cast<size_t>(field.resolution.x) * (held.y + static_cast<size_t>(field.resolution.y) * held.z);
        const size_t to =
          x + static_cast<size_t>(coarse.resolution.x) * (y + static_cast<size_t>(coarse.resolution.y) * z);

        coarse.distances[to] =
          field.distances[from] + glm::length(glm::vec3(fine - held) * field.voxelSize);

        if (from < field.materials.size()) {
          coarse.materials[to] = field.materials[from];
        }
      }
    }
  }

  return coarse;
}

double shapeMismatch(
  const talus::PreparedMesh& authored,
  const talus::PreparedMesh& candidate,
  const field::SampledSdf& field,
  uint32_t factor,
  const glm::uvec3& phase
) {
  std::unordered_set<uint64_t> active;
  std::unordered_set<uint64_t> covered;
  const glm::vec3 coarseOrigin = field.origin + glm::vec3(phase) * field.voxelSize;
  const glm::vec3 coarseVoxel = field.voxelSize * static_cast<float>(factor);

  for (const talus::PreparedHex& hex : authored.hexes) {
    active.insert(cellKey(cellIndex(authored.restPositions[hex.nodes[0]], field.origin, field.voxelSize)));
  }

  for (const talus::PreparedHex& hex : candidate.hexes) {
    const glm::uvec3 coarse = cellIndex(candidate.restPositions[hex.nodes[0]], coarseOrigin, coarseVoxel);

    for (uint32_t k = 0u; k != factor; ++k) {
      for (uint32_t j = 0u; j != factor; ++j) {
        for (uint32_t i = 0u; i != factor; ++i) {
          covered.insert(cellKey(phase + factor * coarse + glm::uvec3(i, j, k)));
        }
      }
    }
  }

  size_t odd = 0u;

  for (const uint64_t key : active) {
    odd += covered.contains(key) ? 0u : 1u;
  }

  for (const uint64_t key : covered) {
    odd += active.contains(key) ? 0u : 1u;
  }

  return static_cast<double>(odd) / static_cast<double>(std::max<size_t>(1u, active.size()));
}

uint32_t meshPieces(const talus::PreparedMesh& mesh) {
  std::vector<uint32_t> parent(mesh.hexes.size());
  std::vector<uint32_t> owner(mesh.restPositions.size(), std::numeric_limits<uint32_t>::max());

  std::iota(parent.begin(), parent.end(), 0u);

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    for (const uint32_t node : mesh.hexes[cell].nodes) {
      if (owner[node] == std::numeric_limits<uint32_t>::max()) {
        owner[node] = cell;
        continue;
      }

      const uint32_t a = rootOf(parent, owner[node]);
      const uint32_t b = rootOf(parent, cell);

      if (a != b) {
        parent[std::max(a, b)] = std::min(a, b);
      }
    }
  }

  uint32_t pieces = 0u;

  for (uint32_t cell = 0u; cell != parent.size(); ++cell) {
    pieces += rootOf(parent, cell) == cell ? 1u : 0u;
  }

  return pieces;
}

ResolvedMesh resolveMesh(
  const field::SampledSdf& field,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::mat4& localToWorld,
  std::span<const deformable::SupportVolume> supports,
  const IntegrationParameters& parameters
) {
  const field::SampledSdf grown = grownField(field);
  ResolvedMesh resolved{
    deformable::buildMesh(grown, materials, localToWorld, supports, talus::VolumeElementType::Hex),
    1u
  };

  if (!resolved.mesh || materials.empty()) {
    return resolved;
  }

  const std::shared_ptr<const talus::PreparedMesh> authoredMesh = resolved.mesh;
  const talus::PreparedMesh& authored = *authoredMesh;
  bool keeps = false;
  double speed = 0.0;

  for (const talus::PreparedHex& hex : authored.hexes) {
    const talus::PhysicalMaterial& material = laneOf(materials, hex.material);

    keeps = keeps || std::isfinite(material.tearStrain) || std::isfinite(material.yieldStress);
    speed = std::max(speed, cellFrequency(material, 1.0));
  }

  if (keeps) {
    return resolved;
  }

  glm::vec3 low(std::numeric_limits<float>::max());
  glm::vec3 high(std::numeric_limits<float>::lowest());

  for (const glm::vec3& rest : authored.restPositions) {
    low = glm::min(low, rest);
    high = glm::max(high, rest);
  }

  const double cell = std::min({field.voxelSize.x, field.voxelSize.y, field.voxelSize.z});
  const glm::vec3 extent = high - low;
  const double span = std::max({extent.x, extent.y, extent.z});
  const double reach = RESOLVED * static_cast<double>(std::max(parameters.softstepSubsteps, 1u)) / parameters.dt;
  const glm::uvec3 lowest = lowestCell(authored, field);
  const uint32_t pieces = meshPieces(authored);
  const std::vector<uint32_t> anchored = anchorsOf(authored, localToWorld, supports);

  for (uint32_t factor = 2u;
       speed / (static_cast<double>(resolved.coarsening) * cell) > reach
       && static_cast<double>(factor) * cell <= span * (1.0 + 1.0e-5);
       ++factor) {
    const glm::uvec3 phase = lowest % factor;
    const field::SampledSdf coarse = coarserField(grown, factor, phase);
    const std::shared_ptr<const talus::PreparedMesh> candidate = deformable::buildMesh(
      coarse,
      materials,
      localToWorld,
      supports,
      talus::VolumeElementType::Hex
    );

    if (candidate && shapeMismatch(authored, *candidate, field, factor, phase) <= SHAPE_TOLERANCE
        && meshPieces(*candidate) == pieces
        && keepsSupports(anchored, anchorsOf(*candidate, localToWorld, supports))) {
      resolved = ResolvedMesh{candidate, factor};
    }
  }

  return resolved;
}

}
