#include "tearing.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/resolution/resolution.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace talus::dynamics {

namespace {

using Face = std::array<uint32_t, 4>;
using Copies = std::vector<std::pair<uint32_t, uint32_t>>;

constexpr std::array<std::array<uint32_t, 4>, 6> FACES{{
  {0u, 2u, 4u, 6u},
  {1u, 3u, 5u, 7u},
  {0u, 1u, 4u, 5u},
  {2u, 3u, 6u, 7u},
  {0u, 1u, 2u, 3u},
  {4u, 5u, 6u, 7u}
}};

Face faceOf(const talus::PreparedHex& hex, uint32_t face) {
  Face nodes{};

  for (uint32_t at = 0u; at != 4u; ++at) {
    nodes[at] = hex.nodes[FACES[face][at]];
  }

  std::sort(nodes.begin(), nodes.end());

  return nodes;
}

bool holds(const Face& face, uint32_t node) {
  return std::find(face.begin(), face.end(), node) != face.end();
}

bool pinnedNode(const talus::PreparedMesh& mesh, uint32_t node) {
  return !mesh.anchored.empty() && mesh.anchored[node] != 0u;
}

glm::dvec3 restOf(const talus::PreparedMesh& mesh, uint32_t node) {
  return glm::dvec3(mesh.restPositions[node]);
}

std::vector<uint32_t> cellsAt(const talus::PreparedMesh& mesh, uint32_t node) {
  std::vector<uint32_t> found;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const std::array<uint32_t, 8>& nodes = mesh.hexes[cell].nodes;

    if (std::find(nodes.begin(), nodes.end(), node) != nodes.end()) {
      found.push_back(cell);
    }
  }

  return found;
}

uint32_t sharedNodes(const talus::PreparedHex& a, const talus::PreparedHex& b) {
  uint32_t count = 0u;

  for (const uint32_t node : a.nodes) {
    count += std::find(b.nodes.begin(), b.nodes.end(), node) != b.nodes.end() ? 1u : 0u;
  }

  return count;
}

uint32_t rootOf(std::vector<uint32_t>& parent, uint32_t at) {
  while (parent[at] != at) {
    parent[at] = parent[parent[at]];
    at = parent[at];
  }

  return at;
}

std::vector<uint32_t> groupsOf(
  const talus::PreparedMesh& mesh,
  const std::vector<uint32_t>& fan,
  const std::vector<uint32_t>& sides
) {
  std::vector<uint32_t> parent(fan.size());

  std::iota(parent.begin(), parent.end(), 0u);

  for (uint32_t i = 0u; i != fan.size(); ++i) {
    for (uint32_t j = i + 1u; j != fan.size(); ++j) {
      if (sides[i] == sides[j] && sharedNodes(mesh.hexes[fan[i]], mesh.hexes[fan[j]]) >= 4u) {
        const uint32_t a = rootOf(parent, i);
        const uint32_t b = rootOf(parent, j);

        if (a != b) {
          parent[std::max(a, b)] = std::min(a, b);
        }
      }
    }
  }

  std::map<uint32_t, uint32_t> groupOf;
  std::vector<uint32_t> result(fan.size(), 0u);

  for (uint32_t side = 0u; side != 2u; ++side) {
    for (uint32_t i = 0u; i != fan.size(); ++i) {
      if (sides[i] != side) {
        continue;
      }

      const uint32_t root = rootOf(parent, i);
      const uint32_t next = static_cast<uint32_t>(groupOf.size());

      groupOf.emplace(root, next);
      result[i] = groupOf.at(root);
    }
  }

  return result;
}

size_t reachable(
  const talus::PreparedMesh& mesh,
  const std::vector<uint32_t>& seeds,
  uint32_t pivot,
  size_t limit
) {
  std::map<Face, std::vector<uint32_t>> byFace;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    for (uint32_t face = 0u; face != 6u; ++face) {
      const Face nodes = faceOf(mesh.hexes[cell], face);

      if (!holds(nodes, pivot)) {
        byFace[nodes].push_back(cell);
      }
    }
  }

  std::set<uint32_t> seen(seeds.begin(), seeds.end());
  std::vector<uint32_t> stack(seeds.begin(), seeds.end());

  while (!stack.empty() && seen.size() < limit) {
    const uint32_t cell = stack.back();

    stack.pop_back();

    for (uint32_t face = 0u; face != 6u; ++face) {
      const Face nodes = faceOf(mesh.hexes[cell], face);
      const auto found = byFace.find(nodes);

      if (holds(nodes, pivot) || found == byFace.end()) {
        continue;
      }

      for (const uint32_t other : found->second) {
        if (seen.insert(other).second) {
          stack.push_back(other);
        }
      }
    }
  }

  return seen.size();
}

uint32_t splitNode(LatticeBody& lattice, talus::PreparedMesh& mesh, uint32_t node) {
  const uint32_t copy = static_cast<uint32_t>(mesh.restPositions.size());
  const glm::vec3 rest = mesh.restPositions[node];
  const glm::dvec3 position = lattice.position[node];
  const glm::dvec3 velocity = lattice.velocity[node];
  const glm::dvec3 initial = lattice.initial[node];
  const glm::dvec3 inertial = lattice.inertial[node];
  const glm::dvec3 previous = lattice.previousVelocity[node];
  const uint8_t colour = lattice.colour[node];

  mesh.restPositions.push_back(rest);
  mesh.nodeMass.push_back(0.0f);

  if (!mesh.anchored.empty()) {
    mesh.anchored.push_back(0u);
  }

  if (!mesh.restSurfaceDistance.empty()) {
    const float distance = mesh.restSurfaceDistance[node];

    mesh.restSurfaceDistance.push_back(distance);
  }

  lattice.position.push_back(position);
  lattice.velocity.push_back(velocity);
  lattice.initial.push_back(initial);
  lattice.inertial.push_back(inertial);
  lattice.previousVelocity.push_back(previous);
  lattice.colour.push_back(colour);

  return copy;
}

bool split(
  LatticeBody& lattice,
  talus::PreparedMesh& mesh,
  uint32_t node,
  const std::vector<uint32_t>& fan,
  const std::vector<uint32_t>& group,
  Copies& copies
) {
  const uint32_t count = group.empty() ? 0u : *std::max_element(group.begin(), group.end()) + 1u;

  if (count < 2u || pinnedNode(mesh, node)) {
    return false;
  }

  for (uint32_t piece = 1u; piece != count; ++piece) {
    const uint32_t copy = splitNode(lattice, mesh, node);

    copies.emplace_back(copy, node);

    for (uint32_t at = 0u; at != fan.size(); ++at) {
      if (group[at] != piece) {
        continue;
      }

      for (uint32_t& corner : mesh.hexes[fan[at]].nodes) {
        if (corner == node) {
          corner = copy;
        }
      }
    }
  }

  return true;
}

CellStretch stretchOf(const LatticeBody& lattice, const talus::PreparedMesh& mesh, uint32_t cell) {
  const talus::PreparedHex& hex = mesh.hexes[cell];
  CellStretch best;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const uint32_t base = hex.nodes[corner];
    glm::dmat3 rest(0.0);
    glm::dmat3 now(0.0);

    for (uint32_t edge = 0u; edge != 3u; ++edge) {
      const uint32_t other = hex.nodes[corner ^ (1u << edge)];

      rest[static_cast<glm::length_t>(edge)] = restOf(mesh, other) - restOf(mesh, base);
      now[static_cast<glm::length_t>(edge)] = lattice.position[other] - lattice.position[base];
    }

    if (!(std::abs(glm::determinant(rest)) > 0.0)) {
      continue;
    }

    const glm::dmat3 deformation = now * glm::inverse(rest);
    const glm::dmat3 right = glm::transpose(deformation) * deformation;
    Eigen::Matrix3d tensor;

    for (int row = 0; row != 3; ++row) {
      for (int column = 0; column != 3; ++column) {
        tensor(row, column) = right[column][row];
      }
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(tensor);
    const double stretch = std::sqrt(std::max(eigen.eigenvalues()(2), 0.0));

    if (stretch > best.stretch) {
      const Eigen::Vector3d direction = eigen.eigenvectors().col(2);

      best.stretch = stretch;
      best.direction = glm::normalize(glm::dvec3(direction(0), direction(1), direction(2)));
    }
  }

  return best;
}

bool crackThroughCell(LatticeBody& lattice, talus::PreparedMesh& mesh, uint32_t cell, Copies& copies) {
  const glm::dvec3 across = stretchOf(lattice, mesh, cell).direction;
  const std::array<uint32_t, 8> corners = mesh.hexes[cell].nodes;
  glm::dvec3 centre(0.0);

  for (const uint32_t node : corners) {
    centre += restOf(mesh, node) / 8.0;
  }

  std::vector<std::pair<double, uint32_t>> order;

  for (const uint32_t node : corners) {
    order.emplace_back(std::abs(glm::dot(restOf(mesh, node) - centre, across)), node);
  }

  std::sort(order.begin(), order.end());

  for (const auto& [distance, node] : order) {
    if (pinnedNode(mesh, node)) {
      continue;
    }

    const std::vector<uint32_t> fan = cellsAt(mesh, node);
    std::vector<uint32_t> sides(fan.size(), 0u);
    bool low = false;
    bool high = false;

    for (uint32_t at = 0u; at != fan.size(); ++at) {
      double offset = 0.0;

      for (const uint32_t other : mesh.hexes[fan[at]].nodes) {
        offset += glm::dot(restOf(mesh, other) - restOf(mesh, node), across);
      }

      sides[at] = offset > 0.0 ? 1u : 0u;
      low = low || sides[at] == 0u;
      high = high || sides[at] == 1u;
    }

    if (!low || !high) {
      continue;
    }

    const std::vector<uint32_t> group = groupsOf(mesh, fan, sides);
    const uint32_t count = *std::max_element(group.begin(), group.end()) + 1u;
    bool whole = true;

    for (uint32_t piece = 0u; piece != count && whole; ++piece) {
      std::vector<uint32_t> seeds;

      for (uint32_t at = 0u; at != fan.size(); ++at) {
        if (group[at] == piece) {
          seeds.push_back(fan[at]);
        }
      }

      whole = reachable(mesh, seeds, node, MIN_PIECE_CELLS) >= MIN_PIECE_CELLS;
    }

    if (whole && split(lattice, mesh, node, fan, group, copies)) {
      return true;
    }
  }

  return false;
}

void splitPinched(LatticeBody& lattice, talus::PreparedMesh& mesh, Copies& copies) {
  std::set<uint32_t> near;

  for (const auto& [copy, original] : copies) {
    near.insert(copy);
    near.insert(original);
  }

  std::set<uint32_t> candidates;

  for (const talus::PreparedHex& hex : mesh.hexes) {
    const bool touches = std::any_of(hex.nodes.begin(), hex.nodes.end(), [&](uint32_t node) {
      return near.contains(node);
    });

    if (touches) {
      candidates.insert(hex.nodes.begin(), hex.nodes.end());
    }
  }

  for (const uint32_t node : candidates) {
    const std::vector<uint32_t> fan = cellsAt(mesh, node);

    if (fan.size() < 2u) {
      continue;
    }

    (void)split(lattice, mesh, node, fan, groupsOf(mesh, fan, std::vector<uint32_t>(fan.size(), 0u)), copies);
  }
}

void recomputeMasses(const LatticeBody& lattice, talus::PreparedMesh& mesh) {
  const talus::PhysicalMaterial fallback{};

  std::fill(mesh.nodeMass.begin(), mesh.nodeMass.end(), 0.0f);

  for (const talus::PreparedHex& hex : mesh.hexes) {
    const float density =
      hex.material < lattice.materials.size() ? lattice.materials[hex.material].density : fallback.density;
    const float share = density * hex.volume * 0.125f;

    for (const uint32_t node : hex.nodes) {
      mesh.nodeMass[node] += share;
    }
  }
}

void markCracks(talus::PreparedMesh& mesh, const Copies& copies) {
  glm::vec3 smallest(std::numeric_limits<float>::max());

  for (const glm::vec3& rest : mesh.restPositions) {
    smallest = glm::min(smallest, rest);
  }

  constexpr int32_t OFFSET = 1 << 20;
  const auto keyOf = [](const glm::ivec3& at) {
    return static_cast<uint64_t>(at.x + OFFSET) | static_cast<uint64_t>(at.y + OFFSET) << 21u
           | static_cast<uint64_t>(at.z + OFFSET) << 42u;
  };
  std::vector<glm::ivec3> index(mesh.hexes.size());
  std::unordered_map<uint64_t, uint32_t> cellAt;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    index[cell] = glm::ivec3(glm::round((mesh.restPositions[mesh.hexes[cell].nodes[0]] - smallest) / mesh.voxelSize));
    cellAt.emplace(keyOf(index[cell]), cell);
  }

  std::set<uint32_t> surface(mesh.surfaceNodes.begin(), mesh.surfaceNodes.end());

  mesh.crackFaces.assign(mesh.hexes.size(), 0u);

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      for (uint32_t side = 0u; side != 2u; ++side) {
        glm::ivec3 next = index[cell];

        next[static_cast<glm::length_t>(axis)] += side == 0u ? -1 : 1;

        const auto found = cellAt.find(keyOf(next));

        if (found == cellAt.end()) {
          continue;
        }

        const Face mine = faceOf(mesh.hexes[cell], 2u * axis + side);
        const Face theirs = faceOf(mesh.hexes[found->second], 2u * axis + (1u - side));

        if (mine != theirs) {
          mesh.crackFaces[cell] = static_cast<uint8_t>(mesh.crackFaces[cell] | (1u << (2u * axis + side)));
          surface.insert(mine.begin(), mine.end());
        }
      }
    }
  }

  for (const auto& [copy, original] : copies) {
    if (surface.contains(original)) {
      surface.insert(copy);
    }
  }

  mesh.surfaceNodes.assign(surface.begin(), surface.end());
}

}

CellStretch cellStretch(const LatticeBody& lattice, uint32_t cell) {
  return stretchOf(lattice, *lattice.mesh, cell);
}

std::vector<uint32_t> overstretched(const LatticeBody& lattice) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::vector<uint32_t> torn;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const size_t lane = std::min<size_t>(mesh.hexes[cell].material, lattice.materials.size() - 1u);
    const double limit = 1.0 + static_cast<double>(lattice.materials[lane].tearStrain);

    if (std::isfinite(limit) && stretchOf(lattice, mesh, cell).stretch > limit) {
      torn.push_back(cell);
    }
  }

  return torn;
}

uint32_t tearCells(LatticeBody& lattice, std::span<const uint32_t> cells) {
  if (cells.empty()) {
    return 0u;
  }

  auto mesh = std::make_shared<talus::PreparedMesh>(*lattice.mesh);
  Copies copies;

  for (const uint32_t cell : cells) {
    (void)crackThroughCell(lattice, *mesh, cell, copies);
  }

  if (copies.empty()) {
    return 0u;
  }

  splitPinched(lattice, *mesh, copies);
  recomputeMasses(lattice, *mesh);
  markCracks(*mesh, copies);

  if (lattice.field != nullptr) {
    deformable::rebuildSurface(*mesh, *lattice.field);
  }

  lattice.mesh = mesh;
  rebuildAdjacency(lattice);

  return static_cast<uint32_t>(copies.size());
}

uint32_t latticePieces(const LatticeBody& lattice) {
  return meshPieces(*lattice.mesh);
}

}
