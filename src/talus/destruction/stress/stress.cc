#include "stress.h"

#include "factor.h"

#include <talus/dynamics/scheduler/stopwatch.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace talus::destruction {

struct Components {
  std::vector<uint32_t> root;
  std::vector<uint8_t> anchored;
  std::vector<uint8_t> active;
  std::vector<int32_t> piece;
  std::vector<MatrixBlock> rigidMass;
};

struct StressPattern {
  Components components;
  BlockPattern blocks;
  std::vector<int32_t> position;
  std::vector<uint8_t> broken;
  std::vector<uint8_t> alive;
  std::vector<uint32_t> chunkAt;
  std::vector<int64_t> bondAt;
  std::vector<uint8_t> bondFlipped;
};

struct StressFactor {
  std::shared_ptr<const StressPattern> pattern;
  uint64_t revision = 0u;
  std::vector<double> values;
  std::vector<glm::dvec3> forceScales;
  std::vector<double> momentScales;
  std::vector<glm::dvec3> offsets;
};

namespace {

constexpr double crackedWeight = 1.0e-2;
constexpr double floorFactor = 0x1.0p-21;
constexpr double wearTolerance = 1.0e-6;
constexpr uint32_t noChunk = 0xFFFFFFFFu;
constexpr uint32_t noBond = 0xFFFFFFFFu;
constexpr double pastLimit = 1.0e9;
constexpr double torqueLever = 0.5;

using Block = MatrixBlock;
using Column = VectorBlock;

struct Six {
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
};

struct Scales {
  std::vector<glm::dvec3> force;
  std::vector<double> moment;
};

struct Pieces {
  std::vector<uint8_t> active;
  std::vector<Column> load;
};

int normalAxis(const glm::dvec3& normal) {
  int axis = 0;

  for (int candidate = 1; candidate != 3; ++candidate) {
    if (std::abs(normal[candidate]) > std::abs(normal[axis])) {
      axis = candidate;
    }
  }

  return axis;
}

Scales scalesOf(const ChunkedBuilding& building, const StructureState& structure, const StressState& stress) {
  const size_t bonds = building.bonds.size();
  Scales scales;

  scales.force.assign(bonds, glm::dvec3(0.0));
  scales.moment.assign(bonds, 0.0);

  for (size_t index = 0u; index != bonds; ++index) {
    const double health = structure.health[index];
    const double moment = stress.momentShare[index] * health * stress.length;

    scales.force[index] = glm::dvec3(stress.forceShare[index] * health);
    scales.moment[index] = moment;

    if (structure.cracked[index] != 0u) {
      const int axis = normalAxis(building.bonds[index].normal);

      scales.force[index][(axis + 1) % 3] *= crackedWeight;
      scales.force[index][(axis + 2) % 3] *= crackedWeight;
      scales.moment[index] = moment * crackedWeight;
    }
  }

  return scales;
}

double excess(double stress, double elastic, double fatal) {
  if (stress >= fatal) {
    return 1.0;
  }

  if (stress <= elastic) {
    return 0.0;
  }

  return (stress - elastic) / (fatal - elastic);
}

double shearStrength(const Bond& bond, const ModeLimits& limits, double pressure) {
  const double cohesion = limits.shear / bond.area;

  if (bond.shearRule == ShearRule::Fixed) {
    return cohesion;
  }

  if (pressure >= 0.0) {
    return cohesion + bond.friction * pressure;
  }

  return cohesion * std::max(0.0, 1.0 + pressure / (limits.tension / bond.area));
}

bool sameLoads(const std::vector<ChunkLoad>& kept, std::span<const ChunkLoad> loads) {
  return std::equal(kept.begin(), kept.end(), loads.begin(), loads.end(), [](const ChunkLoad& a, const ChunkLoad& b) {
    return a.chunk == b.chunk && a.force == b.force && a.torque == b.torque;
  });
}

struct Damage {
  std::vector<uint32_t> broken;
  std::vector<uint32_t> cracked;
  std::vector<uint32_t> eroding;
  std::vector<double> erosion;
  double utilization = 0.0;
};

double shareOf(double load, double limit) {
  return limit > 0.0 ? load / limit : (load > 0.0 ? pastLimit : 0.0);
}

double utilizationOf(const Bond& bond, const BondStress& stress, bool cracked) {
  if (!cracked) {
    return std::max({shareOf(stress.compression, bond.elastic.compression / bond.area),
                     shareOf(stress.tension, bond.elastic.tension / bond.area),
                     shareOf(stress.shear, shearStrength(bond, bond.elastic, stress.pressure))});
  }

  if (!(stress.pressure > 0.0)) {
    return pastLimit;
  }

  return std::max({shareOf(stress.shear, bond.crackFriction * stress.pressure), stress.rocking / stress.pressure,
                   shareOf(stress.compression, bond.elastic.compression / bond.area)});
}

double stiffestShare(const StructureState& structure, const StressState& stress) {
  double stiffest = 0.0;

  for (size_t index = 0u; index != structure.broken.size(); ++index) {
    if (structure.broken[index] == 0u) {
      stiffest = std::max(stiffest, std::max(stress.forceShare[index], stress.momentShare[index]));
    }
  }

  return stiffest;
}

double floorOf(const StructureState& structure, const StressState& stress, double stiffest, size_t index) {
  const double crack = structure.cracked[index] != 0u ? crackedWeight : 1.0;

  return floorFactor * stiffest / (std::min(stress.forceShare[index], stress.momentShare[index]) * crack);
}

Damage assessDamage(const ChunkedBuilding& building, const StructureState& structure, const StressState& stress) {
  Damage damage;
  const double stiffest = stiffestShare(structure, stress);

  for (uint32_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      continue;
    }

    const Bond& bond = building.bonds[index];
    const BondStress state = bondStress(bond, structure.health[index], stress.force[index], stress.moment[index]);
    const BondAssessment assessment = assessBond(bond, state, structure.cracked[index] != 0u);

    damage.utilization = std::max(damage.utilization, utilizationOf(bond, state, structure.cracked[index] != 0u));

    const double remaining = structure.health[index] * (1.0 - assessment.damage);

    if (assessment.cracks) {
      damage.cracked.push_back(index);
    } else if (assessment.damage >= 1.0 || remaining < floorOf(structure, stress, stiffest, index)) {
      damage.broken.push_back(index);
    } else if (assessment.damage >= wearTolerance) {
      damage.eroding.push_back(index);
      damage.erosion.push_back(assessment.damage);
    }
  }

  return damage;
}

void applyCracks(StructureState& structure, const std::vector<uint32_t>& cracked) {
  for (const uint32_t index : cracked) {
    crackBond(structure, index);
  }
}

void applyBreaks(const ChunkedBuilding& building, StructureState& structure, StressState& stress, const Damage& damage) {
  for (size_t slot = 0u; slot != damage.eroding.size(); ++slot) {
    structure.health[damage.eroding[slot]] *= 1.0 - damage.erosion[slot];
  }

  stress.eroding = !damage.eroding.empty();

  if (stress.eroding) {
    structure.revision += 1u;
  }

  for (const uint32_t index : damage.broken) {
    breakBond(structure, building, index);
  }
}

size_t at(int32_t column, int32_t stride, int32_t row) {
  return static_cast<size_t>(column) * static_cast<size_t>(stride) + static_cast<size_t>(row);
}

Block jacobian(const glm::dvec3& offset, double sign) {
  Block g{};

  for (int32_t axis = 0; axis != 6; ++axis) {
    g[at(axis, 6, axis)] = sign;
  }

  g[at(4, 6, 0)] = sign * offset.z;
  g[at(5, 6, 0)] = -sign * offset.y;
  g[at(3, 6, 1)] = -sign * offset.z;
  g[at(5, 6, 1)] = sign * offset.x;
  g[at(3, 6, 2)] = sign * offset.y;
  g[at(4, 6, 2)] = -sign * offset.x;

  return g;
}

constexpr std::array<uint32_t, 6> jacobianRows{0b000001u, 0b000010u, 0b000100u, 0b001110u, 0b010101u, 0b100011u};

Block jacobianProduct(const Block& left, const Column& weight, const Block& right) {
  Block product{};

  for (int32_t column = 0; column != 6; ++column) {
    for (int32_t row = 0; row != 6; ++row) {
      const uint32_t shared = jacobianRows[static_cast<size_t>(row)] & jacobianRows[static_cast<size_t>(column)];
      double sum = 0.0;

      for (int32_t k = 0; k != 6; ++k) {
        if (((shared >> k) & 1u) != 0u) {
          sum += left[at(row, 6, k)] * weight[static_cast<size_t>(k)] * right[at(column, 6, k)];
        }
      }

      product[at(column, 6, row)] = sum;
    }
  }

  return product;
}

void addTo(double* target, int32_t stride, const Block& block, bool flipped) {
  for (int32_t c = 0; c != 6; ++c) {
    for (int32_t r = 0; r != 6; ++r) {
      target[at(c, stride, r)] += flipped ? block[at(r, 6, c)] : block[at(c, 6, r)];
    }
  }
}

Column stiffnessOf(const Scales& scales, size_t index) {
  const glm::dvec3& force = scales.force[index];
  const double moment = scales.moment[index];

  return {force.x * force.x, force.y * force.y, force.z * force.z, moment * moment, moment * moment, moment * moment};
}

std::shared_ptr<const StressPattern> patternOf(
  const ChunkedBuilding& building,
  const StructureState& structure,
  Components components
) {
  const size_t chunks = building.chunks.size();
  std::vector<int32_t> dof(chunks, -1);
  std::vector<std::array<int32_t, 2>> links;
  int32_t columns = 0;

  for (size_t index = 0u; index != chunks; ++index) {
    if (components.active[index] != 0u) {
      dof[index] = columns++;
    }
  }

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    const int32_t a = dof[building.bonds[index].first];
    const int32_t b = dof[building.bonds[index].second];

    if (structure.broken[index] == 0u && a >= 0 && b >= 0) {
      links.push_back({a, b});
    }
  }

  auto pattern = std::make_shared<StressPattern>(StressPattern{
    std::move(components),
    blockPattern(columns, links),
    std::vector<int32_t>(chunks, -1),
    structure.broken,
    structure.alive,
    std::vector<uint32_t>(static_cast<size_t>(columns), 0u),
    std::vector<int64_t>(building.bonds.size(), -1),
    std::vector<uint8_t>(building.bonds.size(), 0u)
  });
  const BlockPattern& blocks = pattern->blocks;

  for (size_t index = 0u; index != chunks; ++index) {
    if (dof[index] >= 0) {
      const int32_t place = blocks.place[static_cast<size_t>(dof[index])];

      pattern->position[index] = place;
      pattern->chunkAt[static_cast<size_t>(place)] = static_cast<uint32_t>(index);
    }
  }

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    const int32_t a = pattern->position[building.bonds[index].first];
    const int32_t b = pattern->position[building.bonds[index].second];

    if (structure.broken[index] != 0u || a < 0 || b < 0) {
      continue;
    }

    const int32_t column = std::min(a, b);
    const auto first = blocks.rows.begin() + blocks.start[static_cast<size_t>(column)];
    const auto last = blocks.rows.begin() + blocks.start[static_cast<size_t>(column) + 1u];
    const auto found = std::lower_bound(first, last, std::max(a, b));

    if (found == last || *found != std::max(a, b)) {
      throw std::logic_error("A block pattern lacks a block its matrix needs");
    }

    pattern->bondAt[index] = blocks.panel[static_cast<size_t>(column)] + 6 * (found - first);
    pattern->bondFlipped[index] = b < a ? 1u : 0u;
  }

  return pattern;
}

Column solveSmall(const Block& system, Column load) {
  if (!solveBlock(system, load)) {
    throw std::runtime_error("A free piece of a structure has no positive mass to move with");
  }

  return load;
}

void validateStates(const ChunkedBuilding& building, const StructureState& structure, const StressState& stress) {
  const size_t chunks = building.chunks.size();
  const size_t bonds = building.bonds.size();

  if (stress.inverseMass.size() != chunks || stress.force.size() != bonds || structure.alive.size() != chunks
      || structure.broken.size() != bonds || structure.cracked.size() != bonds) {
    throw std::invalid_argument("A stress state and a structure state belong to the building they were made from");
  }
}

bool finite(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void validateLoads(const ChunkedBuilding& building, std::span<const ChunkLoad> loads) {
  for (const ChunkLoad& load : loads) {
    if (load.chunk >= building.chunks.size()) {
      throw std::out_of_range("A chunk load names a chunk the building does not hold");
    }

    if (!finite(load.force) || !finite(load.torque)) {
      throw std::invalid_argument("A chunk load needs a finite force and torque");
    }
  }
}

void validateGravity(const glm::dvec3& gravity) {
  if (!finite(gravity)) {
    throw std::invalid_argument("A stress update needs a finite gravity");
  }
}

std::vector<Six> externalOf(const ChunkedBuilding& building, const StructureState& structure, const StressState& stress) {
  const size_t chunks = building.chunks.size();
  std::vector<Six> external(chunks);

  for (size_t index = 0u; index != chunks; ++index) {
    external[index].linear = stress.gravityScale[index] * stress.gravity;
  }

  for (const ChunkLoad& load : stress.loads) {
    if (structure.alive[load.chunk] == 0u) {
      continue;
    }

    external[load.chunk].linear += stress.inverseMass[load.chunk] * load.force;
    external[load.chunk].angular += stress.inverseInertia[load.chunk] * load.torque;
  }

  return external;
}

bool balanced(const ChunkedBuilding& building, const StructureState& structure, const std::vector<Six>& external) {
  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      continue;
    }

    const Bond& bond = building.bonds[index];
    const Six& second = external[bond.second];
    const Six& first = external[bond.first];
    const glm::dvec3 toSecond = bond.centroid - building.chunks[bond.second].centre;
    const glm::dvec3 toFirst = bond.centroid - building.chunks[bond.first].centre;
    const glm::dvec3 linear = (second.linear + glm::cross(second.angular, toSecond))
                              - (first.linear + glm::cross(first.angular, toFirst));
    const glm::dvec3 angular = second.angular - first.angular;

    if (linear != glm::dvec3(0.0) || angular != glm::dvec3(0.0)) {
      return false;
    }
  }

  return true;
}

uint32_t findRoot(std::vector<uint32_t>& parent, uint32_t node) {
  while (parent[node] != node) {
    parent[node] = parent[parent[node]];
    node = parent[node];
  }

  return node;
}

Column massOf(const StressState& stress, size_t index) {
  const glm::dvec3& inertia = stress.inverseInertia[index];
  const double mass = 1.0 / stress.inverseMass[index];

  return {mass, mass, mass, 1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z};
}

Components componentsOf(const ChunkedBuilding& building, const StructureState& structure, const StressState& stress) {
  const size_t chunks = building.chunks.size();
  std::vector<uint32_t> parent(chunks);

  for (size_t index = 0u; index != chunks; ++index) {
    parent[index] = static_cast<uint32_t>(index);
  }

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      continue;
    }

    const uint32_t a = findRoot(parent, building.bonds[index].first);
    const uint32_t b = findRoot(parent, building.bonds[index].second);

    if (a != b) {
      parent[std::max(a, b)] = std::min(a, b);
    }
  }

  Components components;

  components.root.resize(chunks);
  components.anchored.assign(chunks, 0u);
  components.active.assign(chunks, 0u);
  components.piece.assign(chunks, -1);

  for (size_t index = 0u; index != chunks; ++index) {
    components.root[index] = findRoot(parent, static_cast<uint32_t>(index));

    if (!(stress.inverseMass[index] > 0.0)) {
      components.anchored[components.root[index]] = 1u;
    }
  }

  for (size_t index = 0u; index != chunks; ++index) {
    if (!(stress.inverseMass[index] > 0.0)) {
      continue;
    }

    const uint32_t r = components.root[index];

    components.active[index] = components.anchored[r] != 0u || r != index ? 1u : 0u;

    if (components.anchored[r] != 0u) {
      continue;
    }

    if (components.piece[r] < 0) {
      components.piece[r] = static_cast<int32_t>(components.rigidMass.size());
      components.rigidMass.push_back(Block{});
    }

    const Block transport = jacobian(building.chunks[index].centre - building.chunks[r].centre, 1.0);
    const Block inertia = jacobianProduct(transport, massOf(stress, index), transport);
    Block& mass = components.rigidMass[static_cast<size_t>(components.piece[r])];

    for (size_t entry = 0u; entry != 36u; ++entry) {
      mass[entry] += inertia[entry];
    }
  }

  return components;
}

Column aboutRoot(const glm::dvec3& offset, const Column& load) {
  Column sum{};

  sum[0] += load[0];
  sum[1] += load[1];
  sum[2] += load[2];
  sum[3] += -offset.z * load[1];
  sum[3] += offset.y * load[2];
  sum[3] += load[3];
  sum[4] += offset.z * load[0];
  sum[4] += -offset.x * load[2];
  sum[4] += load[4];
  sum[5] += -offset.y * load[0];
  sum[5] += offset.x * load[1];
  sum[5] += load[5];

  return sum;
}

Column fromRoot(const glm::dvec3& offset, const Column& acceleration) {
  Column sum{};

  sum[0] += acceleration[0];
  sum[0] += offset.z * acceleration[4];
  sum[0] += -offset.y * acceleration[5];
  sum[1] += acceleration[1];
  sum[1] += -offset.z * acceleration[3];
  sum[1] += offset.x * acceleration[5];
  sum[2] += acceleration[2];
  sum[2] += offset.y * acceleration[3];
  sum[2] += -offset.x * acceleration[4];
  sum[3] += acceleration[3];
  sum[4] += acceleration[4];
  sum[5] += acceleration[5];

  return sum;
}

Pieces piecesOf(
  const ChunkedBuilding& building,
  const StressState& stress,
  const std::vector<Six>& external,
  const Components& components
) {
  const size_t chunks = building.chunks.size();
  Pieces pieces;
  std::vector<Column> rigidLoad(components.rigidMass.size(), Column{});

  pieces.active = components.active;
  pieces.load.assign(chunks, Column{});

  for (size_t index = 0u; index != chunks; ++index) {
    if (!(stress.inverseMass[index] > 0.0)) {
      continue;
    }

    Column& load = pieces.load[index];

    for (int32_t axis = 0; axis != 3; ++axis) {
      load[static_cast<size_t>(axis)] = external[index].linear[axis] / stress.inverseMass[index];
      load[static_cast<size_t>(axis) + 3u] = external[index].angular[axis] / stress.inverseInertia[index][axis];
    }

    const uint32_t r = components.root[index];

    if (components.anchored[r] != 0u) {
      continue;
    }

    const size_t slot = static_cast<size_t>(components.piece[r]);
    const Column share = aboutRoot(building.chunks[index].centre - building.chunks[r].centre, load);

    for (size_t a = 0u; a != 6u; ++a) {
      rigidLoad[slot][a] += share[a];
    }
  }

  std::vector<Column> rigid(components.rigidMass.size());

  for (size_t slot = 0u; slot != components.rigidMass.size(); ++slot) {
    rigid[slot] = solveSmall(components.rigidMass[slot], rigidLoad[slot]);
  }

  for (size_t index = 0u; index != chunks; ++index) {
    if (!(stress.inverseMass[index] > 0.0) || components.anchored[components.root[index]] != 0u) {
      continue;
    }

    const uint32_t r = components.root[index];
    const Column motion =
      fromRoot(building.chunks[index].centre - building.chunks[r].centre, rigid[static_cast<size_t>(components.piece[r])]);
    const Column mass = massOf(stress, index);

    for (size_t k = 0u; k != 6u; ++k) {
      pieces.load[index][k] -= mass[k] * motion[k];
    }
  }

  return pieces;
}

struct Factored {
  std::shared_ptr<StressFactor> factor;
  uint32_t failed = noChunk;
  uint64_t work = 0u;
};

int32_t strideOf(const BlockPattern& blocks, int32_t column) {
  return 6 * (blocks.start[static_cast<size_t>(column) + 1u] - blocks.start[static_cast<size_t>(column)]);
}

uint64_t workOf(const BlockPattern& blocks, int32_t column) {
  const auto rows = static_cast<uint64_t>(blocks.start[static_cast<size_t>(column) + 1u] - blocks.start[static_cast<size_t>(column)]);

  return rows * rows;
}

uint64_t workOf(const BlockPattern& blocks, std::span<const int32_t> columns) {
  uint64_t work = 0u;

  for (const int32_t column : columns) {
    work += workOf(blocks, column);
  }

  return work;
}

uint64_t workOf(const BlockPattern& blocks) {
  uint64_t work = 0u;

  for (size_t column = 0u; column + 1u < blocks.start.size(); ++column) {
    work += workOf(blocks, static_cast<int32_t>(column));
  }

  return work;
}

void assembleAll(
  const ChunkedBuilding& building,
  const StructureState& structure,
  const Scales& scales,
  const std::vector<glm::dvec3>& offsets,
  const StressPattern& pattern,
  const Pieces& pieces,
  std::vector<double>& values
) {
  const BlockPattern& blocks = pattern.blocks;

  values.assign(static_cast<size_t>(blocks.panel.back()), 0.0);

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      continue;
    }

    const Bond& bond = building.bonds[index];
    const bool secondMoves = pieces.active[bond.second] != 0u;
    const bool firstMoves = pieces.active[bond.first] != 0u;

    if (!secondMoves && !firstMoves) {
      continue;
    }

    const Column stiffness = stiffnessOf(scales, index);
    const Block second = jacobian(offsets[2u * index], 1.0);
    const Block first = jacobian(offsets[2u * index + 1u], -1.0);
    const int32_t b = pattern.position[bond.second];
    const int32_t a = pattern.position[bond.first];

    if (secondMoves) {
      addTo(values.data() + blocks.panel[static_cast<size_t>(b)], strideOf(blocks, b), jacobianProduct(second, stiffness, second), false);
    }

    if (firstMoves) {
      addTo(values.data() + blocks.panel[static_cast<size_t>(a)], strideOf(blocks, a), jacobianProduct(first, stiffness, first), false);
    }

    if (secondMoves && firstMoves) {
      addTo(
        values.data() + pattern.bondAt[index],
        strideOf(blocks, std::min(a, b)),
        jacobianProduct(second, stiffness, first),
        pattern.bondFlipped[index] != 0u
      );
    }
  }
}

void assembleColumn(
  const ChunkedBuilding& building,
  const StructureState& structure,
  const Scales& scales,
  const std::vector<glm::dvec3>& offsets,
  const StressPattern& pattern,
  const Pieces& pieces,
  std::vector<double>& values,
  int32_t column
) {
  const BlockPattern& blocks = pattern.blocks;
  const uint32_t chunk = pattern.chunkAt[static_cast<size_t>(column)];
  const int32_t stride = strideOf(blocks, column);
  double* panel = values.data() + blocks.panel[static_cast<size_t>(column)];

  std::fill(panel, values.data() + blocks.panel[static_cast<size_t>(column) + 1u], 0.0);

  for (uint32_t slot = structure.bondStart[chunk]; slot != structure.bondStart[chunk + 1u]; ++slot) {
    const uint32_t index = structure.bondList[slot];

    if (structure.broken[index] != 0u) {
      continue;
    }

    const Bond& bond = building.bonds[index];
    const Column stiffness = stiffnessOf(scales, index);
    const bool second = bond.second == chunk;
    const uint32_t other = second ? bond.first : bond.second;
    const Block own = second ? jacobian(offsets[2u * index], 1.0) : jacobian(offsets[2u * index + 1u], -1.0);

    addTo(panel, stride, jacobianProduct(own, stiffness, own), false);

    if (pieces.active[other] != 0u && pattern.position[other] > column) {
      const Block secondJacobian = jacobian(offsets[2u * index], 1.0);
      const Block firstJacobian = jacobian(offsets[2u * index + 1u], -1.0);

      addTo(
        values.data() + pattern.bondAt[index],
        stride,
        jacobianProduct(secondJacobian, stiffness, firstJacobian),
        pattern.bondFlipped[index] != 0u
      );
    }
  }
}

std::vector<glm::dvec3> offsetsOf(const ChunkedBuilding& building) {
  std::vector<glm::dvec3> offsets(2u * building.bonds.size());

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    const Bond& bond = building.bonds[index];

    offsets[2u * index] = bond.centroid - building.chunks[bond.second].centre;
    offsets[2u * index + 1u] = bond.centroid - building.chunks[bond.first].centre;
  }

  return offsets;
}

std::vector<int32_t> changedColumns(
  const ChunkedBuilding& building,
  const StructureState& structure,
  const Scales& scales,
  const std::vector<glm::dvec3>& offsets,
  const StressPattern& pattern,
  const Pieces& pieces,
  const StressFactor& previous
) {
  const BlockPattern& blocks = pattern.blocks;
  std::vector<uint8_t> mark(blocks.start.size() - 1u, 0u);
  std::vector<int32_t> columns;

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u
        || (previous.forceScales[index] == scales.force[index] && previous.momentScales[index] == scales.moment[index]
            && previous.offsets[2u * index] == offsets[2u * index]
            && previous.offsets[2u * index + 1u] == offsets[2u * index + 1u])) {
      continue;
    }

    for (const uint32_t chunk : {building.bonds[index].first, building.bonds[index].second}) {
      if (pieces.active[chunk] == 0u) {
        continue;
      }

      for (int32_t j = pattern.position[chunk]; j >= 0 && mark[static_cast<size_t>(j)] == 0u;
           j = blocks.parent[static_cast<size_t>(j)]) {
        mark[static_cast<size_t>(j)] = 1u;
      }
    }
  }

  for (size_t j = 0u; j != mark.size(); ++j) {
    if (mark[j] != 0u) {
      columns.push_back(static_cast<int32_t>(j));
    }
  }

  return columns;
}

Factored factorOf(
  const ChunkedBuilding& building,
  const StructureState& structure,
  const Scales& scales,
  std::shared_ptr<const StressPattern> pattern,
  const Pieces& pieces,
  std::shared_ptr<StressFactor> previous,
  const dynamics::Workers& workers,
  Lanes lanes
) {
  const BlockPattern& blocks = pattern->blocks;
  std::vector<glm::dvec3> offsets = offsetsOf(building);
  std::shared_ptr<StressFactor> factor;
  bool factorized = false;
  uint64_t work = 0u;

  if (previous && previous->pattern == pattern) {
    const std::vector<int32_t> columns = changedColumns(building, structure, scales, offsets, *pattern, pieces, *previous);

    work += workOf(blocks, columns);
    factor = previous.use_count() == 1 ? std::move(previous) : std::make_shared<StressFactor>(*previous);

    for (const int32_t column : columns) {
      assembleColumn(building, structure, scales, offsets, *pattern, pieces, factor->values, column);
    }

    factorized = columns.empty() || factorBlocks(blocks, factor->values, columns, workers, lanes);
  } else if (workers.count > 1u) {
    std::vector<int32_t> columns(blocks.start.size() - 1u);

    for (size_t j = 0u; j != columns.size(); ++j) {
      columns[j] = static_cast<int32_t>(j);
    }

    work += workOf(blocks);
    factor = std::make_shared<StressFactor>();
    assembleAll(building, structure, scales, offsets, *pattern, pieces, factor->values);
    factorized = factorBlocks(blocks, factor->values, columns, workers, lanes);
  }

  if (!factorized) {
    work += workOf(blocks);
    factor = std::make_shared<StressFactor>();
    assembleAll(building, structure, scales, offsets, *pattern, pieces, factor->values);

    const std::optional<int32_t> failed = factorBlocks(blocks, factor->values, lanes);

    if (failed) {
      if (!std::all_of(factor->values.begin(), factor->values.end(), [](double value) { return std::isfinite(value); })) {
        throw std::runtime_error("A structure's stiffness could not be factorized");
      }

      return Factored{nullptr, pattern->chunkAt[static_cast<size_t>(*failed)], work};
    }
  }

  factor->pattern = std::move(pattern);
  factor->revision = structure.revision;
  factor->forceScales = scales.force;
  factor->momentScales = scales.moment;
  factor->offsets = std::move(offsets);

  return Factored{std::move(factor), noChunk, work};
}

struct Round {
  bool factorized = false;
  uint32_t failed = noChunk;
  double factorizing = 0.0;
  uint32_t chunks = 0u;
  uint64_t work = 0u;
};

Round solveRound(
  const ChunkedBuilding& building,
  const StructureState& structure,
  StressState& stress,
  const dynamics::Workers& workers,
  Lanes lanes
) {
  stress.solved = false;

  const size_t chunks = building.chunks.size();
  const std::vector<Six> external = externalOf(building, structure, stress);

  if (balanced(building, structure, external)) {
    std::fill(stress.force.begin(), stress.force.end(), glm::dvec3(0.0));
    std::fill(stress.moment.begin(), stress.moment.end(), glm::dvec3(0.0));
    stress.revision = structure.revision;
    stress.solved = true;

    return Round{};
  }

  const bool live = stress.factor && stress.factor->pattern->broken == structure.broken
                    && stress.factor->pattern->alive == structure.alive;

  if (stress.factor && stress.factor->revision == structure.revision && !live) {
    throw std::logic_error("A stress state is used with a structure state its factor was not built for");
  }

  Components fresh = live ? Components{} : componentsOf(building, structure, stress);
  const Pieces pieces = piecesOf(building, stress, external, live ? stress.factor->pattern->components : fresh);
  const auto active =
    static_cast<uint32_t>(std::count_if(pieces.active.begin(), pieces.active.end(), [](uint8_t flag) { return flag != 0u; }));
  bool factorized = false;
  uint64_t work = 0u;

  double factorizing = 0.0;

  if (!stress.factor || stress.factor->revision != structure.revision) {
    dynamics::Stopwatch watch;
    const Scales scales = scalesOf(building, structure, stress);
    std::shared_ptr<const StressPattern> pattern =
      live ? stress.factor->pattern : patternOf(building, structure, std::move(fresh));
    std::shared_ptr<StressFactor> previous = live ? std::move(stress.factor) : std::shared_ptr<StressFactor>{};
    Factored factored = factorOf(building, structure, scales, std::move(pattern), pieces, std::move(previous), workers, lanes);

    factorizing = watch.lap();
    work = factored.work;

    if (!factored.factor) {
      return Round{true, factored.failed, factorizing, active, work};
    }

    stress.factor = std::move(factored.factor);
    factorized = true;
  }

  const StressPattern& pattern = *stress.factor->pattern;
  std::vector<double> motion(6u * (pattern.blocks.start.size() - 1u), 0.0);

  for (size_t index = 0u; index != chunks; ++index) {
    if (pieces.active[index] == 0u) {
      continue;
    }

    for (int32_t axis = 0; axis != 6; ++axis) {
      motion[at(pattern.position[index], 6, axis)] = -pieces.load[index][static_cast<size_t>(axis)];
    }
  }

  solveBlocks(pattern.blocks, stress.factor->values, motion);

  const auto shiftOf = [&](uint32_t chunk) {
    return pieces.active[chunk] != 0u ? glm::dvec3(motion[at(pattern.position[chunk], 6, 0)],
                                                   motion[at(pattern.position[chunk], 6, 1)],
                                                   motion[at(pattern.position[chunk], 6, 2)])
                                      : glm::dvec3(0.0);
  };
  const auto turnOf = [&](uint32_t chunk) {
    return pieces.active[chunk] != 0u ? glm::dvec3(motion[at(pattern.position[chunk], 6, 3)],
                                                   motion[at(pattern.position[chunk], 6, 4)],
                                                   motion[at(pattern.position[chunk], 6, 5)])
                                      : glm::dvec3(0.0);
  };

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      stress.force[index] = glm::dvec3(0.0);
      stress.moment[index] = glm::dvec3(0.0);
      continue;
    }

    const Bond& bond = building.bonds[index];
    const glm::dvec3 toSecond = bond.centroid - building.chunks[bond.second].centre;
    const glm::dvec3 toFirst = bond.centroid - building.chunks[bond.first].centre;
    const glm::dvec3 linear = (shiftOf(bond.second) + glm::cross(turnOf(bond.second), toSecond))
                              - (shiftOf(bond.first) + glm::cross(turnOf(bond.first), toFirst));
    const glm::dvec3 angular = turnOf(bond.second) - turnOf(bond.first);
    const glm::dvec3 forceScale = stress.factor->forceScales[index];
    const double momentScale = stress.factor->momentScales[index];

    stress.force[index] = (forceScale * forceScale) * linear;
    stress.moment[index] = (momentScale * momentScale) * angular;
  }

  stress.revision = structure.revision;
  stress.solved = true;

  return Round{factorized, noChunk, factorizing, active, work};
}

uint32_t softestBond(
  const ChunkedBuilding& building,
  const StructureState& structure,
  const StressState& stress,
  uint32_t chunk
) {
  std::vector<uint32_t> parent(building.chunks.size());

  for (size_t index = 0u; index != parent.size(); ++index) {
    parent[index] = static_cast<uint32_t>(index);
  }

  for (size_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      continue;
    }

    const uint32_t a = findRoot(parent, building.bonds[index].first);
    const uint32_t b = findRoot(parent, building.bonds[index].second);

    if (a != b) {
      parent[std::max(a, b)] = std::min(a, b);
    }
  }

  const uint32_t root = findRoot(parent, chunk);
  const double stiffest = stiffestShare(structure, stress);
  uint32_t softest = noBond;
  double least = 0.0;

  for (uint32_t index = 0u; index != building.bonds.size(); ++index) {
    if (structure.broken[index] != 0u || findRoot(parent, building.bonds[index].first) != root) {
      continue;
    }

    const double relative = structure.health[index] / floorOf(structure, stress, stiffest, index);
    const bool weak = structure.health[index] < 1.0 || relative < 1.0;

    if (weak && (softest == noBond || relative < least)) {
      softest = index;
      least = relative;
    }
  }

  return softest;
}

}

namespace {

StressState stressFor(
  const ChunkedBuilding& building,
  MassWeighting weighting,
  LoadSharing sharing,
  const StressState* like
) {
  const size_t chunks = building.chunks.size();
  const size_t bonds = building.bonds.size();
  StressState stress;
  double reach = 0.0;
  double area = 0.0;

  for (const Bond& bond : building.bonds) {
    if (!(bond.first < bond.second) || bond.second >= chunks) {
      throw std::invalid_argument("A stress state's bond needs two chunks the building holds, the first below the second");
    }

    if (!(bond.area > 0.0) || !(bond.wu > 0.0) || !(bond.wv > 0.0)) {
      throw std::invalid_argument("A stress state needs bonds with a positive area and face extents");
    }

    reach += glm::length(bond.centroid - building.chunks[bond.first].centre);
    reach += glm::length(bond.centroid - building.chunks[bond.second].centre);
    area += bond.area;
  }

  if (reach > 0.0) {
    stress.length = reach / (2.0 * static_cast<double>(bonds));
  }

  if (area > 0.0) {
    stress.area = area / static_cast<double>(bonds);
  }

  if (like != nullptr && like->area > 0.0) {
    stress.length = like->length;
    stress.area = like->area;
  }

  stress.force.assign(bonds, glm::dvec3(0.0));
  stress.moment.assign(bonds, glm::dvec3(0.0));
  stress.forceShare.assign(bonds, 1.0);
  stress.momentShare.assign(bonds, 1.0);
  stress.inverseMass.assign(chunks, 0.0);
  stress.inverseInertia.assign(chunks, glm::dvec3(0.0));
  stress.gravityScale.assign(chunks, 0.0);

  if (sharing == LoadSharing::Area && stress.area > 0.0) {
    for (size_t index = 0u; index != bonds; ++index) {
      stress.forceShare[index] = std::sqrt(building.bonds[index].area / stress.area);
      stress.momentShare[index] = building.bonds[index].area / stress.area;
    }
  }

  double totalMass = 0.0;
  size_t freeChunks = 0u;

  for (const Chunk& chunk : building.chunks) {
    if (!chunk.anchored && chunk.mass > 0.0) {
      totalMass += chunk.mass;
      freeChunks += 1u;
    }
  }

  const double meanMass = freeChunks != 0u ? totalMass / static_cast<double>(freeChunks) : 1.0;

  for (size_t index = 0u; index != chunks; ++index) {
    const Chunk& chunk = building.chunks[index];

    if (chunk.anchored || !(chunk.mass > 0.0)) {
      continue;
    }

    if (weighting == MassWeighting::Real) {
      const glm::dvec3 h = chunk.halfExtents;
      const glm::dvec3 inertia = (chunk.mass / 3.0)
                                 * glm::dvec3(h.y * h.y + h.z * h.z, h.x * h.x + h.z * h.z, h.x * h.x + h.y * h.y);

      stress.inverseMass[index] = 1.0 / chunk.mass;
      stress.inverseInertia[index] = 1.0 / inertia;
      stress.gravityScale[index] = 1.0;
    } else {
      stress.inverseMass[index] = 1.0 / meanMass;
      stress.inverseInertia[index] = glm::dvec3(1.0 / (meanMass * stress.length * stress.length));
      stress.gravityScale[index] = chunk.mass / meanMass;
    }
  }

  return stress;
}

}

StressState makeStress(const ChunkedBuilding& building, MassWeighting weighting, LoadSharing sharing) {
  return stressFor(building, weighting, sharing, nullptr);
}

StressState makeStressLike(const ChunkedBuilding& building, const StressState& like) {
  return stressFor(building, MassWeighting::Real, LoadSharing::Area, &like);
}

void solveStress(
  const ChunkedBuilding& building,
  const StructureState& structure,
  StressState& stress,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings
) {
  validateStates(building, structure, stress);
  validateLoads(building, loads);
  validateGravity(settings.gravity);

  stress.loads.assign(loads.begin(), loads.end());
  stress.gravity = settings.gravity;

  if (solveRound(building, structure, stress, settings.workers, settings.lanes).failed != noChunk) {
    throw std::runtime_error("A structure's stiffness could not be factorized");
  }
}

uint64_t factorBytes(const StressState& stress) {
  return stress.factor ? static_cast<uint64_t>(stress.factor->values.size() * sizeof(double)) : 0u;
}

BondStress bondStress(const Bond& bond, double health, const glm::dvec3& force, const glm::dvec3& moment) {
  BondStress stress;
  const double area = bond.area * health;

  if (!(area > 0.0)) {
    return stress;
  }

  const int axis = normalAxis(bond.normal);
  const int u = (axis + 1) % 3;
  const int v = (axis + 2) % 3;
  const double longSide = std::max(bond.wu, bond.wv);
  const double shortSide = std::min(bond.wu, bond.wv);
  const double push = glm::dot(force, bond.normal);
  const double sliding = glm::length(force - push * bond.normal);
  const double bending = 6.0
                         * (std::abs(moment[u]) / (bond.wu * bond.wv * bond.wv)
                            + std::abs(moment[v]) / (bond.wv * bond.wu * bond.wu))
                         / health;
  const double twist = std::abs(glm::dot(moment, bond.normal)) * (3.0 + 1.8 * shortSide / longSide)
                       / (longSide * shortSide * shortSide * health);

  stress.pressure = push / area;
  stress.compression = std::max(0.0, stress.pressure + bending);
  stress.tension = std::max(0.0, bending - stress.pressure);
  stress.shear = sliding / area + twist;
  stress.rocking = std::max(2.0 * std::abs(moment[u]) / bond.wv, 2.0 * std::abs(moment[v]) / bond.wu) / area;

  return stress;
}

double damageOf(const Bond& bond, const BondStress& stress) {
  const double compression = excess(
    stress.compression,
    bond.elastic.compression / bond.area,
    bond.fatal.compression / bond.area
  );
  const double tension = excess(stress.tension, bond.elastic.tension / bond.area, bond.fatal.tension / bond.area);
  const double shear = excess(
    stress.shear,
    shearStrength(bond, bond.elastic, stress.pressure),
    shearStrength(bond, bond.fatal, stress.pressure)
  );

  return std::min(1.0, compression + tension + shear);
}

BondAssessment assessBond(const Bond& bond, const BondStress& stress, bool cracked) {
  BondAssessment assessment;

  if (!cracked) {
    if (bond.crackFriction > 0.0 && stress.pressure > 0.0 && stress.tension >= bond.fatal.tension / bond.area) {
      assessment.cracks = true;

      return assessment;
    }

    assessment.damage = damageOf(bond, stress);

    return assessment;
  }

  if (stress.pressure < 0.0 || stress.shear > bond.crackFriction * stress.pressure || stress.rocking > stress.pressure) {
    assessment.damage = 1.0;

    return assessment;
  }

  assessment.damage = excess(
    stress.compression,
    bond.elastic.compression / bond.area,
    bond.fatal.compression / bond.area
  );

  return assessment;
}

StressReport updateStress(
  const ChunkedBuilding& building,
  StructureState& structure,
  StressState& stress,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings
) {
  validateStates(building, structure, stress);

  StressReport report;

  if (!stress.pending) {
    const bool unchanged = stress.solved && stress.revision == structure.revision && stress.gravity == settings.gravity
                           && sameLoads(stress.loads, loads);

    if (unchanged) {
      return report;
    }

    validateLoads(building, loads);
    validateGravity(settings.gravity);
    stress.loads.assign(loads.begin(), loads.end());
    stress.gravity = settings.gravity;
    stress.rounds = 0u;
  }

  report.ran = true;

  std::vector<uint32_t> recovered;
  const bool large = settings.wideChunks != 0u
                     && static_cast<size_t>(std::count(structure.alive.begin(), structure.alive.end(), uint8_t{1}))
                          >= settings.wideChunks;

  for (;;) {
    if (large && (!stress.factor || stress.factor->revision != structure.revision)) {
      report.broken = std::move(recovered);
      stress.pending = true;
      report.pending = true;
      report.wide = true;

      return report;
    }

    const Round round = solveRound(building, structure, stress, settings.workers, settings.lanes);

    report.factorizations += round.factorized ? 1u : 0u;
    report.factorizedChunks += round.factorized ? round.chunks : 0u;
    report.factorWork += round.work;
    report.factorizing += round.factorizing;

    if (round.failed != noChunk) {
      const uint32_t bond = softestBond(building, structure, stress, round.failed);

      if (bond == noBond) {
        throw std::runtime_error("A structure's stiffness could not be factorized");
      }

      breakBond(structure, building, bond);
      recovered.push_back(bond);

      continue;
    }

    report.rounds += 1u;
    report.solvedChunks += round.chunks;
    stress.rounds += 1u;

    Damage damage = assessDamage(building, structure, stress);

    applyCracks(structure, damage.cracked);
    report.cracked.insert(report.cracked.end(), damage.cracked.begin(), damage.cracked.end());

    if (damage.cracked.empty() || stress.rounds >= settings.crackRounds) {
      stress.utilization = damage.utilization;
      applyBreaks(building, structure, stress, damage);
      report.broken = std::move(recovered);
      report.broken.insert(report.broken.end(), damage.broken.begin(), damage.broken.end());
      stress.pending = false;

      return report;
    }

    if (settings.roundsPerCall != 0u && report.rounds >= settings.roundsPerCall) {
      report.broken = std::move(recovered);
      stress.pending = true;
      report.pending = true;

      return report;
    }
  }
}

double loadChange(
  std::span<const ChunkLoad> before,
  const glm::dvec3& beforeGravity,
  std::span<const ChunkLoad> loads,
  const glm::dvec3& gravity
) {
  const auto size = [](const ChunkLoad& load) {
    return glm::length(load.force) + glm::length(load.torque) / torqueLever;
  };
  double change = 0.0;
  double scale = 0.0;
  size_t first = 0u;
  size_t second = 0u;

  while (first != before.size() || second != loads.size()) {
    if (second == loads.size() || (first != before.size() && before[first].chunk < loads[second].chunk)) {
      change += size(before[first]);
      scale += size(before[first]);
      ++first;
    } else if (first == before.size() || loads[second].chunk < before[first].chunk) {
      change += size(loads[second]);
      scale += size(loads[second]);
      ++second;
    } else {
      change += glm::length(loads[second].force - before[first].force)
                + glm::length(loads[second].torque - before[first].torque) / torqueLever;
      scale += std::max(size(before[first]), size(loads[second]));
      ++first;
      ++second;
    }
  }

  const double loaded = scale > 0.0 ? change / scale : 0.0;
  const double weight = glm::length(gravity);
  const double turned = weight > 0.0 ? glm::length(gravity - beforeGravity) / weight : glm::length(beforeGravity);

  return std::max(loaded, turned);
}

double loadChange(const StressState& stress, std::span<const ChunkLoad> loads, const glm::dvec3& gravity) {
  return loadChange(stress.loads, stress.gravity, loads, gravity);
}

bool withinTier(double change, double utilization) {
  return change < 0.01 || (change < 0.03 && utilization < 0.75) || (change < 0.1 && utilization < 0.5);
}

bool tolerates(
  const StressState& stress,
  const StructureState& structure,
  std::span<const ChunkLoad> loads,
  const glm::dvec3& gravity
) {
  if (!stress.solved || stress.eroding || stress.pending || stress.revision != structure.revision) {
    return false;
  }

  return withinTier(loadChange(stress, loads, gravity), stress.utilization);
}

uint64_t factorWorkBound(const StressState& stress, const StructureState& structure) {
  if (!stress.factor) {
    const auto alive = static_cast<uint64_t>(std::count(structure.alive.begin(), structure.alive.end(), uint8_t{1}));

    return alive * alive;
  }

  return stress.factor->revision != structure.revision ? workOf(stress.factor->pattern->blocks) : 0u;
}

}
