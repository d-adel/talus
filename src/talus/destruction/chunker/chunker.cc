#include "chunker.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace talus::destruction {

namespace {

constexpr uint32_t noChunk = 0xFFFFFFFFu;
constexpr double touchTolerance = 1.0e-9;
constexpr double grainTie = 1.0e-9;
constexpr double cosine45 = 0.70710678118654752;
constexpr double rootThree = 1.7320508075688772;

struct Box {
  glm::dvec3 centre{0.0};
  glm::dvec3 halfExtents{0.0};
};

struct Face {
  int axis = 0;
  double side = 1.0;
  glm::dvec3 centroid{0.0};
  glm::dvec2 extents{0.0};
  double area = 0.0;
};

struct SideLimits {
  ModeLimits elastic;
  ModeLimits fatal;
};

double extent(const Box& box) {
  return 2.0 * std::max(box.halfExtents.x, std::max(box.halfExtents.y, box.halfExtents.z));
}

double slackOf(const Box& a, const Box& b) {
  return touchTolerance * std::max(extent(a), extent(b));
}

double depth(const Box& a, const Box& b, int axis) {
  return a.halfExtents[axis] + b.halfExtents[axis] - std::abs(b.centre[axis] - a.centre[axis]);
}

bool near(const Box& a, const Box& b) {
  const double slack = slackOf(a, b);

  for (int axis = 0; axis != 3; ++axis) {
    if (depth(a, b, axis) < -slack) {
      return false;
    }
  }

  return true;
}

bool overlapping(const Box& a, const Box& b) {
  const double slack = slackOf(a, b);

  for (int axis = 0; axis != 3; ++axis) {
    if (!(depth(a, b, axis) > slack)) {
      return false;
    }
  }

  return true;
}

std::optional<Face> sharedFace(const Box& a, const Box& b) {
  const double slack = slackOf(a, b);

  for (int axis = 0; axis != 3; ++axis) {
    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;

    if (std::abs(depth(a, b, axis)) > slack || !(depth(a, b, u) > slack) || !(depth(a, b, v) > slack)) {
      continue;
    }

    Face face;

    face.axis = axis;
    face.side = b.centre[axis] > a.centre[axis] ? 1.0 : -1.0;
    face.centroid[axis] = 0.5 * ((a.centre[axis] + face.side * a.halfExtents[axis])
                                 + (b.centre[axis] - face.side * b.halfExtents[axis]));

    for (const int across : {u, v}) {
      const double low = std::max(
        a.centre[across] - a.halfExtents[across],
        b.centre[across] - b.halfExtents[across]
      );
      const double high = std::min(
        a.centre[across] + a.halfExtents[across],
        b.centre[across] + b.halfExtents[across]
      );

      face.centroid[across] = 0.5 * (low + high);
      face.extents[across == u ? 0 : 1] = high - low;
    }

    face.area = face.extents.x * face.extents.y;

    return face;
  }

  return std::nullopt;
}

Box boxOf(const Member& member) {
  return Box{member.centre, member.halfExtents};
}

Box boxOf(const Chunk& chunk) {
  return Box{chunk.centre, chunk.halfExtents};
}

double boundary(double centre, double half, uint32_t count, uint32_t index) {
  return centre + half * ((2.0 * index - count) / count);
}

Box cellOf(const Member& member, const glm::uvec3& count, const glm::uvec3& cell) {
  Box box;

  for (int axis = 0; axis != 3; ++axis) {
    const double low = boundary(member.centre[axis], member.halfExtents[axis], count[axis], cell[axis]);
    const double high = boundary(member.centre[axis], member.halfExtents[axis], count[axis], cell[axis] + 1u);

    box.centre[axis] = 0.5 * (low + high);
    box.halfExtents[axis] = 0.5 * (high - low);
  }

  return box;
}

bool inside(const glm::dvec3& point, const Opening& opening) {
  for (int axis = 0; axis != 3; ++axis) {
    if (!(std::abs(point[axis] - opening.centre[axis]) < opening.halfExtents[axis])) {
      return false;
    }
  }

  return true;
}

bool cut(const glm::dvec3& point, std::span<const Opening> openings) {
  return std::any_of(openings.begin(), openings.end(), [&](const Opening& opening) {
    return inside(point, opening);
  });
}

size_t cellIndex(const glm::uvec3& count, const glm::uvec3& cell) {
  return cell.x + static_cast<size_t>(count.x) * (cell.y + static_cast<size_t>(count.y) * cell.z);
}

int grainAxis(const Member& member) {
  const double longest = std::max(member.halfExtents.x, std::max(member.halfExtents.y, member.halfExtents.z));

  for (const int axis : {1, 0, 2}) {
    if (member.halfExtents[axis] >= longest * (1.0 - grainTie)) {
      return axis;
    }
  }

  return 1;
}

int thinnestAxis(const Member& member) {
  const double thinnest = std::min(member.halfExtents.x, std::min(member.halfExtents.y, member.halfExtents.z));

  for (const int axis : {1, 0, 2}) {
    if (member.halfExtents[axis] <= thinnest * (1.0 + grainTie)) {
      return axis;
    }
  }

  return 1;
}

glm::dvec3 grainOf(const Member& member) {
  glm::dvec3 grain(0.0);

  grain[grainAxis(member)] = 1.0;

  return grain;
}

SideLimits sideLimits(const StructuralMaterial& material, const Member& member, const glm::dvec3& normal) {
  SideLimits limits{material.elastic, material.fatal};

  if (material.anisotropy == Anisotropy::Grain && std::abs(glm::dot(normal, grainOf(member))) < cosine45) {
    limits = SideLimits{material.acrossElastic, material.acrossFatal};
  }

  if (material.anisotropy == Anisotropy::Courses && std::abs(normal.y) < cosine45) {
    limits.elastic.tension *= material.headJoints;
    limits.fatal.tension *= material.headJoints;
  }

  return limits;
}

bool shearsFirst(
  const SideLimits& a,
  const StructuralMaterial& first,
  const SideLimits& b,
  const StructuralMaterial& second
) {
  if (a.elastic.shear != b.elastic.shear) {
    return a.elastic.shear < b.elastic.shear;
  }

  if (a.fatal.shear != b.fatal.shear) {
    return a.fatal.shear < b.fatal.shear;
  }

  if (first.friction != second.friction) {
    return first.friction < second.friction;
  }

  return first.shearRule <= second.shearRule;
}

std::optional<Bond> joined(
  uint32_t first,
  uint32_t second,
  const Chunk& a,
  const Chunk& b,
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const ChunkerSettings& settings
) {
  const std::optional<Face> face = sharedFace(boxOf(a), boxOf(b));

  if (!face.has_value()) {
    return std::nullopt;
  }

  const StructuralMaterial& aMaterial = materials[a.material];
  const StructuralMaterial& bMaterial = materials[b.material];
  Bond bond;

  bond.first = first;
  bond.second = second;
  bond.centroid = face->centroid;
  bond.normal[face->axis] = face->side;
  bond.area = face->area;
  bond.wu = face->extents.x;
  bond.wv = face->extents.y;

  const SideLimits aLimits = sideLimits(aMaterial, building.members[a.member], bond.normal);
  const SideLimits bLimits = sideLimits(bMaterial, building.members[b.member], bond.normal);
  const bool aShears = shearsFirst(aLimits, aMaterial, bLimits, bMaterial);
  const SideLimits& shearing = aShears ? aLimits : bLimits;
  const StructuralMaterial& rule = aShears ? aMaterial : bMaterial;
  const double factor = settings.variety
                          ? varietyFactor(first, second, std::max(aMaterial.variation, bMaterial.variation))
                          : 1.0;
  const double scale = bond.area * factor;

  bond.elastic = ModeLimits{
    std::min(aLimits.elastic.compression, bLimits.elastic.compression) * scale,
    std::min(aLimits.elastic.tension, bLimits.elastic.tension) * scale,
    shearing.elastic.shear * scale
  };
  bond.fatal = ModeLimits{
    std::min(aLimits.fatal.compression, bLimits.fatal.compression) * scale,
    std::min(aLimits.fatal.tension, bLimits.fatal.tension) * scale,
    shearing.fatal.shear * scale
  };
  bond.shearRule = rule.shearRule;
  bond.friction = rule.friction;
  bond.crackFriction = a.member != b.member
                         ? std::min(aMaterial.friction, bMaterial.friction)
                         : (aMaterial.shearRule == ShearRule::Coulomb ? aMaterial.friction : 0.0);

  return bond;
}

class Chunker {
public:
  Chunker(
    const Building& building,
    std::span<const StructuralMaterial> materials,
    const ChunkerSettings& settings
  )
    : m_building(building), m_materials(materials), m_settings(settings) {}

  ChunkedBuilding run() {
    validate(m_building, m_materials);

    const size_t members = m_building.members.size();

    m_counts.resize(members);
    m_grids.resize(members);
    m_firstChunk.assign(members + 1u, 0u);

    for (uint32_t member = 0u; member != members; ++member) {
      cutMember(member);
      m_firstChunk[member + 1u] = static_cast<uint32_t>(m_chunked.chunks.size());
    }

    for (uint32_t member = 0u; member != members; ++member) {
      bondInside(member);
    }

    for (uint32_t first = 0u; first != members; ++first) {
      for (uint32_t second = first + 1u; second != members; ++second) {
        bondBetween(first, second);
      }
    }

    std::sort(m_chunked.bonds.begin(), m_chunked.bonds.end(), [](const Bond& a, const Bond& b) {
      return a.first != b.first ? a.first < b.first : a.second < b.second;
    });

    return std::move(m_chunked);
  }

private:
  void cutMember(uint32_t index) {
    const Member& member = m_building.members[index];
    const StructuralMaterial& material = m_materials[member.material];
    glm::uvec3 count(1u);
    double cells = 1.0;

    for (int axis = 0; axis != 3; ++axis) {
      const double across = std::max(1.0, std::round(2.0 * member.halfExtents[axis] / material.chunkSize));

      cells *= across;

      if (!(cells + static_cast<double>(m_chunked.chunks.size()) < static_cast<double>(noChunk))) {
        throw std::length_error("A building makes more chunks than a structure can index");
      }

      count[axis] = static_cast<uint32_t>(across);
    }

    m_counts[index] = count;
    m_grids[index].assign(static_cast<size_t>(cells), noChunk);

    for (uint32_t k = 0u; k != count.z; ++k) {
      for (uint32_t j = 0u; j != count.y; ++j) {
        for (uint32_t i = 0u; i != count.x; ++i) {
          const Box cell = cellOf(member, count, glm::uvec3(i, j, k));

          if (member.role != Role::Pane && cut(cell.centre, m_building.openings)) {
            continue;
          }

          Chunk chunk;

          chunk.centre = cell.centre;
          chunk.halfExtents = cell.halfExtents;
          chunk.member = index;
          chunk.material = member.material;
          chunk.volume = 8.0 * cell.halfExtents.x * cell.halfExtents.y * cell.halfExtents.z;
          chunk.mass = material.density * chunk.volume;
          chunk.anchored = member.role == Role::Foundation
                           || std::abs(cell.centre.y - cell.halfExtents.y) <= touchTolerance * extent(cell);

          m_grids[index][cellIndex(count, glm::uvec3(i, j, k))] = static_cast<uint32_t>(m_chunked.chunks.size());
          m_chunked.chunks.push_back(chunk);
        }
      }
    }
  }

  void bondInside(uint32_t index) {
    const glm::uvec3 count = m_counts[index];
    const std::vector<uint32_t>& grid = m_grids[index];

    for (uint32_t k = 0u; k != count.z; ++k) {
      for (uint32_t j = 0u; j != count.y; ++j) {
        for (uint32_t i = 0u; i != count.x; ++i) {
          const glm::uvec3 cell(i, j, k);
          const uint32_t here = grid[cellIndex(count, cell)];

          if (here == noChunk) {
            continue;
          }

          for (int axis = 0; axis != 3; ++axis) {
            glm::uvec3 next = cell;

            next[axis] += 1u;

            if (next[axis] == count[axis]) {
              continue;
            }

            const uint32_t there = grid[cellIndex(count, next)];

            if (there != noChunk) {
              bond(here, there);
            }
          }
        }
      }
    }
  }

  void bondBetween(uint32_t first, uint32_t second) {
    const Box firstBox = boxOf(m_building.members[first]);
    const Box secondBox = boxOf(m_building.members[second]);

    if (!near(firstBox, secondBox)) {
      return;
    }

    const std::vector<uint32_t> firstSide = chunksNear(first, secondBox);
    const std::vector<uint32_t> secondSide = chunksNear(second, firstBox);

    for (const uint32_t a : firstSide) {
      for (const uint32_t b : secondSide) {
        if (overlapping(boxOf(m_chunked.chunks[a]), boxOf(m_chunked.chunks[b]))) {
          throw std::invalid_argument(
            "Chunks of building members " + std::to_string(first) + " and " + std::to_string(second)
            + " overlap"
          );
        }

        bond(a, b);
      }
    }
  }

  std::vector<uint32_t> chunksNear(uint32_t member, const Box& other) const {
    std::vector<uint32_t> found;

    for (uint32_t chunk = m_firstChunk[member]; chunk != m_firstChunk[member + 1u]; ++chunk) {
      if (near(boxOf(m_chunked.chunks[chunk]), other)) {
        found.push_back(chunk);
      }
    }

    return found;
  }

  void bond(uint32_t first, uint32_t second) {
    const std::optional<Bond> made = joined(
      first,
      second,
      m_chunked.chunks[first],
      m_chunked.chunks[second],
      m_building,
      m_materials,
      m_settings
    );

    if (made.has_value()) {
      m_chunked.bonds.push_back(*made);
    }
  }

  const Building& m_building;
  std::span<const StructuralMaterial> m_materials;
  const ChunkerSettings& m_settings;
  ChunkedBuilding m_chunked;
  std::vector<glm::uvec3> m_counts;
  std::vector<std::vector<uint32_t>> m_grids;
  std::vector<uint32_t> m_firstChunk;
};

}

double varietyFactor(uint32_t first, uint32_t second, double variation) {
  uint64_t bits = (static_cast<uint64_t>(first) << 32u) | second;

  bits += 0x9E3779B97F4A7C15ull;
  bits = (bits ^ (bits >> 30u)) * 0xBF58476D1CE4E5B9ull;
  bits = (bits ^ (bits >> 27u)) * 0x94D049BB133111EBull;
  bits ^= bits >> 31u;

  const double uniform = static_cast<double>(bits >> 11u) * 0x1.0p-53;

  return 1.0 + variation * rootThree * (2.0 * uniform - 1.0);
}

ChunkedBuilding chunkBuilding(
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const ChunkerSettings& settings
) {
  return Chunker(building, materials, settings).run();
}

double fractureStrength(const StructuralMaterial& material, const Member& member, const glm::dvec3& normal) {
  return sideLimits(material, member, normal).elastic.tension;
}

double fractureLoad(
  const StructuralMaterial& material,
  const Member& member,
  const glm::dvec3& halfExtents,
  const glm::dvec3& along
) {
  double load = 0.0;

  for (int axis = 0; axis != 3; ++axis) {
    glm::dvec3 normal(0.0);

    normal[axis] = 1.0;
    load += std::abs(along[axis]) * 4.0 * halfExtents[(axis + 1) % 3] * halfExtents[(axis + 2) % 3]
            * fractureStrength(material, member, normal);
  }

  return load;
}

glm::uvec3 finerCells(const StructuralMaterial& material, const Member& member) {
  glm::uvec3 cells(1u);

  switch (material.fracture) {
    case Fracture::None:
      break;
    case Fracture::Blocks:
      cells = glm::uvec3(material.pieces);
      break;
    case Fracture::Splinters:
      cells = glm::uvec3(material.pieces);
      cells[grainAxis(member)] = 1u;
      break;
    case Fracture::Tiles:
      cells = glm::uvec3(material.pieces);
      cells[thinnestAxis(member)] = 1u;
      break;
  }

  return cells;
}

FinerLevel finerLevel(
  const Building& building,
  const ChunkedBuilding& chunked,
  std::span<const StructuralMaterial> materials
) {
  FinerLevel level;

  level.cells.reserve(chunked.chunks.size());
  level.start.reserve(chunked.chunks.size() + 1u);
  level.start.push_back(0u);

  for (const Chunk& chunk : chunked.chunks) {
    if (chunk.material >= materials.size() || chunk.member >= building.members.size()) {
      throw std::out_of_range("A chunk names a material or a member the building does not have");
    }

    const glm::uvec3 cells = finerCells(materials[chunk.material], building.members[chunk.member]);
    const uint32_t count = cells.x * cells.y * cells.z;
    const uint32_t parts = count > 1u ? count : 0u;

    if (parts > 0xFFFFFFFFu - level.start.back()) {
      throw std::length_error("A building's finer level has more sub-boxes than a structure can index");
    }

    level.cells.push_back(cells);
    level.start.push_back(level.start.back() + parts);
  }

  return level;
}

std::vector<Chunk> finerChunks(
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const Chunk& chunk,
  const glm::uvec3& cells
) {
  std::vector<Chunk> parts;

  if (cells.x * cells.y * cells.z <= 1u) {
    return parts;
  }

  const Member whole{chunk.centre, chunk.halfExtents, chunk.material, building.members[chunk.member].role};
  const double density = materials[chunk.material].density;

  parts.reserve(static_cast<size_t>(cells.x) * cells.y * cells.z);

  for (uint32_t k = 0u; k != cells.z; ++k) {
    for (uint32_t j = 0u; j != cells.y; ++j) {
      for (uint32_t i = 0u; i != cells.x; ++i) {
        const Box cell = cellOf(whole, cells, glm::uvec3(i, j, k));
        Chunk part;

        part.centre = cell.centre;
        part.halfExtents = cell.halfExtents;
        part.member = chunk.member;
        part.material = chunk.material;
        part.volume = 8.0 * cell.halfExtents.x * cell.halfExtents.y * cell.halfExtents.z;
        part.mass = density * part.volume;
        part.anchored = whole.role == Role::Foundation
                        || std::abs(cell.centre.y - cell.halfExtents.y) <= touchTolerance * extent(cell);
        parts.push_back(part);
      }
    }
  }

  return parts;
}

std::vector<Bond> innerBonds(
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const ChunkerSettings& settings,
  std::span<const Chunk> parts,
  const glm::uvec3& cells,
  uint32_t firstId
) {
  if (parts.size() != static_cast<size_t>(cells.x) * cells.y * cells.z) {
    throw std::invalid_argument("A chunk's parts do not match its finer level");
  }

  std::vector<Bond> bonds;

  for (uint32_t k = 0u; k != cells.z; ++k) {
    for (uint32_t j = 0u; j != cells.y; ++j) {
      for (uint32_t i = 0u; i != cells.x; ++i) {
        const glm::uvec3 cell(i, j, k);
        const auto here = static_cast<uint32_t>(cellIndex(cells, cell));

        for (int axis = 0; axis != 3; ++axis) {
          glm::uvec3 next = cell;

          next[axis] += 1u;

          if (next[axis] == cells[axis]) {
            continue;
          }

          const auto there = static_cast<uint32_t>(cellIndex(cells, next));
          const std::optional<Bond> made =
            joined(firstId + here, firstId + there, parts[here], parts[there], building, materials, settings);

          if (made.has_value()) {
            bonds.push_back(*made);
          }
        }
      }
    }
  }

  return bonds;
}

std::vector<Bond> splitBond(
  const Bond& bond,
  uint32_t fractured,
  std::span<const Chunk> parts,
  uint32_t firstId
) {
  if (fractured != bond.first && fractured != bond.second) {
    throw std::invalid_argument("A bond can be split only over the parts of one of its two chunks");
  }

  int axis = 0;

  for (int candidate = 1; candidate != 3; ++candidate) {
    if (std::abs(bond.normal[candidate]) > std::abs(bond.normal[axis])) {
      axis = candidate;
    }
  }

  const int u = (axis + 1) % 3;
  const int v = (axis + 2) % 3;
  const uint32_t other = fractured == bond.first ? bond.second : bond.first;
  const double toward = (fractured == bond.first ? 1.0 : -1.0) * (bond.normal[axis] > 0.0 ? 1.0 : -1.0);
  const double plane = bond.centroid[axis];
  std::vector<Bond> split;

  for (uint32_t index = 0u; index != parts.size(); ++index) {
    const Chunk& part = parts[index];
    const double slack = touchTolerance * std::max(extent(boxOf(part)), std::max(bond.wu, bond.wv));
    const double face = part.centre[axis] + toward * part.halfExtents[axis];

    if (std::abs(face - plane) > slack) {
      continue;
    }

    const double lowU = std::max(bond.centroid[u] - 0.5 * bond.wu, part.centre[u] - part.halfExtents[u]);
    const double highU = std::min(bond.centroid[u] + 0.5 * bond.wu, part.centre[u] + part.halfExtents[u]);
    const double lowV = std::max(bond.centroid[v] - 0.5 * bond.wv, part.centre[v] - part.halfExtents[v]);
    const double highV = std::min(bond.centroid[v] + 0.5 * bond.wv, part.centre[v] + part.halfExtents[v]);

    if (!(highU - lowU > slack) || !(highV - lowV > slack)) {
      continue;
    }

    const uint32_t id = firstId + index;
    Bond piece = bond;

    piece.first = std::min(id, other);
    piece.second = std::max(id, other);
    piece.normal = glm::dvec3(0.0);
    piece.normal[axis] = id < other ? toward : -toward;
    piece.centroid[u] = 0.5 * (lowU + highU);
    piece.centroid[v] = 0.5 * (lowV + highV);
    piece.wu = highU - lowU;
    piece.wv = highV - lowV;
    piece.area = piece.wu * piece.wv;

    const double share = piece.area / bond.area;

    piece.elastic = ModeLimits{bond.elastic.compression * share, bond.elastic.tension * share, bond.elastic.shear * share};
    piece.fatal = ModeLimits{bond.fatal.compression * share, bond.fatal.tension * share, bond.fatal.shear * share};
    split.push_back(piece);
  }

  return split;
}

}
