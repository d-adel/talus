#include "district.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>

namespace talus::destruction {

namespace {

enum class Infill : uint8_t { None, Walls, Glass };

enum class Joints : uint8_t { Monolithic, Stacked };

enum class Spot : uint8_t { Sidewalk, Parking, Alley, Square };

struct Style {
  double bay = 0.0;
  double storey = 0.0;
  double column = 0.0;
  double slab = 0.0;
  double beamWidth = 0.0;
  double beamDepth = 0.0;
  double wall = 0.0;
  DistrictMaterial columns = DistrictMaterial::Concrete;
  DistrictMaterial beams = DistrictMaterial::Concrete;
  DistrictMaterial slabs = DistrictMaterial::Concrete;
  DistrictMaterial walls = DistrictMaterial::Brick;
  Joints joints = Joints::Monolithic;
  Infill exterior = Infill::None;
  bool interior = false;
  bool jointed = false;
  bool piers = false;
  bool mullions = false;
  bool oneHead = false;
  uint32_t wallEvery = 1u;
  uint32_t windowCells = 0u;
  uint32_t windowRows = 0u;
  uint32_t sillRows = 0u;
  uint32_t doorCells = 0u;
  uint32_t doorRows = 0u;
};

struct Place {
  glm::dvec2 at{0.0};
  bool alongZ = false;
  Spot spot = Spot::Sidewalk;
};

struct PropShape {
  glm::dvec3 halfExtents{0.0};
  double mass = 0.0;
};

constexpr double joint = 0.01;
constexpr double glassHalf = 0.01;
constexpr double doorWidth = 1.5;
constexpr double bendingRatio = 0.02;
constexpr double barRatio = 0.005;
constexpr double glassChunk = 0.5;

constexpr double plotX = 26.0;
constexpr double plotZ = 20.0;
constexpr double alley = 3.0;
constexpr double street = 14.0;
constexpr double sidewalk = 2.5;
constexpr double kerb = 0.15;
constexpr uint32_t blocksX = 3u;
constexpr uint32_t blocksZ = 2u;
constexpr double blockX = 2.0 * plotX + alley;
constexpr double blockZ = 2.0 * plotZ + alley;
constexpr double pitchX = blockX + street;
constexpr double pitchZ = blockZ + street;
constexpr double placePitch = 1.25;
constexpr double carPitch = 6.0;
constexpr double carInset = 6.0;
constexpr double carKerb = 1.1;
constexpr double squarePitch = 2.0;
constexpr std::array<double, 2> sidewalkRows{0.7, 1.7};

constexpr uint64_t plotStream = 1u;
constexpr uint64_t typeStream = 2u;
constexpr uint64_t planStream = 3u;
constexpr uint64_t propStream = 4u;
constexpr uint64_t kindStream = 5u;
constexpr uint64_t yawStream = 6u;

constexpr std::array<BuildingPlan, 4> framePlans{{
  {BuildingType::ConcreteFrame, 3u, 2u, 2u},
  {BuildingType::ConcreteFrame, 4u, 2u, 2u},
  {BuildingType::ConcreteFrame, 2u, 3u, 2u},
  {BuildingType::ConcreteFrame, 3u, 3u, 2u},
}};

constexpr std::array<BuildingPlan, 4> housePlans{{
  {BuildingType::TimberHouse, 2u, 4u, 4u},
  {BuildingType::TimberHouse, 3u, 4u, 4u},
  {BuildingType::TimberHouse, 2u, 6u, 4u},
  {BuildingType::TimberHouse, 3u, 5u, 4u},
}};

constexpr std::array<BuildingPlan, 4> officePlans{{
  {BuildingType::GlassOffice, 5u, 2u, 1u},
  {BuildingType::GlassOffice, 6u, 2u, 1u},
  {BuildingType::GlassOffice, 3u, 2u, 2u},
  {BuildingType::GlassOffice, 4u, 2u, 2u},
}};

constexpr std::array<BuildingPlan, 2> warehousePlans{{
  {BuildingType::SteelWarehouse, 1u, 4u, 3u},
  {BuildingType::SteelWarehouse, 1u, 5u, 3u},
}};

Style styleOf(BuildingType type) {
  switch (type) {
    case BuildingType::ConcreteFrame:
      return Style{
        .bay = 4.0, .storey = 3.25, .column = 0.5, .slab = 0.25, .beamWidth = 0.3, .beamDepth = 0.5, .wall = 0.25,
        .columns = DistrictMaterial::Concrete, .beams = DistrictMaterial::Concrete, .slabs = DistrictMaterial::Concrete,
        .walls = DistrictMaterial::Brick, .joints = Joints::Monolithic, .exterior = Infill::Walls, .interior = false,
        .jointed = true, .piers = true, .windowCells = 2u, .windowRows = 0u, .sillRows = 0u, .doorCells = 3u,
        .doorRows = 0u
      };
    case BuildingType::TimberHouse:
      return Style{
        .bay = 2.1, .storey = 2.7, .column = 0.2, .slab = 0.2, .beamWidth = 0.2, .beamDepth = 0.3, .wall = 0.2,
        .columns = DistrictMaterial::Timber, .beams = DistrictMaterial::Timber, .slabs = DistrictMaterial::Timber,
        .walls = DistrictMaterial::Timber, .joints = Joints::Monolithic, .exterior = Infill::Walls, .interior = true,
        .jointed = true, .mullions = true, .oneHead = true, .wallEvery = 2u, .windowCells = 2u, .windowRows = 2u,
        .sillRows = 2u, .doorCells = 2u, .doorRows = 4u
      };
    case BuildingType::GlassOffice:
      return Style{
        .bay = 4.0, .storey = 3.75, .column = 0.5, .slab = 0.25, .beamWidth = 0.3, .beamDepth = 0.5, .wall = 0.0,
        .columns = DistrictMaterial::Concrete, .beams = DistrictMaterial::Concrete, .slabs = DistrictMaterial::Concrete,
        .walls = DistrictMaterial::Glass, .joints = Joints::Monolithic, .exterior = Infill::Glass, .interior = false,
        .jointed = false, .windowCells = 0u, .windowRows = 0u, .sillRows = 0u, .doorCells = 0u, .doorRows = 0u
      };
    case BuildingType::SteelWarehouse:
      return Style{
        .bay = 5.0, .storey = 6.65, .column = 0.3, .slab = 0.25, .beamWidth = 0.2, .beamDepth = 0.4, .wall = 0.0,
        .columns = DistrictMaterial::Steel, .beams = DistrictMaterial::Steel, .slabs = DistrictMaterial::Concrete,
        .walls = DistrictMaterial::Steel, .joints = Joints::Stacked, .exterior = Infill::None, .interior = false,
        .jointed = false, .windowCells = 0u, .windowRows = 0u, .sillRows = 0u, .doorCells = 0u, .doorRows = 0u
      };
  }

  throw std::invalid_argument("A building type the district does not know");
}

uint32_t indexOf(DistrictMaterial material) {
  return static_cast<uint32_t>(material);
}

double boundary(double centre, double half, uint32_t count, uint32_t at) {
  return centre + half * ((2.0 * at - count) / count);
}

uint32_t cellsAcross(double size, double chunk) {
  return static_cast<uint32_t>(std::max(1.0, std::round(size / chunk)));
}

class Builder {
public:
  explicit Builder(const BuildingPlan& plan)
    : m_plan(plan),
      m_style(styleOf(plan.type)),
      m_wallChunk(districtMaterials()[indexOf(m_style.walls)].chunkSize) {}

  [[nodiscard]] Building run() {
    for (uint32_t storey = 0u; storey != m_plan.storeys; ++storey) {
      const double base = storey * m_style.storey;
      const double top = (storey + 1u) * m_style.storey - m_style.slab;
      const double under = top - m_style.beamDepth;
      const double roof = top + m_style.slab;

      addColumns(base, under, top, roof);
      addBeams(under, top);
      addInfill(storey, base, under);
      addSlab(top, roof);
    }

    return std::move(m_building);
  }

private:
  [[nodiscard]] double gridX(uint32_t line) const {
    return (static_cast<double>(line) - 0.5 * m_plan.baysX) * m_style.bay;
  }

  [[nodiscard]] double gridZ(uint32_t line) const {
    return (static_cast<double>(line) - 0.5 * m_plan.baysZ) * m_style.bay;
  }

  [[nodiscard]] double half(uint32_t bays) const {
    return 0.5 * bays * m_style.bay + 0.5 * m_style.column;
  }

  [[nodiscard]] static glm::dvec3 point(int along, double a, double y, double across) {
    glm::dvec3 result(0.0);

    result[along] = a;
    result.y = y;
    result[2 - along] = across;

    return result;
  }

  void add(const glm::dvec3& low, const glm::dvec3& high, DistrictMaterial material, Role role) {
    m_building.members.push_back(Member{0.5 * (low + high), 0.5 * (high - low), indexOf(material), role});
  }

  void addColumn(uint32_t i, uint32_t j, double width, double from, double to) {
    const double h = 0.5 * width;

    add(glm::dvec3(gridX(i) - h, from, gridZ(j) - h), glm::dvec3(gridX(i) + h, to, gridZ(j) + h), m_style.columns,
        Role::Column);
  }

  void addColumns(double base, double under, double top, double roof) {
    const double shaft = m_style.jointed ? m_style.column - 2.0 * joint : m_style.column;

    for (uint32_t j = 0u; j <= m_plan.baysZ; ++j) {
      for (uint32_t i = 0u; i <= m_plan.baysX; ++i) {
        switch (m_style.joints) {
          case Joints::Monolithic:
            addColumn(i, j, shaft, base, under);

            if (m_style.oneHead) {
              addColumn(i, j, m_style.column, under, roof);
            } else {
              addColumn(i, j, m_style.column, under, top);
              addColumn(i, j, m_style.column, top, roof);
            }

            break;
          case Joints::Stacked:
            addColumn(i, j, m_style.column, base, under);
            break;
        }
      }
    }
  }

  void addBeams(double under, double top) {
    if (!(m_style.beamDepth > 0.0)) {
      return;
    }

    const double b = 0.5 * m_style.beamWidth;
    const double c = 0.5 * m_style.column;
    const bool stacked = m_style.joints == Joints::Stacked;
    const double end = stacked ? b : c;

    for (uint32_t j = 0u; j <= m_plan.baysZ; ++j) {
      if (stacked) {
        add(glm::dvec3(-half(m_plan.baysX), under, gridZ(j) - b), glm::dvec3(half(m_plan.baysX), top, gridZ(j) + b),
            m_style.beams, Role::Beam);
        continue;
      }

      for (uint32_t i = 0u; i != m_plan.baysX; ++i) {
        add(glm::dvec3(gridX(i) + c, under, gridZ(j) - b), glm::dvec3(gridX(i + 1u) - c, top, gridZ(j) + b),
            m_style.beams, Role::Beam);
      }
    }

    for (uint32_t i = 0u; i <= m_plan.baysX; ++i) {
      for (uint32_t j = 0u; j != m_plan.baysZ; ++j) {
        add(glm::dvec3(gridX(i) - b, under, gridZ(j) + end), glm::dvec3(gridX(i) + b, top, gridZ(j + 1u) - end),
            m_style.beams, Role::Beam);
      }
    }
  }

  void addInfill(uint32_t storey, double base, double under) {
    const double c = 0.5 * m_style.column;

    for (uint32_t j = 0u; j <= m_plan.baysZ; ++j) {
      const bool outside = j == 0u || j == m_plan.baysZ;

      if (!outside && j % m_style.wallEvery != 0u) {
        continue;
      }

      for (uint32_t i = 0u; i != m_plan.baysX; ++i) {
        const bool door = outside ? storey == 0u && j == 0u && i == m_plan.baysX / 2u : i % m_style.wallEvery == 0u;

        addPanel(0, gridX(i) + c, gridX(i + 1u) - c, gridZ(j), base, under, outside, door);
      }
    }

    for (uint32_t i = 0u; i <= m_plan.baysX; ++i) {
      const bool outside = i == 0u || i == m_plan.baysX;

      if (!outside && i % m_style.wallEvery != 0u) {
        continue;
      }

      for (uint32_t j = 0u; j != m_plan.baysZ; ++j) {
        addPanel(2, gridZ(j) + c, gridZ(j + 1u) - c, gridX(i), base, under, outside, !outside && j % m_style.wallEvery == 0u);
      }
    }
  }

  void addPane(int along, double from, double to, double line, double bottom, double head) {
    add(point(along, from + joint, bottom, line - glassHalf), point(along, to - joint, head - joint, line + glassHalf),
        DistrictMaterial::Glass, Role::Pane);
  }

  void addPanel(int along, double from, double to, double line, double base, double under, bool outside, bool door) {
    const Infill infill = outside ? m_style.exterior : (m_style.interior ? Infill::Walls : Infill::None);

    if (infill == Infill::None) {
      return;
    }

    if (infill == Infill::Glass) {
      const double middle = 0.5 * (from + to);

      if (door) {
        addPane(along, from, middle - 0.5 * doorWidth, line, base, under);
        addPane(along, middle + 0.5 * doorWidth, to, line, base, under);
      } else {
        addPane(along, from, to, line, base, under);
      }

      return;
    }

    const double t = 0.5 * m_style.wall;
    const double crown = under - joint;
    const uint32_t cells = cellsAcross(to - from, m_wallChunk);
    const uint32_t rows = cellsAcross(crown - base, m_wallChunk);

    add(point(along, from, base, line - t), point(along, to, crown, line + t), m_style.walls, Role::Wall);

    if (m_style.piers) {
      if (door) {
        const uint32_t side = (cells - 2u - m_style.doorCells) / 2u;

        addOpening(along, line, from, to, base, crown, 1u, 1u + side, 0u, rows, true);
        addOpening(along, line, from, to, base, crown, 1u + side, cells - 1u - side, 0u, rows, false);
        addOpening(along, line, from, to, base, crown, cells - 1u - side, cells - 1u, 0u, rows, true);

        return;
      }

      for (uint32_t first = 1u; first + m_style.windowCells < cells; first += m_style.windowCells + 1u) {
        addOpening(along, line, from, to, base, crown, first, first + m_style.windowCells, 0u, rows, true);
      }

      return;
    }

    const uint32_t wide = door ? m_style.doorCells : m_style.windowCells;
    const uint32_t tall = door ? m_style.doorRows : m_style.windowRows;
    const uint32_t sill = door ? 0u : m_style.sillRows;

    if ((!outside && !door) || wide == 0u || wide > cells || sill + tall > rows) {
      return;
    }

    const uint32_t start = (cells - wide) / 2u;

    addOpening(along, line, from, to, base, crown, start, start + wide, sill, sill + tall, !door);
  }

  void addOpening(
    int along,
    double line,
    double from,
    double to,
    double base,
    double crown,
    uint32_t first,
    uint32_t last,
    uint32_t lowest,
    uint32_t highest,
    bool glazed
  ) {
    const double t = 0.5 * m_style.wall;
    const uint32_t cells = cellsAcross(to - from, m_wallChunk);
    const uint32_t rows = cellsAcross(crown - base, m_wallChunk);
    const double left = boundary(0.5 * (from + to), 0.5 * (to - from), cells, first);
    const double right = boundary(0.5 * (from + to), 0.5 * (to - from), cells, last);
    const double bottom = boundary(0.5 * (base + crown), 0.5 * (crown - base), rows, lowest);
    const double head = boundary(0.5 * (base + crown), 0.5 * (crown - base), rows, highest);
    const glm::dvec3 low = point(along, left, bottom, line - t);
    const glm::dvec3 high = point(along, right, head, line + t);

    m_building.openings.push_back(Opening{0.5 * (low + high), 0.5 * (high - low)});

    if (!glazed) {
      return;
    }

    if (!m_style.mullions) {
      addPane(along, left, right, line, bottom, head);

      return;
    }

    for (uint32_t cell = first; cell != last; ++cell) {
      addPane(
        along,
        boundary(0.5 * (from + to), 0.5 * (to - from), cells, cell),
        boundary(0.5 * (from + to), 0.5 * (to - from), cells, cell + 1u),
        line,
        bottom,
        head
      );
    }
  }

  void addSlab(double top, double roof) {
    if (m_style.joints != Joints::Monolithic) {
      add(glm::dvec3(-half(m_plan.baysX), top, -half(m_plan.baysZ)), glm::dvec3(half(m_plan.baysX), roof, half(m_plan.baysZ)),
          m_style.slabs, Role::Slab);

      return;
    }

    const double c = 0.5 * m_style.column;

    for (uint32_t j = 0u; j <= 2u * m_plan.baysZ; ++j) {
      const double zLow = j % 2u == 0u ? gridZ(j / 2u) - c : gridZ(j / 2u) + c;
      const double zHigh = j % 2u == 0u ? gridZ(j / 2u) + c : gridZ(j / 2u + 1u) - c;

      for (uint32_t i = 0u; i <= 2u * m_plan.baysX; ++i) {
        if (i % 2u == 0u && j % 2u == 0u) {
          continue;
        }

        const double xLow = i % 2u == 0u ? gridX(i / 2u) - c : gridX(i / 2u) + c;
        const double xHigh = i % 2u == 0u ? gridX(i / 2u) + c : gridX(i / 2u + 1u) - c;

        add(glm::dvec3(xLow, top, zLow), glm::dvec3(xHigh, roof, zHigh), m_style.slabs, Role::Slab);
      }
    }
  }

  BuildingPlan m_plan;
  Style m_style;
  double m_wallChunk = 0.0;
  Building m_building;
};

uint64_t mix(uint64_t bits) {
  bits += 0x9E3779B97F4A7C15ull;
  bits = (bits ^ (bits >> 30u)) * 0xBF58476D1CE4E5B9ull;
  bits = (bits ^ (bits >> 27u)) * 0x94D049BB133111EBull;

  return bits ^ (bits >> 31u);
}

uint64_t draw(uint64_t seed, uint64_t stream, uint64_t at) {
  return mix(mix(mix(seed) ^ stream) ^ at);
}

template <size_t Count>
std::array<uint32_t, Count> shuffled(uint64_t seed, uint64_t stream) {
  std::array<uint32_t, Count> order{};

  for (uint32_t at = 0u; at != Count; ++at) {
    order[at] = at;
  }

  std::sort(order.begin(), order.end(), [seed, stream](uint32_t a, uint32_t b) {
    const uint64_t first = draw(seed, stream, a);
    const uint64_t second = draw(seed, stream, b);

    return first != second ? first < second : a < b;
  });

  return order;
}

glm::dvec2 blockCentre(uint32_t block) {
  return glm::dvec2(
    (static_cast<double>(block % blocksX) - 0.5 * (blocksX - 1u)) * pitchX,
    (static_cast<double>(block / blocksX) - 0.5 * (blocksZ - 1u)) * pitchZ
  );
}

glm::dvec2 plotCentre(uint32_t plot) {
  return blockCentre(plot / 4u)
         + glm::dvec2((static_cast<double>(plot % 2u) - 0.5) * (plotX + alley),
                      (static_cast<double>((plot / 2u) % 2u) - 0.5) * (plotZ + alley));
}

glm::dvec2 footprintOf(const BuildingPlan& plan) {
  const Style style = styleOf(plan.type);

  return glm::dvec2(plan.baysX * style.bay + style.column, plan.baysZ * style.bay + style.column);
}

void streetPlaces(uint32_t block, std::vector<Place>& places) {
  const glm::dvec2 centre = blockCentre(block);
  const double halfX = 0.5 * blockX;
  const double halfZ = 0.5 * blockZ;
  const auto alongX = static_cast<uint32_t>((blockX + 2.0 * sidewalk) / placePitch);
  const auto alongZ = static_cast<uint32_t>(blockZ / placePitch);
  const auto carsX = static_cast<uint32_t>((blockX + 2.0 * sidewalk - 2.0 * carInset) / carPitch) + 1u;
  const auto carsZ = static_cast<uint32_t>((blockZ + 2.0 * sidewalk - 2.0 * carInset) / carPitch) + 1u;
  const auto alleyX = static_cast<uint32_t>(plotX / placePitch);
  const auto alleyZ = static_cast<uint32_t>(plotZ / placePitch);

  for (const double row : sidewalkRows) {
    for (const double side : {-1.0, 1.0}) {
      for (uint32_t k = 0u; k != alongX; ++k) {
        places.push_back(Place{centre + glm::dvec2(-halfX - sidewalk + (k + 0.5) * placePitch, side * (halfZ + row)),
                               false, Spot::Sidewalk});
      }

      for (uint32_t k = 0u; k != alongZ; ++k) {
        places.push_back(Place{centre + glm::dvec2(side * (halfX + row), -halfZ + (k + 0.5) * placePitch), true,
                               Spot::Sidewalk});
      }
    }
  }

  for (const double side : {-1.0, 1.0}) {
    for (uint32_t k = 0u; k != carsX; ++k) {
      places.push_back(Place{centre + glm::dvec2(-halfX - sidewalk + carInset + k * carPitch,
                                                 side * (halfZ + sidewalk + carKerb)),
                             false, Spot::Parking});
    }

    for (uint32_t k = 0u; k != carsZ; ++k) {
      places.push_back(Place{centre + glm::dvec2(side * (halfX + sidewalk + carKerb),
                                                 -halfZ - sidewalk + carInset + k * carPitch),
                             true, Spot::Parking});
    }
  }

  for (const double side : {-1.0, 1.0}) {
    for (uint32_t k = 0u; k != alleyX; ++k) {
      places.push_back(Place{centre + glm::dvec2(side * (0.5 * alley + (k + 0.5) * placePitch), 0.0), false, Spot::Alley});
    }

    for (uint32_t k = 0u; k != alleyZ; ++k) {
      places.push_back(Place{centre + glm::dvec2(0.0, side * (0.5 * alley + (k + 0.5) * placePitch)), true, Spot::Alley});
    }
  }
}

void squarePlaces(uint32_t plot, std::vector<Place>& places) {
  const glm::dvec2 centre = plotCentre(plot);
  const auto acrossX = static_cast<uint32_t>(plotX / squarePitch);
  const auto acrossZ = static_cast<uint32_t>(plotZ / squarePitch);

  for (uint32_t b = 0u; b != acrossZ; ++b) {
    for (uint32_t a = 0u; a != acrossX; ++a) {
      places.push_back(Place{centre + glm::dvec2((a + 0.5 - 0.5 * acrossX) * squarePitch,
                                                 (b + 0.5 - 0.5 * acrossZ) * squarePitch),
                             false, Spot::Square});
    }
  }
}

uint32_t placesIn(void (*fill)(uint32_t, std::vector<Place>&)) {
  std::vector<Place> places;

  fill(0u, places);

  return static_cast<uint32_t>(places.size());
}

PropKind kindAt(Spot spot, uint64_t bits) {
  switch (spot) {
    case Spot::Sidewalk: {
      const uint64_t pick = bits % 12u;

      return pick < 4u ? PropKind::Crate
                       : (pick < 7u ? PropKind::Barrel : (pick < 10u ? PropKind::Bollard : PropKind::Pallet));
    }
    case Spot::Parking:
      return PropKind::Car;
    case Spot::Alley:
      return bits % 2u == 0u ? PropKind::Crate : PropKind::Barrel;
    case Spot::Square:
      return bits % 5u < 3u ? PropKind::Crate : PropKind::Pallet;
  }

  throw std::invalid_argument("A prop place the district does not know");
}

PropShape shapeOf(PropKind kind) {
  switch (kind) {
    case PropKind::Crate:
      return PropShape{glm::dvec3(0.3, 0.3, 0.3), 25.0};
    case PropKind::Barrel:
      return PropShape{glm::dvec3(0.3, 0.45, 0.3), 40.0};
    case PropKind::Bollard:
      return PropShape{glm::dvec3(0.15, 0.45, 0.15), 194.4};
    case PropKind::Pallet:
      return PropShape{glm::dvec3(0.5, 0.45, 0.4), 1000.0};
    case PropKind::Car:
      return PropShape{glm::dvec3(2.2, 0.75, 0.9), 1300.0};
  }

  throw std::invalid_argument("A prop kind the district does not know");
}

}

std::vector<StructuralMaterial> districtMaterials() {
  const StructuralMaterial bars = steel();
  StructuralMaterial concrete = reinforcedConcrete(bendingRatio);
  StructuralMaterial pane = glass();

  concrete.elastic.shear += barRatio * bars.elastic.tension * concrete.friction;
  concrete.fatal.shear += barRatio * bars.fatal.tension * concrete.friction;
  pane.chunkSize = glassChunk;

  return {concrete, brickMasonry(), timber(), pane, steel()};
}

std::span<const BuildingPlan> buildingPlans(BuildingType type) {
  switch (type) {
    case BuildingType::ConcreteFrame:
      return framePlans;
    case BuildingType::TimberHouse:
      return housePlans;
    case BuildingType::GlassOffice:
      return officePlans;
    case BuildingType::SteelWarehouse:
      return warehousePlans;
  }

  return {};
}

Building makeBuilding(const BuildingPlan& plan) {
  const std::span<const BuildingPlan> plans = buildingPlans(plan.type);

  if (std::find(plans.begin(), plans.end(), plan) == plans.end()) {
    throw std::invalid_argument("A building plan the district does not make");
  }

  return Builder(plan).run();
}

uint32_t propCapacity(uint32_t buildings) {
  if (buildings > districtBuildings) {
    throw std::invalid_argument("A district holds at most " + std::to_string(districtBuildings) + " buildings");
  }

  return blocksX * blocksZ * placesIn(&streetPlaces) + (districtPlots - buildings) * placesIn(&squarePlaces);
}

District makeDistrict(uint64_t seed, const DistrictCounts& counts) {
  const uint32_t capacity = propCapacity(counts.buildings);

  if (counts.props > capacity) {
    throw std::invalid_argument(
      "A district of " + std::to_string(counts.buildings) + " buildings has room for at most "
      + std::to_string(capacity) + " props"
    );
  }

  const std::array<uint32_t, districtPlots> plots = shuffled<districtPlots>(seed, plotStream);
  const auto offset = static_cast<uint32_t>(draw(seed, typeStream, 0u) % buildingTypes);
  District district;

  district.materials = districtMaterials();

  for (uint32_t built = 0u; built != counts.buildings; ++built) {
    const auto type = static_cast<BuildingType>((built + offset) % buildingTypes);
    const std::span<const BuildingPlan> plans = buildingPlans(type);
    const BuildingPlan plan = plans[draw(seed, planStream, built) % plans.size()];
    const uint32_t plot = plots[built];
    const glm::dvec2 block = blockCentre(plot / 4u);
    const glm::dvec2 footprint = footprintOf(plan);
    const double across = plot % 2u == 1u ? 1.0 : -1.0;
    const bool back = (plot / 2u) % 2u == 1u;
    Building building = makeBuilding(plan);

    building.position = glm::dvec3(
      block.x + across * 0.5 * (alley + footprint.x),
      kerb,
      block.y + (back ? 1.0 : -1.0) * 0.5 * (alley + footprint.y)
    );
    building.rotation = back ? glm::dquat(0.0, 0.0, 1.0, 0.0) : glm::dquat(1.0, 0.0, 0.0, 0.0);
    district.plans.push_back(plan);
    district.buildings.push_back(std::move(building));
  }

  district.ground.push_back(GroundBox{
    glm::dvec3(0.0, -0.5, 0.0),
    glm::dvec3(0.5 * (blocksX * pitchX + street), 0.5, 0.5 * (blocksZ * pitchZ + street)),
    Surface::Street
  });

  for (uint32_t block = 0u; block != blocksX * blocksZ; ++block) {
    const glm::dvec2 centre = blockCentre(block);

    district.ground.push_back(GroundBox{
      glm::dvec3(centre.x, 0.5 * kerb, centre.y),
      glm::dvec3(0.5 * blockX + sidewalk, 0.5 * kerb, 0.5 * blockZ + sidewalk),
      Surface::Pavement
    });
  }

  std::vector<Place> places;

  for (uint32_t block = 0u; block != blocksX * blocksZ; ++block) {
    streetPlaces(block, places);
  }

  std::array<uint32_t, districtPlots> open = plots;

  std::sort(open.begin() + counts.buildings, open.end());

  for (auto at = open.begin() + counts.buildings; at != open.end(); ++at) {
    squarePlaces(*at, places);
  }

  std::vector<uint32_t> chosen(places.size());

  for (uint32_t at = 0u; at != chosen.size(); ++at) {
    chosen[at] = at;
  }

  std::sort(chosen.begin(), chosen.end(), [seed](uint32_t a, uint32_t b) {
    const uint64_t first = draw(seed, propStream, a);
    const uint64_t second = draw(seed, propStream, b);

    return first != second ? first < second : a < b;
  });

  chosen.resize(counts.props);
  std::sort(chosen.begin(), chosen.end());

  for (const uint32_t at : chosen) {
    const Place& place = places[at];
    const PropKind kind = kindAt(place.spot, draw(seed, kindStream, at));
    const PropShape shape = shapeOf(kind);
    const uint64_t turn = draw(seed, yawStream, at);
    const double tilt = kind == PropKind::Car ? (static_cast<double>(turn % 3u) - 1.0) * 0.01
                                              : (static_cast<double>(turn % 11u) - 5.0) * 0.01;
    const double norm = std::sqrt(1.0 + tilt * tilt);
    Prop prop;

    prop.halfExtents = place.alongZ ? glm::dvec3(shape.halfExtents.z, shape.halfExtents.y, shape.halfExtents.x)
                                    : shape.halfExtents;
    prop.centre = glm::dvec3(place.at.x, (place.spot == Spot::Parking ? 0.0 : kerb) + prop.halfExtents.y, place.at.y);
    prop.rotation = glm::dquat(1.0 / norm, 0.0, tilt / norm, 0.0);
    prop.density = shape.mass / (8.0 * prop.halfExtents.x * prop.halfExtents.y * prop.halfExtents.z);
    prop.kind = kind;
    district.props.push_back(prop);
  }

  return district;
}

}
