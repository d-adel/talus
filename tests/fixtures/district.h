#pragma once

#include <talus/destruction/building/building.h>
#include <talus/destruction/materials/materials.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::destruction {

enum class BuildingType : uint8_t { ConcreteFrame, TimberHouse, GlassOffice, SteelWarehouse };

enum class DistrictMaterial : uint32_t { Concrete, Brick, Timber, Glass, Steel };

enum class Surface : uint8_t { Street, Pavement };

enum class PropKind : uint8_t { Crate, Barrel, Bollard, Pallet, Car };

inline constexpr uint32_t buildingTypes = 4u;
inline constexpr uint32_t districtMaterialCount = 5u;
inline constexpr uint32_t districtPlots = 24u;
inline constexpr uint32_t districtBuildings = 20u;

struct BuildingPlan {
  BuildingType type = BuildingType::ConcreteFrame;
  uint32_t storeys = 1u;
  uint32_t baysX = 1u;
  uint32_t baysZ = 1u;

  friend bool operator==(const BuildingPlan&, const BuildingPlan&) = default;
};

struct GroundBox {
  glm::dvec3 centre{0.0};
  glm::dvec3 halfExtents{0.0};
  Surface surface = Surface::Street;
};

struct Prop {
  glm::dvec3 centre{0.0};
  glm::dvec3 halfExtents{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  double density = 0.0;
  PropKind kind = PropKind::Crate;
};

struct DistrictCounts {
  uint32_t buildings = 16u;
  uint32_t props = 3000u;
};

struct District {
  std::vector<StructuralMaterial> materials;
  std::vector<BuildingPlan> plans;
  std::vector<Building> buildings;
  std::vector<GroundBox> ground;
  std::vector<Prop> props;
};

[[nodiscard]] std::vector<StructuralMaterial> districtMaterials();

[[nodiscard]] std::span<const BuildingPlan> buildingPlans(BuildingType type);

[[nodiscard]] Building makeBuilding(const BuildingPlan& plan);

[[nodiscard]] uint32_t propCapacity(uint32_t buildings);

[[nodiscard]] District makeDistrict(uint64_t seed, const DistrictCounts& counts = {});

}
