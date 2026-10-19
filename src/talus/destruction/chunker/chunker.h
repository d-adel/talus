#pragma once

#include <talus/destruction/building/building.h>
#include <talus/destruction/materials/materials.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::destruction {

struct Chunk {
  glm::dvec3 centre{0.0};
  glm::dvec3 halfExtents{0.0};
  uint32_t member = 0u;
  uint32_t material = 0u;
  double volume = 0.0;
  double mass = 0.0;
  bool anchored = false;
};

struct Bond {
  uint32_t first = 0u;
  uint32_t second = 0u;
  glm::dvec3 centroid{0.0};
  glm::dvec3 normal{0.0};
  double area = 0.0;
  double wu = 0.0;
  double wv = 0.0;
  ModeLimits elastic;
  ModeLimits fatal;
  ShearRule shearRule = ShearRule::Fixed;
  double friction = 0.0;
  double crackFriction = 0.0;
};

struct ChunkedBuilding {
  std::vector<Chunk> chunks;
  std::vector<Bond> bonds;
};

struct ChunkerSettings {
  bool variety = true;
};

struct FinerLevel {
  std::vector<glm::uvec3> cells;
  std::vector<uint32_t> start;
};

[[nodiscard]] double varietyFactor(uint32_t first, uint32_t second, double variation);

[[nodiscard]] ChunkedBuilding chunkBuilding(
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const ChunkerSettings& settings = {}
);

[[nodiscard]] double fractureStrength(const StructuralMaterial& material, const Member& member, const glm::dvec3& normal);

[[nodiscard]] double fractureLoad(
  const StructuralMaterial& material,
  const Member& member,
  const glm::dvec3& halfExtents,
  const glm::dvec3& along
);

[[nodiscard]] glm::uvec3 finerCells(const StructuralMaterial& material, const Member& member);

[[nodiscard]] FinerLevel finerLevel(
  const Building& building,
  const ChunkedBuilding& chunked,
  std::span<const StructuralMaterial> materials
);

[[nodiscard]] std::vector<Chunk> finerChunks(
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const Chunk& chunk,
  const glm::uvec3& cells
);

[[nodiscard]] std::vector<Bond> innerBonds(
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const ChunkerSettings& settings,
  std::span<const Chunk> parts,
  const glm::uvec3& cells,
  uint32_t firstId
);

[[nodiscard]] std::vector<Bond> splitBond(
  const Bond& bond,
  uint32_t fractured,
  std::span<const Chunk> parts,
  uint32_t firstId
);

}
