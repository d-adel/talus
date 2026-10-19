#pragma once

#include <talus/destruction/chunker/chunker.h>

#include <cstdint>
#include <vector>

namespace talus::destruction {

inline constexpr uint32_t noComponent = 0xFFFFFFFFu;

struct StructureState {
  std::vector<uint8_t> alive;
  std::vector<uint32_t> component;
  std::vector<uint8_t> anchoredComponent;
  std::vector<double> health;
  std::vector<uint8_t> broken;
  std::vector<uint8_t> cracked;
  std::vector<uint32_t> bondStart;
  std::vector<uint32_t> bondList;
  std::vector<uint32_t> dirty;
  uint64_t revision = 0u;
};

[[nodiscard]] StructureState makeStructure(const ChunkedBuilding& building);

void breakBond(StructureState& state, const ChunkedBuilding& building, uint32_t bond);

void crackBond(StructureState& state, uint32_t bond);

void removeChunk(StructureState& state, const ChunkedBuilding& building, uint32_t chunk);

[[nodiscard]] std::vector<uint32_t> updateComponents(StructureState& state, const ChunkedBuilding& building);

[[nodiscard]] bool isAnchored(const StructureState& state, uint32_t component);

}
