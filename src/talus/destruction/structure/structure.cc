#include "structure.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace talus::destruction {

namespace {

uint32_t findRoot(std::vector<uint32_t>& parent, uint32_t node) {
  while (parent[node] != node) {
    parent[node] = parent[parent[node]];
    node = parent[node];
  }

  return node;
}

}

StructureState makeStructure(const ChunkedBuilding& building) {
  const size_t chunks = building.chunks.size();
  const size_t bonds = building.bonds.size();
  StructureState state;

  for (const Bond& bond : building.bonds) {
    if (!(bond.first < bond.second) || bond.second >= chunks) {
      throw std::invalid_argument("A structure's bond needs two chunks it holds, the first below the second");
    }
  }

  state.alive.assign(chunks, 1u);
  state.component.assign(chunks, 0u);
  state.anchoredComponent.assign(chunks, 0u);
  state.health.assign(bonds, 1.0);
  state.broken.assign(bonds, 0u);
  state.cracked.assign(bonds, 0u);
  state.bondStart.assign(chunks + 1u, 0u);
  state.bondList.resize(2u * bonds);

  for (const Bond& bond : building.bonds) {
    state.bondStart[bond.first + 1u] += 1u;
    state.bondStart[bond.second + 1u] += 1u;
  }

  for (size_t chunk = 0u; chunk != chunks; ++chunk) {
    state.bondStart[chunk + 1u] += state.bondStart[chunk];
  }

  std::vector<uint32_t> cursor(state.bondStart.begin(), state.bondStart.end() - 1);

  for (uint32_t index = 0u; index != bonds; ++index) {
    state.bondList[cursor[building.bonds[index].first]++] = index;
    state.bondList[cursor[building.bonds[index].second]++] = index;
  }

  if (chunks != 0u) {
    state.dirty.push_back(0u);
    static_cast<void>(updateComponents(state, building));
  }

  return state;
}

void breakBond(StructureState& state, const ChunkedBuilding& building, uint32_t bond) {
  if (bond >= state.broken.size()) {
    throw std::out_of_range("A structure has no bond with that index");
  }

  if (state.broken[bond] != 0u) {
    return;
  }

  state.broken[bond] = 1u;
  state.health[bond] = 0.0;
  state.dirty.push_back(state.component[building.bonds[bond].first]);
  state.revision += 1u;
}

void crackBond(StructureState& state, uint32_t bond) {
  if (bond >= state.cracked.size()) {
    throw std::out_of_range("A structure has no bond with that index");
  }

  if (state.broken[bond] != 0u || state.cracked[bond] != 0u) {
    return;
  }

  state.cracked[bond] = 1u;
  state.revision += 1u;
}

void removeChunk(StructureState& state, const ChunkedBuilding& building, uint32_t chunk) {
  if (chunk >= state.alive.size()) {
    throw std::out_of_range("A structure has no chunk with that index");
  }

  if (state.alive[chunk] == 0u) {
    return;
  }

  for (uint32_t slot = state.bondStart[chunk]; slot != state.bondStart[chunk + 1u]; ++slot) {
    breakBond(state, building, state.bondList[slot]);
  }

  state.alive[chunk] = 0u;
  state.dirty.push_back(state.component[chunk]);
  state.revision += 1u;
}

std::vector<uint32_t> updateComponents(StructureState& state, const ChunkedBuilding& building) {
  std::vector<uint32_t> changed;

  if (state.dirty.empty()) {
    return changed;
  }

  const size_t chunks = state.alive.size();
  std::vector<uint8_t> marked(chunks, 0u);
  std::vector<uint32_t> members;

  for (const uint32_t id : state.dirty) {
    marked[id] = 1u;
  }

  state.dirty.clear();

  for (uint32_t chunk = 0u; chunk != chunks; ++chunk) {
    if (state.component[chunk] != noComponent && marked[state.component[chunk]] != 0u) {
      members.push_back(chunk);
    }
  }

  std::vector<uint32_t> parent(chunks, 0u);

  for (const uint32_t chunk : members) {
    parent[chunk] = chunk;
    state.anchoredComponent[chunk] = 0u;
  }

  for (const uint32_t chunk : members) {
    if (state.alive[chunk] == 0u) {
      continue;
    }

    for (uint32_t slot = state.bondStart[chunk]; slot != state.bondStart[chunk + 1u]; ++slot) {
      const uint32_t index = state.bondList[slot];

      if (state.broken[index] != 0u) {
        continue;
      }

      const Bond& bond = building.bonds[index];
      const uint32_t a = findRoot(parent, bond.first);
      const uint32_t b = findRoot(parent, bond.second);

      if (a != b) {
        parent[std::max(a, b)] = std::min(a, b);
      }
    }
  }

  for (const uint32_t chunk : members) {
    if (state.alive[chunk] == 0u) {
      state.component[chunk] = noComponent;
      continue;
    }

    const uint32_t root = findRoot(parent, chunk);

    state.component[chunk] = root;

    if (root == chunk) {
      changed.push_back(chunk);
    }

    if (building.chunks[chunk].anchored) {
      state.anchoredComponent[root] = 1u;
    }
  }

  return changed;
}

bool isAnchored(const StructureState& state, uint32_t component) {
  return component < state.anchoredComponent.size() && state.anchoredComponent[component] != 0u;
}

}
