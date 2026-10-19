#pragma once

#include <talus/dynamics/bodyset/bodyset.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::dynamics {

inline constexpr uint32_t MIN_PIECE_CELLS = 2u;

struct CellStretch {
  double stretch = 0.0;
  glm::dvec3 direction{1.0, 0.0, 0.0};
};

[[nodiscard]] CellStretch cellStretch(const LatticeBody& lattice, uint32_t cell);

[[nodiscard]] std::vector<uint32_t> overstretched(const LatticeBody& lattice);

uint32_t tearCells(LatticeBody& lattice, std::span<const uint32_t> cells);

[[nodiscard]] uint32_t latticePieces(const LatticeBody& lattice);

}
