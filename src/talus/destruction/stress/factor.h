#pragma once

#include <talus/dynamics/scheduler/scheduler.h>

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace talus::destruction {

using MatrixBlock = std::array<double, 36>;
using VectorBlock = std::array<double, 6>;

enum class Lanes : uint8_t { Two, Widest };

struct BlockPattern {
  std::vector<int32_t> place;
  std::vector<int32_t> start;
  std::vector<int32_t> rows;
  std::vector<int64_t> panel;
  std::vector<int32_t> parent;
  std::vector<int32_t> nodeStart;
  std::vector<int32_t> nodeOf;
  std::vector<int32_t> nodeParent;
  std::vector<int32_t> sourceStart;
  std::vector<int32_t> sources;
  std::vector<double> work;
};

[[nodiscard]] BlockPattern blockPattern(int32_t columns, std::span<const std::array<int32_t, 2>> links);

void addBlock(const BlockPattern& pattern, std::vector<double>& values, int32_t row, int32_t column, const MatrixBlock& block);

[[nodiscard]] std::optional<int32_t> factorBlocks(
  const BlockPattern& pattern,
  std::vector<double>& values,
  Lanes lanes = Lanes::Widest
);

[[nodiscard]] std::optional<int32_t> factorBlocks(
  const BlockPattern& pattern,
  std::vector<double>& values,
  std::span<const int32_t> columns,
  Lanes lanes = Lanes::Widest
);

[[nodiscard]] bool factorBlocks(
  const BlockPattern& pattern,
  std::vector<double>& values,
  std::span<const int32_t> columns,
  const dynamics::Workers& workers,
  Lanes lanes = Lanes::Widest
);

void solveBlocks(const BlockPattern& pattern, const std::vector<double>& values, std::vector<double>& x);

[[nodiscard]] bool solveBlock(MatrixBlock system, VectorBlock& x);

}
