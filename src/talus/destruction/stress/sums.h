#pragma once

#include <array>
#include <cstdint>

namespace talus::destruction {

constexpr int32_t widestNode = 8;

struct SumSource {
  const double* rows = nullptr;
  const double* pivot = nullptr;
  int32_t stride = 0;
};

using SumSources = std::array<SumSource, widestNode>;

void sumWide(double* product, int32_t span, const SumSources& sources, int32_t count);

}
