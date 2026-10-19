#include "simdwidth.h"

#if defined(TALUS_WIDE_CONTACTS)
#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <cpuid.h>
#endif
#endif

#include <array>
#include <cstddef>

namespace talus::dynamics {

namespace {

#if defined(TALUS_WIDE_CONTACTS)

std::array<uint32_t, 4> cpuId(uint32_t leaf, uint32_t subLeaf) {
  std::array<uint32_t, 4> registers{};

#if defined(_MSC_VER)
  std::array<int, 4> info{};

  __cpuidex(info.data(), static_cast<int>(leaf), static_cast<int>(subLeaf));

  for (std::size_t index = 0u; index != 4u; ++index) {
    registers[index] = static_cast<uint32_t>(info[index]);
  }
#else
  __cpuid_count(leaf, subLeaf, registers[0], registers[1], registers[2], registers[3]);
#endif

  return registers;
}

uint64_t extendedControl() {
#if defined(_MSC_VER)
  return _xgetbv(0u);
#else
  uint32_t low = 0u;
  uint32_t high = 0u;

  __asm__ volatile("xgetbv" : "=a"(low), "=d"(high) : "c"(0u));

  return (static_cast<uint64_t>(high) << 32u) | low;
#endif
}

bool detectAvx2() {
  if (cpuId(0u, 0u)[0] < 7u) {
    return false;
  }

  const uint32_t osxsaveAndAvx = (1u << 27u) | (1u << 28u);

  if ((cpuId(1u, 0u)[2] & osxsaveAndAvx) != osxsaveAndAvx) {
    return false;
  }

  if ((extendedControl() & 6u) != 6u) {
    return false;
  }

  return (cpuId(7u, 0u)[1] & (1u << 5u)) != 0u;
}

#endif

}

bool hasAvx2() {
#if defined(TALUS_WIDE_CONTACTS)
  static const bool found = detectAvx2();

  return found;
#else
  return false;
#endif
}

uint32_t simdWidthFor(uint32_t requested, bool avx2) {
  if (requested == 1u || requested == 4u) {
    return requested;
  }

  return avx2 ? 8u : 4u;
}

uint32_t resolveSimdWidth(uint32_t requested) {
#if defined(TALUS_WIDE_CONTACTS)
  return simdWidthFor(requested, hasAvx2());
#else
  (void)requested;

  return 1u;
#endif
}

}
