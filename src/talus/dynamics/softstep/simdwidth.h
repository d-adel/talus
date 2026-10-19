#pragma once

#include <cstdint>

namespace talus::dynamics {

[[nodiscard]] bool hasAvx2();
[[nodiscard]] uint32_t simdWidthFor(uint32_t requested, bool avx2);
[[nodiscard]] uint32_t resolveSimdWidth(uint32_t requested);

}
