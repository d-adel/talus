#pragma once

#include <talus/systems/input/input.h>

#include <cstdint>

namespace talus::capi {

[[nodiscard]] talus::systems::Key keyFromWin32(uint32_t virtualKey);

}
