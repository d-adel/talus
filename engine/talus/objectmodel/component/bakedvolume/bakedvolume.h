#pragma once

#include <talus/objectmodel/trait/transient/transient.h>

#include <cstdint>

namespace talus::objectmodel {

struct BakedVolume {
  uint32_t index = UINT32_MAX;
};

template<>
inline constexpr bool transient<BakedVolume> = true;

}
