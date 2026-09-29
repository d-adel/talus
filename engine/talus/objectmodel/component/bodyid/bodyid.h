#pragma once

#include <talus/objectmodel/trait/transient/transient.h>

#include <cstdint>

namespace talus::objectmodel {

struct BodyId {
  uint32_t id = UINT32_MAX;
};

template<>
inline constexpr bool transient<BodyId> = true;

}
