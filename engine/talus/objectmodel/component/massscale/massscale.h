#pragma once

#include <talus/objectmodel/trait/authored/authored.h>

#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

namespace talus::objectmodel {

struct MassScale {
  float factor = 1.0f;
};

template<>
inline constexpr bool selfContained<MassScale> = true;

template<>
inline constexpr bool authored<MassScale> = true;

}
