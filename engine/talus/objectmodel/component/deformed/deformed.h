#pragma once

#include <talus/objectmodel/trait/transient/transient.h>

#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

namespace talus::objectmodel {

struct Deformed {};

template<>
inline constexpr bool selfContained<Deformed> = true;

template<>
inline constexpr bool transient<Deformed> = true;

}
