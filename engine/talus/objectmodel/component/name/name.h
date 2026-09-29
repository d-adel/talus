#pragma once

#include <talus/objectmodel/text/text.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

namespace talus::objectmodel {

struct Name {
  Text text{};
};

template<>
inline constexpr bool selfContained<Name> = true;

template<>
inline constexpr bool authored<Name> = true;

}
