#pragma once

#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/requires/requires.h>

namespace talus::objectmodel {

struct MainCamera {};

template<>
inline constexpr bool authored<MainCamera> = true;

template<>
struct Requires<MainCamera> {
  using List = ComponentList<Lens>;
};

template<>
inline constexpr bool single<MainCamera> = true;

}
