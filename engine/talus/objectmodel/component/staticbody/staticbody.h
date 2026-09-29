#pragma once

#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/requires/requires.h>

namespace talus::objectmodel {

struct StaticBody {};

template<>
inline constexpr bool authored<StaticBody> = true;

template<>
struct Requires<StaticBody> {
  using List = ComponentList<SdfGrid>;
};

}
