#pragma once

#include <talus/objectmodel/trait/componentlist/componentlist.h>

namespace talus::objectmodel {

template<typename T>
struct Requires {
  using List = ComponentList<>;
};

template<typename T>
struct Excludes {
  using List = ComponentList<>;
};

template<typename T>
inline constexpr bool single = false;

}
