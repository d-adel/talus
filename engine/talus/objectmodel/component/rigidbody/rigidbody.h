#pragma once

#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/requires/requires.h>

namespace talus::objectmodel {

struct RigidBody {};

template<>
inline constexpr bool authored<RigidBody> = true;

template<>
struct Requires<RigidBody> {
  using List = ComponentList<SdfGrid>;
};

}
