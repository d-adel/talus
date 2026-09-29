#pragma once

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/rigidbody/rigidbody.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/trait/requires/requires.h>

namespace talus::objectmodel {

template<>
struct Excludes<StaticBody> {
  using List = ComponentList<RigidBody, DeformableBody>;
};

template<>
struct Excludes<RigidBody> {
  using List = ComponentList<StaticBody, DeformableBody>;
};

template<>
struct Excludes<DeformableBody> {
  using List = ComponentList<StaticBody, RigidBody>;
};

}
