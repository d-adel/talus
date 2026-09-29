#pragma once

#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/componentlist/componentlist.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/directionallight/directionallight.h>
#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/maincamera/maincamera.h>
#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/mesh/mesh.h>
#include <talus/objectmodel/component/name/name.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/rigidbody/rigidbody.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/environment/environment.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/component/support/support.h>

namespace talus::objectmodel {

using AuthoredComponents = ComponentList<
  Pose,
  SdfGrid,
  Lens,
  MainCamera,
  MassScale,
  StaticBody,
  RigidBody,
  DeformableBody,
  Support,
  Mesh,
  Material,
  DirectionalLight,
  Environment,
  Name>;

}
