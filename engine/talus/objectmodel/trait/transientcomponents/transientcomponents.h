#pragma once

#include <talus/objectmodel/trait/transient/transient.h>
#include <talus/objectmodel/component/bakedvolume/bakedvolume.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/deformed/deformed.h>
#include <talus/objectmodel/component/renderpose/renderpose.h>
#include <talus/objectmodel/component/sharedfield/sharedfield.h>
#include <talus/objectmodel/sdfprimitivestore/sdfprimitivestore.h>

namespace talus::objectmodel {

template<Transient... Ts>
struct TransientList {
  template<typename Fn>
  static void each(Fn&& fn) {
    (fn.template operator()<Ts>(), ...);
  }
};

using TransientComponents = TransientList<
  BakedVolume,
  BodyId,
  DeformablePrepared,
  DeformableState,
  Deformed,
  RenderPose,
  SdfPrimitiveRange,
  SharedSdf>;

}
