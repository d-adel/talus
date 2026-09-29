#pragma once

#include <talus/core/config/config.h>
#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/view/drawmode/drawmode.h>
#include <talus/view/grid/gridstyle.h>

#include <cstdint>

namespace talus::view {

using TargetId = uint32_t;
using ViewportId = uint32_t;

inline constexpr TargetId INVALID_TARGET = UINT32_MAX;
inline constexpr ViewportId INVALID_VIEWPORT = UINT32_MAX;

struct MarchSettings {
  float epsilon = TALUS_MARCH_EPSILON;
  float maxDistance = TALUS_MARCH_MAX_DISTANCE;
  float omega = TALUS_MARCH_OMEGA;
  float gradientEpsilon = TALUS_GRADIENT_EPSILON;

  float shadowMaxDistance = TALUS_SHADOW_MAX_DISTANCE;
  float shadowSharpness = TALUS_SHADOW_SHARPNESS;

  uint32_t maxIterations = TALUS_MARCH_MAX_ITERATIONS;
};

struct Viewport {
  TargetId target = INVALID_TARGET;

  talus::objectmodel::Entity camera = talus::objectmodel::INVALID_ENTITY;

  talus::objectmodel::Pose pose{};
  talus::objectmodel::Lens lens{};

  DrawMode mode = DrawMode::Both;

  uint32_t fieldRepresentation = talus::core::FIELD_SAMPLED;

  MarchSettings march{};

  float supersample = 1.0f;

  bool gridVisible = false;
  GridPlane gridPlane{};
  GridStyle gridStyle{};
};

}
