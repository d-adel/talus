#pragma once

#include <talus/core/config/config.h>
#include <talus/debugdraw/debugdraw/debugdraw.h>
#include <talus/renderer/frameglobals/frameglobals.h>
#include <talus/renderer/meshbuffers/meshbuffers.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/field/field.h>
#include <talus/renderer/meshrecord/meshrecord.h>
#include <talus/objectmodel/component/directionallight/directionallight.h>
#include <talus/objectmodel/component/environment/environment.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/entity/entity.h>

#include <glm/glm.hpp>
#include <optional>
#include <span>

namespace talus::renderer {

struct Frame {
  glm::mat4 view = glm::mat4(1.0f);
  glm::mat4 proj = glm::mat4(1.0f);
  glm::vec4 cameraPos = glm::vec4(1.0f);
  std::span<const MeshRecord> meshRecords = {};
  std::span<const MeshBuffers> meshes = {};

  uint32_t sdfRecordCount = 0;
  uint32_t droppedVolumes = 0;
  std::span<const talus::objectmodel::Entity> sdfEntities = {};

  talus::objectmodel::Entity selected = talus::objectmodel::INVALID_ENTITY;
  std::span<const talus::field::Primitive> primitives = {};

  bool drawMeshes = true;
  bool drawSdf = true;

  uint32_t fieldRepresentation = talus::core::FIELD_SAMPLED;

  std::optional<talus::objectmodel::DirectionalLight> light{};

  std::optional<talus::objectmodel::Environment> environment{};

  std::span<const talus::debugdraw::LineVertex> debugTested = {};
  std::span<const talus::debugdraw::LineVertex> debugUntested = {};
  std::span<const talus::debugdraw::LineVertex> overlayLines = {};

  MarchParams march{
    TALUS_MARCH_EPSILON,
    TALUS_MARCH_MAX_DISTANCE,
    TALUS_MARCH_OMEGA,
    TALUS_GRADIENT_EPSILON,
    TALUS_SHADOW_MAX_DISTANCE,
    TALUS_SHADOW_SHARPNESS,
    TALUS_MARCH_MAX_ITERATIONS
  };
};

}
