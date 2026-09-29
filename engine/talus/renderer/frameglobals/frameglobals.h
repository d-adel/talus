#pragma once

#include <talus/gpu/descriptorgroup/descriptorgroup.h>
#include <talus/gpu/device/device.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>

namespace talus::renderer {

struct CameraUniform {
  glm::mat4 view;
  glm::mat4 proj;
  glm::mat4 viewProj;
  glm::mat4 inverseViewProj;
  glm::vec4 position;
};

static_assert(
  offsetof(CameraUniform, view) == 0,
  "CameraUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(CameraUniform, proj) == 64,
  "CameraUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(CameraUniform, viewProj) == 128,
  "CameraUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(CameraUniform, inverseViewProj) == 192,
  "CameraUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(CameraUniform, position) == 256,
  "CameraUniform must match its std140 block in the shaders"
);
static_assert(
  sizeof(CameraUniform) == 272,
  "CameraUniform must match its std140 block in the shaders"
);

struct MarchParams {
  float epsilon;
  float maxDistance;
  float omega;
  float gradientEpsilon;

  float shadowMaxDistance;
  float shadowSharpness;
  uint32_t maxIterations;
};

static_assert(
  offsetof(MarchParams, epsilon) == 0,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  offsetof(MarchParams, maxDistance) == 4,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  offsetof(MarchParams, omega) == 8,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  offsetof(MarchParams, gradientEpsilon) == 12,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  offsetof(MarchParams, shadowMaxDistance) == 16,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  offsetof(MarchParams, shadowSharpness) == 20,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  offsetof(MarchParams, maxIterations) == 24,
  "MarchParams must match its std140 block in sdf-shader.frag"
);
static_assert(
  sizeof(MarchParams) == 28,
  "MarchParams must match its std140 block in sdf-shader.frag"
);

struct SceneCounts {
  uint32_t sdfRecords;
};

static_assert(
  offsetof(SceneCounts, sdfRecords) == 0,
  "SceneCounts must match its std140 block in sdf-shader.frag"
);
static_assert(
  sizeof(SceneCounts) == 4,
  "SceneCounts must match its std140 block in sdf-shader.frag"
);

struct LightUniform {
  glm::vec4 direction;
  glm::vec4 skyTop;
  glm::vec4 skyHorizon;
  glm::vec4 groundBottom;
  glm::vec4 shading;
};

static_assert(
  offsetof(LightUniform, direction) == 0,
  "LightUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(LightUniform, skyTop) == 16,
  "LightUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(LightUniform, skyHorizon) == 32,
  "LightUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(LightUniform, groundBottom) == 48,
  "LightUniform must match its std140 block in the shaders"
);
static_assert(
  offsetof(LightUniform, shading) == 64,
  "LightUniform must match its std140 block in the shaders"
);
static_assert(
  sizeof(LightUniform) == 80,
  "LightUniform must match its std140 block in the shaders"
);

class FrameGlobals {
public:
  static constexpr uint32_t CAMERA = 0;
  static constexpr uint32_t MARCH = 1;
  static constexpr uint32_t COUNTS = 2;
  static constexpr uint32_t LIGHT = 3;

  explicit FrameGlobals(const talus::gpu::Device& device);

  FrameGlobals(FrameGlobals const&) = delete;
  FrameGlobals& operator=(FrameGlobals const&) = delete;
  FrameGlobals(FrameGlobals&&) = delete;
  FrameGlobals& operator=(FrameGlobals&&) = delete;

  ~FrameGlobals() = default;

  void update(
    uint32_t frame,
    const CameraUniform& camera,
    const MarchParams& march,
    const SceneCounts& counts,
    const LightUniform& light
  );

  const VkDescriptorSet& descriptorSet(uint32_t frame) const noexcept;
  const VkDescriptorSetLayout& setLayout() const noexcept;

private:
  talus::gpu::DescriptorGroup m_group;
};

}
