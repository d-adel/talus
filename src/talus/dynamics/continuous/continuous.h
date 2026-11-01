#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::dynamics {

struct RigidPose {
  glm::dvec3 position{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
};

[[nodiscard]] double turnBound(const glm::dquat& from, const glm::dquat& to);

[[nodiscard]] double timeOfImpact(
  const RigidBody& body,
  const RigidPose& start,
  const RigidPose& end,
  const field::SampledSdf& field,
  double target
);

[[nodiscard]] std::vector<uint32_t> sweepAnchors(const BodySet& bodies, const Workers& workers = Workers{});

void refreshAnchors(
  std::vector<uint32_t>& anchors,
  const BodySet& bodies,
  std::span<const uint32_t> rigids
);

[[nodiscard]] bool needsSweep(
  const BodySet& bodies,
  std::span<const RigidPose> start,
  uint32_t rigid
);

void solveContinuousBody(
  BodySet& bodies,
  std::span<const RigidPose> start,
  uint32_t rigid,
  const IntegrationParameters& parameters,
  std::span<const uint32_t> anchors
);

}
