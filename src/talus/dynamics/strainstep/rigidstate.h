#pragma once

#include <talus/dynamics/bodyset/bodyset.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>

namespace talus::dynamics {

struct Softness {
  double biasRate = 0.0;
  double massScale = 0.0;
  double impulseScale = 0.0;
};

struct RigidState {
  uint32_t body = 0u;
  glm::dvec3 position{0.0};
  glm::dvec3 v{0.0};
  glm::dvec3 w{0.0};
  glm::dvec3 dp{0.0};
  glm::dquat dq{1.0, 0.0, 0.0, 0.0};
  double invMass = 0.0;
  glm::dmat3 invI{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 moment{0.0};
};

[[nodiscard]] Softness makeSoft(double hertz, double zeta, double h);

[[nodiscard]] RigidState gatherRigid(const RigidBody& body, uint32_t index);

void integrateRigidVelocity(RigidState& state, const glm::dvec3& gravity, double h);

void integrateRigidPosition(
  RigidState& state,
  double h,
  double maxLinearSpeed,
  double maxAngularSpeed
);

void storeRigid(const RigidState& state, RigidBody& body);

}
