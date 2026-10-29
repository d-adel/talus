#pragma once

#include <talus/dynamics/manifoldcache/manifoldcache.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

namespace talus::dynamics {

inline constexpr double jointInfinity = std::numeric_limits<double>::infinity();

enum class JointKind : uint8_t { Ball, Hinge, SwingTwist, Spring };

struct JointFrame {
  glm::dvec3 position{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
};

struct AxisDrive {
  double targetAngle = 0.0;
  double targetVelocity = 0.0;
  double stiffness = 0.0;
  double damping = 0.0;
  double minTorque = -jointInfinity;
  double maxTorque = jointInfinity;
};

struct JointDrive {
  std::array<AxisDrive, 3> axes{};
  double maxTorque = jointInfinity;
  glm::dvec3 feedForward{0.0};
};

struct HingeLimit {
  bool enabled = false;
  double lower = 0.0;
  double upper = 0.0;
};

struct ConeLimit {
  bool enabled = false;
  double twistLower = 0.0;
  double twistUpper = 0.0;
  glm::dvec2 swing{0.0};
  glm::dvec2 centre{0.0};
};

struct SpringSettings {
  JointFrame target{};
  double linearStiffness = 0.0;
  double linearDamping = 0.0;
  double maxForce = jointInfinity;
  double angularStiffness = 0.0;
  double angularDamping = 0.0;
  double maxTorque = jointInfinity;
};

struct JointDef {
  JointKind kind = JointKind::Ball;
  uint32_t bodyA = noBody;
  uint32_t bodyB = noBody;
  JointFrame frameA{};
  JointFrame frameB{};
  double hertz = 60.0;
  double dampingRatio = 2.0;
  bool collideConnected = false;
  HingeLimit hinge{};
  ConeLimit cone{};
  JointDrive drive{};
  SpringSettings spring{};
};

struct JointReport {
  glm::dvec3 force{0.0};
  glm::dvec3 torque{0.0};
  glm::dvec3 driveTorque{0.0};
  bool limitActive = false;
  bool driveSaturated = false;
  bool forceSaturated = false;
  bool torqueSaturated = false;
};

struct JointImpulses {
  glm::dvec3 point{0.0};
  glm::dvec2 alignment{0.0};
  double hingeLower = 0.0;
  double hingeUpper = 0.0;
  double twistLower = 0.0;
  double twistUpper = 0.0;
  double cone = 0.0;
  glm::dvec3 drive{0.0};
  glm::dvec3 springLinear{0.0};
  glm::dvec3 springAngular{0.0};
};

struct Joint {
  uint32_t id = 0u;
  JointDef def;
  JointReport report;
  JointImpulses impulses;
};

class JointSet {
public:
  [[nodiscard]] uint32_t add(const JointDef& def);
  bool remove(uint32_t id);
  Joint* find(uint32_t id);
  const Joint* find(uint32_t id) const;
  [[nodiscard]] std::span<const Joint> all() const;
  [[nodiscard]] std::span<Joint> all();
  [[nodiscard]] bool excludes(uint32_t a, uint32_t b) const;
  [[nodiscard]] bool empty() const;

private:
  std::vector<Joint> joints_;
  std::unordered_map<uint64_t, uint32_t> excluded_;
  uint32_t nextId_ = 1u;
};

}
