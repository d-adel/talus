#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <span>
#include <unordered_map>

namespace talus::dynamics {

enum class Side : uint8_t { Lattice, Rigid, Static };

inline constexpr uint32_t noChild = 0xFFFFFFFFu;

struct ContactSide {
  Side side = Side::Static;
  uint32_t body = 0u;
  uint32_t child = noChild;
  std::array<uint32_t, 8> nodes{};
  std::array<double, 8> weights{};
  glm::dvec3 local{0.0};
};

struct ContactMemory {
  double normal = 0.0;
  glm::dvec2 friction{0.0};
  double approach = 0.0;
  double impulse = 0.0;
};

struct CachedPose {
  bool valid = false;
  double separation = 0.0;
  glm::dvec3 relativePosition{0.0};
  glm::dquat rotationA{1.0, 0.0, 0.0, 0.0};
  glm::dquat rotationB{1.0, 0.0, 0.0, 0.0};
  glm::dquat relativeRotation{1.0, 0.0, 0.0, 0.0};
};

struct Contact {
  ContactSide a;
  ContactSide b;
  std::array<glm::dvec3, 3> basis{
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(1.0, 0.0, 0.0),
    glm::dvec3(0.0, 0.0, 1.0)
  };
  double gap = 0.0;
  double friction = 0.0;
  double restitution = 0.0;
  uint64_t key = 0u;
  uint64_t pair = 0u;
  uint32_t manifold = 0u;
  ContactMemory memory;
  CachedPose cached;
};

struct CarriedForce {
  uint32_t body = 0u;
  uint64_t pair = 0u;
  glm::dvec3 force{0.0};
};

struct ContactHitEvent {
  Side firstSide = Side::Static;
  uint32_t first = 0u;
  Side secondSide = Side::Static;
  uint32_t second = 0u;
  glm::dvec3 point{0.0};
  glm::dvec3 normal{0.0};
  double approachSpeed = 0.0;
  double impulse = 0.0;
};

class ContactMatch {
public:
  explicit ContactMatch(std::span<const Contact> previous);

  [[nodiscard]] int32_t find(uint64_t key) const;

private:
  std::unordered_map<uint64_t, int32_t> m_index;
};

void carryMemory(std::span<Contact> contacts, std::span<const Contact> previous);

}
