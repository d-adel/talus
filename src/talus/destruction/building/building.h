#pragma once

#include <talus/destruction/materials/materials.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::destruction {

enum class Role : uint8_t { Wall, Slab, Column, Beam, Pane, Foundation };

struct Member {
  glm::dvec3 centre{0.0};
  glm::dvec3 halfExtents{0.0};
  uint32_t material = 0u;
  Role role = Role::Wall;
};

struct Opening {
  glm::dvec3 centre{0.0};
  glm::dvec3 halfExtents{0.0};
};

struct Building {
  std::vector<Member> members;
  std::vector<Opening> openings;
  glm::dvec3 position{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
};

void validate(const Building& building, std::span<const StructuralMaterial> materials);

}
