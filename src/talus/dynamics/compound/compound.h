#pragma once

#include <talus/dynamics/bounds/bounds.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace talus::dynamics {

inline constexpr uint32_t childSampleCount = 26u;
inline constexpr uint8_t allFaces = 0x3Fu;

struct CompoundChild {
  glm::dvec3 centre{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 half{0.0};
  double density = 0.0;
  uint32_t key = 0u;
};

struct ChildNode {
  Bounds bounds;
  uint32_t left = 0u;
  uint32_t right = 0u;
  uint32_t first = 0u;
  uint32_t count = 0u;
};

struct CompoundShape {
  std::vector<CompoundChild> children;
  std::vector<Bounds> childBounds;
  std::vector<uint8_t> exposed;
  std::vector<uint16_t> faceCovers;
  std::vector<double> childReach;
  std::vector<ChildNode> nodes;
  std::vector<uint32_t> leaves;
  Bounds bounds;
  double reach = 0.0;
  double thinnest = 0.0;
  uint64_t revision = 0u;
};

struct ChildReplacement {
  uint32_t key = 0u;
  std::vector<CompoundChild> parts;
};

struct CompoundMass {
  double mass = 0.0;
  glm::dvec3 centre{0.0};
  glm::dmat3 inertia{0.0};
};

struct PrincipalFrame {
  double mass = 0.0;
  glm::dvec3 centre{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 moment{0.0};
};

[[nodiscard]] Bounds boxBoundsIn(
  const glm::dvec3& centre,
  const glm::dmat3& turn,
  const glm::dvec3& half
);

[[nodiscard]] std::shared_ptr<const CompoundShape> makeCompoundShape(
  std::vector<CompoundChild> children,
  uint64_t revision = 0u
);

void removeChildren(
  std::shared_ptr<const CompoundShape>& shape,
  std::span<const uint32_t> keys
);

void replaceChildren(
  std::shared_ptr<const CompoundShape>& shape,
  std::span<const ChildReplacement> replacements
);

[[nodiscard]] std::optional<uint32_t> findChild(const CompoundShape& shape, uint32_t key);

void overlappingChildren(
  const CompoundShape& shape,
  const Bounds& probe,
  std::vector<uint32_t>& found
);

[[nodiscard]] double childMass(const CompoundChild& child);

[[nodiscard]] CompoundMass compoundMass(std::span<const CompoundChild> children);

[[nodiscard]] PrincipalFrame principalFrame(std::span<const CompoundChild> children);

[[nodiscard]] std::vector<CompoundChild> childrenInFrame(
  std::span<const CompoundChild> children,
  const glm::dvec3& centre,
  const glm::dquat& rotation
);

[[nodiscard]] std::array<glm::dvec3, childSampleCount> childSamples(const CompoundChild& child);

}
