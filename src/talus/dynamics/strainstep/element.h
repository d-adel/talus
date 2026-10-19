#pragma once

#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace talus::dynamics {

inline constexpr uint32_t ELEMENT_ROWS = 18u;
inline constexpr double RESOLVED = 1.5;
inline constexpr uint32_t MOST_SUBSTEPS = 64u;

using ElementInverse = std::array<double, ELEMENT_ROWS * ELEMENT_ROWS>;

struct ElementRest {
  glm::dmat3 shape{1.0};
  std::array<glm::dvec3, 4> hourglass{};
  double volume = 0.0;
};

struct ElementMaterial {
  double shear = 0.0;
  double first = 0.0;
};

struct ElementDamping {
  double target = 0.0;
  double lowest = 0.0;
  double theta = 1.0;
  double own = 0.0;
};

struct StrainElement {
  uint32_t cell = 0u;
  std::array<uint32_t, 8> nodes{};
  std::array<glm::dvec3, 8> gradient{};
  std::array<double, 8> inverseMass{};
  std::array<glm::dvec3, 4> restHourglass{};
  double bend = 0.0;
  double size = 0.0;
  std::array<double, ELEMENT_ROWS> stiffness{};
  std::array<double, ELEMENT_ROWS> rate{};
  std::array<double, ELEMENT_ROWS> softness{};
  std::array<double, ELEMENT_ROWS> cap{};
  std::shared_ptr<const ElementInverse> inverse;
  double volumeStiffness = 0.0;
  double volumeRate = 0.0;
  double volumeSoftness = 0.0;
  double volumeCap = 0.0;
  bool volumetric = false;
  std::array<double, ELEMENT_ROWS> impulse{};
  double volumeImpulse = 0.0;
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dmat3 turn{1.0};
  std::array<double, ELEMENT_ROWS> strain{};
  std::array<glm::dvec3, 8> volumeGradient{};
  std::array<double, ELEMENT_ROWS> coupling{};
  std::array<double, ELEMENT_ROWS> image{};
  double gain = 0.0;
  double rhs = 0.0;
  double schur = 0.0;
};

[[nodiscard]] glm::dvec3 cornerSigns(uint32_t corner);

[[nodiscard]] double hourglassSign(uint32_t mode, uint32_t corner);

[[nodiscard]] ElementMaterial elementMaterial(const talus::PhysicalMaterial& material);

[[nodiscard]] double cellFrequency(const talus::PhysicalMaterial& material, double cell);

[[nodiscard]] ElementRest cornerRest(std::span<const glm::dvec3, 8> corners, double volume);

[[nodiscard]] glm::dvec3 rowJacobian(const StrainElement& element, uint32_t corner, uint32_t row);

[[nodiscard]] std::optional<StrainElement> makeElement(
  uint32_t cell,
  std::span<const uint32_t, 8> nodes,
  std::span<const double, 8> inverseMass,
  const ElementRest& rest,
  const ElementMaterial& material,
  double h,
  const ElementDamping& damping
);

[[nodiscard]] glm::dquat rotationOf(const glm::dmat3& deformation, const glm::dquat& guess);

void updateElement(StrainElement& element, std::span<const glm::dvec3> positions);

void applyElement(
  const StrainElement& element,
  std::span<glm::dvec3> velocities,
  std::span<const double, ELEMENT_ROWS> impulse,
  double volumeImpulse
);

void solveElement(StrainElement& element, std::span<glm::dvec3> velocities);

}
