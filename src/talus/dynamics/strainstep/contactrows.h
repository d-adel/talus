#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/strainstep/rigidstate.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace talus::dynamics {

struct RowAnchor {
  uint32_t node = 0u;
  double weight = 0.0;
  double inverseMass = 0.0;
};

struct RowSide {
  Side side = Side::Static;
  uint32_t body = 0u;
  uint32_t count = 0u;
  std::array<RowAnchor, 8> anchors{};
  glm::dvec3 local{0.0};
  glm::dvec3 arm{0.0};
  glm::dvec3 start{0.0};
};

struct ContactRow {
  uint32_t contact = 0u;
  RowSide a;
  RowSide b;
  glm::dvec3 normal{0.0, 1.0, 0.0};
  glm::dvec3 tangent1{1.0, 0.0, 0.0};
  glm::dvec3 tangent2{0.0, 0.0, 1.0};
  double gap = 0.0;
  double normalMass = 0.0;
  glm::dmat2 tangentMass{0.0};
  double biasRate = 0.0;
  double massScale = 1.0;
  double friction = 0.0;
  bool soft = false;
  double rhsNormal = 0.0;
  glm::dvec2 rhsTangent{0.0};
  double scale = 1.0;
  double normalImpulse = 0.0;
  glm::dvec2 tangentImpulse{0.0};
  double total = 0.0;
  double approach = 0.0;
};

struct RowBodies {
  std::span<LatticeBody> lattices;
  std::span<RigidState> rigids;
};

struct RowSettings {
  double h = 0.0;
  double contactHertz = 30.0;
  double dampingRatio = 10.0;
};

[[nodiscard]] std::optional<ContactRow> makeContactRow(
  const Contact& contact,
  uint32_t index,
  const RowBodies& bodies,
  std::span<const uint32_t> stateOf,
  std::span<const uint8_t> stiff,
  const RowSettings& settings
);

void refreshContactRow(
  ContactRow& row,
  const RowBodies& bodies,
  double invH,
  double contactSpeed,
  bool relax
);

void warmStartContactRow(const ContactRow& row, const RowBodies& bodies);

void solveContactRow(ContactRow& row, const RowBodies& bodies, bool tangentBias);

void storeContactRow(const ContactRow& row, Contact& contact, double h);

}
