#include "contactrows.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace talus::dynamics {

namespace {

constexpr double LEVER = 0.02;
constexpr double CONTACT_LIMIT = 0.25;
constexpr double SINGULAR = 1000.0 * std::numeric_limits<double>::min();

RowSide sideOf(
  const ContactSide& source,
  const RowBodies& bodies,
  std::span<const uint32_t> stateOf
) {
  RowSide side;

  side.side = source.side;

  if (source.side == Side::Lattice) {
    const LatticeBody& lattice = bodies.lattices[source.body];
    const talus::PreparedMesh& mesh = *lattice.mesh;
    double spread = 0.0;
    double largest = 0.0;

    side.body = source.body;

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      if (source.weights[corner] == 0.0) {
        continue;
      }

      const uint32_t node = source.nodes[corner];
      const bool held = !mesh.anchored.empty() && mesh.anchored[node] != 0u;
      RowAnchor& anchor = side.anchors[side.count];

      anchor.node = node;
      anchor.weight = source.weights[corner];
      anchor.inverseMass = held || !(mesh.nodeMass[node] > 0.0f)
                             ? 0.0
                             : 1.0 / static_cast<double>(mesh.nodeMass[node]);
      spread += anchor.weight * anchor.weight * anchor.inverseMass;
      largest = std::max(largest, anchor.inverseMass);
      ++side.count;
    }

    if (largest > 0.0 && spread < LEVER * largest) {
      for (uint32_t at = 0u; at != side.count; ++at) {
        side.anchors[at].inverseMass = 0.0;
      }
    }

    for (uint32_t at = 0u; at != side.count; ++at) {
      const RowAnchor& anchor = side.anchors[at];

      if (anchor.inverseMass > 0.0) {
        side.start += anchor.weight * lattice.position[anchor.node];
      }
    }

    return side;
  }

  if (source.side == Side::Rigid) {
    const RigidState& state = bodies.rigids[stateOf[source.body]];

    side.body = stateOf[source.body];
    side.local = source.local;
    side.start = state.position + glm::mat3_cast(state.rotation) * source.local;
    side.arm = side.start - state.position;
  }

  return side;
}

double sideMass(const RowSide& side, const RowBodies& bodies, const glm::dvec3& direction) {
  if (side.side == Side::Lattice) {
    double total = 0.0;

    for (uint32_t at = 0u; at != side.count; ++at) {
      const RowAnchor& anchor = side.anchors[at];

      total += anchor.weight * anchor.weight * anchor.inverseMass;
    }

    return total;
  }

  if (side.side == Side::Rigid) {
    const RigidState& state = bodies.rigids[side.body];
    const glm::dvec3 lever = glm::cross(side.arm, direction);

    return state.invMass + glm::dot(lever, state.invI * lever);
  }

  return 0.0;
}

glm::dmat2 tangentBlock(
  const RowSide& side,
  const RowBodies& bodies,
  const glm::dvec3& first,
  const glm::dvec3& second
) {
  if (side.side == Side::Lattice) {
    const double mass = sideMass(side, bodies, first);

    return glm::dmat2(mass, 0.0, 0.0, mass);
  }

  if (side.side == Side::Rigid) {
    const RigidState& state = bodies.rigids[side.body];
    const glm::dvec3 a = glm::cross(side.arm, first);
    const glm::dvec3 b = glm::cross(side.arm, second);
    const double shared = glm::dot(a, state.invI * b);

    return glm::dmat2(
      state.invMass + glm::dot(a, state.invI * a),
      shared,
      shared,
      state.invMass + glm::dot(b, state.invI * b)
    );
  }

  return glm::dmat2(0.0);
}

glm::dmat2 invert(const glm::dmat2& m) {
  const double det = m[0][0] * m[1][1] - m[1][0] * m[0][1];

  if (std::abs(det) <= SINGULAR) {
    return glm::dmat2(0.0);
  }

  const double invDet = 1.0 / det;

  return glm::dmat2(
    glm::dvec2(invDet * m[1][1], -invDet * m[0][1]),
    glm::dvec2(-invDet * m[1][0], invDet * m[0][0])
  );
}

double sideVelocity(const RowSide& side, const RowBodies& bodies, const glm::dvec3& direction) {
  if (side.side == Side::Lattice) {
    const LatticeBody& lattice = bodies.lattices[side.body];
    double total = 0.0;

    for (uint32_t at = 0u; at != side.count; ++at) {
      const RowAnchor& anchor = side.anchors[at];

      if (anchor.inverseMass > 0.0) {
        total += anchor.weight * glm::dot(lattice.velocity[anchor.node], direction);
      }
    }

    return total;
  }

  if (side.side == Side::Rigid) {
    const RigidState& state = bodies.rigids[side.body];

    return glm::dot(state.v + glm::cross(state.w, side.arm), direction);
  }

  return 0.0;
}

double sideMoved(const RowSide& side, const RowBodies& bodies, const glm::dvec3& direction) {
  if (side.side == Side::Lattice) {
    const LatticeBody& lattice = bodies.lattices[side.body];
    glm::dvec3 point(0.0);

    for (uint32_t at = 0u; at != side.count; ++at) {
      const RowAnchor& anchor = side.anchors[at];

      if (anchor.inverseMass > 0.0) {
        point += anchor.weight * lattice.position[anchor.node];
      }
    }

    return glm::dot(point - side.start, direction);
  }

  if (side.side == Side::Rigid) {
    const RigidState& state = bodies.rigids[side.body];
    const glm::dvec3 point =
      state.position + state.dp + glm::mat3_cast(state.dq * state.rotation) * side.local;

    return glm::dot(point - side.start, direction);
  }

  return 0.0;
}

void pushSide(const RowSide& side, const RowBodies& bodies, const glm::dvec3& direction, double impulse) {
  if (side.side == Side::Lattice) {
    LatticeBody& lattice = bodies.lattices[side.body];

    for (uint32_t at = 0u; at != side.count; ++at) {
      const RowAnchor& anchor = side.anchors[at];

      if (anchor.inverseMass > 0.0) {
        lattice.velocity[anchor.node] += anchor.weight * anchor.inverseMass * impulse * direction;
      }
    }

    return;
  }

  if (side.side == Side::Rigid) {
    RigidState& state = bodies.rigids[side.body];

    if (state.invMass > 0.0) {
      state.v += state.invMass * impulse * direction;
      state.w += state.invI * glm::cross(side.arm, direction) * impulse;
    }
  }
}

void applyRow(const ContactRow& row, const RowBodies& bodies, const glm::dvec3& direction, double impulse) {
  pushSide(row.a, bodies, direction, impulse);
  pushSide(row.b, bodies, direction, -impulse);
}

double relativeVelocity(const ContactRow& row, const RowBodies& bodies, const glm::dvec3& direction) {
  return sideVelocity(row.a, bodies, direction) - sideVelocity(row.b, bodies, direction);
}

double relativeMoved(const ContactRow& row, const RowBodies& bodies, const glm::dvec3& direction) {
  return sideMoved(row.a, bodies, direction) - sideMoved(row.b, bodies, direction);
}

}

std::optional<ContactRow> makeContactRow(
  const Contact& contact,
  uint32_t index,
  const RowBodies& bodies,
  std::span<const uint32_t> stateOf,
  std::span<const uint8_t> stiff,
  const RowSettings& settings
) {
  ContactRow row;

  row.contact = index;
  row.a = sideOf(contact.a, bodies, stateOf);
  row.b = sideOf(contact.b, bodies, stateOf);
  row.normal = contact.basis[0];
  row.tangent1 = contact.basis[1];
  row.tangent2 = contact.basis[2];

  const double massA = sideMass(row.a, bodies, row.normal);
  const double massB = sideMass(row.b, bodies, row.normal);

  if (!(massA + massB > 0.0)) {
    return std::nullopt;
  }

  const bool fixed = !(massA > 0.0) || !(massB > 0.0);
  const bool tooStiff = (contact.a.side == Side::Lattice && stiff[contact.a.body] != 0u)
                        || (contact.b.side == Side::Lattice && stiff[contact.b.body] != 0u);
  const double hertz = settings.contactHertz * (fixed ? 8.0 : 4.0);
  const Softness softness = makeSoft(
    tooStiff ? std::min(hertz, CONTACT_LIMIT / settings.h) : hertz,
    settings.dampingRatio,
    settings.h
  );

  row.gap = contact.gap;
  row.normalMass = 1.0 / (massA + massB);
  row.tangentMass = invert(
    tangentBlock(row.a, bodies, row.tangent1, row.tangent2)
    + tangentBlock(row.b, bodies, row.tangent1, row.tangent2)
  );
  row.biasRate = softness.biasRate;
  row.massScale = softness.massScale;
  row.friction = contact.friction;
  row.soft = contact.a.side == Side::Lattice && contact.b.side == Side::Lattice;
  row.normalImpulse = std::max(0.0, contact.memory.normal * settings.h);
  row.tangentImpulse = contact.memory.friction * settings.h;
  row.approach = -relativeVelocity(row, bodies, row.normal);

  return row;
}

void refreshContactRow(
  ContactRow& row,
  const RowBodies& bodies,
  double invH,
  double contactSpeed,
  bool relax
) {
  const double gap = row.gap + relativeMoved(row, bodies, row.normal);
  const double ahead = std::max(gap, 0.0) * invH;

  if (relax) {
    row.rhsNormal = ahead;
    row.scale = 1.0;
    row.rhsTangent = glm::dvec2(0.0);

    return;
  }

  row.rhsNormal = ahead + std::clamp(gap * row.biasRate, -contactSpeed, 0.0);
  row.scale = gap <= 0.0 ? row.massScale : 1.0;
  row.rhsTangent = glm::dvec2(
    relativeMoved(row, bodies, row.tangent1),
    relativeMoved(row, bodies, row.tangent2)
  ) * invH;
}

void warmStartContactRow(const ContactRow& row, const RowBodies& bodies) {
  applyRow(row, bodies, row.normal, row.normalImpulse);
  applyRow(row, bodies, row.tangent1, row.tangentImpulse.x);
  applyRow(row, bodies, row.tangent2, row.tangentImpulse.y);
}

void solveContactRow(ContactRow& row, const RowBodies& bodies, bool tangentBias) {
  const double approach = relativeVelocity(row, bodies, row.normal) + row.rhsNormal;
  const double normal = std::max(row.scale * (row.normalImpulse - row.normalMass * approach), 0.0);

  applyRow(row, bodies, row.normal, normal - row.normalImpulse);
  row.normalImpulse = normal;

  const double limit = row.friction * row.normalImpulse;
  glm::dvec2 slip(
    relativeVelocity(row, bodies, row.tangent1),
    relativeVelocity(row, bodies, row.tangent2)
  );

  if (tangentBias) {
    slip += row.rhsTangent;
  }

  glm::dvec2 tangent = row.tangentImpulse - row.tangentMass * slip;
  const double length = glm::length(tangent);

  if (length > limit) {
    tangent *= limit / length;
  }

  applyRow(row, bodies, row.tangent1, tangent.x - row.tangentImpulse.x);
  applyRow(row, bodies, row.tangent2, tangent.y - row.tangentImpulse.y);
  row.tangentImpulse = tangent;
}

void storeContactRow(const ContactRow& row, Contact& contact, double h) {
  contact.memory.normal = row.normalImpulse / h;
  contact.memory.friction = row.tangentImpulse / h;
  contact.memory.approach = row.approach;
  contact.memory.impulse = row.total;
}

}
