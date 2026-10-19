#include "rigidrows.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace talus::dynamics {

namespace {

constexpr double MIN_FRICTION_WEIGHT = 1.0e-10;
constexpr double SINGULAR = 1000.0 * std::numeric_limits<double>::min();

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

glm::dvec3 centreOf(const BodySet& bodies, const BodyRef& side, const BodyRef& other) {
  if (side.side == Side::Rigid) {
    return bodies.rigids[side.index].position;
  }

  if (other.side == Side::Rigid) {
    return bodies.rigids[other.index].position;
  }

  return glm::dvec3(0.0);
}

glm::dvec3 contactPoint(
  const BodySet& bodies,
  const BodyRef& a,
  const BodyRef& b,
  const CachedManifoldPoint& point,
  double separation,
  const glm::dvec3& normal
) {
  if (a.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[a.index];

    return body.position + glm::mat3_cast(body.rotation) * point.localA - 0.5 * separation * normal;
  }

  if (b.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[b.index];

    return body.position + glm::mat3_cast(body.rotation) * point.localB + 0.5 * separation * normal;
  }

  return glm::dvec3(0.0);
}

void storeState(RigidState& state, const glm::dvec3& v, const glm::dvec3& w) {
  if (state.invMass != 0.0) {
    state.v = v;
    state.w = w;
  }
}

}

void prepareManifolds(
  const BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const uint32_t> slots,
  std::span<const RigidState> states,
  std::span<const uint32_t> stateOf,
  const ManifoldSettings& settings,
  std::vector<ManifoldRow>& rows,
  std::vector<ManifoldRowPoint>& points
) {
  rows.clear();
  points.clear();

  for (const uint32_t slot : slots) {
    const CachedBodyPair& pair = cache.at(slot);

    for (uint32_t index = 0u; index != pair.manifolds.size(); ++index) {
      const CachedManifold& manifold = pair.manifolds[index];
      const BodyRef& endA = manifold.flipped ? pair.second : pair.first;
      const BodyRef& endB = manifold.flipped ? pair.first : pair.second;
      ManifoldRow row;

      row.slot = slot;
      row.manifold = index;
      row.start = static_cast<uint32_t>(points.size());
      row.count = manifold.count;
      row.a = endB.side == Side::Rigid ? stateOf[endB.index] : 0u;
      row.b = endA.side == Side::Rigid ? stateOf[endA.index] : 0u;

      const RigidState& stateA = states[row.a];
      const RigidState& stateB = states[row.b];
      const double mA = stateA.invMass;
      const glm::dmat3 iA = stateA.invI;
      const double mB = stateB.invMass;
      const glm::dmat3 iB = stateB.invI;

      row.normal = manifold.basis[0];
      row.tangent1 = manifold.basis[1];
      row.tangent2 = manifold.basis[2];
      row.friction = pair.friction;
      row.restitution = pair.restitution;
      row.softness = mA == 0.0 || mB == 0.0 ? settings.fixed : settings.contact;

      glm::dvec3 centerA(0.0);
      glm::dvec3 centerB(0.0);
      double totalWeight = 0.0;
      glm::dvec3 frictionImpulse(0.0);

      for (uint32_t at = 0u; at != manifold.count; ++at) {
        ManifoldRowPoint cp;

        cp.point = manifold.start + at;

        const CachedContactPoint& source = pair.points[cp.point];
        const double s = source.separation;

        if (pair.pose.valid) {
          cp.rB = glm::dvec3(source.anchorB);
          cp.rA = glm::dvec3(source.anchorA);
        } else {
          const glm::dvec3 world =
            contactPoint(bodies, endA, endB, pair.manifoldPoints[cp.point], s, manifold.basis[0]);

          cp.rA = world - centreOf(bodies, endB, endA);
          cp.rB = world - centreOf(bodies, endA, endB);
        }

        cp.baseSeparation = s - glm::dot(cp.rB - cp.rA, row.normal);
        cp.normalImpulse = source.memory.normal * settings.h;

        const glm::dvec3 rnA = glm::cross(cp.rA, row.normal);
        const glm::dvec3 rnB = glm::cross(cp.rB, row.normal);
        const double kNormal = mA + mB + glm::dot(rnA, iA * rnA) + glm::dot(rnB, iB * rnB);

        cp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

        const glm::dvec3 vrA = stateA.v + glm::cross(stateA.w, cp.rA);
        const glm::dvec3 vrB = stateB.v + glm::cross(stateB.w, cp.rB);

        cp.relativeVelocity = glm::dot(row.normal, vrB - vrA);

        const double weight =
          std::clamp(2.0 - s / settings.speculativeDistance, MIN_FRICTION_WEIGHT, 1.0);

        centerA += weight * cp.rA;
        centerB += weight * cp.rB;
        totalWeight += weight;
        frictionImpulse += settings.h
                           * (source.memory.friction.x * manifold.basis[1]
                              + source.memory.friction.y * manifold.basis[2]);
        points.push_back(cp);
      }

      centerA /= totalWeight;
      centerB /= totalWeight;
      row.centerA = centerA;
      row.centerB = centerB;

      for (uint32_t at = 0u; at != row.count; ++at) {
        ManifoldRowPoint& cp = points[row.start + at];

        cp.leverArm = glm::distance(cp.rA, centerA);
      }

      const glm::dvec3 rtA1 = glm::cross(centerA, row.tangent1);
      const glm::dvec3 rtA2 = glm::cross(centerA, row.tangent2);
      const glm::dvec3 rtB1 = glm::cross(centerB, row.tangent1);
      const glm::dvec3 rtB2 = glm::cross(centerB, row.tangent2);
      glm::dmat2 k(0.0);

      k[0][0] = mA + mB + glm::dot(rtA1, iA * rtA1) + glm::dot(rtB1, iB * rtB1);
      k[1][1] = mA + mB + glm::dot(rtA2, iA * rtA2) + glm::dot(rtB2, iB * rtB2);
      k[0][1] = glm::dot(rtA1, iA * rtA2) + glm::dot(rtB1, iB * rtB2);
      k[1][0] = k[0][1];
      row.tangentMass = invert(k);
      row.frictionImpulse = glm::dvec2(
        glm::dot(frictionImpulse, row.tangent1),
        glm::dot(frictionImpulse, row.tangent2)
      );

      const double twist = glm::dot(row.normal, (iA + iB) * row.normal);

      row.twistMass = twist > 0.0 ? 1.0 / twist : 0.0;
      row.twistImpulse = manifold.twist * settings.h;
      rows.push_back(row);
    }
  }
}

void warmStartManifold(
  std::span<RigidState> states,
  const ManifoldRow& row,
  std::span<const ManifoldRowPoint> points
) {
  RigidState& stateA = states[row.a];
  RigidState& stateB = states[row.b];
  double totalNormalImpulse = 0.0;
  glm::dvec3 momentA(0.0);
  glm::dvec3 momentB(0.0);

  for (uint32_t index = 0u; index != row.count; ++index) {
    const ManifoldRowPoint& cp = points[row.start + index];

    totalNormalImpulse += cp.normalImpulse;
    momentA += cp.normalImpulse * cp.rA;
    momentB += cp.normalImpulse * cp.rB;
  }

  const glm::dvec3 frictionImpulse =
    row.frictionImpulse.x * row.tangent1 + row.frictionImpulse.y * row.tangent2;
  const glm::dvec3 linearImpulse = frictionImpulse + totalNormalImpulse * row.normal;
  const glm::dvec3 twistImpulse = row.twistImpulse * row.normal;
  const glm::dvec3 angularA =
    glm::cross(momentA, row.normal) + glm::cross(row.centerA, frictionImpulse) + twistImpulse;
  const glm::dvec3 angularB =
    glm::cross(momentB, row.normal) + glm::cross(row.centerB, frictionImpulse) + twistImpulse;

  if (stateA.invMass != 0.0) {
    stateA.v -= stateA.invMass * linearImpulse;
    stateA.w -= stateA.invI * angularA;
  }

  if (stateB.invMass != 0.0) {
    stateB.v += stateB.invMass * linearImpulse;
    stateB.w += stateB.invI * angularB;
  }
}

void pushManifold(
  std::span<RigidState> states,
  const ManifoldRow& row,
  std::span<ManifoldRowPoint> points,
  double invH,
  double contactSpeed
) {
  RigidState& stateA = states[row.a];
  RigidState& stateB = states[row.b];
  const double mA = stateA.invMass;
  const glm::dmat3 iA = stateA.invI;
  const double mB = stateB.invMass;
  const glm::dmat3 iB = stateB.invI;
  glm::dvec3 vA = stateA.v;
  glm::dvec3 wA = stateA.w;
  glm::dvec3 vB = stateB.v;
  glm::dvec3 wB = stateB.w;
  const glm::dvec3 dp = stateB.dp - stateA.dp;

  for (uint32_t index = 0u; index != row.count; ++index) {
    ManifoldRowPoint& cp = points[row.start + index];
    const glm::dvec3 ds = dp + (stateB.dq * cp.rB - stateA.dq * cp.rA);
    const double s = glm::dot(ds, row.normal) + cp.baseSeparation;
    double velocityBias = 0.0;
    double massScale = 1.0;
    double impulseScale = 0.0;

    if (s > 0.0) {
      velocityBias = s * invH;
    } else {
      velocityBias = std::max(row.softness.massScale * row.softness.biasRate * s, -contactSpeed);
      massScale = row.softness.massScale;
      impulseScale = row.softness.impulseScale;
    }

    const glm::dvec3 vrA = vA + glm::cross(wA, cp.rA);
    const glm::dvec3 vrB = vB + glm::cross(wB, cp.rB);
    const double vn = glm::dot(vrB - vrA, row.normal);
    double deltaImpulse =
      -cp.normalMass * (massScale * vn + velocityBias) - impulseScale * cp.normalImpulse;
    const double newImpulse = std::max(cp.normalImpulse + deltaImpulse, 0.0);

    deltaImpulse = newImpulse - cp.normalImpulse;
    cp.normalImpulse = newImpulse;

    const glm::dvec3 P = deltaImpulse * row.normal;

    vA -= mA * P;
    wA -= iA * glm::cross(cp.rA, P);
    vB += mB * P;
    wB += iB * glm::cross(cp.rB, P);
  }

  storeState(stateA, vA, wA);
  storeState(stateB, vB, wB);
}

void relaxManifold(
  std::span<RigidState> states,
  ManifoldRow& row,
  std::span<ManifoldRowPoint> points,
  double invH
) {
  RigidState& stateA = states[row.a];
  RigidState& stateB = states[row.b];
  const double mA = stateA.invMass;
  const glm::dmat3 iA = stateA.invI;
  const double mB = stateB.invMass;
  const glm::dmat3 iB = stateB.invI;
  glm::dvec3 vA = stateA.v;
  glm::dvec3 wA = stateA.w;
  glm::dvec3 vB = stateB.v;
  glm::dvec3 wB = stateB.w;
  const glm::dvec3 dp = stateB.dp - stateA.dp;
  double totalNormalImpulse = 0.0;
  double totalTwistLimit = 0.0;

  for (uint32_t index = 0u; index != row.count; ++index) {
    ManifoldRowPoint& cp = points[row.start + index];
    const glm::dvec3 ds = dp + (stateB.dq * cp.rB - stateA.dq * cp.rA);
    const double s = glm::dot(ds, row.normal) + cp.baseSeparation;
    const double velocityBias = s > 0.0 ? s * invH : 0.0;
    const glm::dvec3 vrA = vA + glm::cross(wA, cp.rA);
    const glm::dvec3 vrB = vB + glm::cross(wB, cp.rB);
    const double vn = glm::dot(vrB - vrA, row.normal);
    double deltaImpulse = -cp.normalMass * (vn + velocityBias);
    const double newImpulse = std::max(cp.normalImpulse + deltaImpulse, 0.0);

    deltaImpulse = newImpulse - cp.normalImpulse;
    cp.normalImpulse = newImpulse;
    cp.totalNormalImpulse += newImpulse;
    totalNormalImpulse += newImpulse;
    totalTwistLimit += cp.leverArm * newImpulse;

    const glm::dvec3 P = deltaImpulse * row.normal;

    vA -= mA * P;
    wA -= iA * glm::cross(cp.rA, P);
    vB += mB * P;
    wB += iB * glm::cross(cp.rB, P);
  }

  {
    const double twistSpeed = glm::dot(row.normal, wB - wA);
    const double maxImpulse = row.friction * totalTwistLimit;
    const double oldImpulse = row.twistImpulse;

    row.twistImpulse =
      std::clamp(oldImpulse - row.twistMass * twistSpeed, -maxImpulse, maxImpulse);

    const double deltaImpulse = row.twistImpulse - oldImpulse;

    wA -= iA * (deltaImpulse * row.normal);
    wB += iB * (deltaImpulse * row.normal);
  }

  {
    const glm::dvec3 rA = row.centerA;
    const glm::dvec3 rB = row.centerB;
    const glm::dvec3 vr = (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA));
    const glm::dvec2 vt(glm::dot(vr, row.tangent1), glm::dot(vr, row.tangent2));
    const glm::dvec2 deltaImpulse = -(row.tangentMass * vt);
    glm::dvec2 newImpulse = row.frictionImpulse + deltaImpulse;
    const double maxImpulse = row.friction * totalNormalImpulse;
    const double lengthSquared = glm::dot(newImpulse, newImpulse);

    if (lengthSquared > maxImpulse * maxImpulse) {
      newImpulse *= maxImpulse / std::sqrt(lengthSquared);
    }

    const glm::dvec2 applied = newImpulse - row.frictionImpulse;

    row.frictionImpulse = newImpulse;

    const glm::dvec3 P = applied.x * row.tangent1 + applied.y * row.tangent2;

    vA -= mA * P;
    wA -= iA * glm::cross(rA, P);
    vB += mB * P;
    wB += iB * glm::cross(rB, P);
  }

  storeState(stateA, vA, wA);
  storeState(stateB, vB, wB);
}

void restituteManifold(
  std::span<RigidState> states,
  const ManifoldRow& row,
  std::span<ManifoldRowPoint> points,
  double invH,
  double threshold
) {
  if (row.restitution == 0.0) {
    return;
  }

  RigidState& stateA = states[row.a];
  RigidState& stateB = states[row.b];
  const double mA = stateA.invMass;
  const glm::dmat3 iA = stateA.invI;
  const double mB = stateB.invMass;
  const glm::dmat3 iB = stateB.invI;
  glm::dvec3 vA = stateA.v;
  glm::dvec3 wA = stateA.w;
  glm::dvec3 vB = stateB.v;
  glm::dvec3 wB = stateB.w;
  const glm::dvec3 dp = stateB.dp - stateA.dp;

  for (uint32_t index = 0u; index != row.count; ++index) {
    ManifoldRowPoint& cp = points[row.start + index];
    const double compressionImpulse = cp.totalNormalImpulse - cp.restitutionImpulse;
    const bool armed = row.restitution > 0.0 && cp.relativeVelocity < -threshold
                       && compressionImpulse > 0.0;
    double velocityBias = 0.0;

    if (armed) {
      velocityBias = row.restitution * cp.relativeVelocity;
    } else {
      const glm::dvec3 ds = dp + (stateB.dq * cp.rB - stateA.dq * cp.rA);
      const double s = glm::dot(ds, row.normal) + cp.baseSeparation;

      velocityBias = s > 0.0 ? s * invH : 0.0;
    }

    const glm::dvec3 vrA = vA + glm::cross(wA, cp.rA);
    const glm::dvec3 vrB = vB + glm::cross(wB, cp.rB);
    const double vn = glm::dot(vrB - vrA, row.normal);
    double impulse = -cp.normalMass * (vn + velocityBias);
    const double newImpulse = std::max(cp.normalImpulse + impulse, 0.0);

    impulse = newImpulse - cp.normalImpulse;

    const double approachImpulse =
      std::min(std::max(-cp.normalMass * vn, 0.0), std::max(impulse, 0.0));

    if (armed) {
      const double allowance =
        row.restitution * (compressionImpulse + approachImpulse) - cp.restitutionImpulse;

      impulse = std::min(impulse, approachImpulse + std::max(allowance, 0.0));
    }

    cp.normalImpulse += impulse;
    cp.restitutionImpulse += impulse - approachImpulse;
    cp.totalNormalImpulse += impulse;

    const glm::dvec3 P = impulse * row.normal;

    vA -= mA * P;
    wA -= iA * glm::cross(cp.rA, P);
    vB += mB * P;
    wB += iB * glm::cross(cp.rB, P);
  }

  storeState(stateA, vA, wA);
  storeState(stateB, vB, wB);
}

void storeManifolds(
  ManifoldCache& cache,
  std::span<const ManifoldRow> rows,
  std::span<const ManifoldRowPoint> points,
  double h
) {
  for (const ManifoldRow& row : rows) {
    CachedBodyPair& pair = cache.at(row.slot);
    const CachedManifold& manifold = pair.manifolds[row.manifold];
    const glm::dvec3 friction = row.frictionImpulse.x * row.tangent1 + row.frictionImpulse.y * row.tangent2;
    const glm::dvec3 share = friction / (h * static_cast<double>(row.count));

    pair.manifolds[row.manifold].twist = row.twistImpulse / h;

    for (uint32_t index = 0u; index != row.count; ++index) {
      const ManifoldRowPoint& cp = points[row.start + index];
      CachedContactPoint& target = pair.points[cp.point];

      target.memory.normal = cp.normalImpulse / h;
      target.memory.approach = -cp.relativeVelocity;
      target.memory.impulse = cp.totalNormalImpulse;
      target.memory.friction = glm::dvec2(
        glm::dot(share, manifold.basis[1]),
        glm::dot(share, manifold.basis[2])
      );
    }
  }
}

}
