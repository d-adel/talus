#include "softstep.h"
#include "widecontact.h"

#include <talus/dynamics/joints/jointrows.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/scheduler/stopwatch.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <vector>

namespace talus::dynamics {

namespace {

constexpr float minFrictionWeight = 1.0e-10f;
constexpr float singular = 1000.0f * std::numeric_limits<float>::min();
constexpr uint32_t batchItem = 0x80000000u;
constexpr uint32_t noLane = 0xFFFFFFFFu;
constexpr uint32_t noFacing = 0xFFFFFFFFu;

struct Softness {
  float biasRate = 0.0f;
  float massScale = 0.0f;
  float impulseScale = 0.0f;
};

struct State {
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  glm::vec3 dp{0.0f};
  glm::quat dq{1.0f, 0.0f, 0.0f, 0.0f};
  float invMass = 0.0f;
  glm::mat3 invI{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 moment{0.0f};
};

struct Point {
  uint32_t point = 0u;
  glm::vec3 rA{0.0f};
  glm::vec3 rB{0.0f};
  float baseSeparation = 0.0f;
  float normalImpulse = 0.0f;
  float totalNormalImpulse = 0.0f;
  float restitutionImpulse = 0.0f;
  float normalMass = 0.0f;
  float relativeVelocity = 0.0f;
  float leverArm = 0.0f;
  float anchorRadius = 0.0f;
  float anchorSpread = 0.0f;
};

struct Constraint {
  uint32_t a = 0u;
  uint32_t b = 0u;
  uint32_t slot = 0u;
  uint32_t manifold = 0u;
  uint32_t start = 0u;
  uint32_t count = 0u;
  uint32_t facing = noFacing;
  glm::vec3 normal{0.0f};
  glm::vec3 tangent1{0.0f};
  glm::vec3 tangent2{0.0f};
  glm::vec3 centerA{0.0f};
  glm::vec3 centerB{0.0f};
  glm::mat2 tangentMass{0.0f};
  glm::vec2 frictionImpulse{0.0f};
  float twistMass = 0.0f;
  float twistImpulse = 0.0f;
  float friction = 0.0f;
  float restitution = 0.0f;
  Softness softness;
};

Softness makeSoft(float hertz, float zeta, float h) {
  if (hertz == 0.0f) {
    return {};
  }

  const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
  const float a1 = 2.0f * zeta + h * omega;
  const float a2 = h * omega * a1;
  const float a3 = 1.0f / (1.0f + a2);

  return {omega / a1, a2 * a3, a3};
}

glm::mat2 invert(const glm::mat2& m) {
  const float det = m[0][0] * m[1][1] - m[1][0] * m[0][1];

  if (std::abs(det) <= singular) {
    return glm::mat2(0.0f);
  }

  const float invDet = 1.0f / det;

  return glm::mat2(
    glm::vec2(invDet * m[1][1], -invDet * m[0][1]),
    glm::vec2(-invDet * m[1][0], invDet * m[0][0])
  );
}

glm::vec3 solve3(const glm::mat3& m, const glm::vec3& a) {
  const float det = glm::dot(m[0], glm::cross(m[1], m[2]));

  if (std::abs(det) <= singular) {
    return glm::vec3(0.0f);
  }

  const float invDet = 1.0f / det;

  return glm::vec3(
    invDet * glm::dot(glm::cross(m[1], m[2]), a),
    invDet * glm::dot(glm::cross(m[2], m[0]), a),
    invDet * glm::dot(glm::cross(m[0], m[1]), a)
  );
}

glm::quat integrateRotation(const glm::quat& q1, const glm::vec3& delta) {
  glm::quat qd(0.0f, 0.5f * delta.x, 0.5f * delta.y, 0.5f * delta.z);

  qd = qd * q1;

  return glm::normalize(
    glm::quat(q1.w + qd.w, q1.x + qd.x, q1.y + qd.y, q1.z + qd.z)
  );
}

void integrateVelocity(State& state, const glm::vec3& gravity, float h) {
  if (state.invMass == 0.0f) {
    return;
  }

  state.v += h * gravity;

  const glm::quat q = state.dq * state.rotation;
  glm::mat3 inertia(0.0f);

  inertia[0][0] = state.moment.x;
  inertia[1][1] = state.moment.y;
  inertia[2][2] = state.moment.z;

  const glm::vec3 omega1 = glm::conjugate(q) * state.w;
  glm::vec3 omega2 = omega1;

  const float i00 = inertia[0][0];
  const float i01 = inertia[1][0];
  const float i02 = inertia[2][0];
  const float i11 = inertia[1][1];
  const float i12 = inertia[2][1];
  const float i22 = inertia[2][2];

  const float w1 = omega2.x;
  const float w2 = omega2.y;
  const float w3 = omega2.z;

  const float Iw1 = i00 * w1 + i01 * w2 + i02 * w3;
  const float Iw2 = i01 * w1 + i11 * w2 + i12 * w3;
  const float Iw3 = i02 * w1 + i12 * w2 + i22 * w3;

  const glm::vec3 dw = omega2 - omega1;
  const glm::vec3 b(
    i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * Iw3 - w3 * Iw2),
    i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * Iw1 - w1 * Iw3),
    i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * Iw2 - w2 * Iw1)
  );

  const glm::mat3 J(
    glm::vec3(
      i00 + h * (w2 * i02 - w3 * i01),
      i01 + h * (w3 * i00 - w1 * i02 - Iw3),
      i02 + h * (w1 * i01 - w2 * i00 + Iw2)
    ),
    glm::vec3(
      i01 + h * (w2 * i12 - w3 * i11 + Iw3),
      i11 + h * (w3 * i01 - w1 * i12),
      i12 + h * (w1 * i11 - w2 * i01 - Iw1)
    ),
    glm::vec3(
      i02 + h * (w2 * i22 - w3 * i12 - Iw2),
      i12 + h * (w3 * i02 - w1 * i22 + Iw1),
      i22 + h * (w1 * i12 - w2 * i02)
    )
  );

  omega2 -= solve3(J, b);
  state.w = q * omega2;
}

void integratePosition(
  State& state,
  float h,
  float maxLinearSpeed,
  float maxAngularSpeed
) {
  glm::vec3 v = state.v;
  glm::vec3 w = state.w;

  if (glm::dot(v, v) > maxLinearSpeed * maxLinearSpeed) {
    v *= maxLinearSpeed / glm::length(v);
  }

  if (glm::dot(w, w) > maxAngularSpeed * maxAngularSpeed) {
    w *= maxAngularSpeed / glm::length(w);
  }

  state.v = v;
  state.w = w;
  state.dp += h * v;
  state.dq = integrateRotation(state.dq, h * w);
}

uint32_t stateIndex(std::span<const uint32_t> stateOf, const BodyRef& ref) {
  return ref.side == Side::Rigid ? stateOf[ref.index] : 0u;
}

glm::dvec3 centreOf(
  const BodySet& bodies,
  const BodyRef& side,
  const BodyRef& other
) {
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

    return body.position + glm::mat3_cast(body.rotation) * point.localA
           - 0.5 * separation * normal;
  }

  if (b.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[b.index];

    return body.position + glm::mat3_cast(body.rotation) * point.localB
           + 0.5 * separation * normal;
  }

  return glm::dvec3(0.0);
}

void warmStart(
  std::vector<State>& states,
  const Constraint& constraint,
  const std::vector<Point>& points
) {
  State& stateA = states[constraint.a];
  State& stateB = states[constraint.b];
  const glm::vec3 normal = constraint.normal;
  float totalNormalImpulse = 0.0f;
  glm::vec3 momentA(0.0f);
  glm::vec3 momentB(0.0f);

  for (uint32_t index = 0u; index != constraint.count; ++index) {
    const Point& cp = points[constraint.start + index];

    totalNormalImpulse += cp.normalImpulse;
    momentA += cp.normalImpulse * cp.rA;
    momentB += cp.normalImpulse * cp.rB;
  }

  const glm::vec3 frictionImpulse =
    constraint.frictionImpulse.x * constraint.tangent1
    + constraint.frictionImpulse.y * constraint.tangent2;
  const glm::vec3 linearImpulse = frictionImpulse + totalNormalImpulse * normal;
  const glm::vec3 twistImpulse = constraint.twistImpulse * normal;
  const glm::vec3 angularA = glm::cross(momentA, normal)
                             + glm::cross(constraint.centerA, frictionImpulse)
                             + twistImpulse;
  const glm::vec3 angularB = glm::cross(momentB, normal)
                             + glm::cross(constraint.centerB, frictionImpulse)
                             + twistImpulse;

  if (stateA.invMass != 0.0f) {
    stateA.v -= stateA.invMass * linearImpulse;
    stateA.w -= stateA.invI * angularA;
  }

  if (stateB.invMass != 0.0f) {
    stateB.v += stateB.invMass * linearImpulse;
    stateB.w += stateB.invI * angularB;
  }
}

bool speculative(float separation) {
  return separation > 0.0f;
}

float speculativeBias(float separation, float invH) {
  return speculative(separation) ? separation * invH : 0.0f;
}

void push(
  std::vector<State>& states,
  const Constraint& constraint,
  std::vector<Point>& points,
  float invH,
  float contactSpeed
) {
  State& stateA = states[constraint.a];
  State& stateB = states[constraint.b];
  const float mA = stateA.invMass;
  const glm::mat3 iA = stateA.invI;
  const float mB = stateB.invMass;
  const glm::mat3 iB = stateB.invI;
  glm::vec3 vA = stateA.v;
  glm::vec3 wA = stateA.w;
  glm::vec3 vB = stateB.v;
  glm::vec3 wB = stateB.w;
  const glm::vec3 dp = stateB.dp - stateA.dp;
  const glm::vec3 normal = constraint.normal;
  const Softness softness = constraint.softness;

  for (uint32_t index = 0u; index != constraint.count; ++index) {
    Point& cp = points[constraint.start + index];
    const glm::vec3 ds = dp + (stateB.dq * cp.rB - stateA.dq * cp.rA);
    const float s = glm::dot(ds, normal) + cp.baseSeparation;
    float velocityBias = 0.0f;
    float massScale = 1.0f;
    float impulseScale = 0.0f;

    if (speculative(s)) {
      velocityBias = speculativeBias(s, invH);
    } else {
      velocityBias =
        std::max(softness.massScale * softness.biasRate * s, -contactSpeed);
      massScale = softness.massScale;
      impulseScale = softness.impulseScale;
    }

    const glm::vec3 vrA = vA + glm::cross(wA, cp.rA);
    const glm::vec3 vrB = vB + glm::cross(wB, cp.rB);
    const float vn = glm::dot(vrB - vrA, normal);
    float deltaImpulse = -cp.normalMass * (massScale * vn + velocityBias)
                         - impulseScale * cp.normalImpulse;
    const float newImpulse = std::max(cp.normalImpulse + deltaImpulse, 0.0f);

    deltaImpulse = newImpulse - cp.normalImpulse;
    cp.normalImpulse = newImpulse;

    const glm::vec3 P = deltaImpulse * normal;

    vA -= mA * P;
    wA -= iA * glm::cross(cp.rA, P);
    vB += mB * P;
    wB += iB * glm::cross(cp.rB, P);
  }

  if (stateA.invMass != 0.0f) {
    stateA.v = vA;
    stateA.w = wA;
  }

  if (stateB.invMass != 0.0f) {
    stateB.v = vB;
    stateB.w = wB;
  }
}

void relax(
  std::vector<State>& states,
  Constraint& constraint,
  std::vector<Point>& points,
  float invH
) {
  State& stateA = states[constraint.a];
  State& stateB = states[constraint.b];
  const float mA = stateA.invMass;
  const glm::mat3 iA = stateA.invI;
  const float mB = stateB.invMass;
  const glm::mat3 iB = stateB.invI;
  glm::vec3 vA = stateA.v;
  glm::vec3 wA = stateA.w;
  glm::vec3 vB = stateB.v;
  glm::vec3 wB = stateB.w;
  const glm::vec3 dp = stateB.dp - stateA.dp;
  const glm::vec3 normal = constraint.normal;
  const float friction = constraint.friction;
  float totalNormalImpulse = 0.0f;
  float totalTwistLimit = 0.0f;

  for (uint32_t index = 0u; index != constraint.count; ++index) {
    Point& cp = points[constraint.start + index];
    const glm::vec3 ds = dp + (stateB.dq * cp.rB - stateA.dq * cp.rA);
    const float s = glm::dot(ds, normal) + cp.baseSeparation;
    const float velocityBias = speculativeBias(s, invH);
    const glm::vec3 vrA = vA + glm::cross(wA, cp.rA);
    const glm::vec3 vrB = vB + glm::cross(wB, cp.rB);
    const float vn = glm::dot(vrB - vrA, normal);
    float deltaImpulse = -cp.normalMass * (vn + velocityBias);
    const float newImpulse = std::max(cp.normalImpulse + deltaImpulse, 0.0f);

    deltaImpulse = newImpulse - cp.normalImpulse;
    cp.normalImpulse = newImpulse;
    cp.totalNormalImpulse += newImpulse;
    totalNormalImpulse += newImpulse;
    totalTwistLimit += cp.leverArm * newImpulse;

    const glm::vec3 P = deltaImpulse * normal;

    vA -= mA * P;
    wA -= iA * glm::cross(cp.rA, P);
    vB += mB * P;
    wB += iB * glm::cross(cp.rB, P);
  }

  {
    const float twistSpeed = glm::dot(normal, wB - wA);
    const float maxImpulse = friction * totalTwistLimit;
    const float oldImpulse = constraint.twistImpulse;

    constraint.twistImpulse = std::clamp(
      oldImpulse - constraint.twistMass * twistSpeed,
      -maxImpulse,
      maxImpulse
    );

    const float deltaImpulse = constraint.twistImpulse - oldImpulse;

    wA -= iA * (deltaImpulse * normal);
    wB += iB * (deltaImpulse * normal);
  }

  {
    const glm::vec3 rA = constraint.centerA;
    const glm::vec3 rB = constraint.centerB;
    const glm::vec3 vr = (vB + glm::cross(wB, rB)) - (vA + glm::cross(wA, rA));
    const glm::vec2 vt(
      glm::dot(vr, constraint.tangent1),
      glm::dot(vr, constraint.tangent2)
    );
    const glm::vec2 deltaImpulse = -(constraint.tangentMass * vt);
    glm::vec2 newImpulse = constraint.frictionImpulse + deltaImpulse;
    const float maxImpulse = friction * totalNormalImpulse;
    const float lengthSquared = glm::dot(newImpulse, newImpulse);

    if (lengthSquared > maxImpulse * maxImpulse) {
      newImpulse *= maxImpulse / std::sqrt(lengthSquared);
    }

    const glm::vec2 applied = newImpulse - constraint.frictionImpulse;

    constraint.frictionImpulse = newImpulse;

    const glm::vec3 P =
      applied.x * constraint.tangent1 + applied.y * constraint.tangent2;

    vA -= mA * P;
    wA -= iA * glm::cross(rA, P);
    vB += mB * P;
    wB += iB * glm::cross(rB, P);
  }

  if (stateA.invMass != 0.0f) {
    stateA.v = vA;
    stateA.w = wA;
  }

  if (stateB.invMass != 0.0f) {
    stateB.v = vB;
    stateB.w = wB;
  }
}

void restitute(
  std::vector<State>& states,
  const Constraint& constraint,
  std::vector<Point>& points,
  float invH,
  float threshold
) {
  const float restitution = constraint.restitution;

  if (restitution == 0.0f) {
    return;
  }

  State& stateA = states[constraint.a];
  State& stateB = states[constraint.b];
  const float mA = stateA.invMass;
  const glm::mat3 iA = stateA.invI;
  const float mB = stateB.invMass;
  const glm::mat3 iB = stateB.invI;
  glm::vec3 vA = stateA.v;
  glm::vec3 wA = stateA.w;
  glm::vec3 vB = stateB.v;
  glm::vec3 wB = stateB.w;
  const glm::vec3 dp = stateB.dp - stateA.dp;
  const glm::vec3 normal = constraint.normal;

  for (uint32_t index = 0u; index != constraint.count; ++index) {
    Point& cp = points[constraint.start + index];
    const float compressionImpulse =
      cp.totalNormalImpulse - cp.restitutionImpulse;
    const bool armed = restitution > 0.0f && cp.relativeVelocity < -threshold
                       && compressionImpulse > 0.0f;
    float velocityBias = 0.0f;

    if (armed) {
      velocityBias = restitution * cp.relativeVelocity;
    } else {
      const glm::vec3 ds = dp + (stateB.dq * cp.rB - stateA.dq * cp.rA);
      const float s = glm::dot(ds, normal) + cp.baseSeparation;

      velocityBias = speculativeBias(s, invH);
    }

    const glm::vec3 vrA = vA + glm::cross(wA, cp.rA);
    const glm::vec3 vrB = vB + glm::cross(wB, cp.rB);
    const float vn = glm::dot(vrB - vrA, normal);
    float impulse = -cp.normalMass * (vn + velocityBias);
    const float newImpulse = std::max(cp.normalImpulse + impulse, 0.0f);

    impulse = newImpulse - cp.normalImpulse;

    const float approachImpulse = std::min(
      std::max(-cp.normalMass * vn, 0.0f),
      std::max(impulse, 0.0f)
    );

    if (armed) {
      const float allowance =
        restitution * (compressionImpulse + approachImpulse)
        - cp.restitutionImpulse;

      impulse = std::min(impulse, approachImpulse + std::max(allowance, 0.0f));
    }

    cp.normalImpulse += impulse;
    cp.restitutionImpulse += impulse - approachImpulse;
    cp.totalNormalImpulse += impulse;

    const glm::vec3 P = impulse * normal;

    vA -= mA * P;
    wA -= iA * glm::cross(cp.rA, P);
    vB += mB * P;
    wB += iB * glm::cross(cp.rB, P);
  }

  if (stateA.invMass != 0.0f) {
    stateA.v = vA;
    stateA.w = wA;
  }

  if (stateB.invMass != 0.0f) {
    stateB.v = vB;
    stateB.w = wB;
  }
}

bool idle(const Constraint& constraint, const std::vector<Point>& points) {
  if (constraint.frictionImpulse.x != 0.0f
      || constraint.frictionImpulse.y != 0.0f
      || constraint.twistImpulse != 0.0f) {
    return false;
  }

  for (uint32_t index = 0u; index != constraint.count; ++index) {
    if (points[constraint.start + index].normalImpulse != 0.0f) {
      return false;
    }
  }

  return true;
}

struct Facing {
  uint32_t a = 0u;
  uint32_t b = 0u;
  glm::vec3 offset{0.0f};
  float nearest = std::numeric_limits<float>::infinity();
  float anchorRadius = 0.0f;
  float anchorSpread = 0.0f;
  float base = 0.0f;
};

struct Calm {
  std::array<Facing, 2> facings;
  uint32_t count = 0u;
  bool idle = false;
  bool mixed = false;
};

struct Drift {
  bool placed = false;
  bool paced = false;
  glm::vec3 shift{0.0f};
  float turn = 0.0f;
  float stretch = 0.0f;
  float span = 0.0f;
  glm::vec3 closing{0.0f};
  float spin = 0.0f;
  float spinB = 0.0f;
  float pace = 0.0f;
  float spins = 0.0f;
};

constexpr float driftMargin = 1.0e-5f;
constexpr float driftRate = 1.0f - driftMargin;

bool cannotPush(float separationBelow, float speedBelow, float invH) {
  return speculative(separationBelow)
         && speedBelow + driftRate * speculativeBias(separationBelow, invH)
              > 0.0f;
}

const Drift& driftOf(
  const std::vector<State>& states,
  const Facing& facing,
  Drift& drift
) {
  const State& stateA = states[facing.a];
  const State& stateB = states[facing.b];

  if (!drift.placed) {
    const glm::quat turned = glm::conjugate(stateA.dq) * stateB.dq;

    drift.shift = stateB.dp - stateA.dp + stateB.dq * facing.offset;
    drift.turn = 2.0f * glm::length(glm::vec3(turned.x, turned.y, turned.z));
    drift.stretch = glm::length(facing.offset);
    drift.span = 1.0f + glm::length(stateA.dp) + glm::length(stateB.dp)
                 + 2.0f * drift.stretch;
    drift.placed = true;
  }

  if (!drift.paced) {
    drift.closing = stateB.v - stateA.v + glm::cross(stateB.w, facing.offset);
    drift.spin = glm::length(stateB.w - stateA.w);
    drift.spinB = glm::length(stateB.w);
    drift.pace = 1.0f + glm::length(stateA.v) + glm::length(stateB.v);
    drift.spins = glm::length(stateA.w) + glm::length(stateB.w);
    drift.paced = true;
  }

  return drift;
}

bool dormant(
  const Constraint& constraint,
  const std::vector<Point>& points,
  const Drift& drift,
  float invH
) {
  const float approach = glm::dot(constraint.normal, drift.closing);
  const float shift = glm::dot(constraint.normal, drift.shift);

  for (uint32_t index = 0u; index != constraint.count; ++index) {
    const Point& cp = points[constraint.start + index];
    const float radius = cp.anchorRadius;
    const float spread = cp.anchorSpread;
    const float gap =
      cp.baseSeparation + shift - spread - drift.turn * radius
      - driftMargin
          * (drift.span + 2.0f * radius + std::abs(cp.baseSeparation));
    const float speed =
      approach - drift.spinB * spread - drift.spin * radius
      - driftMargin
          * (drift.pace + drift.spins * (radius + drift.stretch + spread));

    if (!cannotPush(gap, speed, invH)) {
      return false;
    }
  }

  return true;
}

bool calmPair(
  const std::vector<State>& states,
  const Calm& calm,
  std::array<Drift, 2>& drifts,
  float invH
) {
  if (!calm.idle || calm.mixed) {
    return false;
  }

  for (uint32_t index = 0u; index != calm.count; ++index) {
    const Facing& facing = calm.facings[index];
    const Drift& drift = driftOf(states, facing, drifts[index]);
    const float radius = facing.anchorRadius;
    const float spread = facing.anchorSpread;
    const float gap =
      facing.nearest - glm::length(drift.shift - facing.offset)
      - drift.turn * radius
      - driftMargin * (drift.span + 2.0f * radius + facing.base);
    const float speed =
      -glm::length(drift.closing) - drift.spinB * spread - drift.spin * radius
      - driftMargin
          * (drift.pace + drift.spins * (radius + drift.stretch + spread));

    if (!cannotPush(gap, speed, invH)) {
      return false;
    }
  }

  return true;
}

uint32_t floatOffset(const State& state, const void* member) {
  return static_cast<uint32_t>(
    (static_cast<const char*>(member) - reinterpret_cast<const char*>(&state))
    / static_cast<std::ptrdiff_t>(sizeof(float))
  );
}

}

struct SoftstepArena {
  std::vector<State> states;
  std::vector<uint32_t> rigidOf;
  std::vector<uint32_t> ordered;
  std::vector<Constraint> constraints;
  std::vector<Point> points;
  std::vector<Calm> calms;
  std::vector<uint32_t> pairFirst;
  std::vector<uint32_t> pointFirst;
  std::vector<uint32_t> colourItems;
  std::vector<uint32_t> laneSlot;
  std::vector<uint32_t> wideSlots;
  std::vector<WideContact<4u>> batches4;
  std::vector<WideContact<8u>> batches8;
  std::vector<JointRow<float>> jointRows;
  StageRunner runner;
};

std::shared_ptr<SoftstepArena> makeSoftstepArena() {
  return std::make_shared<SoftstepArena>();
}

SoftstepResult solveSoftstep(
  BodySet& bodies,
  ManifoldCache& cache,
  std::span<const uint32_t> rigids,
  const IntegrationParameters& parameters,
  std::vector<uint32_t>& stateOf,
  const Workers& workers,
  uint32_t width,
  SoftstepArena* arena,
  StepProfile* profile,
  SoftstepTouching* touching,
  const std::function<void()>* alongside,
  JointSet* joints,
  const IslandStore* islandStore,
  std::span<const uint32_t> guests,
  bool partial,
  bool jointed
) {
  const uint32_t substeps = std::max(parameters.softstepSubsteps, 1u);
  const double step = parameters.dt / static_cast<double>(substeps);
  const float dt = static_cast<float>(parameters.dt);
  const float h = static_cast<float>(step);
  const float invH = static_cast<float>(substeps) / dt;
  const glm::vec3 gravity(parameters.gravity);
  const float zeta = static_cast<float>(parameters.contactDampingRatio);
  const float hertz =
    std::min(static_cast<float>(parameters.contactHertz), 0.125f * invH);
  const Softness contactSoftness = makeSoft(hertz, zeta, h);
  const Softness staticSoftness = makeSoft(2.0f * hertz, 0.5f * zeta, h);
  const float contactSpeed = static_cast<float>(parameters.contactSpeed);
  const float maxLinearSpeed = static_cast<float>(parameters.maxLinearSpeed);
  const float maxAngularSpeed =
    0.25f * std::numbers::pi_v<float>
    * (jointed ? static_cast<float>(substeps) : 1.0f) / dt;
  const float threshold = static_cast<float>(parameters.restitutionThreshold);
  const float speculativeDistance =
    static_cast<float>(parameters.speculativeDistance);

  if (stateOf.size() < bodies.rigids.size()) {
    stateOf.resize(bodies.rigids.size(), 0u);
  }

  SoftstepArena local;
  SoftstepArena& scratch = arena != nullptr ? *arena : local;
  std::vector<State>& states = scratch.states;
  std::vector<uint32_t>& rigidOf = scratch.rigidOf;
  const uint32_t ownedCount = static_cast<uint32_t>(rigids.size());
  const uint32_t rigidCount = ownedCount + static_cast<uint32_t>(guests.size());

  states.resize(rigidCount + 1u);
  rigidOf.resize(rigidCount + 1u);
  states[0] = State{};
  rigidOf[0] = 0u;

  for (uint32_t entry = 0u; entry != rigidCount; ++entry) {
    const uint32_t index =
      entry < ownedCount ? rigids[entry] : guests[entry - ownedCount];

    rigidOf[entry + 1u] = index;
    stateOf[index] = entry + 1u;
  }

  const auto gather = [&](uint32_t item) {
    const RigidBody& body = bodies.rigids[rigidOf[item]];
    State state;

    state.v = glm::vec3(body.velocity);
    state.w = glm::vec3(body.angularVelocity);
    state.rotation = glm::quat(body.rotation);

    if (body.mass > 0.0) {
      const glm::dmat3 turn = glm::mat3_cast(body.rotation);
      glm::dmat3 inverse(0.0);

      for (int axis = 0; axis != 3; ++axis) {
        inverse[axis][axis] =
          body.moment[axis] > 0.0 ? 1.0 / body.moment[axis] : 0.0;
      }

      state.invMass = static_cast<float>(1.0 / body.mass);
      state.invI = glm::mat3(turn * inverse * glm::transpose(turn));
      state.moment = glm::vec3(body.moment);
    }

    states[item] = state;
  };

  const auto solved = [&](const BodyRef& end) {
    return end.side == Side::Rigid && stateIndex(stateOf, end) != 0u;
  };
  const auto owned = [&](const BodyRef& end) {
    return solved(end) && stateIndex(stateOf, end) <= ownedCount;
  };
  const auto held = [&](const BodyRef& end) {
    return end.side != Side::Rigid || solved(end) || !(bodies.rigids[end.index].mass > 0.0);
  };
  const auto active = [&](uint32_t slot) {
    const CachedBodyPair& pair = cache.at(slot);

    return (owned(pair.first) || owned(pair.second)) && held(pair.first)
           && held(pair.second);
  };
  const uint32_t batchWidth = width == 4u || width == 8u ? width : 0u;
  std::vector<uint32_t>& ordered = scratch.ordered;
  std::array<uint32_t, colourCount + 1u> colourStart{};
  bool anyRestitution = false;

  ordered.clear();

  for (uint32_t order = 0u; order != colourCount; ++order) {
    const uint32_t colour = order == 0u ? overflowColour : order - 1u;

    colourStart[order] = static_cast<uint32_t>(ordered.size());

    if (batchWidth == 0u) {
      for (const uint32_t slot : cache.widePairs(colour)) {
        if (active(slot)) {
          ordered.push_back(slot);
        }
      }
    }

    for (const uint32_t slot : cache.otherPairs(colour)) {
      if (active(slot)) {
        ordered.push_back(slot);
      }
    }
  }

  colourStart[colourCount] = static_cast<uint32_t>(ordered.size());

  std::vector<uint32_t>& wideSlots = scratch.wideSlots;
  std::array<uint32_t, colourCount + 1u> wideBegin{};

  wideSlots.clear();

  if (partial && batchWidth != 0u) {
    for (uint32_t colour = 0u; colour != colourCount; ++colour) {
      wideBegin[colour] = static_cast<uint32_t>(wideSlots.size());

      for (const uint32_t slot : cache.widePairs(colour)) {
        if (active(slot)) {
          wideSlots.push_back(slot);
        }
      }
    }

    wideBegin[colourCount] = static_cast<uint32_t>(wideSlots.size());
  }

  const auto wideList = [&](uint32_t colour) {
    if (!partial) {
      return cache.widePairs(colour);
    }

    return std::span<const uint32_t>(wideSlots).subspan(
      wideBegin[colour],
      wideBegin[colour + 1u] - wideBegin[colour]
    );
  };

  std::vector<Constraint>& constraints = scratch.constraints;
  std::vector<Point>& points = scratch.points;
  std::vector<Calm>& calms = scratch.calms;
  std::vector<uint32_t>& pairFirst = scratch.pairFirst;
  std::vector<uint32_t>& pointFirst = scratch.pointFirst;
  std::array<uint32_t, colourCount + 1u> constraintStart{};
  const uint32_t pairCount = static_cast<uint32_t>(ordered.size());
  uint32_t constraintTotal = 0u;
  uint32_t pointTotal = 0u;

  pairFirst.resize(pairCount + 1u);
  pointFirst.resize(pairCount + 1u);

  for (uint32_t pairIndex = 0u; pairIndex != pairCount; ++pairIndex) {
    const CachedBodyPair& pair = cache.at(ordered[pairIndex]);

    pairFirst[pairIndex] = constraintTotal;
    pointFirst[pairIndex] = pointTotal;
    anyRestitution =
      anyRestitution || static_cast<float>(pair.restitution) > 0.0f;
    constraintTotal += static_cast<uint32_t>(pair.manifolds.size());

    for (const CachedManifold& manifold : pair.manifolds) {
      pointTotal += manifold.count;
    }
  }

  pairFirst[pairCount] = constraintTotal;
  pointFirst[pairCount] = pointTotal;

  for (uint32_t order = 0u; order != colourCount + 1u; ++order) {
    constraintStart[order] = pairFirst[colourStart[order]];
  }

  constraints.resize(constraintTotal);
  points.resize(pointTotal);
  calms.resize(pairCount);

  std::vector<uint32_t>& laneSlot = scratch.laneSlot;
  std::array<uint32_t, colourCount + 1u> batchStart{};
  std::array<uint32_t, colourCount + 1u> wideStart{};
  uint32_t batchCount = 0u;
  uint32_t wideTotal = 0u;
  uint32_t widePairs = 0u;

  for (uint32_t order = 1u; order != colourCount; ++order) {
    batchStart[order] = batchCount;
    wideStart[order] = wideTotal;

    if (batchWidth != 0u) {
      const uint32_t size = static_cast<uint32_t>(wideList(order - 1u).size());

      batchCount += (size + batchWidth - 1u) / batchWidth;
      wideTotal += size;
    }
  }

  batchStart[colourCount] = batchCount;
  wideStart[colourCount] = wideTotal;
  laneSlot.assign(static_cast<size_t>(batchCount) * batchWidth, noLane);

  std::vector<std::array<uint32_t, 2>> laneCounts(std::max(workers.count, 1u), std::array<uint32_t, 2>{0u, 0u});
  const auto fillLanes = [&](uint32_t begin, uint32_t end, uint32_t worker) {
    uint32_t order = static_cast<uint32_t>(
      std::upper_bound(wideStart.begin() + 1, wideStart.begin() + colourCount, begin) - wideStart.begin()
    ) - 1u;
    uint32_t filled = 0u;
    uint32_t restitution = 0u;

    for (uint32_t entry = begin; entry != end; ++entry) {
      while (entry >= wideStart[order + 1u]) {
        ++order;
      }

      const uint32_t index = entry - wideStart[order];
      const uint32_t slot = wideList(order - 1u)[index];

      if (!active(slot)) {
        continue;
      }

      laneSlot[static_cast<size_t>(batchStart[order]) * batchWidth + index] = slot;
      filled += 1u;
      restitution |= static_cast<float>(cache.at(slot).restitution) > 0.0f ? 1u : 0u;
    }

    laneCounts[worker][0] += filled;
    laneCounts[worker][1] |= restitution;
  };

  std::vector<uint32_t>& colourItems = scratch.colourItems;
  std::array<uint32_t, colourCount + 1u> itemStart{};

  colourItems.clear();

  for (uint32_t order = 1u; order != colourCount; ++order) {
    itemStart[order] = static_cast<uint32_t>(colourItems.size());

    for (uint32_t batch = batchStart[order]; batch != batchStart[order + 1u]; ++batch) {
      colourItems.push_back(batchItem | batch);
    }

    for (uint32_t pair = colourStart[order]; pair != colourStart[order + 1u]; ++pair) {
      colourItems.push_back(pair);
    }
  }

  itemStart[colourCount] = static_cast<uint32_t>(colourItems.size());

  std::vector<WideContact<4u>>& batches4 = scratch.batches4;
  std::vector<WideContact<8u>>& batches8 = scratch.batches8;

  batches4.resize(batchWidth == 4u ? batchCount : 0u);
  batches8.resize(batchWidth == 8u ? batchCount : 0u);
  WideBodies wideBodies;

  wideBodies.base = reinterpret_cast<float*>(states.data());
  wideBodies.stride = static_cast<uint32_t>(sizeof(State) / sizeof(float));
  wideBodies.v = floatOffset(states[0], &states[0].v);
  wideBodies.w = floatOffset(states[0], &states[0].w);
  wideBodies.dp = floatOffset(states[0], &states[0].dp);
  wideBodies.dq = floatOffset(states[0], &states[0].dq);

  const auto prepareConstraint = [&](Constraint& constraint, std::vector<Point>& pointList) {
    const CachedBodyPair& pair = cache.at(constraint.slot);
    const CachedManifold& manifold = pair.manifolds[constraint.manifold];
    const BodyRef& endA = manifold.flipped ? pair.second : pair.first;
    const BodyRef& endB = manifold.flipped ? pair.first : pair.second;

    constraint.a = stateIndex(stateOf, endB);
    constraint.b = stateIndex(stateOf, endA);

    const State& stateA = states[constraint.a];
    const State& stateB = states[constraint.b];
    const float mA = stateA.invMass;
    const glm::mat3 iA = stateA.invI;
    const float mB = stateB.invMass;
    const glm::mat3 iB = stateB.invI;
    const glm::vec3 normal(manifold.basis[0]);
    const glm::vec3 tangent1(manifold.basis[1]);
    const glm::vec3 tangent2(manifold.basis[2]);

    constraint.normal = normal;
    constraint.tangent1 = tangent1;
    constraint.tangent2 = tangent2;
    constraint.friction = static_cast<float>(pair.friction);
    constraint.restitution = static_cast<float>(pair.restitution);
    constraint.softness =
      mA == 0.0f || mB == 0.0f ? staticSoftness : contactSoftness;

    glm::vec3 centerA(0.0f);
    glm::vec3 centerB(0.0f);
    float totalFrictionWeight = 0.0f;
    glm::dvec3 frictionImpulse(0.0);
    for (uint32_t index = 0u; index != constraint.count; ++index) {
      Point& cp = pointList[constraint.start + index];
      const CachedContactPoint& source = pair.points[cp.point];
      const float s = static_cast<float>(source.separation);

      if (pair.pose.valid) {
        cp.rB = source.anchorB;
        cp.rA = source.anchorA;
      } else {
        const glm::dvec3 world =
          contactPoint(bodies, endA, endB, pair.manifoldPoints[cp.point], source.separation, manifold.basis[0]);

        cp.rA = glm::vec3(world - centreOf(bodies, endB, endA));
        cp.rB = glm::vec3(world - centreOf(bodies, endA, endB));
      }
      cp.baseSeparation = s - glm::dot(cp.rB - cp.rA, normal);
      cp.normalImpulse = static_cast<float>(source.memory.normal * step);

      const glm::vec3 rnA = glm::cross(cp.rA, normal);
      const glm::vec3 rnB = glm::cross(cp.rB, normal);
      const float kNormal = mA + mB + glm::dot(rnA, iA * rnA)
                            + glm::dot(rnB, iB * rnB);

      cp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

      const glm::vec3 vrA = stateA.v + glm::cross(stateA.w, cp.rA);
      const glm::vec3 vrB = stateB.v + glm::cross(stateB.w, cp.rB);

      cp.relativeVelocity = glm::dot(normal, vrB - vrA);

      const float weight = std::clamp(
        2.0f - s / speculativeDistance,
        minFrictionWeight,
        1.0f
      );

      centerA += weight * cp.rA;
      centerB += weight * cp.rB;
      totalFrictionWeight += weight;
      frictionImpulse += step
                         * (source.memory.friction.x * manifold.basis[1]
                            + source.memory.friction.y * manifold.basis[2]);
    }

    centerA /= totalFrictionWeight;
    centerB /= totalFrictionWeight;
    constraint.centerA = centerA;
    constraint.centerB = centerB;

    for (uint32_t index = 0u; index != constraint.count; ++index) {
      Point& cp = pointList[constraint.start + index];

      cp.leverArm = glm::distance(cp.rA, centerA);
    }

    const glm::vec3 rtA1 = glm::cross(centerA, tangent1);
    const glm::vec3 rtA2 = glm::cross(centerA, tangent2);
    const glm::vec3 rtB1 = glm::cross(centerB, tangent1);
    const glm::vec3 rtB2 = glm::cross(centerB, tangent2);
    glm::mat2 k(0.0f);

    k[0][0] = mA + mB + glm::dot(rtA1, iA * rtA1) + glm::dot(rtB1, iB * rtB1);
    k[1][1] = mA + mB + glm::dot(rtA2, iA * rtA2) + glm::dot(rtB2, iB * rtB2);
    k[0][1] = glm::dot(rtA1, iA * rtA2) + glm::dot(rtB1, iB * rtB2);
    k[1][0] = k[0][1];
    constraint.tangentMass = invert(k);
    constraint.frictionImpulse = glm::vec2(
      static_cast<float>(glm::dot(frictionImpulse, glm::dvec3(tangent1))),
      static_cast<float>(glm::dot(frictionImpulse, glm::dvec3(tangent2)))
    );

    const float twist = glm::dot(normal, (iA + iB) * normal);

    constraint.twistMass = twist > 0.0f ? 1.0f / twist : 0.0f;
    constraint.twistImpulse = static_cast<float>(manifold.twist * step);
  };
  const auto prepare = [&](uint32_t pairIndex) {
    const uint32_t slot = ordered[pairIndex];
    const CachedBodyPair& pair = cache.at(slot);
    uint32_t nextConstraint = pairFirst[pairIndex];
    uint32_t nextPoint = pointFirst[pairIndex];

    for (uint32_t index = 0u; index != pair.manifolds.size(); ++index) {
      const CachedManifold& manifold = pair.manifolds[index];
      Constraint& constraint = constraints[nextConstraint];

      constraint = Constraint{};
      constraint.slot = slot;
      constraint.manifold = index;
      constraint.start = nextPoint;
      constraint.count = manifold.count;

      for (uint32_t point = 0u; point != manifold.count; ++point) {
        Point cp;

        cp.point = manifold.start + point;
        points[nextPoint] = cp;
        nextPoint += 1u;
      }

      prepareConstraint(constraint, points);
      nextConstraint += 1u;
    }

    Calm& calm = calms[pairIndex];

    calm = Calm{};
    calm.idle = true;

    for (uint32_t index = pairFirst[pairIndex]; index != nextConstraint;
         ++index) {
      Constraint& constraint = constraints[index];

      calm.idle = calm.idle && idle(constraint, points);

      if (constraint.count == 0u) {
        continue;
      }

      for (uint32_t known = 0u; known != calm.count; ++known) {
        if (calm.facings[known].a == constraint.a
            && calm.facings[known].b == constraint.b) {
          constraint.facing = known;
        }
      }

      if (constraint.facing == noFacing && calm.count != calm.facings.size()) {
        const Point& first = points[constraint.start];
        Facing& facing = calm.facings[calm.count];

        facing.a = constraint.a;
        facing.b = constraint.b;
        facing.offset = first.rB - first.rA;
        constraint.facing = calm.count;
        calm.count += 1u;
      }

      if (constraint.facing == noFacing) {
        calm.mixed = true;
        continue;
      }

      Facing& facing = calm.facings[constraint.facing];

      for (uint32_t point = 0u; point != constraint.count; ++point) {
        Point& cp = points[constraint.start + point];
        const glm::vec3 apart = cp.rB - cp.rA;

        cp.anchorRadius = glm::length(cp.rA);
        cp.anchorSpread = glm::length(apart - facing.offset);
        facing.nearest = std::min(
          facing.nearest,
          cp.baseSeparation + glm::dot(apart, constraint.normal)
            - 2.0f * cp.anchorSpread
        );
        facing.anchorRadius = std::max(facing.anchorRadius, cp.anchorRadius);
        facing.anchorSpread = std::max(facing.anchorSpread, cp.anchorSpread);
        facing.base = std::max(facing.base, std::abs(cp.baseSeparation));
      }
    }
  };

  struct Tally {
    uint32_t points = 0u;
    uint32_t active = 0u;
    std::vector<uint32_t> hits;
  };

  const double hitThreshold = std::max(parameters.hitEventThreshold, 0.0);
  std::vector<Tally> tallies(std::max(workers.count, 1u));
  const auto count = [&](Tally& tally, uint32_t slot, const ContactMemory& memory, bool& hit) {
    tally.points += 1u;
    tally.active += memory.normal > 0.0 ? 1u : 0u;

    if (!hit && memory.approach > hitThreshold && memory.impulse > 0.0) {
      hit = true;
      tally.hits.push_back(slot);
    }
  };
  const auto storeConstraint = [&](const Constraint& constraint, const std::vector<Point>& pointList, Tally& tally) {
    CachedBodyPair& pair = cache.at(constraint.slot);
    bool hit = false;
    const CachedManifold& manifold = pair.manifolds[constraint.manifold];
    const glm::dvec3 friction(
      constraint.frictionImpulse.x * constraint.tangent1
      + constraint.frictionImpulse.y * constraint.tangent2
    );
    const glm::dvec3 share = friction / (step * constraint.count);

    pair.manifolds[constraint.manifold].twist = constraint.twistImpulse / step;

    for (uint32_t index = 0u; index != constraint.count; ++index) {
      const Point& cp = pointList[constraint.start + index];
      CachedContactPoint& target = pair.points[cp.point];

      target.memory.normal = cp.normalImpulse / step;
      target.memory.approach = -static_cast<double>(cp.relativeVelocity);
      target.memory.impulse = static_cast<double>(cp.totalNormalImpulse);
      target.memory.friction = glm::dvec2(
        glm::dot(share, manifold.basis[1]),
        glm::dot(share, manifold.basis[2])
      );
      count(tally, constraint.slot, target.memory, hit);
    }
  };
  const auto storeMemory = [&](uint32_t item, Tally& tally) {
    storeConstraint(constraints[item], points, tally);
  };

  const auto storeBody = [&](uint32_t item) {
    const State& state = states[item];
    RigidBody& body = bodies.rigids[rigidOf[item]];

    body.previousVelocity = body.velocity;
    body.initialPosition = body.position;
    body.initialRotation = body.rotation;
    body.position += glm::dvec3(state.dp);
    body.rotation = glm::normalize(glm::dquat(state.dq) * body.rotation);
    body.velocity = glm::dvec3(state.v);
    body.angularVelocity = glm::dvec3(state.w);
  };

  enum class Kind : uint8_t {
    Gather,
    Lanes,
    Prepare,
    Pack,
    Integrate,
    WarmStart,
    Push,
    Advance,
    Relax,
    Restitute,
    Unpack,
    StoreMemory,
    StoreBodies
  };

  struct Stage {
    Kind kind = Kind::Prepare;
    uint32_t begin = 0u;
    uint32_t end = 0u;
    uint32_t block = 1u;
  };

  WidePrepare widePrepare;

  widePrepare.contactBiasRate = contactSoftness.biasRate;
  widePrepare.contactMassScale = contactSoftness.massScale;
  widePrepare.contactImpulseScale = contactSoftness.impulseScale;
  widePrepare.staticBiasRate = staticSoftness.biasRate;
  widePrepare.staticMassScale = staticSoftness.massScale;
  widePrepare.staticImpulseScale = staticSoftness.impulseScale;
  widePrepare.speculativeDistance = speculativeDistance;
  widePrepare.minFrictionWeight = minFrictionWeight;
  widePrepare.singular = singular;

  const auto gatherLane = [&](auto& batch, uint32_t lane, uint32_t slot) {
    const CachedBodyPair& pair = cache.at(slot);
    const CachedManifold& manifold = pair.manifolds[0];
    const BodyRef& endA = manifold.flipped ? pair.second : pair.first;
    const BodyRef& endB = manifold.flipped ? pair.first : pair.second;
    const uint32_t a = stateIndex(stateOf, endB);
    const uint32_t b = stateIndex(stateOf, endA);
    const State& stateA = states[a];
    const State& stateB = states[b];
    const glm::vec3 tangent1(manifold.basis[1]);
    const glm::vec3 tangent2(manifold.basis[2]);
    const glm::vec3 normal(manifold.basis[0]);
    glm::dvec3 frictionImpulse(0.0);

    batch.a[lane] = a;
    batch.b[lane] = b;
    batch.invMassA[lane] = stateA.invMass;
    batch.invMassB[lane] = stateB.invMass;
    batch.friction[lane] = static_cast<float>(pair.friction);
    batch.restitution[lane] = static_cast<float>(pair.restitution);

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        batch.invIA[column][row][lane] = stateA.invI[column][row];
        batch.invIB[column][row][lane] = stateB.invI[column][row];
      }
    }

    for (int axis = 0; axis != 3; ++axis) {
      batch.normal[axis][lane] = normal[axis];
      batch.tangent1[axis][lane] = tangent1[axis];
      batch.tangent2[axis][lane] = tangent2[axis];
    }

    for (uint32_t index = 0u; index != manifold.count; ++index) {
      const CachedContactPoint& source = pair.points[manifold.start + index];
      auto& point = batch.points[index];
      glm::vec3 rA(0.0f);
      glm::vec3 rB(0.0f);

      if (pair.pose.valid) {
        rB = source.anchorB;
        rA = source.anchorA;
      } else {
        const glm::dvec3 world =
          contactPoint(bodies, endA, endB, pair.manifoldPoints[manifold.start + index], source.separation, manifold.basis[0]);

        rA = glm::vec3(world - centreOf(bodies, endB, endA));
        rB = glm::vec3(world - centreOf(bodies, endA, endB));
      }

      for (int axis = 0; axis != 3; ++axis) {
        point.anchorA[axis][lane] = rA[axis];
        point.anchorB[axis][lane] = rB[axis];
      }

      point.baseSeparation[lane] = static_cast<float>(source.separation);
      point.normalImpulse[lane] = static_cast<float>(source.memory.normal * step);
      point.live[lane] = 0xFFFFFFFFu;
      frictionImpulse += step
                         * (source.memory.friction.x * manifold.basis[1]
                            + source.memory.friction.y * manifold.basis[2]);
    }

    batch.frictionImpulse[0][lane] =
      static_cast<float>(glm::dot(frictionImpulse, glm::dvec3(tangent1)));
    batch.frictionImpulse[1][lane] =
      static_cast<float>(glm::dot(frictionImpulse, glm::dvec3(tangent2)));
    batch.twistImpulse[lane] = static_cast<float>(manifold.twist * step);

    for (uint32_t index = 0u; index != 4u; ++index) {
      auto& point = batch.points[index];

      point.totalNormalImpulse[lane] = 0.0f;
      point.restitutionImpulse[lane] = 0.0f;

      if (index >= manifold.count) {
        for (int axis = 0; axis != 3; ++axis) {
          point.anchorA[axis][lane] = 0.0f;
          point.anchorB[axis][lane] = 0.0f;
        }

        point.baseSeparation[lane] = 0.0f;
        point.normalImpulse[lane] = 0.0f;
        point.live[lane] = 0u;
      }
    }
  };
  const auto emptyLane = [&](auto& batch, uint32_t lane) {
    batch.a[lane] = 0u;
    batch.b[lane] = 0u;
    batch.invMassA[lane] = 0.0f;
    batch.invMassB[lane] = 0.0f;
    batch.friction[lane] = 0.0f;
    batch.restitution[lane] = 0.0f;
    batch.frictionImpulse[0][lane] = 0.0f;
    batch.frictionImpulse[1][lane] = 0.0f;
    batch.twistImpulse[lane] = 0.0f;

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        batch.invIA[column][row][lane] = 0.0f;
        batch.invIB[column][row][lane] = 0.0f;
      }

      batch.normal[column][lane] = 0.0f;
      batch.tangent1[column][lane] = 0.0f;
      batch.tangent2[column][lane] = 0.0f;
    }

    for (auto& point : batch.points) {
      for (int axis = 0; axis != 3; ++axis) {
        point.anchorA[axis][lane] = 0.0f;
        point.anchorB[axis][lane] = 0.0f;
      }

      point.baseSeparation[lane] = 0.0f;
      point.normalImpulse[lane] = 0.0f;
      point.totalNormalImpulse[lane] = 0.0f;
      point.restitutionImpulse[lane] = 0.0f;
      point.live[lane] = 0u;
    }
  };
  const auto storeLane = [&](const auto& batch, uint32_t lane, uint32_t slot, Tally& tally) {
    CachedBodyPair& pair = cache.at(slot);
    bool hit = false;
    const CachedManifold& manifold = pair.manifolds[0];
    const glm::vec3 tangent1(batch.tangent1[0][lane], batch.tangent1[1][lane], batch.tangent1[2][lane]);
    const glm::vec3 tangent2(batch.tangent2[0][lane], batch.tangent2[1][lane], batch.tangent2[2][lane]);
    const glm::vec2 impulse(batch.frictionImpulse[0][lane], batch.frictionImpulse[1][lane]);
    const glm::dvec3 friction(impulse.x * tangent1 + impulse.y * tangent2);
    const glm::dvec3 share = friction / (step * manifold.count);

    pair.manifolds[0].twist = batch.twistImpulse[lane] / step;

    for (uint32_t index = 0u; index != manifold.count; ++index) {
      const auto& point = batch.points[index];
      CachedContactPoint& target = pair.points[manifold.start + index];

      target.memory.normal = point.normalImpulse[lane] / step;
      target.memory.approach = -static_cast<double>(point.relativeVelocity[lane]);
      target.memory.impulse = static_cast<double>(point.totalNormalImpulse[lane]);
      target.memory.friction = glm::dvec2(
        glm::dot(share, manifold.basis[1]),
        glm::dot(share, manifold.basis[2])
      );
      count(tally, slot, target.memory, hit);
    }
  };
  const auto prepareBatch = [&](uint32_t batch) {
    for (uint32_t lane = 0u; lane != batchWidth; ++lane) {
      const uint32_t slot = laneSlot[batch * batchWidth + lane];

      if (slot == noLane) {
        if (batchWidth == 8u) {
          emptyLane(batches8[batch], lane);
        } else {
          emptyLane(batches4[batch], lane);
        }
      } else if (batchWidth == 8u) {
        gatherLane(batches8[batch], lane, slot);
      } else {
        gatherLane(batches4[batch], lane, slot);
      }
    }

#if defined(TALUS_WIDE_CONTACTS)
    if (batchWidth == 8u) {
      prepareW8(batches8[batch], wideBodies, widePrepare);
    } else {
      prepareW4(batches4[batch], wideBodies, widePrepare);
    }
#endif
  };
  const auto storeBatch = [&](uint32_t batch, Tally& tally) {
    for (uint32_t lane = 0u; lane != batchWidth; ++lane) {
      const uint32_t slot = laneSlot[batch * batchWidth + lane];

      if (slot == noLane) {
        continue;
      }

      if (batchWidth == 8u) {
        storeLane(batches8[batch], lane, slot, tally);
      } else {
        storeLane(batches4[batch], lane, slot, tally);
      }
    }
  };
  const auto solveBatch = [&](Kind kind, uint32_t batch) {
#if defined(TALUS_WIDE_CONTACTS)
    if (batchWidth == 8u) {
      WideContact<8u>& contact = batches8[batch];

      switch (kind) {
        case Kind::WarmStart:
          warmStartW8(contact, wideBodies);
          break;
        case Kind::Push:
          pushW8(contact, wideBodies, invH, contactSpeed);
          break;
        case Kind::Relax:
          relaxW8(contact, wideBodies, invH);
          break;
        case Kind::Restitute:
          restituteW8(contact, wideBodies, invH, threshold);
          break;
        default:
          break;
      }
    } else {
      WideContact<4u>& contact = batches4[batch];

      switch (kind) {
        case Kind::WarmStart:
          warmStartW4(contact, wideBodies);
          break;
        case Kind::Push:
          pushW4(contact, wideBodies, invH, contactSpeed);
          break;
        case Kind::Relax:
          relaxW4(contact, wideBodies, invH);
          break;
        case Kind::Restitute:
          restituteW4(contact, wideBodies, invH, threshold);
          break;
        default:
          break;
      }
    }
#else
    (void)kind;
    (void)batch;
    (void)wideBodies;
#endif
  };
  const auto solveItem = [&](Kind kind, uint32_t item) {
    if ((item & batchItem) != 0u) {
      solveBatch(kind, item & ~batchItem);
      return;
    }

    Calm& calm = calms[item];
    const bool solving = kind == Kind::Push || kind == Kind::Relax;
    std::array<Drift, 2> drifts{};

    if ((kind == Kind::WarmStart && calm.idle)
        || (solving && calmPair(states, calm, drifts, invH))) {
      return;
    }

    bool quiet = true;

    for (uint32_t entry = pairFirst[item]; entry != pairFirst[item + 1u]; ++entry) {
      Constraint& constraint = constraints[entry];
      const bool still = idle(constraint, points);

      if (solving && still && constraint.facing != noFacing
          && dormant(
            constraint,
            points,
            driftOf(
              states,
              calm.facings[constraint.facing],
              drifts[constraint.facing]
            ),
            invH
          )) {
        continue;
      }

      switch (kind) {
        case Kind::WarmStart:
          if (!still) {
            warmStart(states, constraint, points);
          }
          break;
        case Kind::Push:
          push(states, constraint, points, invH, contactSpeed);
          break;
        case Kind::Relax:
          relax(states, constraint, points, invH);
          break;
        case Kind::Restitute:
          restitute(states, constraint, points, invH, threshold);
          break;
        default:
          break;
      }

      if (kind != Kind::WarmStart) {
        drifts[0].paced = false;
        drifts[1].paced = false;
        quiet = quiet && idle(constraint, points);
      }
    }

    if (kind != Kind::WarmStart) {
      calm.idle = quiet;
    }
  };

  std::vector<Stage> stages;
  std::vector<uint32_t> blockCounts;
  const auto add = [&](Kind kind, uint32_t begin, uint32_t end, uint32_t minimum) {
    const uint32_t count = end - begin;
    const uint32_t block = std::max(minimum, count / (4u * workers.count));

    stages.push_back({kind, begin, end, block});
    blockCounts.push_back(count == 0u ? 0u : (count + block - 1u) / block);

    return static_cast<uint32_t>(stages.size() - 1u);
  };
  const auto colours = [&](Kind kind) {
    const uint32_t first = static_cast<uint32_t>(stages.size());

    for (uint32_t order = 1u; order != colourCount; ++order) {
      (void)add(kind, itemStart[order], itemStart[order + 1u], 4u);
    }

    return first;
  };
  const uint32_t stateCount = static_cast<uint32_t>(states.size());
  const uint32_t gatherStage = add(Kind::Gather, 1u, stateCount, 32u);
  const uint32_t laneStage = add(Kind::Lanes, 0u, wideTotal, 64u);
  const uint32_t prepareStage = add(Kind::Prepare, 0u, pairCount, 4u);
  const uint32_t packStage = add(Kind::Pack, 0u, batchCount, 4u);
  std::vector<std::array<uint32_t, 5>> substepStages(substeps);

  for (uint32_t substep = 0u; substep != substeps; ++substep) {
    substepStages[substep][0] = add(Kind::Integrate, 1u, stateCount, 32u);
    substepStages[substep][1] = colours(Kind::WarmStart);
    substepStages[substep][2] = colours(Kind::Push);
    substepStages[substep][3] = add(Kind::Advance, 1u, stateCount, 32u);
    substepStages[substep][4] = colours(Kind::Relax);
  }

  std::vector<uint32_t> restitutionStages(parameters.restitutionIterations);

  for (uint32_t& first : restitutionStages) {
    first = colours(Kind::Restitute);
  }

  const uint32_t unpackStage = add(Kind::Unpack, 0u, batchCount, 4u);
  const uint32_t memoryStage =
    add(Kind::StoreMemory, 0u, static_cast<uint32_t>(constraints.size()), 4u);
  const uint32_t bodyStage = add(Kind::StoreBodies, 1u, ownedCount + 1u, 32u);

  StageRunner& runner = scratch.runner;

  runner.reset(workers, blockCounts, [&](uint32_t index, uint32_t block, uint32_t worker) {
    const Stage& stage = stages[index];
    const uint32_t begin = stage.begin + block * stage.block;
    const uint32_t end = std::min(begin + stage.block, stage.end);
    Tally tally;

    if (stage.kind == Kind::Lanes) {
      fillLanes(begin, end, worker);
      return;
    }

    for (uint32_t item = begin; item != end; ++item) {
      switch (stage.kind) {
        case Kind::Gather:
          gather(item);
          break;
        case Kind::Lanes:
          break;
        case Kind::Prepare:
          prepare(item);
          break;
        case Kind::Pack:
          prepareBatch(item);
          break;
        case Kind::Integrate:
          integrateVelocity(states[item], gravity, h);
          break;
        case Kind::WarmStart:
        case Kind::Push:
        case Kind::Relax:
        case Kind::Restitute:
          solveItem(stage.kind, colourItems[item]);
          break;
        case Kind::Advance:
          integratePosition(states[item], h, maxLinearSpeed, maxAngularSpeed);
          break;
        case Kind::Unpack:
          storeBatch(item, tally);
          break;
        case Kind::StoreMemory:
          storeMemory(item, tally);
          break;
        case Kind::StoreBodies:
          storeBody(item);
          break;
      }
    }

    if (tally.points != 0u) {
      Tally& total = tallies[worker];

      total.points += tally.points;
      total.active += tally.active;
      total.hits.insert(total.hits.end(), tally.hits.begin(), tally.hits.end());
    }
  }, Claiming::Ranges);
  const uint32_t overflowEnd = constraintStart[1];
  const auto runColours = [&](uint32_t first) {
    for (uint32_t order = 1u; order != colourCount; ++order) {
      runner.execute(first + order - 1u);
    }
  };

  Stopwatch clock;
  double gatherTime = 0.0;
  double prepareTime = 0.0;
  double substepTime = 0.0;
  double integrateTime = 0.0;
  double solveTime = 0.0;
  double restituteTime = 0.0;
  double storeTime = 0.0;
  const bool jointWork = joints != nullptr && !joints->empty();
  std::vector<JointRow<float>>& jointRows = scratch.jointRows;
  Stopwatch jointClock;
  double jointTime = 0.0;
  uint32_t limitPassRows = 0u;

  runner.run([&] {
    (void)clock.lap();
    runner.execute(gatherStage);
    runner.execute(laneStage);

    for (const std::array<uint32_t, 2>& lanes : laneCounts) {
      anyRestitution = anyRestitution || lanes[1] != 0u;
    }

    gatherTime = clock.lap();
    runner.execute(prepareStage);
    runner.execute(packStage);

    if (jointWork) {
      (void)jointClock.lap();
      prepareJointRows<float, State>(
        *joints,
        bodies,
        *islandStore,
        states,
        stateOf,
        h,
        jointRows
      );
      jointTime += jointClock.lap();
    }

    prepareTime = clock.lap();

    for (uint32_t substep = 0u; substep != substeps; ++substep) {
      runner.execute(substepStages[substep][0]);
      integrateTime += clock.lap();

      if (jointWork) {
        (void)jointClock.lap();
        warmStartJointRows<float, State>(states, jointRows);
        jointTime += jointClock.lap();
      }

      for (uint32_t item = 0u; item != overflowEnd; ++item) {
        warmStart(states, constraints[item], points);
      }

      runColours(substepStages[substep][1]);

      if (jointWork) {
        (void)jointClock.lap();
        limitPassRows +=
          solveJointRows<float, State>(states, jointRows, invH, true);
        jointTime += jointClock.lap();
      }

      for (uint32_t item = 0u; item != overflowEnd; ++item) {
        push(states, constraints[item], points, invH, contactSpeed);
      }

      runColours(substepStages[substep][2]);
      solveTime += clock.lap();
      runner.execute(substepStages[substep][3]);
      integrateTime += clock.lap();

      if (jointWork) {
        (void)jointClock.lap();
        advanceJointRows<float>(jointRows, h);
        limitPassRows +=
          solveJointRows<float, State>(states, jointRows, invH, false);
        jointTime += jointClock.lap();
      }

      for (uint32_t item = 0u; item != overflowEnd; ++item) {
        relax(states, constraints[item], points, invH);
      }

      runColours(substepStages[substep][4]);
      solveTime += clock.lap();
    }

    if (anyRestitution) {
      for (const uint32_t first : restitutionStages) {
        for (uint32_t item = 0u; item != overflowEnd; ++item) {
          restitute(states, constraints[item], points, invH, threshold);
        }

        runColours(first);
      }
    }

    restituteTime = clock.lap();
    substepTime = integrateTime + solveTime + restituteTime;
    runner.execute(unpackStage);
    runner.execute(memoryStage);

    if (jointWork) {
      (void)jointClock.lap();
      storeJointRows<float>(*joints, jointRows, invH);
      jointTime += jointClock.lap();
    }

    runner.execute(bodyStage);
    storeTime = clock.lap();
  }, alongside);

  for (const uint32_t index : rigids) {
    stateOf[index] = 0u;
  }

  for (const uint32_t index : guests) {
    stateOf[index] = 0u;
  }

  for (const std::array<uint32_t, 2>& lanes : laneCounts) {
    widePairs += lanes[0];
  }

  if (touching != nullptr) {
    touching->points = 0u;
    touching->active = 0u;
    touching->hits.clear();

    for (const Tally& tally : tallies) {
      touching->points += tally.points;
      touching->active += tally.active;
      touching->hits.insert(touching->hits.end(), tally.hits.begin(), tally.hits.end());
    }

    std::sort(touching->hits.begin(), touching->hits.end());
    touching->hits.erase(std::unique(touching->hits.begin(), touching->hits.end()), touching->hits.end());
  }

  if (profile != nullptr) {
    profile->gather = gatherTime;
    profile->prepare = prepareTime;
    profile->substeps = substepTime;
    profile->integrate = integrateTime;
    profile->restitute = restituteTime;
    profile->storeBack = storeTime + clock.lap();
    profile->joints = jointTime;
  }

  return {
    widePairs,
    substeps,
    jointWork ? static_cast<uint32_t>(jointRows.size()) : 0u,
    limitPassRows
  };
}
}
