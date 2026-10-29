#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/joints/joints.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

namespace talus::dynamics {

template <typename Real>
using JointVec = glm::vec<3, Real>;

template <typename Real>
using JointQuat = glm::qua<Real>;

template <typename Real>
using JointMat = glm::mat<3, 3, Real>;

template <typename Real>
struct JointSoftness {
  Real biasRate = Real(0);
  Real massScale = Real(0);
  Real impulseScale = Real(0);
};

template <typename Real>
struct JointEnd {
  uint32_t state = 0u;
  Real invMass = Real(0);
  JointMat<Real> invI{Real(0)};
  JointVec<Real> anchor{Real(0)};
  JointVec<Real> v{Real(0)};
  JointVec<Real> w{Real(0)};
  JointVec<Real> dp{Real(0)};
  JointQuat<Real> dq{Real(1), Real(0), Real(0), Real(0)};
};

template <typename Real>
struct JointRow {
  uint32_t joint = 0u;
  JointEnd<Real> a;
  JointEnd<Real> b;
  JointVec<Real> deltaCenter{Real(0)};
  JointSoftness<Real> softness;
  JointVec<Real> pointImpulse{Real(0)};
};

template <typename Real>
struct JointMotion {
  JointVec<Real> v{Real(0)};
  JointVec<Real> w{Real(0)};
  JointVec<Real> dp{Real(0)};
  JointQuat<Real> dq{Real(1), Real(0), Real(0), Real(0)};
};

template <typename Real>
JointSoftness<Real> jointSoftness(Real hertz, Real zeta, Real h) {
  if (hertz == Real(0)) {
    return {};
  }

  const Real omega = Real(2) * std::numbers::pi_v<Real> * hertz;
  const Real a1 = Real(2) * zeta + h * omega;
  const Real a2 = h * omega * a1;
  const Real a3 = Real(1) / (Real(1) + a2);

  return {omega / a1, a2 * a3, a3};
}

template <typename Real>
JointMat<Real> jointSkew(const JointVec<Real>& v) {
  return JointMat<Real>(
    JointVec<Real>(Real(0), v.z, -v.y),
    JointVec<Real>(-v.z, Real(0), v.x),
    JointVec<Real>(v.y, -v.x, Real(0))
  );
}

template <typename Real>
JointVec<Real> jointSolve3(const JointMat<Real>& m, const JointVec<Real>& a) {
  const Real det = glm::dot(m[0], glm::cross(m[1], m[2]));

  if (std::abs(det) <= Real(1000) * std::numeric_limits<Real>::min()) {
    return JointVec<Real>(Real(0));
  }

  const Real invDet = Real(1) / det;

  return JointVec<Real>(
    invDet * glm::dot(glm::cross(m[1], m[2]), a),
    invDet * glm::dot(glm::cross(m[2], m[0]), a),
    invDet * glm::dot(glm::cross(m[0], m[1]), a)
  );
}

template <typename Real>
JointQuat<Real> jointTurn(const JointQuat<Real>& q, const JointVec<Real>& delta) {
  const JointQuat<Real> qd =
    JointQuat<Real>(Real(0), Real(0.5) * delta.x, Real(0.5) * delta.y, Real(0.5) * delta.z) * q;

  return glm::normalize(JointQuat<Real>(q.w + qd.w, q.x + qd.x, q.y + qd.y, q.z + qd.z));
}

template <typename Real, typename State>
bool resolveJointEnd(
  const BodySet& bodies,
  const IslandStore& islands,
  std::span<const State> states,
  std::span<const uint32_t> stateOf,
  uint32_t id,
  const JointFrame& frame,
  JointEnd<Real>& end,
  glm::dvec3& centre
) {
  end = JointEnd<Real>{};

  if (id == noBody) {
    centre = frame.position;
    return true;
  }

  const BodyRef ref = islands.refOf(id);

  if (ref.side != Side::Rigid || ref.index >= bodies.rigids.size() || bodies.rigids[ref.index].id != id) {
    return false;
  }

  const RigidBody& body = bodies.rigids[ref.index];

  end.state = stateOf[ref.index];

  if (end.state == 0u && body.mass > 0.0) {
    return false;
  }

  if (end.state == 0u) {
    end.v = JointVec<Real>(body.velocity);
    end.w = JointVec<Real>(body.angularVelocity);
  } else {
    end.invMass = states[end.state].invMass;
    end.invI = states[end.state].invI;
  }

  end.anchor = JointVec<Real>(body.rotation * frame.position);
  centre = body.position;

  return true;
}

template <typename Real, typename State>
JointMotion<Real> jointMotion(std::span<State> states, const JointEnd<Real>& end) {
  if (end.state == 0u) {
    return {end.v, end.w, end.dp, end.dq};
  }

  const State& state = states[end.state];

  return {state.v, state.w, state.dp, state.dq};
}

template <typename Real, typename State>
void storeJointMotion(std::span<State> states, const JointEnd<Real>& end, const JointMotion<Real>& motion) {
  if (end.invMass != Real(0)) {
    states[end.state].v = motion.v;
    states[end.state].w = motion.w;
  }
}

template <typename Real, typename State>
void prepareJointRows(
  const JointSet& joints,
  const BodySet& bodies,
  const IslandStore& islands,
  std::span<const State> states,
  std::span<const uint32_t> stateOf,
  Real h,
  std::vector<JointRow<Real>>& rows
) {
  const std::span<const Joint> all = joints.all();
  const Real hertzCap = Real(0.25) / h;

  rows.clear();

  for (uint32_t index = 0u; index != all.size(); ++index) {
    const Joint& joint = all[index];
    JointRow<Real> row;
    glm::dvec3 centreA(0.0);
    glm::dvec3 centreB(0.0);

    if (!resolveJointEnd(bodies, islands, states, stateOf, joint.def.bodyA, joint.def.frameA, row.a, centreA)
        || !resolveJointEnd(bodies, islands, states, stateOf, joint.def.bodyB, joint.def.frameB, row.b, centreB)
        || (row.a.state == 0u && row.b.state == 0u)) {
      continue;
    }

    row.joint = index;
    row.deltaCenter = JointVec<Real>(centreB - centreA);
    row.softness = jointSoftness(
      std::min(static_cast<Real>(joint.def.hertz), hertzCap),
      static_cast<Real>(joint.def.dampingRatio),
      h
    );
    row.pointImpulse = JointVec<Real>(joint.impulses.point * static_cast<double>(h));
    rows.push_back(row);
  }
}

template <typename Real, typename State>
void warmStartJointRows(std::span<State> states, std::span<const JointRow<Real>> rows) {
  for (const JointRow<Real>& row : rows) {
    JointMotion<Real> a = jointMotion(states, row.a);
    JointMotion<Real> b = jointMotion(states, row.b);
    const JointVec<Real> rA = a.dq * row.a.anchor;
    const JointVec<Real> rB = b.dq * row.b.anchor;
    const JointVec<Real>& impulse = row.pointImpulse;

    a.v -= row.a.invMass * impulse;
    a.w -= row.a.invI * glm::cross(rA, impulse);
    b.v += row.b.invMass * impulse;
    b.w += row.b.invI * glm::cross(rB, impulse);
    storeJointMotion(states, row.a, a);
    storeJointMotion(states, row.b, b);
  }
}

template <typename Real, typename State>
void solveJointRows(
  std::span<State> states,
  std::span<JointRow<Real>> rows,
  [[maybe_unused]] Real invH,
  bool useBias
) {
  for (JointRow<Real>& row : rows) {
    JointMotion<Real> a = jointMotion(states, row.a);
    JointMotion<Real> b = jointMotion(states, row.b);
    const Real mA = row.a.invMass;
    const Real mB = row.b.invMass;
    const JointMat<Real>& iA = row.a.invI;
    const JointMat<Real>& iB = row.b.invI;
    const JointVec<Real> rA = a.dq * row.a.anchor;
    const JointVec<Real> rB = b.dq * row.b.anchor;
    const JointVec<Real> cdot = b.v + glm::cross(b.w, rB) - a.v - glm::cross(a.w, rA);
    JointVec<Real> bias(Real(0));
    Real massScale = Real(1);
    Real impulseScale = Real(0);

    if (useBias) {
      const JointVec<Real> separation = (b.dp - a.dp) + (rB - rA) + row.deltaCenter;

      bias = row.softness.biasRate * separation;
      massScale = row.softness.massScale;
      impulseScale = row.softness.impulseScale;
    }

    const JointMat<Real> sA = jointSkew(rA);
    const JointMat<Real> sB = jointSkew(rB);
    JointMat<Real> k = -(sA * iA * sA + sB * iB * sB);

    k[0][0] += mA + mB;
    k[1][1] += mA + mB;
    k[2][2] += mA + mB;

    const JointVec<Real> impulse =
      -massScale * jointSolve3(k, cdot + bias) - impulseScale * row.pointImpulse;

    row.pointImpulse += impulse;
    a.v -= mA * impulse;
    a.w -= iA * glm::cross(rA, impulse);
    b.v += mB * impulse;
    b.w += iB * glm::cross(rB, impulse);
    storeJointMotion(states, row.a, a);
    storeJointMotion(states, row.b, b);
  }
}

template <typename Real>
void advanceJointEnd(JointEnd<Real>& end, Real h) {
  if (end.state != 0u) {
    return;
  }

  end.dp += h * end.v;
  end.dq = jointTurn(end.dq, h * end.w);
}

template <typename Real>
void advanceJointRows(std::span<JointRow<Real>> rows, Real h) {
  for (JointRow<Real>& row : rows) {
    advanceJointEnd(row.a, h);
    advanceJointEnd(row.b, h);
  }
}

template <typename Real>
void storeJointRows(JointSet& joints, std::span<const JointRow<Real>> rows, Real invH) {
  const std::span<Joint> all = joints.all();

  for (const JointRow<Real>& row : rows) {
    Joint& joint = all[row.joint];
    const glm::dvec3 force = glm::dvec3(row.pointImpulse) * static_cast<double>(invH);

    joint.impulses.point = force;
    joint.report.force = force;
    joint.report.torque = glm::dvec3(0.0);
  }
}

}
