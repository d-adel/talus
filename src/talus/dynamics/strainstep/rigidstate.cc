#include "rigidstate.h"

#include <cmath>
#include <limits>
#include <numbers>

namespace talus::dynamics {

namespace {

constexpr double SINGULAR = 1000.0 * std::numeric_limits<double>::min();

glm::dvec3 solve3(const glm::dmat3& m, const glm::dvec3& a) {
  const double det = glm::dot(m[0], glm::cross(m[1], m[2]));

  if (std::abs(det) <= SINGULAR) {
    return glm::dvec3(0.0);
  }

  const double invDet = 1.0 / det;

  return glm::dvec3(
    invDet * glm::dot(glm::cross(m[1], m[2]), a),
    invDet * glm::dot(glm::cross(m[2], m[0]), a),
    invDet * glm::dot(glm::cross(m[0], m[1]), a)
  );
}

glm::dquat integrateRotation(const glm::dquat& q1, const glm::dvec3& delta) {
  glm::dquat qd(0.0, 0.5 * delta.x, 0.5 * delta.y, 0.5 * delta.z);

  qd = qd * q1;

  return glm::normalize(glm::dquat(q1.w + qd.w, q1.x + qd.x, q1.y + qd.y, q1.z + qd.z));
}

}

Softness makeSoft(double hertz, double zeta, double h) {
  if (hertz == 0.0) {
    return {};
  }

  const double omega = 2.0 * std::numbers::pi * hertz;
  const double a1 = 2.0 * zeta + h * omega;
  const double a2 = h * omega * a1;
  const double a3 = 1.0 / (1.0 + a2);

  return {omega / a1, a2 * a3, a3};
}

RigidState gatherRigid(const RigidBody& body, uint32_t index) {
  RigidState state;

  state.body = index;
  state.position = body.position;
  state.v = body.velocity;
  state.w = body.angularVelocity;
  state.rotation = body.rotation;

  if (body.mass > 0.0) {
    const glm::dmat3 turn = glm::mat3_cast(body.rotation);
    glm::dmat3 inverse(0.0);

    for (int axis = 0; axis != 3; ++axis) {
      inverse[axis][axis] = body.moment[axis] > 0.0 ? 1.0 / body.moment[axis] : 0.0;
    }

    state.invMass = 1.0 / body.mass;
    state.invI = turn * inverse * glm::transpose(turn);
    state.moment = body.moment;
  }

  return state;
}

void integrateRigidVelocity(RigidState& state, const glm::dvec3& gravity, double h) {
  if (state.invMass == 0.0) {
    return;
  }

  state.v += h * gravity;

  const glm::dquat q = state.dq * state.rotation;
  const double i00 = state.moment.x;
  const double i11 = state.moment.y;
  const double i22 = state.moment.z;
  const double i01 = 0.0;
  const double i02 = 0.0;
  const double i12 = 0.0;
  const glm::dvec3 omega1 = glm::conjugate(q) * state.w;
  glm::dvec3 omega2 = omega1;
  const double w1 = omega2.x;
  const double w2 = omega2.y;
  const double w3 = omega2.z;
  const double Iw1 = i00 * w1 + i01 * w2 + i02 * w3;
  const double Iw2 = i01 * w1 + i11 * w2 + i12 * w3;
  const double Iw3 = i02 * w1 + i12 * w2 + i22 * w3;
  const glm::dvec3 dw = omega2 - omega1;
  const glm::dvec3 b(
    i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * Iw3 - w3 * Iw2),
    i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * Iw1 - w1 * Iw3),
    i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * Iw2 - w2 * Iw1)
  );
  const glm::dmat3 J(
    glm::dvec3(
      i00 + h * (w2 * i02 - w3 * i01),
      i01 + h * (w3 * i00 - w1 * i02 - Iw3),
      i02 + h * (w1 * i01 - w2 * i00 + Iw2)
    ),
    glm::dvec3(
      i01 + h * (w2 * i12 - w3 * i11 + Iw3),
      i11 + h * (w3 * i01 - w1 * i12),
      i12 + h * (w1 * i11 - w2 * i01 - Iw1)
    ),
    glm::dvec3(
      i02 + h * (w2 * i22 - w3 * i12 - Iw2),
      i12 + h * (w3 * i02 - w1 * i22 + Iw1),
      i22 + h * (w1 * i12 - w2 * i02)
    )
  );

  omega2 -= solve3(J, b);
  state.w = q * omega2;
}

void integrateRigidPosition(
  RigidState& state,
  double h,
  double maxLinearSpeed,
  double maxAngularSpeed
) {
  glm::dvec3 v = state.v;
  glm::dvec3 w = state.w;

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

void storeRigid(const RigidState& state, RigidBody& body) {
  body.previousVelocity = body.velocity;
  body.initialPosition = body.position;
  body.initialRotation = body.rotation;
  body.position += state.dp;
  body.rotation = glm::normalize(state.dq * body.rotation);
  body.velocity = state.v;
  body.angularVelocity = state.w;
}

}
