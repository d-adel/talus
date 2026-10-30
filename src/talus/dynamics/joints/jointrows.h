#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/joints/joints.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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
using JointPair = glm::vec<2, Real>;

inline constexpr double jointLimitSpeed = 3.0;

inline constexpr double jointSmall = 1.0e-6;

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
  JointQuat<Real> frame{Real(1), Real(0), Real(0), Real(0)};
  JointVec<Real> v{Real(0)};
  JointVec<Real> w{Real(0)};
  JointVec<Real> dp{Real(0)};
  JointQuat<Real> dq{Real(1), Real(0), Real(0), Real(0)};
};

template <typename Real>
struct JointDriveRow {
  JointSoftness<Real> softness;
  JointVec<Real> axis{Real(0)};
  Real mass = Real(0);
  Real target = Real(0);
  Real speed = Real(0);
  Real lower = Real(0);
  Real upper = Real(0);
  Real cap = Real(0);
  Real impulse = Real(0);
  bool active = false;
  bool saturated = false;
};

template <typename Real>
struct JointRow {
  uint32_t joint = 0u;
  JointKind kind = JointKind::Ball;
  JointEnd<Real> a;
  JointEnd<Real> b;
  JointVec<Real> deltaCenter{Real(0)};
  JointSoftness<Real> softness;
  JointVec<Real> pointImpulse{Real(0)};
  JointVec<Real> axis{Real(0)};
  Real axialMass = Real(0);
  JointVec<Real> perpY{Real(0)};
  JointVec<Real> perpZ{Real(0)};
  JointPair<Real> alignmentImpulse{Real(0)};
  bool limit = false;
  Real lower = Real(0);
  Real upper = Real(0);
  Real lowerImpulse = Real(0);
  Real upperImpulse = Real(0);
  JointPair<Real> coneCentre{Real(0)};
  JointPair<Real> coneHalf{Real(0)};
  JointVec<Real> coneAxis{Real(0)};
  Real coneImpulse = Real(0);
  JointQuat<Real> target{Real(1), Real(0), Real(0), Real(0)};
  std::array<JointDriveRow<Real>, 3> drives;
  JointVec<Real> feedForward{Real(0)};
};

template <typename Real>
struct JointSwing {
  Real twist = Real(0);
  JointPair<Real> swing{Real(0)};
  JointPair<Real> sine{Real(0)};
  Real cosine = Real(1);
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
JointSoftness<Real> jointDriveSoftness(Real stiffness, Real damping, Real mass, Real h) {
  const Real resistance = damping + h * stiffness;

  if (resistance == Real(0) || mass == Real(0)) {
    return {};
  }

  const Real a2 = h * resistance / mass;
  const Real a3 = Real(1) / (Real(1) + a2);

  return {stiffness / resistance, a2 * a3, a3};
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
JointPair<Real> jointSolve2(Real kxx, Real kxy, Real kyy, const JointPair<Real>& a) {
  const Real det = kxx * kyy - kxy * kxy;

  if (det <= Real(1000) * std::numeric_limits<Real>::min()) {
    return JointPair<Real>(Real(0));
  }

  const Real invDet = Real(1) / det;

  return JointPair<Real>(invDet * (kyy * a.x - kxy * a.y), invDet * (kxx * a.y - kxy * a.x));
}

template <typename Real>
JointQuat<Real> jointTurn(const JointQuat<Real>& q, const JointVec<Real>& delta) {
  const JointQuat<Real> qd =
    JointQuat<Real>(Real(0), Real(0.5) * delta.x, Real(0.5) * delta.y, Real(0.5) * delta.z) * q;

  return glm::normalize(JointQuat<Real>(q.w + qd.w, q.x + qd.x, q.y + qd.y, q.z + qd.z));
}

template <typename Real>
JointQuat<Real> jointRelative(const JointQuat<Real>& frameA, JointQuat<Real> frameB) {
  if (glm::dot(frameA, frameB) < Real(0)) {
    frameB = -frameB;
  }

  return glm::conjugate(frameA) * frameB;
}

template <typename Real>
Real jointTwist(const JointQuat<Real>& relative) {
  return relative.w < Real(0) ? Real(2) * std::atan2(-relative.x, -relative.w)
                              : Real(2) * std::atan2(relative.x, relative.w);
}

template <typename Real>
JointVec<Real> jointPerpAxis(const JointQuat<Real>& frameA, const JointQuat<Real>& relative, const JointVec<Real>& axis) {
  const JointVec<Real> v(relative.x, relative.y, relative.z);

  return Real(0.5) * (frameA * (relative.w * axis + glm::cross(v, axis)));
}

template <typename Real>
Real jointLimitImpulse(Real gap, Real rate, Real mass, Real invH, bool useBias, Real& impulse) {
  Real bias = Real(0);

  if (gap > Real(0)) {
    bias = gap * invH;
  } else if (useBias) {
    bias = std::max(gap * invH, -static_cast<Real>(jointLimitSpeed));
  }

  const Real old = impulse;

  impulse = std::max(old - mass * (rate + bias), Real(0));

  return impulse - old;
}

template <typename Real>
Real jointDriveImpulse(JointDriveRow<Real>& drive, Real angle, Real rate) {
  const JointSoftness<Real>& soft = drive.softness;
  const Real old = drive.impulse;
  const Real unbounded = old
                         - soft.massScale * drive.mass * (rate - drive.speed + soft.biasRate * (angle - drive.target))
                         - soft.impulseScale * old;
  const Real bounded = std::clamp(std::clamp(unbounded, drive.lower, drive.upper), -drive.cap, drive.cap);

  drive.saturated = bounded != unbounded;
  drive.impulse = bounded;

  return bounded - old;
}

template <typename Real>
Real jointAngularMass(const JointMat<Real>& invI, const JointVec<Real>& axis) {
  const Real k = glm::dot(axis, invI * axis);

  return k > Real(0) ? Real(1) / k : Real(0);
}

template <typename Real>
JointSwing<Real> jointSwingTwist(const JointQuat<Real>& rotation) {
  const Real small = static_cast<Real>(jointSmall);
  const JointQuat<Real> q = rotation.w < Real(0) ? -rotation : rotation;
  const Real twistNorm = std::sqrt(q.w * q.w + q.x * q.x);

  if (twistNorm < small) {
    const JointPair<Real> axis = glm::normalize(JointPair<Real>(q.y, q.z));

    return {Real(0), axis * std::numbers::pi_v<Real>, axis, twistNorm};
  }

  const Real cosine = q.w / twistNorm;
  const Real sine = q.x / twistNorm;
  const JointPair<Real> swing(q.y * cosine - q.z * sine, q.y * sine + q.z * cosine);
  const Real swingSine = glm::length(swing);
  const Real twist = Real(2) * std::atan2(sine, cosine);

  if (swingSine < small) {
    return {twist, swing * (Real(2) / twistNorm), swing, twistNorm};
  }

  const Real swingAngle = Real(2) * std::atan2(swingSine, twistNorm);

  return {twist, swing * (swingAngle / swingSine), swing, twistNorm};
}

template <typename Real>
JointQuat<Real> jointSwingTwistRotation(Real twist, const JointPair<Real>& swing) {
  const Real half = Real(0.5) * twist;
  const JointQuat<Real> turn(std::cos(half), std::sin(half), Real(0), Real(0));
  const Real angle = glm::length(swing);

  if (angle < static_cast<Real>(jointSmall)) {
    return glm::normalize(JointQuat<Real>(Real(1), Real(0), Real(0.5) * swing.x, Real(0.5) * swing.y)) * turn;
  }

  const JointPair<Real> axis = swing / angle;
  const Real sine = std::sin(Real(0.5) * angle);

  return JointQuat<Real>(std::cos(Real(0.5) * angle), Real(0), axis.x * sine, axis.y * sine) * turn;
}

template <typename Real>
JointVec<Real> jointRotationVector(const JointQuat<Real>& rotation) {
  const JointQuat<Real> q = rotation.w < Real(0) ? -rotation : rotation;
  const JointVec<Real> v(q.x, q.y, q.z);
  const Real sine = glm::length(v);

  if (sine < static_cast<Real>(jointSmall)) {
    return Real(2) * v;
  }

  return (Real(2) * std::atan2(sine, q.w) / sine) * v;
}

template <typename Real>
JointVec<Real> jointTwistAxis(const JointSwing<Real>& angles) {
  if (angles.cosine < static_cast<Real>(jointSmall)) {
    return JointVec<Real>(Real(1), Real(0), Real(0));
  }

  return JointVec<Real>(Real(1), angles.sine.y / angles.cosine, -angles.sine.x / angles.cosine);
}

template <typename Real>
JointVec<Real> jointSwingAxis(const JointSwing<Real>& angles, const JointPair<Real>& normal) {
  const Real small = static_cast<Real>(jointSmall);
  const Real sine = glm::length(angles.sine);

  if (sine < small) {
    return JointVec<Real>(Real(0), normal.x, normal.y);
  }

  const JointPair<Real> radial = angles.sine / sine;
  const JointPair<Real> around(-radial.y, radial.x);
  const Real angle = glm::length(angles.swing);
  const Real spread = Real(2) * sine * angles.cosine;
  const Real curl = spread > small ? angle * (angles.cosine * angles.cosine - sine * sine) / spread : Real(0);

  return glm::dot(normal, radial) * JointVec<Real>(Real(0), radial.x, radial.y)
         + glm::dot(normal, around) * JointVec<Real>(angle, curl * around.x, curl * around.y);
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
    end.frame = JointQuat<Real>(frame.rotation);
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
  end.frame = JointQuat<Real>(body.rotation * frame.rotation);
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

template <typename Real>
void prepareJointDrive(
  const AxisDrive& axisDrive,
  const JointDrive& jointDrive,
  double stored,
  const JointVec<Real>& axis,
  Real mass,
  Real h,
  JointDriveRow<Real>& drive
) {
  const double step = static_cast<double>(h);

  drive.axis = axis;
  drive.active = (axisDrive.stiffness > 0.0 || axisDrive.damping > 0.0) && mass > Real(0);

  if (drive.active) {
    drive.mass = mass;
    drive.softness = jointDriveSoftness(
      static_cast<Real>(axisDrive.stiffness),
      static_cast<Real>(axisDrive.damping),
      drive.mass,
      h
    );
    drive.speed = static_cast<Real>(axisDrive.targetVelocity);
    drive.lower = static_cast<Real>(axisDrive.minTorque * step);
    drive.upper = static_cast<Real>(axisDrive.maxTorque * step);
    drive.cap = static_cast<Real>(jointDrive.maxTorque * step);
    drive.impulse = static_cast<Real>(stored * step);
  }
}

template <typename Real>
void prepareHingeRows(const Joint& joint, Real h, JointRow<Real>& row) {
  const JointDef& def = joint.def;
  const JointImpulses& impulses = joint.impulses;
  const AxisDrive& axisDrive = def.drive.axes[0];
  const double step = static_cast<double>(h);
  const JointQuat<Real> relative = jointRelative(row.a.frame, row.b.frame);

  row.axis = row.a.frame * JointVec<Real>(Real(1), Real(0), Real(0));
  row.axialMass = jointAngularMass(row.a.invI + row.b.invI, row.axis);
  row.perpY = jointPerpAxis(row.a.frame, relative, JointVec<Real>(Real(0), Real(1), Real(0)));
  row.perpZ = jointPerpAxis(row.a.frame, relative, JointVec<Real>(Real(0), Real(0), Real(1)));
  row.alignmentImpulse = JointPair<Real>(impulses.alignment * step);
  row.limit = def.hinge.enabled;

  if (row.limit) {
    row.lower = static_cast<Real>(def.hinge.lower);
    row.upper = static_cast<Real>(def.hinge.upper);
    row.lowerImpulse = static_cast<Real>(impulses.hingeLower * step);
    row.upperImpulse = static_cast<Real>(impulses.hingeUpper * step);
  }

  prepareJointDrive(axisDrive, def.drive, impulses.drive.x, row.axis, row.axialMass, h, row.drives[0]);
  row.drives[0].target = static_cast<Real>(axisDrive.targetAngle);
  row.feedForward = JointVec<Real>(def.drive.feedForward * step);
}

template <typename Real>
JointVec<Real> hingeWarmImpulse(const JointRow<Real>& row) {
  return row.alignmentImpulse.x * row.perpY + row.alignmentImpulse.y * row.perpZ
         + (row.drives[0].impulse + row.lowerImpulse - row.upperImpulse) * row.axis + row.feedForward;
}

template <typename Real>
void solveHingeRows(JointRow<Real>& row, JointMotion<Real>& a, JointMotion<Real>& b, Real invH, bool useBias) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  const JointQuat<Real> frameA = a.dq * row.a.frame;
  const JointQuat<Real> relative = jointRelative(frameA, b.dq * row.b.frame);
  const Real angle = jointTwist(relative);
  const JointVec<Real> turnA = iA * row.axis;
  const JointVec<Real> turnB = iB * row.axis;

  if (row.drives[0].active) {
    const Real impulse = jointDriveImpulse(row.drives[0], angle, glm::dot(b.w - a.w, row.axis));

    a.w -= impulse * turnA;
    b.w += impulse * turnB;
  }

  if (row.limit) {
    const Real lower =
      jointLimitImpulse(angle - row.lower, glm::dot(b.w - a.w, row.axis), row.axialMass, invH, useBias, row.lowerImpulse);

    a.w -= lower * turnA;
    b.w += lower * turnB;

    const Real upper =
      jointLimitImpulse(row.upper - angle, glm::dot(a.w - b.w, row.axis), row.axialMass, invH, useBias, row.upperImpulse);

    a.w += upper * turnA;
    b.w -= upper * turnB;
  }

  JointPair<Real> bias(Real(0));
  Real massScale = Real(1);
  Real impulseScale = Real(0);

  if (useBias) {
    bias = row.softness.biasRate * JointPair<Real>(relative.y, relative.z);
    massScale = row.softness.massScale;
    impulseScale = row.softness.impulseScale;
  }

  row.perpY = jointPerpAxis(frameA, relative, JointVec<Real>(Real(0), Real(1), Real(0)));
  row.perpZ = jointPerpAxis(frameA, relative, JointVec<Real>(Real(0), Real(0), Real(1)));

  const JointMat<Real> invI = iA + iB;
  const JointVec<Real> spreadY = invI * row.perpY;
  const JointVec<Real> spreadZ = invI * row.perpZ;
  const JointVec<Real> spin = b.w - a.w;
  const JointPair<Real> rate(glm::dot(spin, row.perpY), glm::dot(spin, row.perpZ));
  const JointPair<Real> impulse =
    -massScale
      * jointSolve2(glm::dot(row.perpY, spreadY), glm::dot(row.perpY, spreadZ), glm::dot(row.perpZ, spreadZ), rate + bias)
    - impulseScale * row.alignmentImpulse;
  const JointVec<Real> angular = impulse.x * row.perpY + impulse.y * row.perpZ;

  row.alignmentImpulse += impulse;
  a.w -= iA * angular;
  b.w += iB * angular;
}

template <typename Real>
void storeHingeRows(Joint& joint, const JointRow<Real>& row, Real invH) {
  const double rate = static_cast<double>(invH);
  const JointVec<Real> torque = row.alignmentImpulse.x * row.perpY + row.alignmentImpulse.y * row.perpZ
                                + (row.lowerImpulse - row.upperImpulse) * row.axis;
  const double drive = static_cast<double>(row.drives[0].impulse) * rate;

  joint.impulses.alignment = glm::dvec2(row.alignmentImpulse) * rate;
  joint.impulses.hingeLower = static_cast<double>(row.lowerImpulse) * rate;
  joint.impulses.hingeUpper = static_cast<double>(row.upperImpulse) * rate;
  joint.impulses.drive = glm::dvec3(drive, 0.0, 0.0);
  joint.report.torque = glm::dvec3(torque) * rate;
  joint.report.driveTorque = glm::dvec3(drive, 0.0, 0.0);
  joint.report.limitActive = row.lowerImpulse > Real(0) || row.upperImpulse > Real(0);
  joint.report.driveSaturated = row.drives[0].active && row.drives[0].saturated;
}

template <typename Real>
Real jointConeGap(JointRow<Real>& row, const JointQuat<Real>& frameA, const JointSwing<Real>& angles) {
  const JointPair<Real> scaled = (angles.swing - row.coneCentre) / row.coneHalf;
  const JointPair<Real> gradient = scaled / row.coneHalf;
  const Real slope = glm::length(gradient);

  if (!(slope > Real(0))) {
    row.coneAxis = JointVec<Real>(Real(0));

    return std::min(row.coneHalf.x, row.coneHalf.y);
  }

  const Real reach = glm::length(scaled);

  row.coneAxis = frameA * jointSwingAxis(angles, gradient / slope);

  return (Real(1) - reach) * reach / slope;
}

template <typename Real>
void prepareSwingTwistRows(const Joint& joint, Real h, JointRow<Real>& row) {
  const JointDef& def = joint.def;
  const JointImpulses& impulses = joint.impulses;
  const std::array<AxisDrive, 3>& axes = def.drive.axes;
  const double step = static_cast<double>(h);
  const JointMat<Real> invI = row.a.invI + row.b.invI;
  const std::array<JointVec<Real>, 3> frameAxes{
    row.b.frame * JointVec<Real>(Real(1), Real(0), Real(0)),
    row.b.frame * JointVec<Real>(Real(0), Real(1), Real(0)),
    row.b.frame * JointVec<Real>(Real(0), Real(0), Real(1))
  };

  row.target = JointQuat<Real>(
    jointSwingTwistRotation(axes[0].targetAngle, glm::dvec2(axes[1].targetAngle, axes[2].targetAngle))
  );

  for (size_t index = 0u; index != frameAxes.size(); ++index) {
    const JointVec<Real>& axis = frameAxes[index];

    prepareJointDrive(
      axes[index],
      def.drive,
      impulses.drive[static_cast<glm::length_t>(index)],
      axis,
      jointAngularMass(invI, axis),
      h,
      row.drives[index]
    );
  }

  row.feedForward = JointVec<Real>(def.drive.feedForward * step);
  row.limit = def.cone.enabled;

  if (row.limit) {
    row.lower = static_cast<Real>(def.cone.twistLower);
    row.upper = static_cast<Real>(def.cone.twistUpper);
    row.coneCentre = JointPair<Real>(def.cone.centre);
    row.coneHalf = JointPair<Real>(def.cone.swing);
    row.lowerImpulse = static_cast<Real>(impulses.twistLower * step);
    row.upperImpulse = static_cast<Real>(impulses.twistUpper * step);
    row.coneImpulse = static_cast<Real>(impulses.cone * step);

    const JointSwing<Real> angles = jointSwingTwist(jointRelative(row.a.frame, row.b.frame));

    row.axis = row.a.frame * jointTwistAxis(angles);
    (void)jointConeGap(row, row.a.frame, angles);
  }
}

template <typename Real>
JointVec<Real> swingTwistWarmImpulse(const JointRow<Real>& row) {
  JointVec<Real> angular =
    (row.lowerImpulse - row.upperImpulse) * row.axis - row.coneImpulse * row.coneAxis + row.feedForward;

  for (const JointDriveRow<Real>& drive : row.drives) {
    angular += drive.impulse * drive.axis;
  }

  return angular;
}

template <typename Real>
void solveSwingTwistDrive(JointRow<Real>& row, JointMotion<Real>& a, JointMotion<Real>& b, const JointVec<Real>& error) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  Real cap = Real(0);
  Real total = Real(0);

  for (JointDriveRow<Real>& drive : row.drives) {
    if (!drive.active) {
      continue;
    }

    const Real impulse = jointDriveImpulse(drive, glm::dot(error, drive.axis), glm::dot(b.w - a.w, drive.axis));

    a.w -= impulse * (iA * drive.axis);
    b.w += impulse * (iB * drive.axis);
    cap = drive.cap;
    total += drive.impulse * drive.impulse;
  }

  if (total <= cap * cap) {
    return;
  }

  const Real scale = cap / std::sqrt(total);

  for (JointDriveRow<Real>& drive : row.drives) {
    if (!drive.active) {
      continue;
    }

    const Real bounded = scale * drive.impulse;
    const Real impulse = bounded - drive.impulse;

    drive.impulse = bounded;
    drive.saturated = true;
    a.w -= impulse * (iA * drive.axis);
    b.w += impulse * (iB * drive.axis);
  }
}

template <typename Real>
void solveSwingTwistRows(JointRow<Real>& row, JointMotion<Real>& a, JointMotion<Real>& b, Real invH, bool useBias) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  const JointQuat<Real> frameA = a.dq * row.a.frame;
  const JointQuat<Real> frameB = b.dq * row.b.frame;
  const JointQuat<Real> relative = jointRelative(frameA, frameB);

  solveSwingTwistDrive(row, a, b, frameB * jointRotationVector(glm::conjugate(row.target) * relative));

  if (!row.limit) {
    return;
  }

  const JointSwing<Real> angles = jointSwingTwist(relative);
  const JointMat<Real> invI = iA + iB;

  row.axis = frameA * jointTwistAxis(angles);

  const Real twistMass = jointAngularMass(invI, row.axis);
  const JointVec<Real> turnA = iA * row.axis;
  const JointVec<Real> turnB = iB * row.axis;
  const Real lower =
    jointLimitImpulse(angles.twist - row.lower, glm::dot(b.w - a.w, row.axis), twistMass, invH, useBias, row.lowerImpulse);

  a.w -= lower * turnA;
  b.w += lower * turnB;

  const Real upper =
    jointLimitImpulse(row.upper - angles.twist, glm::dot(a.w - b.w, row.axis), twistMass, invH, useBias, row.upperImpulse);

  a.w += upper * turnA;
  b.w -= upper * turnB;

  const Real gap = jointConeGap(row, frameA, angles);
  const Real cone = jointLimitImpulse(
    gap,
    glm::dot(a.w - b.w, row.coneAxis),
    jointAngularMass(invI, row.coneAxis),
    invH,
    useBias,
    row.coneImpulse
  );

  a.w += cone * (iA * row.coneAxis);
  b.w -= cone * (iB * row.coneAxis);
}

template <typename Real>
void storeSwingTwistRows(Joint& joint, const JointRow<Real>& row, Real invH) {
  const double rate = static_cast<double>(invH);
  const JointVec<Real> torque = (row.lowerImpulse - row.upperImpulse) * row.axis - row.coneImpulse * row.coneAxis;
  glm::dvec3 drive(0.0);
  bool saturated = false;

  for (size_t index = 0u; index != row.drives.size(); ++index) {
    const JointDriveRow<Real>& axis = row.drives[index];

    drive[static_cast<glm::length_t>(index)] = static_cast<double>(axis.impulse) * rate;
    saturated = saturated || (axis.active && axis.saturated);
  }

  joint.impulses.twistLower = static_cast<double>(row.lowerImpulse) * rate;
  joint.impulses.twistUpper = static_cast<double>(row.upperImpulse) * rate;
  joint.impulses.cone = static_cast<double>(row.coneImpulse) * rate;
  joint.impulses.drive = drive;
  joint.report.torque = glm::dvec3(torque) * rate;
  joint.report.driveTorque = drive;
  joint.report.limitActive = row.lowerImpulse > Real(0) || row.upperImpulse > Real(0) || row.coneImpulse > Real(0);
  joint.report.driveSaturated = saturated;
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
    row.kind = joint.def.kind;

    if (row.kind == JointKind::Hinge) {
      prepareHingeRows(joint, h, row);
    } else if (row.kind == JointKind::SwingTwist) {
      prepareSwingTwistRows(joint, h, row);
    }

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
    JointVec<Real> turnA = glm::cross(rA, impulse);
    JointVec<Real> turnB = glm::cross(rB, impulse);

    if (row.kind == JointKind::Hinge || row.kind == JointKind::SwingTwist) {
      const JointVec<Real> angular = row.kind == JointKind::Hinge ? hingeWarmImpulse(row) : swingTwistWarmImpulse(row);

      turnA += angular;
      turnB += angular;
    }

    a.v -= row.a.invMass * impulse;
    a.w -= row.a.invI * turnA;
    b.v += row.b.invMass * impulse;
    b.w += row.b.invI * turnB;
    storeJointMotion(states, row.a, a);
    storeJointMotion(states, row.b, b);
  }
}

template <typename Real, typename State>
void solveJointRows(
  std::span<State> states,
  std::span<JointRow<Real>> rows,
  Real invH,
  bool useBias
) {
  for (JointRow<Real>& row : rows) {
    JointMotion<Real> a = jointMotion(states, row.a);
    JointMotion<Real> b = jointMotion(states, row.b);

    if (row.kind == JointKind::Hinge) {
      solveHingeRows(row, a, b, invH, useBias);
    } else if (row.kind == JointKind::SwingTwist) {
      solveSwingTwistRows(row, a, b, invH, useBias);
    }

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

    if (row.kind == JointKind::Hinge) {
      storeHingeRows(joint, row, invH);
    } else if (row.kind == JointKind::SwingTwist) {
      storeSwingTwistRows(joint, row, invH);
    }
  }
}

}
