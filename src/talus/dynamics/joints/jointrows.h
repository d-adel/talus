#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/joints/jointrecords.h>
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

template<typename Real>
using JointVec = glm::vec<3, Real>;

template<typename Real>
using JointQuat = glm::qua<Real>;

template<typename Real>
using JointMat = glm::mat<3, 3, Real>;

template<typename Real>
using JointPair = glm::vec<2, Real>;

inline constexpr double jointLimitSpeed = 30.0;

inline constexpr double jointLimitWatchGap = 0.05;

inline constexpr uint32_t jointLimitPasses = 2u;

inline constexpr double jointSmall = 1.0e-6;

template<typename Real>
struct JointSoftness {
  Real biasRate = Real(0);
  Real massScale = Real(0);
  Real impulseScale = Real(0);
};

template<typename Real>
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

template<typename Real>
struct JointDriveRow {
  JointSoftness<Real> softness;
  JointVec<Real> axis{Real(0)};
  Real stiffness = Real(0);
  Real damping = Real(0);
  Real step = Real(0);
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

template<typename Real>
struct JointSpringBlock {
  Real rateGain = Real(0);
  Real errorGain = Real(0);
  Real cap = Real(0);
  JointVec<Real> impulse{Real(0)};
  bool active = false;
  bool saturated = false;
};

template<typename Real>
struct JointPlanarRow {
  JointVec<Real> axisX{Real(0)};
  JointVec<Real> axisZ{Real(0)};
  JointVec<Real> target{Real(0)};
  Real mass = Real(0);
  Real cap = Real(0);
  JointPair<Real> impulse{Real(0)};
  bool active = false;
  bool saturated = false;
};

template<typename Real>
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
  bool watched = false;
  Real lower = Real(0);
  Real upper = Real(0);
  Real lowerGap = Real(0);
  Real upperGap = Real(0);
  Real lowerImpulse = Real(0);
  Real upperImpulse = Real(0);
  JointPair<Real> coneCentre{Real(0)};
  JointPair<Real> coneHalf{Real(0)};
  JointVec<Real> coneAxis{Real(0)};
  Real coneGap = Real(0);
  Real coneMass = Real(0);
  Real coneImpulse = Real(0);
  std::array<JointDriveRow<Real>, 3> drives;
  JointVec<Real> feedForward{Real(0)};
  JointSpringBlock<Real> springLinear;
  JointSpringBlock<Real> springAngular;
  JointPlanarRow<Real> planar;
};

template<typename Real>
struct JointSwing {
  Real twist = Real(0);
  JointPair<Real> swing{Real(0)};
  JointPair<Real> sine{Real(0)};
  Real cosine = Real(1);
};

template<typename Real>
struct JointMotion {
  JointVec<Real> v{Real(0)};
  JointVec<Real> w{Real(0)};
  JointVec<Real> dp{Real(0)};
  JointQuat<Real> dq{Real(1), Real(0), Real(0), Real(0)};
};

template<typename Real>
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

template<typename Real>
JointSoftness<Real> jointDriveSoftness(
  Real stiffness,
  Real damping,
  Real mass,
  Real h
) {
  const Real resistance = damping + h * stiffness;

  if (resistance == Real(0) || mass == Real(0)) {
    return {};
  }

  const Real a2 = h * resistance / mass;
  const Real a3 = Real(1) / (Real(1) + a2);

  return {stiffness / resistance, a2 * a3, a3};
}

template<typename Real>
JointMat<Real> jointSkew(const JointVec<Real>& v) {
  return JointMat<Real>(
    JointVec<Real>(Real(0), v.z, -v.y),
    JointVec<Real>(-v.z, Real(0), v.x),
    JointVec<Real>(v.y, -v.x, Real(0))
  );
}

template<typename Real>
JointMat<Real> jointPointMass(
  Real mA,
  Real mB,
  const JointMat<Real>& iA,
  const JointMat<Real>& iB,
  const JointVec<Real>& rA,
  const JointVec<Real>& rB
) {
  const JointMat<Real> sA = jointSkew(rA);
  const JointMat<Real> sB = jointSkew(rB);
  JointMat<Real> k = -(sA * iA * sA + sB * iB * sB);

  k[0][0] += mA + mB;
  k[1][1] += mA + mB;
  k[2][2] += mA + mB;

  return k;
}

template<typename Real>
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

template<typename Real>
JointPair<Real> jointSolve2(
  Real kxx,
  Real kxy,
  Real kyy,
  const JointPair<Real>& a
) {
  const Real det = kxx * kyy - kxy * kxy;

  if (det <= Real(1000) * std::numeric_limits<Real>::min()) {
    return JointPair<Real>(Real(0));
  }

  const Real invDet = Real(1) / det;

  return JointPair<Real>(
    invDet * (kyy * a.x - kxy * a.y),
    invDet * (kxx * a.y - kxy * a.x)
  );
}

template<typename Real>
JointQuat<Real> jointTurn(
  const JointQuat<Real>& q,
  const JointVec<Real>& delta
) {
  const JointQuat<Real> qd = JointQuat<Real>(
                               Real(0),
                               Real(0.5) * delta.x,
                               Real(0.5) * delta.y,
                               Real(0.5) * delta.z
                             )
                             * q;

  return glm::normalize(
    JointQuat<Real>(q.w + qd.w, q.x + qd.x, q.y + qd.y, q.z + qd.z)
  );
}

template<typename Real>
JointQuat<Real> jointRelative(
  const JointQuat<Real>& frameA,
  JointQuat<Real> frameB
) {
  if (glm::dot(frameA, frameB) < Real(0)) {
    frameB = -frameB;
  }

  return glm::conjugate(frameA) * frameB;
}

template<typename Real>
Real jointTwist(const JointQuat<Real>& relative) {
  return relative.w < Real(0) ? Real(2) * std::atan2(-relative.x, -relative.w)
                              : Real(2) * std::atan2(relative.x, relative.w);
}

template<typename Real>
JointVec<Real> jointPerpAxis(
  const JointQuat<Real>& frameA,
  const JointQuat<Real>& relative,
  const JointVec<Real>& axis
) {
  const JointVec<Real> v(relative.x, relative.y, relative.z);

  return Real(0.5) * (frameA * (relative.w * axis + glm::cross(v, axis)));
}

template<typename Real>
Real jointLimitImpulse(
  Real gap,
  Real rate,
  Real mass,
  Real invH,
  bool useBias,
  Real& impulse
) {
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

template<typename Real>
void solveJointLimits(
  JointRow<Real>& row,
  JointMotion<Real>& a,
  JointMotion<Real>& b,
  Real invH,
  bool useBias
) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  const JointVec<Real> turnA = iA * row.axis;
  const JointVec<Real> turnB = iB * row.axis;
  const Real lower = jointLimitImpulse(
    row.lowerGap,
    glm::dot(b.w - a.w, row.axis),
    row.axialMass,
    invH,
    useBias,
    row.lowerImpulse
  );

  a.w -= lower * turnA;
  b.w += lower * turnB;

  const Real upper = jointLimitImpulse(
    row.upperGap,
    glm::dot(a.w - b.w, row.axis),
    row.axialMass,
    invH,
    useBias,
    row.upperImpulse
  );

  a.w += upper * turnA;
  b.w -= upper * turnB;

  if (row.kind != JointKind::SwingTwist) {
    return;
  }

  const Real cone = jointLimitImpulse(
    row.coneGap,
    glm::dot(a.w - b.w, row.coneAxis),
    row.coneMass,
    invH,
    useBias,
    row.coneImpulse
  );

  a.w += cone * (iA * row.coneAxis);
  b.w -= cone * (iB * row.coneAxis);
}

template<typename Real>
bool jointLimitWatched(const JointRow<Real>& row) {
  const Real gap = static_cast<Real>(jointLimitWatchGap);
  const bool range = row.lowerImpulse > Real(0) || row.upperImpulse > Real(0)
                     || row.lowerGap < gap || row.upperGap < gap;

  return range
         || (row.kind == JointKind::SwingTwist && (row.coneImpulse > Real(0) || row.coneGap < gap));
}

template<typename Real>
Real jointDriveImpulse(JointDriveRow<Real>& drive, Real angle, Real rate) {
  const JointSoftness<Real>& soft = drive.softness;
  const Real old = drive.impulse;
  const Real unbounded =
    old
    - soft.massScale * drive.mass
        * (rate - drive.speed + soft.biasRate * (angle - drive.target))
    - soft.impulseScale * old;
  const Real bounded = std::clamp(
    std::clamp(unbounded, drive.lower, drive.upper),
    -drive.cap,
    drive.cap
  );

  drive.saturated = bounded != unbounded;
  drive.impulse = bounded;

  return bounded - old;
}

template<typename Real>
Real jointAngularMass(const JointMat<Real>& invI, const JointVec<Real>& axis) {
  const Real k = glm::dot(axis, invI * axis);

  return k > Real(0) ? Real(1) / k : Real(0);
}

template<typename Real>
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
  const JointPair<Real> swing(
    q.y * cosine - q.z * sine,
    q.y * sine + q.z * cosine
  );
  const Real swingSine = glm::length(swing);
  const Real twist = Real(2) * std::atan2(sine, cosine);

  if (swingSine < small) {
    return {twist, swing * (Real(2) / twistNorm), swing, twistNorm};
  }

  const Real swingAngle = Real(2) * std::atan2(swingSine, twistNorm);

  return {twist, swing * (swingAngle / swingSine), swing, twistNorm};
}

template<typename Real>
JointVec<Real> jointTwistAxis(const JointSwing<Real>& angles) {
  if (angles.cosine < static_cast<Real>(jointSmall)) {
    return JointVec<Real>(Real(1), Real(0), Real(0));
  }

  return JointVec<Real>(
    Real(1),
    angles.sine.y / angles.cosine,
    -angles.sine.x / angles.cosine
  );
}

template<typename Real>
JointVec<Real> jointSwingAxis(
  const JointSwing<Real>& angles,
  const JointPair<Real>& normal
) {
  const Real small = static_cast<Real>(jointSmall);
  const Real sine = glm::length(angles.sine);

  if (sine < small) {
    return JointVec<Real>(Real(0), normal.x, normal.y);
  }

  const JointPair<Real> radial = angles.sine / sine;
  const JointPair<Real> around(-radial.y, radial.x);
  const Real angle = glm::length(angles.swing);
  const Real spread = Real(2) * sine * angles.cosine;
  const Real curl =
    spread > small
      ? angle * (angles.cosine * angles.cosine - sine * sine) / spread
      : Real(0);

  return glm::dot(normal, radial) * JointVec<Real>(Real(0), radial.x, radial.y)
         + glm::dot(normal, around)
             * JointVec<Real>(angle, curl * around.x, curl * around.y);
}

template<typename Real, typename State>
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

  if (ref.side != Side::Rigid || ref.index >= bodies.rigids.size()
      || bodies.rigids[ref.index].id != id) {
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

template<typename Real, typename State>
JointMotion<Real> jointMotion(
  std::span<State> states,
  const JointEnd<Real>& end
) {
  if (end.state == 0u) {
    return {end.v, end.w, end.dp, end.dq};
  }

  const State& state = states[end.state];

  return {state.v, state.w, state.dp, state.dq};
}

template<typename Real, typename State>
void storeJointMotion(
  std::span<State> states,
  const JointEnd<Real>& end,
  const JointMotion<Real>& motion
) {
  if (end.invMass != Real(0)) {
    states[end.state].v = motion.v;
    states[end.state].w = motion.w;
  }
}

template<typename Real>
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
  drive.active =
    (axisDrive.stiffness > 0.0 || axisDrive.damping > 0.0) && mass > Real(0);

  if (drive.active) {
    drive.stiffness = static_cast<Real>(axisDrive.stiffness);
    drive.damping = static_cast<Real>(axisDrive.damping);
    drive.step = h;
    drive.mass = mass;
    drive.softness =
      jointDriveSoftness(drive.stiffness, drive.damping, drive.mass, h);
    drive.speed = static_cast<Real>(axisDrive.targetVelocity);
    drive.lower = static_cast<Real>(axisDrive.minTorque * step);
    drive.upper = static_cast<Real>(axisDrive.maxTorque * step);
    drive.cap = static_cast<Real>(jointDrive.maxTorque * step);
    drive.impulse = static_cast<Real>(stored * step);
  }
}

template<typename Real>
void aimJointDrive(
  JointDriveRow<Real>& drive,
  const JointMat<Real>& invI,
  const JointVec<Real>& axis
) {
  drive.axis = axis;
  drive.mass = jointAngularMass(invI, axis);
  drive.softness =
    jointDriveSoftness(drive.stiffness, drive.damping, drive.mass, drive.step);
}

template<typename Real>
void prepareHingeRows(const Joint& joint, Real h, JointRow<Real>& row) {
  const JointDef& def = joint.def;
  const JointImpulses& impulses = joint.impulses;
  const AxisDrive& axisDrive = def.drive.axes[0];
  const double step = static_cast<double>(h);
  const JointQuat<Real> relative = jointRelative(row.a.frame, row.b.frame);

  row.axis = row.a.frame * JointVec<Real>(Real(1), Real(0), Real(0));
  row.axialMass = jointAngularMass(row.a.invI + row.b.invI, row.axis);
  row.perpY = jointPerpAxis(
    row.a.frame,
    relative,
    JointVec<Real>(Real(0), Real(1), Real(0))
  );
  row.perpZ = jointPerpAxis(
    row.a.frame,
    relative,
    JointVec<Real>(Real(0), Real(0), Real(1))
  );
  row.alignmentImpulse = JointPair<Real>(impulses.alignment * step);
  row.limit = def.hinge.enabled;

  if (row.limit) {
    row.lower = static_cast<Real>(def.hinge.lower);
    row.upper = static_cast<Real>(def.hinge.upper);
    row.lowerImpulse = static_cast<Real>(impulses.hingeLower * step);
    row.upperImpulse = static_cast<Real>(impulses.hingeUpper * step);
  }

  prepareJointDrive(
    axisDrive,
    def.drive,
    impulses.drive.x,
    row.axis,
    row.axialMass,
    h,
    row.drives[0]
  );
  row.drives[0].target = static_cast<Real>(axisDrive.targetAngle);
  row.feedForward = JointVec<Real>(def.drive.feedForward * step);
}

template<typename Real>
JointVec<Real> hingeWarmImpulse(const JointRow<Real>& row) {
  return row.alignmentImpulse.x * row.perpY + row.alignmentImpulse.y * row.perpZ
         + (row.drives[0].impulse + row.lowerImpulse - row.upperImpulse)
             * row.axis
         + row.feedForward;
}

template<typename Real>
void solveHingeRows(
  JointRow<Real>& row,
  JointMotion<Real>& a,
  JointMotion<Real>& b,
  Real invH,
  bool useBias
) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  const JointQuat<Real> frameA = a.dq * row.a.frame;
  const JointQuat<Real> relative = jointRelative(frameA, b.dq * row.b.frame);
  const Real angle = jointTwist(relative);
  const JointVec<Real> turnA = iA * row.axis;
  const JointVec<Real> turnB = iB * row.axis;

  if (row.drives[0].active) {
    const Real impulse =
      jointDriveImpulse(row.drives[0], angle, glm::dot(b.w - a.w, row.axis));

    a.w -= impulse * turnA;
    b.w += impulse * turnB;
  }

  if (row.limit) {
    row.lowerGap = angle - row.lower;
    row.upperGap = row.upper - angle;
    solveJointLimits(row, a, b, invH, useBias);
  }

  JointPair<Real> bias(Real(0));
  Real massScale = Real(1);
  Real impulseScale = Real(0);

  if (useBias) {
    bias = row.softness.biasRate * JointPair<Real>(relative.y, relative.z);
    massScale = row.softness.massScale;
    impulseScale = row.softness.impulseScale;
  }

  row.perpY =
    jointPerpAxis(frameA, relative, JointVec<Real>(Real(0), Real(1), Real(0)));
  row.perpZ =
    jointPerpAxis(frameA, relative, JointVec<Real>(Real(0), Real(0), Real(1)));

  const JointMat<Real> invI = iA + iB;
  const JointVec<Real> spreadY = invI * row.perpY;
  const JointVec<Real> spreadZ = invI * row.perpZ;
  const JointVec<Real> spin = b.w - a.w;
  const JointPair<Real> rate(
    glm::dot(spin, row.perpY),
    glm::dot(spin, row.perpZ)
  );
  const JointPair<Real> impulse = -massScale
                                    * jointSolve2(
                                      glm::dot(row.perpY, spreadY),
                                      glm::dot(row.perpY, spreadZ),
                                      glm::dot(row.perpZ, spreadZ),
                                      rate + bias
                                    )
                                  - impulseScale * row.alignmentImpulse;
  const JointVec<Real> angular = impulse.x * row.perpY + impulse.y * row.perpZ;

  row.alignmentImpulse += impulse;
  a.w -= iA * angular;
  b.w += iB * angular;
}

template<typename Real>
void storeHingeRows(Joint& joint, const JointRow<Real>& row, Real invH) {
  const double rate = static_cast<double>(invH);
  const JointVec<Real> torque =
    row.alignmentImpulse.x * row.perpY + row.alignmentImpulse.y * row.perpZ
    + (row.lowerImpulse - row.upperImpulse) * row.axis;
  const double drive = static_cast<double>(row.drives[0].impulse) * rate;

  joint.impulses.alignment = glm::dvec2(row.alignmentImpulse) * rate;
  joint.impulses.hingeLower = static_cast<double>(row.lowerImpulse) * rate;
  joint.impulses.hingeUpper = static_cast<double>(row.upperImpulse) * rate;
  joint.impulses.drive = glm::dvec3(drive, 0.0, 0.0);
  joint.report.torque = glm::dvec3(torque) * rate;
  joint.report.driveTorque = glm::dvec3(drive, 0.0, 0.0);
  joint.report.limitActive =
    row.lowerImpulse > Real(0) || row.upperImpulse > Real(0);
  joint.report.driveSaturated = row.drives[0].active && row.drives[0].saturated;
}

template<typename Real>
Real jointConeGap(
  JointRow<Real>& row,
  const JointQuat<Real>& frameA,
  const JointSwing<Real>& angles
) {
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

template<typename Real>
std::array<JointVec<Real>, 3> jointSwingTwistAxes(
  const JointQuat<Real>& frameA,
  const JointSwing<Real>& angles
) {
  return {
    frameA * jointTwistAxis(angles),
    frameA * jointSwingAxis(angles, JointPair<Real>(Real(1), Real(0))),
    frameA * jointSwingAxis(angles, JointPair<Real>(Real(0), Real(1)))
  };
}

template<typename Real>
void prepareSwingTwistRows(const Joint& joint, Real h, JointRow<Real>& row) {
  const JointDef& def = joint.def;
  const JointImpulses& impulses = joint.impulses;
  const double step = static_cast<double>(h);
  const JointMat<Real> invI = row.a.invI + row.b.invI;
  const JointSwing<Real> angles =
    jointSwingTwist(jointRelative(row.a.frame, row.b.frame));
  const std::array<JointVec<Real>, 3> directions =
    jointSwingTwistAxes(row.a.frame, angles);

  for (size_t index = 0u; index != directions.size(); ++index) {
    const AxisDrive& axisDrive = def.drive.axes[index];
    const JointVec<Real>& direction = directions[index];

    prepareJointDrive(
      axisDrive,
      def.drive,
      impulses.drive[static_cast<glm::length_t>(index)],
      direction,
      jointAngularMass(invI, direction),
      h,
      row.drives[index]
    );
    row.drives[index].target = static_cast<Real>(axisDrive.targetAngle);
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
    row.axis = directions[0];
    (void)jointConeGap(row, row.a.frame, angles);
  }
}

template<typename Real>
JointVec<Real> swingTwistWarmImpulse(const JointRow<Real>& row) {
  JointVec<Real> angular = (row.lowerImpulse - row.upperImpulse) * row.axis
                           - row.coneImpulse * row.coneAxis + row.feedForward;

  for (const JointDriveRow<Real>& drive : row.drives) {
    angular += drive.impulse * drive.axis;
  }

  return angular;
}

template<typename Real>
void solveSwingTwistDrive(
  JointRow<Real>& row,
  JointMotion<Real>& a,
  JointMotion<Real>& b,
  const JointSwing<Real>& angles
) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  const std::array<Real, 3> coordinates{
    angles.twist,
    angles.swing.x,
    angles.swing.y
  };
  Real cap = Real(0);
  JointVec<Real> torque(Real(0));

  for (size_t index = 0u; index != row.drives.size(); ++index) {
    JointDriveRow<Real>& drive = row.drives[index];

    if (!drive.active) {
      continue;
    }

    const Real impulse = jointDriveImpulse(
      drive,
      coordinates[index],
      glm::dot(b.w - a.w, drive.axis)
    );

    a.w -= impulse * (iA * drive.axis);
    b.w += impulse * (iB * drive.axis);
    cap = drive.cap;
    torque += drive.impulse * drive.axis;
  }

  if (glm::dot(torque, torque) <= cap * cap) {
    return;
  }

  const Real scale = cap / glm::length(torque);

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

template<typename Real>
void solveSwingTwistRows(
  JointRow<Real>& row,
  JointMotion<Real>& a,
  JointMotion<Real>& b,
  Real invH,
  bool useBias
) {
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;
  const JointMat<Real> invI = iA + iB;
  const JointQuat<Real> frameA = a.dq * row.a.frame;
  const JointSwing<Real> angles =
    jointSwingTwist(jointRelative(frameA, b.dq * row.b.frame));
  const std::array<JointVec<Real>, 3> directions =
    jointSwingTwistAxes(frameA, angles);

  for (size_t index = 0u; index != directions.size(); ++index) {
    if (row.drives[index].active) {
      aimJointDrive(row.drives[index], invI, directions[index]);
    }
  }

  solveSwingTwistDrive(row, a, b, angles);

  if (!row.limit) {
    return;
  }

  row.axis = directions[0];
  row.axialMass = jointAngularMass(invI, row.axis);
  row.lowerGap = angles.twist - row.lower;
  row.upperGap = row.upper - angles.twist;
  row.coneGap = jointConeGap(row, frameA, angles);
  row.coneMass = jointAngularMass(invI, row.coneAxis);
  solveJointLimits(row, a, b, invH, useBias);
}

template<typename Real>
void storeSwingTwistRows(Joint& joint, const JointRow<Real>& row, Real invH) {
  const double rate = static_cast<double>(invH);
  const JointVec<Real> torque = (row.lowerImpulse - row.upperImpulse) * row.axis
                                - row.coneImpulse * row.coneAxis;
  glm::dvec3 drive(0.0);
  bool saturated = false;

  for (size_t index = 0u; index != row.drives.size(); ++index) {
    const JointDriveRow<Real>& axis = row.drives[index];

    drive[static_cast<glm::length_t>(index)] =
      static_cast<double>(axis.impulse) * rate;
    saturated = saturated || (axis.active && axis.saturated);
  }

  joint.impulses.twistLower = static_cast<double>(row.lowerImpulse) * rate;
  joint.impulses.twistUpper = static_cast<double>(row.upperImpulse) * rate;
  joint.impulses.cone = static_cast<double>(row.coneImpulse) * rate;
  joint.impulses.drive = drive;
  joint.report.torque = glm::dvec3(torque) * rate;
  joint.report.driveTorque = drive;
  joint.report.limitActive = row.lowerImpulse > Real(0)
                             || row.upperImpulse > Real(0)
                             || row.coneImpulse > Real(0);
  joint.report.driveSaturated = saturated;
}

inline JointFrame jointSpringFrame(const JointDef& def) {
  const JointFrame& frame = def.frameA;
  const JointFrame& target = def.spring.target;

  return {
    frame.position + frame.rotation * target.position,
    frame.rotation * target.rotation
  };
}

template<typename Real>
void prepareSpringBlock(
  double stiffness,
  double damping,
  double cap,
  const glm::dvec3& stored,
  Real h,
  JointSpringBlock<Real>& block
) {
  const double step = static_cast<double>(h);

  block.active = (stiffness > 0.0 || damping > 0.0) && cap > 0.0;

  if (block.active) {
    block.rateGain = static_cast<Real>(step * (damping + step * stiffness));
    block.errorGain = static_cast<Real>(step * stiffness);
    block.cap = static_cast<Real>(cap * step);
    block.impulse = JointVec<Real>(stored * step);
  }
}

template<typename Real>
void prepareSpringRows(const Joint& joint, Real h, JointRow<Real>& row) {
  const SpringSettings& spring = joint.def.spring;
  const JointImpulses& impulses = joint.impulses;

  prepareSpringBlock(
    spring.linearStiffness,
    spring.linearDamping,
    spring.maxForce,
    impulses.springLinear,
    h,
    row.springLinear
  );
  prepareSpringBlock(
    spring.angularStiffness,
    spring.angularDamping,
    spring.maxTorque,
    impulses.springAngular,
    h,
    row.springAngular
  );
}

template<typename Real>
JointVec<Real> jointSpringImpulse(
  JointSpringBlock<Real>& block,
  const JointMat<Real>& k,
  const JointVec<Real>& rate,
  const JointVec<Real>& error
) {
  const JointVec<Real> old = block.impulse;
  JointMat<Real> system = block.rateGain * k;

  system[0][0] += Real(1);
  system[1][1] += Real(1);
  system[2][2] += Real(1);

  const JointVec<Real> unbounded =
    old
    - jointSolve3(
      system,
      block.rateGain * rate + block.errorGain * error + old
    );
  const Real square = glm::dot(unbounded, unbounded);

  block.saturated = square > block.cap * block.cap;
  block.impulse =
    block.saturated ? (block.cap / std::sqrt(square)) * unbounded : unbounded;

  return block.impulse - old;
}

template<typename Real>
void solveSpringRows(
  JointRow<Real>& row,
  JointMotion<Real>& a,
  JointMotion<Real>& b
) {
  const Real mA = row.a.invMass;
  const Real mB = row.b.invMass;
  const JointMat<Real>& iA = row.a.invI;
  const JointMat<Real>& iB = row.b.invI;

  if (row.springAngular.active) {
    const JointQuat<Real> frameA = a.dq * row.a.frame;
    const JointQuat<Real> relative = jointRelative(frameA, b.dq * row.b.frame);
    const JointVec<Real> error =
      Real(2) * (frameA * JointVec<Real>(relative.x, relative.y, relative.z));
    const JointVec<Real> impulse =
      jointSpringImpulse(row.springAngular, iA + iB, b.w - a.w, error);

    a.w -= iA * impulse;
    b.w += iB * impulse;
  }

  if (row.springLinear.active) {
    const JointVec<Real> rA = a.dq * row.a.anchor;
    const JointVec<Real> rB = b.dq * row.b.anchor;
    const JointVec<Real> error = (b.dp - a.dp) + (rB - rA) + row.deltaCenter;
    const JointVec<Real> rate =
      b.v + glm::cross(b.w, rB) - a.v - glm::cross(a.w, rA);
    const JointVec<Real> impulse = jointSpringImpulse(
      row.springLinear,
      jointPointMass(mA, mB, iA, iB, rA, rB),
      rate,
      error
    );

    a.v -= mA * impulse;
    a.w -= iA * glm::cross(rA, impulse);
    b.v += mB * impulse;
    b.w += iB * glm::cross(rB, impulse);
  }
}

template<typename Real>
void storeSpringRows(Joint& joint, const JointRow<Real>& row, Real invH) {
  const double rate = static_cast<double>(invH);
  const glm::dvec3 force = glm::dvec3(row.springLinear.impulse) * rate;
  const glm::dvec3 torque = glm::dvec3(row.springAngular.impulse) * rate;

  joint.impulses.springLinear = force;
  joint.impulses.springAngular = torque;
  joint.report.force = force;
  joint.report.torque = torque;
  joint.report.forceSaturated = row.springLinear.saturated;
  joint.report.torqueSaturated = row.springAngular.saturated;
}

template<typename Real>
void preparePlanarRows(const Joint& joint, Real h, JointRow<Real>& row) {
  const PlanarVelocitySettings& settings = joint.def.planar;
  const double step = static_cast<double>(h);
  JointPlanarRow<Real>& planar = row.planar;

  planar.axisX = row.b.frame * JointVec<Real>(Real(1), Real(0), Real(0));
  planar.axisZ = row.b.frame * JointVec<Real>(Real(0), Real(0), Real(1));
  planar.active = settings.maxForce > 0.0;

  if (planar.active) {
    const Real k = row.a.invMass + row.b.invMass;

    planar.target = JointVec<Real>(settings.targetVelocity);
    planar.mass = k > Real(0) ? Real(1) / k : Real(0);
    planar.cap = static_cast<Real>(settings.maxForce * step);
    planar.impulse = JointPair<Real>(joint.impulses.planar * step);
  }
}

template<typename Real>
JointVec<Real> jointPlanarImpulse(
  const JointPlanarRow<Real>& planar,
  const JointPair<Real>& impulse
) {
  return impulse.x * planar.axisX + impulse.y * planar.axisZ;
}

template<typename Real>
void solvePlanarRows(
  JointRow<Real>& row,
  JointMotion<Real>& a,
  JointMotion<Real>& b
) {
  JointPlanarRow<Real>& planar = row.planar;

  if (!planar.active) {
    return;
  }

  const JointVec<Real> rate = b.v - a.v - planar.target;
  const JointPair<Real> old = planar.impulse;
  const JointPair<Real> unbounded =
    old
    - planar.mass
        * JointPair<Real>(
          glm::dot(rate, planar.axisX),
          glm::dot(rate, planar.axisZ)
        );
  const Real square = glm::dot(unbounded, unbounded);

  planar.saturated = square > planar.cap * planar.cap;
  planar.impulse = planar.saturated
                     ? (planar.cap / std::sqrt(square)) * unbounded
                     : unbounded;

  const JointVec<Real> impulse = jointPlanarImpulse(planar, planar.impulse - old);

  a.v -= row.a.invMass * impulse;
  b.v += row.b.invMass * impulse;
}

template<typename Real>
void storePlanarRows(Joint& joint, const JointRow<Real>& row, Real invH) {
  const double rate = static_cast<double>(invH);
  const JointPlanarRow<Real>& planar = row.planar;

  joint.impulses.planar = glm::dvec2(planar.impulse) * rate;
  joint.report.force =
    glm::dvec3(jointPlanarImpulse(planar, planar.impulse)) * rate;
  joint.report.forceSaturated = planar.active && planar.saturated;
}

template<typename Real>
JointVec<Real> jointWarmAngular(const JointRow<Real>& row) {
  if (row.kind == JointKind::Hinge) {
    return hingeWarmImpulse(row);
  }

  if (row.kind == JointKind::SwingTwist) {
    return swingTwistWarmImpulse(row);
  }

  return row.springAngular.impulse;
}

template<typename Real, typename State>
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
    const JointFrame frameA = joint.def.kind == JointKind::Spring
                                ? jointSpringFrame(joint.def)
                                : joint.def.frameA;
    JointRow<Real> row;
    glm::dvec3 centreA(0.0);
    glm::dvec3 centreB(0.0);

    if (!resolveJointEnd(
          bodies,
          islands,
          states,
          stateOf,
          joint.def.bodyA,
          frameA,
          row.a,
          centreA
        )
        || !resolveJointEnd(
          bodies,
          islands,
          states,
          stateOf,
          joint.def.bodyB,
          joint.def.frameB,
          row.b,
          centreB
        )
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
    row.pointImpulse =
      JointVec<Real>(joint.impulses.point * static_cast<double>(h));
    row.kind = joint.def.kind;

    if (row.kind == JointKind::Hinge) {
      prepareHingeRows(joint, h, row);
    } else if (row.kind == JointKind::SwingTwist) {
      prepareSwingTwistRows(joint, h, row);
    } else if (row.kind == JointKind::Spring) {
      prepareSpringRows(joint, h, row);
    } else if (row.kind == JointKind::PlanarVelocity) {
      preparePlanarRows(joint, h, row);
    }

    rows.push_back(row);
  }
}

template<typename Real, typename State>
void warmStartJointRows(
  std::span<State> states,
  std::span<const JointRow<Real>> rows
) {
  for (const JointRow<Real>& row : rows) {
    JointMotion<Real> a = jointMotion(states, row.a);
    JointMotion<Real> b = jointMotion(states, row.b);

    if (row.kind == JointKind::PlanarVelocity) {
      const JointVec<Real> impulse =
        jointPlanarImpulse(row.planar, row.planar.impulse);

      a.v -= row.a.invMass * impulse;
      b.v += row.b.invMass * impulse;
      storeJointMotion(states, row.a, a);
      storeJointMotion(states, row.b, b);
      continue;
    }

    const JointVec<Real> rA = a.dq * row.a.anchor;
    const JointVec<Real> rB = b.dq * row.b.anchor;
    const JointVec<Real>& impulse = row.kind == JointKind::Spring
                                      ? row.springLinear.impulse
                                      : row.pointImpulse;
    JointVec<Real> turnA = glm::cross(rA, impulse);
    JointVec<Real> turnB = glm::cross(rB, impulse);

    if (row.kind != JointKind::Ball) {
      const JointVec<Real> angular = jointWarmAngular(row);

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

template<typename Real, typename State>
uint32_t solveWatchedLimits(
  std::span<State> states,
  std::span<JointRow<Real>> rows,
  Real invH,
  bool useBias
) {
  uint32_t visited = 0u;

  for (uint32_t pass = 0u; pass != jointLimitPasses; ++pass) {
    for (JointRow<Real>& row : rows) {
      if (!row.watched) {
        continue;
      }

      JointMotion<Real> a = jointMotion(states, row.a);
      JointMotion<Real> b = jointMotion(states, row.b);

      solveJointLimits(row, a, b, invH, useBias);
      storeJointMotion(states, row.a, a);
      storeJointMotion(states, row.b, b);
      ++visited;
    }
  }

  return visited;
}

template<typename Real, typename State>
uint32_t solveJointRows(
  std::span<State> states,
  std::span<JointRow<Real>> rows,
  Real invH,
  bool useBias
) {
  bool watching = false;

  for (JointRow<Real>& row : rows) {
    JointMotion<Real> a = jointMotion(states, row.a);
    JointMotion<Real> b = jointMotion(states, row.b);

    if (row.kind == JointKind::Spring) {
      solveSpringRows(row, a, b);
      storeJointMotion(states, row.a, a);
      storeJointMotion(states, row.b, b);
      continue;
    }

    if (row.kind == JointKind::PlanarVelocity) {
      solvePlanarRows(row, a, b);
      storeJointMotion(states, row.a, a);
      storeJointMotion(states, row.b, b);
      continue;
    }

    if (row.kind == JointKind::Hinge) {
      solveHingeRows(row, a, b, invH, useBias);
    } else if (row.kind == JointKind::SwingTwist) {
      solveSwingTwistRows(row, a, b, invH, useBias);
    }

    if (useBias) {
      row.watched = row.limit && jointLimitWatched(row);
    }

    watching = watching || row.watched;

    const Real mA = row.a.invMass;
    const Real mB = row.b.invMass;
    const JointMat<Real>& iA = row.a.invI;
    const JointMat<Real>& iB = row.b.invI;
    const JointVec<Real> rA = a.dq * row.a.anchor;
    const JointVec<Real> rB = b.dq * row.b.anchor;
    const JointVec<Real> cdot =
      b.v + glm::cross(b.w, rB) - a.v - glm::cross(a.w, rA);
    JointVec<Real> bias(Real(0));
    Real massScale = Real(1);
    Real impulseScale = Real(0);

    if (useBias) {
      const JointVec<Real> separation =
        (b.dp - a.dp) + (rB - rA) + row.deltaCenter;

      bias = row.softness.biasRate * separation;
      massScale = row.softness.massScale;
      impulseScale = row.softness.impulseScale;
    }

    const JointVec<Real> impulse =
      -massScale
        * jointSolve3(jointPointMass(mA, mB, iA, iB, rA, rB), cdot + bias)
      - impulseScale * row.pointImpulse;

    row.pointImpulse += impulse;
    a.v -= mA * impulse;
    a.w -= iA * glm::cross(rA, impulse);
    b.v += mB * impulse;
    b.w += iB * glm::cross(rB, impulse);
    storeJointMotion(states, row.a, a);
    storeJointMotion(states, row.b, b);
  }

  return watching ? solveWatchedLimits(states, rows, invH, useBias) : 0u;
}

template<typename Real>
void advanceJointEnd(JointEnd<Real>& end, Real h) {
  if (end.state != 0u) {
    return;
  }

  end.dp += h * end.v;
  end.dq = jointTurn(end.dq, h * end.w);
}

template<typename Real>
void advanceJointRows(std::span<JointRow<Real>> rows, Real h) {
  for (JointRow<Real>& row : rows) {
    advanceJointEnd(row.a, h);
    advanceJointEnd(row.b, h);
  }
}

template<typename Real>
void storeJointRows(
  JointSet& joints,
  std::span<const JointRow<Real>> rows,
  Real invH
) {
  const std::span<Joint> all = JointRecords::all(joints);

  for (const JointRow<Real>& row : rows) {
    Joint& joint = all[row.joint];
    const glm::dvec3 force =
      glm::dvec3(row.pointImpulse) * static_cast<double>(invH);

    joint.impulses.point = force;
    joint.report.force = force;
    joint.report.torque = glm::dvec3(0.0);

    if (row.kind == JointKind::Hinge) {
      storeHingeRows(joint, row, invH);
    } else if (row.kind == JointKind::SwingTwist) {
      storeSwingTwistRows(joint, row, invH);
    } else if (row.kind == JointKind::Spring) {
      storeSpringRows(joint, row, invH);
    } else if (row.kind == JointKind::PlanarVelocity) {
      storePlanarRows(joint, row, invH);
    }
  }
}

}
