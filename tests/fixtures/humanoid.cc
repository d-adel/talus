#include "humanoid.h"

#include "fieldprimitives.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace fixture {

namespace {

using talus::dynamics::JointFrame;
using talus::dynamics::JointKind;

enum Part : uint32_t {
  pelvis,
  abdomen,
  thorax,
  chest,
  neck,
  head,
  leftClavicle,
  leftUpperArm,
  leftForearm,
  leftHand,
  rightClavicle,
  rightUpperArm,
  rightForearm,
  rightHand,
  leftThigh,
  leftShin,
  leftFoot,
  rightThigh,
  rightShin,
  rightFoot,
  partCount
};

struct Piece {
  double mass = 0.0;
  glm::dvec3 halfExtents{0.0};
  glm::dvec3 moment{0.0};
};

struct Link {
  Part parent = pelvis;
  Part child = pelvis;
  JointFrame parentFrame;
  JointFrame childFrame;
  JointKind kind = JointKind::SwingTwist;
  glm::dvec2 range{0.0};
  glm::dvec2 swing{0.0};
  glm::dvec2 centre{0.0};
  double maxTorque = 0.0;
};

Piece boxPiece(double mass, const glm::dvec3& h, const glm::dvec3& moment) {
  return {mass, h, moment};
}

Piece boxPiece(double mass, const glm::dvec3& h) {
  const glm::dvec3 moment(
    h.y * h.y + h.z * h.z,
    h.x * h.x + h.z * h.z,
    h.x * h.x + h.y * h.y
  );

  return boxPiece(mass, h, mass * moment / 3.0);
}

Piece capsulePiece(double mass, double radius, double length) {
  const double r2 = radius * radius;
  const double cylinder = std::numbers::pi * r2 * length;
  const double hemispheres = 4.0 / 3.0 * std::numbers::pi * r2 * radius;
  const double cylinderMass = mass * cylinder / (cylinder + hemispheres);
  const double hemisphereMass = mass - cylinderMass;
  const double axial =
    cylinderMass * r2 / 2.0 + hemisphereMass * 2.0 * r2 / 5.0;
  const double ends =
    2.0 * r2 / 5.0 + length * length / 4.0 + 3.0 * length * radius / 8.0;
  const double across =
    cylinderMass * (length * length / 12.0 + r2 / 4.0) + hemisphereMass * ends;

  return {
    mass,
    glm::dvec3(radius, 0.5 * length + radius, radius),
    glm::dvec3(across, axial, across)
  };
}

std::array<Piece, partCount> pieces() {
  return {
    boxPiece(8.38, {0.14, 0.06, 0.09}),
    boxPiece(6.12, {0.13, 0.06, 0.09}),
    boxPiece(6.12, {0.14, 0.06, 0.10}),
    boxPiece(10.47, {0.17, 0.115, 0.11}),
    capsulePiece(1.13, 0.045, 0.02),
    capsulePiece(4.08, 0.09, 0.02),
    boxPiece(0.75, {0.08, 0.025, 0.025}, {0.0018, 0.0025, 0.0025}),
    capsulePiece(2.03, 0.045, 0.20),
    capsulePiece(1.22, 0.04, 0.18),
    boxPiece(0.46, {0.02, 0.09, 0.045}),
    boxPiece(0.75, {0.08, 0.025, 0.025}, {0.0018, 0.0025, 0.0025}),
    capsulePiece(2.03, 0.045, 0.20),
    capsulePiece(1.22, 0.04, 0.18),
    boxPiece(0.46, {0.02, 0.09, 0.045}),
    capsulePiece(10.62, 0.07, 0.27),
    capsulePiece(3.25, 0.05, 0.32),
    boxPiece(1.03, {0.045, 0.04, 0.125}),
    capsulePiece(10.62, 0.07, 0.27),
    capsulePiece(3.25, 0.05, 0.32),
    boxPiece(1.03, {0.045, 0.04, 0.125})
  };
}

const std::vector<talus::field::SampledSdf>& fields() {
  static const std::vector<talus::field::SampledSdf> baked = [] {
    std::vector<talus::field::SampledSdf> result;

    for (const Piece& piece : pieces()) {
      const glm::vec3 half(piece.halfExtents);
      const glm::vec3 reach = half + glm::vec3(0.04f);
      const std::vector<talus::field::Primitive> shape{
        box(glm::vec3(0.0f), half)
      };

      result.push_back(
        talus::field::bake(
          shape,
          -reach,
          reach,
          glm::uvec3(glm::round(2.0f * reach / 0.02f)) + glm::uvec3(1u)
        )
      );
    }

    return result;
  }();

  return baked;
}

JointFrame frame(const glm::dvec3& position, const glm::dquat& rotation) {
  return {position, rotation};
}

const glm::dquat identity(1.0, 0.0, 0.0, 0.0);
const glm::dquat xUp(0.70710678, 0.0, 0.0, 0.70710678);
const glm::dquat xDown(0.70710678, 0.0, 0.0, -0.70710678);
const glm::dquat halfTurn(0.0, 0.0, 1.0, 0.0);

Link swingTwist(
  Part parent,
  Part child,
  const JointFrame& parentFrame,
  const JointFrame& childFrame,
  const glm::dvec2& twist,
  const glm::dvec2& swing,
  const glm::dvec2& centre,
  double maxTorque
) {
  return {
    parent,
    child,
    parentFrame,
    childFrame,
    JointKind::SwingTwist,
    twist,
    swing,
    centre,
    maxTorque
  };
}

Link hinge(
  Part parent,
  Part child,
  const JointFrame& parentFrame,
  const JointFrame& childFrame,
  const glm::dvec2& angle,
  double maxTorque
) {
  return {
    parent,
    child,
    parentFrame,
    childFrame,
    JointKind::Hinge,
    angle,
    glm::dvec2(0.0),
    glm::dvec2(0.0),
    maxTorque
  };
}

std::array<Link, humanoidJointNames.size()> links() {
  const glm::dquat
    leftShoulder(0.94703778, -0.19657629, -0.19657629, -0.16073261);
  const glm::dquat
    rightShoulder(0.16073261, -0.19657629, -0.19657629, -0.94703778);
  const glm::dvec2 spineTwist(-0.26, 0.26);
  const glm::dvec2 spineSwing(0.31, 0.2);
  const glm::dvec2 spineCentre(-0.16, 0.0);
  const glm::dvec2 neckTwist(-0.7, 0.7);
  const glm::dvec2 neckSwing(0.48, 0.39);
  const glm::dvec2 neckCentre(0.04, 0.0);
  const glm::dvec2 none(0.0);

  return {
    swingTwist(
      pelvis,
      abdomen,
      frame({0.0, 0.06, 0.0}, xUp),
      frame({0.0, -0.06, 0.0}, xUp),
      spineTwist,
      spineSwing,
      spineCentre,
      300.0
    ),
    swingTwist(
      abdomen,
      thorax,
      frame({0.0, 0.06, 0.0}, xUp),
      frame({0.0, -0.06, 0.0}, xUp),
      spineTwist,
      spineSwing,
      spineCentre,
      300.0
    ),
    swingTwist(
      thorax,
      chest,
      frame({0.0, 0.06, 0.0}, xUp),
      frame({0.0, -0.115, 0.0}, xUp),
      spineTwist,
      spineSwing,
      spineCentre,
      300.0
    ),
    swingTwist(
      chest,
      neck,
      frame({0.0, 0.115, 0.0}, xUp),
      frame({0.0, -0.04, 0.0}, xUp),
      neckTwist,
      neckSwing,
      neckCentre,
      60.0
    ),
    swingTwist(
      neck,
      head,
      frame({0.0, 0.04, 0.0}, xUp),
      frame({0.0, -0.085, 0.0}, xUp),
      neckTwist,
      neckSwing,
      neckCentre,
      60.0
    ),
    swingTwist(
      chest,
      leftClavicle,
      frame({0.02, 0.065, 0.0}, identity),
      frame({-0.08, 0.0, 0.0}, identity),
      {-0.3, 0.3},
      {0.35, 0.35},
      none,
      100.0
    ),
    swingTwist(
      leftClavicle,
      leftUpperArm,
      frame({0.08, 0.0, 0.0}, leftShoulder),
      frame({0.0, 0.145, 0.0}, xDown),
      {-2.3, 1.1},
      {1.85, 1.86},
      none,
      120.0
    ),
    hinge(
      leftUpperArm,
      leftForearm,
      frame({0.0, -0.145, 0.0}, halfTurn),
      frame({0.0, 0.13, 0.0}, halfTurn),
      {0.0, 2.5},
      80.0
    ),
    swingTwist(
      leftForearm,
      leftHand,
      frame({0.0, -0.13, 0.0}, xDown),
      frame({0.0, 0.09, 0.0}, xDown),
      {-1.3, 1.3},
      {1.3, 0.45},
      none,
      25.0
    ),
    swingTwist(
      chest,
      rightClavicle,
      frame({-0.02, 0.065, 0.0}, halfTurn),
      frame({0.08, 0.0, 0.0}, halfTurn),
      {-0.3, 0.3},
      {0.35, 0.35},
      none,
      100.0
    ),
    swingTwist(
      rightClavicle,
      rightUpperArm,
      frame({-0.08, 0.0, 0.0}, rightShoulder),
      frame({0.0, 0.145, 0.0}, xDown),
      {-1.1, 2.3},
      {1.85, 1.86},
      none,
      120.0
    ),
    hinge(
      rightUpperArm,
      rightForearm,
      frame({0.0, -0.145, 0.0}, halfTurn),
      frame({0.0, 0.13, 0.0}, halfTurn),
      {0.0, 2.5},
      80.0
    ),
    swingTwist(
      rightForearm,
      rightHand,
      frame({0.0, -0.13, 0.0}, xDown),
      frame({0.0, 0.09, 0.0}, xDown),
      {-1.3, 1.3},
      {1.3, 0.45},
      none,
      25.0
    ),
    swingTwist(
      pelvis,
      leftThigh,
      frame({0.09, -0.06, 0.0}, xDown),
      frame({0.0, 0.205, 0.0}, xDown),
      {-0.6, 0.6},
      {1.3, 0.65},
      {-0.8, 0.15},
      250.0
    ),
    hinge(
      leftThigh,
      leftShin,
      frame({0.0, -0.205, 0.0}, identity),
      frame({0.0, 0.21, 0.0}, identity),
      {0.0, 2.4},
      250.0
    ),
    swingTwist(
      leftShin,
      leftFoot,
      frame({0.0, -0.21, 0.0}, identity),
      frame({0.0, 0.04, -0.075}, identity),
      {-0.35, 0.87},
      {0.2, 0.35},
      none,
      150.0
    ),
    swingTwist(
      pelvis,
      rightThigh,
      frame({-0.09, -0.06, 0.0}, xDown),
      frame({0.0, 0.205, 0.0}, xDown),
      {-0.6, 0.6},
      {1.3, 0.65},
      {-0.8, -0.15},
      250.0
    ),
    hinge(
      rightThigh,
      rightShin,
      frame({0.0, -0.205, 0.0}, identity),
      frame({0.0, 0.21, 0.0}, identity),
      {0.0, 2.4},
      250.0
    ),
    swingTwist(
      rightShin,
      rightFoot,
      frame({0.0, -0.21, 0.0}, identity),
      frame({0.0, 0.04, -0.075}, identity),
      {-0.35, 0.87},
      {0.2, 0.35},
      none,
      150.0
    )
  };
}

}

Humanoid addHumanoid(
  talus::dynamics::World& world,
  const glm::dvec3& at,
  bool pinPelvis
) {
  const std::array<Piece, partCount> table = pieces();
  const std::array<Link, humanoidJointNames.size()> joints = links();
  std::array<glm::dvec3, partCount> positions{};
  positions[pelvis] = at;

  for (const Link& link : joints) {
    positions[link.child] = positions[link.parent] + link.parentFrame.position
                            - link.childFrame.position;
  }

  Humanoid humanoid;

  for (uint32_t part = 0u; part != partCount; ++part) {
    const Piece& piece = table[part];
    const bool pinned = pinPelvis && part == pelvis;
    const uint32_t index = talus::dynamics::addRigidBox(
      world.bodies,
      fields()[part],
      piece.halfExtents,
      pinned ? 0.0 : piece.mass,
      positions[part],
      glm::dvec3(0.0),
      0.5,
      3u
    );
    talus::dynamics::RigidBody& body = world.bodies.rigids[index];

    if (!pinned) {
      body.moment = piece.moment;
    }

    humanoid.bodies.push_back(body.id);
  }

  for (const Link& link : joints) {
    talus::dynamics::JointDef def;
    def.kind = link.kind;
    def.bodyA = humanoid.bodies[link.parent];
    def.bodyB = humanoid.bodies[link.child];
    def.frameA = link.parentFrame;
    def.frameB = link.childFrame;
    def.drive.maxTorque = link.maxTorque;

    if (link.kind == JointKind::Hinge) {
      def.hinge = {true, link.range.x, link.range.y};
    } else {
      def.cone = {true, link.range.x, link.range.y, link.swing, link.centre};
    }

    const uint32_t joint = talus::dynamics::addJoint(world, def);

    if (joint == 0u) {
      throw std::runtime_error("a humanoid joint was rejected");
    }

    humanoid.joints.push_back(joint);
  }

  return humanoid;
}

void excludeSelfContact(
  talus::dynamics::World& world,
  const Humanoid& humanoid
) {
  for (size_t first = 0u; first != humanoid.bodies.size(); ++first) {
    for (size_t second = first + 1u; second != humanoid.bodies.size();
         ++second) {
      talus::dynamics::JointDef def;
      def.kind = JointKind::Spring;
      def.bodyA = humanoid.bodies[first];
      def.bodyB = humanoid.bodies[second];

      if (world.joints.excludes(def.bodyA, def.bodyB)) {
        continue;
      }

      if (talus::dynamics::addJoint(world, def) == 0u) {
        throw std::runtime_error("a humanoid contact exclusion was rejected");
      }
    }
  }
}

}
