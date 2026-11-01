#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using namespace talus::dynamics;

constexpr double crateHalf = 0.05;
const glm::dvec3 paddleHalf(0.04, 0.05, 0.3);
const glm::dquat upright(1.0, 0.0, 0.0, 0.0);

talus::field::SampledSdf boxField(const glm::dvec3& half, double voxel) {
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

const talus::field::SampledSdf& floorField() {
  static const talus::field::SampledSdf field =
    boxField(glm::dvec3(3.0, 0.5, 3.0), 0.1);

  return field;
}

const talus::field::SampledSdf& crateField() {
  static const talus::field::SampledSdf field =
    boxField(glm::dvec3(crateHalf), 0.02);

  return field;
}

const talus::field::SampledSdf& paddleField() {
  static const talus::field::SampledSdf field = boxField(paddleHalf, 0.02);

  return field;
}

const talus::field::SampledSdf& blockField() {
  static const talus::field::SampledSdf field =
    boxField(glm::dvec3(0.05, 0.025, 0.05), 0.025);

  return field;
}

constexpr double ballRadius = 0.05;

const talus::field::SampledSdf& ballField() {
  static const talus::field::SampledSdf field = [] {
    const double voxel = ballRadius / 5.0;
    const glm::vec3 reach(static_cast<float>(ballRadius + 2.0 * voxel));

    return talus::field::bake(
      std::vector<talus::field::Primitive>{
        fixture::sphere(glm::vec3(0.0f), static_cast<float>(ballRadius))
      },
      -reach,
      reach,
      glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
        + glm::uvec3(1u)
    );
  }();

  return field;
}

const std::vector<talus::PhysicalMaterial>& rubber() {
  static const std::vector<talus::PhysicalMaterial> lanes = [] {
    talus::PhysicalMaterial material;

    material.density = 1000.0f;
    material.youngsModulus = 1.2854547e6f;
    material.poissonsRatio = 0.45f;

    return std::vector<talus::PhysicalMaterial>{material};
  }();

  return lanes;
}

const std::shared_ptr<const talus::PreparedMesh>& blockMesh() {
  static const std::shared_ptr<const talus::PreparedMesh> mesh =
    talus::deformable::buildMesh(
      blockField(),
      rubber(),
      glm::mat4(1.0f),
      std::span<const talus::deformable::SupportVolume>{},
      talus::VolumeElementType::Hex
    );

  return mesh;
}

std::vector<uint32_t> scannedAwake(const World& world) {
  std::vector<uint32_t> awake;

  for (uint32_t index = 0u; index != world.bodies.rigids.size(); ++index) {
    if (!world.bodies.rigids[index].asleep) {
      awake.push_back(index);
    }
  }

  return awake;
}

std::vector<uint32_t> listedAwake(const World& world) {
  const std::span<const uint32_t> listed = awakeRigids(world);

  return {listed.begin(), listed.end()};
}

bool listed(const World& world, uint32_t index) {
  return std::ranges::binary_search(awakeRigids(world), index);
}

uint32_t indexOf(const World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  REQUIRE(ref.has_value());
  REQUIRE(ref->side == Side::Rigid);

  return ref->index;
}

std::vector<uint32_t> idsWhere(const World& world, bool asleep) {
  std::vector<uint32_t> ids;

  for (const RigidBody& body : world.bodies.rigids) {
    if (body.asleep == asleep) {
      ids.push_back(body.id);
    }
  }

  std::ranges::sort(ids);

  return ids;
}

uint32_t wokenSince(const World& world, std::span<const uint32_t> sleeping) {
  return static_cast<uint32_t>(
    std::ranges::count_if(world.bodies.rigids, [&](const RigidBody& body) {
      return !body.asleep && std::ranges::binary_search(sleeping, body.id);
    })
  );
}

bool newSleepersStirred(const World& world, std::span<const uint32_t> awake) {
  return std::ranges::all_of(awake, [&](uint32_t id) {
    const std::optional<BodyRef> ref = findBody(world.bodies, id);

    return !ref || ref->side != Side::Rigid
           || !world.bodies.rigids[ref->index].asleep
           || std::ranges::find(world.bodies.stirred, ref->index)
                != world.bodies.stirred.end();
  });
}

bool allResting(const World& world) {
  return std::ranges::all_of(
           world.bodies.rigids,
           [](const RigidBody& body) { return body.asleep; }
         )
         && std::ranges::all_of(
           world.bodies.lattices,
           [](const LatticeBody& body) { return body.asleep; }
         );
}

uint32_t addBox(
  World& world,
  const talus::field::SampledSdf& field,
  const glm::dvec3& half,
  double mass,
  const glm::dvec3& at
) {
  const uint32_t index =
    addRigidBox(world.bodies, field, half, mass, at, glm::dvec3(0.0), 0.5, 3u);

  return world.bodies.rigids[index].id;
}

uint32_t addCrate(World& world, const glm::dvec3& at) {
  return addBox(world, crateField(), glm::dvec3(crateHalf), 1.5, at);
}

CompoundChild cube(double x, uint32_t key) {
  CompoundChild child;

  child.centre = glm::dvec3(x, 0.0, 0.0);
  child.half = glm::dvec3(crateHalf);
  child.density = 400.0;
  child.key = key;

  return child;
}

uint32_t addBar(World& world, const glm::dvec3& at) {
  const std::array<CompoundChild, 3> children{
    cube(-0.1, 1u),
    cube(0.0, 2u),
    cube(0.1, 3u)
  };
  const uint32_t index = addCompoundBody(
    world.bodies,
    children,
    at,
    upright,
    glm::dvec3(0.0),
    0.5,
    0.0,
    false
  );

  return world.bodies.rigids[index].id;
}

void addFloor(World& world) {
  addStatic(world.bodies, floorField(), 0.5, 0.0, glm::dvec3(0.0, -0.5, 0.0));
}

struct Yard {
  uint32_t post = 0u;
  uint32_t paddle = noBody;
  uint32_t puller = noBody;
  uint32_t towed = noBody;
  uint32_t block = noBody;
  uint32_t lattice = noBody;
  std::vector<uint32_t> springs;
  double sweep = -1.0;
  double tow = 0.0;
};

bool special(const Yard& yard, uint32_t id) {
  return id == yard.paddle || id == yard.puller || id == yard.towed
         || id == yard.block;
}

Yard buildYard(World& world) {
  Yard yard;

  addFloor(world);
  const uint32_t post = addStatic(
    world.bodies,
    crateField(),
    0.5,
    0.0,
    glm::dvec3(2.5, crateHalf, 2.5)
  );

  yard.post = world.bodies.statics[post].id;

  for (uint32_t row = 0u; row != 4u; ++row) {
    for (uint32_t column = 0u; column != 5u; ++column) {
      const glm::dvec3 at(
        0.3 * static_cast<double>(column) - 0.6,
        crateHalf,
        0.3 * static_cast<double>(row) - 0.45
      );

      addCrate(world, at);

      if ((row + column) % 3u == 0u) {
        addCrate(world, at + glm::dvec3(0.0, 2.0 * crateHalf + 0.001, 0.0));
      }
    }
  }

  for (const double x : {-0.5, 0.0, 0.5}) {
    addBar(world, glm::dvec3(x, crateHalf, 0.85));
  }

  const glm::dvec3 anchor(-1.5, 0.62, 0.0);
  uint32_t above = noBody;

  for (uint32_t link = 0u; link != 3u; ++link) {
    const uint32_t id = addCrate(
      world,
      anchor - glm::dvec3(0.0, 0.06 + 0.12 * static_cast<double>(link), 0.0)
    );
    JointDef def;

    def.bodyA = above;
    def.bodyB = id;
    def.frameA.position =
      above == noBody ? anchor : glm::dvec3(0.0, -0.06, 0.0);
    def.frameB.position = glm::dvec3(0.0, 0.06, 0.0);
    REQUIRE(addJoint(world, def) != 0u);
    above = id;
  }

  yard.paddle =
    addBox(world, paddleField(), paddleHalf, 0.0, glm::dvec3(1.2, 0.06, 0.0));
  yard.puller = addBox(
    world,
    crateField(),
    glm::dvec3(crateHalf),
    0.0,
    glm::dvec3(1.5, 0.35, -1.0)
  );
  yard.towed = addCrate(world, glm::dvec3(1.5, crateHalf, -1.0));

  JointDef tow;

  tow.kind = JointKind::Spring;
  tow.bodyA = yard.puller;
  tow.bodyB = yard.towed;
  tow.frameA.position = glm::dvec3(0.0, -0.3, 0.0);
  tow.spring.linearStiffness = 300.0;
  tow.spring.linearDamping = 30.0;
  tow.spring.maxForce = 40.0;
  REQUIRE(addJoint(world, tow) != 0u);

  yard.lattice = world.bodies
                   .lattices[addLattice(
                     world.bodies,
                     blockMesh(),
                     rubber(),
                     glm::dvec3(1.5, 0.03, 1.0),
                     glm::dvec3(0.0),
                     0.5,
                     &blockField()
                   )]
                   .id;
  yard.block = addSwitchable(
    world.bodies,
    blockMesh(),
    rubber(),
    blockField(),
    glm::dvec3(-1.5, 0.03, 1.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  REQUIRE(
    freezeBody(world, addCrate(world, glm::dvec3(0.95, crateHalf, 0.15)))
  );

  return yard;
}

struct Dice {
  std::mt19937_64 engine{20261031u};

  uint32_t below(size_t count) {
    return static_cast<uint32_t>(engine() % count);
  }

  double between(double low, double high) {
    return low
           + (high - low) * static_cast<double>(engine() >> 11u) * 0x1.0p-53;
  }

  glm::dvec3 within(double reach) {
    return glm::dvec3{
      between(-reach, reach),
      between(-reach, reach),
      between(-reach, reach)
    };
  }
};

std::optional<uint32_t> pickBody(
  const World& world,
  const Yard& yard,
  Dice& dice
) {
  for (uint32_t attempt = 0u; attempt != 8u; ++attempt) {
    const uint32_t id =
      world.bodies.rigids[dice.below(world.bodies.rigids.size())].id;

    if (!special(yard, id)) {
      return id;
    }
  }

  return std::nullopt;
}

enum class Edit : uint8_t {
  Pose,
  Velocity,
  Impulse,
  Spin,
  Mass,
  Freeze,
  Thaw,
  Strike,
  AddSpring,
  EditSpring,
  RemoveSpring,
  AddCrate,
  AddBar,
  Split,
  RemoveOne,
  RemoveSome,
  SwitchTier,
  Band,
  Sweep,
  Tow,
  MoveStatic,
  PauseSleep
};

constexpr size_t editKinds = 22u;

constexpr std::array<Edit, 49u> editDeck{
  Edit::Pose,       Edit::Pose,         Edit::Pose,         Edit::Velocity,
  Edit::Velocity,   Edit::Velocity,     Edit::Impulse,      Edit::Impulse,
  Edit::Impulse,    Edit::Spin,         Edit::Spin,         Edit::Spin,
  Edit::Mass,       Edit::Mass,         Edit::Mass,         Edit::Freeze,
  Edit::Freeze,     Edit::Freeze,       Edit::Thaw,         Edit::Thaw,
  Edit::Thaw,       Edit::Strike,       Edit::Strike,       Edit::AddSpring,
  Edit::AddSpring,  Edit::AddSpring,    Edit::EditSpring,   Edit::EditSpring,
  Edit::EditSpring, Edit::RemoveSpring, Edit::RemoveSpring, Edit::RemoveSpring,
  Edit::AddCrate,   Edit::AddCrate,     Edit::AddCrate,     Edit::AddBar,
  Edit::AddBar,     Edit::Split,        Edit::Split,        Edit::RemoveOne,
  Edit::RemoveOne,  Edit::RemoveSome,   Edit::RemoveSome,   Edit::SwitchTier,
  Edit::SwitchTier, Edit::Band,         Edit::Band,         Edit::Sweep,
  Edit::Tow
};

constexpr size_t springLimit = 3u;
constexpr uint32_t moveStaticFrame = 1000u;
constexpr uint32_t pauseSleepFrame = 500u;
constexpr double sweepSpeed = 2.0;
constexpr double strikeSpeed = 2.0;

std::optional<Edit> wakeGroup(Edit edit) {
  switch (edit) {
    case Edit::Thaw:
      return Edit::Freeze;
    case Edit::EditSpring:
    case Edit::RemoveSpring:
      return Edit::AddSpring;
    case Edit::RemoveSome:
      return Edit::RemoveOne;
    case Edit::Strike:
    case Edit::AddCrate:
    case Edit::AddBar:
    case Edit::SwitchTier:
    case Edit::Band:
    case Edit::Sweep:
    case Edit::PauseSleep:
      return std::nullopt;
    default:
      return edit;
  }
}

JointFrame offsetBetween(const World& world, uint32_t a, uint32_t b) {
  const RigidBody& first = world.bodies.rigids[indexOf(world, a)];
  const RigidBody& second = world.bodies.rigids[indexOf(world, b)];
  JointFrame frame;

  frame.position =
    glm::conjugate(first.rotation) * (second.position - first.position);
  frame.rotation = glm::conjugate(first.rotation) * second.rotation;

  return frame;
}

std::vector<uint32_t> frozenIds(const World& world) {
  std::vector<uint32_t> frozen;

  for (const RigidBody& body : world.bodies.rigids) {
    if (isFrozen(body)) {
      frozen.push_back(body.id);
    }
  }

  return frozen;
}

bool sleeps(const World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  return ref && ref->side == Side::Rigid
         && world.bodies.rigids[ref->index].asleep;
}

void forgetRemovedSprings(const World& world, Yard& yard) {
  std::erase_if(yard.springs, [&](uint32_t joint) {
    return !jointDef(world, joint).has_value();
  });
}

std::optional<uint32_t> pickSpring(const World& world, Yard& yard, Dice& dice) {
  forgetRemovedSprings(world, yard);

  std::vector<uint32_t> resting;

  for (const uint32_t joint : yard.springs) {
    const JointDef def = *jointDef(world, joint);

    if (sleeps(world, def.bodyA) && sleeps(world, def.bodyB)) {
      resting.push_back(joint);
    }
  }

  const std::vector<uint32_t>& from = resting.empty() ? yard.springs : resting;

  if (from.empty()) {
    return std::nullopt;
  }

  return from[dice.below(from.size())];
}

bool applyEdit(World& world, Yard& yard, Dice& dice, Edit edit) {
  const auto spot = [&](double height) {
    const double x = dice.between(-0.7, 0.7);

    return glm::dvec3(x, height, dice.between(-0.5, 0.5));
  };
  const std::optional<uint32_t> id = pickBody(world, yard, dice);

  switch (edit) {
    case Edit::Pose: {
      const glm::dvec3 position = spot(0.3);
      const glm::dquat turn =
        glm::angleAxis(dice.between(0.0, 3.0), glm::dvec3(0.0, 1.0, 0.0));

      return id && setRigidPose(world, *id, position, turn);
    }
    case Edit::Velocity:
      return dice.below(5u) == 0u
               ? setVelocity(world, yard.lattice, dice.within(0.5))
               : id && setVelocity(world, *id, dice.within(0.5));
    case Edit::Impulse: {
      const glm::dvec3 impulse = dice.within(1.0);
      const glm::dvec3 offset = dice.within(crateHalf);

      return id
             && applyImpulse(
               world,
               *id,
               impulse,
               world.bodies.rigids[indexOf(world, *id)].position + offset
             );
    }
    case Edit::Spin:
      return id && setAngularVelocity(world, *id, dice.within(3.0));
    case Edit::Mass:
      return id && setMass(world, *id, dice.between(0.8, 3.0));
    case Edit::Freeze:
      return id && freezeBody(world, *id);
    case Edit::Thaw: {
      const std::vector<uint32_t> frozen = frozenIds(world);

      return !frozen.empty()
             && thawBody(world, frozen[dice.below(frozen.size())]);
    }
    case Edit::Strike: {
      const std::vector<uint32_t> frozen = frozenIds(world);

      if (!id || frozen.empty()) {
        return false;
      }

      const uint32_t target = frozen[dice.below(frozen.size())];

      if (target == *id) {
        return false;
      }

      const glm::dvec3 at =
        world.bodies.rigids[indexOf(world, target)].position;

      return setRigidPose(world, *id, at + glm::dvec3(0.3, 0.0, 0.0), upright)
             && setVelocity(world, *id, glm::dvec3(-strikeSpeed, 0.0, 0.0));
    }
    case Edit::AddSpring: {
      const std::optional<uint32_t> other = pickBody(world, yard, dice);

      forgetRemovedSprings(world, yard);

      if (!id || !other || *id == *other
          || yard.springs.size() >= springLimit) {
        return false;
      }

      JointDef def;

      def.kind = JointKind::Spring;
      def.bodyA = *id;
      def.bodyB = *other;
      def.spring.target = offsetBetween(world, *id, *other);
      def.spring.linearStiffness = 100.0;
      def.spring.linearDamping = 10.0;
      def.spring.maxForce = 20.0;

      const uint32_t joint = addJoint(world, def);

      if (joint != 0u) {
        yard.springs.push_back(joint);
      }

      return joint != 0u;
    }
    case Edit::EditSpring: {
      const std::optional<uint32_t> joint = pickSpring(world, yard, dice);
      JointFrame target;

      target.position = dice.within(0.3);

      return joint && setSpringTarget(world, *joint, target);
    }
    case Edit::RemoveSpring: {
      const std::optional<uint32_t> joint = pickSpring(world, yard, dice);

      if (!joint) {
        return false;
      }

      std::erase(yard.springs, *joint);

      return removeJoint(world, *joint);
    }
    case Edit::AddCrate:
      if (world.bodies.rigids.size() >= 60u) {
        return false;
      }

      addCrate(world, spot(0.4));

      return true;
    case Edit::AddBar:
      if (world.bodies.rigids.size() >= 60u) {
        return false;
      }

      addBar(world, spot(0.4));

      return true;
    case Edit::Split: {
      std::vector<std::vector<uint32_t>> groups(2u);
      std::vector<uint32_t> bars;

      for (const RigidBody& body : world.bodies.rigids) {
        if (body.shape == RigidShape::Compound && body.mass > 0.0
            && body.compound->children.size() >= 2u) {
          bars.push_back(body.id);
        }
      }

      if (bars.empty()) {
        return false;
      }

      const uint32_t bar = bars[dice.below(bars.size())];

      for (const CompoundChild& child :
           world.bodies.rigids[indexOf(world, bar)].compound->children) {
        groups[groups[0].empty() ? 0u : 1u].push_back(child.key);
      }

      return !splitCompound(world, bar, groups).empty();
    }
    case Edit::RemoveOne:
      return id && removeBody(world, *id);
    case Edit::RemoveSome: {
      std::vector<uint32_t> ids;

      for (uint32_t count = 0u; count != 2u; ++count) {
        if (const std::optional<uint32_t> gone = pickBody(world, yard, dice)) {
          ids.push_back(*gone);
        }
      }

      return removeBodies(world, ids) != 0u;
    }
    case Edit::SwitchTier: {
      const std::optional<BodyRef> ref = findBody(world.bodies, yard.block);

      if (!ref) {
        return false;
      }

      if (ref->side == Side::Rigid) {
        return promote(world, yard.block);
      }

      demote(world, yard.block);

      return findBody(world.bodies, yard.block)->side == Side::Rigid;
    }
    case Edit::Band:
      world.parameters.speculativeDistance =
        world.parameters.speculativeDistance == 0.02 ? 0.03 : 0.02;

      return true;
    case Edit::Sweep:
      yard.sweep = -yard.sweep;

      return setRigidPose(
               world,
               yard.paddle,
               glm::dvec3(-1.2 * yard.sweep, 0.06, dice.between(-0.3, 0.3)),
               upright
             )
             && setVelocity(
               world,
               yard.paddle,
               glm::dvec3(sweepSpeed * yard.sweep, 0.0, 0.0)
             );
    case Edit::Tow:
      yard.tow = yard.tow != 0.0 ? 0.0 : (dice.below(2u) == 0u ? 0.3 : -0.3);

      return setVelocity(world, yard.puller, glm::dvec3(0.0, 0.0, yard.tow));
    case Edit::MoveStatic:
      return setStaticPose(
        world,
        yard.post,
        glm::dvec3(2.5, crateHalf, 2.5),
        upright
      );
    case Edit::PauseSleep:
      world.parameters.sleep = false;

      return true;
  }

  return false;
}

void keepInYard(World& world, Yard& yard) {
  const RigidBody& paddle = world.bodies.rigids[indexOf(world, yard.paddle)];

  if (paddle.velocity.x * paddle.position.x > 0.0
      && std::abs(paddle.position.x) > 1.0) {
    REQUIRE(setVelocity(world, yard.paddle, glm::dvec3(0.0)));
  }

  const double drift =
    world.bodies.rigids[indexOf(world, yard.puller)].position.z + 1.0;

  if (yard.tow * drift > 0.0 && std::abs(drift) > 0.8) {
    yard.tow = 0.0;
    REQUIRE(setVelocity(world, yard.puller, glm::dvec3(0.0)));
  }

  std::vector<uint32_t> fallen;

  for (const RigidBody& body : world.bodies.rigids) {
    if (body.position.y < -1.0) {
      fallen.push_back(body.id);
    }
  }

  if (!fallen.empty()) {
    REQUIRE(removeBodies(world, fallen) == fallen.size());
  }
}

std::vector<Edit> rollEdits(Dice& dice, uint32_t frame) {
  std::vector<Edit> edits;

  if (dice.below(10u) == 0u) {
    edits.resize(1u + dice.below(2u));
    std::ranges::generate(edits, [&] {
      return editDeck[dice.below(editDeck.size())];
    });
  }

  if (frame == moveStaticFrame) {
    edits.push_back(Edit::MoveStatic);
  }

  if (frame == pauseSleepFrame) {
    edits.push_back(Edit::PauseSleep);
  }

  return edits;
}

void editYard(World& world, Yard& yard, Dice& dice, uint32_t frame) {
  world.parameters.sleep = true;
  keepInYard(world, yard);

  for (const Edit edit : rollEdits(dice, frame)) {
    static_cast<void>(applyEdit(world, yard, dice, edit));
  }
}

bool walksAwakeOnly(const World& world) {
  const AwakeList& awake = world.awake;

  return world.parameters.sleep && awake.layout == world.bodies.layout
         && awake.latticeCount == world.bodies.lattices.size()
         && awake.rigidCount <= world.bodies.rigids.size()
         && awake.band
              == world.parameters.speculativeDistance + world.parameters.margin;
}

void forceFullStep(World& world) {
  world.bodies.layout += 1u;
}

struct Lanes {
  uint32_t workers;
  uint32_t width;
};

std::vector<Lanes> twinLanes() {
  std::vector<Lanes> lanes{{1u, 0u}, {8u, 4u}};

  lanes.push_back({4u, hasAvx2() ? 8u : 1u});

  return lanes;
}

StepReport stepCounting(World& world, uint32_t& sparse) {
  const bool listed = walksAwakeOnly(world);
  const uint64_t layout = world.bodies.layout;
  const StepReport report = step(world);

  sparse += listed && world.bodies.layout == layout ? 1u : 0u;

  return report;
}

bool sameBits(double a, double b) {
  return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

bool sameBits(const glm::dvec3& a, const glm::dvec3& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z);
}

bool sameBits(const glm::dquat& a, const glm::dquat& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z)
         && sameBits(a.w, b.w);
}

bool sameNodes(std::span<const glm::dvec3> a, std::span<const glm::dvec3> b) {
  return std::ranges::equal(a, b, [](const glm::dvec3& x, const glm::dvec3& y) {
    return sameBits(x, y);
  });
}

void requireSameReport(const StepReport& walked, const StepReport& full) {
  REQUIRE(walked.nonFinite == full.nonFinite);
  REQUIRE(walked.thawed == full.thawed);
  REQUIRE(walked.contacts == full.contacts);
  REQUIRE(walked.activeContacts == full.activeContacts);
  REQUIRE(walked.softstepIslands == full.softstepIslands);
  REQUIRE(walked.strainstepIslands == full.strainstepIslands);
  REQUIRE(walked.hits.size() == full.hits.size());
}

void requireSameContacts(const World& walked, const World& full) {
  REQUIRE(walked.manifolds.size() == full.manifolds.size());

  for (uint32_t slot = 0u; slot != walked.manifolds.size(); ++slot) {
    CAPTURE(slot);
    REQUIRE(walked.manifolds.live(slot) == full.manifolds.live(slot));

    if (!walked.manifolds.live(slot)) {
      continue;
    }

    const CachedBodyPair& a = walked.manifolds.at(slot);
    const CachedBodyPair& b = full.manifolds.at(slot);

    REQUIRE(a.pair == b.pair);
    REQUIRE(a.colour == b.colour);
    REQUIRE(a.points.size() == b.points.size());
  }
}

void requireSameBodies(const World& walked, const World& full) {
  requireSameContacts(walked, full);
  REQUIRE(walked.bodies.rigids.size() == full.bodies.rigids.size());
  REQUIRE(walked.bodies.lattices.size() == full.bodies.lattices.size());

  for (size_t index = 0u; index != walked.bodies.rigids.size(); ++index) {
    const RigidBody& a = walked.bodies.rigids[index];
    const RigidBody& b = full.bodies.rigids[index];

    CAPTURE(index, a.id);
    REQUIRE(a.id == b.id);
    REQUIRE(a.asleep == b.asleep);
    REQUIRE(sameBits(a.sleepTime, b.sleepTime));
    REQUIRE(sameBits(a.mass, b.mass));
    REQUIRE(sameBits(a.position, b.position));
    REQUIRE(sameBits(a.rotation, b.rotation));
    REQUIRE(sameBits(a.velocity, b.velocity));
    REQUIRE(sameBits(a.angularVelocity, b.angularVelocity));
  }

  for (size_t index = 0u; index != walked.bodies.lattices.size(); ++index) {
    const LatticeBody& a = walked.bodies.lattices[index];
    const LatticeBody& b = full.bodies.lattices[index];

    CAPTURE(index, a.id);
    REQUIRE(a.id == b.id);
    REQUIRE(a.asleep == b.asleep);
    REQUIRE(sameNodes(a.position, b.position));
    REQUIRE(sameNodes(a.velocity, b.velocity));
  }
}

uint32_t addAnchoredBar(
  World& world,
  const glm::dvec3& at,
  const glm::dquat& turn
) {
  const std::array<CompoundChild, 3> children{
    cube(-0.1, 1u),
    cube(0.0, 2u),
    cube(0.1, 3u)
  };
  const uint32_t index = addCompoundBody(
    world.bodies,
    children,
    at,
    turn,
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  );

  return world.bodies.rigids[index].id;
}

RigidBody anchoredBarShape() {
  BodySet holder;
  const std::array<CompoundChild, 3> children{
    cube(-0.1, 1u),
    cube(0.0, 2u),
    cube(0.1, 3u)
  };

  return holder.rigids[addCompoundBody(
    holder,
    children,
    glm::dvec3(0.0),
    upright,
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  )];
}

uint32_t addBlock(World& world, const glm::dvec3& at) {
  return world.bodies
    .lattices[addLattice(
      world.bodies,
      blockMesh(),
      rubber(),
      at,
      glm::dvec3(0.0),
      0.5,
      &blockField()
    )]
    .id;
}

struct Bench {
  uint32_t lattice = noBody;
  uint32_t pressed = noBody;
  uint32_t toucher = noBody;
  uint32_t post = noBody;
  uint32_t shot = noBody;
  uint32_t row = noBody;
  uint32_t shelf = noBody;
};

constexpr double wallAt = -10.0;
constexpr double postAt = 10.0;
constexpr double shotSpeed = 100.0;
constexpr double shotRange = 1.5;

Bench buildBench(World& world) {
  Bench bench;

  addFloor(world);
  bench.lattice = addBlock(world, glm::dvec3(0.0, 0.03, 0.0));
  addCrate(world, glm::dvec3(0.0, 0.056 + crateHalf, 0.0));

  for (uint32_t column = 0u; column != 5u; ++column) {
    const uint32_t id = addCrate(
      world,
      glm::dvec3(0.3 * static_cast<double>(column) - 0.6, crateHalf, 0.6)
    );

    bench.row = column == 1u ? id : bench.row;
  }

  const uint32_t held = addCrate(world, glm::dvec3(0.6, crateHalf, -0.6));

  REQUIRE(freezeBody(world, held));
  addCrate(world, glm::dvec3(0.6, 3.0 * crateHalf + 0.001, -0.6));
  bench.pressed = addCrate(world, glm::dvec3(-0.6, crateHalf, -0.6));
  bench.toucher =
    addCrate(world, glm::dvec3(-0.6, 3.0 * crateHalf + 0.001, -0.6));
  world.thawOnContact.push_back(bench.toucher);
  addAnchoredBar(
    world,
    glm::dvec3(wallAt, crateHalf, 0.0),
    glm::angleAxis(0.5 * std::numbers::pi, glm::dvec3(0.0, 1.0, 0.0))
  );
  bench.post = addBox(
    world,
    crateField(),
    glm::dvec3(crateHalf),
    0.0,
    glm::dvec3(postAt, crateHalf, 0.0)
  );
  bench.shot = world.bodies
                 .rigids[addRigidSphere(
                   world.bodies,
                   ballField(),
                   ballRadius,
                   4.1,
                   glm::dvec3(-0.9, ballRadius, -1.2),
                   glm::dvec3(0.0),
                   0.5,
                   0.0
                 )]
                 .id;
  bench.shelf = addAnchoredBar(world, glm::dvec3(0.0, 0.25, -0.6), upright);
  addCrate(world, glm::dvec3(0.1, 0.3 + crateHalf + 0.001, -0.6));

  return bench;
}

void editBench(World& world, Bench& bench, uint32_t frame) {
  const auto shoot = [&](double at, double toward) {
    REQUIRE(setRigidPose(
      world,
      bench.shot,
      glm::dvec3(at - shotRange * toward, crateHalf, 0.0),
      upright
    ));
    REQUIRE(
      setVelocity(world, bench.shot, glm::dvec3(shotSpeed * toward, 0.0, 0.0))
    );
  };

  switch (frame) {
    case 120u:
      REQUIRE(freezeBody(world, bench.pressed));
      break;
    case 150u:
      shoot(wallAt, 1.0);
      break;
    case 200u:
      REQUIRE(replaceRigidShape(world, bench.post, anchoredBarShape()));
      break;
    case 330u:
      shoot(postAt, -1.0);
      break;
    case 400u:
      REQUIRE(setVelocity(world, bench.lattice, glm::dvec3(0.0, 0.5, 0.0)));
      break;
    case 480u:
      REQUIRE(removeBody(world, bench.lattice));
      bench.lattice = addBlock(world, glm::dvec3(0.0, 0.03, -1.2));
      break;
    case 540u:
      addCrate(world, glm::dvec3(-0.3, 0.5, 0.6));
      break;
    case 600u:
      REQUIRE(removeBody(world, bench.row));
      break;
    case 640u:
      shoot(wallAt, 1.0);
      break;
    case 660u:
      REQUIRE(
        removeCompoundChildren(world, bench.shelf, std::array<uint32_t, 1>{3u})
      );
      break;
    default:
      break;
  }
}

void requireTwinSteps(World& walked, World& full, uint32_t frames) {
  for (uint32_t frame = 0u; frame != frames; ++frame) {
    CAPTURE(frame);
    forceFullStep(full);

    const StepReport fromList = step(walked);
    const StepReport fromAll = step(full);

    requireSameReport(fromList, fromAll);
    requireSameBodies(walked, full);
  }
}

}

TEST_CASE(
  "the awake list matches a full scan after every kind of edit",
  "[dynamics][awake]"
) {
  World world;
  Yard yard = buildYard(world);
  Dice dice;
  std::array<uint32_t, editKinds> done{};
  std::array<uint32_t, editKinds> woke{};
  uint32_t stepWakes = 0u;
  uint32_t thawed = 0u;
  uint32_t sleepers = 0u;
  uint32_t bodies = 0u;

  for (uint32_t frame = 0u; frame != 2000u; ++frame) {
    CAPTURE(frame);
    world.parameters.sleep = true;
    keepInYard(world, yard);

    for (const Edit edit : rollEdits(dice, frame)) {
      const size_t kind = static_cast<size_t>(edit);
      const std::vector<uint32_t> sleeping = idsWhere(world, true);

      done[kind] += applyEdit(world, yard, dice, edit) ? 1u : 0u;

      if (const std::optional<Edit> group = wakeGroup(edit)) {
        woke[static_cast<size_t>(*group)] += wokenSince(world, sleeping);
      }
    }

    const std::vector<uint32_t> sleeping = idsWhere(world, true);
    const std::vector<uint32_t> awake = idsWhere(world, false);
    const StepReport report = step(world);

    REQUIRE_FALSE(report.nonFinite);
    REQUIRE(listedAwake(world) == scannedAwake(world));
    REQUIRE(newSleepersStirred(world, awake));
    stepWakes += wokenSince(world, sleeping);
    thawed += report.thawed;
    sleepers += static_cast<uint32_t>(sleeping.size());
    bodies += static_cast<uint32_t>(sleeping.size() + awake.size());
  }

  for (size_t kind = 0u; kind != editKinds; ++kind) {
    const bool leads =
      wakeGroup(static_cast<Edit>(kind)) == static_cast<Edit>(kind);

    CAPTURE(kind, done[kind], woke[kind]);
    CHECK(done[kind] > 0u);
    CHECK((!leads || woke[kind] > 0u));
  }

  CAPTURE(stepWakes, thawed, sleepers, bodies);
  CHECK(stepWakes > 0u);
  CHECK(thawed > 0u);
  CHECK(3u * sleepers > bodies);
}

TEST_CASE(
  "a removal or tier switch makes the next step a full step",
  "[dynamics][awake]"
) {
  World world;

  world.parameters.switchFrames = 1000u;
  addFloor(world);

  const uint32_t block = addSwitchable(
    world.bodies,
    blockMesh(),
    rubber(),
    blockField(),
    glm::dvec3(-0.6, 0.03, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  std::vector<uint32_t> crates;

  for (uint32_t index = 0u; index != 5u; ++index) {
    crates.push_back(addCrate(
      world,
      glm::dvec3(0.3 * static_cast<double>(index) - 0.3, crateHalf, 0.0)
    ));
  }

  for (uint32_t frame = 0u; frame != 600u && !allResting(world); ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);
  }

  REQUIRE(allResting(world));
  REQUIRE_FALSE(step(world).nonFinite);
  REQUIRE(awakeRigids(world).empty());

  const auto wakeBehindTheList = [&](uint32_t id) {
    RigidBody& body = world.bodies.rigids[indexOf(world, id)];

    body.asleep = false;
    body.sleepTime = 0.0;
  };
  const auto wakeAndList = [&](uint32_t id) {
    REQUIRE(setVelocity(world, id, glm::dvec3(0.0)));
    REQUIRE_FALSE(step(world).nonFinite);
    REQUIRE(listedAwake(world) == std::vector<uint32_t>{indexOf(world, id)});

    return indexOf(world, id);
  };
  const uint32_t woken = crates[3];
  const uint32_t shifted = crates[4];

  SECTION("an ordinary step keeps the last step's list") {
    wakeBehindTheList(woken);
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(awakeRigids(world).empty());
    CHECK(scannedAwake(world) == std::vector<uint32_t>{indexOf(world, woken)});
  }

  SECTION("a body removed and another added") {
    const uint32_t stale = wakeAndList(woken);

    REQUIRE(removeBody(world, crates[1]));
    addCrate(world, glm::dvec3(1.5, crateHalf, 1.0));
    REQUIRE(world.bodies.rigids[stale].id == shifted);
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
    CHECK(listed(world, indexOf(world, woken)));
    CHECK(world.bodies.rigids[stale].asleep);
    CHECK_FALSE(listed(world, stale));
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
  }

  SECTION("a promotion") {
    const uint32_t stale = wakeAndList(woken);

    REQUIRE(promote(world, block));
    REQUIRE(world.bodies.rigids[stale].id == shifted);
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
    CHECK(listed(world, indexOf(world, woken)));
    CHECK(world.bodies.rigids[stale].asleep);
    CHECK_FALSE(listed(world, stale));
  }

  SECTION("a demotion") {
    REQUIRE(promote(world, block));
    REQUIRE_FALSE(step(world).nonFinite);
    REQUIRE_FALSE(step(world).nonFinite);
    wakeBehindTheList(woken);
    demote(world, block);
    REQUIRE(findBody(world.bodies, block)->side == Side::Rigid);
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
    CHECK(listed(world, indexOf(world, woken)));
  }

  SECTION("a lattice removed and another added") {
    REQUIRE(promote(world, block));
    REQUIRE_FALSE(step(world).nonFinite);
    REQUIRE_FALSE(step(world).nonFinite);
    wakeBehindTheList(woken);
    REQUIRE(removeBody(world, block));
    addBlock(world, glm::dvec3(-0.6, 0.03, 0.6));
    REQUIRE(world.bodies.lattices.size() == 1u);
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
    CHECK(listed(world, indexOf(world, woken)));
  }

  SECTION("a contact band change") {
    wakeBehindTheList(woken);
    world.parameters.speculativeDistance += 0.01;
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
    CHECK(listed(world, indexOf(world, woken)));
  }

  SECTION("sleeping switched off") {
    wakeBehindTheList(woken);
    world.parameters.sleep = false;
    REQUIRE_FALSE(step(world).nonFinite);
    CHECK(listedAwake(world) == scannedAwake(world));
    CHECK(listed(world, indexOf(world, woken)));
  }
}

TEST_CASE(
  "a step over the awake list ends bit for bit where a full step ends, "
  "through every kind of edit",
  "[dynamics][awake]"
) {
  for (const Lanes& lanes : twinLanes()) {
    CAPTURE(lanes.workers, lanes.width);
    World walked;
    World full;
    walked.workerCount = full.workerCount = lanes.workers;
    walked.simdWidth = full.simdWidth = lanes.width;
    Yard walkedYard = buildYard(walked);
    Yard fullYard = buildYard(full);
    Dice walkedDice;
    Dice fullDice;
    uint32_t sparse = 0u;
    constexpr uint32_t frames = 2000u;

    for (uint32_t frame = 0u; frame != frames; ++frame) {
      CAPTURE(frame);
      editYard(walked, walkedYard, walkedDice, frame);
      editYard(full, fullYard, fullDice, frame);
      forceFullStep(full);

      const StepReport fromList = stepCounting(walked, sparse);
      const StepReport fromAll = step(full);

      requireSameReport(fromList, fromAll);
      requireSameBodies(walked, full);
    }

    CAPTURE(sparse);
    REQUIRE(2u * sparse > frames);
  }
}

TEST_CASE(
  "a step over the awake list ends bit for bit where a full step ends, "
  "around a lattice, frozen bodies, a body that thaws on contact and "
  "zero-mass compounds",
  "[dynamics][awake]"
) {
  for (const Lanes& lanes : twinLanes()) {
    CAPTURE(lanes.workers, lanes.width);
    World walked;
    World full;
    walked.workerCount = full.workerCount = lanes.workers;
    walked.simdWidth = full.simdWidth = lanes.width;
    Bench walkedBench = buildBench(walked);
    Bench fullBench = buildBench(full);
    uint32_t sparse = 0u;
    uint32_t thawed = 0u;
    constexpr uint32_t frames = 720u;

    for (uint32_t frame = 0u; frame != frames; ++frame) {
      CAPTURE(frame);
      editBench(walked, walkedBench, frame);
      editBench(full, fullBench, frame);
      forceFullStep(full);

      const StepReport fromList = stepCounting(walked, sparse);
      const StepReport fromAll = step(full);

      REQUIRE_FALSE(fromList.nonFinite);
      requireSameReport(fromList, fromAll);
      requireSameBodies(walked, full);
      thawed += fromList.thawed;
    }

    CAPTURE(sparse, thawed);
    REQUIRE(2u * sparse > frames);
    CHECK(thawed >= 1u);
    CHECK_FALSE(
      isFrozen(walked.bodies.rigids[indexOf(walked, walkedBench.pressed)])
    );
  }
}

TEST_CASE(
  "a body with a non-finite velocity is still reported after the step it "
  "would have slept",
  "[dynamics][awake]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t lost = addCrate(world, glm::dvec3(0.0, 1.0, 0.0));

  addCrate(world, glm::dvec3(1.0, 1.0, 0.0));

  for (uint32_t frame = 0u; frame != 600u && !allResting(world); ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);
  }

  REQUIRE(allResting(world));

  const double nan = std::numeric_limits<double>::quiet_NaN();

  SECTION("in its velocity") {
    REQUIRE(setVelocity(world, lost, glm::dvec3(nan, 0.0, 0.0)));
  }

  SECTION("in its position") {
    REQUIRE(setRigidPose(world, lost, glm::dvec3(nan, 1.0, 0.0), upright));
  }

  const uint32_t sleepSteps = static_cast<uint32_t>(
    std::ceil(world.parameters.timeToSleep / world.parameters.dt)
  );

  for (uint32_t frame = 0u; frame != 3u * sleepSteps; ++frame) {
    CAPTURE(frame);
    REQUIRE(step(world).nonFinite);
  }
}

TEST_CASE(
  "a frozen body struck only through sleepers stays frozen, and one struck "
  "by an awake body thaws",
  "[dynamics][awake]"
) {
  World world;

  addFloor(world);

  const uint32_t held = addCrate(world, glm::dvec3(-0.6, crateHalf, 0.0));

  addCrate(world, glm::dvec3(-0.6, 3.0 * crateHalf + 0.001, 0.0));

  const uint32_t struck = addCrate(world, glm::dvec3(0.6, crateHalf, 0.0));
  const uint32_t thrown = addCrate(world, glm::dvec3(1.5, crateHalf, 0.6));

  for (uint32_t frame = 0u; frame != 600u && !allResting(world); ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);
  }

  REQUIRE(allResting(world));
  REQUIRE(freezeBody(world, held));
  REQUIRE(freezeBody(world, struck));

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(step(world).thawed == 0u);
  }

  REQUIRE(walksAwakeOnly(world));
  REQUIRE(isFrozen(world.bodies.rigids[indexOf(world, held)]));
  REQUIRE(setRigidPose(
    world,
    thrown,
    world.bodies.rigids[indexOf(world, struck)].position
      + glm::dvec3(0.3, 0.002, 0.0),
    upright
  ));
  REQUIRE(setVelocity(world, thrown, glm::dvec3(-strikeSpeed, 0.0, 0.0)));

  uint32_t thawed = 0u;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    thawed += step(world).thawed;
  }

  CHECK(thawed == 1u);
  CHECK_FALSE(isFrozen(world.bodies.rigids[indexOf(world, struck)]));
  CHECK(isFrozen(world.bodies.rigids[indexOf(world, held)]));
}

TEST_CASE(
  "a frozen body touched only by a sleeping body that thaws on contact thaws, "
  "as a full step thaws it",
  "[dynamics][awake]"
) {
  World walked;
  World full;
  uint32_t pressed = noBody;
  uint32_t toucher = noBody;

  for (World* world : {&walked, &full}) {
    addFloor(*world);
    pressed = addCrate(*world, glm::dvec3(0.0, crateHalf, 0.0));
    toucher = addCrate(*world, glm::dvec3(0.0, 3.0 * crateHalf + 0.001, 0.0));
    addCrate(*world, glm::dvec3(0.6, crateHalf, 0.0));
    REQUIRE(freezeBody(*world, pressed));
  }

  requireTwinSteps(walked, full, 90u);
  REQUIRE(allResting(walked));
  REQUIRE(isFrozen(walked.bodies.rigids[indexOf(walked, pressed)]));
  walked.thawOnContact.push_back(toucher);
  full.thawOnContact.push_back(toucher);
  REQUIRE(walksAwakeOnly(walked));
  forceFullStep(full);

  const StepReport fromList = step(walked);
  const StepReport fromAll = step(full);

  CHECK(fromList.thawed == 1u);
  requireSameReport(fromList, fromAll);
  requireSameBodies(walked, full);
  requireTwinSteps(walked, full, 60u);
  CHECK_FALSE(isFrozen(walked.bodies.rigids[indexOf(walked, pressed)]));
}

namespace {

struct ThrowOnce {
  bool armed = false;
  uint32_t calls = 0u;
};

void* throwingEnqueue(
  TaskCallback* task,
  void* taskContext,
  void* userContext,
  const char*
) {
  ThrowOnce& host = *static_cast<ThrowOnce*>(userContext);

  if (host.armed) {
    host.armed = false;
    ++host.calls;

    throw std::runtime_error("host task failure");
  }

  task(taskContext);

  return nullptr;
}

void finishInline(void*, void*) {}

}

TEST_CASE("a step that throws leaves the next step full", "[dynamics][awake]") {
  ThrowOnce host;
  World world;

  world.workerCount = 4u;
  world.enqueueTask = throwingEnqueue;
  world.finishTask = finishInline;
  world.userTaskContext = &host;
  addFloor(world);

  std::vector<uint32_t> crates;

  for (uint32_t row = 0u; row != 20u; ++row) {
    for (uint32_t column = 0u; column != 20u; ++column) {
      crates.push_back(addCrate(
        world,
        glm::dvec3(
          0.3 * static_cast<double>(column) - 2.85,
          crateHalf,
          0.3 * static_cast<double>(row) - 2.85
        )
      ));
    }
  }

  for (uint32_t frame = 0u; frame != 600u && !allResting(world); ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);
  }

  REQUIRE(allResting(world));
  REQUIRE_FALSE(step(world).nonFinite);
  REQUIRE(walksAwakeOnly(world));

  const uint32_t lifted = crates[0];

  for (uint32_t index = 0u; index != 300u; ++index) {
    const RigidBody& body = world.bodies.rigids[indexOf(world, crates[index])];

    REQUIRE(setRigidPose(
      world,
      crates[index],
      body.position + glm::dvec3(0.0, 0.5, 0.0),
      upright
    ));
  }

  host.armed = true;

  REQUIRE_THROWS_AS(step(world), std::runtime_error);
  REQUIRE(host.calls == 1u);

  const double before = world.bodies.rigids[indexOf(world, lifted)].position.y;
  const bool listed = walksAwakeOnly(world);

  CHECK_FALSE(listed);

  uint32_t sparse = 0u;

  REQUIRE_FALSE(stepCounting(world, sparse).nonFinite);
  CHECK(sparse == 0u);
  CHECK(world.bodies.rigids[indexOf(world, lifted)].position.y < before);
  CHECK(listedAwake(world) == scannedAwake(world));
}
