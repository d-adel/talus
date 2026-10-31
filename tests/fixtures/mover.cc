#include "mover.h"

#include "fieldprimitives.h"

#include <catch2/catch_test_macros.hpp>

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/tiers/tiers.h>

#include <span>

namespace fixture {

using namespace talus::dynamics;

namespace {

talus::field::SampledSdf boxOf(
  const glm::dvec3& half,
  double voxel,
  double outset = 0.0
) {
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(
      glm::vec3(0.0f),
      glm::vec3(half) + glm::vec3(static_cast<float>(outset))
    )},
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

talus::PhysicalMaterial jelly() {
  talus::PhysicalMaterial material;
  material.density = 500.0f;
  material.friction = 0.7f;
  material.youngsModulus = 5.0e4f;
  material.poissonsRatio = 0.4f;

  return material;
}

const Island* islandWith(const World& world, uint32_t index) {
  for (const Island& island : world.islands) {
    for (const BodyRef& ref : island.bodies) {
      if (ref.side == Side::Rigid && ref.index == index) {
        return &island;
      }
    }
  }

  return nullptr;
}

}

const talus::field::SampledSdf& characterField() {
  static const talus::field::SampledSdf field = boxOf(characterHalf, 0.05);

  return field;
}

const talus::field::SampledSdf& groundField() {
  static const talus::field::SampledSdf field =
    boxOf(glm::dvec3(3.0, 0.5, 3.0), 0.05);

  return field;
}

Character addCharacter(World& world, const glm::dvec3& at, uint32_t planarBase) {
  const uint32_t index = addRigidBox(
    world.bodies,
    characterField(),
    characterHalf,
    characterMass,
    at,
    glm::dvec3(0.0),
    0.0,
    3u
  );
  Character character;

  character.body = world.bodies.rigids[index].id;

  JointDef def;
  def.kind = JointKind::PlanarVelocity;
  def.bodyA = planarBase;
  def.bodyB = character.body;
  def.collideConnected = true;
  def.planar.maxForce = groundForce;

  character.planar = addJoint(world, def);
  REQUIRE(character.planar != 0u);

  return character;
}

bool touching(const World& world, uint32_t a, uint32_t b) {
  const BodyRef ref = *findBody(world.bodies, a);

  for (const Contact& contact : contactsOf(world, ref)) {
    if (contact.a.side == Side::Rigid && contact.b.side == Side::Rigid) {
      const uint32_t idA = world.bodies.rigids[contact.a.body].id;
      const uint32_t idB = world.bodies.rigids[contact.b.body].id;

      if ((idA == a && idB == b) || (idA == b && idB == a)) {
        return true;
      }
    }
  }

  return false;
}

SoftBlock softBlock(const glm::dvec3& half, double voxel) {
  SoftBlock block;
  block.field = boxOf(half, voxel, 1.0e-4);
  block.lanes.assign(2u, jelly());
  block.mesh = talus::deformable::buildMesh(
    block.field,
    block.lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );

  return block;
}

void addSoftBlock(
  World& world,
  const SoftBlock& block,
  const glm::dvec3& at,
  const glm::dvec3& velocity
) {
  (void)addLattice(
    world.bodies,
    block.mesh,
    block.lanes,
    at,
    velocity,
    block.lanes.front().friction,
    &block.field
  );
}

void requireStrainstep(const World& world, uint32_t index) {
  const Island* island = islandWith(world, index);

  REQUIRE(island != nullptr);
  REQUIRE(island->owner == Owner::Strainstep);
}

}
