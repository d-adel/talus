#include "mover.h"

#include "fieldprimitives.h"

#include <catch2/catch_test_macros.hpp>

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/tiers/tiers.h>

#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <numbers>
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

const RigidBody& rigidOf(const World& world, uint32_t id) {
  return world.bodies.rigids[findBody(world.bodies, id)->index];
}

glm::dvec3 groundPoint(
  const World& world,
  uint32_t ground,
  const glm::dvec3& point
) {
  if (ground == noBody) {
    return point;
  }

  const RigidBody& body = rigidOf(world, ground);

  return glm::conjugate(body.rotation) * (point - body.position);
}

double angularFrequency(double hertz) {
  return 2.0 * std::numbers::pi * hertz;
}

double dampedShare(double dampingRatio) {
  return std::sqrt(1.0 - dampingRatio * dampingRatio);
}

}

const talus::field::SampledSdf& stepField() {
  static const talus::field::SampledSdf field = [] {
    const float voxel = 0.02f;
    const glm::vec3 low(-1.54f, -0.24f, -0.64f);
    const glm::vec3 high(2.04f, 0.2f, 0.64f);

    return talus::field::bake(
      std::vector<talus::field::Primitive>{
        fixture::box(glm::vec3(0.25f, -0.1f, 0.0f), glm::vec3(1.75f, 0.1f, 0.6f)),
        fixture::box(
          glm::vec3(1.15f, -0.025f, 0.0f),
          glm::vec3(0.85f, 0.175f, 0.6f)
        )
      },
      low,
      high,
      glm::uvec3(glm::round((high - low) / voxel)) + glm::uvec3(1u)
    );
  }();

  return field;
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

uint32_t addUprightSpring(World& world, uint32_t body) {
  JointDef def;
  def.kind = JointKind::Spring;
  def.bodyB = body;
  def.spring.angularStiffness = 2.0e4;
  def.spring.angularDamping = 1.25e3;

  const uint32_t joint = addJoint(world, def);
  REQUIRE(joint != 0u);

  return joint;
}

double weightOf(const World& world) {
  return characterMass * glm::length(world.parameters.gravity);
}

PogoSettings pogoFor(const World& world, const glm::dvec3& normal) {
  const double cap = capScale * weightOf(world);
  PogoSettings pogo;
  pogo.normal = normal;
  pogo.restLength = pogoRest;
  pogo.hertz = pogoHertz;
  pogo.dampingRatio = pogoDamping;
  pogo.maxTension = cap;
  pogo.maxCompression = cap;

  return pogo;
}

std::optional<WorldRayHit> moveCharacter(
  World& world,
  Character& character,
  const MoverStep& step
) {
  const RigidBody& body = rigidOf(world, character.body);
  const glm::dvec3 from = body.position + body.rotation * footAnchor;
  const glm::dvec3 axis = body.rotation * glm::dvec3(0.0, 1.0, 0.0);
  const double reach = character.onGround ? 2.0 * pogoRest : pogoRest;
  std::vector<RaySkip> skips{RaySkip{character.body, noChild}};

  skips.insert(skips.end(), step.skips.begin(), step.skips.end());

  const std::optional<WorldRayHit> hit =
    castRay(world, from, from - glm::dvec3(0.0, reach, 0.0), skips);
  double force = 0.0;
  double speed = 0.0;
  double released = 0.0;

  if (character.pogo != 0u) {
    const JointReport report = *jointReport(world, character.pogo);

    force = report.pogoForce;
    speed = report.pogoSpeed;
    released = report.pogoReleased;
    REQUIRE(removeJoint(world, character.pogo));
    character.pogo = 0u;
  }

  character.onGround =
    hit && glm::dot(glm::normalize(hit->normal), axis) >= pogoSteepestCosine;
  character.ground = noBody;

  if (character.onGround) {
    character.ground = hit->side == Side::Rigid ? hit->body : noBody;

    JointDef def;
    def.kind = JointKind::Pogo;
    def.bodyA = character.ground;
    def.bodyB = character.body;
    def.frameA.position = groundPoint(world, character.ground, hit->point);
    def.frameB.position = footAnchor;
    def.collideConnected = true;
    def.pogo = pogoFor(world, hit->normal);
    def.pogo.force = force;
    def.pogo.speed = speed;
    def.pogo.released = released;

    character.pogo = addJoint(world, def);
    REQUIRE(character.pogo != 0u);
  }

  REQUIRE(setPlanarVelocity(
    world,
    character.planar,
    {step.velocity, step.maxForce}
  ));

  return hit;
}

double springResponse(
  double hertz,
  double dampingRatio,
  double displacement,
  double time
) {
  const double omega = angularFrequency(hertz);
  const double share = dampedShare(dampingRatio);
  const double damped = omega * share * time;

  return displacement * std::exp(-dampingRatio * omega * time)
         * (std::cos(damped) + dampingRatio / share * std::sin(damped));
}

double springPeakSpeed(
  double hertz,
  double dampingRatio,
  double displacement
) {
  const double share = dampedShare(dampingRatio);
  const double phase = std::atan2(share, dampingRatio);

  return displacement * angularFrequency(hertz)
         * std::exp(-dampingRatio * phase / share);
}

double springOvershoot(double dampingRatio) {
  return std::exp(
    -dampingRatio * std::numbers::pi / dampedShare(dampingRatio)
  );
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
