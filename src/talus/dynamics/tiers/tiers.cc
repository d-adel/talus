#include "tiers.h"

#include <talus/deformable/deformedfield/deformedfield.h>
#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/fieldbody/fieldbody.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/resolution/resolution.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/strainstep/plasticflow.h>
#include <talus/pose.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

#include <Eigen/Dense>

#include <glm/gtc/quaternion.hpp>

#include <vector>

namespace talus::dynamics {

namespace {

constexpr uint32_t SAMPLE_BUDGET = 64u;

bool sameModuli(std::span<const Moduli> a, std::span<const Moduli> b) {
  if (a.size() != b.size()) {
    return false;
  }

  for (size_t index = 0u; index != a.size(); ++index) {
    if (a[index].shear != b[index].shear
        || a[index].shifted != b[index].shifted) {
      return false;
    }
  }

  return true;
}

std::shared_ptr<const ScreenFactor> screenFor(
  const BodySet& bodies,
  const LatticeBody& lattice
) {
  for (const SwitchableBody& other : bodies.switchables) {
    if (other.lattice.mesh == lattice.mesh
        && sameModuli(other.lattice.moduli, lattice.moduli)) {
      return other.screen;
    }
  }

  return factorScreen(*lattice.mesh, lattice.moduli);
}

glm::dvec3 forceOnSide(const Contact& contact, bool onA) {
  const glm::dvec3 onSideA = contact.memory.normal * contact.basis[0]
                             + contact.memory.friction.x * contact.basis[1]
                             + contact.memory.friction.y * contact.basis[2];

  return onA ? onSideA : -onSideA;
}

struct PairLoad {
  uint64_t pair = 0u;
  ContactSide partner;
  glm::dvec3 force{0.0};
  glm::dvec3 moment{0.0};
  std::vector<std::pair<glm::dvec3, glm::dvec3>> points;
};

bool partnerDistance(
  const World& world,
  const ContactSide& partner,
  const glm::dvec3& point,
  double& distance
) {
  if (partner.side == Side::Static) {
    const StaticField& fixed = world.bodies.statics[partner.body];

    distance =
      field::probe<double>(*fixed.field, staticLocal(fixed, point)).distance;

    return true;
  }

  if (partner.side == Side::Rigid) {
    const RigidBody& other = world.bodies.rigids[partner.body];

    if (other.field == nullptr) {
      return false;
    }

    const glm::dvec3 local =
      glm::transpose(glm::mat3_cast(other.rotation)) * (point - other.position);

    distance = field::probe<double>(
                 *other.field,
                 other.fieldOrigin + glm::mat3_cast(other.fieldRotation) * local
    )
                 .distance;

    return true;
  }

  return false;
}

void placePoint(
  const SwitchableBody& body,
  const glm::dmat3& toMesh,
  const glm::dvec3& local,
  const glm::dvec3& push,
  std::vector<glm::dvec3>& forces
) {
  const deformable::DeformedDistance hit = deformable::probeDeformed(
    *body.restProbe,
    body.rigid.fieldOrigin + glm::mat3_cast(body.rigid.fieldRotation) * local
  );

  if (!hit.found) {
    return;
  }

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    forces[hit.nodes[corner]] += hit.weights[corner] * (toMesh * push);
  }
}

bool touchingNodes(
  const World& world,
  const SwitchableBody& body,
  const RigidBody& rigid,
  const ContactSide& partner,
  std::vector<uint32_t>& touching,
  std::vector<glm::dvec3>& places
) {
  const talus::PreparedMesh& mesh = *body.lattice.mesh;
  const glm::dmat3 meshToWorld =
    glm::mat3_cast(rigid.rotation) * glm::transpose(glm::mat3_cast(rigid.fieldRotation));
  const double reach = 0.25 * body.lattice.cell;

  for (const uint32_t node : mesh.surfaceNodes) {
    const glm::dvec3 place =
      rigid.position
      + meshToWorld * (glm::dvec3(mesh.restPositions[node]) - rigid.fieldOrigin);
    double distance = 0.0;

    if (!partnerDistance(world, partner, place, distance)) {
      touching.clear();
      places.clear();

      return false;
    }

    if (distance <= reach) {
      touching.push_back(node);
      places.push_back(place);
    }
  }

  return true;
}

void spreadPair(
  const World& world,
  const SwitchableBody& body,
  const RigidBody& rigid,
  const PairLoad& load,
  std::vector<glm::dvec3>& forces
) {
  const glm::dmat3 toMesh =
    glm::mat3_cast(rigid.fieldRotation) * glm::transpose(glm::mat3_cast(rigid.rotation));
  std::vector<uint32_t> touching;
  std::vector<glm::dvec3> places;

  (void)touchingNodes(world, body, rigid, load.partner, touching, places);

  if (touching.empty()) {
    for (const auto& [local, push] : load.points) {
      placePoint(body, toMesh, local, push, forces);
    }

    return;
  }

  glm::dvec3 middle(0.0);
  double weight = 0.0;

  for (size_t index = 0u; index != touching.size(); ++index) {
    middle += body.rest.mass[touching[index]] * places[index];
    weight += body.rest.mass[touching[index]];
  }

  middle /= weight;

  const glm::dvec3 moment =
    load.moment - glm::cross(middle - rigid.position, load.force);
  glm::dmat3 spread(0.0);

  for (size_t index = 0u; index != touching.size(); ++index) {
    const glm::dvec3 arm = places[index] - middle;

    spread += body.rest.mass[touching[index]]
              * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const double size = spread[0][0] + spread[1][1] + spread[2][2];
  const bool solvable = places.size() >= 3u
                        && std::abs(glm::determinant(spread))
                             > 1.0e-9 * size * size * size;
  const glm::dvec3 tilt =
    solvable ? glm::inverse(spread) * moment : glm::dvec3(0.0);
  const glm::dvec3 share = load.force / weight;

  for (size_t index = 0u; index != touching.size(); ++index) {
    forces[touching[index]] +=
      body.rest.mass[touching[index]]
      * (toMesh * (share + glm::cross(tilt, places[index] - middle)));
  }
}

constexpr double saneDent = 0.5;

bool holdsAllModes(const SwitchableBody& body, std::span<const uint32_t> held) {
  if (held.size() < 3u) {
    return false;
  }

  const talus::PreparedMesh& mesh = *body.lattice.mesh;
  glm::dvec3 middle(0.0);
  double weight = 0.0;

  for (const uint32_t node : held) {
    middle += body.rest.mass[node] * glm::dvec3(mesh.restPositions[node]);
    weight += body.rest.mass[node];
  }

  middle /= weight;

  glm::dmat3 spread(0.0);

  for (const uint32_t node : held) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - middle;

    spread += body.rest.mass[node]
              * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const double size = spread[0][0] + spread[1][1] + spread[2][2];

  return std::abs(glm::determinant(spread)) > 1.0e-9 * size * size * size;
}

void screenRigid(
  const World& world,
  SwitchableBody& body,
  uint32_t index,
  const std::unordered_set<uint64_t>& recorded
) {
  const RigidBody& rigid = world.bodies.rigids[index];
  const glm::dmat3 turn = glm::mat3_cast(rigid.rotation);
  const glm::dmat3 toMesh = glm::mat3_cast(rigid.fieldRotation) * glm::transpose(turn);
  std::vector<PairLoad> loads;

  for (const Contact& contact : contactsOf(world, BodyRef{Side::Rigid, index}, recorded)) {
    const bool onA = contact.a.side == Side::Rigid && contact.a.body == index;
    const bool onB = contact.b.side == Side::Rigid && contact.b.body == index;

    if (!onA && !onB) {
      continue;
    }

    const glm::dvec3 push = forceOnSide(contact, onA);
    const glm::dvec3 local = (onA ? contact.a : contact.b).local;
    PairLoad* load = nullptr;

    for (PairLoad& entry : loads) {
      if (entry.pair == contact.pair) {
        load = &entry;
      }
    }

    if (load == nullptr) {
      loads.emplace_back();
      load = &loads.back();
      load->pair = contact.pair;
      load->partner = onA ? contact.b : contact.a;
    }

    load->force += push;
    load->moment += glm::cross(turn * local, push);
    load->points.emplace_back(local, push);
  }

  double total = 0.0;
  double turning = 0.0;

  for (const PairLoad& load : loads) {
    total += glm::length(load.force);
    turning += glm::length(load.moment);
  }

  turning /= body.rest.thickness;

  const double weight =
    body.rest.totalMass * glm::length(world.parameters.gravity);
  const double scale = body.checkedLoad + body.checkedTurn + weight;
  const double change = std::abs(total - body.checkedLoad)
                        + std::abs(turning - body.checkedTurn);

  if (body.checked && change <= world.parameters.screenLoadChange * scale) {
    return;
  }

  std::vector<glm::dvec3> forces(body.rest.mass.size());

  for (size_t node = 0u; node != forces.size(); ++node) {
    forces[node] = body.rest.mass[node] * (toMesh * world.parameters.gravity);
  }

  std::vector<uint32_t> held;

  for (const PairLoad& load : loads) {
    if (load.partner.side != Side::Static) {
      continue;
    }

    std::vector<uint32_t> touching;
    std::vector<glm::dvec3> places;

    (void)touchingNodes(world, body, rigid, load.partner, touching, places);
    held.insert(held.end(), touching.begin(), touching.end());
  }

  std::sort(held.begin(), held.end());
  held.erase(std::unique(held.begin(), held.end()), held.end());

  if (!holdsAllModes(body, held)) {
    held.clear();
  }

  for (const PairLoad& load : loads) {
    if (held.empty() || load.partner.side != Side::Static) {
      spreadPair(world, body, rigid, load, forces);
    }
  }

  if (!held.empty() && held != body.supportNodes) {
    body.supportScreen =
      factorScreen(*body.lattice.mesh, body.lattice.moduli, held);
    body.supportNodes = held;
  }

  const ScreenFactor& factor = held.empty() ? *body.screen : *body.supportScreen;
  ScreenResult result =
    screenLoad(factor, body.rest, *body.lattice.mesh, forces);

  body.dent = result.dent;
  body.displacement = std::move(result.displacement);
  body.checkedLoad = total;
  body.checkedTurn = turning;
  body.checked = true;
  ++body.checks;
}

struct PendingForce {
  uint32_t body = 0u;
  Side side = Side::Static;
  uint32_t other = 0u;
  glm::dvec3 force{0.0};
};

SwitchableBody* switchableOf(BodySet& bodies, uint32_t id) {
  for (SwitchableBody& body : bodies.switchables) {
    if (body.id == id) {
      return &body;
    }
  }

  return nullptr;
}

uint32_t otherKey(const BodySet& bodies, const ContactSide& side) {
  if (side.side == Side::Rigid) {
    return bodies.rigids[side.body].id;
  }

  if (side.side == Side::Lattice) {
    return bodies.lattices[side.body].id;
  }

  return side.body;
}

void collectForces(
  const World& world,
  uint32_t id,
  std::vector<PendingForce>& pending,
  const std::unordered_set<uint64_t>& recorded
) {
  const std::optional<BodyRef> self = findBody(world.bodies, id);

  if (!self) {
    return;
  }

  for (const Contact& contact : contactsOf(world, *self, recorded)) {
    const bool onA = contact.a.side == self->side && contact.a.body == self->index;
    const bool onB = contact.b.side == self->side && contact.b.body == self->index;

    if (!onA && !onB) {
      continue;
    }

    const ContactSide& other = onA ? contact.b : contact.a;
    const uint32_t key = otherKey(world.bodies, other);
    const glm::dvec3 push = forceOnSide(contact, onA);
    bool merged = false;

    for (PendingForce& entry : pending) {
      if (entry.body == id && entry.side == other.side && entry.other == key) {
        entry.force += push;
        merged = true;
      }
    }

    if (!merged) {
      pending.push_back({id, other.side, key, push});
    }
  }
}

void resolveForces(World& world, std::span<const PendingForce> pending) {
  for (const PendingForce& entry : pending) {
    const std::optional<BodyRef> self = findBody(world.bodies, entry.body);
    const std::optional<BodyRef> other =
      entry.side == Side::Static ? std::optional<BodyRef>(BodyRef{Side::Static, entry.other})
                                 : findBody(world.bodies, entry.other);

    if (self && other) {
      world.carried.push_back(
        {entry.body, pairKey(world.bodies, *self, *other), entry.force}
      );
    }
  }
}

struct PendingMove {
  uint32_t body = 0u;
  glm::dvec3 shift{0.0};
};

void collectMoves(
  World& world,
  uint32_t id,
  std::vector<PendingMove>& moves,
  const std::unordered_set<uint64_t>& recorded
) {
  const SwitchableBody* body = switchableOf(world.bodies, id);
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (body == nullptr || !ref || ref->side != Side::Rigid
      || body->displacement.size() != body->rest.mass.size()
      || !(body->dent <= saneDent)) {
    return;
  }

  const RigidBody& rigid = world.bodies.rigids[ref->index];
  const talus::PreparedMesh& mesh = *body->lattice.mesh;
  const glm::dmat3 meshToWorld =
    glm::mat3_cast(rigid.rotation) * glm::transpose(glm::mat3_cast(rigid.fieldRotation));
  const double reach = 0.25 * body->lattice.cell;
  std::vector<uint32_t> partners;

  for (const Contact& contact : contactsOf(world, *ref, recorded)) {
    const bool onA = contact.a.side == Side::Rigid && contact.a.body == ref->index;
    const bool onB = contact.b.side == Side::Rigid && contact.b.body == ref->index;
    const ContactSide& other = onA ? contact.b : contact.a;

    if ((onA || onB) && other.side == Side::Rigid
        && std::find(partners.begin(), partners.end(), other.body) == partners.end()) {
      partners.push_back(other.body);
    }
  }

  for (const uint32_t partner : partners) {
    ContactSide side;
    glm::dvec3 shift(0.0);
    double count = 0.0;

    side.side = Side::Rigid;
    side.body = partner;

    for (const uint32_t node : mesh.surfaceNodes) {
      const glm::dvec3 place =
        rigid.position
        + meshToWorld * (glm::dvec3(mesh.restPositions[node]) - rigid.fieldOrigin);
      double distance = 0.0;

      if (partnerDistance(world, side, place, distance) && distance <= reach) {
        shift += meshToWorld * body->displacement[node];
        count += 1.0;
      }
    }

    if (count > 0.0) {
      moves.push_back({world.bodies.rigids[partner].id, shift / count});
    }
  }
}

void applyMoves(World& world, std::span<const PendingMove> moves) {
  for (const PendingMove& move : moves) {
    const std::optional<BodyRef> ref = findBody(world.bodies, move.body);

    if (ref && ref->side == Side::Rigid) {
      world.bodies.rigids[ref->index].position += move.shift;
    }
  }
}

constexpr uint32_t PLASTIC_ITERATIONS = 16u;

double plasticDent(const LatticeBody& lattice, const RestBody& rest) {
  const std::vector<glm::dvec3> shape = plasticShape(lattice, PLASTIC_ITERATIONS);
  double dent = 0.0;

  for (size_t node = 0u; node != shape.size(); ++node) {
    dent = std::max(dent, glm::length(lattice.position[node] - shape[node]));
  }

  return dent / rest.thickness;
}

[[nodiscard]] std::shared_ptr<const talus::PreparedMesh> withMass(
  std::shared_ptr<const talus::PreparedMesh> mesh,
  double mass
) {
  double total = 0.0;

  for (const float nodeMass : mesh->nodeMass) {
    total += nodeMass;
  }

  if (!(mass > 0.0) || !(total > 0.0) || total == mass) {
    return mesh;
  }

  auto scaled = std::make_shared<talus::PreparedMesh>(*mesh);
  const float keep = static_cast<float>(mass / total);

  for (float& nodeMass : scaled->nodeMass) {
    nodeMass *= keep;
  }

  return scaled;
}

void rebuildSwitchable(SwitchableBody& body, std::shared_ptr<const talus::PreparedMesh> mesh) {
  const std::vector<talus::PhysicalMaterial> materials = body.lattice.materials;
  const double friction = body.lattice.friction;
  const uint32_t substeps = body.lattice.substeps;

  body.lattice = makeLattice(mesh, materials, glm::dvec3(0.0), glm::dvec3(0.0), friction, body.baked.get());
  body.lattice.id = body.id;
  body.lattice.substeps = substeps;
  body.rest = restBodyOf(*mesh);
  body.screen = factorScreen(*mesh, body.lattice.moduli);
  body.supportScreen = nullptr;
  body.supportNodes.clear();
  body.displacement.clear();
  body.dent = 0.0;

  auto probe = std::make_shared<deformable::DeformedProbe>(deformable::deformedProbe(*mesh, *body.baked));
  std::vector<glm::dvec3> rest(mesh->restPositions.size());

  for (size_t node = 0u; node != rest.size(); ++node) {
    rest[node] = glm::dvec3(mesh->restPositions[node]);
  }

  deformable::placeDeformedProbe(*probe, rest);
  body.restProbe = std::move(probe);

  RigidBody shape = rigidFromField(*body.baked, {}, materials, SAMPLE_BUDGET);
  const double keep = shape.mass > 0.0 ? body.rigid.mass / shape.mass : 1.0;

  body.rigid.field = body.baked.get();
  body.rigid.samples = std::move(shape.samples);
  body.rigid.moment = keep * shape.moment;
  body.rigid.fieldOrigin = shape.fieldOrigin;
  body.rigid.fieldRotation = shape.fieldRotation;
}

RigidBody latticeShape(const talus::PreparedMesh& mesh, double friction, double restitution) {
  const RestBody rest = restBodyOf(mesh);
  const glm::dmat3 axes = glm::mat3_cast(rest.axes);
  RigidBody shape;

  shape.samples.reserve(mesh.surfaceSeeds.size() + mesh.surfaceNodes.size());

  for (const glm::vec3& seed : mesh.surfaceSeeds) {
    shape.samples.push_back(glm::transpose(axes) * (glm::dvec3(seed) - rest.centre));
  }

  for (const uint32_t node : mesh.surfaceNodes) {
    shape.samples.push_back(glm::transpose(axes) * (glm::dvec3(mesh.restPositions[node]) - rest.centre));
  }

  shape.mass = rest.totalMass;
  shape.moment = rest.moments;
  shape.fieldOrigin = rest.centre;
  shape.fieldRotation = rest.axes;
  shape.friction = friction;
  shape.restitution = restitution;

  return shape;
}

[[nodiscard]] bool bakeDent(
  SwitchableBody& body,
  const LatticeBody& lattice,
  const LatticeBody& shaped,
  const LatticeFit& fit
) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::vector<glm::vec3> local(shaped.position.size());

  for (size_t node = 0u; node != local.size(); ++node) {
    local[node] = glm::vec3(body.rest.centre + glm::transpose(fit.rotation) * (shaped.position[node] - fit.centre));
  }

  auto baked = std::make_shared<field::SampledSdf>(*lattice.field);

  if (baked->materials.size() != baked->distances.size()) {
    baked->materials.assign(baked->distances.size(), 0u);
  }

  (void)deformable::bakeDeformedField(*lattice.field, mesh, local, talus::Pose{}, *baked, true);

  const std::shared_ptr<const talus::PreparedMesh> built = deformable::buildMesh(
    grownField(*baked),
    body.lattice.materials,
    glm::mat4(1.0f),
    std::span<const deformable::SupportVolume>{},
    talus::VolumeElementType::Hex
  );

  if (built == nullptr || built->hexes.empty()) {
    return false;
  }

  body.baked = std::move(baked);
  rebuildSwitchable(body, withMass(built, body.rest.totalMass));

  return true;
}

[[nodiscard]] glm::dvec3 spinOf(const glm::dmat3& inertia, const glm::dvec3& turning, const glm::dvec3& otherwise) {
  const double size = inertia[0][0] + inertia[1][1] + inertia[2][2];

  return std::abs(glm::determinant(inertia)) > 1.0e-9 * size * size * size
           ? glm::inverse(inertia) * turning
           : otherwise;
}

void moveLikeRigid(LatticeBody& lattice, const RestBody& rest, const RigidBody& rigid) {
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    centre += rest.mass[node] * lattice.position[node];
  }

  centre /= rest.totalMass;

  glm::dmat3 inertia(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const glm::dvec3 arm = lattice.position[node] - centre;

    inertia += rest.mass[node] * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const glm::dvec3 mean = rigid.mass * rigid.velocity / rest.totalMass;
  const glm::dvec3 turning =
    worldInertia(rigid) * rigid.angularVelocity
    - rest.totalMass * glm::cross(centre - rigid.position, mean);
  const glm::dvec3 spin = spinOf(inertia, turning, rigid.angularVelocity);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    lattice.velocity[node] = mean + glm::cross(spin, lattice.position[node] - centre);
  }
}

bool jointEnd(const World& world, uint32_t id) {
  const std::span<const Joint> joints = world.joints.all();

  return std::any_of(joints.begin(), joints.end(), [id](const Joint& joint) {
    return joint.def.bodyA == id || joint.def.bodyB == id;
  });
}

[[nodiscard]] bool toLattice(World& world, uint32_t id) {
  SwitchableBody* body = switchableOf(world.bodies, id);
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (body == nullptr || !ref || ref->side != Side::Rigid) {
    return false;
  }

  const RigidBody rigid = world.bodies.rigids[ref->index];
  const glm::dmat3 meshToWorld =
    glm::mat3_cast(rigid.rotation) * glm::transpose(glm::mat3_cast(rigid.fieldRotation));
  LatticeBody lattice = body->lattice;
  const talus::PreparedMesh& mesh = *lattice.mesh;

  world.bodies.rigids.erase(world.bodies.rigids.begin() + ref->index);
  forgetBody(world, Side::Rigid, ref->index, id);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rigid.fieldOrigin;
    const glm::dvec3 bend =
      body->displacement.size() == lattice.position.size()
          && body->dent <= saneDent
        ? body->displacement[node]
        : glm::dvec3(0.0);

    lattice.position[node] = rigid.position + meshToWorld * (arm + bend);
  }

  moveLikeRigid(lattice, body->rest, rigid);
  lattice.id = id;
  lattice.previousVelocity = lattice.velocity;
  lattice.initial = lattice.position;
  lattice.inertial = lattice.position;
  world.bodies.lattices.push_back(std::move(lattice));
  body->tier = Tier::Lattice;
  body->frames = 0u;

  return true;
}

[[nodiscard]] bool toRigid(World& world, uint32_t id) {
  SwitchableBody* body = switchableOf(world.bodies, id);
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (body == nullptr || !ref || ref->side != Side::Lattice
      || world.bodies.lattices[ref->index].mesh != body->lattice.mesh) {
    return false;
  }

  const LatticeBody lattice = world.bodies.lattices[ref->index];
  LatticeFit fit = fitLattice(lattice, body->rest);
  const glm::dvec3 reference = body->rest.centre;
  const glm::dvec3 massCentre = fit.centre;
  glm::dvec3 momentum(0.0);
  glm::dvec3 turning(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    momentum += body->rest.mass[node] * lattice.velocity[node];
    turning += body->rest.mass[node]
               * glm::cross(lattice.position[node] - massCentre, lattice.velocity[node]);
  }

  if (flowed(lattice) && lattice.field != nullptr) {
    LatticeBody shaped = lattice;

    shaped.position = plasticShape(lattice, PLASTIC_ITERATIONS);

    const LatticeFit plasticFit = fitLattice(shaped, body->rest);

    if (!bakeDent(*body, lattice, shaped, plasticFit)) {
      body->frames = 0u;

      return false;
    }

    fit.rotation = plasticFit.rotation;
    fit.centre = plasticFit.centre;
  }

  world.bodies.lattices.erase(world.bodies.lattices.begin() + ref->index);
  forgetBody(world, Side::Lattice, ref->index, id);

  RigidBody rigid = body->rigid;
  const glm::dvec3 arm = fit.rotation * (rigid.fieldOrigin - reference);

  rigid.id = id;
  rigid.position = fit.centre + arm;
  rigid.rotation = glm::quat_cast(fit.rotation * glm::mat3_cast(rigid.fieldRotation));
  rigid.velocity = momentum / rigid.mass;
  rigid.previousVelocity = rigid.velocity;
  rigid.initialPosition = rigid.position;
  rigid.inertialPosition = rigid.position;
  rigid.initialRotation = rigid.rotation;
  rigid.inertialRotation = rigid.rotation;
  rigid.angularVelocity = spinOf(
    worldInertia(rigid),
    turning - glm::cross(rigid.position - massCentre, momentum),
    glm::dvec3(0.0)
  );
  world.bodies.rigids.push_back(rigid);
  body->tier = Tier::Rigid;
  body->frames = 0u;
  body->checked = false;

  return true;
}

}

LatticeFit fitLattice(const LatticeBody& lattice, const RestBody& rest) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  LatticeFit fit;
  glm::dmat3 moment(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    fit.centre += rest.mass[node] * lattice.position[node];
  }

  fit.centre /= rest.totalMass;

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    moment += rest.mass[node]
              * glm::outerProduct(
                lattice.position[node] - fit.centre,
                glm::dvec3(mesh.restPositions[node]) - rest.centre
              );
  }

  Eigen::Matrix3d matrix;

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      matrix(row, column) = moment[column][row];
    }
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    matrix,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );
  Eigen::Matrix3d left = svd.matrixU();
  Eigen::Matrix3d turn = left * svd.matrixV().transpose();

  if (turn.determinant() < 0.0) {
    left.col(2) = -left.col(2);
    turn = left * svd.matrixV().transpose();
  }

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      fit.rotation[column][row] = turn(row, column);
    }
  }

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rest.centre;

    fit.dent = std::max(
      fit.dent,
      glm::length(lattice.position[node] - fit.centre - fit.rotation * arm)
    );
  }

  fit.dent /= rest.thickness;

  return fit;
}

uint32_t addSwitchable(
  BodySet& bodies,
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const field::SampledSdf& field,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  double friction,
  double restitution
) {
  const RigidBody shape = latticeShape(*mesh, friction, restitution);

  return addSwitchable(bodies, std::move(mesh), materials, field, offset, velocity, shape);
}

uint32_t addSwitchable(
  BodySet& bodies,
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const field::SampledSdf& field,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  const RigidBody& shape
) {
  SwitchableBody body;

  mesh = withMass(std::move(mesh), shape.mass);
  body.lattice =
    makeLattice(mesh, materials, offset, velocity, shape.friction, &field);
  body.rest = restBodyOf(*mesh);
  body.screen = screenFor(bodies, body.lattice);

  auto probe = std::make_shared<deformable::DeformedProbe>(
    deformable::deformedProbe(*mesh, field)
  );
  std::vector<glm::dvec3> rest(mesh->restPositions.size());

  for (size_t node = 0u; node != rest.size(); ++node) {
    rest[node] = glm::dvec3(mesh->restPositions[node]);
  }

  deformable::placeDeformedProbe(*probe, rest);
  body.restProbe = std::move(probe);

  const uint32_t index = addRigidBody(
    bodies,
    field,
    shape.samples,
    shape.mass,
    shape.moment,
    offset + shape.fieldOrigin,
    velocity,
    shape.friction,
    shape.restitution
  );
  RigidBody& rigid = bodies.rigids[index];

  rigid.rotation = shape.fieldRotation;
  rigid.initialRotation = shape.fieldRotation;
  rigid.inertialRotation = shape.fieldRotation;
  rigid.fieldOrigin = shape.fieldOrigin;
  rigid.fieldRotation = shape.fieldRotation;
  body.id = rigid.id;
  body.rigid = rigid;
  body.lattice.id = rigid.id;
  bodies.switchables.push_back(std::move(body));

  return rigid.id;
}

std::optional<BodyRef> findBody(const BodySet& bodies, uint32_t id) {
  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    if (bodies.rigids[index].id == id) {
      return BodyRef{Side::Rigid, index};
    }
  }

  for (uint32_t index = 0u; index != bodies.lattices.size(); ++index) {
    if (bodies.lattices[index].id == id) {
      return BodyRef{Side::Lattice, index};
    }
  }

  return std::nullopt;
}

void switchTiers(World& world, StepReport& report) {
  std::vector<uint32_t> promoting;
  std::vector<uint32_t> demoting;
  std::optional<std::unordered_set<uint64_t>> recorded;
  const auto recordedNow = [&]() -> const std::unordered_set<uint64_t>& {
    if (!recorded) {
      recorded = recordedPairs(world.contacts);
    }

    return *recorded;
  };

  for (SwitchableBody& body : world.bodies.switchables) {
    const std::optional<BodyRef> ref = findBody(world.bodies, body.id);

    if (!ref) {
      continue;
    }

    if (body.tier == Tier::Rigid && jointEnd(world, body.id)) {
      body.frames = 0u;
    } else if (body.tier == Tier::Rigid) {
      screenRigid(world, body, ref->index, recordedNow());
      body.frames =
        body.dent >= world.parameters.promoteDent ? body.frames + 1u : 0u;

      if (body.frames >= world.parameters.switchFrames) {
        promoting.push_back(body.id);
      }
    } else {
      if (world.bodies.lattices[ref->index].mesh != body.lattice.mesh) {
        body.frames = 0u;
        continue;
      }

      const LatticeBody& lattice = world.bodies.lattices[ref->index];
      const bool plastic = flowed(lattice);

      body.dent = plastic ? plasticDent(lattice, body.rest) : fitLattice(lattice, body.rest).dent;
      body.frames =
        body.dent < world.parameters.demoteDent
            && (!plastic || sleepSpeed(lattice, world.parameters.dt) < world.parameters.sleepThreshold)
          ? body.frames + 1u
          : 0u;

      if (body.frames >= world.parameters.switchFrames) {
        demoting.push_back(body.id);
      }
    }
  }

  std::vector<PendingForce> pending;
  std::vector<PendingMove> moves;

  for (const uint32_t id : promoting) {
    collectForces(world, id, pending, recordedNow());
    collectMoves(world, id, moves, recordedNow());
  }

  for (const uint32_t id : demoting) {
    collectForces(world, id, pending, recordedNow());
  }

  for (const uint32_t id : promoting) {
    (void)toLattice(world, id);
    ++report.promotions;
  }

  for (const uint32_t id : demoting) {
    if (toRigid(world, id)) {
      ++report.demotions;
    } else {
      std::erase_if(pending, [id](const PendingForce& entry) { return entry.body == id; });
    }
  }

  applyMoves(world, moves);
  resolveForces(world, pending);
}

bool promote(World& world, uint32_t id) {
  if (jointEnd(world, id)) {
    return false;
  }

  std::vector<PendingForce> pending;
  std::vector<PendingMove> moves;
  const std::unordered_set<uint64_t> recorded = recordedPairs(world.contacts);

  collectForces(world, id, pending, recorded);
  collectMoves(world, id, moves, recorded);
  const bool promoted = toLattice(world, id);
  applyMoves(world, moves);
  resolveForces(world, pending);

  return promoted;
}

std::vector<uint8_t> shareCarriedForces(
  std::span<const CarriedForce> carried,
  std::span<Contact> contacts,
  const BodySet& bodies
) {
  std::vector<uint8_t> shared(contacts.size(), 0u);

  for (const CarriedForce& share : carried) {
    const std::optional<BodyRef> self = findBody(bodies, share.body);

    if (!self) {
      continue;
    }

    uint32_t touching = 0u;
    uint32_t all = 0u;

    for (const Contact& contact : contacts) {
      if (contact.pair == share.pair) {
        ++all;
        touching += contact.gap <= 0.0 ? 1u : 0u;
      }
    }

    if (all == 0u) {
      continue;
    }

    const bool onlyTouching = touching != 0u;
    const glm::dvec3 each =
      share.force / static_cast<double>(onlyTouching ? touching : all);

    for (size_t index = 0u; index != contacts.size(); ++index) {
      Contact& contact = contacts[index];

      if (contact.pair != share.pair || (onlyTouching && contact.gap > 0.0)) {
        continue;
      }

      const bool onA = contact.a.side == self->side && contact.a.body == self->index;
      const glm::dvec3 onSideA = onA ? each : -each;

      contact.memory.normal = std::max(glm::dot(onSideA, contact.basis[0]), 0.0);
      contact.memory.friction = glm::dvec2(
        glm::dot(onSideA, contact.basis[1]),
        glm::dot(onSideA, contact.basis[2])
      );
      shared[index] = 1u;
    }
  }

  return shared;
}

void demote(World& world, uint32_t id) {
  std::vector<PendingForce> pending;

  collectForces(world, id, pending, recordedPairs(world.contacts));

  if (toRigid(world, id)) {
    resolveForces(world, pending);
  }
}

}
