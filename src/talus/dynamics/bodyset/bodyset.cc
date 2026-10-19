#include "bodyset.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace talus::dynamics {

LatticeBody makeLattice(
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  double friction,
  const field::SampledSdf* restField
) {
  LatticeBody body;
  const size_t nodes = mesh->restPositions.size();
  glm::dvec3 smallest(std::numeric_limits<double>::infinity());

  for (const glm::vec3& rest : mesh->restPositions) {
    smallest = glm::min(smallest, glm::dvec3(rest));
  }

  const glm::dvec3 voxel(mesh->voxelSize);

  body.position.resize(nodes);
  body.velocity.assign(nodes, velocity);
  body.previousVelocity.assign(nodes, velocity);
  body.colour.resize(nodes);

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::dvec3 rest(mesh->restPositions[node]);
    const glm::dvec3 index = glm::round((rest - smallest) / voxel);

    body.position[node] = rest + offset;
    body.colour[node] = static_cast<uint8_t>(
      (static_cast<uint32_t>(index.x) & 1u)
      | (static_cast<uint32_t>(index.y) & 1u) << 1u
      | (static_cast<uint32_t>(index.z) & 1u) << 2u
    );

    if (!mesh->anchored.empty() && mesh->anchored[node] != 0u) {
      body.velocity[node] = glm::dvec3(0.0);
      body.previousVelocity[node] = glm::dvec3(0.0);
    }
  }

  body.initial = body.position;
  body.inertial = body.position;
  body.mesh = std::move(mesh);

  rebuildAdjacency(body);

  body.moduli.reserve(body.mesh->hexes.size());
  body.gaussGradientsOf.reserve(body.mesh->hexes.size());

  std::vector<glm::vec3> sizes;

  for (const talus::PreparedHex& hex : body.mesh->hexes) {
    const size_t lane = std::min<size_t>(hex.material, materials.size() - 1u);
    const uint32_t size = static_cast<uint32_t>(
      std::find(sizes.begin(), sizes.end(), hex.halfExtents) - sizes.begin()
    );

    if (size == sizes.size()) {
      sizes.push_back(hex.halfExtents);
      body.gaussGradients.push_back(
        gaussGradients(glm::dvec3(hex.halfExtents))
      );
    }

    body.moduli.push_back(moduliOf(materials[lane]));
    body.gaussGradientsOf.push_back(size);
  }

  body.friction = friction;
  body.materials.assign(materials.begin(), materials.end());
  body.field = restField;
  body.cell = std::min({voxel.x, voxel.y, voxel.z});

  return body;
}

void rebuildAdjacency(LatticeBody& lattice) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  const size_t nodes = mesh.restPositions.size();

  lattice.cellStart.assign(nodes + 1u, 0u);

  for (const talus::PreparedHex& hex : mesh.hexes) {
    for (const uint32_t node : hex.nodes) {
      ++lattice.cellStart[node + 1u];
    }
  }

  for (size_t node = 0u; node != nodes; ++node) {
    lattice.cellStart[node + 1u] += lattice.cellStart[node];
  }

  lattice.cellOf.resize(lattice.cellStart.back());
  lattice.cornerOf.resize(lattice.cellStart.back());

  std::vector<uint32_t> filled(lattice.cellStart.begin(), lattice.cellStart.end() - 1);

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const uint32_t node = mesh.hexes[cell].nodes[corner];

      lattice.cellOf[filled[node]] = cell;
      lattice.cornerOf[filled[node]] = static_cast<uint8_t>(corner);
      ++filled[node];
    }
  }
}

uint32_t addLattice(
  BodySet& bodies,
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  double friction,
  const field::SampledSdf* restField
) {
  LatticeBody body =
    makeLattice(std::move(mesh), materials, offset, velocity, friction, restField);

  body.id = bodies.nextId++;
  bodies.lattices.push_back(std::move(body));

  return static_cast<uint32_t>(bodies.lattices.size() - 1u);
}

uint32_t addRigidBody(
  BodySet& bodies,
  const field::SampledSdf& field,
  std::span<const glm::dvec3> samples,
  double mass,
  const glm::dvec3& moment,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double friction,
  double restitution
) {
  RigidBody body;

  body.field = &field;
  body.samples.assign(samples.begin(), samples.end());
  body.mass = mass;
  body.moment = moment;
  body.position = position;
  body.velocity = velocity;
  body.previousVelocity = velocity;
  body.initialPosition = position;
  body.inertialPosition = position;
  body.friction = friction;
  body.restitution = restitution;
  body.id = bodies.nextId++;
  bodies.rigids.push_back(std::move(body));

  return static_cast<uint32_t>(bodies.rigids.size() - 1u);
}

uint32_t addRigidSphere(
  BodySet& bodies,
  const field::SampledSdf& field,
  double radius,
  double mass,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double friction,
  double restitution
) {
  const uint32_t index = addRigidBody(
    bodies,
    field,
    std::span<const glm::dvec3>{},
    mass,
    glm::dvec3(0.4 * mass * radius * radius),
    position,
    velocity,
    friction,
    restitution
  );

  bodies.rigids[index].radius = radius;
  bodies.rigids[index].shape = RigidShape::Sphere;

  return index;
}

uint32_t addRigidBox(
  BodySet& bodies,
  const field::SampledSdf& field,
  const glm::dvec3& halfExtents,
  double mass,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double friction,
  uint32_t samplesPerEdge
) {
  const glm::dvec3 size = 2.0 * halfExtents;
  const glm::dvec3 moment(
    mass * (size.y * size.y + size.z * size.z) / 12.0,
    mass * (size.x * size.x + size.z * size.z) / 12.0,
    mass * (size.x * size.x + size.y * size.y) / 12.0
  );
  std::vector<glm::dvec3> samples;
  const uint32_t count = std::max(samplesPerEdge, 2u);
  const double last = static_cast<double>(count - 1u);

  for (uint32_t i = 0u; i != count; ++i) {
    for (uint32_t j = 0u; j != count; ++j) {
      for (uint32_t k = 0u; k != count; ++k) {
        const bool surface = i == 0u || j == 0u || k == 0u || i == count - 1u
                             || j == count - 1u || k == count - 1u;

        if (!surface) {
          continue;
        }

        const glm::dvec3 unit(i / last, j / last, k / last);

        samples.push_back((2.0 * unit - glm::dvec3(1.0)) * halfExtents);
      }
    }
  }

  const uint32_t index = addRigidBody(
    bodies,
    field,
    samples,
    mass,
    moment,
    position,
    velocity,
    friction,
    0.0
  );

  bodies.rigids[index].shape = RigidShape::Box;
  bodies.rigids[index].halfExtents = halfExtents;

  return index;
}

uint32_t addCompoundBody(
  BodySet& bodies,
  std::span<const CompoundChild> children,
  const glm::dvec3& position,
  const glm::dquat& rotation,
  const glm::dvec3& velocity,
  double friction,
  double restitution,
  bool anchored
) {
  std::vector<CompoundChild> given(children.begin(), children.end());

  for (CompoundChild& child : given) {
    child.rotation = glm::normalize(child.rotation);
  }

  const std::shared_ptr<const CompoundShape> checked = makeCompoundShape(std::move(given));
  const glm::dquat turn = glm::normalize(rotation);
  RigidBody body;

  body.shape = RigidShape::Compound;
  body.friction = friction;
  body.restitution = restitution;

  if (anchored) {
    body.compound = checked;
    body.position = position;
    body.rotation = turn;
  } else {
    const PrincipalFrame frame = principalFrame(checked->children);

    if (!(frame.mass > 0.0)) {
      throw std::runtime_error("a moving compound body needs a child with mass");
    }

    body.compound = makeCompoundShape(
      childrenInFrame(checked->children, frame.centre, frame.rotation)
    );
    body.position = position + glm::mat3_cast(turn) * frame.centre;
    body.rotation = glm::normalize(turn * frame.rotation);
    body.mass = frame.mass;
    body.moment = frame.moment;
    body.velocity = velocity;
  }

  body.previousVelocity = body.velocity;
  body.initialPosition = body.position;
  body.inertialPosition = body.position;
  body.initialRotation = body.rotation;
  body.inertialRotation = body.rotation;
  body.id = bodies.nextId++;
  bodies.rigids.push_back(std::move(body));

  return static_cast<uint32_t>(bodies.rigids.size() - 1u);
}

uint32_t addStatic(
  BodySet& bodies,
  const field::SampledSdf& field,
  double friction,
  double restitution,
  const glm::dvec3& position,
  const glm::dquat& rotation
) {
  bodies.statics.push_back(
    StaticField{&field, friction, restitution, position, glm::normalize(rotation)}
  );
  bodies.statics.back().id = bodies.nextStaticId++;

  return static_cast<uint32_t>(bodies.statics.size() - 1u);
}

double reachOf(const RigidBody& body) {
  if (body.shape == RigidShape::Box) {
    return std::max(body.radius, glm::length(body.halfExtents));
  }

  if (body.shape == RigidShape::Compound) {
    return std::max(body.radius, body.compound->reach);
  }

  double reach = body.radius;

  for (const glm::dvec3& sample : body.samples) {
    reach = std::max(reach, glm::length(sample));
  }

  return reach;
}

glm::dmat3 worldInertia(const RigidBody& body) {
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);
  glm::dmat3 diagonal(0.0);

  diagonal[0][0] = body.moment.x;
  diagonal[1][1] = body.moment.y;
  diagonal[2][2] = body.moment.z;

  return turn * diagonal * glm::transpose(turn);
}

std::optional<uint32_t> findStatic(const BodySet& bodies, uint32_t id) {
  for (uint32_t index = 0u; index != bodies.statics.size(); ++index) {
    if (bodies.statics[index].id == id) {
      return index;
    }
  }

  return std::nullopt;
}

glm::dvec3 staticLocal(const StaticField& body, const glm::dvec3& world) {
  return glm::conjugate(body.rotation) * (world - body.position);
}

glm::dvec3 staticDirection(const StaticField& body, const glm::dvec3& local) {
  return body.rotation * local;
}

}
