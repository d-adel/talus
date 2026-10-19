#include "ledger.h"

namespace talus::dynamics {

double Energy::total() const {
  return kinetic + gravity + elastic;
}

Energy latticeEnergy(const LatticeBody& body, const glm::dvec3& gravity) {
  const talus::PreparedMesh& mesh = *body.mesh;
  Energy energy;

  for (size_t node = 0u; node != body.position.size(); ++node) {
    const double mass = mesh.nodeMass[node];
    const bool anchored = !mesh.anchored.empty() && mesh.anchored[node] != 0u;

    if (!anchored) {
      energy.kinetic +=
        0.5 * mass * glm::dot(body.velocity[node], body.velocity[node]);
    }

    energy.gravity -= mass * glm::dot(gravity, body.position[node]);
  }

  for (size_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    Corners deformed{};
    Corners rest{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      deformed[corner] = body.position[hex.nodes[corner]];
      rest[corner] = glm::dvec3(mesh.restPositions[hex.nodes[corner]]);
    }

    const glm::dvec3 shift = deformed[0] - rest[0];

    for (glm::dvec3& corner : deformed) {
      corner -= shift;
    }

    energy.elastic += cellEnergy(
      deformed,
      rest,
      glm::dvec3(hex.halfExtents),
      hex.volume,
      body.moduli[cell]
    );
  }

  return energy;
}

Energy rigidEnergy(const RigidBody& body, const glm::dvec3& gravity) {
  Energy energy;

  energy.kinetic =
    0.5 * body.mass * glm::dot(body.velocity, body.velocity)
    + 0.5
        * glm::dot(
          body.angularVelocity,
          worldInertia(body) * body.angularVelocity
        );
  energy.gravity = -body.mass * glm::dot(gravity, body.position);

  return energy;
}

Energy energyOf(const BodySet& bodies, const glm::dvec3& gravity) {
  Energy energy;

  for (const LatticeBody& body : bodies.lattices) {
    const Energy part = latticeEnergy(body, gravity);

    energy.kinetic += part.kinetic;
    energy.gravity += part.gravity;
    energy.elastic += part.elastic;
  }

  for (const RigidBody& body : bodies.rigids) {
    const Energy part = rigidEnergy(body, gravity);

    energy.kinetic += part.kinetic;
    energy.gravity += part.gravity;
  }

  return energy;
}

}
