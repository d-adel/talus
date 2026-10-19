#include "projection.h"

#include <glm/glm.hpp>

#include <algorithm>

namespace talus::dynamics {

double speedOf(const BodySet& bodies, const BodyRef& ref) {
  double fastest = 0.0;

  if (ref.side == Side::Lattice) {
    for (const glm::dvec3& velocity : bodies.lattices[ref.index].velocity) {
      fastest = std::max(fastest, glm::length(velocity));
    }
  } else if (ref.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[ref.index];

    fastest = glm::length(body.velocity)
              + glm::length(body.angularVelocity) * reachOf(body);
  }

  return fastest;
}

}
