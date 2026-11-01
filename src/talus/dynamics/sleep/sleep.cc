#include "sleep.h"

#include <talus/dynamics/compound/compound.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace talus::dynamics {

namespace {

double extentOf(const RigidBody& body) {
  double extent = body.radius;

  if (body.shape == RigidShape::Compound) {
    extent = std::max(extent, body.compound->reach);
  }

  for (const glm::dvec3& sample : body.samples) {
    extent = std::max(extent, glm::length(sample));
  }

  return extent;
}

double& sleepTimeOf(BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Rigid ? bodies.rigids[ref.index].sleepTime
                                 : bodies.lattices[ref.index].sleepTime;
}

void putToSleep(BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Rigid) {
    RigidBody& body = bodies.rigids[ref.index];

    if (!body.asleep) {
      bodies.stirred.push_back(ref.index);
    }

    body.asleep = true;
    body.velocity = glm::dvec3(0.0);
    body.angularVelocity = glm::dvec3(0.0);
    body.previousVelocity = glm::dvec3(0.0);

    return;
  }

  LatticeBody& body = bodies.lattices[ref.index];

  body.asleep = true;
  std::fill(body.velocity.begin(), body.velocity.end(), glm::dvec3(0.0));
  std::fill(
    body.previousVelocity.begin(),
    body.previousVelocity.end(),
    glm::dvec3(0.0)
  );
}

}

bool resting(const BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Static) {
    return true;
  }

  return ref.side == Side::Rigid ? bodies.rigids[ref.index].asleep
                                 : bodies.lattices[ref.index].asleep;
}

bool islandAsleep(const BodySet& bodies, const Island& island) {
  return !island.bodies.empty()
         && std::all_of(
           island.bodies.begin(),
           island.bodies.end(),
           [&bodies](const BodyRef& ref) { return resting(bodies, ref); }
         );
}

void wakeBody(BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Rigid) {
    RigidBody& body = bodies.rigids[ref.index];

    if (body.asleep) {
      bodies.stirred.push_back(ref.index);
    }

    body.asleep = false;
    body.sleepTime = 0.0;
  } else if (ref.side == Side::Lattice) {
    bodies.lattices[ref.index].asleep = false;
    bodies.lattices[ref.index].sleepTime = 0.0;
  }
}

void wakeAll(BodySet& bodies) {
  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    wakeBody(bodies, BodyRef{Side::Rigid, index});
  }

  for (LatticeBody& body : bodies.lattices) {
    body.asleep = false;
    body.sleepTime = 0.0;
  }
}

double sleepSpeed(const RigidBody& body, const RigidPose& start, double dt) {
  const double extent = extentOf(body);
  const glm::dquat turn = body.rotation * glm::conjugate(start.rotation);
  const double moved =
    glm::length(body.position - start.position)
    + 2.0 * glm::length(glm::dvec3(turn.x, turn.y, turn.z)) * extent;
  const double moving =
    glm::length(body.velocity) + glm::length(body.angularVelocity) * extent;
  const double drift = 0.5 * moved / dt;

  return moving < drift || std::isnan(drift) ? drift : moving;
}

double sleepSpeed(const LatticeBody& body, double dt) {
  double fastest = 0.0;

  for (size_t node = 0u; node != body.position.size(); ++node) {
    fastest = std::max(
      {fastest,
       glm::length(body.velocity[node]),
       0.5 * glm::length(body.position[node] - body.initial[node]) / dt}
    );
  }

  return fastest;
}

void settleIslands(
  BodySet& bodies,
  ManifoldCache& cache,
  IslandStore& store,
  std::span<const Island> islands,
  std::span<const RigidPose> start,
  const IntegrationParameters& parameters
) {
  const auto idOf = [&](const BodyRef& ref) {
    return ref.side == Side::Rigid ? bodies.rigids[ref.index].id
                                   : bodies.lattices[ref.index].id;
  };
  uint32_t splitIsland = noIsland;
  uint32_t splitBody = noBody;
  double splitTime = 0.0;

  for (const Island& island : islands) {
    if (island.bodies.empty() || island.asleep) {
      continue;
    }

    for (const BodyRef& ref : island.bodies) {
      const double speed =
        ref.side == Side::Rigid
          ? sleepSpeed(bodies.rigids[ref.index], start[ref.index], parameters.dt)
          : sleepSpeed(bodies.lattices[ref.index], parameters.dt);
      double& time = sleepTimeOf(bodies, ref);

      time = speed <= parameters.sleepThreshold ? time + parameters.dt : 0.0;

      const uint32_t candidate = store.islandOf(idOf(ref));

      if (time >= parameters.timeToSleep && store.removed(candidate) > 0u
          && (time > splitTime || (time == splitTime && candidate > splitIsland))) {
        splitIsland = candidate;
        splitBody = idOf(ref);
        splitTime = time;
      }
    }
  }

  store.requestSplit(splitBody);

  for (const Island& island : islands) {
    if (island.bodies.empty() || island.asleep) {
      continue;
    }

    for (const BodyRef& ref : island.bodies) {
      const uint32_t candidate = store.islandOf(idOf(ref));

      if (!store.visit(candidate)) {
        continue;
      }

      const std::span<const uint32_t> members = store.members(candidate);

      if (store.removed(candidate) > 0u && members.size() > 1u) {
        continue;
      }

      const bool sleepy = std::all_of(members.begin(), members.end(), [&](uint32_t id) {
        return sleepTimeOf(bodies, store.refOf(id)) >= parameters.timeToSleep;
      });

      if (!sleepy) {
        continue;
      }

      for (const uint32_t id : members) {
        putToSleep(bodies, store.refOf(id));
      }

      for (const uint32_t id : members) {
        if (store.refOf(id).side == Side::Rigid) {
          cache.sleepBody(bodies, id);
        }
      }
    }
  }
}

}
