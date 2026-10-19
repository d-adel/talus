#include "destructionfixtures.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

namespace destructionfixture {

namespace {

constexpr uint32_t NO_INDEX = 0xFFFFFFFFu;
constexpr double REFERENCE_SPEED = 1.0;
constexpr double REFERENCE_LEVER = 1.0;
constexpr uint64_t HASH_START = 1469598103934665603ull;
constexpr uint64_t HASH_PRIME = 1099511628211ull;

uint64_t mixWord(uint64_t hash, uint64_t word) {
  for (int byte = 0; byte != 8; ++byte) {
    hash ^= (word >> (8 * byte)) & 0xFFu;
    hash *= HASH_PRIME;
  }

  return hash;
}

uint64_t mixDouble(uint64_t hash, double value) {
  uint64_t bits = 0u;

  std::memcpy(&bits, &value, sizeof(bits));

  return mixWord(hash, bits);
}

}

Motion motionOf(const talus::dynamics::World& world) {
  Motion motion;

  for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
    if (!(body.mass > 0.0)) {
      continue;
    }

    const glm::dmat3 inertia = talus::dynamics::worldInertia(body);

    motion.momentum += body.mass * body.velocity;
    motion.angular += body.mass * glm::cross(body.position, body.velocity) + inertia * body.angularVelocity;
    motion.energy += 0.5 * body.mass * glm::dot(body.velocity, body.velocity)
                     + 0.5 * glm::dot(body.angularVelocity, inertia * body.angularVelocity);
    motion.mass += body.mass;
  }

  return motion;
}

double linearScale(const Motion& motion) {
  return std::max(glm::length(motion.momentum), motion.mass * REFERENCE_SPEED);
}

double angularScale(const Motion& motion) {
  return std::max(glm::length(motion.angular), motion.mass * REFERENCE_SPEED * REFERENCE_LEVER);
}

double energyScale(const Motion& motion) {
  return std::max(motion.energy, 0.5 * motion.mass * REFERENCE_SPEED * REFERENCE_SPEED);
}

std::string mismatch(
  const talus::destruction::DestructionWorld& destruction,
  const talus::dynamics::World& world
) {
  std::vector<uint32_t> index(world.bodies.nextId, NO_INDEX);
  std::vector<size_t> held(world.bodies.nextId, 0u);

  for (uint32_t at = 0u; at != world.bodies.rigids.size(); ++at) {
    index[world.bodies.rigids[at].id] = at;
  }

  for (uint32_t building = 0u; building != destruction.buildings(); ++building) {
    const uint32_t chunks = destruction.chunkCount(building);

    for (uint32_t chunk = 0u; chunk != chunks; ++chunk) {
      const std::optional<uint32_t> body = destruction.bodyOf(building, chunk);

      if (!body.has_value()) {
        continue;
      }

      const std::string where = "building " + std::to_string(building) + " chunk " + std::to_string(chunk);

      if (*body >= index.size() || index[*body] == NO_INDEX) {
        return where + " maps to body " + std::to_string(*body) + ", which the world does not hold";
      }

      const talus::dynamics::RigidBody& rigid = world.bodies.rigids[index[*body]];

      if (rigid.shape != talus::dynamics::RigidShape::Compound || rigid.compound == nullptr) {
        return where + " maps to body " + std::to_string(*body) + ", which is not a compound";
      }

      if (!talus::dynamics::findChild(*rigid.compound, destruction.firstKey(building) + chunk).has_value()) {
        return where + " maps to body " + std::to_string(*body) + ", which does not hold its key";
      }

      ++held[*body];
    }
  }

  for (const uint32_t body : destruction.bodies()) {
    if (body >= index.size() || index[body] == NO_INDEX) {
      return "destruction body " + std::to_string(body) + " is missing from the world";
    }

    const size_t children = world.bodies.rigids[index[body]].compound->children.size();

    if (children != held[body]) {
      return "body " + std::to_string(body) + " holds " + std::to_string(children) + " children, but "
             + std::to_string(held[body]) + " chunks map to it";
    }
  }

  return {};
}

double heldEnergy(
  const talus::destruction::DestructionWorld& destruction,
  const talus::dynamics::World& world
) {
  const glm::dvec3 gravity = world.parameters.gravity;
  std::vector<uint32_t> index(world.bodies.nextId, NO_INDEX);
  double energy = 0.0;

  for (uint32_t at = 0u; at != world.bodies.rigids.size(); ++at) {
    const talus::dynamics::RigidBody& body = world.bodies.rigids[at];

    index[body.id] = at;

    if (!(body.mass > 0.0)) {
      continue;
    }

    energy += 0.5 * body.mass * glm::dot(body.velocity, body.velocity)
              + 0.5 * glm::dot(body.angularVelocity, talus::dynamics::worldInertia(body) * body.angularVelocity)
              - body.mass * glm::dot(gravity, body.position);
  }

  for (uint32_t building = 0u; building != destruction.buildings(); ++building) {
    for (uint32_t chunk = 0u; chunk != destruction.chunkCount(building); ++chunk) {
      const std::optional<uint32_t> held = destruction.bodyOf(building, chunk);

      if (!held.has_value() || *held >= index.size() || index[*held] == NO_INDEX) {
        continue;
      }

      const talus::dynamics::RigidBody& body = world.bodies.rigids[index[*held]];

      if (body.mass > 0.0) {
        continue;
      }

      const talus::destruction::Chunk part = destruction.chunkOf(building, chunk);

      energy -= part.mass * glm::dot(gravity, body.position + body.rotation * part.centre);
    }
  }

  return energy;
}

uint64_t stateHash(
  const talus::destruction::DestructionWorld& destruction,
  const talus::dynamics::World& world
) {
  uint64_t hash = HASH_START;

  for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
    hash = mixWord(hash, body.id);
    hash = mixDouble(hash, body.mass);
    hash = mixDouble(hash, body.frozenMass);
    hash = mixWord(hash, body.asleep ? 1u : 0u);
    hash = mixDouble(hash, body.sleepTime);

    for (int axis = 0; axis != 3; ++axis) {
      hash = mixDouble(hash, body.position[axis]);
      hash = mixDouble(hash, body.velocity[axis]);
      hash = mixDouble(hash, body.angularVelocity[axis]);
      hash = mixDouble(hash, body.moment[axis]);
      hash = mixDouble(hash, body.frozenMoment[axis]);
    }

    for (int axis = 0; axis != 4; ++axis) {
      hash = mixDouble(hash, body.rotation[axis]);
    }
  }

  for (uint32_t building = 0u; building != destruction.buildings(); ++building) {
    for (uint32_t chunk = 0u; chunk != destruction.chunkCount(building); ++chunk) {
      hash = mixWord(hash, destruction.bodyOf(building, chunk).value_or(NO_INDEX));
    }

    for (uint32_t bond = 0u; bond != destruction.bondCount(building); ++bond) {
      hash = mixWord(hash, static_cast<uint64_t>(destruction.bondState(building, bond)));
      hash = mixDouble(hash, destruction.bondHealth(building, bond));
    }
  }

  return hash;
}

bool sameOutcome(const talus::destruction::DestructionReport& a, const talus::destruction::DestructionReport& b) {
  return a.step == b.step && a.removed == b.removed && a.gathered == b.gathered && a.refused == b.refused && a.loads == b.loads
         && a.solved == b.solved && a.tolerated == b.tolerated
         && a.rounds == b.rounds && a.factorizations == b.factorizations && a.solvedChunks == b.solvedChunks
         && a.factorizedChunks == b.factorizedChunks && a.pending == b.pending && a.blasted == b.blasted
         && a.driven == b.driven && a.fired == b.fired && a.broken == b.broken && a.created == b.created
         && a.fractured == b.fractured && a.frozen == b.frozen && a.thawed == b.thawed
         && a.removedFrozen == b.removedFrozen && a.discarded == b.discarded && a.guarded == b.guarded && a.deferredFractures == b.deferredFractures
         && a.maxPendingAge == b.maxPendingAge && a.maxBreakDelay == b.maxBreakDelay
         && a.casesQueued == b.casesQueued && a.casesReplaced == b.casesReplaced && a.casesMerged == b.casesMerged
         && a.casesStarted == b.casesStarted && a.deferred == b.deferred && a.overRounds == b.overRounds;
}

}
