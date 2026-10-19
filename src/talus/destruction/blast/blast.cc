#include "blast.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/bounds/bounds.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/rays/rays.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>

namespace talus::destruction {

namespace {

constexpr double nearest = 1.0e-9;
constexpr double halfSky = 0.5;

bool finite(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double faceArea(const glm::dvec3& half, int axis) {
  return 4.0 * half[(axis + 1) % 3] * half[(axis + 2) % 3];
}

double childExposure(const dynamics::RigidBody& body, uint32_t index, const glm::dvec3& toward) {
  const dynamics::CompoundChild& child = body.compound->children[index];
  const glm::dmat3 turn = glm::mat3_cast(body.rotation * child.rotation);
  const uint8_t exposed = body.compound->exposed[index];
  double area = 0.0;

  for (int axis = 0; axis != 3; ++axis) {
    const double facing = glm::dot(turn[axis], toward);
    const uint32_t face = 2u * static_cast<uint32_t>(axis) + (facing > 0.0 ? 1u : 0u);

    if (facing != 0.0 && (exposed & (1u << face)) != 0u) {
      area += std::abs(facing) * faceArea(child.half, axis);
    }
  }

  return area;
}

double bodyExposure(const dynamics::RigidBody& body, const glm::dvec3& toward) {
  if (body.shape == dynamics::RigidShape::Sphere) {
    return std::numbers::pi * body.radius * body.radius;
  }

  glm::dvec3 half = body.halfExtents;

  if (body.shape != dynamics::RigidShape::Box) {
    half = glm::dvec3(body.radius);

    for (const glm::dvec3& sample : body.samples) {
      half = glm::max(half, glm::abs(sample));
    }
  }

  const glm::dmat3 turn = glm::mat3_cast(body.rotation);
  double area = 0.0;

  for (int axis = 0; axis != 3; ++axis) {
    area += std::abs(glm::dot(turn[axis], toward)) * faceArea(half, axis);
  }

  return area;
}

}

double blastStrength(double impulsePerArea, double distance, double radius) {
  if (!(impulsePerArea >= 0.0) || !std::isfinite(impulsePerArea) || !(distance > 0.0) || !(radius > distance)
      || !std::isfinite(radius)) {
    throw std::invalid_argument("A blast's strength needs an impulse per area of at least zero at a distance inside its radius");
  }

  return 4.0 * std::numbers::pi * distance * distance * impulsePerArea / (1.0 - distance / radius);
}

double blastFraction(double exposed, double distance, double radius) {
  if (!(distance > nearest) || !(distance < radius) || !(exposed > 0.0)) {
    return 0.0;
  }

  return std::min(exposed / (4.0 * std::numbers::pi * distance * distance), halfSky) * (1.0 - distance / radius);
}

std::vector<BlastImpulse> blastImpulses(const dynamics::World& world, const Blast& blast) {
  if (!finite(blast.centre) || !(blast.strength >= 0.0) || !std::isfinite(blast.strength) || !(blast.radius > 0.0)
      || !std::isfinite(blast.radius)) {
    throw std::invalid_argument("A blast needs a finite centre, a strength of at least zero and a positive radius");
  }

  const dynamics::BodySet& bodies = world.bodies;
  const dynamics::Bounds region{blast.centre - glm::dvec3(blast.radius), blast.centre + glm::dvec3(blast.radius)};
  const std::vector<uint32_t> blockers = dynamics::rayBlockers(bodies, region);
  std::vector<BlastImpulse> impulses;

  const auto reach = [&](uint32_t id, uint32_t child, const glm::dvec3& centre, const auto& exposure) {
    const glm::dvec3 away = centre - blast.centre;
    const double distance = glm::length(away);

    if (!(distance > nearest) || !(distance < blast.radius)) {
      return;
    }

    const double fraction = blastFraction(exposure(-away / distance), distance, blast.radius);

    if (!(fraction > 0.0)
        || dynamics::firstHit(bodies, blockers, blast.centre, centre, dynamics::RaySkip{id, child}).has_value()) {
      return;
    }

    impulses.push_back(BlastImpulse{id, child, (blast.strength * fraction / distance) * away, centre});
  };

  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    const dynamics::RigidBody& body = bodies.rigids[index];

    if (!dynamics::overlaps(dynamics::boundsOf(bodies, dynamics::BodyRef{dynamics::Side::Rigid, index}, 0.0), region)) {
      continue;
    }

    if (body.shape == dynamics::RigidShape::Compound && body.compound != nullptr) {
      for (uint32_t child = 0u; child != body.compound->children.size(); ++child) {
        reach(
          body.id,
          body.compound->children[child].key,
          body.position + body.rotation * body.compound->children[child].centre,
          [&](const glm::dvec3& toward) { return childExposure(body, child, toward); }
        );
      }
    } else if (body.mass > 0.0) {
      reach(body.id, dynamics::noChild, body.position, [&](const glm::dvec3& toward) {
        return bodyExposure(body, toward);
      });
    }
  }

  return impulses;
}

}
