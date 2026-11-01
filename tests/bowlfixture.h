#pragma once

#include "fieldprimitives.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/fieldbody/fieldbody.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace bowlfixture {

inline constexpr glm::vec3 half{0.4f, 0.2f, 0.4f};
inline constexpr float floorGap = 0.05f;
inline constexpr float voxel = 0.01f;

inline glm::vec3 cavity(float radius) {
  return glm::vec3(0.0f, radius - half.y + floorGap, 0.0f);
}

inline talus::field::SampledSdf field(float radius) {
  const glm::vec3 reach = half + glm::vec3(2.0f * voxel);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), half),
      fixture::sphere(
        cavity(radius),
        radius,
        talus::field::FIELD_OP_SUBTRACTION
      )
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / voxel)) + glm::uvec3(1u)
  );
}

inline uint32_t addBowl(
  talus::dynamics::BodySet& bodies,
  const talus::field::SampledSdf& bowl,
  const glm::dvec3& at,
  double mass,
  double friction
) {
  const talus::dynamics::RigidBody shape = talus::dynamics::rigidFromField(
    bowl,
    {},
    std::vector<talus::PhysicalMaterial>(1),
    4000u
  );
  const uint32_t index = talus::dynamics::addRigidBody(
    bodies,
    bowl,
    shape.samples,
    mass,
    shape.moment * (mass / shape.mass),
    at + shape.fieldOrigin,
    glm::dvec3(0.0),
    friction,
    0.0
  );
  talus::dynamics::RigidBody& body = bodies.rigids[index];

  body.rotation = shape.fieldRotation;
  body.initialRotation = shape.fieldRotation;
  body.inertialRotation = shape.fieldRotation;
  body.fieldOrigin = shape.fieldOrigin;
  body.fieldRotation = shape.fieldRotation;

  return index;
}

}
