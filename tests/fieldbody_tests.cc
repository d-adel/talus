#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/field/constants.h>
#include <talus/dynamics/fieldbody/fieldbody.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace {

talus::field::Primitive onLane(talus::field::Primitive primitive, uint32_t lane) {
  primitive.shapeConfig.y = lane;

  return primitive;
}

talus::field::SampledSdf baked(
  const std::vector<talus::field::Primitive>& primitives,
  const glm::vec3& low,
  const glm::vec3& high,
  float voxel
) {
  return talus::field::bake(
    primitives,
    low,
    high,
    glm::uvec3(glm::round((high - low) / voxel)) + glm::uvec3(1u)
  );
}

std::vector<talus::PhysicalMaterial> lanes() {
  std::vector<talus::PhysicalMaterial> materials(3);

  materials[1].density = 500.0f;
  materials[1].friction = 0.7f;
  materials[1].restitution = 0.2f;
  materials[2].density = 150.0f;

  return materials;
}

}

TEST_CASE(
  "a field of one box becomes an exact box with its material's mass",
  "[dynamics][fieldbody]"
) {
  const std::vector<talus::field::Primitive> primitives{
    onLane(fixture::box(glm::vec3(0.3f, 0.0f, 0.0f), glm::vec3(0.1f, 0.05f, 0.2f)), 1u)
  };
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(0.1f, -0.1f, -0.3f), glm::vec3(0.5f, 0.1f, 0.3f), 0.01f);
  const auto materials = lanes();
  const talus::dynamics::RigidBody body =
    talus::dynamics::rigidFromField(field, primitives, materials, 64u);

  CHECK(body.shape == talus::dynamics::RigidShape::Box);
  CHECK(std::abs(body.mass - 500.0 * 0.2 * 0.1 * 0.4) <= 1.0e-6);
  CHECK(glm::length(body.halfExtents - glm::dvec3(0.1, 0.05, 0.2)) <= 1.0e-6);
  CHECK(glm::length(body.fieldOrigin - glm::dvec3(0.3, 0.0, 0.0)) <= 1.0e-6);
  CHECK(std::abs(body.friction - 0.7) <= 1.0e-6);
  CHECK(std::abs(body.restitution - 0.2) <= 1.0e-6);
  CHECK(body.field == &field);
}

TEST_CASE(
  "a field of one turned box keeps the box's turn",
  "[dynamics][fieldbody]"
) {
  talus::field::Primitive box = fixture::box(glm::vec3(0.0f), glm::vec3(0.1f, 0.05f, 0.2f));
  const glm::mat4 model = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

  box.inverseModel = glm::inverse(model);
  talus::field::updateBounds(box);

  const std::vector<talus::field::Primitive> primitives{onLane(box, 1u)};
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(-0.3f), glm::vec3(0.3f), 0.01f);
  const auto materials = lanes();
  const talus::dynamics::RigidBody body =
    talus::dynamics::rigidFromField(field, primitives, materials, 64u);
  const glm::dquat expected(glm::quat_cast(glm::mat3(model)));

  REQUIRE(body.shape == talus::dynamics::RigidShape::Box);
  CHECK(std::abs(std::abs(glm::dot(body.fieldRotation, expected)) - 1.0) <= 1.0e-6);
}

TEST_CASE(
  "a field of one sphere becomes an exact sphere",
  "[dynamics][fieldbody]"
) {
  const std::vector<talus::field::Primitive> primitives{fixture::sphere(glm::vec3(0.0f), 0.1f)};
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(-0.14f), glm::vec3(0.14f), 0.01f);
  const auto materials = lanes();
  const talus::dynamics::RigidBody body =
    talus::dynamics::rigidFromField(field, primitives, materials, 64u);
  const double mass = 500.0 * 4.0 / 3.0 * std::numbers::pi * 0.1 * 0.1 * 0.1;

  CHECK(body.shape == talus::dynamics::RigidShape::Sphere);
  CHECK(std::abs(body.radius - 0.1) <= 1.0e-6);
  CHECK(std::abs(body.mass - mass) <= 1.0e-6 * mass);
  CHECK(std::abs(body.moment.x - 0.4 * mass * 0.01) <= 1.0e-6 * mass);
}

TEST_CASE(
  "a box with a hole becomes a sampled body with the hole's mass missing",
  "[dynamics][fieldbody]"
) {
  const float voxel = 0.004f;
  const std::vector<talus::field::Primitive> primitives{
    fixture::box(glm::vec3(0.0f), glm::vec3(0.1f)),
    fixture::sphere(glm::vec3(0.0f), 0.05f, talus::field::FIELD_OP_SUBTRACTION)
  };
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(-0.12f), glm::vec3(0.12f), voxel);
  const auto materials = lanes();
  const talus::dynamics::RigidBody body =
    talus::dynamics::rigidFromField(field, primitives, materials, 64u);
  const double hole = 4.0 / 3.0 * std::numbers::pi * 0.05 * 0.05 * 0.05;
  const double volume = 0.2 * 0.2 * 0.2 - hole;
  const double area = 6.0 * 0.2 * 0.2 + 4.0 * std::numbers::pi * 0.05 * 0.05;
  const double shell = area * 0.5 * static_cast<double>(voxel);

  REQUIRE(body.shape == talus::dynamics::RigidShape::Sampled);
  CHECK(std::abs(body.mass - 500.0 * volume) <= 500.0 * shell);
  CHECK(glm::length(body.fieldOrigin) <= 0.5 * static_cast<double>(voxel));
  REQUIRE(!body.samples.empty());
  CHECK(body.samples.size() <= 64u);

  for (const glm::dvec3& sample : body.samples) {
    const glm::dvec3 point = body.fieldOrigin + body.fieldRotation * sample;
    const float distance =
      talus::field::probe<float>(field, glm::vec3(point)).distance;

    CHECK(std::abs(distance) <= voxel);
  }
}

TEST_CASE(
  "a sampled body takes its mass from its material's density",
  "[dynamics][fieldbody]"
) {
  const std::vector<talus::field::Primitive> primitives{
    onLane(fixture::box(glm::vec3(0.0f), glm::vec3(0.1f)), 2u),
    onLane(fixture::sphere(glm::vec3(0.0f), 0.05f, talus::field::FIELD_OP_SUBTRACTION), 2u)
  };
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(-0.12f), glm::vec3(0.12f), 0.004f);
  const auto materials = lanes();
  const talus::dynamics::RigidBody heavy = talus::dynamics::rigidFromField(
    field,
    primitives,
    std::vector<talus::PhysicalMaterial>(3),
    64u
  );
  const talus::dynamics::RigidBody light =
    talus::dynamics::rigidFromField(field, primitives, materials, 64u);

  REQUIRE(heavy.mass > 0.0);
  CHECK(std::abs(light.mass / heavy.mass - 0.15) <= 1.0e-6);
}

TEST_CASE(
  "a field with nothing solid gives a body without mass",
  "[dynamics][fieldbody]"
) {
  const std::vector<talus::field::Primitive> primitives{
    fixture::box(glm::vec3(0.0f), glm::vec3(0.1f)),
    fixture::box(glm::vec3(0.0f), glm::vec3(0.2f), talus::field::FIELD_OP_SUBTRACTION)
  };
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(-0.3f), glm::vec3(0.3f), 0.02f);
  const auto materials = lanes();

  CHECK(talus::dynamics::rigidFromField(field, primitives, materials, 64u).mass == 0.0);
}

TEST_CASE(
  "a field asked for no samples still carries its material",
  "[dynamics][fieldbody]"
) {
  const std::vector<talus::field::Primitive> primitives{
    onLane(fixture::box(glm::vec3(0.0f), glm::vec3(0.1f)), 1u),
    onLane(fixture::box(glm::vec3(0.1f, 0.0f, 0.0f), glm::vec3(0.1f)), 1u)
  };
  const talus::field::SampledSdf field =
    baked(primitives, glm::vec3(-0.2f), glm::vec3(0.3f), 0.01f);
  const talus::dynamics::RigidBody body =
    talus::dynamics::rigidFromField(field, primitives, lanes(), 0u);

  CHECK(body.samples.empty());
  CHECK(body.mass > 0.0);
  CHECK(std::abs(body.friction - 0.7) <= 1.0e-6);
}

TEST_CASE(
  "the outside gap bound never exceeds the probed gap",
  "[dynamics][fieldbody]"
) {
  const glm::vec3 half(0.1f, 0.05f, 0.08f);
  const std::vector<talus::field::SampledSdf> fields{
    baked(
      {fixture::box(glm::vec3(0.0f), half)},
      -half - 0.03f,
      half + 0.03f,
      0.01f
    ),
    baked(
      {fixture::box(glm::vec3(0.0f), half)},
      -half + 0.02f,
      half - 0.02f,
      0.01f
    ),
    baked(
      {fixture::sphere(glm::vec3(0.0f), 0.1f)},
      glm::vec3(-0.104f),
      glm::vec3(0.104f),
      0.008f
    )
  };

  for (size_t index = 0u; index != fields.size(); ++index) {
    const talus::field::SampledSdf& field = fields[index];
    const glm::dvec3 low(field.origin);
    const glm::dvec3 high = low
                            + glm::dvec3(field.voxelSize)
                                * glm::dvec3(field.resolution - glm::uvec3(1u));
    uint32_t outside = 0u;
    uint32_t bounded = 0u;
    double worst = std::numeric_limits<double>::infinity();

    for (int x = -12; x <= 12; ++x) {
      for (int y = -12; y <= 12; ++y) {
        for (int z = -12; z <= 12; ++z) {
          const glm::dvec3 at = glm::dvec3(x, y, z) / 12.0;
          const glm::dvec3 point =
            0.5 * (low + high) + at * (0.5 * (high - low) + glm::dvec3(0.12));
          const talus::field::FieldProbeOf<double> probed =
            talus::field::probe<double>(field, point);
          const double gap = probed.distance / glm::length(probed.gradient);
          const double bound = talus::field::outsideGapBound(field, point);
          const bool beyond = glm::any(glm::lessThan(point, low))
                              || glm::any(glm::greaterThan(point, high));

          outside += beyond ? 1u : 0u;
          bounded += bound >= 0.02 ? 1u : 0u;
          worst = std::min(worst, gap - bound);

          if (!beyond) {
            CHECK(bound == -std::numeric_limits<double>::infinity());
          }
        }
      }
    }

    CAPTURE(index, outside, bounded, worst);
    CHECK(worst >= 0.0);
    CHECK(outside > 10000u);
    CHECK(bounded > outside / 4u);
  }
}
