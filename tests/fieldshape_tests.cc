#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "fieldprimitives.h"

#include <talus/core/config/config.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/deform/deform.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/physics/fieldshape/fieldshape.h>
#include <talus/physics/impact/impact.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/physics/layers/layers.h>
#include <talus/physics/physicsworld/physicsworld.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/ManifoldBetweenTwoFaces.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/component_wise.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using talus::field::Primitive;
using talus::field::SampledSdf;
using talus::physics::FieldShape;
using talus::physics::JoltLibrary;

namespace {

constexpr float DENSITY = 1000.0f;

SampledSdf bakeSphere(
  float radius,
  uint32_t resolution,
  const glm::vec3& centre = glm::vec3(0.0f)
) {
  const float reach = radius * 1.5f;

  return talus::field::bake(
    std::vector<Primitive>{fixture::sphere(centre, radius)},
    centre - glm::vec3(reach),
    centre + glm::vec3(reach),
    glm::uvec3(resolution)
  );
}

float sphereMass(float radius) {
  return DENSITY * 4.0f / 3.0f * glm::pi<float>() * radius * radius * radius;
}

}

TEST_CASE("the local bounds are the solid, not the grid", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 65u);
  const FieldShape shape(field);

  const JPH::AABox bounds = shape.GetLocalBounds();

  for (int axis = 0; axis != 3; ++axis) {
    REQUIRE(bounds.mMin[axis] <= -1.0f);
    REQUIRE(bounds.mMax[axis] >= 1.0f);

    REQUIRE(bounds.mMin[axis] > -1.5f);
    REQUIRE(bounds.mMax[axis] < 1.5f);
  }
}

TEST_CASE(
  "a solid away from the origin says where its mass is",
  "[fieldshape]"
) {
  JoltLibrary library;

  const glm::vec3 offset(0.0f, 3.0f, 0.0f);

  const SampledSdf field = bakeSphere(1.0f, 65u, offset);
  const FieldShape shape(field);

  const JPH::Vec3 centre = shape.GetCenterOfMass();

  REQUIRE_THAT(centre.GetX(), WithinAbs(offset.x, 0.01f));
  REQUIRE_THAT(centre.GetY(), WithinAbs(offset.y, 0.01f));
  REQUIRE_THAT(centre.GetZ(), WithinAbs(offset.z, 0.01f));
}

TEST_CASE("an offset solid is bounded about its own centre", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 65u, glm::vec3(0.0f, 3.0f, 0.0f));
  const FieldShape shape(field);

  const JPH::AABox bounds = shape.GetLocalBounds();

  for (int axis = 0; axis != 3; ++axis) {
    REQUIRE(bounds.mMin[axis] <= -1.0f);
    REQUIRE(bounds.mMax[axis] >= 1.0f);
    REQUIRE(bounds.mMin[axis] > -1.5f);
    REQUIRE(bounds.mMax[axis] < 1.5f);
  }
}

TEST_CASE(
  "an offset solid resists turning like its own shape",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf centred = bakeSphere(1.0f, 65u);
  const SampledSdf offset = bakeSphere(1.0f, 65u, glm::vec3(0.0f, 3.0f, 0.0f));

  const JPH::MassProperties here = FieldShape(centred).GetMassProperties();
  const JPH::MassProperties there = FieldShape(offset).GetMassProperties();

  REQUIRE_THAT(there.mMass, WithinRel(here.mMass, 0.02f));

  for (int axis = 0; axis != 3; ++axis) {
    REQUIRE_THAT(
      there.mInertia(axis, axis),
      WithinRel(here.mInertia(axis, axis), 0.02f)
    );
  }
}

TEST_CASE("an offset solid is asked about in the right space", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 65u, glm::vec3(0.0f, 3.0f, 0.0f));
  const FieldShape shape(field);

  JPH::AllHitCollisionCollector<JPH::CollidePointCollector> inside;
  shape.CollidePoint(JPH::Vec3::sZero(), JPH::SubShapeIDCreator(), inside, {});

  REQUIRE(inside.mHits.size() == 1u);

  JPH::AllHitCollisionCollector<JPH::CollidePointCollector> outside;
  shape.CollidePoint(
    JPH::Vec3(0.0f, 3.0f, 0.0f),
    JPH::SubShapeIDCreator(),
    outside,
    {}
  );

  REQUIRE(outside.mHits.empty());
}

TEST_CASE("an empty grid answers for its whole box", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = talus::field::bake(
    std::vector<Primitive>{fixture::sphere(glm::vec3(10.0f), 0.5f)},
    glm::vec3(-1.0f),
    glm::vec3(1.0f),
    glm::uvec3(17u)
  );

  const FieldShape shape(field);
  const JPH::AABox bounds = shape.GetLocalBounds();

  for (int axis = 0; axis != 3; ++axis) {
    REQUIRE_THAT(bounds.mMin[axis], WithinAbs(field.origin[axis], 1e-6f));
    REQUIRE_THAT(
      bounds.mMax[axis],
      WithinAbs(
        field.origin[axis]
          + field.voxelSize[axis] * float(field.resolution[axis] - 1u),
        1e-6f
      )
    );
  }
}

TEST_CASE(
  "the seeds are on the surface, and every surface cell has one near it",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 65u);
  const FieldShape shape(field);

  const glm::uvec3 counts = talus::field::brickCounts(field);
  const uint32_t bricks = counts.x * counts.y * counts.z;

  uint32_t holding = 0u;

  for (uint32_t brick = 0u; brick != bricks; ++brick) {
    if (talus::field::brickHoldsSurface(field, brick)) {
      ++holding;
    }
  }

  REQUIRE(holding > 0u);
  REQUIRE(shape.surfaceBricks().size() == holding);

  uint32_t next = 0u;
  float previous = -1.0f;

  for (const talus::physics::SurfaceBrick& surface : shape.surfaceBricks()) {
    REQUIRE(talus::field::brickHoldsSurface(field, surface.brick));
    REQUIRE(float(surface.brick) > previous);
    REQUIRE(surface.begin == next);
    REQUIRE(surface.end > surface.begin);

    previous = float(surface.brick);
    next = surface.end;

    for (uint32_t index = surface.begin; index != surface.end; ++index) {
      const talus::physics::FieldSeed& seed = shape.seeds()[index];

      REQUIRE(seed.brick == surface.brick);
      REQUIRE_THAT(
        talus::field::distance(field, seed.point),
        WithinAbs(0.0f, 1.0e-4f)
      );
    }
  }

  REQUIRE(next == shape.seeds().size());

  const glm::uvec3 cells = field.resolution - glm::uvec3(1u);
  const uint32_t alongY = field.resolution.x;
  const uint32_t alongZ = field.resolution.x * field.resolution.y;
  const float reach = glm::length(field.voxelSize);

  uint32_t surfaceCells = 0u;
  uint32_t uncovered = 0u;

  for (uint32_t z = 0u; z != cells.z; ++z) {
    for (uint32_t y = 0u; y != cells.y; ++y) {
      for (uint32_t x = 0u; x != cells.x; ++x) {
        const uint32_t lowest = x + alongY * y + alongZ * z;

        float low = field.distances[lowest];
        float high = low;

        for (uint32_t corner = 1u; corner != 8u; ++corner) {
          const float value =
            field.distances
              [lowest + (corner & 1u) + alongY * ((corner >> 1u) & 1u)
               + alongZ * ((corner >> 2u) & 1u)];

          low = std::min(low, value);
          high = std::max(high, value);
        }

        if (!(low < 0.0f && high > 0.0f)) {
          continue;
        }

        ++surfaceCells;

        const glm::vec3 centre =
          field.origin
          + field.voxelSize * (glm::vec3(x, y, z) + glm::vec3(0.5f));

        const glm::uvec3 brick =
          glm::uvec3(x, y, z) / glm::uvec3(talus::field::BRICK_CELLS);
        const uint32_t index =
          brick.x + counts.x * brick.y + counts.x * counts.y * brick.z;

        const auto surface = std::find_if(
          shape.surfaceBricks().begin(),
          shape.surfaceBricks().end(),
          [&](const talus::physics::SurfaceBrick& held) {
            return held.brick == index;
          }
        );

        const bool near = surface != shape.surfaceBricks().end()
                          && std::any_of(
                            shape.seeds().begin() + surface->begin,
                            shape.seeds().begin() + surface->end,
                            [&](const talus::physics::FieldSeed& seed) {
                              return glm::distance(seed.point, centre) <= reach;
                            }
                          );

        uncovered += near ? 0u : 1u;
      }
    }
  }

  REQUIRE(surfaceCells > 0u);
  REQUIRE(uncovered == 0u);
}

TEST_CASE("the inner radius is the deepest point inside", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 33u);
  const FieldShape shape(field);

  REQUIRE_THAT(shape.GetInnerRadius(), WithinAbs(1.0f, field.voxelSize.x));
}

TEST_CASE("an empty grid still reports a usable inner radius", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = talus::field::bake(
    std::vector<Primitive>{fixture::sphere(glm::vec3(10.0f), 0.5f)},
    glm::vec3(-1.0f),
    glm::vec3(1.0f),
    glm::uvec3(17u)
  );

  const FieldShape shape(field);

  REQUIRE(shape.GetInnerRadius() > 0.0f);
}

TEST_CASE("the mass is the solid's, not the grid box's", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 65u);
  const FieldShape shape(field);

  REQUIRE_THAT(
    shape.GetMassProperties().mMass,
    WithinRel(sphereMass(1.0f), 0.05f)
  );
}

TEST_CASE("the inertia is the solid's, about its own axes", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 65u);
  const FieldShape shape(field);

  const JPH::MassProperties properties = shape.GetMassProperties();
  const float expected = 0.4f * properties.mMass;

  for (int axis = 0; axis != 3; ++axis) {
    REQUIRE_THAT(properties.mInertia(axis, axis), WithinRel(expected, 0.05f));
  }

  REQUIRE_THAT(properties.mInertia(0, 1), WithinAbs(0.0f, expected * 0.01f));
  REQUIRE_THAT(properties.mInertia(0, 2), WithinAbs(0.0f, expected * 0.01f));
  REQUIRE_THAT(properties.mInertia(1, 2), WithinAbs(0.0f, expected * 0.01f));
}

TEST_CASE(
  "each shape keeps its own mass, not the last one asked",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf small = bakeSphere(0.5f, 65u);
  const SampledSdf large = bakeSphere(1.0f, 65u);

  const FieldShape smallShape(small);
  const FieldShape largeShape(large);

  const float smallMass = smallShape.GetMassProperties().mMass;
  const float largeMass = largeShape.GetMassProperties().mMass;

  REQUIRE_THAT(smallMass, WithinRel(sphereMass(0.5f), 0.05f));
  REQUIRE_THAT(largeMass, WithinRel(sphereMass(1.0f), 0.05f));

  REQUIRE(largeShape.GetMassProperties().mMass == largeMass);
  REQUIRE(smallShape.GetMassProperties().mMass == smallMass);
}

TEST_CASE("a solid box weighs and turns like a solid box", "[fieldshape]") {
  JoltLibrary library;

  const glm::vec3 half(0.6f, 0.4f, 0.3f);

  Primitive box{};
  box.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f)));
  box.shape = glm::vec4(half, 0.0f);
  box.shapeConfig = glm::uvec4(1u, 1u, talus::core::FIELD_OP_UNION, 0u);
  talus::field::updateBounds(box);

  const SampledSdf field = talus::field::bake(
    std::vector<Primitive>{box},
    -half - glm::vec3(0.25f),
    half + glm::vec3(0.25f),
    glm::uvec3(65u)
  );

  const JPH::MassProperties properties = FieldShape(field).GetMassProperties();

  const float expected = DENSITY * 8.0f * half.x * half.y * half.z;

  REQUIRE_THAT(properties.mMass, WithinRel(expected, 0.03f));

  const glm::vec3 squared = half * half;

  REQUIRE_THAT(
    properties.mInertia(0, 0),
    WithinRel(properties.mMass * (squared.y + squared.z) / 3.0f, 0.03f)
  );
  REQUIRE_THAT(
    properties.mInertia(1, 1),
    WithinRel(properties.mMass * (squared.x + squared.z) / 3.0f, 0.03f)
  );
  REQUIRE_THAT(
    properties.mInertia(2, 2),
    WithinRel(properties.mMass * (squared.x + squared.y) / 3.0f, 0.03f)
  );

  REQUIRE_THAT(
    FieldShape(field).GetCenterOfMass().GetX(),
    WithinAbs(0.0f, 0.005f)
  );
  REQUIRE_THAT(
    FieldShape(field).GetCenterOfMass().GetY(),
    WithinAbs(0.0f, 0.005f)
  );
  REQUIRE_THAT(
    FieldShape(field).GetCenterOfMass().GetZ(),
    WithinAbs(0.0f, 0.005f)
  );
}

TEST_CASE("carving the solid takes mass with it", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf whole = bakeSphere(1.0f, 65u);

  const SampledSdf carved = talus::field::bake(
    std::vector<Primitive>{
      fixture::sphere(glm::vec3(0.0f), 1.0f),
      fixture::sphere(
        glm::vec3(0.9f, 0.0f, 0.0f),
        0.5f,
        talus::core::FIELD_OP_SUBTRACTION
      )
    },
    glm::vec3(-1.5f),
    glm::vec3(1.5f),
    glm::uvec3(65u)
  );

  const float lost = FieldShape(whole).GetMassProperties().mMass
                     - FieldShape(carved).GetMassProperties().mMass;

  REQUIRE(lost > sphereMass(0.5f) * 0.5f);
  REQUIRE(lost < sphereMass(0.5f));
}

TEST_CASE("every brick has a sub shape id of its own", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 33u);
  const FieldShape shape(field);

  const glm::uvec3 bricks = talus::field::brickCounts(field);
  const uint32_t count = bricks.x * bricks.y * bricks.z;

  REQUIRE(count > 1u);
  REQUIRE((uint64_t{1} << shape.GetSubShapeIDBitsRecursive()) >= count);
}

TEST_CASE("the surface normal points out of the solid", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 33u);
  const FieldShape shape(field);

  const JPH::Vec3 normal =
    shape.GetSurfaceNormal(JPH::SubShapeID(), JPH::Vec3(1.0f, 0.0f, 0.0f));

  REQUIRE_THAT(normal.GetX(), WithinAbs(1.0f, 0.05f));
  REQUIRE_THAT(normal.GetY(), WithinAbs(0.0f, 0.05f));
  REQUIRE_THAT(normal.GetZ(), WithinAbs(0.0f, 0.05f));
}

TEST_CASE("a point inside collides and one outside does not", "[fieldshape]") {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 33u);
  const FieldShape shape(field);

  const JPH::TransformedShape context{};

  JPH::AllHitCollisionCollector<JPH::CollidePointCollector> inside;
  inside.SetContext(&context);

  shape.CollidePoint(
    JPH::Vec3(0.0f, 0.0f, 0.0f),
    JPH::SubShapeIDCreator(),
    inside,
    JPH::ShapeFilter()
  );

  JPH::AllHitCollisionCollector<JPH::CollidePointCollector> outside;
  outside.SetContext(&context);

  shape.CollidePoint(
    JPH::Vec3(1.4f, 0.0f, 0.0f),
    JPH::SubShapeIDCreator(),
    outside,
    JPH::ShapeFilter()
  );

  REQUIRE(inside.mHits.size() == 1u);
  REQUIRE(outside.mHits.empty());
}

TEST_CASE(
  "the shape borrows the field rather than copying it",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf field = bakeSphere(1.0f, 17u);
  const FieldShape shape(field);

  REQUIRE(&shape.field() == &field);
}

namespace {

SampledSdf bakeNotchedSlab(uint32_t resolution) {
  Primitive slab{};
  slab.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f)));
  slab.shape = glm::vec4(10.0f, 0.5f, 10.0f, 0.0f);
  slab.shapeConfig = glm::uvec4(1u, 1u, talus::core::FIELD_OP_UNION, 0u);
  talus::field::updateBounds(slab);

  return talus::field::bake(
    std::vector<Primitive>{
      slab,
      fixture::box(
        glm::vec3(5.0f, 0.5f, 5.0f),
        glm::vec3(1.5f, 0.25f, 1.5f),
        talus::core::FIELD_OP_SUBTRACTION
      )
    },
    glm::vec3(-10.5f, -1.0f, -10.5f),
    glm::vec3(10.5f, 1.0f, 10.5f),
    glm::uvec3(resolution)
  );
}

std::vector<JPH::CollideShapeResult> hitsBetween(
  const FieldShape& one,
  const FieldShape& two,
  const glm::vec3& atOne,
  const glm::vec3& atTwo,
  float separation
) {
  const JPH::TransformedShape context{};

  JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
  collector.SetContext(&context);

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance = separation;

  JPH::CollisionDispatch::sCollideShapeVsShape(
    &one,
    &two,
    JPH::Vec3::sReplicate(1.0f),
    JPH::Vec3::sReplicate(1.0f),
    JPH::Mat44::sTranslation(JPH::Vec3(atOne.x, atOne.y, atOne.z)),
    JPH::Mat44::sTranslation(JPH::Vec3(atTwo.x, atTwo.y, atTwo.z)),
    JPH::SubShapeIDCreator(),
    JPH::SubShapeIDCreator(),
    settings,
    collector
  );

  return {collector.mHits.begin(), collector.mHits.end()};
}

uint32_t contactsBetween(
  const FieldShape& one,
  const FieldShape& two,
  const glm::vec3& atOne,
  const glm::vec3& atTwo
) {
  return static_cast<uint32_t>(
    hitsBetween(one, two, atOne, atTwo, 0.02f).size()
  );
}

}

TEST_CASE(
  "a coarse body against a fine one collides either way round",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf slab = bakeNotchedSlab(33u);
  const SampledSdf ball = bakeSphere(0.5f, 65u);

  REQUIRE(slab.voxelSize.x > ball.voxelSize.x * 4.0f);

  const FieldShape slabShape(slab);
  const FieldShape ballShape(ball);

  REQUIRE(slabShape.hull() == nullptr);

  const glm::vec3 slabAt(0.0f);
  const glm::vec3 ballAt(0.0f, 0.95f, 0.0f);

  const uint32_t fineFirst =
    contactsBetween(ballShape, slabShape, ballAt, slabAt);
  const uint32_t coarseFirst =
    contactsBetween(slabShape, ballShape, slabAt, ballAt);

  REQUIRE(fineFirst > 0u);
  REQUIRE(coarseFirst == fineFirst);
}

TEST_CASE(
  "a field pair that penetrates hands Jolt no gap hits and a separated one "
  "keeps them",
  "[fieldshape]"
) {
  constexpr float RADIUS = 0.5f;
  constexpr float SINK = 0.01f;

  JoltLibrary library;

  const float reach = JPH::PhysicsSettings().mSpeculativeContactDistance;

  const SampledSdf slab = bakeNotchedSlab(33u);
  const SampledSdf ball = bakeSphere(RADIUS, 33u);

  const FieldShape slabShape(slab);
  const FieldShape ballShape(ball);

  REQUIRE(slabShape.hull() == nullptr);

  const glm::vec3 slabAt(0.0f);
  const glm::vec3 touching = glm::vec3(0.0f, 0.5f + RADIUS, 0.0f)
                             - slabShape.centerOfMass()
                             + ballShape.centerOfMass();

  const auto hitsAt = [&](float lift, bool ballFirst) {
    const glm::vec3 ballAt = touching + glm::vec3(0.0f, lift, 0.0f);

    return ballFirst ? hitsBetween(ballShape, slabShape, ballAt, slabAt, reach)
                     : hitsBetween(slabShape, ballShape, slabAt, ballAt, reach);
  };

  for (const bool ballFirst : {true, false}) {
    CAPTURE(ballFirst);

    const std::vector<JPH::CollideShapeResult> sunk = hitsAt(-SINK, ballFirst);

    const auto sunkGaps =
      std::ranges::count_if(sunk, [](const JPH::CollideShapeResult& hit) {
        return hit.mPenetrationDepth <= 0.0f;
      });

    INFO("sunk hits " << sunk.size() << " of which gaps " << sunkGaps);

    REQUIRE_FALSE(sunk.empty());
    CHECK(sunkGaps == 0);

    const std::vector<JPH::CollideShapeResult> hovering =
      hitsAt(0.5f * reach, ballFirst);

    const auto hoveringGaps =
      std::ranges::count_if(hovering, [](const JPH::CollideShapeResult& hit) {
        return hit.mPenetrationDepth <= 0.0f;
      });

    INFO(
      "hovering hits " << hovering.size() << " of which gaps " << hoveringGaps
    );

    REQUIRE_FALSE(hovering.empty());
    CHECK(hoveringGaps == std::ssize(hovering));
  }
}

TEST_CASE("a degenerate gradient still names a direction", "[fieldshape]") {
  JoltLibrary library;

  SampledSdf flat = bakeSphere(1.0f, 17u);

  std::fill(flat.distances.begin(), flat.distances.end(), -1.0f);

  const talus::field::FieldProbe probed =
    talus::field::probe(flat, glm::vec3(0.0f, 0.0f, 0.0f));

  REQUIRE(glm::length(talus::field::gradient(flat, glm::vec3(0.0f))) == 0.0f);
  REQUIRE_THAT(glm::length(probed.normal), WithinAbs(1.0f, 1e-6f));

  const JPH::Vec3 normal = FieldShape(flat).GetSurfaceNormal(
    JPH::SubShapeID(),
    JPH::Vec3(0.0f, 0.0f, 0.0f)
  );

  REQUIRE_THAT(normal.Length(), WithinAbs(1.0f, 1e-6f));
}

TEST_CASE(
  "a shape rebuilt from a deformation matches one built from scratch",
  "[fieldshape]"
) {
  JoltLibrary library;

  const std::vector<Primitive> primitives{
    fixture::sphere(glm::vec3(0.0f), 1.0f)
  };

  SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-1.6f),
    glm::vec3(1.6f),
    glm::uvec3(65u)
  );

  const JPH::Ref<FieldShape> before = new FieldShape(field);

  Primitive cutter = talus::field::makeSphere(
    glm::vec3(0.9f, 0.0f, 0.0f),
    0.4f,
    0u,
    talus::core::FIELD_OP_SUBTRACTION
  );

  talus::field::updateBounds(cutter);

  const talus::field::WriteReport report =
    talus::field::deformAndRepair(field, cutter).write;

  REQUIRE(report.brickEnd.x > report.brickBegin.x);

  const JPH::Ref<FieldShape> patched =
    new FieldShape(field, *before, report.brickBegin, report.brickEnd);

  const JPH::Ref<FieldShape> scratch = new FieldShape(field);

  const JPH::MassProperties wanted = scratch->GetMassProperties();
  const JPH::MassProperties got = patched->GetMassProperties();

  REQUIRE(wanted.mMass < before->GetMassProperties().mMass * 0.999f);

  REQUIRE_THAT(got.mMass, WithinRel(wanted.mMass, 1.0e-6f));

  REQUIRE_THAT(
    patched->centerOfMass().x,
    WithinAbs(scratch->centerOfMass().x, 1.0e-6f)
  );
  REQUIRE_THAT(
    patched->centerOfMass().y,
    WithinAbs(scratch->centerOfMass().y, 1.0e-6f)
  );
  REQUIRE_THAT(
    patched->centerOfMass().z,
    WithinAbs(scratch->centerOfMass().z, 1.0e-6f)
  );

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      REQUIRE_THAT(
        got.mInertia(row, column),
        WithinAbs(wanted.mInertia(row, column), 1.0e-3f)
      );
    }
  }

  REQUIRE(patched->seeds().size() == scratch->seeds().size());

  for (size_t i = 0; i < patched->seeds().size(); ++i) {
    REQUIRE(patched->seeds()[i].brick == scratch->seeds()[i].brick);

    REQUIRE_THAT(
      patched->seeds()[i].point.x,
      WithinAbs(scratch->seeds()[i].point.x, 1.0e-5f)
    );
    REQUIRE_THAT(
      patched->seeds()[i].point.y,
      WithinAbs(scratch->seeds()[i].point.y, 1.0e-5f)
    );
    REQUIRE_THAT(
      patched->seeds()[i].point.z,
      WithinAbs(scratch->seeds()[i].point.z, 1.0e-5f)
    );
  }

  const JPH::Ref<FieldShape> allReused =
    new FieldShape(field, *before, glm::uvec3(0u), glm::uvec3(0u));

  REQUIRE(
    allReused->GetMassProperties().mMass == before->GetMassProperties().mMass
  );
  REQUIRE(allReused->GetMassProperties().mMass > wanted.mMass);
}

TEST_CASE("a box of one brick rests on a face and sleeps", "[fieldshape]") {
  constexpr float HALF = 0.1f;
  constexpr float DROP = 0.4f;
  constexpr uint32_t FRAMES = 300u;

  JoltLibrary library;
  talus::physics::PhysicsWorld world;

  const glm::vec3 groundHalf(2.0f, 0.5f, 2.0f);

  const SampledSdf groundField = talus::field::bake(
    std::vector<Primitive>{fixture::box(glm::vec3(0.0f), groundHalf)},
    -1.5f * groundHalf,
    1.5f * groundHalf,
    glm::uvec3(33u)
  );

  const SampledSdf boxField = talus::field::bake(
    std::vector<Primitive>{fixture::box(glm::vec3(0.0f), glm::vec3(HALF))},
    glm::vec3(-1.5f * HALF),
    glm::vec3(1.5f * HALF),
    glm::uvec3(9u)
  );

  REQUIRE(talus::field::brickCounts(boxField) == glm::uvec3(1u));
  REQUIRE(FieldShape(groundField).hull() != nullptr);
  REQUIRE(FieldShape(boxField).hull() != nullptr);

  JPH::BodyInterface& bodies = world.bodies();

  const JPH::BodyID ground = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      new FieldShape(groundField),
      JPH::RVec3(0.0f, -groundHalf.y, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );

  JPH::BodyCreationSettings dropped(
    new FieldShape(boxField),
    JPH::RVec3(0.0f, HALF + DROP, 0.0f),
    JPH::Quat::sIdentity(),
    JPH::EMotionType::Dynamic,
    talus::physics::ObjectLayers::MOVING
  );

  dropped.mMotionQuality = JPH::EMotionQuality::LinearCast;

  const JPH::BodyID box =
    bodies.CreateAndAddBody(dropped, JPH::EActivation::Activate);

  REQUIRE_FALSE(ground.IsInvalid());
  REQUIRE_FALSE(box.IsInvalid());

  world.optimizeBroadPhase();

  const JPH::PhysicsSettings& settings = world.system().GetPhysicsSettings();

  const float fall =
    std::sqrt(2.0f * DROP / talus::physics::PhysicsWorld::GRAVITY);
  const uint32_t allowed = static_cast<uint32_t>(std::ceil(
    (fall + settings.mTimeBeforeSleep + 2.0f)
    / talus::physics::PhysicsWorld::TIME_STEP
  ));

  uint32_t asleepAt = 0u;

  for (uint32_t frame = 1u; frame <= FRAMES && asleepAt == 0u; ++frame) {
    world.step();

    if (!bodies.IsActive(box)) {
      asleepAt = frame;
    }
  }

  const JPH::Vec3 at = bodies.GetPosition(box);
  const JPH::Quat turned = bodies.GetRotation(box);

  float upright = 0.0f;

  for (const JPH::Vec3 axis :
       {JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ()}) {
    upright = std::max(upright, std::abs((turned * axis).GetY()));
  }

  const float tilt = glm::degrees(std::acos(std::min(1.0f, upright)));

  INFO(
    "asleep at " << asleepAt << " of " << allowed << " allowed, height "
                 << at.GetY() << ", tilt " << tilt << " degrees, slop "
                 << settings.mPenetrationSlop
  );

  REQUIRE(asleepAt != 0u);
  REQUIRE(asleepAt <= allowed);
  REQUIRE(at.GetY() >= HALF - settings.mPenetrationSlop);
  REQUIRE(at.GetY() <= HALF + settings.mPenetrationSlop);
  REQUIRE(tilt < 5.0f);

  bodies.RemoveBody(box);
  bodies.DestroyBody(box);
  bodies.RemoveBody(ground);
  bodies.DestroyBody(ground);
}

namespace {

struct Slide {
  float travelled = 0.0f;
  float lifted = 0.0f;
  bool stopped = false;
  float settledGap = 0.0f;
  float slop = 0.0f;
};

Slide slideOn(
  const JPH::ShapeRefC& floor,
  const JPH::Vec3& floorAt,
  float floorTop,
  const JPH::ShapeRefC& crate,
  float half,
  float speed
) {
  constexpr uint32_t SETTLE = 120u;
  constexpr uint32_t SLIDING = 240u;

  talus::physics::PhysicsWorld world;
  JPH::BodyInterface& bodies = world.bodies();

  const JPH::BodyID ground = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      floor,
      floorAt,
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );

  JPH::BodyCreationSettings moving(
    crate,
    JPH::RVec3(0.0f, floorTop + half + 0.02f, 0.0f),
    JPH::Quat::sIdentity(),
    JPH::EMotionType::Dynamic,
    talus::physics::ObjectLayers::MOVING
  );

  moving.mMotionQuality = JPH::EMotionQuality::LinearCast;
  moving.mFriction = 0.5f;

  const JPH::BodyID box =
    bodies.CreateAndAddBody(moving, JPH::EActivation::Activate);

  bodies.SetFriction(ground, 0.5f);

  world.optimizeBroadPhase();

  for (uint32_t frame = 0u; frame != SETTLE; ++frame) {
    world.step();
  }

  const JPH::Vec3 start = bodies.GetPosition(box);

  Slide slide;

  slide.settledGap = start.GetY() - (floorTop + half);
  slide.slop = world.system().GetPhysicsSettings().mPenetrationSlop;

  bodies.SetLinearVelocity(box, JPH::Vec3(speed, 0.0f, 0.0f));
  bodies.ActivateBody(box);

  for (uint32_t frame = 0u; frame != SLIDING; ++frame) {
    world.step();

    slide.lifted =
      std::max(slide.lifted, bodies.GetPosition(box).GetY() - start.GetY());
  }

  slide.travelled = bodies.GetPosition(box).GetX() - start.GetX();
  slide.stopped = bodies.GetLinearVelocity(box).Length() < 1.0e-3f;

  bodies.RemoveBody(box);
  bodies.DestroyBody(box);
  bodies.RemoveBody(ground);
  bodies.DestroyBody(ground);

  return slide;
}

SampledSdf bakeBlock(const glm::vec3& half, uint32_t samples) {
  return talus::field::bake(
    std::vector<Primitive>{fixture::box(glm::vec3(0.0f), half)},
    -1.5f * half,
    1.5f * half,
    glm::uvec3(samples)
  );
}

}

TEST_CASE(
  "a field box slides on a coarse field ground like a Jolt box",
  "[fieldshape]"
) {
  constexpr float HALF = 0.1f;
  constexpr float SPEED = 2.0f;

  JoltLibrary library;

  const glm::vec3 groundHalf(2.0f, 0.5f, 2.0f);
  const SampledSdf groundField = bakeBlock(groundHalf, 33u);
  const SampledSdf boxField = bakeBlock(glm::vec3(HALF), 9u);

  REQUIRE(FieldShape(groundField).hull() != nullptr);
  REQUIRE(FieldShape(boxField).hull() != nullptr);

  const JPH::Vec3 groundAt(0.0f, -groundHalf.y, 0.0f);

  const Slide field = slideOn(
    new FieldShape(groundField),
    groundAt,
    groundAt.GetY() + groundHalf.y,
    new FieldShape(boxField),
    HALF,
    SPEED
  );
  const Slide jolt = slideOn(
    new JPH::BoxShape(JPH::Vec3(groundHalf.x, groundHalf.y, groundHalf.z)),
    groundAt,
    groundAt.GetY() + groundHalf.y,
    new JPH::BoxShape(JPH::Vec3::sReplicate(HALF)),
    HALF,
    SPEED
  );

  INFO(
    "field settled " << field.settledGap << " travelled " << field.travelled
                     << " lifted " << field.lifted << "; jolt settled "
                     << jolt.settledGap << " travelled " << jolt.travelled
                     << " lifted " << jolt.lifted
  );

  REQUIRE(std::abs(jolt.settledGap) <= jolt.slop);
  REQUIRE(jolt.stopped);
  REQUIRE(std::abs(field.settledGap) <= field.slop);
  REQUIRE(field.stopped);
  REQUIRE(
    std::abs(field.travelled - jolt.travelled) <= 0.005f * jolt.travelled
  );
  REQUIRE(field.lifted <= jolt.lifted + 1.0e-3f);
}

TEST_CASE(
  "a field box slides on a field slab of its own or a finer voxel like a Jolt "
  "box",
  "[fieldshape]"
) {
  constexpr float HALF = 0.1f;
  constexpr float SPEED = 1.0f;

  JoltLibrary library;

  const glm::vec3 slabHalf(0.4f, HALF, 0.4f);
  const JPH::Vec3 slabAt(0.0f, slabHalf.y, 0.0f);
  const SampledSdf boxField = bakeBlock(glm::vec3(HALF), 9u);

  REQUIRE(FieldShape(boxField).hull() != nullptr);

  const Slide jolt = slideOn(
    new JPH::BoxShape(JPH::Vec3(slabHalf.x, slabHalf.y, slabHalf.z)),
    slabAt,
    slabAt.GetY() + slabHalf.y,
    new JPH::BoxShape(JPH::Vec3::sReplicate(HALF)),
    HALF,
    SPEED
  );

  REQUIRE(std::abs(jolt.settledGap) <= jolt.slop);
  REQUIRE(jolt.stopped);

  for (const uint32_t refinement : {1u, 2u}) {
    const SampledSdf slabField = talus::field::bake(
      std::vector<Primitive>{fixture::box(glm::vec3(0.0f), slabHalf)},
      -1.5f * slabHalf,
      1.5f * slabHalf,
      glm::uvec3(32u, 8u, 32u) * refinement + 1u
    );

    REQUIRE(
      glm::compMax(slabField.voxelSize) * static_cast<float>(refinement)
      == glm::compMax(boxField.voxelSize)
    );

    REQUIRE(FieldShape(slabField).hull() != nullptr);

    const Slide field = slideOn(
      new FieldShape(slabField),
      slabAt,
      slabAt.GetY() + slabHalf.y,
      new FieldShape(boxField),
      HALF,
      SPEED
    );

    INFO(
      "refinement " << refinement << ": field settled " << field.settledGap
                    << " travelled " << field.travelled << " lifted "
                    << field.lifted << "; jolt settled " << jolt.settledGap
                    << " travelled " << jolt.travelled << " lifted "
                    << jolt.lifted
    );

    REQUIRE(std::abs(field.settledGap) <= field.slop);
    REQUIRE(field.stopped);
    REQUIRE(
      std::abs(field.travelled - jolt.travelled) <= 0.005f * jolt.travelled
    );
    REQUIRE(field.lifted <= jolt.lifted + 1.0e-3f);
  }
}

TEST_CASE(
  "a field shape knows whether its lattice holds one material",
  "[fieldshape]"
) {
  constexpr uint32_t OTHER_LANE = 2u;

  JoltLibrary library;

  const glm::vec3 half(0.1f);

  Primitive near = fixture::box(glm::vec3(-0.1f, 0.0f, 0.0f), half);
  Primitive far = fixture::box(glm::vec3(0.1f, 0.0f, 0.0f), half);

  far.shapeConfig.y = OTHER_LANE;

  SampledSdf single = talus::field::bake(
    std::vector<Primitive>{near},
    glm::vec3(-0.6f),
    glm::vec3(0.6f),
    glm::uvec3(33u)
  );
  const SampledSdf mixed = talus::field::bake(
    std::vector<Primitive>{near, far},
    glm::vec3(-0.6f),
    glm::vec3(0.6f),
    glm::uvec3(33u)
  );

  REQUIRE(near.shapeConfig.y != far.shapeConfig.y);
  REQUIRE(FieldShape(single).uniformMaterial());
  REQUIRE_FALSE(FieldShape(mixed).uniformMaterial());

  const JPH::Ref<FieldShape> before = new FieldShape(single);

  const glm::uvec3 bricks = talus::field::brickCounts(single);

  const talus::field::WriteReport same = talus::field::deform(
    single,
    fixture::box(glm::vec3(-0.1f, 0.3f, 0.0f), half)
  );

  REQUIRE(glm::all(glm::lessThan(same.brickBegin, same.brickEnd)));
  REQUIRE(glm::any(glm::lessThan(same.brickEnd - same.brickBegin, bricks)));

  const JPH::Ref<FieldShape> kept =
    new FieldShape(single, *before, same.brickBegin, same.brickEnd);

  REQUIRE(kept->uniformMaterial());
  REQUIRE(FieldShape(single).uniformMaterial());

  const talus::field::WriteReport report = talus::field::deform(single, far);

  REQUIRE(glm::all(glm::lessThan(report.brickBegin, report.brickEnd)));
  REQUIRE(glm::any(glm::lessThan(report.brickEnd - report.brickBegin, bricks)));

  const JPH::Ref<FieldShape> patched =
    new FieldShape(single, *kept, report.brickBegin, report.brickEnd);

  REQUIRE(before->uniformMaterial());
  REQUIRE_FALSE(patched->uniformMaterial());
  REQUIRE_FALSE(FieldShape(single).uniformMaterial());
}

namespace {

struct Persisted {
  float friction = 0.0f;
  std::vector<uint32_t> lanes;
};

Persisted persistOnFloor(
  const SampledSdf& field,
  std::span<const talus::objectmodel::PhysicalMaterial> table,
  uint32_t floorLane,
  std::initializer_list<float> alongX
) {
  talus::physics::PhysicsWorld world;

  world.impacts().setMaterials(table);

  const JPH::Ref<FieldShape> shape = new FieldShape(field);

  JPH::BodyInterface& bodies = world.bodies();

  const JPH::BodyID block = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      shape,
      JPH::RVec3(0.0f, 1.0f, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Dynamic,
      talus::physics::ObjectLayers::MOVING
    ),
    JPH::EActivation::Activate
  );

  JPH::BoxShapeSettings floorShape(JPH::Vec3(1.0f, 0.5f, 1.0f));
  floorShape.SetEmbedded();
  floorShape.mUserData = floorLane;

  const JPH::BodyID floor = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      floorShape.Create().Get(),
      JPH::RVec3(0.0f, 0.4f, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );

  REQUIRE_FALSE(block.IsInvalid());
  REQUIRE_FALSE(floor.IsInvalid());

  bodies.SetAngularVelocity(block, JPH::Vec3(0.0f, 0.0f, -2.0f));

  JPH::ContactManifold manifold;

  manifold.mBaseOffset = JPH::RVec3(0.0f, 1.0f, 0.0f);
  manifold.mWorldSpaceNormal = JPH::Vec3(0.0f, -1.0f, 0.0f);
  manifold.mPenetrationDepth = 0.0f;

  for (const float x : alongX) {
    manifold.mRelativeContactPointsOn1.push_back(JPH::Vec3(x, -0.05f, 0.0f));
    manifold.mRelativeContactPointsOn2.push_back(JPH::Vec3(x, -0.05f, 0.0f));
  }

  JPH::ContactSettings settings;

  Persisted persisted;

  {
    const JPH::BodyLockInterfaceNoLock& locks =
      world.system().GetBodyLockInterfaceNoLock();

    const JPH::BodyLockRead blockLock(locks, block);
    const JPH::BodyLockRead floorLock(locks, floor);

    REQUIRE(blockLock.Succeeded());
    REQUIRE(floorLock.Succeeded());

    for (uint32_t point = 0u;
         point != manifold.mRelativeContactPointsOn1.size();
         ++point) {
      persisted.lanes.push_back(
        talus::physics::ImpactListener::materialAt(
          blockLock.GetBody(),
          manifold.GetWorldSpaceContactPointOn1(point)
        )
      );
    }

    world.impacts().OnContactPersisted(
      blockLock.GetBody(),
      floorLock.GetBody(),
      manifold,
      settings
    );
  }

  persisted.friction = settings.mCombinedFriction;

  bodies.RemoveBody(block);
  bodies.DestroyBody(block);
  bodies.RemoveBody(floor);
  bodies.DestroyBody(floor);

  return persisted;
}

SampledSdf bakeLaneBlock(uint32_t lane) {
  Primitive block = fixture::box(glm::vec3(0.0f), glm::vec3(0.1f));

  block.shapeConfig.y = lane;

  return talus::field::bake(
    std::vector<Primitive>{block},
    glm::vec3(-0.3f),
    glm::vec3(0.3f),
    glm::uvec3(17u)
  );
}

}

TEST_CASE(
  "a persisted contact on a body of two materials takes the strongest point",
  "[fieldshape]"
) {
  constexpr uint32_t FLOOR_LANE = 0u;
  constexpr uint32_t RISING_LANE = 1u;
  constexpr uint32_t PRESSING_LANE = 2u;

  JoltLibrary library;

  std::vector<talus::objectmodel::PhysicalMaterial> table(3);

  table[FLOOR_LANE].friction = 1.0f;
  table[RISING_LANE].friction = 0.1f;
  table[PRESSING_LANE].friction = 0.9f;

  const glm::vec3 half(0.1f);

  Primitive rising = fixture::box(glm::vec3(-0.1f, 0.0f, 0.0f), half);
  Primitive pressing = fixture::box(glm::vec3(0.1f, 0.0f, 0.0f), half);

  rising.shapeConfig.y = RISING_LANE;
  pressing.shapeConfig.y = PRESSING_LANE;

  const SampledSdf field = talus::field::bake(
    std::vector<Primitive>{rising, pressing},
    glm::vec3(-0.3f),
    glm::vec3(0.3f),
    glm::uvec3(17u)
  );

  REQUIRE_FALSE(FieldShape(field).uniformMaterial());

  const Persisted persisted =
    persistOnFloor(field, table, FLOOR_LANE, {-0.15f, 0.15f});

  REQUIRE(persisted.lanes == std::vector<uint32_t>{RISING_LANE, PRESSING_LANE});

  const float pressed =
    std::sqrt(table[PRESSING_LANE].friction * table[FLOOR_LANE].friction);
  const float risen =
    std::sqrt(table[RISING_LANE].friction * table[FLOOR_LANE].friction);

  REQUIRE(pressed != risen);
  REQUIRE(persisted.friction == pressed);
}

TEST_CASE(
  "a persisted contact between bodies of one material each combines both",
  "[fieldshape]"
) {
  constexpr uint32_t UNREAD_LANE = 0u;
  constexpr uint32_t FLOOR_LANE = 1u;
  constexpr uint32_t BLOCK_LANE = 2u;

  JoltLibrary library;

  std::vector<talus::objectmodel::PhysicalMaterial> table(3);

  table[UNREAD_LANE].friction = 0.5f;
  table[FLOOR_LANE].friction = 0.2f;
  table[BLOCK_LANE].friction = 0.8f;

  const SampledSdf field = bakeLaneBlock(BLOCK_LANE);

  REQUIRE(FieldShape(field).uniformMaterial());

  const Persisted persisted =
    persistOnFloor(field, table, FLOOR_LANE, {-0.15f, 0.15f});

  REQUIRE(persisted.lanes == std::vector<uint32_t>{BLOCK_LANE, BLOCK_LANE});

  const float combined =
    std::sqrt(table[BLOCK_LANE].friction * table[FLOOR_LANE].friction);

  for (const uint32_t one : {UNREAD_LANE, FLOOR_LANE, BLOCK_LANE}) {
    for (const uint32_t two : {UNREAD_LANE, FLOOR_LANE, BLOCK_LANE}) {
      const bool right = (one == BLOCK_LANE && two == FLOOR_LANE)
                         || (one == FLOOR_LANE && two == BLOCK_LANE);

      REQUIRE(
        right
        == (std::sqrt(table[one].friction * table[two].friction) == combined)
      );
    }
  }

  REQUIRE(persisted.friction == combined);
}

TEST_CASE(
  "a persisted contact that leaves a uniform body's lattice takes the "
  "strongest point",
  "[fieldshape]"
) {
  constexpr uint32_t BEYOND_LANE = 0u;
  constexpr uint32_t FLOOR_LANE = 1u;
  constexpr uint32_t BLOCK_LANE = 2u;

  JoltLibrary library;

  std::vector<talus::objectmodel::PhysicalMaterial> table(3);

  table[BEYOND_LANE].friction = 0.5f;
  table[FLOOR_LANE].friction = 0.2f;
  table[BLOCK_LANE].friction = 0.8f;

  const SampledSdf field = bakeLaneBlock(BLOCK_LANE);

  REQUIRE(FieldShape(field).uniformMaterial());

  const Persisted persisted =
    persistOnFloor(field, table, FLOOR_LANE, {-0.45f, 0.15f});

  REQUIRE(persisted.lanes == std::vector<uint32_t>{BEYOND_LANE, BLOCK_LANE});

  const float pressed =
    std::sqrt(table[BLOCK_LANE].friction * table[FLOOR_LANE].friction);
  const float beyond =
    std::sqrt(table[BEYOND_LANE].friction * table[FLOOR_LANE].friction);

  REQUIRE(pressed != beyond);
  REQUIRE(persisted.friction == pressed);
}

namespace {

constexpr float FIN_VOXEL = 1.5f / 32.0f;

std::vector<Primitive> finPrimitives(float depth) {
  return std::vector<Primitive>{
    fixture::box(glm::vec3(0.0f, -0.45f, 0.0f), glm::vec3(0.5f, 0.1f, 0.5f)),
    fixture::box(
      glm::vec3(0.0f, 0.0f, depth),
      glm::vec3(0.3f, 0.4f, 0.3f * FIN_VOXEL)
    )
  };
}

SampledSdf bakeFinFrom(const std::vector<Primitive>& primitives) {
  return talus::field::bake(
    primitives,
    glm::vec3(-0.75f),
    glm::vec3(0.75f),
    glm::uvec3(33u)
  );
}

SampledSdf bakeFin(float depth) {
  return bakeFinFrom(finPrimitives(depth));
}

SampledSdf bakeNotchedFin(float depth) {
  std::vector<Primitive> primitives = finPrimitives(depth);

  primitives.push_back(
    fixture::box(
      glm::vec3(-0.35f, -0.35f, -0.35f),
      glm::vec3(0.1f),
      talus::core::FIELD_OP_SUBTRACTION
    )
  );

  return bakeFinFrom(primitives);
}

JPH::Vec3 localTo(const FieldShape& shape, const glm::vec3& point) {
  const glm::vec3 local = point - shape.centerOfMass();

  return JPH::Vec3(local.x, local.y, local.z);
}

struct ReachSweep {
  uint32_t culled = 0u;
  uint32_t missed = 0u;
  float nearest = std::numeric_limits<float>::max();
  glm::vec3 at{0.0f};
};

ReachSweep sweepReach(const FieldShape& shape, float reach) {
  constexpr int MARGIN = 3;
  constexpr std::array<float, 2> WITHIN{0.0f, 0.41f};

  const SampledSdf& field = shape.field();

  std::array<std::vector<float>, 3> along;

  for (int axis = 0; axis != 3; ++axis) {
    const int last = static_cast<int>(field.resolution[axis]) - 1 + MARGIN;

    for (int step = -MARGIN; step <= last; ++step) {
      for (const float within : WITHIN) {
        along[axis].push_back(
          field.origin[axis]
          + field.voxelSize[axis] * (static_cast<float>(step) + within)
        );
      }
    }
  }

  ReachSweep sweep;

  for (const float z : along[2]) {
    for (const float y : along[1]) {
      for (const float x : along[0]) {
        const JPH::Vec3 local = localTo(shape, glm::vec3(x, y, z));

        if (!shape.beyondReach(local, reach)) {
          continue;
        }

        ++sweep.culled;

        const glm::vec3 probed =
          glm::vec3(local.GetX(), local.GetY(), local.GetZ())
          + shape.centerOfMass();

        const float measured = talus::field::probeDistance(field, probed);

        if (measured < reach) {
          ++sweep.missed;
        }

        if (measured < sweep.nearest) {
          sweep.nearest = measured;
          sweep.at = probed;
        }
      }
    }
  }

  return sweep;
}

std::vector<JPH::CollideShapeResult> uncutHits(
  const FieldShape& one,
  const FieldShape& two,
  const glm::vec3& atOne,
  const glm::vec3& atTwo,
  float reach
) {
  std::vector<JPH::CollideShapeResult> hits = hitsBetween(
    one,
    two,
    atOne,
    atTwo,
    std::nextafter(reach, std::numeric_limits<float>::infinity())
  );

  std::erase_if(hits, [reach](const JPH::CollideShapeResult& hit) {
    return -hit.mPenetrationDepth >= reach;
  });

  return hits;
}

bool sameHits(
  const std::vector<JPH::CollideShapeResult>& left,
  const std::vector<JPH::CollideShapeResult>& right
) {
  return std::equal(
    left.begin(),
    left.end(),
    right.begin(),
    right.end(),
    [](const JPH::CollideShapeResult& a, const JPH::CollideShapeResult& b) {
      return a.mSubShapeID1 == b.mSubShapeID1
             && a.mSubShapeID2 == b.mSubShapeID2
             && a.mContactPointOn1 == b.mContactPointOn1
             && a.mContactPointOn2 == b.mContactPointOn2
             && a.mPenetrationAxis == b.mPenetrationAxis
             && a.mPenetrationDepth == b.mPenetrationDepth;
    }
  );
}

}

TEST_CASE(
  "every point the reach cull skips lies at least the reach away",
  "[fieldshape]"
) {
  JoltLibrary library;

  const float reach = JPH::PhysicsSettings().mSpeculativeContactDistance;

  const SampledSdf baked = bakeSphere(0.5f, 33u);

  SampledSdf carved = baked;

  REQUIRE(
    talus::field::deformAndRepair(
      carved,
      fixture::sphere(
        glm::vec3(0.5f, 0.0f, 0.0f),
        0.25f,
        talus::core::FIELD_OP_SUBTRACTION
      )
    )
      .cellsRemoved
    > 0u
  );

  const SampledSdf finOnPlane = bakeFin(0.0f);
  const SampledSdf finBetweenPlanes = bakeFin(0.5f * FIN_VOXEL);

  REQUIRE(finBetweenPlanes.voxelSize.z == FIN_VOXEL);

  const float between =
    talus::field::distance(finBetweenPlanes, glm::vec3(0.0f));

  REQUIRE(between > 0.0f);
  REQUIRE(between < reach);

  const SampledSdf throughFloor = talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(0.0f, -0.75f, 0.0f), glm::vec3(0.5f, 0.2f, 0.5f))
    },
    glm::vec3(-0.75f),
    glm::vec3(0.75f),
    glm::uvec3(33u)
  );

  REQUIRE(
    talus::field::probeDistance(throughFloor, glm::vec3(0.0f, -0.8f, 0.0f))
    < 0.0f
  );

  const std::array<std::pair<const char*, const SampledSdf*>, 5> fields{
    {{"baked", &baked},
     {"carved", &carved},
     {"fin on a lattice plane", &finOnPlane},
     {"fin between lattice planes", &finBetweenPlanes},
     {"solid through the grid floor", &throughFloor}}
  };

  for (const auto& [name, field] : fields) {
    const FieldShape shape(*field);
    const ReachSweep sweep = sweepReach(shape, reach);

    INFO(
      name << ": reach " << reach << " culled " << sweep.culled << " missed "
           << sweep.missed << " nearest " << sweep.nearest << " at "
           << sweep.at.x << " " << sweep.at.y << " " << sweep.at.z
    );

    CHECK(sweep.culled > 0u);
    CHECK(sweep.missed == 0u);
  }
}

TEST_CASE(
  "a crate beside a fin between lattice planes keeps the contacts of an "
  "uncut probe",
  "[fieldshape]"
) {
  constexpr float CRATE_HALF = 0.1f;

  JoltLibrary library;

  const float reach = JPH::PhysicsSettings().mSpeculativeContactDistance;

  const SampledSdf finField = bakeNotchedFin(0.5f * FIN_VOXEL);
  const FieldShape fin(finField);

  REQUIRE(fin.hull() == nullptr);

  const glm::vec3 face(0.0f, -0.2f, 1.1f * FIN_VOXEL);

  REQUIRE(
    talus::field::distance(finField, glm::vec3(face.x, face.y, 0.0f)) > 0.0f
  );
  REQUIRE(talus::field::probeDistance(finField, face) < reach);

  const SampledSdf crateField = bakeBlock(glm::vec3(CRATE_HALF), 9u);
  const FieldShape crate(crateField);

  const glm::vec3 crateAt = face + glm::vec3(0.0f, 0.0f, CRATE_HALF)
                            - fin.centerOfMass() + crate.centerOfMass();

  const std::vector<JPH::CollideShapeResult> hits =
    hitsBetween(crate, fin, crateAt, glm::vec3(0.0f), reach);
  const std::vector<JPH::CollideShapeResult> uncut =
    uncutHits(crate, fin, crateAt, glm::vec3(0.0f), reach);

  INFO("hits " << hits.size() << " uncut " << uncut.size());

  REQUIRE_FALSE(uncut.empty());
  REQUIRE(sameHits(hits, uncut));
}

TEST_CASE(
  "a crate hovering over a surface just below a lattice plane has a "
  "speculative contact",
  "[fieldshape]"
) {
  constexpr float CRATE_HALF = 0.1f;

  JoltLibrary library;

  const float reach = JPH::PhysicsSettings().mSpeculativeContactDistance;

  const float plane = -0.75f + 20.0f * FIN_VOXEL;
  const float surface = plane - 0.1f * reach;
  const float hover = 0.5f * reach;

  const SampledSdf groundField = talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(0.5f, surface, 0.5f)),
      fixture::box(
        glm::vec3(0.35f, surface, 0.35f),
        glm::vec3(0.1f),
        talus::core::FIELD_OP_SUBTRACTION
      )
    },
    glm::vec3(-0.75f),
    glm::vec3(0.75f),
    glm::uvec3(33u)
  );

  REQUIRE(groundField.voxelSize.y == FIN_VOXEL);
  REQUIRE(
    talus::field::distance(groundField, glm::vec3(0.0f, plane, 0.0f)) > 0.0f
  );
  REQUIRE(surface + hover > plane);

  const FieldShape ground(groundField);

  REQUIRE(ground.hull() == nullptr);

  const SampledSdf crateField = bakeBlock(glm::vec3(CRATE_HALF), 9u);
  const FieldShape crate(crateField);

  const glm::vec3 crateAt = glm::vec3(0.0f, surface + hover + CRATE_HALF, 0.0f)
                            - ground.centerOfMass() + crate.centerOfMass();

  const std::vector<JPH::CollideShapeResult> hits =
    hitsBetween(crate, ground, crateAt, glm::vec3(0.0f), reach);

  REQUIRE_FALSE(hits.empty());

  for (const JPH::CollideShapeResult& hit : hits) {
    REQUIRE(hit.mPenetrationDepth < 0.0f);
    REQUIRE(hit.mPenetrationDepth > -reach);
  }

  REQUIRE(
    sameHits(hits, uncutHits(crate, ground, crateAt, glm::vec3(0.0f), reach))
  );
}

TEST_CASE(
  "an edit that adds solid beyond the reach box moves the box",
  "[fieldshape]"
) {
  constexpr float ADDED_TOP = 1.3f;
  constexpr float CRATE_HALF = 0.1f;
  constexpr float PRESSED = 0.005f;

  JoltLibrary library;

  const float reach = JPH::PhysicsSettings().mSpeculativeContactDistance;

  SampledSdf field = talus::field::bake(
    std::vector<Primitive>{fixture::sphere(glm::vec3(0.0f), 0.5f)},
    glm::vec3(-1.5f),
    glm::vec3(1.5f),
    glm::uvec3(33u)
  );

  const JPH::Ref<FieldShape> before = new FieldShape(field);

  const glm::vec3 above(0.0f, ADDED_TOP + 0.5f * reach, 0.0f);

  REQUIRE(before->beyondReach(localTo(*before, above), reach));

  const talus::field::WriteReport report = talus::field::deform(
    field,
    fixture::sphere(glm::vec3(0.0f, 1.0f, 0.0f), ADDED_TOP - 1.0f)
  );

  REQUIRE(
    glm::any(
      glm::lessThan(
        report.brickEnd - report.brickBegin,
        talus::field::brickCounts(field)
      )
    )
  );

  const JPH::Ref<FieldShape> patched =
    new FieldShape(field, *before, report.brickBegin, report.brickEnd);

  REQUIRE(talus::field::probeDistance(field, above) < reach);
  REQUIRE_FALSE(patched->beyondReach(localTo(*patched, above), reach));

  const ReachSweep sweep = sweepReach(*patched, reach);

  INFO(
    "culled " << sweep.culled << " missed " << sweep.missed << " nearest "
              << sweep.nearest << " at " << sweep.at.x << " " << sweep.at.y
              << " " << sweep.at.z
  );

  CHECK(sweep.culled > 0u);
  CHECK(sweep.missed == 0u);

  const SampledSdf crateField = bakeBlock(glm::vec3(CRATE_HALF), 9u);
  const FieldShape crate(crateField);

  const glm::vec3 crateAt =
    glm::vec3(0.0f, ADDED_TOP - PRESSED + CRATE_HALF, 0.0f)
    - patched->centerOfMass() + crate.centerOfMass();

  REQUIRE(contactsBetween(crate, *patched, crateAt, glm::vec3(0.0f)) > 0u);
}

namespace {

constexpr float STACK_HALF = 0.1f;
constexpr float SMALL_HALF = 0.05f;
constexpr float GROOVE_DEGREES = 12.0f;
constexpr float CRITERION_DEGREES = 1.0f;
constexpr uint32_t CRITERION_POINTS = 4u;

const glm::vec3 GROOVE_HALF(0.25f, 0.1f, 0.3f);

SampledSdf bakeTurnedBlock(
  const glm::vec3& half,
  uint32_t samples,
  const glm::mat4& turn
) {
  Primitive block = fixture::box(glm::vec3(0.0f), half);

  block.inverseModel = glm::inverse(turn);
  talus::field::updateBounds(block);

  return talus::field::bake(
    std::vector<Primitive>{block},
    -1.5f * half,
    1.5f * half,
    glm::uvec3(samples)
  );
}

Primitive tiltedSlab(
  const glm::vec3& top,
  const glm::vec3& half,
  float degrees
) {
  Primitive slab = fixture::box(glm::vec3(0.0f), half);

  const glm::mat4 turned = glm::rotate(
    glm::mat4(1.0f),
    glm::radians(degrees),
    glm::vec3(0.0f, 0.0f, 1.0f)
  );
  const glm::vec3 up = glm::vec3(turned * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));

  slab.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), top - half.y * up) * turned);
  talus::field::updateBounds(slab);

  return slab;
}

glm::vec3 grooveTop(float sign) {
  const float slope = glm::radians(GROOVE_DEGREES);

  return GROOVE_HALF.x
         * glm::vec3(sign * std::cos(slope), std::sin(slope), 0.0f);
}

SampledSdf bakeGroove() {
  return talus::field::bake(
    std::vector<Primitive>{
      tiltedSlab(grooveTop(-1.0f), GROOVE_HALF, -GROOVE_DEGREES),
      tiltedSlab(grooveTop(1.0f), GROOVE_HALF, GROOVE_DEGREES)
    },
    glm::vec3(-0.6f, -0.35f, -0.4f),
    glm::vec3(0.6f, 0.25f, 0.4f),
    glm::uvec3(49u, 25u, 33u)
  );
}

JPH::Mat44 restingOn(float top, float sink, JPH::QuatArg turned, float alongX) {
  float lowest = std::numeric_limits<float>::max();

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const JPH::Vec3 at(
      (corner & 1u) != 0u ? STACK_HALF : -STACK_HALF,
      (corner & 2u) != 0u ? STACK_HALF : -STACK_HALF,
      (corner & 4u) != 0u ? STACK_HALF : -STACK_HALF
    );

    lowest = std::min(lowest, (turned * at).GetY());
  }

  return JPH::Mat44::sRotationTranslation(
    turned,
    JPH::Vec3(alongX, top - lowest - sink, 0.0f)
  );
}

struct EdgePose {
  float tilt = 0.0f;
  bool smallFace = false;
};

constexpr std::array<EdgePose, 5> EDGE_POSES{
  {{20.0f, false},
   {30.0f, false},
   {40.0f, false},
   {45.0f, false},
   {30.0f, true}}
};

JPH::Mat44 upperFor(const EdgePose& pose, float sink) {
  return restingOn(
    pose.smallFace ? SMALL_HALF : STACK_HALF,
    sink,
    JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), glm::radians(pose.tilt)),
    pose.smallFace ? 0.025f : 0.0f
  );
}

float degreesFrom(JPH::Vec3Arg axis, JPH::Vec3Arg expected) {
  if (axis.LengthSq() <= 0.0f) {
    return 180.0f;
  }

  return glm::degrees(
    std::acos(std::clamp(axis.Normalized().Dot(expected), -1.0f, 1.0f))
  );
}

std::vector<JPH::CollideShapeResult> hitsPosed(
  const JPH::Shape& one,
  const JPH::Shape& two,
  JPH::Mat44Arg atOne,
  JPH::Mat44Arg atTwo
) {
  const JPH::TransformedShape context{};

  JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
  collector.SetContext(&context);

  JPH::CollideShapeSettings settings;
  settings.mMaxSeparationDistance =
    JPH::PhysicsSettings().mSpeculativeContactDistance;
  settings.mCollectFacesMode = JPH::ECollectFacesMode::CollectFaces;

  JPH::CollisionDispatch::sCollideShapeVsShape(
    &one,
    &two,
    JPH::Vec3::sReplicate(1.0f),
    JPH::Vec3::sReplicate(1.0f),
    atOne,
    atTwo,
    JPH::SubShapeIDCreator(),
    JPH::SubShapeIDCreator(),
    settings,
    collector
  );

  return {collector.mHits.begin(), collector.mHits.end()};
}

struct Manifold {
  JPH::Vec3 first = JPH::Vec3::sZero();
  JPH::Vec3 normal = JPH::Vec3::sZero();
  JPH::ContactPoints on1;
  JPH::ContactPoints on2;
};

std::vector<Manifold> manifoldsOf(
  const std::vector<JPH::CollideShapeResult>& hits
) {
  const JPH::PhysicsSettings physics;

  std::vector<Manifold> manifolds;

  for (const JPH::CollideShapeResult& hit : hits) {
    const JPH::Vec3 normal = hit.mPenetrationAxis.Normalized();

    auto found = std::find_if(
      manifolds.begin(),
      manifolds.end(),
      [&](const Manifold& manifold) {
        return normal.Dot(manifold.first)
               >= physics.mContactNormalCosMaxDeltaRotation;
      }
    );

    if (found == manifolds.end()) {
      manifolds.push_back(Manifold{normal, JPH::Vec3::sZero(), {}, {}});
      found = manifolds.end() - 1;
    }

    found->normal += normal;

    JPH::ManifoldBetweenTwoFaces(
      hit.mContactPointOn1,
      hit.mContactPointOn2,
      hit.mPenetrationAxis,
      physics.mSpeculativeContactDistance + physics.mManifoldTolerance,
      hit.mShape1Face,
      hit.mShape2Face,
      found->on1,
      found->on2 JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero())
    );

    if (found->on1.size() > 32u) {
      JPH::PruneContactPoints(
        found->first,
        found->on1,
        found->on2 JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero())
      );
    }
  }

  for (Manifold& manifold : manifolds) {
    manifold.normal = manifold.normal.Normalized();

    if (manifold.on1.size() > 4u) {
      JPH::PruneContactPoints(
        manifold.normal,
        manifold.on1,
        manifold.on2 JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero())
      );
    }
  }

  return manifolds;
}

struct Verdict {
  size_t manifolds = 0u;
  uint32_t mostPoints = 0u;
  float worstNormal = 0.0f;
  bool pass = false;
};

Verdict judgeConvex(const std::vector<Manifold>& manifolds, JPH::Vec3Arg down) {
  Verdict verdict;

  verdict.manifolds = manifolds.size();

  for (const Manifold& manifold : manifolds) {
    verdict.mostPoints =
      std::max(verdict.mostPoints, static_cast<uint32_t>(manifold.on1.size()));
    verdict.worstNormal =
      std::max(verdict.worstNormal, degreesFrom(manifold.normal, down));
  }

  verdict.pass = verdict.manifolds == 1u
                 && verdict.mostPoints <= CRITERION_POINTS
                 && verdict.worstNormal <= CRITERION_DEGREES;

  return verdict;
}

glm::vec3 hullVertex(const JPH::ConvexHullShape& hull, JPH::uint index) {
  const JPH::Vec3 at = hull.GetPoint(index) + hull.GetCenterOfMass();

  return glm::vec3(at.GetX(), at.GetY(), at.GetZ());
}

float nearestVertex(const JPH::ConvexHullShape& hull, const glm::vec3& to) {
  float nearest = std::numeric_limits<float>::max();

  for (JPH::uint index = 0u; index != hull.GetNumPoints(); ++index) {
    nearest = std::min(nearest, glm::length(hullVertex(hull, index) - to));
  }

  return nearest;
}

float furthestAlong(const JPH::ConvexHullShape& hull, const glm::vec3& axis) {
  float furthest = std::numeric_limits<float>::lowest();

  for (JPH::uint index = 0u; index != hull.GetNumPoints(); ++index) {
    furthest = std::max(furthest, glm::dot(hullVertex(hull, index), axis));
  }

  return furthest;
}

Primitive cornerCut(const glm::vec3& axis, float offset) {
  constexpr float CUTTER_HALF = 0.3f;

  const glm::vec3 up(0.0f, 1.0f, 0.0f);

  return talus::field::makeBox(
    (offset + CUTTER_HALF) * axis,
    glm::vec3(CUTTER_HALF),
    std::acos(glm::dot(up, axis)),
    glm::normalize(glm::cross(up, axis)),
    0u,
    talus::core::FIELD_OP_SUBTRACTION
  );
}

Primitive topDent(float top, float voxel) {
  return fixture::box(
    glm::vec3(0.0f, top, 0.0f),
    glm::vec3(2.0f * voxel, voxel, 2.0f * voxel),
    talus::core::FIELD_OP_SUBTRACTION
  );
}

JPH::Vec3 intoSlab(float sign) {
  const float slope = glm::radians(GROOVE_DEGREES);

  return -JPH::Vec3(-sign * std::sin(slope), std::cos(slope), 0.0f);
}

float grooveTouch(const FieldShape& crate, const SampledSdf& groove) {
  const auto nearestSeed = [&](float height) {
    float nearest = std::numeric_limits<float>::max();

    for (const talus::physics::FieldSeed& seed : crate.seeds()) {
      nearest = std::min(
        nearest,
        talus::field::probeDistance(
          groove,
          seed.point - crate.centerOfMass() + glm::vec3(0.0f, height, 0.0f)
        )
      );
    }

    return nearest;
  };

  float below = 0.0f;
  float above = 0.3f;

  REQUIRE(nearestSeed(below) < 0.0f);
  REQUIRE(nearestSeed(above) > 0.0f);

  for (uint32_t step = 0u; step != 40u; ++step) {
    const float middle = 0.5f * (below + above);

    (nearestSeed(middle) < 0.0f ? below : above) = middle;
  }

  return above;
}

constexpr uint32_t DENTED_SAMPLES = 17u;

float dentedVoxel() {
  return 3.0f * STACK_HALF / float(DENTED_SAMPLES - 1u);
}

SampledSdf bakeCarvedBlock(const Primitive& carve, uint32_t samples) {
  return talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(STACK_HALF)),
      carve
    },
    glm::vec3(-1.5f * STACK_HALF),
    glm::vec3(1.5f * STACK_HALF),
    glm::uvec3(samples)
  );
}

const glm::vec3 OFF_CENTRE_DENT(
  0.5f * STACK_HALF,
  STACK_HALF,
  0.4f * STACK_HALF
);

Primitive offCentreDent(float voxel) {
  return fixture::box(
    OFF_CENTRE_DENT,
    glm::vec3(2.0f * voxel, voxel, 2.0f * voxel),
    talus::core::FIELD_OP_SUBTRACTION
  );
}

Primitive closedPocket(float voxel) {
  return fixture::box(
    glm::vec3(0.0f, STACK_HALF - 3.0f * voxel, 0.0f),
    glm::vec3(3.0f * voxel, voxel, 3.0f * voxel),
    talus::core::FIELD_OP_SUBTRACTION
  );
}

SampledSdf bakeCornerBlock(uint32_t samples) {
  return talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(STACK_HALF), glm::vec3(STACK_HALF))
    },
    glm::vec3(-1.5f * STACK_HALF),
    glm::vec3(1.5f * STACK_HALF),
    glm::uvec3(samples)
  );
}

SampledSdf bakeEdgeStep(uint32_t samples) {
  return talus::field::bake(
    std::vector<Primitive>{
      fixture::box(
        glm::vec3(0.0f, -1.5f * STACK_HALF, 0.0f),
        glm::vec3(2.0f * STACK_HALF, 0.5f * STACK_HALF, 2.0f * STACK_HALF)
      ),
      fixture::box(
        glm::vec3(1.5f * STACK_HALF, 0.5f * STACK_HALF, 1.5f * STACK_HALF),
        glm::vec3(0.5f * STACK_HALF, 2.0f * STACK_HALF, 0.5f * STACK_HALF)
      )
    },
    glm::vec3(-1.5f * STACK_HALF),
    glm::vec3(1.5f * STACK_HALF),
    glm::uvec3(samples)
  );
}

Primitive topNotch() {
  return fixture::box(
    glm::vec3(0.5f * STACK_HALF, STACK_HALF, 0.4f * STACK_HALF),
    glm::vec3(0.3f * STACK_HALF, 0.4f * STACK_HALF, 0.3f * STACK_HALF),
    talus::core::FIELD_OP_SUBTRACTION
  );
}

struct NamedField {
  const char* name = "";
  SampledSdf field;
};

std::vector<NamedField> nonConvexFields() {
  const float voxel = dentedVoxel();

  std::vector<NamedField> fields;

  fields.push_back({"groove", bakeGroove()});
  fields.push_back(
    {"off-centre dent", bakeCarvedBlock(offCentreDent(voxel), DENTED_SAMPLES)}
  );
  fields.push_back(
    {"closed pocket", bakeCarvedBlock(closedPocket(voxel), DENTED_SAMPLES)}
  );
  fields.push_back({"fin on a lattice plane", bakeFin(0.0f)});
  fields.push_back(
    {"step through the lattice's edges", bakeEdgeStep(DENTED_SAMPLES)}
  );
  fields.push_back(
    {"notched block of 129 samples", bakeCarvedBlock(topNotch(), 129u)}
  );

  return fields;
}

SampledSdf bakeGroundChunk() {
  const glm::vec3 half(16.0f, 0.5f, 16.0f);

  return talus::field::bake(
    std::vector<Primitive>{fixture::box(glm::vec3(0.0f), half)},
    -1.5f * half,
    1.5f * half,
    glm::uvec3(65u)
  );
}

}

TEST_CASE(
  "convex field bodies meet through Jolt's convex path with exact face "
  "normals",
  "[fieldshape]"
) {
  JoltLibrary library;

  const JPH::Vec3 down = -JPH::Vec3::sAxisY();

  const std::array<std::pair<JPH::Vec3, float>, 3> turns{
    {{JPH::Vec3::sAxisY(), 0.0f},
     {JPH::Vec3::sAxisY(), 20.0f},
     {JPH::Vec3(1.0f, 1.0f, 1.0f).Normalized(), 20.0f}}
  };

  uint32_t poses = 0u;
  uint32_t passed = 0u;
  float worstNormal = 0.0f;

  for (const uint32_t samples : {9u, 17u, 129u}) {
    for (const auto& [axis, degrees] : turns) {
      const glm::mat4 turn = glm::rotate(
        glm::mat4(1.0f),
        glm::radians(degrees),
        glm::vec3(axis.GetX(), axis.GetY(), axis.GetZ())
      );
      const JPH::Mat44 back = JPH::Mat44::sRotation(
        JPH::Quat::sRotation(axis, glm::radians(degrees)).Conjugated()
      );

      const SampledSdf boxField =
        bakeTurnedBlock(glm::vec3(STACK_HALF), samples, turn);
      const SampledSdf smallField =
        bakeTurnedBlock(glm::vec3(SMALL_HALF), samples, turn);

      const FieldShape box(boxField);
      const FieldShape small(smallField);

      for (const float sink : {0.001f, 0.0147f, 0.02f}) {
        for (const EdgePose& pose : EDGE_POSES) {
          const FieldShape& lower = pose.smallFace ? small : box;
          const JPH::Mat44 upper = upperFor(pose, sink) * back;

          const Verdict verdict = judgeConvex(
            manifoldsOf(hitsPosed(
              box,
              lower,
              upper.PreTranslated(box.GetCenterOfMass()),
              back.PreTranslated(lower.GetCenterOfMass())
            )),
            down
          );

          ++poses;
          passed += verdict.pass ? 1u : 0u;
          worstNormal = std::max(worstNormal, verdict.worstNormal);

          CAPTURE(
            samples,
            degrees,
            pose.tilt,
            pose.smallFace,
            sink,
            verdict.manifolds,
            verdict.mostPoints,
            verdict.worstNormal
          );
          CHECK(verdict.pass);
        }
      }
    }
  }

  UNSCOPED_INFO(
    passed << " of " << poses << " poses pass, worst normal " << worstNormal
           << " degrees"
  );
  REQUIRE(poses == 135u);
  REQUIRE(passed == poses);
}

namespace {

float worstCrossing(
  const SampledSdf& field,
  std::span<const talus::field::Crossing> crossings,
  const JPH::ConvexHullShape& hull
) {
  const JPH::Vec3 centre = hull.GetCenterOfMass();

  float worst = 0.0f;

  for (const talus::field::Crossing& crossing : crossings) {
    const glm::vec3 at = talus::field::latticeToField(field, crossing.lattice);
    const JPH::Vec3 local = JPH::Vec3(at.x, at.y, at.z) - centre;

    float highest = std::numeric_limits<float>::lowest();

    for (const JPH::Plane& plane : hull.GetPlanes()) {
      const JPH::Vec3 normal = plane.GetNormal();
      const float voxels = glm::length(
        glm::vec3(normal.GetX(), normal.GetY(), normal.GetZ()) * field.voxelSize
      );

      highest = std::max(highest, plane.SignedDistance(local) / voxels);
    }

    worst = std::max(worst, std::abs(highest));
  }

  return worst;
}

}

TEST_CASE("a field owns a hull only when it is convex", "[fieldshape]") {
  JoltLibrary library;

  const glm::mat4 aboutY = glm::rotate(
    glm::mat4(1.0f),
    glm::radians(20.0f),
    glm::vec3(0.0f, 1.0f, 0.0f)
  );
  const glm::mat4 aboutDiagonal = glm::rotate(
    glm::mat4(1.0f),
    glm::radians(20.0f),
    glm::normalize(glm::vec3(1.0f))
  );

  for (const uint32_t samples : {9u, 17u, 33u, 129u}) {
    CAPTURE(samples);

    const SampledSdf aligned = bakeBlock(glm::vec3(STACK_HALF), samples);
    const SampledSdf turnedY =
      bakeTurnedBlock(glm::vec3(STACK_HALF), samples, aboutY);
    const SampledSdf turnedDiagonal =
      bakeTurnedBlock(glm::vec3(STACK_HALF), samples, aboutDiagonal);

    CHECK(FieldShape(aligned).hull() != nullptr);
    CHECK(FieldShape(turnedY).hull() != nullptr);
    CHECK(FieldShape(turnedDiagonal).hull() != nullptr);
  }

  for (const uint32_t samples : {9u, 17u, 33u}) {
    CAPTURE(samples);
    CHECK(FieldShape(bakeSphere(0.5f, samples)).hull() != nullptr);
  }

  const SampledSdf chunk = bakeGroundChunk();

  REQUIRE(chunk.voxelSize.x > 20.0f * chunk.voxelSize.y);

  CHECK(FieldShape(chunk).hull() != nullptr);

  const SampledSdf corner = bakeCornerBlock(DENTED_SAMPLES);
  const glm::vec3 latticeCorner(1.5f * STACK_HALF);

  REQUIRE(talus::field::distance(corner, latticeCorner) < 0.0f);

  const FieldShape cornerShape(corner);

  CHECK(cornerShape.hull() != nullptr);

  if (cornerShape.hull() != nullptr) {
    CHECK(nearestVertex(*cornerShape.hull(), latticeCorner) <= 1.0e-5f);
  }

  for (const uint32_t samples : {65u, 129u}) {
    const SampledSdf sphere = bakeSphere(0.5f, samples);
    const talus::field::LatticeSurface surface =
      talus::field::latticeSurface(sphere);
    const talus::physics::SliceHulls slices =
      talus::physics::sliceHulls(sphere, surface);

    uint32_t vertices = 0u;
    float worst = 0.0f;

    if (slices.convex) {
      const JPH::RefConst<JPH::ConvexHullShape> built =
        talus::physics::hullThrough(sphere, surface, slices.vertices);

      REQUIRE(built != nullptr);

      vertices = built->GetNumPoints();
      worst = worstCrossing(sphere, surface.crossings, *built);
    }

    const bool owned = talus::physics::convexHullOf(sphere).shape != nullptr;

    UNSCOPED_INFO(
      "sphere of " << samples << " samples: " << (owned ? "hull" : "fallback")
                   << ", slices convex " << slices.convex << ", crossings "
                   << surface.crossings.size() << ", inside "
                   << surface.inside.size() << ", hull points "
                   << slices.vertices.size() << ", vertices " << vertices
                   << ", worst crossing " << worst << " voxels"
    );
    CHECK_FALSE(surface.crossings.empty());
  }

  const float voxel = dentedVoxel();

  const std::vector<NamedField> fields = nonConvexFields();

  const auto named = [&](std::string_view name) -> const SampledSdf& {
    return std::find_if(
             fields.begin(),
             fields.end(),
             [&](const NamedField& f) { return name == f.name; }
    )->field;
  };

  REQUIRE_THAT(named("off-centre dent").voxelSize.x, WithinRel(voxel, 1.0e-5f));
  REQUIRE(
    talus::field::distance(named("off-centre dent"), OFF_CENTRE_DENT)
    > 0.5f * voxel
  );
  REQUIRE(
    talus::field::distance(
      named("closed pocket"),
      glm::vec3(0.0f, STACK_HALF - 3.0f * voxel, 0.0f)
    )
    > 0.5f * voxel
  );
  REQUIRE(
    talus::field::distance(
      named("closed pocket"),
      glm::vec3(0.0f, STACK_HALF - 1.5f * voxel, 0.0f)
    )
    < 0.0f
  );
  REQUIRE(
    talus::field::distance(named("fin on a lattice plane"), glm::vec3(0.0f))
    < 0.0f
  );
  REQUIRE(
    talus::field::distance(
      named("step through the lattice's edges"),
      latticeCorner
    )
    < 0.0f
  );
  REQUIRE(
    talus::field::distance(
      named("step through the lattice's edges"),
      -latticeCorner
    )
    < 0.0f
  );
  REQUIRE(
    talus::field::distance(
      named("notched block of 129 samples"),
      OFF_CENTRE_DENT
    )
    > 0.0f
  );

  for (const NamedField& field : fields) {
    CAPTURE(field.name);
    CHECK(FieldShape(field.field).hull() == nullptr);
  }
}

namespace {

constexpr float PILE_SLAB_VOXEL = 0.1f;
constexpr float PILE_SLAB_CENTRE = -0.05f;
constexpr float PILE_SLAB_REACH_VOXELS = 2.0f;
constexpr float SPLIT_TOP_REACH_VOXELS = 2.5f;
constexpr float PILE_DROP = 0.005f;
constexpr float PILE_TURN_LIMIT = 0.5f;

constexpr std::array<float, 2> PILE_BAR_VOXELS{0.0125f, 0.025f};
constexpr std::array<float, 2> PROBE_SINKS{2.0e-4f, 8.4e-3f};

const glm::vec3 PILE_SLAB_HALF(5.2f, 0.05f, 3.325f);
const glm::vec3 PILE_BAR_HALF(0.2f, 0.025f, 0.025f);

struct PilePose {
  float x = 0.0f;
  float z = 0.0f;
  float yaw = 0.0f;
};

constexpr PilePose PROBE_POSE{1.5f, 1.375f, -114.178f};
constexpr PilePose CENTRE_POSE{0.0f, 0.0f, 0.0f};

SampledSdf bakePileSlab(float reachVoxels) {
  const glm::vec3 reach =
    PILE_SLAB_HALF + glm::vec3(reachVoxels * PILE_SLAB_VOXEL);

  return talus::field::bake(
    std::vector<Primitive>{fixture::box(glm::vec3(0.0f), PILE_SLAB_HALF)},
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / PILE_SLAB_VOXEL)) + glm::uvec3(1u)
  );
}

SampledSdf bakePileBar(float voxel, bool padded) {
  const glm::vec3 reach =
    padded ? PILE_BAR_HALF + glm::vec3(voxel) : PILE_BAR_HALF;

  return talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(0.0f), PILE_BAR_HALF + glm::vec3(0.2f * voxel))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / voxel)) + glm::uvec3(1u)
  );
}

JPH::Mat44 pileSlabAt() {
  return JPH::Mat44::sTranslation(JPH::Vec3(0.0f, PILE_SLAB_CENTRE, 0.0f));
}

JPH::Quat pileTurn(const PilePose& pose) {
  return JPH::Quat::sRotation(JPH::Vec3::sAxisY(), glm::radians(pose.yaw));
}

JPH::Mat44 barOnPileSlab(
  const FieldShape& bar,
  const PilePose& pose,
  float sink
) {
  float lowest = std::numeric_limits<float>::max();

  for (JPH::uint index = 0u; index != bar.hull()->GetNumPoints(); ++index) {
    lowest = std::min(lowest, hullVertex(*bar.hull(), index).y);
  }

  return JPH::Mat44::sRotationTranslation(
    pileTurn(pose),
    JPH::Vec3(pose.x, -sink - lowest, pose.z)
  );
}

float lowestHullPoint(const FieldShape& shape, JPH::Mat44Arg centreOfMass) {
  const JPH::ConvexHullShape& hull = *shape.hull();
  const JPH::Mat44 at = centreOfMass.PreTranslated(
    hull.GetCenterOfMass() - shape.GetCenterOfMass()
  );

  float lowest = std::numeric_limits<float>::max();

  for (JPH::uint index = 0u; index != hull.GetNumPoints(); ++index) {
    lowest = std::min(lowest, (at * hull.GetPoint(index)).GetY());
  }

  return lowest;
}

size_t upwardFaces(const JPH::ConvexHullShape& hull) {
  const float flat = std::cos(glm::radians(CRITERION_DEGREES));

  return static_cast<size_t>(
    std::ranges::count_if(hull.GetPlanes(), [flat](const JPH::Plane& plane) {
      return plane.GetNormal().GetY() > flat;
    })
  );
}

struct PileRest {
  uint32_t asleepAt = 0u;
  uint32_t allowed = 0u;
  float centre = 0.0f;
  float lowest = 0.0f;
  float fastestTurn = 0.0f;
  float slop = 0.0f;
};

PileRest restOnPileSlab(
  const SampledSdf& slabField,
  const SampledSdf& barField,
  const PilePose& pose
) {
  talus::physics::PhysicsWorld world;
  JPH::BodyInterface& bodies = world.bodies();

  const JPH::Ref<FieldShape> barShape = new FieldShape(barField);

  const JPH::BodyID slab = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      new FieldShape(slabField),
      JPH::RVec3(0.0f, PILE_SLAB_CENTRE, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );

  JPH::BodyCreationSettings dropped(
    barShape.GetPtr(),
    JPH::RVec3(pose.x, PILE_DROP - barField.origin.y, pose.z),
    pileTurn(pose),
    JPH::EMotionType::Dynamic,
    talus::physics::ObjectLayers::MOVING
  );

  dropped.mMotionQuality = JPH::EMotionQuality::LinearCast;

  const JPH::BodyID bar =
    bodies.CreateAndAddBody(dropped, JPH::EActivation::Activate);

  REQUIRE_FALSE(slab.IsInvalid());
  REQUIRE_FALSE(bar.IsInvalid());

  world.optimizeBroadPhase();

  const JPH::PhysicsSettings& settings = world.system().GetPhysicsSettings();

  PileRest rest;

  rest.slop = settings.mPenetrationSlop;
  rest.allowed = static_cast<uint32_t>(std::ceil(
    (std::sqrt(2.0f * PILE_DROP / talus::physics::PhysicsWorld::GRAVITY)
     + settings.mTimeBeforeSleep + 2.0f)
    / talus::physics::PhysicsWorld::TIME_STEP
  ));

  for (uint32_t frame = 1u; frame <= rest.allowed && rest.asleepAt == 0u;
       ++frame) {
    world.step();

    rest.fastestTurn =
      std::max(rest.fastestTurn, bodies.GetAngularVelocity(bar).Length());

    if (!bodies.IsActive(bar)) {
      rest.asleepAt = frame;
    }
  }

  rest.centre = bodies.GetPosition(bar).GetY();
  rest.lowest =
    barShape->hull() != nullptr
      ? lowestHullPoint(*barShape, bodies.GetCenterOfMassTransform(bar))
      : std::numeric_limits<float>::quiet_NaN();

  bodies.RemoveBody(bar);
  bodies.DestroyBody(bar);
  bodies.RemoveBody(slab);
  bodies.DestroyBody(slab);

  return rest;
}

}

namespace {

constexpr float LATTICE_SURFACE_ULPS = 32.0f;

float latticeSurfaceTolerance(const SampledSdf& field) {
  const glm::vec3 upper =
    field.origin
    + field.voxelSize * glm::vec3(field.resolution - glm::uvec3(1u));
  const glm::vec3 corner = glm::max(glm::abs(field.origin), glm::abs(upper));

  return LATTICE_SURFACE_ULPS * std::numeric_limits<float>::epsilon()
         * std::max({corner.x, corner.y, corner.z});
}

talus::field::LatticeSurface fullLatticeScan(const SampledSdf& field) {
  talus::field::LatticeSurface found;

  const glm::uvec3 size = field.resolution;

  const uint32_t alongY = size.x;
  const uint32_t alongZ = size.x * size.y;

  const glm::uvec3 stride(1u, alongY, alongZ);

  const float tolerance = latticeSurfaceTolerance(field);

  for (uint32_t z = 0u; z != size.z; ++z) {
    for (uint32_t y = 0u; y != size.y; ++y) {
      for (uint32_t x = 0u; x != size.x; ++x) {
        const glm::uvec3 at(x, y, z);
        const uint32_t index = x + alongY * y + alongZ * z;
        const float here = field.distances[index];
        const bool inside = here < tolerance;

        bool border = false;

        for (uint32_t axis = 0u; axis != 3u; ++axis) {
          if (at[axis] == 0u || at[axis] + 1u == size[axis]) {
            border = true;
          } else if (!(field.distances[index - stride[axis]] < tolerance)) {
            border = true;
          }

          if (at[axis] + 1u >= size[axis]) {
            continue;
          }

          const float there = field.distances[index + stride[axis]];

          if (inside == (there < tolerance)) {
            continue;
          }

          border = true;

          glm::vec3 lattice(at);
          lattice[axis] += std::clamp(here / (here - there), 0.0f, 1.0f);

          found.crossings.push_back(talus::field::Crossing{lattice, axis});
        }

        if (inside && border) {
          found.inside.push_back(at);
        }
      }
    }
  }

  return found;
}

void checkMatchesFullScan(std::string_view name, const SampledSdf& field) {
  const talus::field::LatticeSurface scanned =
    talus::field::latticeSurface(field);
  const talus::field::LatticeSurface reference = fullLatticeScan(field);

  const auto sameCrossing = [](
                              const talus::field::Crossing& left,
                              const talus::field::Crossing& right
                            ) {
    return left.axis == right.axis && left.lattice == right.lattice;
  };

  const auto crossing = std::mismatch(
    scanned.crossings.begin(),
    scanned.crossings.end(),
    reference.crossings.begin(),
    reference.crossings.end(),
    sameCrossing
  );
  const auto inside = std::mismatch(
    scanned.inside.begin(),
    scanned.inside.end(),
    reference.inside.begin(),
    reference.inside.end()
  );

  CAPTURE(
    name,
    scanned.crossings.size(),
    reference.crossings.size(),
    crossing.first - scanned.crossings.begin(),
    scanned.inside.size(),
    reference.inside.size(),
    inside.first - scanned.inside.begin()
  );
  CHECK(!reference.crossings.empty());
  CHECK(crossing.first == scanned.crossings.end());
  CHECK(crossing.second == reference.crossings.end());
  CHECK(inside.first == scanned.inside.end());
  CHECK(inside.second == reference.inside.end());
}

}

TEST_CASE(
  "the lattice surface read by brick matches the full scan",
  "[fieldshape]"
) {
  const glm::mat4 aboutY = glm::rotate(
    glm::mat4(1.0f),
    glm::radians(20.0f),
    glm::vec3(0.0f, 1.0f, 0.0f)
  );
  const glm::mat4 aboutDiagonal = glm::rotate(
    glm::mat4(1.0f),
    glm::radians(20.0f),
    glm::normalize(glm::vec3(1.0f))
  );

  for (const uint32_t samples : {9u, 17u, 33u}) {
    CAPTURE(samples);

    checkMatchesFullScan(
      "aligned block",
      bakeBlock(glm::vec3(STACK_HALF), samples)
    );
    checkMatchesFullScan(
      "block turned about y",
      bakeTurnedBlock(glm::vec3(STACK_HALF), samples, aboutY)
    );
    checkMatchesFullScan(
      "block turned about the diagonal",
      bakeTurnedBlock(glm::vec3(STACK_HALF), samples, aboutDiagonal)
    );
  }

  for (const uint32_t samples : {9u, 17u, 33u, 65u}) {
    CAPTURE(samples);
    checkMatchesFullScan("sphere", bakeSphere(0.5f, samples));
  }

  checkMatchesFullScan("ground chunk", bakeGroundChunk());

  checkMatchesFullScan("corner block", bakeCornerBlock(DENTED_SAMPLES));

  const SampledSdf faceBrick = bakeCornerBlock(33u);

  REQUIRE(faceBrick.brickMaximum.back() < 0.0f);

  checkMatchesFullScan("corner block with a solid brick on a face", faceBrick);

  checkMatchesFullScan(
    "pile slab with its faces on sample planes",
    bakePileSlab(PILE_SLAB_REACH_VOXELS)
  );

  const float voxel = dentedVoxel();

  checkMatchesFullScan("groove", bakeGroove());
  checkMatchesFullScan(
    "off-centre dent",
    bakeCarvedBlock(offCentreDent(voxel), DENTED_SAMPLES)
  );
  checkMatchesFullScan(
    "closed pocket",
    bakeCarvedBlock(closedPocket(voxel), DENTED_SAMPLES)
  );
  checkMatchesFullScan("fin on a lattice plane", bakeFin(0.0f));
  checkMatchesFullScan(
    "step through the lattice's edges",
    bakeEdgeStep(DENTED_SAMPLES)
  );

  SampledSdf unbricked = bakeSphere(0.5f, DENTED_SAMPLES);

  unbricked.brickMinimum.clear();
  unbricked.brickMaximum.clear();

  checkMatchesFullScan("sphere without brick bounds", unbricked);
}

TEST_CASE(
  "a convex crate meets a non-convex field through the field pipeline",
  "[fieldshape]"
) {
  constexpr float SINK = 0.001f;
  constexpr float DROP = 0.3f;
  constexpr uint32_t FRAMES = 600u;

  JoltLibrary library;

  const SampledSdf grooveField = bakeGroove();
  const SampledSdf crateField = bakeBlock(glm::vec3(STACK_HALF), 9u);

  const FieldShape groove(grooveField);
  const FieldShape crate(crateField);

  REQUIRE(groove.hull() == nullptr);
  REQUIRE(crate.hull() != nullptr);

  const float touch = grooveTouch(crate, grooveField);

  const std::vector<JPH::CollideShapeResult> hits = hitsPosed(
    crate,
    groove,
    JPH::Mat44::sTranslation(
      JPH::Vec3(0.0f, touch - SINK, 0.0f) + crate.GetCenterOfMass()
    ),
    JPH::Mat44::sTranslation(groove.GetCenterOfMass())
  );

  const float within = 0.5f * GROOVE_DEGREES;

  float left = 180.0f;
  float right = 180.0f;

  for (const JPH::CollideShapeResult& hit : hits) {
    left = std::min(left, degreesFrom(hit.mPenetrationAxis, intoSlab(-1.0f)));
    right = std::min(right, degreesFrom(hit.mPenetrationAxis, intoSlab(1.0f)));
  }

  INFO(
    "touch " << touch << " hits " << hits.size() << " left " << left
             << " right " << right
  );

  CHECK(left <= within);
  CHECK(right <= within);

  talus::physics::PhysicsWorld world;
  JPH::BodyInterface& bodies = world.bodies();

  const JPH::BodyID ground = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      new FieldShape(grooveField),
      JPH::RVec3::sZero(),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );

  JPH::BodyCreationSettings dropped(
    new FieldShape(crateField),
    JPH::RVec3(0.0f, touch + DROP, 0.0f),
    JPH::Quat::sIdentity(),
    JPH::EMotionType::Dynamic,
    talus::physics::ObjectLayers::MOVING
  );

  dropped.mMotionQuality = JPH::EMotionQuality::LinearCast;

  const JPH::BodyID box =
    bodies.CreateAndAddBody(dropped, JPH::EActivation::Activate);

  world.optimizeBroadPhase();

  uint32_t asleepAt = 0u;

  for (uint32_t frame = 1u; frame <= FRAMES && asleepAt == 0u; ++frame) {
    world.step();

    if (!bodies.IsActive(box)) {
      asleepAt = frame;
    }
  }

  const float height = bodies.GetPosition(box).GetY();

  INFO("asleep at " << asleepAt << " height " << height);

  CHECK(asleepAt != 0u);
  CHECK(std::abs(height - touch) <= STACK_HALF);

  bodies.RemoveBody(box);
  bodies.DestroyBody(box);
  bodies.RemoveBody(ground);
  bodies.DestroyBody(ground);
}

TEST_CASE(
  "an edit rebuilds the hull and drops it when the field stops being convex",
  "[fieldshape]"
) {
  constexpr uint32_t SAMPLES = 17u;

  JoltLibrary library;

  const glm::vec3 axis = glm::normalize(glm::vec3(1.0f));
  const float offset = 2.0f * STACK_HALF / std::sqrt(3.0f);

  SampledSdf cut = bakeBlock(glm::vec3(STACK_HALF), SAMPLES);

  const float voxel = cut.voxelSize.x;

  const JPH::Ref<FieldShape> whole = new FieldShape(cut);

  REQUIRE(whole->hull() != nullptr);
  REQUIRE(furthestAlong(*whole->hull(), axis) > offset + voxel);

  const talus::field::WriteReport cutReport =
    talus::field::deformAndRepair(cut, cornerCut(axis, offset)).write;

  const JPH::Ref<FieldShape> trimmed =
    new FieldShape(cut, *whole, cutReport.brickBegin, cutReport.brickEnd);

  REQUIRE(trimmed->hull() != nullptr);

  const float reach = furthestAlong(*trimmed->hull(), axis);

  INFO("reach along the cut " << reach << " cut at " << offset);

  CHECK(reach <= offset + voxel);

  for (const glm::vec3& corner :
       {glm::vec3(STACK_HALF, STACK_HALF, 0.0f),
        glm::vec3(STACK_HALF, 0.0f, STACK_HALF),
        glm::vec3(0.0f, STACK_HALF, STACK_HALF)}) {
    CAPTURE(corner.x, corner.y, corner.z);
    CHECK(nearestVertex(*trimmed->hull(), corner) <= voxel);
  }

  SampledSdf dented = bakeBlock(glm::vec3(STACK_HALF), SAMPLES);

  const JPH::Ref<FieldShape> flat = new FieldShape(dented);

  REQUIRE(flat->hull() != nullptr);

  const talus::field::WriteReport dentReport =
    talus::field::deformAndRepair(dented, topDent(STACK_HALF, voxel)).write;

  const JPH::Ref<FieldShape> hollowed =
    new FieldShape(dented, *flat, dentReport.brickBegin, dentReport.brickEnd);

  CHECK(hollowed->hull() == nullptr);
}

TEST_CASE(
  "a non-convex field is judged before a 3D hull is built",
  "[fieldshape]"
) {
  JoltLibrary library;

  const std::vector<NamedField> fields = nonConvexFields();

  uint32_t built = 0u;

  for (const NamedField& field : fields) {
    const talus::physics::FieldHull hull =
      talus::physics::convexHullOf(field.field);

    UNSCOPED_INFO(
      field.name << ": samples " << field.field.resolution.x << " crossings "
                 << hull.crossings << " hull points " << hull.points
                 << " built " << hull.built
    );

    CAPTURE(field.name);
    CHECK(hull.shape.GetPtr() == nullptr);
    CHECK_FALSE(hull.built);

    built += hull.built ? 1u : 0u;
  }

  UNSCOPED_INFO(
    built << " 3D hull builds over " << fields.size() << " non-convex fields"
  );
  REQUIRE(built == 0u);
}

namespace {

JPH::Mat44 centreOf(const JPH::Shape& shape, JPH::Mat44Arg body) {
  return body.PreTranslated(shape.GetCenterOfMass());
}

std::vector<JPH::CollideShapeResult> collidedAt(
  const JPH::Shape& one,
  const JPH::Shape& two,
  JPH::Mat44Arg bodyOne,
  JPH::Mat44Arg bodyTwo
) {
  return hitsPosed(one, two, centreOf(one, bodyOne), centreOf(two, bodyTwo));
}

std::vector<JPH::ShapeCastResult> castAt(
  const JPH::Shape& moving,
  const JPH::Shape& fixed,
  JPH::Mat44Arg bodyMoving,
  JPH::Mat44Arg bodyFixed,
  JPH::Vec3Arg direction
) {
  const JPH::TransformedShape context{};

  JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
  collector.SetContext(&context);

  JPH::ShapeCastSettings settings;
  settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
  settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;

  JPH::CollisionDispatch::sCastShapeVsShapeWorldSpace(
    JPH::ShapeCast::sFromWorldTransform(
      &moving,
      JPH::Vec3::sReplicate(1.0f),
      bodyMoving,
      direction
    ),
    settings,
    &fixed,
    JPH::Vec3::sReplicate(1.0f),
    JPH::ShapeFilter(),
    centreOf(fixed, bodyFixed),
    JPH::SubShapeIDCreator(),
    JPH::SubShapeIDCreator(),
    collector
  );

  return {collector.mHits.begin(), collector.mHits.end()};
}

struct PlacementRole {
  const char* name = "";
  const FieldShape* one = nullptr;
  const FieldShape* two = nullptr;
  JPH::Mat44 onePressed = JPH::Mat44::sIdentity();
  JPH::Mat44 twoPressed = JPH::Mat44::sIdentity();
  JPH::Mat44 oneApart = JPH::Mat44::sIdentity();
  JPH::Mat44 twoApart = JPH::Mat44::sIdentity();
  JPH::Vec3 approach = JPH::Vec3::sZero();
};

}

TEST_CASE(
  "an off-centre convex pair collides and casts like its hulls as Jolt bodies",
  "[fieldshape]"
) {
  constexpr uint32_t SAMPLES = 9u;
  constexpr float PRESSED = 0.005f;
  constexpr float GAP = 0.1f;

  JoltLibrary library;

  const float tolerance = JPH::cDefaultCollisionTolerance;

  SampledSdf wedgeField = bakeBlock(glm::vec3(STACK_HALF), SAMPLES);

  REQUIRE(
    talus::field::deformAndRepair(
      wedgeField,
      cornerCut(glm::normalize(glm::vec3(1.0f)), 0.3f * STACK_HALF)
    )
      .cellsRemoved
    > 0u
  );

  const SampledSdf blockField = bakeBlock(glm::vec3(STACK_HALF), SAMPLES);

  const FieldShape wedge(wedgeField);
  const FieldShape block(blockField);

  REQUIRE(wedge.hull() != nullptr);
  REQUIRE(block.hull() != nullptr);

  const JPH::Vec3 offset =
    wedge.hull()->GetCenterOfMass() - wedge.GetCenterOfMass();

  UNSCOPED_INFO(
    "hull centre of mass minus the field's "
    << offset.GetX() << " " << offset.GetY() << " " << offset.GetZ()
  );
  REQUIRE(offset.Length() > 10.0f * tolerance);

  const JPH::Quat turned =
    JPH::Quat::sRotation(JPH::Vec3(0.3f, 1.0f, 0.2f).Normalized(), 0.6f);

  const JPH::Mat44 below = JPH::Mat44::sIdentity();

  float lowest = std::numeric_limits<float>::max();

  for (JPH::uint index = 0u; index != wedge.hull()->GetNumPoints(); ++index) {
    lowest = std::min(
      lowest,
      (turned
       * (wedge.hull()->GetPoint(index) + wedge.hull()->GetCenterOfMass()))
        .GetY()
    );
  }

  const float top = furthestAlong(*block.hull(), glm::vec3(0.0f, 1.0f, 0.0f));

  const JPH::Mat44 pressed = JPH::Mat44::sRotationTranslation(
    turned,
    JPH::Vec3(0.01f, top - lowest - PRESSED, -0.02f)
  );

  const JPH::Mat44 above = JPH::Mat44::sRotationTranslation(
    turned,
    JPH::Vec3(0.01f, top - lowest + GAP, -0.02f)
  );
  const JPH::Vec3 fall(0.0f, -2.0f * GAP, 0.0f);

  const std::array<PlacementRole, 2> roles{
    PlacementRole{
      "the off-centre body moving, as shape 1",
      &wedge,
      &block,
      pressed,
      below,
      above,
      below,
      fall
    },
    PlacementRole{
      "the off-centre body fixed, as shape 2",
      &block,
      &wedge,
      below,
      pressed,
      below,
      above,
      -fall
    }
  };

  for (const PlacementRole& role : roles) {
    CAPTURE(role.name);

    const std::vector<JPH::CollideShapeResult> fieldHits =
      collidedAt(*role.one, *role.two, role.onePressed, role.twoPressed);
    const std::vector<JPH::CollideShapeResult> hullHits = collidedAt(
      *role.one->hull(),
      *role.two->hull(),
      role.onePressed,
      role.twoPressed
    );

    REQUIRE(fieldHits.size() == 1u);
    REQUIRE(hullHits.size() == 1u);

    const JPH::CollideShapeResult& fieldHit = fieldHits.front();
    const JPH::CollideShapeResult& hullHit = hullHits.front();

    const float normalApart = degreesFrom(
      fieldHit.mPenetrationAxis,
      hullHit.mPenetrationAxis.Normalized()
    );

    UNSCOPED_INFO(
      role.name << ": collide depth " << fieldHit.mPenetrationDepth
                << " against " << hullHit.mPenetrationDepth << ", normals "
                << normalApart << " degrees apart"
    );

    CHECK(hullHit.mPenetrationDepth > 0.0f);
    CHECK(normalApart <= CRITERION_DEGREES);
    CHECK(
      std::abs(fieldHit.mPenetrationDepth - hullHit.mPenetrationDepth)
      <= tolerance
    );
    CHECK(
      (fieldHit.mContactPointOn1 - hullHit.mContactPointOn1).Length()
      <= tolerance
    );
    CHECK(
      (fieldHit.mContactPointOn2 - hullHit.mContactPointOn2).Length()
      <= tolerance
    );

    const std::vector<JPH::ShapeCastResult> fieldCast =
      castAt(*role.one, *role.two, role.oneApart, role.twoApart, role.approach);
    const std::vector<JPH::ShapeCastResult> hullCast = castAt(
      *role.one->hull(),
      *role.two->hull(),
      role.oneApart,
      role.twoApart,
      role.approach
    );

    REQUIRE(fieldCast.size() == 1u);
    REQUIRE(hullCast.size() == 1u);

    const float castApart = degreesFrom(
      fieldCast.front().mPenetrationAxis,
      hullCast.front().mPenetrationAxis.Normalized()
    );

    UNSCOPED_INFO(
      role.name << ": cast fraction " << fieldCast.front().mFraction
                << " against " << hullCast.front().mFraction << ", normals "
                << castApart << " degrees apart"
    );

    CHECK(castApart <= CRITERION_DEGREES);
    CHECK(
      std::abs(fieldCast.front().mFraction - hullCast.front().mFraction)
      <= tolerance / role.approach.Length()
    );
    CHECK(
      (fieldCast.front().mContactPointOn2 - hullCast.front().mContactPointOn2)
        .Length()
      <= tolerance
    );
  }
}

TEST_CASE(
  "a cast onto a notched field meets its far face through the field pipeline",
  "[fieldshape]"
) {
  constexpr uint32_t SAMPLES = 9u;
  constexpr float GAP = 0.1f;

  JoltLibrary library;

  const SampledSdf notchedField = bakeCarvedBlock(topNotch(), SAMPLES);
  const SampledSdf blockField = bakeBlock(glm::vec3(SMALL_HALF), SAMPLES);

  const FieldShape notched(notchedField);
  const FieldShape block(blockField);

  REQUIRE(notched.hull() == nullptr);

  const JPH::Vec3 rise(0.0f, 2.0f * GAP, 0.0f);

  const std::vector<JPH::ShapeCastResult> casts = castAt(
    block,
    notched,
    JPH::Mat44::sTranslation(
      JPH::Vec3(0.0f, -STACK_HALF - SMALL_HALF - GAP, 0.0f)
    ),
    JPH::Mat44::sIdentity(),
    rise
  );

  REQUIRE(casts.size() == 1u);

  const JPH::ShapeCastResult& cast = casts.front();

  const float firstContact = GAP / rise.Length();
  const float normalApart =
    degreesFrom(cast.mPenetrationAxis, JPH::Vec3::sAxisY());

  INFO(
    "fraction " << cast.mFraction << " against " << firstContact << ", normal "
                << normalApart << " degrees from the approach"
  );

  CHECK(
    std::abs(cast.mFraction - firstContact)
    <= notchedField.voxelSize.x / rise.Length()
  );
  CHECK(normalApart <= CRITERION_DEGREES);
}

TEST_CASE(
  "an all-solid pile bar has its grid box as hull and meets the pile slab "
  "through it",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf slabField = bakePileSlab(PILE_SLAB_REACH_VOXELS);
  const FieldShape slab(slabField);

  REQUIRE(slab.hull() != nullptr);

  for (const float voxel : PILE_BAR_VOXELS) {
    CAPTURE(voxel);

    const SampledSdf barField = bakePileBar(voxel, false);

    REQUIRE(std::ranges::all_of(barField.distances, [](float sampled) {
      return sampled < 0.0f;
    }));

    const FieldShape bar(barField);

    REQUIRE(bar.hull() != nullptr);

    const JPH::AABox bounds = bar.hull()->GetLocalBounds();
    const JPH::Vec3 centre = bar.hull()->GetCenterOfMass();
    const JPH::Vec3 low = bounds.mMin + centre;
    const JPH::Vec3 high = bounds.mMax + centre;
    const glm::vec3 gridHigh =
      barField.origin
      + barField.voxelSize * glm::vec3(barField.resolution - glm::uvec3(1u));

    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      CAPTURE(axis);
      CHECK_THAT(low[axis], WithinAbs(barField.origin[axis], 1.0e-6f));
      CHECK_THAT(high[axis], WithinAbs(gridHigh[axis], 1.0e-6f));
    }

    const std::vector<JPH::CollideShapeResult> hits = hitsPosed(
      bar,
      slab,
      centreOf(bar, barOnPileSlab(bar, PROBE_POSE, PROBE_SINKS.front())),
      centreOf(slab, pileSlabAt())
    );

    CHECK_FALSE(hits.empty());
  }
}

TEST_CASE(
  "a slab whose faces lie on sample planes has its hull on its faces",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf slabField = bakePileSlab(PILE_SLAB_REACH_VOXELS);

  REQUIRE(slabField.resolution.y == 6u);
  REQUIRE_THAT(
    slabField.origin.y + 2.0f * slabField.voxelSize.y,
    WithinAbs(-PILE_SLAB_HALF.y, 1.0e-6f)
  );
  REQUIRE_THAT(
    slabField.origin.y + 3.0f * slabField.voxelSize.y,
    WithinAbs(PILE_SLAB_HALF.y, 1.0e-6f)
  );

  const FieldShape slab(slabField);

  REQUIRE(slab.hull() != nullptr);

  const float top =
    PILE_SLAB_CENTRE + furthestAlong(*slab.hull(), glm::vec3(0.0f, 1.0f, 0.0f));
  const float bottom =
    PILE_SLAB_CENTRE
    - furthestAlong(*slab.hull(), glm::vec3(0.0f, -1.0f, 0.0f));

  INFO("hull top " << top << " bottom " << bottom);

  CHECK_THAT(top, WithinAbs(PILE_SLAB_CENTRE + PILE_SLAB_HALF.y, 1.0e-5f));
  CHECK_THAT(bottom, WithinAbs(PILE_SLAB_CENTRE - PILE_SLAB_HALF.y, 1.0e-5f));
}

TEST_CASE(
  "a flat bar over the split top of a slab's hull meets it on three points "
  "or more",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf slabField = bakePileSlab(SPLIT_TOP_REACH_VOXELS);
  const FieldShape slab(slabField);

  REQUIRE(slab.hull() != nullptr);
  REQUIRE(upwardFaces(*slab.hull()) >= 2u);

  const JPH::Mat44 slabAt = centreOf(slab, pileSlabAt());

  for (const float voxel : PILE_BAR_VOXELS) {
    const SampledSdf barField = bakePileBar(voxel, true);
    const FieldShape bar(barField);

    REQUIRE(bar.hull() != nullptr);

    for (const float sink : PROBE_SINKS) {
      const JPH::Mat44 barAt =
        centreOf(bar, barOnPileSlab(bar, PROBE_POSE, sink));

      for (const bool slabFirst : {true, false}) {
        const std::vector<Manifold> manifolds = manifoldsOf(
          slabFirst ? hitsPosed(slab, bar, slabAt, barAt)
                    : hitsPosed(bar, slab, barAt, slabAt)
        );

        const JPH::Vec3 expected =
          slabFirst ? JPH::Vec3::sAxisY() : -JPH::Vec3::sAxisY();
        const size_t points =
          manifolds.empty() ? 0u : manifolds.front().on1.size();
        const float normal =
          manifolds.empty() ? 180.0f
                            : degreesFrom(manifolds.front().normal, expected);

        CAPTURE(voxel, sink, slabFirst, manifolds.size(), points, normal);
        CHECK(manifolds.size() == 1u);
        CHECK(points >= 3u);
        CHECK(normal <= CRITERION_DEGREES);
      }
    }
  }
}

TEST_CASE(
  "the pile bar rests flat and sleeps on the pile slab as a Jolt body",
  "[fieldshape]"
) {
  JoltLibrary library;

  const SampledSdf slabField = bakePileSlab(PILE_SLAB_REACH_VOXELS);

  for (const float voxel : PILE_BAR_VOXELS) {
    const SampledSdf barField = bakePileBar(voxel, false);

    for (const PilePose& pose : {PROBE_POSE, CENTRE_POSE}) {
      const PileRest rest = restOnPileSlab(slabField, barField, pose);

      CAPTURE(voxel, pose.x, pose.z, pose.yaw);
      INFO(
        "asleep at " << rest.asleepAt << " of " << rest.allowed
                     << " allowed, centre " << rest.centre << ", lowest "
                     << rest.lowest << ", fastest turn " << rest.fastestTurn
                     << " rad/s, slop " << rest.slop
      );

      CHECK(rest.asleepAt != 0u);
      CHECK(rest.asleepAt <= rest.allowed);
      CHECK(rest.lowest >= -rest.slop);
      CHECK(rest.lowest <= rest.slop);
      CHECK(rest.fastestTurn <= PILE_TURN_LIMIT);
    }
  }
}

namespace {

constexpr std::array<std::array<float, 3>, 28> ROLLED_BAR_HULL{
  {{-0.200074986f, 0.0257225838f, 0.0248540342f},
   {0.198658571f, 0.0260115862f, 0.0253642406f},
   {0.19890736f, 0.026286507f, -0.0246078242f},
   {-0.200866267f, -0.0242187232f, 0.0251018349f},
   {-0.201076999f, -0.0239327885f, -0.024932541f},
   {-0.200073019f, 0.0255287196f, -0.0251321848f},
   {0.199712202f, -0.0232014172f, 0.0253921952f},
   {0.200050697f, -0.0232779402f, -0.0248720944f},
   {0.187232271f, -0.0243152399f, 0.0252523366f},
   {0.0120310048f, 0.0229592826f, -0.0253883377f},
   {0.1125504f, -0.025723407f, -0.0253350884f},
   {0.187552795f, -0.0240361802f, -0.0251185149f},
   {0.124897882f, -0.0258986522f, 0.0246816147f},
   {0.0747919977f, -0.0268012546f, 0.0245432258f},
   {-0.200903192f, -0.0243844204f, 0.000112649061f},
   {0.0246124882f, -0.0270286389f, 0.024629876f},
   {-0.0757313147f, -0.0260078516f, 0.0250848625f},
   {0.0622970313f, -0.02672516f, -0.0128708566f},
   {0.0119362939f, -0.0269423462f, -0.0252300426f},
   {0.0622971803f, -0.0265485942f, -0.0253813863f},
   {0.199842334f, -0.0110761952f, -0.0249887295f},
   {-0.0383067355f, -0.0267712884f, -0.0250211358f},
   {-0.0758517087f, -0.0263767373f, -0.0249283053f},
   {-0.163365319f, -0.025045637f, -0.0123882368f},
   {-0.125873446f, -0.0255474988f, -0.0249192435f},
   {-0.0381776281f, -0.0266092941f, 0.0124397511f},
   {0.199770257f, -0.0111782681f, 0.000182768854f},
   {0.0496989824f, -0.0270290263f, 0.0245431941f}}
};

constexpr float ROLLED_SLAB_HALF_X = 7.19999981f;
constexpr float ROLLED_SLAB_HALF_Y = 0.150000021f;
constexpr float ROLLED_SLAB_HALF_Z = 4.3238883f;
constexpr float ROLLED_SLAB_X = -2.48455596f;
constexpr float ROLLED_SLAB_Y = -0.186262771f;
constexpr float ROLLED_SLAB_Z = 0.828947842f;
constexpr int ROLLED_STEPS = 2000;
constexpr float ROLLED_STEP = 1.0e-6f;
constexpr float ROLLED_AXIS = 0.99f;
constexpr float ROLLED_DEPTH = 1.0e-5f;

JPH::RefConst<JPH::ConvexHullShape> rolledHull(
  std::span<const JPH::Vec3> points
) {
  JPH::ConvexHullShapeSettings settings(
    points.data(),
    static_cast<int>(points.size()),
    0.0f
  );

  settings.mHullTolerance = 0.0f;

  const JPH::ShapeSettings::ShapeResult made = settings.Create();

  REQUIRE(made.IsValid());

  return JPH::RefConst<JPH::ConvexHullShape>(
    static_cast<const JPH::ConvexHullShape*>(made.Get().GetPtr())
  );
}

JPH::Mat44 rolledCentre(const FieldShape& shape, JPH::Mat44Arg hullAt) {
  return hullAt.PreTranslated(
    shape.GetCenterOfMass() - shape.hull()->GetCenterOfMass()
  );
}

}

TEST_CASE(
  "a rolled hull within a millimetre of a slab hull is pushed out of the slab "
  "at every height",
  "[fieldshape]"
) {
  JoltLibrary library;

  std::vector<JPH::Vec3> barPoints;

  for (const std::array<float, 3>& point : ROLLED_BAR_HULL) {
    barPoints.emplace_back(point[0], point[1], point[2]);
  }

  std::vector<JPH::Vec3> slabPoints;

  for (const float x : {-ROLLED_SLAB_HALF_X, ROLLED_SLAB_HALF_X}) {
    for (const float y : {-ROLLED_SLAB_HALF_Y, ROLLED_SLAB_HALF_Y}) {
      for (const float z : {-ROLLED_SLAB_HALF_Z, ROLLED_SLAB_HALF_Z}) {
        slabPoints.emplace_back(x, y, z);
      }
    }
  }

  const JPH::RefConst<JPH::ConvexHullShape> barHull = rolledHull(barPoints);
  const JPH::RefConst<JPH::ConvexHullShape> slabHull = rolledHull(slabPoints);

  REQUIRE(barHull->GetNumPoints() == ROLLED_BAR_HULL.size());
  REQUIRE(slabHull->GetNumPoints() == 8u);

  const SampledSdf barField = talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(0.2f, 0.025f, 0.025f))
    },
    glm::vec3(-0.25f, -0.075f, -0.075f),
    glm::vec3(0.25f, 0.075f, 0.075f),
    glm::uvec3(21u, 7u, 7u)
  );
  const SampledSdf slabField = talus::field::bake(
    std::vector<Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(0.5f, 0.15f, 0.5f))
    },
    glm::vec3(-0.75f, -0.35f, -0.75f),
    glm::vec3(0.75f, 0.35f, 0.75f),
    glm::uvec3(16u, 8u, 16u)
  );

  const FieldShape barAuthored(barField);
  const FieldShape slabAuthored(slabField);
  const JPH::Ref<FieldShape> bar = new FieldShape(barAuthored, barHull);
  const JPH::Ref<FieldShape> slab = new FieldShape(slabAuthored, slabHull);

  REQUIRE(bar->hull() != nullptr);
  REQUIRE(slab->hull() != nullptr);

  JPH::Mat44 barAt = JPH::Mat44::sIdentity();

  barAt.SetColumn4(
    0,
    JPH::Vec4(0.990068793f, 0.0121816024f, -0.14005506f, 0.0f)
  );
  barAt.SetColumn4(
    1,
    JPH::Vec4(0.0912420079f, 0.702228904f, 0.706080437f, 0.0f)
  );
  barAt.SetColumn4(2, JPH::Vec4(0.1069519f, -0.711847067f, 0.694143355f, 0.0f));
  barAt.SetColumn4(
    3,
    JPH::Vec4(0.000590533018f, 0.000544311421f, 0.000633112621f, 1.0f)
  );

  float lowest = std::numeric_limits<float>::max();

  for (const JPH::Vec3& point : barPoints) {
    lowest = std::min(lowest, (barAt * point).GetY());
  }

  uint32_t hits = 0u;
  uint32_t wrong = 0u;
  int firstStep = 0;
  float firstAxis = 0.0f;
  float firstDepth = 0.0f;
  float firstGap = 0.0f;

  for (int step = -ROLLED_STEPS; step <= ROLLED_STEPS; ++step) {
    const float slabY = ROLLED_SLAB_Y + ROLLED_STEP * static_cast<float>(step);
    const JPH::Mat44 slabAt =
      JPH::Mat44::sTranslation(JPH::Vec3(ROLLED_SLAB_X, slabY, ROLLED_SLAB_Z));
    const float gap = lowest - (slabY + ROLLED_SLAB_HALF_Y);

    for (const JPH::CollideShapeResult& hit : hitsPosed(
           *bar,
           *slab,
           rolledCentre(*bar, barAt),
           rolledCentre(*slab, slabAt)
         )) {
      ++hits;

      const JPH::Vec3 axis = hit.mPenetrationAxis.Normalized();

      if (
        axis.GetY() < -ROLLED_AXIS
        && std::fabs(hit.mPenetrationDepth + gap) <= ROLLED_DEPTH
      ) {
        continue;
      }

      if (wrong == 0u) {
        firstStep = step;
        firstAxis = axis.GetY();
        firstDepth = hit.mPenetrationDepth;
        firstGap = gap;
      }

      ++wrong;
    }
  }

  INFO(
    "hits " << hits << " wrong " << wrong << " first step " << firstStep
            << " axis y " << firstAxis << " depth " << firstDepth << " gap "
            << firstGap
  );

  REQUIRE(hits == 2u * ROLLED_STEPS + 1u);
  CHECK(wrong == 0u);
}

namespace {

constexpr uint32_t AXIS_HULL_POINTS = 256u;
constexpr float AXIS_HULL_RADIUS = 0.1f;
constexpr float AXIS_HULL_OVERLAP = 1.0e-3f;
constexpr uint32_t AXIS_REPEATS = 3u;

std::vector<JPH::Vec3> spherePoints(uint32_t count, float radius) {
  const float golden = glm::pi<float>() * (3.0f - std::sqrt(5.0f));

  std::vector<JPH::Vec3> points;

  for (uint32_t index = 0u; index != count; ++index) {
    const float height =
      1.0f
      - 2.0f * (static_cast<float>(index) + 0.5f) / static_cast<float>(count);
    const float ring = std::sqrt(1.0f - height * height);
    const float angle = golden * static_cast<float>(index);

    points.emplace_back(
      radius * ring * std::cos(angle),
      radius * height,
      radius * ring * std::sin(angle)
    );
  }

  return points;
}

}

TEST_CASE(
  "the replacement axis of two 256-point hulls points along their overlap and "
  "reports its cost",
  "[.][fieldshape]"
) {
  JoltLibrary library;

  const std::vector<JPH::Vec3> points =
    spherePoints(AXIS_HULL_POINTS, AXIS_HULL_RADIUS);
  const JPH::RefConst<JPH::ConvexHullShape> hull = rolledHull(points);

  REQUIRE(hull->GetNumPoints() == AXIS_HULL_POINTS);

  float widest = std::numeric_limits<float>::lowest();
  float narrowest = std::numeric_limits<float>::max();

  for (JPH::uint point = 0u; point != hull->GetNumPoints(); ++point) {
    widest = std::max(widest, hull->GetPoint(point).GetX());
    narrowest = std::min(narrowest, hull->GetPoint(point).GetX());
  }

  const JPH::Mat44 firstAt = JPH::Mat44::sIdentity();
  const JPH::Mat44 secondAt = JPH::Mat44::sTranslation(
    JPH::Vec3(widest - narrowest - AXIS_HULL_OVERLAP, 0.0f, 0.0f)
  );

  talus::physics::HullAxis found;
  double fastest = std::numeric_limits<double>::max();

  for (uint32_t repeat = 0u; repeat != AXIS_REPEATS; ++repeat) {
    const auto began = std::chrono::steady_clock::now();

    found = talus::physics::hullSeparatingAxis(
      *hull,
      firstAt,
      *hull,
      secondAt,
      JPH::Vec3(0.0f, 1.0f, 0.0f)
    );

    fastest = std::min(
      fastest,
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - began
      )
        .count()
    );
  }

  std::cout << "hullaxis: points " << hull->GetNumPoints() << ' '
            << hull->GetNumPoints() << " faces " << hull->GetNumFaces()
            << " candidates " << found.candidates << " milliseconds " << fastest
            << " axis " << found.axis.GetX() << ' ' << found.axis.GetY() << ' '
            << found.axis.GetZ() << " overlap " << found.overlap << std::endl;

  CHECK(found.axis.GetX() > 0.99f);
  CHECK(found.overlap > 0.0f);
  CHECK(found.overlap <= AXIS_HULL_OVERLAP * 1.001f);
}
