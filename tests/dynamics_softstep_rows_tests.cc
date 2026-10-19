#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/ledger/ledger.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace {

constexpr double box3dStack10cmDrift = 0.007464;
constexpr double box3dStack10cmPeak = 0.012207;
constexpr double box3dStack10cmSink = 0.025539;
constexpr double box3dStack1mDrift = 0.007465;
constexpr double box3dStack1mPeak = 0.012210;
constexpr double box3dStack1mSink = 0.025536;
constexpr double box3dPyramid10cmDrift = 0.000146;
constexpr double box3dPyramid10cmPeak = 0.000991;
constexpr double box3dPyramid10cmSink = 0.007140;
constexpr double box3dPyramid1mDrift = 0.000091;
constexpr double box3dPyramid1mPeak = 0.001401;
constexpr double box3dPyramid1mSink = 0.007265;
constexpr double box3dHeavy100LightSink = 0.006972;
constexpr double box3dHeavy100HeavySink = 0.034702;
constexpr double box3dHeavy1000LightSink = 0.020181;
constexpr double box3dHeavy1000HeavySink = 0.100050;
constexpr double box3dRestBallDrift = 0.000040;
constexpr double box3dRestBallSink = 0.000069;
constexpr double box3dSlide = 0.40417;
constexpr double box3dPlateFinal = 0.04993;
constexpr double box3dSmallOnBigSmallDrift = 0.014247;
constexpr double box3dSmallOnBigSmallPeak = 0.021276;
constexpr double box3dSmallOnBigSmallSink = 0.027696;
constexpr double box3dBigOnSmallLargeDrift = 0.004645;
constexpr double box3dBigOnSmallLargePeak = 0.016116;
constexpr double box3dBigOnSmallLargeSink = 0.086441;

struct RowResult {
  double drift = 0.0;
  double peak = 0.0;
  double fastest = 0.0;
  double sink = 0.0;
  double topDrop = 0.0;
  double energyRatio = 1.0;
  double millisecondsPerFrame = 0.0;
};

talus::field::SampledSdf boxShape(const glm::dvec3& half) {
  const double voxel = std::min({half.x, half.y, half.z}) / 5.0;
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

talus::field::SampledSdf sphereShape(double radius) {
  const double voxel = radius / 5.0;
  const float reach = static_cast<float>(radius + 2.0 * voxel);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::sphere(glm::vec3(0.0f), static_cast<float>(radius))
    },
    glm::vec3(-reach),
    glm::vec3(reach),
    glm::uvec3(
      static_cast<uint32_t>(std::round(2.0f * reach / static_cast<float>(voxel)))
      + 1u
    )
  );
}

const talus::field::SampledSdf& groundShape() {
  static const talus::field::SampledSdf ground = [] {
    const glm::vec3 centre(0.0f, -0.5f, 0.0f);
    const glm::vec3 half(20.0f, 0.5f, 20.0f);
    const glm::vec3 low = centre - half - glm::vec3(0.2f);
    const glm::vec3 high = centre + half + glm::vec3(0.2f);

    return talus::field::bake(
      std::vector<talus::field::Primitive>{fixture::box(centre, half)},
      low,
      high,
      glm::uvec3(glm::round((high - low) / 0.1f)) + glm::uvec3(1u)
    );
  }();

  return ground;
}

talus::field::SampledSdf plateShape() {
  const glm::vec3 low(-2.1f, -0.2f, -2.1f);
  const glm::vec3 high(2.1f, 0.4f, 2.1f);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f, -0.02f, 0.0f), glm::vec3(2.0f, 0.02f, 2.0f))
    },
    low,
    high,
    glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
  );
}

uint32_t addCube(
  talus::dynamics::World& world,
  const talus::field::SampledSdf& shape,
  double half,
  double mass,
  const glm::dvec3& position,
  const glm::dvec3& velocity
) {
  return talus::dynamics::addRigidBox(
    world.bodies,
    shape,
    glm::dvec3(half),
    mass,
    position,
    velocity,
    0.5,
    3u
  );
}

RowResult measure(
  talus::dynamics::World& world,
  double seconds,
  double judgedFrom
) {
  RowResult result;
  const glm::dvec3 g = world.parameters.gravity;
  const double start = talus::dynamics::energyOf(world.bodies, g).total();
  const uint32_t frames = static_cast<uint32_t>(std::lround(seconds * 60.0));
  const uint32_t judged = static_cast<uint32_t>(std::lround(judgedFrom * 60.0));
  const size_t count = world.bodies.rigids.size();
  std::vector<glm::dvec3> initial(count);
  std::vector<glm::dvec3> settled(count);
  size_t top = 0u;

  for (size_t index = 0u; index != count; ++index) {
    initial[index] = world.bodies.rigids[index].position;

    if (initial[index].y > initial[top].y) {
      top = index;
    }
  }

  double elapsed = 0.0;

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    const auto before = std::chrono::steady_clock::now();

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    elapsed += std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - before
    )
                 .count();

    for (size_t index = 0u; index != count; ++index) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[index];

      result.fastest = std::max(result.fastest, glm::length(body.velocity));
      result.sink = std::max(result.sink, initial[index].y - body.position.y);

      if (frame + 1u == judged) {
        settled[index] = body.position;
      } else if (frame + 1u > judged) {
        result.drift =
          std::max(result.drift, glm::length(body.position - settled[index]));
        result.peak = std::max(result.peak, glm::length(body.velocity));
      }
    }

    result.energyRatio = std::max(
      result.energyRatio,
      talus::dynamics::energyOf(world.bodies, g).total() / start
    );
  }

  result.topDrop = initial[top].y - world.bodies.rigids[top].position.y;
  result.millisecondsPerFrame = elapsed / frames;

  return result;
}

void restRule(const RowResult& result) {
  CHECK(result.drift < 0.01);
  CHECK(result.peak < 0.05);
  CHECK(result.energyRatio <= 1.02);
}

double bounce(double gap) {
  const talus::field::SampledSdf ball = sphereShape(0.05);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.05 + gap, 0.0),
    glm::dvec3(0.0, -2.0, 0.0),
    0.5,
    0.5
  );

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  return world.bodies.rigids[0].velocity.y / 2.0;
}

void stack(
  double half,
  double mass,
  const char* name,
  double drift,
  double peak,
  double sink
) {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(half));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);

  for (uint32_t level = 0u; level != 10u; ++level) {
    addCube(
      world,
      crate,
      half,
      mass,
      glm::dvec3(0.0, half * (1.0 + 2.0 * level), 0.0),
      glm::dvec3(0.0)
    );
  }

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row %s | drift %.6f peak %.6f top sink %.6f energy %.4f ms %.3f | box3d "
    "drift %.6f peak %.6f top sink %.6f\n",
    name,
    result.drift,
    result.peak,
    result.topDrop,
    result.energyRatio,
    result.millisecondsPerFrame,
    drift,
    peak,
    sink
  );
  restRule(result);
}

void pyramid(
  double half,
  double mass,
  const char* name,
  double drift,
  double peak,
  double sink
) {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(half));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);

  for (uint32_t row = 0u; row != 10u; ++row) {
    const uint32_t count = 10u - row;

    for (uint32_t index = 0u; index != count; ++index) {
      const double x = (index - 0.5 * (count - 1.0)) * 2.1 * half;

      addCube(
        world,
        crate,
        half,
        mass,
        glm::dvec3(x, half * (1.0 + 2.0 * row), 0.0),
        glm::dvec3(0.0)
      );
    }
  }

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row %s | drift %.6f peak %.6f sink %.6f energy %.4f ms %.3f | box3d drift "
    "%.6f peak %.6f sink %.6f\n",
    name,
    result.drift,
    result.peak,
    result.sink,
    result.energyRatio,
    result.millisecondsPerFrame,
    drift,
    peak,
    sink
  );
  restRule(result);
}

void heavy(
  double ratio,
  const char* name,
  double lightSink,
  double heavySink
) {
  const talus::field::SampledSdf small = boxShape(glm::dvec3(0.05));
  const talus::field::SampledSdf large = boxShape(glm::dvec3(0.1));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addCube(
    world,
    small,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.05, 0.0),
    glm::dvec3(0.0)
  );
  addCube(
    world,
    large,
    0.1,
    ratio,
    glm::dvec3(0.0, 0.2, 0.0),
    glm::dvec3(0.0)
  );

  const RowResult result = measure(world, 6.0, 1.0);
  const double light = 0.05 - world.bodies.rigids[0].position.y;
  const double load = 0.2 - world.bodies.rigids[1].position.y;

  std::printf(
    "row %s | light sink %.6f heavy sink %.6f drift %.6f fastest %.6f ms %.3f "
    "| box3d light sink %.6f heavy sink %.6f\n",
    name,
    light,
    load,
    result.drift,
    result.fastest,
    result.millisecondsPerFrame,
    lightSink,
    heavySink
  );
  CHECK(result.fastest < 10.0);
}

}

TEST_CASE("softstep row: bounce from contact", "[.][twosolverrows]") {
  const double ratio = bounce(0.0);

  std::printf("row bounce-contact | ratio %.6f | box3d ratio 0.500000\n", ratio);
  CHECK(std::abs(ratio - 0.5) <= std::max(0.10 * 0.5, 0.02));
}

TEST_CASE(
  "softstep row: bounce from half a millimetre above",
  "[.][twosolverrows]"
) {
  const double ratio = bounce(5.0e-4);

  std::printf("row bounce-above | ratio %.6f | box3d ratio 0.500000\n", ratio);
  CHECK(std::abs(ratio - 0.5) <= std::max(0.10 * 0.5, 0.02));
}

TEST_CASE("softstep row: resting ball", "[.][twosolverrows]") {
  const talus::field::SampledSdf ball = sphereShape(0.05);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.05, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.9
  );

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row rest-ball | drift %.6f peak %.6f sink %.6f energy %.4f | box3d drift "
    "%.6f sink %.6f\n",
    result.drift,
    result.peak,
    result.sink,
    result.energyRatio,
    box3dRestBallDrift,
    box3dRestBallSink
  );
  restRule(result);
}

TEST_CASE("softstep row: resting box", "[.][twosolverrows]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addCube(
    world,
    crate,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.05, 0.0),
    glm::dvec3(0.0)
  );

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row rest-box | drift %.6f peak %.6f sink %.6f energy %.4f | box3d none\n",
    result.drift,
    result.peak,
    result.sink,
    result.energyRatio
  );
  restRule(result);
}

TEST_CASE("softstep row: sliding box", "[.][twosolverrows]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addCube(
    world,
    crate,
    0.05,
    1.5,
    glm::dvec3(-0.3, 0.05, 0.0),
    glm::dvec3(2.0, 0.0, 0.0)
  );

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::RigidBody& box = world.bodies.rigids[0];

  std::printf(
    "row slide | distance %.5f final speed %.6f | coulomb %.5f box3d %.5f\n",
    box.position.x + 0.3,
    glm::length(box.velocity),
    4.0 / (2.0 * 0.5 * 9.81),
    box3dSlide
  );
}

TEST_CASE("softstep row: fast box onto a thin plate", "[.][twosolverrows]") {
  const talus::field::SampledSdf plate = plateShape();
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, plate, 0.5, 0.0);
  addCube(
    world,
    crate,
    0.05,
    1.5,
    glm::dvec3(0.0, 0.2, 0.0),
    glm::dvec3(0.0, -20.0, 0.0)
  );

  double lowestTop = 1.0;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& box = world.bodies.rigids[0];
    const glm::dmat3 turn = glm::mat3_cast(box.rotation);
    const double reach = 0.05
                         * (std::abs(turn[0][1]) + std::abs(turn[1][1])
                            + std::abs(turn[2][1]));

    lowestTop = std::min(lowestTop, box.position.y + reach);
  }

  std::printf(
    "row plate | final y %.5f lowest top %.5f | box3d final y %.5f (resting "
    "0.05, plate bottom -0.04)\n",
    world.bodies.rigids[0].position.y,
    lowestTop,
    box3dPlateFinal
  );
  CHECK(lowestTop >= -0.04);
}

TEST_CASE("softstep row: ten-high stack of 10 cm boxes", "[.][twosolverrows]") {
  stack(
    0.05,
    1.0,
    "stack-10cm",
    box3dStack10cmDrift,
    box3dStack10cmPeak,
    box3dStack10cmSink
  );
}

TEST_CASE("softstep row: ten-high stack of 1 m boxes", "[.][twosolverrows]") {
  stack(
    0.5,
    1000.0,
    "stack-1m",
    box3dStack1mDrift,
    box3dStack1mPeak,
    box3dStack1mSink
  );
}

TEST_CASE("softstep row: pyramid of 55 10 cm boxes", "[.][twosolverrows]") {
  pyramid(
    0.05,
    1.0,
    "pyramid-10cm",
    box3dPyramid10cmDrift,
    box3dPyramid10cmPeak,
    box3dPyramid10cmSink
  );
}

TEST_CASE("softstep row: pyramid of 55 1 m boxes", "[.][twosolverrows]") {
  pyramid(
    0.5,
    1000.0,
    "pyramid-1m",
    box3dPyramid1mDrift,
    box3dPyramid1mPeak,
    box3dPyramid1mSink
  );
}

TEST_CASE(
  "softstep row: heavy box on a light box, 100 to 1",
  "[.][twosolverrows]"
) {
  heavy(100.0, "heavy-100", box3dHeavy100LightSink, box3dHeavy100HeavySink);
}

TEST_CASE(
  "softstep row: heavy box on a light box, 1000 to 1",
  "[.][twosolverrows]"
) {
  heavy(1000.0, "heavy-1000", box3dHeavy1000LightSink, box3dHeavy1000HeavySink);
}

TEST_CASE("softstep row: small stack on a 1 m box", "[.][twosolverrows]") {
  const talus::field::SampledSdf small = boxShape(glm::dvec3(0.05));
  const talus::field::SampledSdf large = boxShape(glm::dvec3(0.5));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addCube(
    world,
    large,
    0.5,
    1000.0,
    glm::dvec3(0.0, 0.5, 0.0),
    glm::dvec3(0.0)
  );

  for (uint32_t level = 0u; level != 10u; ++level) {
    addCube(
      world,
      small,
      0.05,
      1.0,
      glm::dvec3(0.0, 1.05 + 0.1 * level, 0.0),
      glm::dvec3(0.0)
    );
  }

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row mixed-small-on-big | drift %.6f peak %.6f top sink %.6f energy %.4f "
    "| box3d small drift %.6f peak %.6f sink %.6f\n",
    result.drift,
    result.peak,
    result.topDrop,
    result.energyRatio,
    box3dSmallOnBigSmallDrift,
    box3dSmallOnBigSmallPeak,
    box3dSmallOnBigSmallSink
  );
  restRule(result);
}

TEST_CASE("softstep row: a 1 m box on four small boxes", "[.][twosolverrows]") {
  const talus::field::SampledSdf small = boxShape(glm::dvec3(0.05));
  const talus::field::SampledSdf large = boxShape(glm::dvec3(0.5));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);

  for (uint32_t corner = 0u; corner != 4u; ++corner) {
    const double x = corner & 1u ? 0.4 : -0.4;
    const double z = corner & 2u ? 0.4 : -0.4;

    addCube(world, small, 0.05, 1.0, glm::dvec3(x, 0.05, z), glm::dvec3(0.0));
  }

  addCube(
    world,
    large,
    0.5,
    1000.0,
    glm::dvec3(0.0, 0.6, 0.0),
    glm::dvec3(0.0)
  );

  const RowResult result = measure(world, 6.0, 1.0);
  const double largeSink = 0.6 - world.bodies.rigids[4].position.y;

  std::printf(
    "row mixed-big-on-small | drift %.6f peak %.6f large sink %.6f energy %.4f "
    "| box3d large drift %.6f peak %.6f sink %.6f\n",
    result.drift,
    result.peak,
    largeSink,
    result.energyRatio,
    box3dBigOnSmallLargeDrift,
    box3dBigOnSmallLargePeak,
    box3dBigOnSmallLargeSink
  );
  restRule(result);
}

namespace {

std::vector<talus::PhysicalMaterial> foam() {
  std::vector<talus::PhysicalMaterial> lanes(1);

  lanes[0].density = 50.0f;
  lanes[0].youngsModulus = 5.0e4f;
  lanes[0].poissonsRatio = 0.3f;

  return lanes;
}

std::vector<talus::PhysicalMaterial> ballRubber() {
  std::vector<talus::PhysicalMaterial> lanes(1);

  lanes[0].density = 1000.0f;
  lanes[0].youngsModulus = static_cast<float>(2.0 * 443260.29 * 1.45);
  lanes[0].poissonsRatio = 0.45f;

  return lanes;
}

talus::field::SampledSdf latticeField(const glm::dvec3& half, double voxel) {
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

talus::field::SampledSdf ballField(double radius, double voxel) {
  const float reach = static_cast<float>(radius + 2.0 * voxel);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::sphere(glm::vec3(0.0f), static_cast<float>(radius))
    },
    glm::vec3(-reach),
    glm::vec3(reach),
    glm::uvec3(
      static_cast<uint32_t>(std::round(2.0f * reach / static_cast<float>(voxel)))
      + 1u
    )
  );
}

std::shared_ptr<const talus::PreparedMesh> latticeOf(
  const talus::field::SampledSdf& field,
  std::span<const talus::PhysicalMaterial> lanes
) {
  return talus::deformable::buildMesh(
    field,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
}

void printIslands(talus::dynamics::World& world) {
  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  std::printf(
    " islands soft %u strainstep %u\n",
    report.softstepIslands,
    report.strainstepIslands
  );
}

void pileOnFoam(uint32_t height) {
  const talus::field::SampledSdf& ground = groundShape();
  const glm::dvec3 blockHalf(0.2, 0.05, 0.2);
  const talus::field::SampledSdf blockField = latticeField(blockHalf, 0.025);
  const std::vector<talus::PhysicalMaterial> lanes = foam();
  const auto block = latticeOf(blockField, lanes);
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, ground, 0.6, 0.0);
  talus::dynamics::addLattice(
    world.bodies,
    block,
    lanes,
    glm::dvec3(0.0, 0.05, 0.0),
    glm::dvec3(0.0),
    0.6,
    &blockField
  );

  for (uint32_t level = 0u; level != height; ++level) {
    addCube(
      world,
      crate,
      0.05,
      1.0,
      glm::dvec3(0.0, 0.15 + 0.1 * level, 0.0),
      glm::dvec3(0.0)
    );
  }

  const RowResult result = measure(world, 6.0, 1.0);
  const talus::dynamics::LatticeBody& body = world.bodies.lattices[0];
  double topMid = -1.0e30;

  for (const glm::dvec3& node : body.position) {
    if (std::abs(node.x) < 0.03 && std::abs(node.z) < 0.03) {
      topMid = std::max(topMid, node.y);
    }
  }

  std::printf(
    "row pile-%u-on-foam | drift %.4f peak %.4f sag %.4f energy %.4f ms %.3f",
    height,
    result.drift,
    result.peak,
    0.1 - topMid,
    result.energyRatio,
    result.millisecondsPerFrame
  );
  printIslands(world);
  CHECK(result.drift < 0.01);
  CHECK(result.peak < 0.05);
  CHECK(result.energyRatio <= 1.02);
}

}

TEST_CASE("mixed row: piles on a foam block", "[.][twosolverrows]") {
  pileOnFoam(3u);
  pileOnFoam(10u);
  pileOnFoam(14u);
}

TEST_CASE("mixed row: soft ball on a pile", "[.][twosolverrows]") {
  const talus::field::SampledSdf& ground = groundShape();
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  const talus::field::SampledSdf field = ballField(0.05, 0.0125);
  const std::vector<talus::PhysicalMaterial> lanes = ballRubber();
  const auto ball = latticeOf(field, lanes);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, ground, 0.6, 0.0);

  for (uint32_t level = 0u; level != 3u; ++level) {
    addCube(
      world,
      crate,
      0.05,
      1.0,
      glm::dvec3(0.0, 0.05 + 0.1 * level, 0.0),
      glm::dvec3(0.0)
    );
  }

  talus::dynamics::addLattice(
    world.bodies,
    ball,
    lanes,
    glm::dvec3(0.0, 0.351, 0.0),
    glm::dvec3(0.0),
    0.6,
    &field
  );

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row soft-ball-on-pile | drift %.4f peak %.4f energy %.4f ms %.3f",
    result.drift,
    result.peak,
    result.energyRatio,
    result.millisecondsPerFrame
  );
  printIslands(world);
  restRule(result);
}

TEST_CASE("mixed row: rigid ball on a clamped beam", "[.][twosolverrows]") {
  const talus::field::SampledSdf& ground = groundShape();
  const glm::dvec3 beamHalf(0.3, 0.025, 0.05);
  const talus::field::SampledSdf beamField = latticeField(beamHalf, 0.025);
  const std::vector<talus::PhysicalMaterial> lanes = ballRubber();
  const auto built = latticeOf(beamField, lanes);
  auto beam = std::make_shared<talus::PreparedMesh>();
  double reach = 0.0;

  *beam = *built;

  for (const glm::vec3& position : beam->restPositions) {
    reach = std::max(reach, std::abs(static_cast<double>(position.x)));
  }

  beam->anchored.assign(beam->restPositions.size(), 0u);

  for (size_t node = 0u; node != beam->restPositions.size(); ++node) {
    if (std::abs(static_cast<double>(beam->restPositions[node].x)) > reach - 0.0125) {
      beam->anchored[node] = 1u;
    }
  }

  const talus::field::SampledSdf ballShapeField = sphereShape(0.05);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, ground, 0.6, 0.0);
  talus::dynamics::addLattice(
    world.bodies,
    beam,
    lanes,
    glm::dvec3(0.0, 0.3, 0.0),
    glm::dvec3(0.0),
    0.6,
    &beamField
  );
  talus::dynamics::addRigidSphere(
    world.bodies,
    ballShapeField,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.45, 0.0),
    glm::dvec3(0.0),
    0.6,
    0.0
  );

  const RowResult result = measure(world, 4.0, 1.0);

  std::printf(
    "row rigid-ball-on-beam | energy %.4f drift %.4f peak %.4f ms %.3f",
    result.energyRatio,
    result.drift,
    result.peak,
    result.millisecondsPerFrame
  );
  printIslands(world);
  CHECK(result.energyRatio <= 1.02);
}

TEST_CASE("mixed row: soft ball into a crate", "[.][twosolverrows]") {
  const talus::field::SampledSdf cubeField =
    latticeField(glm::dvec3(0.025), 0.0125);
  const std::vector<talus::PhysicalMaterial> lanes = ballRubber();
  const auto cube = latticeOf(cubeField, lanes);
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addLattice(
    world.bodies,
    cube,
    lanes,
    glm::dvec3(-0.2, 0.0, 0.0),
    glm::dvec3(2.0, 0.0, 0.0),
    0.6,
    &cubeField
  );
  addCube(world, crate, 0.05, 1.0, glm::dvec3(0.0), glm::dvec3(0.0));

  const auto momentum = [&world] {
    glm::dvec3 total(0.0);
    double mass = 0.0;
    const talus::dynamics::LatticeBody& body = world.bodies.lattices[0];

    for (size_t node = 0u; node != body.position.size(); ++node) {
      total += static_cast<double>(body.mesh->nodeMass[node]) * body.velocity[node];
      mass += body.mesh->nodeMass[node];
    }

    for (const talus::dynamics::RigidBody& rigid : world.bodies.rigids) {
      total += rigid.mass * rigid.velocity;
      mass += rigid.mass;
    }

    return std::pair<glm::dvec3, double>(total, mass);
  };
  const std::pair<glm::dvec3, double> start = momentum();
  double worst = 0.0;
  double elapsed = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    const auto before = std::chrono::steady_clock::now();

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    elapsed += std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - before
    )
                 .count();
    worst =
      std::max(worst, glm::length(momentum().first - start.first) / start.second);
  }

  std::printf(
    "row soft-ball-into-crate | momentum %.3e ms %.3f",
    worst,
    elapsed / 120.0
  );
  printIslands(world);
  CHECK(worst <= 1.0e-4);
}

TEST_CASE("mixed row: hand-off pile", "[.][twosolverrows]") {
  const talus::field::SampledSdf& ground = groundShape();
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  const talus::field::SampledSdf field = ballField(0.05, 0.0125);
  const std::vector<talus::PhysicalMaterial> lanes = ballRubber();
  const auto ball = latticeOf(field, lanes);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, ground, 0.6, 0.0);

  for (uint32_t level = 0u; level != 3u; ++level) {
    addCube(
      world,
      crate,
      0.05,
      1.0,
      glm::dvec3(0.0, 0.05 + 0.1 * level, 0.0),
      glm::dvec3(0.0)
    );
  }

  talus::dynamics::addLattice(
    world.bodies,
    ball,
    lanes,
    glm::dvec3(-0.4, 0.051, 0.0),
    glm::dvec3(0.5, 0.0, 0.0),
    0.6,
    &field
  );

  const glm::dvec3 g = world.parameters.gravity;
  const double start = talus::dynamics::energyOf(world.bodies, g).total();
  std::vector<glm::dvec3> settled(3u);
  double drift = 0.0;
  double peak = 0.0;
  double energy = 1.0;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    if (frame == 180u) {
      talus::dynamics::LatticeBody& body = world.bodies.lattices[0];

      for (size_t node = 0u; node != body.position.size(); ++node) {
        body.position[node] += glm::dvec3(10.0, 0.0, 0.0);
        body.velocity[node] = glm::dvec3(0.0);
        body.previousVelocity[node] = glm::dvec3(0.0);
      }

      for (size_t index = 0u; index != 3u; ++index) {
        settled[index] = world.bodies.rigids[index].position;
      }
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    energy =
      std::max(energy, talus::dynamics::energyOf(world.bodies, g).total() / start);

    if (frame > 180u) {
      for (size_t index = 0u; index != 3u; ++index) {
        drift = std::max(
          drift,
          glm::length(world.bodies.rigids[index].position - settled[index])
        );
        peak = std::max(peak, glm::length(world.bodies.rigids[index].velocity));
      }
    }
  }

  std::printf(
    "row handoff-pile | drift %.4f peak %.4f energy %.4f",
    drift,
    peak,
    energy
  );
  printIslands(world);
  CHECK(drift < 0.01);
  CHECK(peak < 0.05);
  CHECK(energy <= 1.02);
}

TEST_CASE("mixed row: tall stack touched", "[.][twosolverrows]") {
  const talus::field::SampledSdf& ground = groundShape();
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  const talus::field::SampledSdf field = ballField(0.05, 0.0125);
  const std::vector<talus::PhysicalMaterial> lanes = ballRubber();
  const auto ball = latticeOf(field, lanes);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, ground, 0.6, 0.0);

  for (uint32_t level = 0u; level != 28u; ++level) {
    addCube(
      world,
      crate,
      0.05,
      1.0,
      glm::dvec3(0.0, 0.05 + 0.1 * level, 0.0),
      glm::dvec3(0.0)
    );
  }

  talus::dynamics::addLattice(
    world.bodies,
    ball,
    lanes,
    glm::dvec3(-0.3, 0.051, 0.0),
    glm::dvec3(0.5, 0.0, 0.0),
    0.6,
    &field
  );

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf(
    "row tall-touched | drift %.4f peak %.4f top drop %.4f energy %.4f ms %.3f",
    result.drift,
    result.peak,
    result.topDrop,
    result.energyRatio,
    result.millisecondsPerFrame
  );
  printIslands(world);
}
