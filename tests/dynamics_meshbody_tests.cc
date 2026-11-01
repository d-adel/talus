#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "meshfixtures.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/meshbody/meshbody.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/scheduler/scheduler.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using namespace talus::dynamics;
using meshfixture::Mesh;

const glm::dquat noTurn(1.0, 0.0, 0.0, 0.0);
const glm::vec3 boxHalf(0.1f, 0.2f, 0.05f);

struct Threads {
  std::shared_ptr<Scheduler> scheduler;
  Workers workers = resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);
};

Mesh placedBox(
  const glm::vec3& half,
  const glm::dvec3& offset,
  const glm::dquat& turn
) {
  Mesh mesh = meshfixture::box(-half, half);

  for (glm::vec3& vertex : mesh.vertices) {
    vertex = glm::vec3(offset + turn * glm::dvec3(vertex));
  }

  return mesh;
}

RigidBody& bodyOf(BodySet& bodies, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(bodies, id);

  REQUIRE(ref.has_value());
  REQUIRE(ref->side == Side::Rigid);

  return bodies.rigids[ref->index];
}

uint32_t addFloor(World& world) {
  CompoundChild slab;

  slab.half = glm::dvec3(2.0, 0.5, 2.0);
  slab.density = 1000.0;

  return addCompoundBody(
    world.bodies,
    std::vector<CompoundChild>{slab},
    glm::dvec3(0.0, -0.5, 0.0),
    noTurn,
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  );
}

void run(World& world, uint32_t steps) {
  for (uint32_t frame = 0u; frame != steps; ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);
  }
}

double boxSurfaceDistance(const glm::dvec3& point, const glm::dvec3& half) {
  const glm::dvec3 outside = glm::abs(point) - half;

  return glm::length(glm::max(outside, glm::dvec3(0.0)))
         + std::min(std::max({outside.x, outside.y, outside.z}), 0.0);
}

double largest(const glm::vec3& value) {
  return static_cast<double>(std::max({value.x, value.y, value.z}));
}

bool sameBits(double a, double b) {
  return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

bool sameBits(const glm::dvec3& a, const glm::dvec3& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z);
}

bool sameBits(const glm::dquat& a, const glm::dquat& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z)
         && sameBits(a.w, b.w);
}

std::vector<glm::dvec2> facePoints(
  const RigidBody& body,
  uint32_t axis,
  double level,
  double tolerance
) {
  std::vector<glm::dvec2> points;

  for (const glm::dvec3& sample : body.samples) {
    const glm::dvec3 point = meshPoint(body, sample);

    if (std::abs(point[axis] - level) <= tolerance) {
      points.emplace_back(point[(axis + 1u) % 3u], point[(axis + 2u) % 3u]);
    }
  }

  return points;
}

double faceGap(
  std::vector<glm::dvec2> points,
  const glm::dvec2& low,
  const glm::dvec2& high,
  double step
) {
  const auto alongU = [](const glm::dvec2& a, const glm::dvec2& b) {
    return a.x < b.x;
  };
  const glm::uvec2 counts(glm::ceil((high - low) / step) + glm::dvec2(1.0));
  double farthest = 0.0;

  std::sort(points.begin(), points.end(), alongU);

  for (uint32_t i = 0u; i != counts.x; ++i) {
    for (uint32_t j = 0u; j != counts.y; ++j) {
      const glm::dvec2 at = glm::min(low + step * glm::dvec2(i, j), high);
      const auto split =
        std::lower_bound(points.begin(), points.end(), at, alongU);
      double nearest = std::numeric_limits<double>::infinity();

      for (auto point = split;
           point != points.end() && point->x - at.x < nearest;
           ++point) {
        nearest = std::min(nearest, glm::length(*point - at));
      }

      for (auto point = split;
           point != points.begin() && at.x - (point - 1)->x < nearest;) {
        --point;
        nearest = std::min(nearest, glm::length(*point - at));
      }

      farthest = std::max(farthest, nearest);
    }
  }

  return std::numbers::sqrt2 * farthest + step;
}

Mesh joined(const Mesh& first, const Mesh& second) {
  Mesh both = first;
  const uint32_t offset = static_cast<uint32_t>(first.vertices.size());

  both.vertices.insert(
    both.vertices.end(),
    second.vertices.begin(),
    second.vertices.end()
  );

  for (const uint32_t index : second.indices) {
    both.indices.push_back(index + offset);
  }

  return both;
}

Mesh inward(Mesh mesh) {
  for (size_t face = 0u; face < mesh.indices.size(); face += 3u) {
    std::swap(mesh.indices[face + 1u], mesh.indices[face + 2u]);
  }

  return mesh;
}

Mesh shifted(Mesh mesh, const glm::vec3& by) {
  for (glm::vec3& vertex : mesh.vertices) {
    vertex += by;
  }

  return mesh;
}

constexpr glm::vec3 seamSlabHalf(0.3f, 0.025f, 0.15f);
constexpr double seamSlabMass = 10.0;

struct Seam {
  World world;
  std::shared_ptr<MeshShape> shape;
  uint32_t slab = 0u;
  uint32_t ground = 0u;
};

void buildSeam(
  Seam& seam,
  double degrees,
  bool tiltedRight,
  double friction,
  bool meshSlab
) {
  const double angle = glm::radians(degrees);
  const double side = tiltedRight ? 1.0 : -1.0;
  const glm::dvec3 half(0.3, 0.1, 0.3);
  const glm::dvec3 axis(0.0, 0.0, 1.0);
  CompoundChild flat;
  CompoundChild tilted;

  flat.centre = glm::dvec3(-side * half.x, -half.y, 0.0);
  flat.half = half;
  flat.density = 1000.0;
  flat.key = 0u;
  tilted.rotation = glm::angleAxis(side * angle, axis);
  tilted.centre = tilted.rotation * glm::dvec3(side * half.x, -half.y, 0.0);
  tilted.half = half;
  tilted.density = 1000.0;
  tilted.key = 1u;
  seam.ground = addCompoundBody(
    seam.world.bodies,
    std::vector<CompoundChild>{flat, tilted},
    glm::dvec3(0.0),
    noTurn,
    glm::dvec3(0.0),
    friction,
    0.0,
    true
  );

  const glm::dvec3 slabHalf(seamSlabHalf);
  const double rise = slabHalf.x * std::tan(angle);
  const glm::dquat bridge =
    glm::angleAxis(side * std::atan2(rise, 2.0 * slabHalf.x), axis);
  const glm::dvec3 at = glm::dvec3(0.0, 0.5 * rise, 0.0)
                        + bridge * glm::dvec3(0.0, slabHalf.y + 0.001, 0.0);

  if (!meshSlab) {
    CompoundChild box;

    box.half = slabHalf;
    box.density = seamSlabMass / (8.0 * slabHalf.x * slabHalf.y * slabHalf.z);
    seam.slab = seam.world.bodies
                  .rigids[addCompoundBody(
                    seam.world.bodies,
                    std::vector<CompoundChild>{box},
                    at,
                    bridge,
                    glm::dvec3(0.0),
                    friction,
                    0.0,
                    false
                  )]
                  .id;

    return;
  }

  const Mesh slab = meshfixture::box(-seamSlabHalf, seamSlabHalf);
  const Threads threads;
  MeshShapeSettings settings;

  settings.mass = seamSlabMass;
  settings.friction = friction;
  seam.shape = std::make_shared<MeshShape>(
    meshShape(slab.vertices, slab.indices, settings, threads.workers)
  );
  seam.slab =
    addMeshBody(seam.world.bodies, *seam.shape, at, bridge, glm::dvec3(0.0));
}

double seamDrift(
  double degrees,
  bool tiltedRight,
  double friction,
  bool meshSlab,
  uint32_t steps
) {
  Seam seam;

  buildSeam(seam, degrees, tiltedRight, friction, meshSlab);
  run(seam.world, 30u);

  const glm::dvec3 start = bodyOf(seam.world.bodies, seam.slab).position;

  run(seam.world, steps);

  return bodyOf(seam.world.bodies, seam.slab).position.x - start.x;
}
}

TEST_CASE(
  "a mesh shape has the requested mass and box-like inertia",
  "[dynamics][meshbody]"
) {
  const Mesh mesh = meshfixture::box(-boxHalf, boxHalf);
  MeshShapeSettings settings;

  settings.mass = 3.0;

  const MeshShape shape = meshShape(mesh.vertices, mesh.indices, settings);

  REQUIRE(shape.field != nullptr);
  CHECK(shape.body.field == shape.field.get());
  CHECK(shape.body.shape == RigidShape::Sampled);
  CHECK(std::abs(shape.body.mass - 3.0) <= 3.0e-12);

  const glm::dvec3 size = 2.0 * glm::dvec3(boxHalf);
  const glm::dvec3 expected(
    3.0 * (size.y * size.y + size.z * size.z) / 12.0,
    3.0 * (size.x * size.x + size.z * size.z) / 12.0,
    3.0 * (size.x * size.x + size.y * size.y) / 12.0
  );

  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    glm::dvec3 unit(0.0);

    unit[axis] = 1.0;

    const glm::dvec3 along = glm::abs(shape.body.fieldRotation * unit);
    const uint32_t side = along.x >= along.y && along.x >= along.z ? 0u
                          : along.y >= along.z                     ? 1u
                                                                   : 2u;

    CAPTURE(axis, side, along[side], shape.body.moment[axis], expected[side]);
    CHECK(along[side] >= 0.999);
    CHECK(
      std::abs(shape.body.moment[axis] - expected[side])
      <= 0.03 * expected[side]
    );
  }

  const double area =
    2.0 * (size.x * size.y + size.y * size.z + size.z * size.x);
  const size_t budget = static_cast<size_t>(
    std::max(32.0, std::ceil(area / (settings.spacing * settings.spacing)))
  );

  CAPTURE(budget, shape.body.samples.size());
  CHECK(shape.body.samples.size() <= budget);
  CHECK(shape.body.samples.size() > budget / 2u);
}

TEST_CASE(
  "a mesh shape built on workers matches the serial build",
  "[dynamics][meshbody]"
) {
  const Mesh mesh = placedBox(
    boxHalf,
    glm::dvec3(0.3, 0.0, 0.0),
    glm::angleAxis(0.5, glm::normalize(glm::dvec3(0.3, 1.0, 0.5)))
  );
  const MeshShape serial = meshShape(mesh.vertices, mesh.indices, {});
  const Threads threads;
  const MeshShape parallel =
    meshShape(mesh.vertices, mesh.indices, {}, threads.workers);

  REQUIRE(serial.field->distances.size() == parallel.field->distances.size());
  CHECK(serial.field->distances == parallel.field->distances);
  CHECK(sameBits(serial.body.mass, parallel.body.mass));
  CHECK(sameBits(serial.body.moment, parallel.body.moment));
  CHECK(sameBits(serial.body.fieldOrigin, parallel.body.fieldOrigin));
  CHECK(sameBits(serial.body.fieldRotation, parallel.body.fieldRotation));
  REQUIRE(serial.body.samples.size() == parallel.body.samples.size());

  for (size_t index = 0u; index != serial.body.samples.size(); ++index) {
    CAPTURE(index);
    CHECK(sameBits(serial.body.samples[index], parallel.body.samples[index]));
  }
}

TEST_CASE("a mesh body lands at the requested pose", "[dynamics][meshbody]") {
  const glm::dvec3 position(1.2, 0.7, -0.4);
  const glm::dquat rotation =
    glm::angleAxis(1.1, glm::normalize(glm::dvec3(0.2, 0.9, -0.4)));
  const glm::dvec3 velocity(0.5, -0.25, 0.125);
  const glm::dvec3 offset =
    GENERATE(glm::dvec3(0.0), glm::dvec3(0.3, 0.0, 0.0));
  const double angle = offset.x > 0.0 ? std::numbers::pi / 6.0 : 0.0;
  const glm::dquat inside =
    glm::angleAxis(angle, glm::normalize(glm::dvec3(0.3, 1.0, 0.5)));
  const Mesh mesh = placedBox(boxHalf, offset, inside);
  const MeshShape shape = meshShape(mesh.vertices, mesh.indices, {});
  const double voxel = largest(shape.field->voxelSize);
  BodySet bodies;

  CAPTURE(offset.x);
  CHECK(glm::length(shape.body.fieldOrigin - offset) <= 0.5 * voxel);

  const uint32_t id = addMeshBody(bodies, shape, position, rotation, velocity);
  const RigidBody& body = bodyOf(bodies, id);
  const MeshFrame frame = meshFrame(body);

  CHECK(body.shape == RigidShape::Sampled);
  CHECK(body.field == shape.field.get());
  CHECK(body.velocity == velocity);
  CHECK(glm::length(frame.position - position) <= 1.0e-9);
  CHECK(std::abs(glm::dot(frame.rotation, rotation)) >= 1.0 - 1.0e-9);
  CHECK(
    glm::length(body.position - (position + rotation * offset)) <= 0.5 * voxel
  );

  const glm::dquat back = glm::conjugate(rotation);
  const glm::dquat unturn = glm::conjugate(inside);
  double farthest = 0.0;
  double mismatch = 0.0;

  for (const glm::dvec3& sample : body.samples) {
    const glm::dvec3 world = body.position + body.rotation * sample;
    const glm::dvec3 local = unturn * (back * (world - position) - offset);
    const glm::dvec3 drawn =
      frame.position + frame.rotation * meshPoint(body, sample);

    farthest = std::max(
      farthest,
      std::abs(boxSurfaceDistance(local, glm::dvec3(boxHalf)))
    );
    mismatch = std::max(mismatch, glm::length(drawn - world));
  }

  CAPTURE(farthest, mismatch, voxel);
  CHECK(farthest <= voxel);
  CHECK(mismatch <= 1.0e-9);
}

TEST_CASE("a mesh body rests on an anchored floor", "[dynamics][meshbody]") {
  const Mesh ball = meshfixture::icosphere(0.25f, 4u);
  const Threads threads;
  MeshShapeSettings settings;

  settings.mass = 5.0;

  const MeshShape shape =
    meshShape(ball.vertices, ball.indices, settings, threads.workers);
  World world;

  (void)addFloor(world);

  const uint32_t id = addMeshBody(
    world.bodies,
    shape,
    glm::dvec3(0.1, 0.35, -0.2),
    glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, 0.4, 0.2))),
    glm::dvec3(0.0)
  );

  run(world, 240u);

  const RigidBody& body = bodyOf(world.bodies, id);
  const MeshFrame frame = meshFrame(body);

  CAPTURE(frame.position.y, body.samples.size());
  CHECK(std::abs(frame.position.y - 0.25) <= 0.003);
  CHECK(glm::length(body.velocity) < 0.01);
}

TEST_CASE(
  "a box edge resting on a field face sinks less than half the point spacing",
  "[dynamics][meshbody]"
) {
  const glm::vec3 slabHalf(0.5f, 0.05f, 0.5f);
  const Mesh slab = meshfixture::box(-slabHalf, slabHalf);
  const Threads threads;
  MeshShapeSettings settings;

  settings.mass = 500.0;

  const MeshShape shape =
    meshShape(slab.vertices, slab.indices, settings, threads.workers);
  World world;

  (void)addFloor(world);

  const uint32_t slabId = addMeshBody(
    world.bodies,
    shape,
    glm::dvec3(0.0, 0.05, 0.0),
    noTurn,
    glm::dvec3(0.0)
  );
  const glm::dvec3 barHalf(0.15, 0.02, 0.02);
  const double barMass = 1.0;
  const glm::dquat edgeDown =
    glm::angleAxis(std::numbers::pi / 4.0, glm::dvec3(1.0, 0.0, 0.0));
  const double barDrop = barHalf.y * std::numbers::sqrt2 + 0.002;
  CompoundChild bar;

  bar.half = barHalf;
  bar.density = barMass / (8.0 * barHalf.x * barHalf.y * barHalf.z);

  std::vector<uint32_t> bars;

  for (uint32_t index = 0u; index != 6u; ++index) {
    bars.push_back(addCompoundBody(
      world.bodies,
      std::vector<CompoundChild>{bar},
      glm::dvec3(
        0.07 - 0.04 * static_cast<double>(index % 2u),
        0.1 + barDrop,
        -0.36 + 0.124 * static_cast<double>(index)
      ),
      edgeDown,
      glm::dvec3(0.0),
      0.5,
      0.0,
      false
    ));
  }

  run(world, 240u);

  const MeshFrame top = meshFrame(bodyOf(world.bodies, slabId));
  const glm::dvec3 up = top.rotation * glm::dvec3(0.0, 1.0, 0.0);
  const glm::dvec3 surface =
    top.position + top.rotation * glm::dvec3(0.0, slabHalf.y, 0.0);

  for (const uint32_t index : bars) {
    const RigidBody& body = world.bodies.rigids[index];
    const CompoundChild& child = body.compound->children.front();
    const glm::dquat turn = body.rotation * child.rotation;
    double depth = -std::numeric_limits<double>::infinity();

    for (const glm::dvec3& sample : childSamples(child)) {
      depth = std::max(
        depth,
        glm::dot(surface - (body.position + body.rotation * sample), up)
      );
    }

    const double sideY =
      std::abs(glm::dot(turn * glm::dvec3(0.0, 1.0, 0.0), up));
    const double sideZ =
      std::abs(glm::dot(turn * glm::dvec3(0.0, 0.0, 1.0), up));

    CAPTURE(index, depth, sideY, sideZ, glm::length(body.velocity));
    CHECK(depth <= 0.5 * settings.spacing + 0.002);
    CHECK(sideY > 0.6);
    CHECK(sideZ > 0.6);
    CHECK(glm::length(body.velocity) < 0.01);
  }
}

TEST_CASE(
  "a box edge crossing mesh rails between their points sinks less than half "
  "the point spacing",
  "[dynamics][meshbody]"
) {
  const Threads threads;
  const glm::vec3 railHalf(0.02f, 0.02f, 0.3f);
  const Mesh rail = meshfixture::box(-railHalf, railHalf);
  MeshShapeSettings settings;

  settings.mass = 5.0;

  const MeshShape shape =
    meshShape(rail.vertices, rail.indices, settings, threads.workers);
  World world;

  (void)addFloor(world);

  const std::array<double, 2> railX{-0.25, 0.25};
  std::array<uint32_t, 2> rails{};

  for (size_t side = 0u; side != rails.size(); ++side) {
    rails[side] = addMeshBody(
      world.bodies,
      shape,
      glm::dvec3(railX[side], static_cast<double>(railHalf.y), 0.0),
      noTurn,
      glm::dvec3(0.0)
    );
  }

  const glm::dvec3 barHalf(0.5, 0.02, 0.02);
  CompoundChild bar;

  bar.half = barHalf;
  bar.density = 1.0 / (8.0 * barHalf.x * barHalf.y * barHalf.z);

  std::vector<uint32_t> bars;

  for (uint32_t index = 0u; index != 5u; ++index) {
    bars.push_back(addCompoundBody(
      world.bodies,
      std::vector<CompoundChild>{bar},
      glm::dvec3(
        0.0,
        2.0 * static_cast<double>(railHalf.y) + barHalf.y * std::numbers::sqrt2
          + 0.002,
        -0.2 + 0.084 * static_cast<double>(index)
      ),
      glm::angleAxis(std::numbers::pi / 4.0, glm::dvec3(1.0, 0.0, 0.0)),
      glm::dvec3(0.0),
      0.5,
      0.0,
      false
    ));
  }

  run(world, 240u);

  for (const uint32_t index : bars) {
    const RigidBody& body = world.bodies.rigids[index];
    const glm::dquat turn =
      body.rotation * body.compound->children.front().rotation;
    std::array<glm::dvec3, 8> corners{};

    for (uint32_t corner = 0u; corner != corners.size(); ++corner) {
      const glm::dvec3 sign(
        (corner & 1u) != 0u ? 1.0 : -1.0,
        (corner & 2u) != 0u ? 1.0 : -1.0,
        (corner & 4u) != 0u ? 1.0 : -1.0
      );

      corners[corner] = body.position + turn * (sign * barHalf);
    }

    std::ranges::sort(corners, [](const glm::dvec3& a, const glm::dvec3& b) {
      return a.y < b.y;
    });

    const glm::dvec3 from = corners[0];
    const glm::dvec3 along = corners[1] - corners[0];
    double depth = -std::numeric_limits<double>::infinity();

    for (size_t side = 0u; side != rails.size(); ++side) {
      const MeshFrame top = meshFrame(bodyOf(world.bodies, rails[side]));
      const glm::dvec3 up = top.rotation * glm::dvec3(0.0, 1.0, 0.0);
      const glm::dvec3 surface =
        top.position + top.rotation * glm::dvec3(0.0, railHalf.y, 0.0);
      const glm::dvec3 over =
        from + along * ((top.position.x - from.x) / along.x);

      depth = std::max(depth, glm::dot(surface - over, up));
    }

    CAPTURE(index, depth, glm::length(body.velocity));
    CHECK(depth <= 0.5 * settings.spacing + 0.002);
    CHECK(glm::length(body.velocity) < 0.01);
  }
}

TEST_CASE(
  "a thin plate gets contact points on both large faces",
  "[dynamics][meshbody]"
) {
  const glm::vec3 half(0.3f, 0.006f, 0.3f);
  const Mesh plate = meshfixture::box(-half, half);
  const Threads threads;
  const MeshShapeSettings settings;
  const MeshShape shape =
    meshShape(plate.vertices, plate.indices, settings, threads.workers);
  const double voxel = largest(shape.field->voxelSize);
  const glm::dvec2 inner(static_cast<double>(half.z) - voxel);

  for (const double side : {-1.0, 1.0}) {
    const std::vector<glm::dvec2> points = facePoints(
      shape.body,
      1u,
      side * static_cast<double>(half.y),
      0.25 * static_cast<double>(half.y)
    );
    const double gap = faceGap(points, -inner, inner, 0.002);

    CAPTURE(side, points.size(), gap);
    CHECK(!points.empty());
    CHECK(gap <= 1.3 * settings.spacing);
  }
}

TEST_CASE(
  "contact points on a curved mesh body sit on the field's zero level",
  "[dynamics][meshbody]"
) {
  const float radius = 0.25f;
  const Mesh ball = meshfixture::icosphere(radius, 4u);
  const Threads threads;
  const MeshShape shape =
    meshShape(ball.vertices, ball.indices, {}, threads.workers);
  const double voxel = largest(shape.field->voxelSize);
  double innermost = std::numeric_limits<double>::infinity();

  for (size_t face = 0u; face < ball.indices.size(); face += 3u) {
    const glm::dvec3 a(ball.vertices[ball.indices[face]]);
    const glm::dvec3 b(ball.vertices[ball.indices[face + 1u]]);
    const glm::dvec3 c(ball.vertices[ball.indices[face + 2u]]);

    innermost = std::min(
      innermost,
      std::abs(glm::dot(glm::normalize(glm::cross(b - a, c - a)), a))
    );
  }

  double farthest = 0.0;
  double deepest = 0.0;
  double nearest = std::numeric_limits<double>::infinity();
  double outermost = 0.0;

  for (const glm::dvec3& sample : shape.body.samples) {
    const glm::dvec3 point = meshPoint(shape.body, sample);
    const double distance =
      talus::field::probe<double>(*shape.field, point).distance;

    farthest = std::max(farthest, std::abs(distance));
    deepest = std::min(deepest, distance);
    nearest = std::min(nearest, glm::length(point));
    outermost = std::max(outermost, glm::length(point));
  }

  CAPTURE(shape.body.samples.size(), voxel, farthest, deepest, innermost);
  CAPTURE(nearest, outermost);
  CHECK(shape.body.samples.size() > 1000u);
  CHECK(farthest <= voxel);
  CHECK(deepest >= -0.1 * voxel);
  CHECK(nearest >= innermost - 0.1 * voxel);
  CHECK(outermost <= static_cast<double>(radius) + voxel);
}

TEST_CASE("a long beam keeps its contact spacing", "[dynamics][meshbody]") {
  const glm::vec3 half(2.0f, 0.12f, 0.12f);
  const Mesh beam = meshfixture::box(-half, half);
  const Threads threads;
  const MeshShapeSettings settings;
  const MeshShape shape =
    meshShape(beam.vertices, beam.indices, settings, threads.workers);
  const double voxel = largest(shape.field->voxelSize);
  const std::vector<glm::dvec2> points = facePoints(
    shape.body,
    1u,
    static_cast<double>(half.y),
    0.25 * static_cast<double>(half.y)
  );
  const glm::dvec2 inner(
    static_cast<double>(half.z) - voxel,
    static_cast<double>(half.x) - voxel
  );
  const double gap = faceGap(points, -inner, inner, 0.002);

  CAPTURE(points.size(), gap);
  CHECK(gap <= 1.3 * settings.spacing);
}

TEST_CASE("meshShape refuses bad settings", "[dynamics][meshbody]") {
  const Mesh mesh = meshfixture::box(-boxHalf, boxHalf);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const auto refused =
    [](const Mesh& given, const MeshShapeSettings& settings) {
      CHECK_THROWS_AS(
        (void)meshShape(given.vertices, given.indices, settings),
        std::invalid_argument
      );
    };
  const auto with = [](double MeshShapeSettings::* field, double value) {
    MeshShapeSettings settings;

    settings.*field = value;

    return settings;
  };

  for (const double value : {0.0, -1.0, nan, infinity}) {
    CAPTURE(value);
    refused(mesh, with(&MeshShapeSettings::mass, value));
  }

  for (const double value : {0.0, -0.02, nan, infinity}) {
    CAPTURE(value);
    refused(mesh, with(&MeshShapeSettings::spacing, value));
  }

  for (const double value : {-0.01, nan, infinity, 1.0e-30}) {
    CAPTURE(value);
    refused(mesh, with(&MeshShapeSettings::voxel, value));
  }

  for (const double value : {-0.1, nan, infinity}) {
    CAPTURE(value);
    refused(mesh, with(&MeshShapeSettings::friction, value));
  }

  for (const double value : {-0.1, 1.5, nan}) {
    CAPTURE(value);
    refused(mesh, with(&MeshShapeSettings::restitution, value));
  }

  Mesh open = mesh;

  open.indices.resize(open.indices.size() - 3u);
  refused(open, {});

  Mesh unbounded = mesh;

  unbounded.vertices.front().x = std::numeric_limits<float>::infinity();
  refused(unbounded, {});
  refused(Mesh{}, {});

  const Mesh speck{
    {{0.002f, 0.0f, 0.0f},
     {0.0f, 0.002f, 0.0f},
     {0.0f, 0.0f, 0.002f},
     {0.002f, 0.002f, 0.002f}},
    {0u, 1u, 2u, 0u, 3u, 1u, 0u, 2u, 3u, 1u, 3u, 2u}
  };

  for (const double value : {0.01, 0.0025}) {
    CAPTURE(value);
    refused(speck, with(&MeshShapeSettings::voxel, value));
  }
}

TEST_CASE(
  "a mesh body resting across a compound crease stays where it is",
  "[dynamics][meshbody]"
) {
  for (const double degrees : {6.0, 15.0, 30.0}) {
    for (const bool tiltedRight : {true, false}) {
      const double mesh = seamDrift(degrees, tiltedRight, 0.6, true, 300u);
      const double box = seamDrift(degrees, tiltedRight, 0.6, false, 300u);

      CAPTURE(degrees, tiltedRight, mesh, box);
      CHECK(std::abs(mesh) < 0.001);
      CHECK(std::abs(box) < 0.001);
    }
  }
}

TEST_CASE(
  "meshShape refuses overlapping or wrongly nested shells",
  "[dynamics][meshbody]"
) {
  const Mesh bar = meshfixture::box(
    glm::vec3(-0.125f, -0.05f, -0.05f),
    glm::vec3(0.125f, 0.05f, 0.05f)
  );
  const Mesh outer = meshfixture::box(glm::vec3(-0.2f), glm::vec3(0.2f));
  const Mesh inner = meshfixture::box(glm::vec3(-0.1f), glm::vec3(0.1f));
  const std::array<Mesh, 2> refused{
    joined(bar, shifted(bar, glm::vec3(0.15f, 0.0f, 0.0f))),
    joined(outer, inner)
  };

  for (const Mesh& mesh : refused) {
    CHECK_THROWS_AS(
      (void)meshShape(mesh.vertices, mesh.indices, {}),
      std::invalid_argument
    );
  }
}

TEST_CASE(
  "meshShape accepts single and separate shells of either winding",
  "[dynamics][meshbody]"
) {
  const Threads threads;
  const Mesh bar = meshfixture::box(
    glm::vec3(-0.125f, -0.05f, -0.05f),
    glm::vec3(0.125f, 0.05f, 0.05f)
  );
  const std::vector<Mesh> accepted{
    meshfixture::box(-boxHalf, boxHalf),
    meshfixture::icosphere(0.25f, 4u),
    meshfixture::icosphere(0.05f, 3u),
    meshfixture::torus(0.2, 0.07, 96u, 32u),
    meshfixture::prism(
      {{0.0f, 0.0f},
       {0.3f, 0.0f},
       {0.3f, 0.1f},
       {0.1f, 0.1f},
       {0.1f, 0.3f},
       {0.0f, 0.3f}},
      3u,
      0.0f,
      0.08f
    ),
    meshfixture::box(glm::vec3(-0.005f), glm::vec3(0.005f)),
    joined(bar, shifted(bar, glm::vec3(0.4f, 0.0f, 0.0f))),
    joined(
      meshfixture::box(glm::vec3(-0.2f), glm::vec3(0.2f)),
      inward(meshfixture::box(glm::vec3(-0.1f), glm::vec3(0.1f)))
    )
  };

  for (size_t index = 0u; index != accepted.size(); ++index) {
    CAPTURE(index);
    CHECK_NOTHROW((void)meshShape(
      accepted[index].vertices,
      accepted[index].indices,
      {},
      threads.workers
    ));
  }

  const Mesh box = meshfixture::box(-boxHalf, boxHalf);
  const Mesh turned = inward(box);
  const MeshShape outside = meshShape(box.vertices, box.indices, {});
  const MeshShape inside = meshShape(turned.vertices, turned.indices, {});

  REQUIRE(outside.field->distances.size() == inside.field->distances.size());
  REQUIRE(outside.body.samples.size() == inside.body.samples.size());

  double distance = 0.0;
  double sample = 0.0;

  for (size_t index = 0u; index != outside.field->distances.size(); ++index) {
    distance = std::max(
      distance,
      std::abs(
        static_cast<double>(outside.field->distances[index])
        - static_cast<double>(inside.field->distances[index])
      )
    );
  }

  for (size_t index = 0u; index != outside.body.samples.size(); ++index) {
    sample = std::max(
      sample,
      glm::length(outside.body.samples[index] - inside.body.samples[index])
    );
  }

  const double moment = glm::length(outside.body.moment - inside.body.moment)
                        / glm::length(outside.body.moment);

  CAPTURE(distance, sample, moment);
  CHECK(distance <= 1.0e-6);
  CHECK(sample <= 1.0e-6);
  CHECK(moment <= 1.0e-6);
}

TEST_CASE(
  "meshShape refuses more contact points than contact keys can number",
  "[dynamics][meshbody]"
) {
  const Threads threads;
  const glm::vec3 half(0.8f, 0.006f, 0.8f);
  const Mesh plate = meshfixture::box(-half, half);
  MeshShapeSettings settings;

  settings.spacing = 0.004;
  settings.voxel = 0.004;
  CHECK_THROWS_WITH(
    (void)meshShape(plate.vertices, plate.indices, settings, threads.workers),
    Catch::Matchers::ContainsSubstring("contact points")
  );

  settings.spacing = 0.008;
  CHECK(
    meshShape(plate.vertices, plate.indices, settings, threads.workers)
      .body.samples.size()
    <= size_t{1} << 18u
  );
}

TEST_CASE(
  "a fast mesh bar that reaches a block within the step stops on it",
  "[dynamics][meshbody]"
) {
  const bool spun = GENERATE(false, true);
  const bool jointed = GENERATE(false, true);
  const bool meshBlock = GENERATE(false, true);
  const glm::vec3 barHalf(0.25f, 0.025f, 0.025f);
  const Mesh bar = meshfixture::box(-barHalf, barHalf);
  const glm::dvec3 blockHalf(0.14, 0.1475, 0.15);
  const glm::dvec3 blockCentre(0.26, blockHalf.y, 0.0);
  const double blockTop = blockCentre.y + blockHalf.y;
  const IntegrationParameters parameters;
  const double substeps = static_cast<double>(
    jointed ? parameters.jointSubsteps : parameters.softstepSubsteps
  );
  const double speed = 10.0;
  const double gap =
    spun ? 0.08 : speed * parameters.dt * (1.0 - 0.75 / substeps);
  const glm::dvec3 hub(0.0, blockTop + gap + barHalf.y, 0.0);
  const Threads threads;
  MeshShapeSettings settings;

  settings.mass = 2.0;

  const MeshShape barShape =
    meshShape(bar.vertices, bar.indices, settings, threads.workers);
  const glm::vec3 blockMeshHalf(blockHalf);
  const Mesh blockMesh = meshfixture::box(-blockMeshHalf, blockMeshHalf);
  MeshShapeSettings blockSettings;

  blockSettings.mass = 400.0;
  blockSettings.spacing = 0.03;

  const MeshShape blockShape = meshShape(
    blockMesh.vertices,
    blockMesh.indices,
    blockSettings,
    threads.workers
  );
  World world;

  (void)addFloor(world);

  if (meshBlock) {
    (void)addMeshBody(
      world.bodies,
      blockShape,
      blockCentre,
      noTurn,
      glm::dvec3(0.0)
    );
  } else {
    CompoundChild block;

    block.half = blockHalf;
    block.density = 1000.0;
    (void)addCompoundBody(
      world.bodies,
      std::vector<CompoundChild>{block},
      blockCentre,
      noTurn,
      glm::dvec3(0.0),
      0.5,
      0.0,
      true
    );
  }

  const uint32_t id =
    addMeshBody(world.bodies, barShape, hub, noTurn, glm::dvec3(0.0));

  if (spun) {
    bodyOf(world.bodies, id).angularVelocity =
      glm::dvec3(0.0, 0.0, -speed / static_cast<double>(barHalf.x));
  } else {
    bodyOf(world.bodies, id).velocity = glm::dvec3(0.0, -speed, 0.0);
  }

  if (jointed) {
    JointDef pin;

    pin.kind = spun ? JointKind::Ball : JointKind::Spring;
    pin.bodyB = id;
    pin.frameA.position = hub;
    REQUIRE(addJoint(world, pin) != 0u);
  }

  double deepest = 0.0;
  double lowest = std::numeric_limits<double>::infinity();
  double after = 0.0;

  for (uint32_t frame = 0u; frame != 6u; ++frame) {
    const StepReport report = step(world);

    REQUIRE_FALSE(report.nonFinite);

    if (jointed) {
      CHECK(report.jointSubsteps == world.parameters.jointSubsteps);
    }

    const RigidBody& body = bodyOf(world.bodies, id);

    if (frame == 0u) {
      after = spun ? static_cast<double>(barHalf.x) * body.angularVelocity.z
                   : body.velocity.y;
    }

    for (const glm::dvec3& sample : body.samples) {
      const glm::dvec3 point =
        body.position + glm::mat3_cast(body.rotation) * sample;

      if (std::abs(point.x - blockCentre.x) < blockHalf.x
          && std::abs(point.z - blockCentre.z) < blockHalf.z) {
        deepest = std::max(deepest, blockTop - point.y);
        lowest = std::min(lowest, point.y - blockTop);
      }
    }
  }

  CAPTURE(spun, jointed, meshBlock, gap, deepest, lowest, after);
  CHECK(lowest < 0.02);
  CHECK(after > -0.9 * speed);
  CHECK(deepest < 0.005);
}
