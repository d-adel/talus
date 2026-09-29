#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/field/analytic/analytic.h>
#include <talus/field/deform/deform.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include "tolerances.h"

#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::AnalyticSample;
using talus::field::Primitive;

namespace {

constexpr uint32_t SPHERE = 0;
constexpr uint32_t BOX = 1;
constexpr uint32_t PLANE = 2;

constexpr uint32_t UNION = 0;
constexpr uint32_t SMOOTH_UNION = 1;
constexpr uint32_t INTERSECTION = 2;
constexpr uint32_t SUBTRACTION = 3;
constexpr uint32_t SMOOTH_SUBTRACTION = 4;

Primitive makePrimitive(
  uint32_t type,
  const glm::vec3& translation,
  const glm::vec4& shape,
  uint32_t material,
  uint32_t operation
) {
  Primitive primitive{};
  primitive.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), translation));
  primitive.shape = shape;
  primitive.shapeConfig = glm::uvec4(type, material, operation, 0);

  return primitive;
}

Primitive makeTransformedPrimitive(
  uint32_t type,
  const glm::mat4& model,
  const glm::vec4& shape,
  uint32_t material,
  uint32_t operation
) {
  Primitive primitive{};
  primitive.inverseModel = glm::inverse(model);
  primitive.shape = shape;
  primitive.shapeConfig = glm::uvec4(type, material, operation, 0);

  return primitive;
}

float distanceAt(const std::vector<Primitive>& primitives, const glm::vec3& p) {
  return talus::field::sample(primitives, p).distance;
}

struct ProbeAgreement {
  uint32_t inside = 0u;
  uint32_t boundary = 0u;
  uint32_t outside = 0u;
  uint32_t differing = 0u;
};

ProbeAgreement probeAgreement(const talus::field::SampledSdf& field) {
  constexpr std::array<float, 11> ALONG{
    -0.4f,
    -0.01f,
    0.0f,
    0.07f,
    0.21f,
    0.5f,
    0.63f,
    0.88f,
    1.0f,
    1.01f,
    1.3f
  };

  const glm::vec3 low = field.origin;
  const glm::vec3 high =
    field.origin
    + field.voxelSize * glm::vec3(field.resolution - glm::uvec3(1u));

  const auto coordinate = [&](int axis, float t) {
    if (t == 0.0f) {
      return low[axis];
    }

    if (t == 1.0f) {
      return high[axis];
    }

    return low[axis] + (high[axis] - low[axis]) * t;
  };

  ProbeAgreement agreement;

  for (const float x : ALONG) {
    for (const float y : ALONG) {
      for (const float z : ALONG) {
        const glm::vec3 point(
          coordinate(0, x),
          coordinate(1, y),
          coordinate(2, z)
        );

        const float probed = talus::field::probe(field, point).distance;
        const float measured = talus::field::probeDistance(field, point);

        if (
          std::bit_cast<uint32_t>(probed) != std::bit_cast<uint32_t>(measured)
        ) {
          ++agreement.differing;
        }

        if (
          glm::any(glm::lessThan(point, low))
          || glm::any(glm::greaterThan(point, high))
        ) {
          ++agreement.outside;
        } else if (
          glm::any(glm::equal(point, low)) || glm::any(glm::equal(point, high))
        ) {
          ++agreement.boundary;
        } else {
          ++agreement.inside;
        }
      }
    }
  }

  return agreement;
}

}

TEST_CASE("Primitive keeps the layout the shader indexes by", "[field]") {
  REQUIRE(sizeof(Primitive) == 112);
  REQUIRE(alignof(Primitive) == 16);
  REQUIRE(offsetof(Primitive, inverseModel) == 0);
  REQUIRE(offsetof(Primitive, shape) == 64);
  REQUIRE(offsetof(Primitive, shapeConfig) == 80);
  REQUIRE(offsetof(Primitive, bounds) == 96);
}

TEST_CASE("bounds culling does not change the field", "[field][bounds]") {
  std::vector<Primitive> unbounded{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION),
    makePrimitive(
      SPHERE,
      glm::vec3(1.4f, 0.0f, 0.0f),
      glm::vec4(0.8f, 0.0f, 0.0f, 0.5f),
      5,
      SMOOTH_UNION
    ),
    makePrimitive(
      SPHERE,
      glm::vec3(0.0f, 0.9f, 0.0f),
      glm::vec4(0.4f, 0, 0, 0),
      7,
      SUBTRACTION
    ),
    makePrimitive(
      SPHERE,
      glm::vec3(-0.6f, 0.0f, 0.5f),
      glm::vec4(0.45f, 0.0f, 0.0f, 0.3f),
      11,
      SMOOTH_SUBTRACTION
    ),
    makePrimitive(
      SPHERE,
      glm::vec3(6.0f, 6.0f, 6.0f),
      glm::vec4(0.5f, 0, 0, 0),
      9,
      UNION
    )
  };

  std::vector<Primitive> bounded = unbounded;
  for (Primitive& primitive : bounded) {
    talus::field::updateBounds(primitive);
  }

  for (float x = -3.0f; x <= 3.0f; x += 0.25f) {
    for (float y = -3.0f; y <= 3.0f; y += 0.25f) {
      for (float z = -3.0f; z <= 3.0f; z += 0.5f) {
        const glm::vec3 point(x, y, z);

        const AnalyticSample plain = talus::field::sample(unbounded, point);
        const AnalyticSample culled = talus::field::sample(bounded, point);

        REQUIRE_THAT(
          culled.distance,
          WithinAbs(plain.distance, ANALYTIC_TOLERANCE)
        );
        REQUIRE(culled.material == plain.material);
      }
    }
  }
}

TEST_CASE("an empty field is far away and has material zero", "[field]") {
  const std::vector<Primitive> primitives;

  const AnalyticSample result =
    talus::field::sample(primitives, glm::vec3(0.0f));

  REQUIRE(result.distance > 1e8f);
  REQUIRE(result.material == 0);
}

TEST_CASE("a sphere reports signed distance to its surface", "[field]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 0, UNION)
  };

  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(2.0f, 0.0f, 0.0f)),
    WithinAbs(1.0f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(0.0f)),
    WithinAbs(-1.0f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(0.0f, 0.0f, 1.0f)),
    WithinAbs(0.0f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("a box reports signed distance to its surface", "[field]") {
  const std::vector<Primitive> primitives{makePrimitive(
    BOX,
    glm::vec3(0.0f),
    glm::vec4(0.5f, 0.5f, 0.5f, 0.0f),
    0,
    UNION
  )};

  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(1.0f, 0.0f, 0.0f)),
    WithinAbs(0.5f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(0.0f)),
    WithinAbs(-0.5f, ANALYTIC_TOLERANCE)
  );

  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(1.0f, 1.0f, 0.5f)),
    WithinAbs(std::sqrt(0.5f), ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("a plane measures along its own up axis", "[field]") {
  const std::vector<Primitive> primitives{makePrimitive(
    PLANE,
    glm::vec3(0.0f, -2.0f, 0.0f),
    glm::vec4(0.0f),
    0,
    UNION
  )};

  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(0.0f)),
    WithinAbs(2.0f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(0.0f, -3.0f, 0.0f)),
    WithinAbs(-1.0f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("a rigid transform carries the field with the shape", "[field]") {
  const std::vector<Primitive> primitives{makePrimitive(
    SPHERE,
    glm::vec3(5.0f, 0.0f, 0.0f),
    glm::vec4(1.0f, 0, 0, 0),
    0,
    UNION
  )};

  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(5.0f, 0.0f, 0.0f)),
    WithinAbs(-1.0f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(7.0f, 0.0f, 0.0f)),
    WithinAbs(1.0f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("a rotation turns the field with the shape", "[field]") {
  const glm::vec4 longBox(1.0f, 0.2f, 0.2f, 0.0f);
  const glm::vec3 probe(0.0f, 0.0f, 0.9f);

  const std::vector<Primitive> unrotated{
    makeTransformedPrimitive(BOX, glm::mat4(1.0f), longBox, 0, UNION)
  };

  const std::vector<Primitive> rotated{makeTransformedPrimitive(
    BOX,
    glm::rotate(
      glm::mat4(1.0f),
      glm::radians(90.0f),
      glm::vec3(0.0f, 1.0f, 0.0f)
    ),
    longBox,
    0,
    UNION
  )};

  REQUIRE_THAT(
    distanceAt(unrotated, probe),
    WithinAbs(0.7f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(rotated, probe),
    WithinAbs(-0.1f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("union takes the nearer surface and its material", "[field][csg]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION),
    makePrimitive(
      SPHERE,
      glm::vec3(4.0f, 0.0f, 0.0f),
      glm::vec4(1.0f, 0, 0, 0),
      8,
      UNION
    )
  };

  const AnalyticSample near =
    talus::field::sample(primitives, glm::vec3(4.0f, 0.0f, 0.0f));

  REQUIRE_THAT(near.distance, WithinAbs(-1.0f, ANALYTIC_TOLERANCE));
  REQUIRE(near.material == 8);
}

TEST_CASE("intersection takes the farther surface", "[field][csg]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION),
    makePrimitive(
      SPHERE,
      glm::vec3(1.0f, 0.0f, 0.0f),
      glm::vec4(1.0f, 0, 0, 0),
      8,
      INTERSECTION
    )
  };

  const AnalyticSample result =
    talus::field::sample(primitives, glm::vec3(-0.5f, 0.0f, 0.0f));

  REQUIRE_THAT(result.distance, WithinAbs(0.5f, ANALYTIC_TOLERANCE));
  REQUIRE(result.material == 8);
}

TEST_CASE("subtraction keeps the accumulated material", "[field][csg]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION),
    makePrimitive(
      SPHERE,
      glm::vec3(0.0f),
      glm::vec4(0.5f, 0, 0, 0),
      8,
      SUBTRACTION
    )
  };

  const AnalyticSample result =
    talus::field::sample(primitives, glm::vec3(0.0f));

  REQUIRE_THAT(result.distance, WithinAbs(0.5f, ANALYTIC_TOLERANCE));
  REQUIRE(result.material == 3);
}

TEST_CASE(
  "smooth subtraction rounds where the hard one creases",
  "[field][csg]"
) {
  const Primitive solid =
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION);

  const std::vector<Primitive> hard{
    solid,
    makePrimitive(
      SPHERE,
      glm::vec3(0.0f),
      glm::vec4(0.5f, 0.0f, 0.0f, 0.5f),
      8,
      SUBTRACTION
    )
  };

  const std::vector<Primitive> smooth{
    solid,
    makePrimitive(
      SPHERE,
      glm::vec3(0.0f),
      glm::vec4(0.5f, 0.0f, 0.0f, 0.5f),
      8,
      SMOOTH_SUBTRACTION
    )
  };

  const glm::vec3 probe(0.75f, 0.0f, 0.0f);

  REQUIRE_THAT(distanceAt(hard, probe), WithinAbs(-0.25f, ANALYTIC_TOLERANCE));
  REQUIRE_THAT(
    distanceAt(smooth, probe),
    WithinAbs(-0.125f, ANALYTIC_TOLERANCE)
  );

  REQUIRE(distanceAt(smooth, probe) > distanceAt(hard, probe));

  REQUIRE(talus::field::sample(smooth, probe).material == 3);
}

TEST_CASE("smooth union blends by the shader's smin", "[field][csg]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION),
    makePrimitive(
      SPHERE,
      glm::vec3(1.5f, 0.0f, 0.0f),
      glm::vec4(1.0f, 0.0f, 0.0f, 0.5f),
      8,
      SMOOTH_UNION
    )
  };

  REQUIRE_THAT(
    distanceAt(primitives, glm::vec3(0.75f, 0.0f, 0.0f)),
    WithinAbs(-0.375f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("the fold is order dependent", "[field][csg]") {
  const Primitive big =
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 3, UNION);
  const Primitive carve = makePrimitive(
    SPHERE,
    glm::vec3(0.0f),
    glm::vec4(0.5f, 0, 0, 0),
    8,
    SUBTRACTION
  );
  const Primitive added = makePrimitive(
    SPHERE,
    glm::vec3(0.3f, 0.0f, 0.0f),
    glm::vec4(1.0f, 0, 0, 0),
    9,
    UNION
  );

  const std::vector<Primitive> carveThenAdd{big, carve, added};
  const std::vector<Primitive> addThenCarve{big, added, carve};

  REQUIRE_THAT(
    distanceAt(carveThenAdd, glm::vec3(0.0f)),
    WithinAbs(-0.7f, ANALYTIC_TOLERANCE)
  );
  REQUIRE_THAT(
    distanceAt(addThenCarve, glm::vec3(0.0f)),
    WithinAbs(0.5f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("the gradient has unit length away from surfaces", "[field]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 0, UNION)
  };

  const glm::vec3 g =
    talus::field::gradient(primitives, glm::vec3(1.5f, 1.5f, 1.5f));

  REQUIRE_THAT(glm::length(g), WithinAbs(1.0f, GRADIENT_TOLERANCE));
}

TEST_CASE("the normal points away from a sphere's centre", "[field]") {
  const std::vector<Primitive> primitives{
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 0, UNION)
  };

  const glm::vec3 n =
    talus::field::normal(primitives, glm::vec3(3.0f, 0.0f, 0.0f));

  REQUIRE_THAT(n.x, WithinAbs(1.0f, GRADIENT_TOLERANCE));
  REQUIRE_THAT(n.y, WithinAbs(0.0f, GRADIENT_TOLERANCE));
  REQUIRE_THAT(n.z, WithinAbs(0.0f, GRADIENT_TOLERANCE));
}

TEST_CASE(
  "a probe reports the probe distance bit for bit, inside, on and beyond "
  "the grid",
  "[field]"
) {
  Primitive ball =
    makePrimitive(SPHERE, glm::vec3(0.0f), glm::vec4(1.0f, 0, 0, 0), 0, UNION);
  Primitive cutter = makePrimitive(
    SPHERE,
    glm::vec3(0.9f, 0.0f, 0.0f),
    glm::vec4(0.5f, 0, 0, 0),
    0,
    SUBTRACTION
  );

  talus::field::updateBounds(ball);
  talus::field::updateBounds(cutter);

  const talus::field::SampledSdf baked = talus::field::bake(
    std::vector<Primitive>{ball},
    glm::vec3(-1.5f),
    glm::vec3(1.5f),
    glm::uvec3(33u)
  );

  talus::field::SampledSdf carved = baked;

  REQUIRE(talus::field::deformAndRepair(carved, cutter).cellsRemoved > 0u);

  const std::array<const talus::field::SampledSdf*, 2> fields{&baked, &carved};

  for (const talus::field::SampledSdf* field : fields) {
    const ProbeAgreement agreement = probeAgreement(*field);

    INFO(
      "inside " << agreement.inside << " boundary " << agreement.boundary
                << " outside " << agreement.outside << " differing "
                << agreement.differing
    );

    REQUIRE(agreement.inside > 0u);
    REQUIRE(agreement.boundary > 0u);
    REQUIRE(agreement.outside > 0u);
    REQUIRE(agreement.differing == 0u);
  }
}
