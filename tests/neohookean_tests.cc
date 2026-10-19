#include <catch2/catch_test_macros.hpp>

#include <talus/deformable/material/material.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <cmath>
#include <cstdint>

namespace {

using talus::deformable::ElasticParams;
using talus::deformable::neoHookeanHex;
using talus::deformable::NeoHookeanHex;

constexpr float HALF = 0.05f;
constexpr float YOUNG = 1.0e6f;
constexpr float POISSON = 0.45f;
constexpr float SHEAR = YOUNG / (2.0f * (1.0f + POISSON));

talus::PreparedHex unitHex() {
  talus::PreparedHex hex;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    hex.nodes[corner] = corner;
  }

  hex.halfExtents = glm::vec3(HALF);
  hex.volume = 8.0f * HALF * HALF * HALF;

  return hex;
}

glm::vec3 cornerSign(uint32_t corner) {
  return glm::vec3(
    (corner & 1u) != 0u ? 1.0f : -1.0f,
    (corner & 2u) != 0u ? 1.0f : -1.0f,
    (corner & 4u) != 0u ? 1.0f : -1.0f
  );
}

std::array<glm::vec3, 8> restCorners() {
  std::array<glm::vec3, 8> rest{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    rest[corner] = HALF * cornerSign(corner);
  }

  return rest;
}

std::array<glm::vec3, 8> mapped(const glm::mat3& gradient) {
  std::array<glm::vec3, 8> deformed = restCorners();

  for (glm::vec3& corner : deformed) {
    corner = gradient * corner;
  }

  return deformed;
}

ElasticParams rubber() {
  return ElasticParams{YOUNG, POISSON, 1000.0f};
}

NeoHookeanHex cellAt(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& velocity,
  float hysteresis,
  float step
) {
  return neoHookeanHex(
    deformed,
    restCorners(),
    velocity,
    unitHex(),
    rubber(),
    hysteresis,
    step
  );
}

float largest(const std::array<glm::vec3, 8>& force) {
  float found = 0.0f;

  for (const glm::vec3& value : force) {
    found = glm::max(found, glm::length(value));
  }

  return found;
}

double cycledEnergy(double stretch) {
  const double invariant = stretch * stretch + 2.0 / stretch;

  return 0.5 * static_cast<double>(SHEAR) * (invariant - 3.0)
         * static_cast<double>(unitHex().volume);
}

glm::mat3 uniaxial(float stretch) {
  const float lateral = 1.0f / std::sqrt(stretch);

  return glm::mat3(
    glm::vec3(stretch, 0.0f, 0.0f),
    glm::vec3(0.0f, lateral, 0.0f),
    glm::vec3(0.0f, 0.0f, lateral)
  );
}

}

TEST_CASE(
  "a neo-Hookean hex at rest or rigidly turned carries no force",
  "[neohookean]"
) {
  const std::array<glm::vec3, 8> still{};
  const float scale = SHEAR * HALF * HALF;

  const NeoHookeanHex rest = cellAt(restCorners(), still, 0.4f, 1.0e-4f);

  REQUIRE(largest(rest.force) < 1.0e-6f * scale);
  REQUIRE(std::abs(rest.energy) < 1.0e-9f * SHEAR * unitHex().volume);

  const glm::mat3 turn = glm::mat3(
    glm::rotate(glm::mat4(1.0f), 0.7f, glm::normalize(glm::vec3(1, 2, 3)))
  );

  const NeoHookeanHex turned = cellAt(mapped(turn), still, 0.4f, 1.0e-4f);

  REQUIRE(largest(turned.force) < 1.0e-5f * scale);
  REQUIRE(std::abs(turned.energy) < 1.0e-6f * SHEAR * unitHex().volume);
}

TEST_CASE(
  "a neo-Hookean hex matches the corotated law at small strain",
  "[neohookean]"
) {
  const glm::mat3 strain(
    glm::vec3(2.0e-4f, 0.7e-4f, -0.4e-4f),
    glm::vec3(0.7e-4f, -1.1e-4f, 0.9e-4f),
    glm::vec3(-0.4e-4f, 0.9e-4f, 0.6e-4f)
  );
  const std::array<glm::vec3, 8> deformed = mapped(glm::mat3(1.0f) + strain);
  const std::array<glm::vec3, 8> still{};
  const std::array<talus::PlasticPoint, 8> committed{};

  const NeoHookeanHex neo = cellAt(deformed, still, 0.0f, 1.0e-4f);
  const talus::deformable::HexPointTrial corotated =
    talus::deformable::hexPointTrial(
      deformed,
      restCorners(),
      unitHex(),
      rubber(),
      talus::deformable::PlasticParams{},
      committed,
      0.0f,
      talus::deformable::ElementRotation::Polar,
      false,
      false
    );

  const float scale = largest(corotated.force);

  REQUIRE(scale > 0.0f);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    REQUIRE(
      glm::length(neo.force[corner] - corotated.force[corner]) < 2.0e-3f * scale
    );
  }

  REQUIRE(std::abs(neo.energy - corotated.energy) < 2.0e-3f * corotated.energy);
}

TEST_CASE("an inverted neo-Hookean hex keeps finite forces", "[neohookean]") {
  const glm::mat3 mirror(
    glm::vec3(-1.0f, 0.0f, 0.0f),
    glm::vec3(0.0f, 1.0f, 0.0f),
    glm::vec3(0.0f, 0.0f, 1.0f)
  );
  std::array<glm::vec3, 8> moving{};
  moving.fill(glm::vec3(0.3f, -0.2f, 0.1f));

  const NeoHookeanHex inverted =
    cellAt(mapped(mirror), moving, 0.4f, 1.0e-4f);

  for (const glm::vec3& value : inverted.force) {
    REQUIRE(std::isfinite(value.x));
    REQUIRE(std::isfinite(value.y));
    REQUIRE(std::isfinite(value.z));
  }

  REQUIRE(std::isfinite(inverted.energy));
  REQUIRE(inverted.energy > 0.0f);
  REQUIRE(largest(inverted.force) > 0.0f);
}

TEST_CASE(
  "a neo-Hookean hex loses its hysteresis share of the energy it cycles",
  "[neohookean]"
) {
  constexpr uint32_t LEG = 400u;
  constexpr float PEAK = 1.3f;
  constexpr float HYSTERESIS = 0.4f;
  constexpr float STEP = 1.0e-4f;

  std::array<float, 2u * LEG + 1u> path{};

  for (uint32_t at = 0u; at <= LEG; ++at) {
    const float share = static_cast<float>(at) / static_cast<float>(LEG);
    path[at] = 1.0f + (PEAK - 1.0f) * share;
    path[2u * LEG - at] = path[at];
  }

  double lost = 0.0;
  double unlossy = 0.0;

  for (uint32_t at = 1u; at + 1u < path.size(); ++at) {
    const std::array<glm::vec3, 8> before = mapped(uniaxial(path[at - 1u]));
    const std::array<glm::vec3, 8> now = mapped(uniaxial(path[at]));
    const std::array<glm::vec3, 8> after = mapped(uniaxial(path[at + 1u]));

    std::array<glm::vec3, 8> velocity{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      velocity[corner] = (now[corner] - before[corner]) / STEP;
    }

    const NeoHookeanHex lossy = cellAt(now, velocity, HYSTERESIS, STEP);
    const NeoHookeanHex plain = cellAt(now, velocity, 0.0f, STEP);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::dvec3 moved = glm::dvec3(after[corner] - now[corner]);

      lost -= glm::dot(glm::dvec3(lossy.lossForce[corner]), moved);
      unlossy -= glm::dot(glm::dvec3(plain.lossForce[corner]), moved);
    }
  }

  const double expected =
    static_cast<double>(HYSTERESIS) * 2.0 * cycledEnergy(PEAK);

  REQUIRE(unlossy == 0.0);
  REQUIRE(lost > 0.0);
  REQUIRE(std::abs(lost / expected - 1.0) < 0.02);
}
