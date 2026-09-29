#include <catch2/catch_test_macros.hpp>

#include <talus/serialize/glmmeta/glmmeta.h>

#include <string>

TEST_CASE("a vec3 writes as three bare numbers", "[glmmeta]") {
  std::string out;

  REQUIRE(!glz::write_json(glm::vec3{1.0f, 2.0f, 3.0f}, out));
  REQUIRE(out == "[1,2,3]");
}

TEST_CASE("a vec4 keeps w last", "[glmmeta]") {
  std::string out;

  REQUIRE(!glz::write_json(glm::vec4{1.0f, 2.0f, 3.0f, 4.0f}, out));
  REQUIRE(out == "[1,2,3,4]");
}

TEST_CASE("a quat writes w first, matching its constructor", "[glmmeta]") {
  std::string out;

  REQUIRE(!glz::write_json(glm::quat{0.5f, 0.1f, 0.2f, 0.3f}, out));
  REQUIRE(out == "[0.5,0.1,0.2,0.3]");
}

TEST_CASE("an unsigned vector stays unsigned", "[glmmeta]") {
  std::string out;

  REQUIRE(!glz::write_json(glm::uvec3{64u, 65u, 66u}, out));
  REQUIRE(out == "[64,65,66]");
}

TEST_CASE("a vec3 round trips to the same bits", "[glmmeta]") {
  const glm::vec3 original{-1.25f, 0.0f, 3.5e7f};

  std::string out;
  REQUIRE(!glz::write_json(original, out));

  const auto back = glz::read_json<glm::vec3>(out);
  REQUIRE(back.has_value());
  REQUIRE(back->x == original.x);
  REQUIRE(back->y == original.y);
  REQUIRE(back->z == original.z);
}

TEST_CASE("a quat round trips to the same bits", "[glmmeta]") {
  const glm::quat original{0.7071068f, 0.0f, 0.7071068f, 0.0f};

  std::string out;
  REQUIRE(!glz::write_json(original, out));

  const auto back = glz::read_json<glm::quat>(out);
  REQUIRE(back.has_value());
  REQUIRE(back->w == original.w);
  REQUIRE(back->x == original.x);
  REQUIRE(back->y == original.y);
  REQUIRE(back->z == original.z);
}

TEST_CASE("the same meta serves a binary format", "[glmmeta]") {
  const glm::vec4 original{1.0f, -2.0f, 3.0f, -4.0f};

  std::string beve;
  REQUIRE(!glz::write_beve(original, beve));

  const auto back = glz::read_beve<glm::vec4>(beve);
  REQUIRE(back.has_value());
  REQUIRE(back->x == original.x);
  REQUIRE(back->w == original.w);
}
