#include <catch2/catch_test_macros.hpp>

#include <talus/serialize/worldmeta/worldmeta.h>

#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/mesh/mesh.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <string>

TEST_CASE("a primitive round trips every value it carries", "[worldmeta]") {
  const talus::field::Primitive original = talus::field::makeBox(
    glm::vec3(1.5f, -2.0f, 0.25f),
    glm::vec3(0.6f, 0.4f, 0.3f),
    0.7f,
    glm::vec3(0.0f, 1.0f, 0.0f),
    3u,
    1u,
    0.125f
  );

  std::string out;
  REQUIRE(!glz::write_json(original, out));

  const auto back = glz::read_json<talus::field::Primitive>(out);
  REQUIRE(back.has_value());

  for (int column = 0; column != 4; ++column) {
    for (int row = 0; row != 4; ++row) {
      REQUIRE(
        back->inverseModel[column][row] == original.inverseModel[column][row]
      );
    }
  }

  REQUIRE(back->shape == original.shape);
  REQUIRE(back->shapeConfig == original.shapeConfig);
  REQUIRE(back->bounds == original.bounds);
}

TEST_CASE("a grid stores only what an author chose", "[worldmeta]") {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(65u, 66u, 67u);
  grid.margin = 0.25f;
  grid.shadowCullMargin = 1.5f;
  grid.voxelSize = glm::vec3(0.125f);
  grid.origin = glm::vec4(9.0f);

  std::string out;
  REQUIRE(!glz::write_json(grid, out));

  REQUIRE(
    out == R"({"resolution":[65,66,67],"margin":0.25,"shadowCullMargin":1.5})"
  );
}

TEST_CASE("a grid read back leaves the derived halves default", "[worldmeta]") {
  const auto back =
    glz::read_json<talus::objectmodel::SdfGrid>(R"({"resolution":[65,65,65]})");

  REQUIRE(back.has_value());
  REQUIRE(back->resolution == glm::uvec3(65u));
  REQUIRE(!back->margin.has_value());
  REQUIRE(!back->shadowCullMargin.has_value());

  REQUIRE(back->origin == glm::vec4(0.0f));
}

TEST_CASE("a pose reflects through the glm metas alone", "[worldmeta]") {
  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(1.0f, 2.0f, 3.0f);
  pose.rotation = glm::quat(0.5f, 0.5f, 0.5f, 0.5f);

  std::string out;
  REQUIRE(!glz::write_json(pose, out));

  REQUIRE(out == R"({"position":[1,2,3],"rotation":[0.5,0.5,0.5,0.5]})");
}

TEST_CASE("single field components need no meta at all", "[worldmeta]") {
  std::string out;

  REQUIRE(!glz::write_json(talus::objectmodel::Mesh{7u}, out));
  REQUIRE(out == R"({"index":7})");

  REQUIRE(!glz::write_json(talus::objectmodel::Material{2u}, out));
  REQUIRE(out == R"({"index":2})");

  REQUIRE(!glz::write_json(talus::objectmodel::MassScale{0.25f}, out));
  REQUIRE(out == R"({"factor":0.25})");
}
