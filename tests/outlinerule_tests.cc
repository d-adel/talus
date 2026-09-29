#include <catch2/catch_test_macros.hpp>

#include <talus/view/outlinerule/outlinerule.h>

#include <cstdint>
#include <vector>

using talus::view::Outlined;
using talus::view::outlined;

namespace {

constexpr uint32_t SIZE = 9u;
constexpr uint32_t RADIUS = 1u;
constexpr float FAR = 1.0f;

struct Picture {
  std::vector<float> selection = std::vector<float>(SIZE * SIZE, FAR);
  std::vector<float> scene = std::vector<float>(SIZE * SIZE, FAR);

  void select(
    uint32_t left,
    uint32_t top,
    uint32_t right,
    uint32_t bottom,
    float depth
  ) {
    for (uint32_t y = top; y <= bottom; ++y) {
      for (uint32_t x = left; x <= right; ++x) {
        selection[y * SIZE + x] = depth;

        if (depth < scene[y * SIZE + x]) {
          scene[y * SIZE + x] = depth;
        }
      }
    }
  }

  void other(
    uint32_t left,
    uint32_t top,
    uint32_t right,
    uint32_t bottom,
    float depth
  ) {
    for (uint32_t y = top; y <= bottom; ++y) {
      for (uint32_t x = left; x <= right; ++x) {
        if (depth < scene[y * SIZE + x]) {
          scene[y * SIZE + x] = depth;
        }
      }
    }
  }

  [[nodiscard]] Outlined at(uint32_t x, uint32_t y) const {
    return outlined(selection, scene, SIZE, SIZE, x, y, RADIUS);
  }
};

}

TEST_CASE("a selected object gets a ring against the background", "[outline]") {
  Picture picture;
  picture.select(3, 3, 5, 5, 0.5f);

  REQUIRE(picture.at(2, 4) == Outlined::Visible);
  REQUIRE(picture.at(6, 4) == Outlined::Visible);
  REQUIRE(picture.at(4, 2) == Outlined::Visible);
  REQUIRE(picture.at(2, 2) == Outlined::Visible);

  REQUIRE(picture.at(4, 4) == Outlined::None);
  REQUIRE(picture.at(0, 0) == Outlined::None);
  REQUIRE(picture.at(1, 4) == Outlined::None);
}

TEST_CASE("an object in front of the selection punches no hole", "[outline]") {
  Picture picture;
  picture.select(1, 1, 7, 7, 0.5f);
  picture.other(4, 4, 4, 4, 0.2f);

  REQUIRE(picture.at(4, 4) == Outlined::None);
  REQUIRE(picture.at(3, 4) == Outlined::None);

  REQUIRE(picture.at(0, 4) == Outlined::Visible);
  REQUIRE(picture.at(8, 4) == Outlined::Visible);
}

TEST_CASE(
  "the ring continues dimmed where the scene is in front",
  "[outline]"
) {
  Picture picture;
  picture.select(2, 2, 6, 6, 0.5f);
  picture.other(0, 0, 8, 3, 0.2f);

  REQUIRE(picture.at(4, 1) == Outlined::Occluded);
  REQUIRE(picture.at(1, 2) == Outlined::Occluded);

  REQUIRE(picture.at(4, 7) == Outlined::Visible);
  REQUIRE(picture.at(1, 5) == Outlined::Visible);
}

TEST_CASE("an object behind the selection keeps the ring bright", "[outline]") {
  Picture picture;
  picture.select(2, 2, 6, 6, 0.5f);
  picture.other(4, 7, 4, 7, 0.8f);

  REQUIRE(picture.at(4, 7) == Outlined::Visible);
}

TEST_CASE("nothing selected outlines nothing", "[outline]") {
  Picture picture;
  picture.other(3, 3, 5, 5, 0.5f);

  for (uint32_t y = 0; y != SIZE; ++y) {
    for (uint32_t x = 0; x != SIZE; ++x) {
      REQUIRE(picture.at(x, y) == Outlined::None);
    }
  }
}
