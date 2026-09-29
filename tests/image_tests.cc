#include <talus/imagecompare/imagecompare.h>

#include <catch2/catch_test_macros.hpp>

using talus::image::Bitmap;
using talus::image::Difference;
using talus::image::Tolerance;

namespace {

Bitmap filled(uint32_t width, uint32_t height, uint8_t value) {
  Bitmap bitmap;
  bitmap.width = width;
  bitmap.height = height;
  bitmap.pixels.assign(static_cast<size_t>(width) * height * 4u, value);

  return bitmap;
}

void setPixel(Bitmap& bitmap, uint32_t x, uint32_t y, uint8_t value) {
  const size_t at = (static_cast<size_t>(y) * bitmap.width + x) * 4u;

  bitmap.pixels[at] = value;
  bitmap.pixels[at + 1] = value;
  bitmap.pixels[at + 2] = value;
  bitmap.pixels[at + 3] = value;
}

}

TEST_CASE("identical images differ nowhere") {
  const Bitmap reference = filled(8, 4, 100);
  const Bitmap candidate = filled(8, 4, 100);

  const Difference difference =
    talus::image::compare(reference, candidate, Tolerance{});

  REQUIRE(difference.comparable);
  REQUIRE(difference.differingPixels == 0);
  REQUIRE(difference.worstChannelDelta == 0);
  REQUIRE(talus::image::passes(difference, Tolerance{}));
}

TEST_CASE("a single changed pixel is found, and where") {
  const Bitmap reference = filled(8, 4, 100);

  Bitmap candidate = filled(8, 4, 100);
  setPixel(candidate, 5, 2, 110);

  const Difference difference =
    talus::image::compare(reference, candidate, Tolerance{});

  REQUIRE(difference.differingPixels == 1);
  REQUIRE(difference.worstChannelDelta == 10);
  REQUIRE(difference.worstX == 5);
  REQUIRE(difference.worstY == 2);
  REQUIRE_FALSE(talus::image::passes(difference, Tolerance{}));
}

TEST_CASE("faint drift everywhere reads as a count, not a magnitude") {
  const Bitmap reference = filled(8, 4, 100);
  const Bitmap candidate = filled(8, 4, 101);

  const Difference difference =
    talus::image::compare(reference, candidate, Tolerance{});

  REQUIRE(difference.differingPixels == 32);
  REQUIRE(difference.worstChannelDelta == 1);

  const Tolerance forgiving{1, 0, 255};

  const Difference forgiven =
    talus::image::compare(reference, candidate, forgiving);

  REQUIRE(forgiven.differingPixels == 0);
  REQUIRE(talus::image::passes(forgiven, forgiving));
}

TEST_CASE("one catastrophic pixel cannot hide inside an allowance") {
  const Bitmap reference = filled(8, 4, 0);

  Bitmap candidate = filled(8, 4, 0);
  setPixel(candidate, 1, 1, 255);

  const Tolerance generous{0, 100, 255};

  const Difference difference =
    talus::image::compare(reference, candidate, generous);

  REQUIRE(difference.differingPixels == 1);
  REQUIRE(difference.differingPixels < generous.allowedPixels);
  REQUIRE(difference.worstChannelDelta == 255);
  REQUIRE_FALSE(talus::image::passes(difference, generous));
}

TEST_CASE("mismatched sizes are a different failure, not a large one") {
  const Bitmap reference = filled(8, 4, 100);
  const Bitmap candidate = filled(8, 5, 100);

  const Difference difference =
    talus::image::compare(reference, candidate, Tolerance{});

  REQUIRE_FALSE(difference.comparable);

  REQUIRE(difference.differingPixels == 0);
  REQUIRE(difference.worstChannelDelta == 0);
  REQUIRE_FALSE(talus::image::passes(difference, Tolerance{}));
}

TEST_CASE("the difference image marks where, in context") {
  const Bitmap reference = filled(4, 4, 120);

  Bitmap candidate = filled(4, 4, 120);
  setPixel(candidate, 3, 0, 200);

  const Bitmap picture =
    talus::image::differenceImage(reference, candidate, Tolerance{});

  REQUIRE(picture.width == 4);
  REQUIRE(picture.height == 4);

  const size_t marked = 3u * 4u;
  REQUIRE(picture.pixels[marked] == 255);
  REQUIRE(picture.pixels[marked + 1] == 0);
  REQUIRE(picture.pixels[marked + 2] == 0);

  REQUIRE(picture.pixels[0] == 30);
  REQUIRE(picture.pixels[1] == 30);
  REQUIRE(picture.pixels[2] == 30);
}

TEST_CASE("a bitmap survives a round trip through a png") {
  Bitmap original = filled(6, 3, 40);
  setPixel(original, 2, 1, 210);

  talus::image::save(original, "roundtrip.png");

  const Bitmap reloaded = talus::image::load("roundtrip.png");

  const Difference difference =
    talus::image::compare(original, reloaded, Tolerance{});

  REQUIRE(difference.comparable);
  REQUIRE(difference.worstChannelDelta == 0);
}

TEST_CASE("a missing file is an error, not an empty image") {
  REQUIRE_THROWS(talus::image::load("no-such-file-exists.png"));
}
