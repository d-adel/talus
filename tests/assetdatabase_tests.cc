#include <catch2/catch_test_macros.hpp>

#include <talus/core/asset/assetdatabase.h>

using talus::core::AssetDatabase;
using talus::core::INVALID_ASSET;
using talus::core::UNBOUND_ASSET;

namespace {

constexpr talus::core::AssetId STONE = 0xA1B2C3D4E5F60718ULL;
constexpr talus::core::AssetId TIMBER = 0x1807F6E5D4C3B2A1ULL;

struct FakeMaterial {};

struct FakeMesh {};

struct FakeTexture {};

}

TEST_CASE("each kind keeps its own index space", "[assetdatabase]") {
  AssetDatabase assets;

  assets.bind<FakeMaterial>(STONE, 3u);
  assets.bind<FakeMesh>(STONE, 7u);

  REQUIRE(assets.index<FakeMaterial>(STONE) == 3u);
  REQUIRE(assets.index<FakeMesh>(STONE) == 7u);
  REQUIRE(assets.kinds() == 2);
}

TEST_CASE("a kind nobody bound answers unbound", "[assetdatabase]") {
  const AssetDatabase assets;

  REQUIRE(assets.index<FakeTexture>(STONE) == UNBOUND_ASSET);
  REQUIRE(assets.id<FakeTexture>(0u) == INVALID_ASSET);
}

TEST_CASE("reading an absent kind does not create it", "[assetdatabase]") {
  AssetDatabase assets;

  assets.bind<FakeMaterial>(STONE, 0u);

  const AssetDatabase& reading = assets;

  REQUIRE(reading.index<FakeTexture>(TIMBER) == UNBOUND_ASSET);
  REQUIRE(assets.kinds() == 1);
}

TEST_CASE("a new kind needs no change to the database", "[assetdatabase]") {
  AssetDatabase assets;

  assets.bind<FakeMaterial>(STONE, 0u);
  assets.bind<FakeMesh>(TIMBER, 1u);
  assets.bind<FakeTexture>(STONE, 5u);

  REQUIRE(assets.kinds() == 3);
  REQUIRE(assets.index<FakeTexture>(STONE) == 5u);
  REQUIRE(assets.id<FakeTexture>(5u) == STONE);
}
