#include <catch2/catch_test_macros.hpp>

#include <talus/core/asset/asset.h>

using talus::core::AssetRegistry;
using talus::core::INVALID_ASSET;
using talus::core::UNBOUND_ASSET;

namespace {

constexpr talus::core::AssetId STONE = 0xA1B2C3D4E5F60718ULL;
constexpr talus::core::AssetId TIMBER = 0x1807F6E5D4C3B2A1ULL;

}

TEST_CASE("an id resolves to the index it was bound to", "[asset]") {
  AssetRegistry registry;

  registry.bind(STONE, 3u);
  registry.bind(TIMBER, 7u);

  REQUIRE(registry.index(STONE) == 3u);
  REQUIRE(registry.index(TIMBER) == 7u);
  REQUIRE(registry.size() == 2);
}

TEST_CASE("an index resolves back to its id", "[asset]") {
  AssetRegistry registry;

  registry.bind(STONE, 3u);

  REQUIRE(registry.id(3u) == STONE);
  REQUIRE(registry.id(9u) == INVALID_ASSET);
}

TEST_CASE("an id nobody bound is unbound rather than zero", "[asset]") {
  AssetRegistry registry;

  registry.bind(STONE, 0u);

  REQUIRE(registry.index(TIMBER) == UNBOUND_ASSET);
}

TEST_CASE("rebinding moves an id rather than duplicating it", "[asset]") {
  AssetRegistry registry;

  registry.bind(STONE, 3u);
  registry.bind(STONE, 5u);

  REQUIRE(registry.index(STONE) == 5u);
  REQUIRE(registry.size() == 1);
  REQUIRE(registry.id(3u) == INVALID_ASSET);
}

TEST_CASE("the null id cannot name an asset", "[asset]") {
  AssetRegistry registry;

  REQUIRE_THROWS(registry.bind(INVALID_ASSET, 0u));
  REQUIRE(registry.size() == 0);
}
