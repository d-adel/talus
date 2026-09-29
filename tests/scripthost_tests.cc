#include <catch2/catch_test_macros.hpp>

#include <talus/script/host/host.h>

#include <filesystem>

using talus::script::Host;

namespace {

std::filesystem::path managedDirectory() {
  return std::filesystem::path(TALUS_MANAGED_DIR);
}

}

TEST_CASE(
  "the runtime loads and a managed entry point resolves",
  "[scripthost]"
) {
  Host host;

  REQUIRE(host.start(managedDirectory() / "Talus.Engine.runtimeconfig.json"));
  REQUIRE(host.running());

  void* found = host.entryPoint(
    managedDirectory() / "Talus.Engine.dll",
    "Talus.Engine.Boot, Talus.Engine",
    "Initialize"
  );

  REQUIRE(found != nullptr);
}

TEST_CASE("a missing configuration is reported, not crashed", "[scripthost]") {
  Host host;

  REQUIRE_FALSE(host.start(managedDirectory() / "absent.runtimeconfig.json"));
  REQUIRE_FALSE(host.running());
  REQUIRE_FALSE(host.lastError().empty());
}

TEST_CASE("an absent entry point is reported, not crashed", "[scripthost]") {
  Host host;

  REQUIRE(host.start(managedDirectory() / "Talus.Engine.runtimeconfig.json"));

  REQUIRE(
    host.entryPoint(
      managedDirectory() / "Talus.Engine.dll",
      "Talus.Engine.Boot, Talus.Engine",
      "NoSuchMethod"
    )
    == nullptr
  );

  REQUIRE_FALSE(host.lastError().empty());
}
