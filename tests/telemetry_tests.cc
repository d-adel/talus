#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/core/telemetry/telemetry.h>

using Catch::Matchers::WithinAbs;
using talus::core::Telemetry;

TEST_CASE(
  "a name interns to one key however often it is asked for",
  "[telemetry]"
) {
  Telemetry telemetry;

  const Telemetry::Key first = telemetry.key("physics");
  const Telemetry::Key again = telemetry.key("physics");

  REQUIRE(first == again);
  REQUIRE(telemetry.size() == 1);

  const Telemetry::Key other = telemetry.key("carve");

  REQUIRE(other != first);
  REQUIRE(telemetry.size() == 2);
}

TEST_CASE("find refuses a name nobody has interned", "[telemetry]") {
  Telemetry telemetry;

  REQUIRE(telemetry.key("physics") == 0u);

  REQUIRE(telemetry.find("physics") == 0u);
  REQUIRE(telemetry.find("carve") == Telemetry::INVALID_KEY);
}

TEST_CASE("a channel nobody wrote reads as zero", "[telemetry]") {
  Telemetry telemetry;

  const Telemetry::Key key = telemetry.key("physics");

  REQUIRE_THAT(telemetry.get(key), WithinAbs(0.0, 1e-12));
  REQUIRE_THAT(telemetry.get(Telemetry::INVALID_KEY), WithinAbs(0.0, 1e-12));
}

TEST_CASE(
  "add accumulates, set replaces, max keeps the larger",
  "[telemetry]"
) {
  Telemetry telemetry;

  const Telemetry::Key key = telemetry.key("craters");

  telemetry.add(key, 1.0);
  telemetry.add(key, 2.0);

  REQUIRE_THAT(telemetry.get(key), WithinAbs(3.0, 1e-12));

  telemetry.set(key, 10.0);

  REQUIRE_THAT(telemetry.get(key), WithinAbs(10.0, 1e-12));

  telemetry.max(key, 4.0);

  REQUIRE_THAT(telemetry.get(key), WithinAbs(10.0, 1e-12));

  telemetry.max(key, 25.0);

  REQUIRE_THAT(telemetry.get(key), WithinAbs(25.0, 1e-12));
}

TEST_CASE("clear zeroes the values and keeps the keys", "[telemetry]") {
  Telemetry telemetry;

  const Telemetry::Key physics = telemetry.key("physics");
  const Telemetry::Key carve = telemetry.key("carve");

  telemetry.set(physics, 6.0);
  telemetry.set(carve, 2.0);

  telemetry.clear();

  REQUIRE(telemetry.size() == 2);
  REQUIRE(telemetry.key("physics") == physics);
  REQUIRE(telemetry.key("carve") == carve);

  REQUIRE_THAT(telemetry.get(physics), WithinAbs(0.0, 1e-12));
  REQUIRE_THAT(telemetry.get(carve), WithinAbs(0.0, 1e-12));
}

TEST_CASE("values line up with names by key", "[telemetry]") {
  Telemetry telemetry;

  const Telemetry::Key physics = telemetry.key("physics");
  const Telemetry::Key carve = telemetry.key("carve");

  telemetry.set(physics, 6.0);
  telemetry.set(carve, 2.0);

  REQUIRE(telemetry.names().size() == telemetry.values().size());
  REQUIRE(telemetry.names()[physics] == "physics");
  REQUIRE(telemetry.names()[carve] == "carve");

  REQUIRE_THAT(telemetry.values()[physics], WithinAbs(6.0, 1e-12));
  REQUIRE_THAT(telemetry.values()[carve], WithinAbs(2.0, 1e-12));
}
