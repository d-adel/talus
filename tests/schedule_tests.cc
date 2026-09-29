#include <catch2/catch_test_macros.hpp>

#include <talus/systems/input/input.h>
#include <talus/systems/schedule/schedule.h>

#include <string>
#include <vector>

using talus::objectmodel::World;
using talus::systems::Context;

namespace {

const talus::systems::InputState& noInput() {
  static const talus::systems::InputState idle;

  return idle;
}

}

using talus::systems::Entry;

using talus::systems::Phase;

using talus::systems::phaseFrom;

using talus::systems::Schedule;

namespace {

std::vector<std::string> runOrder(Schedule& schedule, World& world) {
  std::vector<std::string> seen;

  Context context{world, noInput(), 0.016f, 1u};

  schedule.run(context);

  for (const Entry& entry : schedule.systems()) {
    if (entry.enabled) {
      seen.push_back(entry.name);
    }
  }

  return seen;
}

}

TEST_CASE("systems run in ascending order, not insertion order", "[schedule]") {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  REQUIRE(schedule.add(Phase::PostPhysics, 100, "win", [&](Context&) {
    ran.emplace_back("win");
  }));
  REQUIRE(schedule.add(Phase::PostPhysics, -100, "intent", [&](Context&) {
    ran.emplace_back("intent");
  }));
  REQUIRE(schedule.add(Phase::PostPhysics, 0, "budget", [&](Context&) {
    ran.emplace_back("budget");
  }));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"intent", "budget", "win"});
}

TEST_CASE("insertion order is kept within one order key", "[schedule]") {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  for (const char* name : {"first", "second", "third"}) {
    REQUIRE(schedule.add(Phase::PostPhysics, 0, name, [&ran, name](Context&) {
      ran.emplace_back(name);
    }));
  }

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"first", "second", "third"});
}

TEST_CASE("phases run in engine order before order keys count", "[schedule]") {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  REQUIRE(schedule.add(Phase::PostUpdateWork, -100, "late", [&](Context&) {
    ran.emplace_back("late");
  }));
  REQUIRE(schedule.add(Phase::PrePhysics, 100, "early", [&](Context&) {
    ran.emplace_back("early");
  }));
  REQUIRE(schedule.add(Phase::PostPhysics, 0, "mid", [&](Context&) {
    ran.emplace_back("mid");
  }));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"early", "mid", "late"});
}

TEST_CASE("running one phase leaves the other phases alone", "[schedule]") {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  REQUIRE(schedule.add(Phase::PrePhysics, 0, "early", [&](Context&) {
    ran.emplace_back("early");
  }));
  REQUIRE(schedule.add(Phase::PostPhysics, 0, "mid", [&](Context&) {
    ran.emplace_back("mid");
  }));
  REQUIRE(schedule.add(Phase::PostUpdateWork, 0, "late", [&](Context&) {
    ran.emplace_back("late");
  }));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(Phase::PostPhysics, context);

  REQUIRE(ran == std::vector<std::string>{"mid"});
}

TEST_CASE("only the three engine phases are known by number", "[schedule]") {
  REQUIRE(phaseFrom(0u) == Phase::PrePhysics);
  REQUIRE(phaseFrom(1u) == Phase::PostPhysics);
  REQUIRE(phaseFrom(2u) == Phase::PostUpdateWork);
  REQUIRE_FALSE(phaseFrom(3u).has_value());
}

TEST_CASE("a system sees the world it was handed", "[schedule]") {
  Schedule schedule;
  World world;

  size_t counted = 0u;

  REQUIRE(schedule.add(Phase::PostPhysics, 0, "count", [&](Context& context) {
    counted = context.world.sdfGrids().size();
  }));

  world.createSdfGrid({}, {});
  world.createSdfGrid({}, {});

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(counted == 2u);
}

TEST_CASE(
  "a duplicate name is refused and the first one survives",
  "[schedule]"
) {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  REQUIRE(schedule.add(Phase::PostPhysics, 0, "carve", [&](Context&) {
    ran.emplace_back("original");
  }));
  REQUIRE_FALSE(schedule.add(Phase::PostPhysics, 0, "carve", [&](Context&) {
    ran.emplace_back("impostor");
  }));

  REQUIRE(schedule.size() == 1u);

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"original"});
}

TEST_CASE("an empty name and an empty function are refused", "[schedule]") {
  Schedule schedule;

  REQUIRE_FALSE(schedule.add(Phase::PostPhysics, 0, "", [](Context&) {}));
  REQUIRE_FALSE(schedule.add(Phase::PostPhysics, 0, "nothing", {}));

  REQUIRE(schedule.size() == 0u);
}

TEST_CASE("a disabled system keeps its place but does not run", "[schedule]") {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  REQUIRE(schedule.add(Phase::PostPhysics, 0, "a", [&](Context&) {
    ran.emplace_back("a");
  }));
  REQUIRE(schedule.add(Phase::PostPhysics, 0, "b", [&](Context&) {
    ran.emplace_back("b");
  }));

  REQUIRE(schedule.enable("b", false));
  REQUIRE_FALSE(schedule.enabled("b"));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"a"});
  REQUIRE(schedule.size() == 2u);

  REQUIRE(schedule.enable("b", true));
  ran.clear();
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"a", "b"});
}

TEST_CASE(
  "enabling and removing an absent system report failure",
  "[schedule]"
) {
  Schedule schedule;

  REQUIRE_FALSE(schedule.enable("absent", false));
  REQUIRE_FALSE(schedule.enabled("absent"));
  REQUIRE_FALSE(schedule.remove("absent"));
}

TEST_CASE(
  "a removed system stops running and the rest keep order",
  "[schedule]"
) {
  Schedule schedule;
  World world;

  std::vector<std::string> ran;

  for (const char* name : {"a", "b", "c"}) {
    REQUIRE(schedule.add(Phase::PostPhysics, 0, name, [&ran, name](Context&) {
      ran.emplace_back(name);
    }));
  }

  REQUIRE(schedule.remove("b"));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(ran == std::vector<std::string>{"a", "c"});
}

TEST_CASE("clear empties the schedule and its timing", "[schedule]") {
  Schedule schedule;
  World world;

  REQUIRE(schedule.add(Phase::PostPhysics, 0, "a", [](Context&) {}));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  schedule.clear();

  REQUIRE(schedule.size() == 0u);
  REQUIRE(schedule.lastMilliseconds() == 0.0);
}

TEST_CASE(
  "a disabled system reports no time and the total drops it",
  "[schedule]"
) {
  Schedule schedule;
  World world;

  volatile double sink = 0.0;

  const auto busy = [&sink](Context&) {
    for (int step = 0; step < 200000; ++step) {
      sink = sink + static_cast<double>(step);
    }
  };

  REQUIRE(schedule.add(Phase::PostPhysics, 0, "busy", busy));

  Context context{world, noInput(), 0.016f, 0u};
  schedule.run(context);

  REQUIRE(schedule.systems()[0].lastMilliseconds > 0.0);
  REQUIRE(schedule.lastMilliseconds() > 0.0);

  REQUIRE(schedule.enable("busy", false));
  schedule.run(context);

  REQUIRE(schedule.systems()[0].lastMilliseconds == 0.0);
  REQUIRE(schedule.lastMilliseconds() == 0.0);
}

TEST_CASE("the context carries the frame it was built for", "[schedule]") {
  Schedule schedule;
  World world;

  uint32_t seenFrame = 0u;
  float seenDelta = 0.0f;

  REQUIRE(schedule.add(Phase::PostPhysics, -100, "read", [&](Context& context) {
    seenFrame = context.frameIndex;
    seenDelta = context.deltaSeconds;
  }));

  Context context{world, noInput(), 0.25f, 77u};
  schedule.run(context);

  REQUIRE(seenFrame == 77u);
  REQUIRE(seenDelta == 0.25f);
}

TEST_CASE("systems() is reported in execution order", "[schedule]") {
  Schedule schedule;
  World world;

  REQUIRE(schedule.add(Phase::PostPhysics, 100, "z", [](Context&) {}));
  REQUIRE(schedule.add(Phase::PostPhysics, -100, "a", [](Context&) {}));

  const std::vector<std::string> listed = runOrder(schedule, world);

  REQUIRE(listed == std::vector<std::string>{"a", "z"});
}
