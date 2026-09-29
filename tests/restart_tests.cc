#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>

#include <catch2/catch_test_macros.hpp>

#include "../engine/capi/win32windowsystem/win32windowsystem.h"

#include <talus/core/telemetry/telemetry.h>
#include <talus/engine/engine/engine.h>
#include <talus/gpu/device/device.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>
#include <talus/script/api/api.h>
#include <talus/systems/schedule/schedule.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace talus::engine {

struct RestartTestAccess {
  static uint64_t generation(const Engine& engine) {
    return engine.m_playGeneration;
  }
};

}

namespace {

using talus::engine::Engine;
using talus::engine::RestartTestAccess;
using talus::objectmodel::Entity;
using talus::objectmodel::Pose;
using talus::objectmodel::World;
using talus::script::api;
using talus::systems::Context;
using talus::systems::Phase;

struct WindowDeleter {
  void operator()(HWND window) const {
    CHECK(DestroyWindow(window) != FALSE);
  }
};

struct EngineDeleter {
  void operator()(Engine* engine) const {
    engine->context().device().waitIdle();
    delete engine;
  }
};

struct Recording {
  void* world = nullptr;
  uint32_t calls = 0;
  int32_t result = 1;
};

int32_t recordRestart(void* host, void* world) {
  auto& recording = *static_cast<Recording*>(host);
  recording.world = world;
  ++recording.calls;
  return recording.result;
}

constexpr talus::script::PhysicsVerbs RECORDING_VERBS{
  nullptr,
  nullptr,
  nullptr,
  &recordRestart
};

void render(Engine& engine) {
  talus::core::Telemetry telemetry;
  talus::engine::FrameRequest request;
  request.deltaSeconds = 1.0f / 60.0f;
  const uint32_t frame = engine.frameIndex();
  REQUIRE_FALSE(engine.renderFrame(request, telemetry));
  REQUIRE(engine.frameIndex() == frame + 1);
}

float marker(const World& world, Entity entity) {
  return world.registry().get<Pose>(entity).position.x;
}

void mark(World& world, Entity entity, float value) {
  world.patch<Pose>(entity, [value](Pose& pose) { pose.position.x = value; });
}

}

TEST_CASE(
  "restart forwards the installed physics host and world",
  "[restart]"
) {
  talus::script::PhysicsScope none(nullptr, nullptr);
  World world;
  REQUIRE(api().size == sizeof(talus::script::Api));
  REQUIRE(api().version == 16u);
  REQUIRE(api().restart(&world) == 0);
  REQUIRE(api().restart(nullptr) == 0);

  Recording recording;
  {
    talus::script::PhysicsScope installed(&RECORDING_VERBS, &recording);
    REQUIRE(api().restart(&world) == 1);
    REQUIRE(recording.world == &world);
    REQUIRE(recording.calls == 1);
    recording.result = 0;
    REQUIRE(api().restart(nullptr) == 0);
    REQUIRE(recording.world == nullptr);
    REQUIRE(recording.calls == 2);
    {
      const talus::script::PhysicsVerbs empty{};
      talus::script::PhysicsScope missing(&empty, &recording);
      REQUIRE(api().restart(&world) == 0);
      REQUIRE(recording.calls == 2);
    }
    recording.result = 1;
    REQUIRE(api().restart(&world) == 1);
    REQUIRE(recording.calls == 3);
  }
  REQUIRE(api().restart(&world) == 0);
}

TEST_CASE("restart uses the engine tick and runtime lifecycle", "[restart]") {
  talus::script::PhysicsScope none(nullptr, nullptr);
  const std::unique_ptr<HWND__, WindowDeleter> window(CreateWindowExW(
    0,
    L"STATIC",
    L"Talus restart tests",
    WS_OVERLAPPEDWINDOW,
    0,
    0,
    640,
    480,
    nullptr,
    nullptr,
    GetModuleHandleW(nullptr),
    nullptr
  ));
  REQUIRE(window != nullptr);
  const std::unique_ptr<Engine, EngineDeleter> owned(new Engine(
    std::make_unique<talus::capi::Win32WindowSystem>(window.get()),
    TALUS_DEMO_PROJECT
  ));
  Engine& engine = *owned;
  engine.loadWorld("main");
  const Entity authored =
    static_cast<Entity>(api().mainCamera(&engine.world()));
  REQUIRE(engine.world().registry().all_of<Pose>(authored));
  const float diskValue = marker(engine.world(), authored);
  const float memoryValue = diskValue + 17.0f;
  mark(engine.world(), authored, memoryValue);
  REQUIRE(marker(engine.world(), authored) != diskValue);
  REQUIRE(api().restart(&engine.world()) == 0);
  REQUIRE(RestartTestAccess::generation(engine) == 0);

  SECTION("each phase defers duplicate requests through all later systems") {
    for (const Phase requested : talus::systems::PHASES) {
      CAPTURE(requested);
      engine.play();
      const uint64_t generation = RestartTestAccess::generation(engine);
      engine.play();
      REQUIRE(RestartTestAccess::generation(engine) == generation);
      World* live = &engine.activeWorld();
      const Entity transient = live->createAuthored();
      live->emplaceAuthored(transient, Pose{});
      mark(*live, authored, -11.0f);
      bool requestedOnce = false;
      std::array<uint32_t, 3> visits{};
      REQUIRE(
        engine.schedule().add(requested, 0, "request", [&](Context& context) {
          if (requestedOnce) {
            return;
          }
          requestedOnce = true;
          REQUIRE(&context.world == live);
          REQUIRE(api().restart(&context.world) == 1);
          REQUIRE(api().restart(&context.world) == 1);
          REQUIRE(RestartTestAccess::generation(engine) == generation);
          REQUIRE(context.world.registry().valid(transient));
          REQUIRE(marker(context.world, authored) == -11.0f);
          mark(context.world, authored, -23.0f);
          mark(context.world, transient, 31.0f);
        })
      );
      for (const Phase phase : talus::systems::PHASES) {
        const size_t slot = static_cast<size_t>(phase);
        REQUIRE(engine.schedule().add(
          phase,
          1,
          "observe" + std::to_string(slot),
          [&, phase, slot](Context& context) {
            ++visits[slot];
            if (visits[slot] != 1) {
              return;
            }
            REQUIRE(&context.world == live);
            REQUIRE(RestartTestAccess::generation(engine) == generation);
            REQUIRE(context.world.registry().valid(transient));
            REQUIRE(
              marker(context.world, authored)
              == (phase >= requested ? -23.0f : -11.0f)
            );
            if (phase >= requested) {
              REQUIRE(marker(context.world, transient) == 31.0f);
            }
          }
        ));
      }
      render(engine);
      REQUIRE(requestedOnce);
      REQUIRE(visits == std::array<uint32_t, 3>{1, 1, 1});
      REQUIRE(RestartTestAccess::generation(engine) == generation + 1);
      REQUIRE(engine.playing());
      REQUIRE_FALSE(engine.activeWorld().registry().valid(transient));
      REQUIRE(marker(engine.activeWorld(), authored) == memoryValue);
      REQUIRE(marker(engine.world(), authored) == memoryValue);
      REQUIRE(engine.schedule().size() == 4);
      render(engine);
      REQUIRE(RestartTestAccess::generation(engine) == generation + 1);
      REQUIRE(visits == std::array<uint32_t, 3>{2, 2, 2});
      REQUIRE(engine.playing());
      engine.schedule().clear();
      engine.stop();
    }
  }

  SECTION("only the active runtime world can request restart") {
    engine.play();
    World unrelated;
    REQUIRE(api().restart(&engine.world()) == 0);
    REQUIRE(api().restart(&unrelated) == 0);
    REQUIRE(api().restart(nullptr) == 0);
    render(engine);
    REQUIRE(RestartTestAccess::generation(engine) == 1);
    engine.stop();
    REQUIRE(api().restart(&engine.activeWorld()) == 0);
    render(engine);
    REQUIRE_FALSE(engine.playing());
    REQUIRE(RestartTestAccess::generation(engine) == 1);
  }

  SECTION("stop cancels an accepted request across subsequent play") {
    engine.play();
    REQUIRE(api().restart(&engine.activeWorld()) == 1);
    engine.stop();
    engine.stop();
    REQUIRE(api().restart(&engine.world()) == 0);
    render(engine);
    REQUIRE_FALSE(engine.playing());
    REQUIRE(RestartTestAccess::generation(engine) == 1);
    engine.play();
    render(engine);
    REQUIRE(engine.playing());
    REQUIRE(RestartTestAccess::generation(engine) == 2);
  }

  SECTION("paused ticks consume restart without unpausing or running systems") {
    engine.play();
    engine.setPaused(true);
    const Entity transient = engine.activeWorld().createAuthored();
    mark(engine.activeWorld(), authored, -11.0f);
    uint32_t runs = 0;
    REQUIRE(engine.schedule().add(Phase::PrePhysics, 0, "count", [&](Context&) {
      ++runs;
    }));
    REQUIRE(api().restart(&engine.activeWorld()) == 1);
    REQUIRE(api().restart(&engine.activeWorld()) == 1);
    render(engine);
    REQUIRE(RestartTestAccess::generation(engine) == 2);
    REQUIRE(engine.playing());
    REQUIRE(engine.paused());
    REQUIRE(runs == 0);
    REQUIRE_FALSE(engine.activeWorld().registry().valid(transient));
    REQUIRE(marker(engine.activeWorld(), authored) == memoryValue);
    render(engine);
    REQUIRE(RestartTestAccess::generation(engine) == 2);
    REQUIRE(runs == 0);
    engine.stepOnce();
    render(engine);
    REQUIRE(runs == 1);
    REQUIRE(engine.paused());
    render(engine);
    REQUIRE(runs == 1);
    engine.setPaused(false);
    render(engine);
    REQUIRE(runs == 2);
  }

  SECTION("restart and stop preserve the previous physics scope") {
    Recording recording;
    talus::script::PhysicsScope installed(&RECORDING_VERBS, &recording);
    engine.play();
    REQUIRE(api().restart(&engine.activeWorld()) == 1);
    REQUIRE(recording.calls == 0);
    render(engine);
    REQUIRE(RestartTestAccess::generation(engine) == 2);
    REQUIRE(api().restart(&engine.world()) == 0);
    REQUIRE(recording.calls == 0);
    engine.stop();
    REQUIRE(api().restart(&engine.world()) == 1);
    REQUIRE(recording.calls == 1);
    REQUIRE(recording.world == &engine.world());
  }
}
