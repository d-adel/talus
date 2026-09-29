#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>

#include <catch2/catch_test_macros.hpp>

#include "../engine/capi/win32windowsystem/win32windowsystem.h"

#include <talus/core/telemetry/telemetry.h>
#include <talus/engine/engine/engine.h>
#include <talus/gpu/device/device.h>
#include <talus/gpu/swapchain/swapchain.h>
#include <talus/script/api/api.h>
#include <talus/systems/schedule/schedule.h>

#include <cstdint>
#include <memory>
#include <optional>

namespace {

using talus::engine::Engine;
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

void render(Engine& engine, float deltaSeconds) {
  talus::core::Telemetry telemetry;
  talus::engine::FrameRequest request;
  request.deltaSeconds = deltaSeconds;
  const uint32_t frame = engine.frameIndex();
  REQUIRE_FALSE(engine.renderFrame(request, telemetry));
  REQUIRE(engine.frameIndex() == frame + 1);
}

}

TEST_CASE("a fixed step replaces the measured delta", "[fixedstep]") {
  talus::script::PhysicsScope none(nullptr, nullptr);
  const std::unique_ptr<HWND__, WindowDeleter> window(CreateWindowExW(
    0,
    L"STATIC",
    L"Talus fixed step tests",
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

  constexpr float MEASURED = 0.5f;
  constexpr float FIXED = 1.0f / 50.0f;
  constexpr uint32_t TICKS = 4u;

  double played = 0.0;
  uint32_t ticks = 0u;
  REQUIRE(engine.schedule().add(
    Phase::PrePhysics,
    0,
    "played",
    [&](Context& context) {
      played += static_cast<double>(context.deltaSeconds);
      ++ticks;
    }
  ));

  engine.play();
  REQUIRE_FALSE(engine.fixedStep().has_value());
  REQUIRE_FALSE(engine.swapChain().uncapped());

  engine.setFixedStep(FIXED);
  REQUIRE(engine.fixedStep().has_value());
  REQUIRE(*engine.fixedStep() == FIXED);
  REQUIRE(engine.swapChain().uncapped());

  const uint32_t started = engine.frameIndex();

  for (uint32_t tick = 0u; tick != TICKS; ++tick) {
    render(engine, MEASURED);
    REQUIRE(ticks == tick + 1u);
    REQUIRE(engine.frameIndex() == started + tick + 1u);
    REQUIRE(played == static_cast<double>(tick + 1u) * double(FIXED));
  }

  engine.setFixedStep(std::nullopt);
  REQUIRE_FALSE(engine.fixedStep().has_value());
  REQUIRE_FALSE(engine.swapChain().uncapped());

  const double fixedPlayed = played;
  render(engine, MEASURED);
  REQUIRE(ticks == TICKS + 1u);
  REQUIRE(played == fixedPlayed + double(MEASURED));

  engine.schedule().clear();
  engine.stop();
}

TEST_CASE("a fixed step is a positive number of seconds", "[fixedstep]") {
  talus::script::PhysicsScope none(nullptr, nullptr);
  const std::unique_ptr<HWND__, WindowDeleter> window(CreateWindowExW(
    0,
    L"STATIC",
    L"Talus fixed step rejects",
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

  REQUIRE_THROWS(engine.setFixedStep(0.0f));
  REQUIRE_THROWS(engine.setFixedStep(-1.0f));
  REQUIRE_FALSE(engine.fixedStep().has_value());
  REQUIRE_FALSE(engine.swapChain().uncapped());
}
