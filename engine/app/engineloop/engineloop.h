#pragma once

#include <app/framecheck/framecheck.h>
#include <app/startupoptions/startupoptions.h>
#include <talus/engine/engine/engine.h>
#include <talus/objectmodel/worldquery/worldquery.h>
#include <talus/platform/glfwlibrary/glfwlibrary.h>
#include <talus/platform/input/input.h>
#include <talus/platform/window/window.h>
#include <talus/script/runner/runner.h>

#include <vector>

class EngineLoop {
public:
  explicit EngineLoop(const StartupOptions& options);
  ~EngineLoop();

  EngineLoop(const EngineLoop&) = delete;
  EngineLoop& operator=(const EngineLoop&) = delete;
  EngineLoop(EngineLoop&&) = delete;
  EngineLoop& operator=(EngineLoop&&) = delete;

  int start(const StartupOptions& options = {});

private:
  void bindCamera();

  void handleKeyEvent(const talus::platform::KeyEvent& event);

  void handleMouseButtonEvent(const talus::platform::MouseButtonEvent& event);

  [[nodiscard]] bool verifyBakes();

  [[nodiscard]] bool writeBenchmark(
    const StartupOptions& options,
    std::vector<double> frameMilliseconds
  );

  talus::platform::GLFWLibrary m_glfwLibrary;
  talus::platform::Window m_window;

  talus::engine::Engine m_engine;

  talus::platform::Input m_input;

  talus::script::Runner m_script;

  talus::core::Telemetry m_telemetry;

  bool m_reportedMissingCamera = false;

  talus::view::ViewportId m_viewport = talus::view::INVALID_VIEWPORT;

  talus::view::Viewport m_port{};

  talus::core::EventDispatcher::SubscriptionId m_keySubscription{};
  talus::core::EventDispatcher::SubscriptionId m_mouseSubscription{};
};
