#include "engineloop.ih"

EngineLoop::EngineLoop(const StartupOptions& options)
    : m_engine(std::make_unique<GlfwWindowSystem>(m_window), options.project),
      m_input(m_window.handle()) {
  m_engine.loadWorld(options.world);

  m_keySubscription = talus::core::EventDispatcher::instance()
                        .subscribe<talus::platform::KeyEvent>(
                          [this](const talus::platform::KeyEvent& event) {
                            handleKeyEvent(event);
                          }
                        );

  m_mouseSubscription =
    talus::core::EventDispatcher::instance()
      .subscribe<talus::platform::MouseButtonEvent>(
        [this](const talus::platform::MouseButtonEvent& event) {
          handleMouseButtonEvent(event);
        }
      );
}

bool EngineLoop::writeBenchmark(
  const StartupOptions& options,
  std::vector<double> frameMilliseconds
) {
  if (frameMilliseconds.empty()) {
    std::cerr << "benchmark: no frames measured. --frames must exceed --warmup"
              << std::endl;

    return false;
  }

  std::sort(frameMilliseconds.begin(), frameMilliseconds.end());

  const size_t count = frameMilliseconds.size();

  double sum = 0.0;

  for (const double value : frameMilliseconds) {
    sum += value;
  }

  const auto at = [&frameMilliseconds, count](double fraction) {
    const size_t index =
      std::min(count - 1u, size_t(fraction * double(count - 1u) + 0.5));

    return frameMilliseconds[index];
  };

  std::ostringstream out;

  out << std::fixed << std::setprecision(3);
  out << "world\t" << options.world << '\n';
  out << "frames\t" << count << '\n';
  out << "warmup\t" << options.warmup << '\n';
  out << "frame.mean.ms\t" << sum / double(count) << '\n';
  out << "frame.p50.ms\t" << at(0.50) << '\n';
  out << "frame.p95.ms\t" << at(0.95) << '\n';
  out << "frame.max.ms\t" << frameMilliseconds.back() << '\n';

  const std::span<const std::string> names = m_telemetry.names();
  const std::span<const double> values = m_telemetry.values();

  std::vector<size_t> order(names.size());

  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }

  std::sort(order.begin(), order.end(), [names](size_t a, size_t b) {
    return names[a] < names[b];
  });

  for (const size_t i : order) {
    out << names[i] << '\t' << values[i] << '\n';
  }

  const std::filesystem::path path(options.benchmark);

  if (path.has_parent_path()) {
    std::filesystem::create_directories(path.parent_path());
  }

  std::ofstream file(path, std::ios::trunc);

  if (!file) {
    std::cerr << "benchmark: cannot write " << options.benchmark << std::endl;

    return false;
  }

  file << out.str();

  std::cout << out.str();

  return true;
}

EngineLoop::~EngineLoop() {
  talus::core::EventDispatcher::instance()
    .unsubscribe<talus::platform::MouseButtonEvent>(m_mouseSubscription);
  talus::core::EventDispatcher::instance()
    .unsubscribe<talus::platform::KeyEvent>(m_keySubscription);
}

void EngineLoop::handleKeyEvent(const talus::platform::KeyEvent& event) {
  if (
    const std::optional<bool> down = talus::platform::keyTransition(event.type)
  ) {
    m_engine.input().setKey(talus::app::keyFromGlfw(event.key), *down);
  }
}

void EngineLoop::handleMouseButtonEvent(
  const talus::platform::MouseButtonEvent& event
) {
  m_engine.input().setMouseButton(
    talus::app::buttonFromGlfw(event.button),
    event.type == talus::platform::MouseButtonEventType::Pressed
  );
}

void EngineLoop::bindCamera() {
  const talus::objectmodel::Entity eye =
    talus::objectmodel::mainCamera(m_engine.activeWorld());

  m_port.camera = eye;
  m_port.mode = talus::view::DrawMode::Both;

  if (eye == talus::objectmodel::INVALID_ENTITY) {
    m_port.mode = talus::view::DrawMode::Nothing;

    if (!m_reportedMissingCamera) {
      m_reportedMissingCamera = true;

      std::cout << "no camera in this world, so it draws nothing\n";
    }
  }

  if (!m_engine.configureViewport(m_viewport, m_port)) {
    throw std::runtime_error("the player's viewport went missing");
  }
}

bool EngineLoop::verifyBakes() {
  m_engine.context().device().waitIdle();

  talus::objectmodel::World& world = m_engine.activeWorld();

  const entt::registry& registry = world.registry();

  uint32_t checked = 0;

  for (talus::objectmodel::Entity entity : world.sdfGrids()) {
    const auto* volume =
      registry.try_get<talus::objectmodel::BakedVolume>(entity);

    if (volume == nullptr) {
      continue;
    }

    m_engine.storage().sdfBaker().verifyBake(
      registry.get<talus::objectmodel::SdfGrid>(entity),
      volume->index,
      world.primitives(entity),
      m_engine.storage().volumePool().images(volume->index)
    );

    ++checked;
  }

  return checked != 0;
}

int EngineLoop::start(const StartupOptions& options) {
  m_viewport = m_engine.createViewport();

  glfwSetInputMode(m_window.handle(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);

  if (!m_script.load("managed")) {
    throw std::runtime_error(
      "The script assembly did not load: " + m_script.lastError()
    );
  }

  if (!m_script.initialize(
        m_engine.schedule(),
        m_engine.world(),
        std::filesystem::path(options.project) / "scripts"
      )) {
    throw std::runtime_error(
      "The script assembly did not initialise: " + m_script.lastError()
    );
  }

  if (options.analytic) {
    m_port.fieldRepresentation = talus::core::FIELD_ANALYTIC;
  }

  bool bakeCheckPending = options.verifyBake;

  FrameCheck frameCheck(m_engine.context(), m_engine.swapChain(), options);

  std::function<void(VkImage)> captureHook;

  const bool stillFrame = frameCheck.wanted() && options.frames == 0u;

  if (frameCheck.wanted() || frameCheck.capturing()) {
    captureHook = [this, &frameCheck, &options](VkImage image) {
      if (!frameCheck.capturing() && m_engine.frameIndex() < options.frames) {
        return;
      }

      frameCheck.run(image);
    };
  }

  if (frameCheck.capturing()) {
    std::filesystem::create_directories(options.capture);
  }

  double prevTime = glfwGetTime();

  const bool benchmarking = !options.benchmark.empty();

  const float fixedStep =
    (frameCheck.capturing() || benchmarking) ? 1.0f / 60.0f : options.fixedStep;

  if (fixedStep > 0.0f) {
    m_engine.setFixedStep(fixedStep);
  }

  std::vector<double> frameMilliseconds;

  if (benchmarking && options.frames != 0u) {
    frameMilliseconds.reserve(options.frames);
  }

  while (!glfwWindowShouldClose(m_window.handle())) {
    const std::chrono::steady_clock::time_point frameBegan =
      std::chrono::steady_clock::now();

    const double now = glfwGetTime();

    const float dt = static_cast<float>(now - prevTime);

    prevTime = now;

    glfwPollEvents();

    const talus::platform::Cursor where = m_input.cursor();

    m_engine.input().setCursor(
      static_cast<float>(where.x),
      static_cast<float>(where.y)
    );

    constexpr uint32_t FIRST_PLAY_FRAME = 4u;

    if (
      m_engine.frameIndex() == FIRST_PLAY_FRAME && !stillFrame
      && !m_engine.playing()
    ) {
      m_engine.play();
    }

    bindCamera();

    talus::engine::FrameRequest request;
    request.viewport = m_viewport;
    request.deltaSeconds = dt;
    request.surfaceStale = m_window.takeResized();
    request.capture = captureHook;

    if (m_engine.renderFrame(request, m_telemetry)) {
      m_engine.resize();
    }

    if (bakeCheckPending) {
      bakeCheckPending = !verifyBakes();
    }

    if (benchmarking && m_engine.frameIndex() > options.warmup) {
      const std::chrono::duration<double, std::milli> elapsed =
        std::chrono::steady_clock::now() - frameBegan;

      frameMilliseconds.push_back(elapsed.count());
    }

    if (
      frameCheck.ran()
      || (options.frames != 0u && m_engine.frameIndex() >= options.frames)
    ) {
      glfwSetWindowShouldClose(m_window.handle(), GLFW_TRUE);
    }
  }

  m_engine.context().device().waitIdle();

  if (benchmarking && !writeBenchmark(options, frameMilliseconds)) {
    return 1;
  }

  if (bakeCheckPending) {
    std::cout << "bake check: nothing was verified" << std::endl;

    return 1;
  }

  return frameCheck.failed() ? 1 : 0;
}
