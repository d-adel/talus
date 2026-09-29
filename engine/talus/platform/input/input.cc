#include "input.ih"

namespace talus::platform {

Input::Input(GLFWwindow* window, KeyTiming timing)
    : m_enabled(true),
      m_cursorDeltaInvalid(false),
      m_window(window),
      m_timing(timing) {
  if (m_window == nullptr) {
    throw std::invalid_argument("Input requires a valid GLFWwindow");
  }

  if (s_inputs.contains(m_window)) {
    throw std::logic_error("An Input instance already exists for this window");
  }

  s_inputs.emplace(m_window, this);
  glfwGetCursorPos(m_window, &m_cursor.x, &m_cursor.y);
  m_previousKeyCallback = glfwSetKeyCallback(m_window, &Input::keyCallback);
  m_previousMouseButtonCallback =
    glfwSetMouseButtonCallback(m_window, &Input::mouseButtonCallback);
}

Input::~Input() {
  if (m_window != nullptr) {
    glfwSetMouseButtonCallback(m_window, m_previousMouseButtonCallback);
    glfwSetKeyCallback(m_window, m_previousKeyCallback);

    s_inputs.erase(m_window);
  }
}

Cursor Input::cursor() const noexcept {
  return m_cursor;
}

bool Input::enabled() const noexcept {
  return m_enabled;
}

void Input::toggleEnabled() {
  m_enabled = !m_enabled;
}

void Input::cursorDeltaInvalid() {
  m_cursorDeltaInvalid = true;
}

void Input::updateCursor() {
  double x, y;
  glfwGetCursorPos(m_window, &x, &y);

  if (m_cursorDeltaInvalid) {
    m_cursor.x = x;
    m_cursor.y = y;
    m_cursor.dx = 0;
    m_cursor.dy = 0;

    m_cursorDeltaInvalid = false;
    return;
  }

  m_cursor.dx = x - m_cursor.x;
  m_cursor.dy = y - m_cursor.y;

  m_cursor.x = x;
  m_cursor.y = y;
}

void Input::update() {
  updateCursor();

  const std::uint64_t now = currentTimeMs();
  for (int key = 0; key <= GLFW_KEY_LAST; ++key) {
    KeyState& state = m_keyStates[key];

    if (
      state.pendingSingle && now - state.lastPressMs >= m_timing.doubleWindowMs
    ) {
      state.pendingSingle = false;

      dispatch(key, KeyEventType::SinglePressed);
    }

    if (!state.prevDown) {
      continue;
    }

    if (!state.holdFired && now - state.pressedAtMs >= m_timing.holdDelayMs) {
      state.holdFired = true;

      dispatch(key, KeyEventType::Hold);
    }

    if (state.repeatArmed && now >= state.nextRepeatMs) {
      dispatch(key, KeyEventType::Repeat);

      const std::uint64_t interval =
        std::max<std::uint64_t>(1, m_timing.repeatIntervalMs);

      const std::uint64_t intervalsPassed =
        ((now - state.nextRepeatMs) / interval) + 1;

      state.nextRepeatMs += intervalsPassed * interval;
    }
  }
}

bool Input::isDown(int key) const noexcept {
  if (!isValidKey(key)) {
    return false;
  }

  return m_keyStates[key].prevDown;
}

void Input::keyCallback(
  GLFWwindow* window,
  int key,
  int scancode,
  int action,
  int mods
) {
  const auto it = s_inputs.find(window);

  if (it == s_inputs.end()) {
    return;
  }

  Input* input = it->second;

  input->handleKey(key, action);

  if (input->m_previousKeyCallback != nullptr) {
    input->m_previousKeyCallback(window, key, scancode, action, mods);
  }
}

void Input::mouseButtonCallback(
  GLFWwindow* window,
  int button,
  int action,
  int mods
) {
  const auto it = s_inputs.find(window);

  if (it == s_inputs.end()) {
    return;
  }

  Input* input = it->second;

  if (action == GLFW_PRESS || action == GLFW_RELEASE) {
    talus::core::EventDispatcher::instance().dispatch(
      MouseButtonEvent{
        .button = button,
        .type = action == GLFW_PRESS ? MouseButtonEventType::Pressed
                                     : MouseButtonEventType::Released,
        .mods = mods
      }
    );
  }

  if (input->m_previousMouseButtonCallback != nullptr) {
    input->m_previousMouseButtonCallback(window, button, action, mods);
  }
}

void Input::handleKey(int key, int action) {
  if (!isValidKey(key)) {
    return;
  }

  KeyState& state = m_keyStates[key];
  const std::uint64_t now = currentTimeMs();

  switch (action) {
    case GLFW_PRESS: {
      if (state.prevDown) {
        return;
      }

      state.prevDown = true;
      state.pressedAtMs = now;

      state.holdFired = false;

      state.repeatArmed = true;
      state.nextRepeatMs = now + m_timing.repeatDelayMs;

      dispatch(key, KeyEventType::Pressed);

      if (state.pendingSingle) {
        const std::uint64_t gap = now - state.lastPressMs;

        if (gap <= m_timing.doubleWindowMs) {
          state.pendingSingle = false;

          dispatch(key, KeyEventType::DoublePressed);
        } else {
          state.pendingSingle = false;

          dispatch(key, KeyEventType::SinglePressed);

          state.pendingSingle = true;
        }
      } else {
        state.pendingSingle = true;
      }

      state.lastPressMs = now;
      break;
    }

    case GLFW_RELEASE:
      if (!state.prevDown) {
        return;
      }

      state.prevDown = false;
      state.repeatArmed = false;

      dispatch(key, KeyEventType::Released);

      break;

    case GLFW_REPEAT:
      break;

    default:
      break;
  }
}

void Input::dispatch(int key, KeyEventType type) {
  talus::core::EventDispatcher::instance().dispatch(
    KeyEvent{.key = key, .type = type}
  );
}

std::uint64_t Input::currentTimeMs() {
  using Clock = std::chrono::steady_clock;

  const auto now = Clock::now().time_since_epoch();

  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count()
  );
}

bool Input::isValidKey(int key) noexcept {
  return key >= 0 && key <= GLFW_KEY_LAST;
}

}
