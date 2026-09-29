#include "input.ih"

namespace talus::systems {

void InputState::beginFrame() noexcept {
  m_keyPressed = {};
  m_keyReleased = {};
  m_buttonPressed = {};
  m_buttonReleased = {};
}

void InputState::setKey(Key key, bool down) noexcept {
  if (key == Key::Unknown || key >= Key::Count) {
    return;
  }

  const size_t at = static_cast<size_t>(key);

  if (m_keys[at] == down) {
    return;
  }

  m_keys[at] = down;

  (down ? m_keyPressed : m_keyReleased)[at] = true;
}

void InputState::setMouseButton(MouseButton button, bool down) noexcept {
  if (button >= MouseButton::Count) {
    return;
  }

  const size_t at = static_cast<size_t>(button);

  if (m_buttons[at] == down) {
    return;
  }

  m_buttons[at] = down;

  (down ? m_buttonPressed : m_buttonReleased)[at] = true;
}

void InputState::setCursor(float x, float y) noexcept {
  m_cursorX = x;
  m_cursorY = y;
}

void InputState::setViewport(float width, float height) noexcept {
  m_viewportWidth = width;
  m_viewportHeight = height;
}

void InputState::clear() noexcept {
  m_keys = {};
  m_keyPressed = {};
  m_keyReleased = {};
  m_buttons = {};
  m_buttonPressed = {};
  m_buttonReleased = {};
}

bool InputState::held(Key key) const noexcept {
  return key < Key::Count && m_keys[static_cast<size_t>(key)];
}

bool InputState::pressed(Key key) const noexcept {
  return key < Key::Count && m_keyPressed[static_cast<size_t>(key)];
}

bool InputState::released(Key key) const noexcept {
  return key < Key::Count && m_keyReleased[static_cast<size_t>(key)];
}

bool InputState::held(MouseButton button) const noexcept {
  return button < MouseButton::Count && m_buttons[static_cast<size_t>(button)];
}

bool InputState::pressed(MouseButton button) const noexcept {
  return button < MouseButton::Count
         && m_buttonPressed[static_cast<size_t>(button)];
}

bool InputState::released(MouseButton button) const noexcept {
  return button < MouseButton::Count
         && m_buttonReleased[static_cast<size_t>(button)];
}

float InputState::cursorX() const noexcept {
  return m_cursorX;
}

float InputState::cursorY() const noexcept {
  return m_cursorY;
}

float InputState::viewportWidth() const noexcept {
  return m_viewportWidth;
}

float InputState::viewportHeight() const noexcept {
  return m_viewportHeight;
}

}
