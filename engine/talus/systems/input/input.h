#pragma once

#include <array>
#include <cstdint>

namespace talus::systems {

enum class Key : uint16_t {
  Unknown = 0,

  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  J,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z,

  Digit0,
  Digit1,
  Digit2,
  Digit3,
  Digit4,
  Digit5,
  Digit6,
  Digit7,
  Digit8,
  Digit9,

  Space,
  Enter,
  Escape,
  Tab,
  Backspace,
  Delete,

  Shift,
  Control,
  Alt,

  Left,
  Right,
  Up,
  Down,

  F1,
  F2,
  F3,
  F4,
  F5,
  F6,
  F7,
  F8,
  F9,
  F10,
  F11,
  F12,

  Count
};

enum class MouseButton : uint8_t {
  Left,
  Right,
  Middle,

  Count
};

class InputState {
public:
  void beginFrame() noexcept;

  void setKey(Key key, bool down) noexcept;

  void setMouseButton(MouseButton button, bool down) noexcept;

  void setCursor(float x, float y) noexcept;

  void setViewport(float width, float height) noexcept;

  void clear() noexcept;

  [[nodiscard]] bool held(Key key) const noexcept;

  [[nodiscard]] bool pressed(Key key) const noexcept;

  [[nodiscard]] bool released(Key key) const noexcept;

  [[nodiscard]] bool held(MouseButton button) const noexcept;

  [[nodiscard]] bool pressed(MouseButton button) const noexcept;

  [[nodiscard]] bool released(MouseButton button) const noexcept;

  [[nodiscard]] float cursorX() const noexcept;

  [[nodiscard]] float cursorY() const noexcept;

  [[nodiscard]] float viewportWidth() const noexcept;

  [[nodiscard]] float viewportHeight() const noexcept;

private:
  static constexpr size_t KEYS = static_cast<size_t>(Key::Count);
  static constexpr size_t BUTTONS = static_cast<size_t>(MouseButton::Count);

  std::array<bool, KEYS> m_keys{};
  std::array<bool, KEYS> m_keyPressed{};
  std::array<bool, KEYS> m_keyReleased{};

  std::array<bool, BUTTONS> m_buttons{};
  std::array<bool, BUTTONS> m_buttonPressed{};
  std::array<bool, BUTTONS> m_buttonReleased{};

  float m_cursorX = 0.0f;
  float m_cursorY = 0.0f;

  float m_viewportWidth = 0.0f;
  float m_viewportHeight = 0.0f;
};

}
