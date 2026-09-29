#pragma once

#include <optional>

namespace talus::platform {

enum class KeyEventType {
  Pressed,
  Released,
  SinglePressed,
  DoublePressed,
  Hold,
  Repeat
};

struct KeyEvent {
  int key;
  KeyEventType type;
};

[[nodiscard]] constexpr std::optional<bool> keyTransition(
  KeyEventType type
) noexcept {
  switch (type) {
    case KeyEventType::Pressed:
      return true;

    case KeyEventType::Released:
      return false;

    default:
      return std::nullopt;
  }
}

enum class MouseButtonEventType {
  Pressed,
  Released
};

struct MouseButtonEvent {
  int button;
  MouseButtonEventType type;
  int mods;
};

}
