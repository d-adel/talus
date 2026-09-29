#include "inputbridge.ih"

namespace talus::app {

talus::systems::Key keyFromGlfw(int key) {
  using talus::systems::Key;

  if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z) {
    return static_cast<Key>(
      static_cast<uint16_t>(Key::A) + (key - GLFW_KEY_A)
    );
  }

  if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
    return static_cast<Key>(
      static_cast<uint16_t>(Key::Digit0) + (key - GLFW_KEY_0)
    );
  }

  if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F12) {
    return static_cast<Key>(
      static_cast<uint16_t>(Key::F1) + (key - GLFW_KEY_F1)
    );
  }

  switch (key) {
    case GLFW_KEY_SPACE:
      return Key::Space;
    case GLFW_KEY_ENTER:
      return Key::Enter;
    case GLFW_KEY_ESCAPE:
      return Key::Escape;
    case GLFW_KEY_TAB:
      return Key::Tab;
    case GLFW_KEY_BACKSPACE:
      return Key::Backspace;
    case GLFW_KEY_DELETE:
      return Key::Delete;
    case GLFW_KEY_LEFT_SHIFT:
    case GLFW_KEY_RIGHT_SHIFT:
      return Key::Shift;
    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL:
      return Key::Control;
    case GLFW_KEY_LEFT_ALT:
    case GLFW_KEY_RIGHT_ALT:
      return Key::Alt;
    case GLFW_KEY_LEFT:
      return Key::Left;
    case GLFW_KEY_RIGHT:
      return Key::Right;
    case GLFW_KEY_UP:
      return Key::Up;
    case GLFW_KEY_DOWN:
      return Key::Down;
    default:
      return Key::Unknown;
  }
}

talus::systems::MouseButton buttonFromGlfw(int button) {
  using talus::systems::MouseButton;

  switch (button) {
    case GLFW_MOUSE_BUTTON_RIGHT:
      return MouseButton::Right;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      return MouseButton::Middle;
    default:
      return MouseButton::Left;
  }
}

}
