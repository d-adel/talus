#include "win32keys.ih"

namespace talus::capi {

talus::systems::Key keyFromWin32(uint32_t virtualKey) {
  using talus::systems::Key;

  if (virtualKey >= 0x41 && virtualKey <= 0x5A) {
    return static_cast<Key>(
      static_cast<uint16_t>(Key::A) + (virtualKey - 0x41)
    );
  }

  if (virtualKey >= 0x30 && virtualKey <= 0x39) {
    return static_cast<Key>(
      static_cast<uint16_t>(Key::Digit0) + (virtualKey - 0x30)
    );
  }

  if (virtualKey >= 0x70 && virtualKey <= 0x7B) {
    return static_cast<Key>(
      static_cast<uint16_t>(Key::F1) + (virtualKey - 0x70)
    );
  }

  switch (virtualKey) {
    case 0x20:
      return Key::Space;
    case 0x0D:
      return Key::Enter;
    case 0x1B:
      return Key::Escape;
    case 0x09:
      return Key::Tab;
    case 0x08:
      return Key::Backspace;
    case 0x2E:
      return Key::Delete;
    case 0x10:
      return Key::Shift;
    case 0x11:
      return Key::Control;
    case 0x12:
      return Key::Alt;
    case 0x25:
      return Key::Left;
    case 0x26:
      return Key::Up;
    case 0x27:
      return Key::Right;
    case 0x28:
      return Key::Down;
    default:
      return Key::Unknown;
  }
}

}
