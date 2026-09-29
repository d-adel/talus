#pragma once

#include <talus/systems/input/input.h>

namespace talus::app {

[[nodiscard]] talus::systems::Key keyFromGlfw(int key);

[[nodiscard]] talus::systems::MouseButton buttonFromGlfw(int button);

}
