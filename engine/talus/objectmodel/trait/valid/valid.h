#pragma once

namespace talus::objectmodel {

template<typename T>
[[nodiscard]] bool valid(const T&) noexcept {
  return true;
}

}
