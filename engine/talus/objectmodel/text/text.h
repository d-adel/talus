#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <string_view>

namespace talus::objectmodel {

inline constexpr size_t TEXT_CAPACITY = 64u;

struct Text {
  std::array<char, TEXT_CAPACITY> chars{};

  [[nodiscard]] std::string_view view() const noexcept {
    size_t length = 0u;

    while (length < TEXT_CAPACITY && chars[length] != '\0') {
      ++length;
    }

    return {chars.data(), length};
  }

  [[nodiscard]] static Text of(std::string_view value) noexcept {
    Text text{};

    const size_t length = std::min(value.size(), TEXT_CAPACITY - 1u);

    std::memcpy(text.chars.data(), value.data(), length);

    return text;
  }

  friend bool operator==(const Text&, const Text&) = default;
};

}
