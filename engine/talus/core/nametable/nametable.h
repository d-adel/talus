#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talus::core {

class NameTable {
public:
  using Index = uint32_t;

  static constexpr Index INVALID = UINT32_MAX;

  Index intern(std::string_view name);

  [[nodiscard]] Index find(std::string_view name) const;

  [[nodiscard]] std::string_view name(Index index) const;

  [[nodiscard]] size_t size() const noexcept;

  [[nodiscard]] std::span<const std::string> names() const noexcept;

private:
  std::vector<std::string> m_names;
};

}
