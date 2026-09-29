#pragma once

#include <talus/core/nametable/nametable.h>

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talus::core {

class Telemetry {
public:
  using Key = NameTable::Index;

  static constexpr Key INVALID_KEY = NameTable::INVALID;

  [[nodiscard]] Key key(std::string_view name);

  [[nodiscard]] Key find(std::string_view name) const;

  void add(Key key, double amount);
  void set(Key key, double amount);
  void max(Key key, double amount);

  [[nodiscard]] double get(Key key) const;

  void clear();

  [[nodiscard]] std::span<const double> values() const noexcept;
  [[nodiscard]] std::span<const std::string> names() const noexcept;

  [[nodiscard]] size_t size() const noexcept;

private:
  NameTable m_names;
  std::vector<double> m_values;
};

}
