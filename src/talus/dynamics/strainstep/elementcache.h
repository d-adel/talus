#pragma once

#include <talus/dynamics/strainstep/element.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace talus::dynamics {

inline constexpr size_t MOST_CACHED_CELLS = 16384u;

struct BuiltElements {
  double lowest = 0.0;
  bool exact = false;
  std::vector<ElementRest> rests;
  std::vector<StrainElement> elements;
};

class ElementCache {
public:
  explicit ElementCache(size_t capacity = MOST_CACHED_CELLS);

  [[nodiscard]] const BuiltElements* find(const std::string& key);

  void store(std::string key, BuiltElements built);

  [[nodiscard]] size_t cells() const;

  [[nodiscard]] size_t size() const;

private:
  struct Entry {
    BuiltElements built;
    uint64_t used = 0u;
  };

  std::unordered_map<std::string, Entry> m_entries;
  size_t m_capacity;
  size_t m_cells = 0u;
  uint64_t m_clock = 0u;
};

}
