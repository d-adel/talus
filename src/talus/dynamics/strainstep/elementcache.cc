#include "elementcache.h"

#include <algorithm>
#include <utility>

namespace talus::dynamics {

ElementCache::ElementCache(size_t capacity)
  : m_capacity(capacity) {}

const BuiltElements* ElementCache::find(const std::string& key) {
  const auto found = m_entries.find(key);

  if (found == m_entries.end()) {
    return nullptr;
  }

  found->second.used = ++m_clock;

  return &found->second.built;
}

void ElementCache::store(std::string key, BuiltElements built) {
  const size_t cells = built.elements.size();

  if (const auto old = m_entries.find(key); old != m_entries.end()) {
    m_cells -= old->second.built.elements.size();
    m_entries.erase(old);
  }

  if (cells > m_capacity) {
    return;
  }

  while (m_cells + cells > m_capacity) {
    const auto oldest = std::min_element(m_entries.begin(), m_entries.end(), [](const auto& a, const auto& b) {
      return a.second.used < b.second.used;
    });

    m_cells -= oldest->second.built.elements.size();
    m_entries.erase(oldest);
  }

  m_entries.emplace(std::move(key), Entry{std::move(built), ++m_clock});
  m_cells += cells;
}

size_t ElementCache::cells() const {
  return m_cells;
}

size_t ElementCache::size() const {
  return m_entries.size();
}

}
