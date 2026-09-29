#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace talus::objectmodel {

template<typename T>
class RangeStore {
public:
  struct Range {
    uint32_t offset = 0;
    uint32_t capacity = 0;
  };

  explicit RangeStore(uint32_t poolCapacity, uint32_t initialCapacity = 4)
      : m_poolCapacity(poolCapacity), m_initialCapacity(initialCapacity) {}

  std::optional<Range> grow(
    Range current,
    uint32_t count,
    uint32_t required,
    uint32_t maxCapacity
  ) {
    const std::optional<uint32_t> capacity =
      nextCapacity(current.capacity, required, maxCapacity);

    if (!capacity) {
      return std::nullopt;
    }

    const std::optional<Range> grown = allocate(*capacity);

    if (!grown) {
      return std::nullopt;
    }

    for (uint32_t i = 0; i != count; ++i) {
      m_elements[grown->offset + i] = std::move(m_elements[current.offset + i]);
    }

    release(current);

    return grown;
  }

  void release(Range range) {
    if (range.capacity == 0) {
      return;
    }

    uint32_t offset = range.offset;
    uint32_t capacity = range.capacity;

    auto next = m_free.lower_bound(offset);

    if (next != m_free.begin()) {
      auto previous = std::prev(next);

      if (previous->first + previous->second == offset) {
        offset = previous->first;
        capacity += previous->second;

        m_free.erase(previous);
      }
    }

    next = m_free.lower_bound(offset);

    if (next != m_free.end() && offset + capacity == next->first) {
      capacity += next->second;

      m_free.erase(next);
    }

    if (offset + capacity == m_elements.size()) {
      m_elements.resize(offset);
      return;
    }

    m_free.emplace(offset, capacity);
  }

  std::span<T> at(Range range, uint32_t count) {
    return {m_elements.data() + range.offset, count};
  }

  std::span<const T> at(Range range, uint32_t count) const {
    return {m_elements.data() + range.offset, count};
  }

  std::span<const T> pool() const noexcept {
    return m_elements;
  }

  uint32_t size() const noexcept {
    return static_cast<uint32_t>(m_elements.size());
  }

  uint32_t freeRangeCount() const noexcept {
    return static_cast<uint32_t>(m_free.size());
  }

private:
  std::optional<Range> allocate(uint32_t capacity) {
    if (capacity == 0) {
      return Range{};
    }

    auto best = m_free.end();

    for (auto it = m_free.begin(); it != m_free.end(); ++it) {
      if (it->second < capacity) {
        continue;
      }

      if (best == m_free.end() || it->second < best->second) {
        best = it;
      }
    }

    if (best != m_free.end()) {
      const uint32_t offset = best->first;
      const uint32_t available = best->second;

      m_free.erase(best);

      if (available > capacity) {
        m_free.emplace(offset + capacity, available - capacity);
      }

      return Range{offset, capacity};
    }

    if (m_elements.size() > std::numeric_limits<uint32_t>::max() - capacity) {
      throw std::overflow_error("Range store exceeded uint32_t address space");
    }

    if (m_elements.size() + capacity > m_poolCapacity) {
      return std::nullopt;
    }

    const uint32_t offset = static_cast<uint32_t>(m_elements.size());

    m_elements.resize(m_elements.size() + capacity);

    return Range{offset, capacity};
  }

  std::optional<uint32_t> nextCapacity(
    uint32_t current,
    uint32_t required,
    uint32_t maxCapacity
  ) const {
    if (required > maxCapacity) {
      return std::nullopt;
    }

    uint32_t capacity =
      current == 0 ? std::min(m_initialCapacity, maxCapacity) : current;

    while (capacity < required) {
      capacity = std::min(capacity * 2, maxCapacity);
    }

    return capacity;
  }

  std::vector<T> m_elements;

  std::map<uint32_t, uint32_t> m_free;

  uint32_t m_poolCapacity;
  uint32_t m_initialCapacity;
};

}
