#include "inversepool.h"

#include <algorithm>
#include <cstring>

namespace talus::dynamics {

namespace {

constexpr size_t FIRST_SWEEP = 256u;

uint64_t bitsHash(const ElementInverse& block) {
  uint64_t hash = 1469598103934665603ull;

  for (const double value : block) {
    uint64_t bits = 0u;

    std::memcpy(&bits, &value, sizeof(bits));
    hash = (hash ^ bits) * 1099511628211ull;
    hash ^= hash >> 29u;
  }

  return hash;
}

bool sameBits(const ElementInverse& a, const ElementInverse& b) {
  return std::memcmp(a.data(), b.data(), sizeof(ElementInverse)) == 0;
}

}

std::shared_ptr<const ElementInverse> InversePool::intern(std::shared_ptr<const ElementInverse> block) {
  const uint64_t key = bitsHash(*block);
  const auto [first, last] = m_blocks.equal_range(key);

  for (auto at = first; at != last; ++at) {
    if (sameBits(*at->second, *block)) {
      return at->second;
    }
  }

  if (m_blocks.size() >= std::max(m_sweepAt, FIRST_SWEEP)) {
    std::erase_if(m_blocks, [](const auto& entry) { return entry.second.use_count() == 1; });
    m_sweepAt = 2u * m_blocks.size();
  }

  m_blocks.emplace(key, block);

  return block;
}

size_t InversePool::size() const {
  return m_blocks.size();
}

}
