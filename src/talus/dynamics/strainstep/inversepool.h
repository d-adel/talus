#pragma once

#include <talus/dynamics/strainstep/element.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace talus::dynamics {

class InversePool {
public:
  [[nodiscard]] std::shared_ptr<const ElementInverse> intern(
    std::shared_ptr<const ElementInverse> block
  );

  [[nodiscard]] size_t size() const;

private:
  std::unordered_multimap<uint64_t, std::shared_ptr<const ElementInverse>> m_blocks;
  size_t m_sweepAt = 0u;
};

}
