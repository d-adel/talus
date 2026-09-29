#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace talus::core {

using AssetId = uint64_t;

inline constexpr AssetId INVALID_ASSET = 0;

inline constexpr uint32_t UNBOUND_ASSET = UINT32_MAX;

class AssetRegistry {
public:
  void bind(AssetId id, uint32_t index);

  [[nodiscard]] uint32_t index(AssetId id) const;

  [[nodiscard]] AssetId id(uint32_t index) const;

  [[nodiscard]] size_t size() const noexcept;

private:
  std::vector<std::pair<AssetId, uint32_t>> m_bound;
};

}
