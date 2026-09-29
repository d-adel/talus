#include "asset.ih"

namespace talus::core {

void AssetRegistry::bind(AssetId id, uint32_t index) {
  if (id == INVALID_ASSET) {
    throw std::runtime_error("an asset cannot be bound under the null id");
  }

  for (auto& bound : m_bound) {
    if (bound.first == id) {
      bound.second = index;

      return;
    }
  }

  m_bound.emplace_back(id, index);
}

uint32_t AssetRegistry::index(AssetId id) const {
  for (const auto& bound : m_bound) {
    if (bound.first == id) {
      return bound.second;
    }
  }

  return UNBOUND_ASSET;
}

AssetId AssetRegistry::id(uint32_t index) const {
  for (const auto& bound : m_bound) {
    if (bound.second == index) {
      return bound.first;
    }
  }

  return INVALID_ASSET;
}

size_t AssetRegistry::size() const noexcept {
  return m_bound.size();
}

}
