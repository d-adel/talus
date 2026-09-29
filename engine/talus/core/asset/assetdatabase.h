#pragma once

#include <talus/core/asset/asset.h>

#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>

namespace talus::core {

class AssetDatabase {
public:
  template<typename T>
  [[nodiscard]] AssetRegistry& of() {
    return m_tables[std::type_index(typeid(T))];
  }

  template<typename T>
  [[nodiscard]] const AssetRegistry& of() const {
    const auto found = m_tables.find(std::type_index(typeid(T)));

    return found == m_tables.end() ? empty() : found->second;
  }

  template<typename T>
  [[nodiscard]] uint32_t index(AssetId id) const {
    return of<T>().index(id);
  }

  template<typename T>
  [[nodiscard]] AssetId id(uint32_t index) const {
    return of<T>().id(index);
  }

  template<typename T>
  void bind(AssetId id, uint32_t index) {
    of<T>().bind(id, index);
  }

  [[nodiscard]] size_t kinds() const noexcept {
    return m_tables.size();
  }

private:
  [[nodiscard]] static const AssetRegistry& empty() {
    static const AssetRegistry none;

    return none;
  }

  std::unordered_map<std::type_index, AssetRegistry> m_tables;
};

}
