#pragma once

#include <concepts>
#include <cstdint>

namespace talus::objectmodel {

template<typename T>
inline constexpr bool assetBacked = false;

template<typename T>
concept AssetBacked = assetBacked<T> && requires(T value) {
  { value.index } -> std::convertible_to<uint32_t>;
};

}
