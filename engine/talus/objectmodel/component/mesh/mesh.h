#pragma once

#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/assetbacked/assetbacked.h>

#include <talus/objectmodel/trait/selfcontained/selfcontained.h>

#include <cstdint>

namespace talus::objectmodel {

struct Mesh {
  uint32_t index = UINT32_MAX;
};

template<>
inline constexpr bool selfContained<Mesh> = true;

template<>
inline constexpr bool authored<Mesh> = true;

template<>
inline constexpr bool assetBacked<Mesh> = true;

}
