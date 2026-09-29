#pragma once

namespace talus::objectmodel {

template<typename T>
inline constexpr bool selfContained = false;

template<typename T>
concept SelfContained = selfContained<T>;

}
