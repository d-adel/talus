#pragma once

namespace talus::objectmodel {

template<typename T>
inline constexpr bool authored = false;

template<typename T>
concept Authored = authored<T>;

}
