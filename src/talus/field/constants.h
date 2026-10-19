#pragma once

#include <cstdint>

namespace talus::field {

inline constexpr uint32_t FIELD_OP_UNION = 0u;
inline constexpr uint32_t FIELD_OP_SMOOTH_UNION = 1u;
inline constexpr uint32_t FIELD_OP_INTERSECTION = 2u;
inline constexpr uint32_t FIELD_OP_SUBTRACTION = 3u;
inline constexpr uint32_t FIELD_OP_SMOOTH_SUBTRACTION = 4u;

inline constexpr bool fieldOpRemovesMaterial(uint32_t operation) {
  return operation == FIELD_OP_SUBTRACTION || operation == FIELD_OP_SMOOTH_SUBTRACTION;
}

}
