#pragma once

#include <cstdint>

namespace talus::core {

constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;

constexpr uint32_t MAX_TEXTURES = TALUS_MAX_TEXTURES;
constexpr uint32_t MAX_SAMPLERS = TALUS_MAX_SAMPLERS;
constexpr uint32_t MAX_MATERIALS = TALUS_MAX_MATERIALS;
constexpr uint32_t MAX_SDF_VOLUMES = TALUS_MAX_SDF_VOLUMES;
constexpr uint32_t MAX_SDF_PRIMITIVES = TALUS_MAX_SDF_PRIMITIVES;
constexpr size_t MAX_PRIMITIVE_POOL = TALUS_MAX_PRIMITIVE_POOL;

static_assert(
  MAX_MATERIALS <= 256,
  "A material id is stored as a byte in the lattice and read as R8_UINT"
);

constexpr uint32_t TEXTURE_WHITE = 0;
constexpr uint32_t TEXTURE_FLAT_NORMAL = 1;
constexpr uint32_t TEXTURE_BLACK = 2;
constexpr uint32_t RESERVED_TEXTURES = 3;

constexpr uint32_t SAMPLER_LINEAR_REPEAT = TALUS_SAMPLER_LINEAR_REPEAT;
constexpr uint32_t SAMPLER_LINEAR_CLAMP = TALUS_SAMPLER_LINEAR_CLAMP;
constexpr uint32_t SAMPLER_NEAREST_CLAMP = TALUS_SAMPLER_NEAREST_CLAMP;

constexpr uint32_t FIELD_ANALYTIC = 0;
constexpr uint32_t FIELD_SAMPLED = 1;

constexpr uint32_t FIELD_OP_UNION = 0;
constexpr uint32_t FIELD_OP_SMOOTH_UNION = 1;
constexpr uint32_t FIELD_OP_INTERSECTION = 2;
constexpr uint32_t FIELD_OP_SUBTRACTION = 3;

constexpr uint32_t FIELD_OP_SMOOTH_SUBTRACTION = 4;

constexpr bool fieldOpRemovesMaterial(uint32_t operation) {
  return operation == FIELD_OP_SUBTRACTION
         || operation == FIELD_OP_SMOOTH_SUBTRACTION;
}

constexpr float EDIT_RADIUS = 0.35f;

constexpr float EDIT_ADVANCE = 0.5f * EDIT_RADIUS;

constexpr float EDIT_BLEND = 0.6f * EDIT_ADVANCE;

constexpr uint32_t FIELD_GRID_RESOLUTION = 128;

constexpr uint32_t BRICK_CELLS = TALUS_BRICK_CELLS;
constexpr uint32_t BRICKS_PER_AXIS =
  (FIELD_GRID_RESOLUTION - 1u + BRICK_CELLS - 1u) / BRICK_CELLS;
constexpr uint32_t MAX_BRICKS_PER_OBJECT =
  BRICKS_PER_AXIS * BRICKS_PER_AXIS * BRICKS_PER_AXIS;

constexpr uint32_t BRICK_TABLE_STRIDE = 1u + MAX_BRICKS_PER_OBJECT;

static_assert(
  static_cast<uint64_t>(MAX_SDF_VOLUMES) * BRICK_TABLE_STRIDE < (1ull << 24),
  "The bound table's largest index must stay exact in a float"
);
constexpr float FIELD_GRID_MARGIN = 0.5f;

constexpr uint32_t FIELD_GRID_MARGIN_CELLS = 4u;

constexpr float SHADOW_CULL_MARGIN = 0.5f;

}
