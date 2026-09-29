#include "redistance.ih"

namespace talus::field {

namespace {

constexpr uint32_t BAND_VOXELS = 8u;

constexpr float FAR = std::numeric_limits<float>::max();

uint32_t latticeIndex(const SampledSdf& field, const glm::uvec3& at) {
  return at.x + field.resolution.x * (at.y + field.resolution.y * at.z);
}

float larger(const float* d, const float* h, uint32_t k) {
  double a = 0.0;
  double b = 0.0;
  double c = -1.0;

  for (uint32_t i = 0; i < k; ++i) {
    const double weight = 1.0 / (static_cast<double>(h[i]) * h[i]);

    a += weight;
    b -= 2.0 * weight * d[i];
    c += weight * static_cast<double>(d[i]) * d[i];
  }

  const double discriminant = b * b - 4.0 * a * c;

  if (discriminant < 0.0) {
    return FAR;
  }

  return static_cast<float>((-b + std::sqrt(discriminant)) / (2.0 * a));
}

struct Grid {
  glm::uvec3 extent{0u};

  std::vector<float> value;
  std::vector<uint8_t> frozen;
  std::vector<uint8_t> negative;

  size_t at(uint32_t x, uint32_t y, uint32_t z) const {
    return x + static_cast<size_t>(extent.x) * (y + extent.y * z);
  }
};

float axisMinimum(
  const Grid& grid,
  uint32_t x,
  uint32_t y,
  uint32_t z,
  uint32_t axis
) {
  float lowest = FAR;

  const glm::uvec3 at(x, y, z);

  if (at[axis] > 0u) {
    glm::uvec3 before = at;
    before[axis] -= 1u;
    lowest =
      std::min(lowest, grid.value[grid.at(before.x, before.y, before.z)]);
  }

  if (at[axis] + 1u < grid.extent[axis]) {
    glm::uvec3 after = at;
    after[axis] += 1u;
    lowest = std::min(lowest, grid.value[grid.at(after.x, after.y, after.z)]);
  }

  return lowest;
}

}

float godunov(float a, float b, float c, const glm::vec3& h) {
  float d[3] = {a, b, c};
  float spacing[3] = {h.x, h.y, h.z};

  for (uint32_t i = 0; i < 2u; ++i) {
    for (uint32_t j = i + 1u; j < 3u; ++j) {
      if (d[j] < d[i]) {
        std::swap(d[i], d[j]);
        std::swap(spacing[i], spacing[j]);
      }
    }
  }

  if (d[0] >= FAR) {
    return FAR;
  }

  float answer = FAR;

  for (uint32_t k = 1u; k <= 3u; ++k) {
    if (k > 1u && d[k - 1u] >= FAR) {
      break;
    }

    answer = k == 1u ? d[0] + spacing[0] : larger(d, spacing, k);

    if (k == 3u || answer <= d[k]) {
      break;
    }
  }

  return answer;
}

float redistance(
  SampledSdf& field,
  const SampleBox& box,
  std::span<const uint8_t> damaged,
  std::span<float> values,
  uint32_t sweeps
) {
  const glm::uvec3 beyond = box.minimum + box.extent;

  if (
    beyond.x > field.resolution.x || beyond.y > field.resolution.y
    || beyond.z > field.resolution.z
  ) {
    throw std::runtime_error("A redistance must stay inside the lattice");
  }

  const size_t expected =
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z;

  if (damaged.size() != expected) {
    throw std::runtime_error(
      "A redistance needs one damaged flag per sample in the box"
    );
  }

  if (!values.empty() && values.size() != expected) {
    throw std::runtime_error(
      "A redistance buffer must be one value per sample in the box"
    );
  }

  if (box.extent.x < 3u || box.extent.y < 3u || box.extent.z < 3u) {
    return 0.0f;
  }

  Grid grid;
  grid.extent = box.extent;

  const size_t count =
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z;

  grid.value.assign(count, FAR);
  grid.frozen.assign(count, 0u);
  grid.negative.assign(count, 0u);

  std::vector<float> signed_(count, 0.0f);

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const size_t here = grid.at(x, y, z);

        const float value =
          values.empty()
            ? field.distances
                [latticeIndex(field, box.minimum + glm::uvec3(x, y, z))]
            : values[here];

        signed_[here] = value;
        grid.negative[here] = value < 0.0f ? 1u : 0u;
      }
    }
  }

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const size_t here = grid.at(x, y, z);
        const float value = signed_[here];

        bool bordersSurface = false;

        for (uint32_t axis = 0; axis < 3u; ++axis) {
          const glm::uvec3 at(x, y, z);

          for (int step = -1; step <= 1; step += 2) {
            const int64_t moved =
              static_cast<int64_t>(at[axis]) + static_cast<int64_t>(step);

            if (moved < 0 || moved >= static_cast<int64_t>(grid.extent[axis])) {
              continue;
            }

            glm::uvec3 other = at;
            other[axis] = static_cast<uint32_t>(moved);

            const float across = signed_[grid.at(other.x, other.y, other.z)];

            if ((value < 0.0f) == (across < 0.0f)) {
              continue;
            }

            bordersSurface = true;
            break;
          }
        }

        if (bordersSurface) {
          grid.value[here] = std::abs(value);
          grid.frozen[here] = 1u;
        }
      }
    }
  }

  for (size_t i = 0; i < count; ++i) {
    if (grid.frozen[i] != 0u || damaged[i] != 0u) {
      continue;
    }

    grid.value[i] = std::abs(signed_[i]);
    grid.frozen[i] = 1u;
  }

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const bool shell = x == 0u || y == 0u || z == 0u
                           || x + 1u == box.extent.x || y + 1u == box.extent.y
                           || z + 1u == box.extent.z;

        if (!shell) {
          continue;
        }

        const size_t here = grid.at(x, y, z);

        if (grid.frozen[here] != 0u) {
          continue;
        }

        grid.value[here] = std::abs(signed_[here]);
        grid.frozen[here] = 1u;
      }
    }
  }

  float lastMove = 0.0f;

  for (uint32_t pass = 0; pass < sweeps; ++pass) {
    lastMove = 0.0f;

    const uint32_t ordering = pass % 8u;

    const bool downX = (ordering & 1u) != 0u;
    const bool downY = (ordering & 2u) != 0u;
    const bool downZ = (ordering & 4u) != 0u;

    for (uint32_t iz = 0; iz < box.extent.z; ++iz) {
      const uint32_t z = downZ ? box.extent.z - 1u - iz : iz;

      for (uint32_t iy = 0; iy < box.extent.y; ++iy) {
        const uint32_t y = downY ? box.extent.y - 1u - iy : iy;

        for (uint32_t ix = 0; ix < box.extent.x; ++ix) {
          const uint32_t x = downX ? box.extent.x - 1u - ix : ix;

          const size_t here = grid.at(x, y, z);

          if (grid.frozen[here] != 0u) {
            continue;
          }

          const float candidate = godunov(
            axisMinimum(grid, x, y, z, 0u),
            axisMinimum(grid, x, y, z, 1u),
            axisMinimum(grid, x, y, z, 2u),
            field.voxelSize
          );

          const float settled = std::min(grid.value[here], candidate);

          if (grid.value[here] < FAR && settled < grid.value[here]) {
            lastMove = std::max(lastMove, grid.value[here] - settled);
          }

          grid.value[here] = settled;
        }
      }
    }
  }

  std::vector<float> repaired(count, 0.0f);

  for (size_t i = 0; i < count; ++i) {
    const float magnitude =
      grid.value[i] >= FAR ? std::abs(signed_[i]) : grid.value[i];

    repaired[i] = grid.negative[i] != 0u ? -magnitude : magnitude;
  }

  if (!values.empty()) {
    std::copy(repaired.begin(), repaired.end(), values.begin());
  } else {
    std::vector<uint8_t> materials(count, 0u);

    size_t at = 0;

    for (uint32_t z = 0; z < box.extent.z; ++z) {
      for (uint32_t y = 0; y < box.extent.y; ++y) {
        for (uint32_t x = 0; x < box.extent.x; ++x) {
          materials[at] =
            field.materials
              [latticeIndex(field, box.minimum + glm::uvec3(x, y, z))];

          ++at;
        }
      }
    }

    write(field, box, repaired, materials);
  }

  return lastMove;
}

float redistance(
  SampledSdf& field,
  const SampleBox& box,
  std::span<const uint8_t> damaged,
  uint32_t sweeps
) {
  return redistance(field, box, damaged, std::span<float>(), sweeps);
}

float redistance(SampledSdf& field, const SampleBox& box) {
  const std::vector<uint8_t> everywhere(
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z,
    1u
  );

  return redistance(field, box, everywhere);
}

}
