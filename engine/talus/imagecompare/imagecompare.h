#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace talus::image {

struct Bitmap {
  uint32_t width = 0;
  uint32_t height = 0;

  std::vector<uint8_t> pixels;
};

Bitmap load(const std::string& path);
void save(const Bitmap& bitmap, const std::string& path);

struct Difference {
  bool comparable = false;

  uint64_t differingPixels = 0;

  uint32_t worstChannelDelta = 0;
  uint32_t worstX = 0;
  uint32_t worstY = 0;
};

struct Tolerance {
  uint32_t channelDelta = 0;

  uint64_t allowedPixels = 0;

  uint32_t fatalChannelDelta = 255;
};

Difference compare(
  const Bitmap& reference,
  const Bitmap& candidate,
  const Tolerance& tolerance
);

[[nodiscard]]
bool passes(const Difference& difference, const Tolerance& tolerance) noexcept;

Bitmap differenceImage(
  const Bitmap& reference,
  const Bitmap& candidate,
  const Tolerance& tolerance
);

}
