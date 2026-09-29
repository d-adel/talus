#include "imagecompare.ih"

namespace talus::image {

namespace {

constexpr uint32_t CHANNELS = 4;

uint32_t absoluteDelta(uint8_t a, uint8_t b) noexcept {
  return a > b ? static_cast<uint32_t>(a - b) : static_cast<uint32_t>(b - a);
}

uint32_t pixelDelta(const uint8_t* a, const uint8_t* b) noexcept {
  uint32_t worst = 0;

  for (uint32_t channel = 0; channel < CHANNELS; ++channel) {
    worst = std::max(worst, absoluteDelta(a[channel], b[channel]));
  }

  return worst;
}

size_t byteCount(const Bitmap& bitmap) noexcept {
  return static_cast<size_t>(bitmap.width) * bitmap.height * CHANNELS;
}

bool sameSize(const Bitmap& a, const Bitmap& b) noexcept {
  return a.width == b.width && a.height == b.height;
}

}

Bitmap load(const std::string& path) {
  int width = 0;
  int height = 0;
  int channelsInFile = 0;

  const std::unique_ptr<stbi_uc, void (*)(void*)> pixels(
    stbi_load(
      path.c_str(),
      &width,
      &height,
      &channelsInFile,
      static_cast<int>(CHANNELS)
    ),
    stbi_image_free
  );

  if (pixels == nullptr) {
    const char* reason = stbi_failure_reason();

    throw std::runtime_error(
      "Failed to read image " + path + ": " + (reason == nullptr ? "?" : reason)
    );
  }

  Bitmap bitmap;
  bitmap.width = static_cast<uint32_t>(width);
  bitmap.height = static_cast<uint32_t>(height);

  const size_t count = byteCount(bitmap);
  bitmap.pixels.assign(pixels.get(), pixels.get() + count);

  return bitmap;
}

void save(const Bitmap& bitmap, const std::string& path) {
  if (bitmap.pixels.size() != byteCount(bitmap)) {
    throw std::runtime_error("Bitmap pixel count disagrees with its size");
  }

  const int stride = static_cast<int>(bitmap.width * CHANNELS);

  const int written = stbi_write_png(
    path.c_str(),
    static_cast<int>(bitmap.width),
    static_cast<int>(bitmap.height),
    static_cast<int>(CHANNELS),
    bitmap.pixels.data(),
    stride
  );

  if (written == 0) {
    throw std::runtime_error("Failed to write image " + path);
  }
}

Difference compare(
  const Bitmap& reference,
  const Bitmap& candidate,
  const Tolerance& tolerance
) {
  Difference difference;

  if (!sameSize(reference, candidate)) {
    return difference;
  }

  if (
    reference.pixels.size() != byteCount(reference)
    || candidate.pixels.size() != byteCount(candidate)
  ) {
    throw std::runtime_error("Bitmap pixel count disagrees with its size");
  }

  difference.comparable = true;

  const size_t pixels = static_cast<size_t>(reference.width) * reference.height;

  for (size_t i = 0; i < pixels; ++i) {
    const size_t at = i * CHANNELS;

    const uint32_t delta =
      pixelDelta(&reference.pixels[at], &candidate.pixels[at]);

    if (delta > difference.worstChannelDelta) {
      difference.worstChannelDelta = delta;
      difference.worstX = static_cast<uint32_t>(i % reference.width);
      difference.worstY = static_cast<uint32_t>(i / reference.width);
    }

    if (delta > tolerance.channelDelta) {
      ++difference.differingPixels;
    }
  }

  return difference;
}

bool passes(const Difference& difference, const Tolerance& tolerance) noexcept {
  if (!difference.comparable) {
    return false;
  }

  if (difference.worstChannelDelta >= tolerance.fatalChannelDelta) {
    return false;
  }

  return difference.differingPixels <= tolerance.allowedPixels;
}

Bitmap differenceImage(
  const Bitmap& reference,
  const Bitmap& candidate,
  const Tolerance& tolerance
) {
  if (!sameSize(reference, candidate)) {
    throw std::runtime_error(
      "Cannot picture the difference between images of different sizes"
    );
  }

  Bitmap result;
  result.width = reference.width;
  result.height = reference.height;
  result.pixels.resize(byteCount(reference));

  const size_t pixels = static_cast<size_t>(reference.width) * reference.height;

  for (size_t i = 0; i < pixels; ++i) {
    const size_t at = i * CHANNELS;

    const uint32_t delta =
      pixelDelta(&reference.pixels[at], &candidate.pixels[at]);

    const uint8_t grey = static_cast<uint8_t>(
      (static_cast<uint32_t>(reference.pixels[at]) + reference.pixels[at + 1]
       + reference.pixels[at + 2])
      / 12u
    );

    const bool differs = delta > tolerance.channelDelta;

    result.pixels[at] = differs ? uint8_t{255} : grey;
    result.pixels[at + 1] = differs ? uint8_t{0} : grey;
    result.pixels[at + 2] = differs ? uint8_t{0} : grey;
    result.pixels[at + 3] = 255;
  }

  return result;
}

}
