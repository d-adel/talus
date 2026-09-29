#include "framecheck.ih"

namespace {

constexpr talus::image::Tolerance TOLERANCE{0, 0, 255};

}

FrameCheck::FrameCheck(
  const talus::gpu::Context& context,
  const talus::gpu::SwapChain& swapChain,
  const StartupOptions& options
)
    : m_context(context),
      m_swapChain(swapChain),
      m_capture(options.capture),
      m_screenshot(options.screenshot),
      m_reference(options.golden) {}

bool FrameCheck::wanted() const noexcept {
  return !m_screenshot.empty() || !m_reference.empty();
}

FrameCheck::~FrameCheck() {
  if (!m_stream.is_open()) {
    return;
  }

  m_stream.close();

  std::cout << "Recorded " << m_captured << " frames of " << m_width << "x"
            << m_height << " to " << m_capture << "/frames.raw\n"
            << "  ffmpeg -f rawvideo -pixel_format rgba -video_size " << m_width
            << "x" << m_height << " -framerate 60 -i " << m_capture
            << "/frames.raw ...\n";
}

double FrameCheck::lastCaptureMs() const noexcept {
  return m_lastCaptureMs;
}

bool FrameCheck::capturing() const noexcept {
  return !m_capture.empty();
}

bool FrameCheck::ran() const noexcept {
  return m_ran;
}

bool FrameCheck::failed() const noexcept {
  return m_failed;
}

talus::image::Bitmap FrameCheck::read(VkImage image) const {
  const VkExtent2D extent = m_swapChain.extent();

  if (extent.width == 0 || extent.height == 0) {
    throw std::runtime_error("Cannot capture a frame from a zero-sized window");
  }

  return talus::capture::read(
    m_context.device(),
    image,
    VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
    extent,
    m_swapChain.imageFormat()
  );
}

bool FrameCheck::compareToReference(const talus::image::Bitmap& frame) const {
  const std::filesystem::path reference(m_reference);

  if (!std::filesystem::exists(reference)) {
    talus::image::save(frame, m_reference);

    std::cout << "golden " << m_reference
              << ": MISSING, wrote it. Look at it, then commit it.\n";

    return false;
  }

  const talus::image::Bitmap expected = talus::image::load(m_reference);

  const talus::image::Difference difference =
    talus::image::compare(expected, frame, TOLERANCE);

  if (talus::image::passes(difference, TOLERANCE)) {
    std::cout << "golden " << m_reference << ": ok\n";
    return true;
  }

  std::cout << "golden " << m_reference << ": FAILED\n";

  if (!difference.comparable) {
    std::cout << "  size " << expected.width << "x" << expected.height
              << " became " << frame.width << "x" << frame.height << '\n';
    return false;
  }

  const uint64_t total =
    static_cast<uint64_t>(expected.width) * expected.height;

  std::cout << "  " << difference.differingPixels << " of " << total
            << " pixels differ, worst channel delta "
            << difference.worstChannelDelta << " at (" << difference.worstX
            << ", " << difference.worstY << ")\n";

  const std::string stem = reference.stem().string();

  talus::image::save(frame, stem + "-actual.png");
  talus::image::save(
    talus::image::differenceImage(expected, frame, TOLERANCE),
    stem + "-diff.png"
  );

  std::cout << "  wrote " << stem << "-actual.png and " << stem
            << "-diff.png\n";

  return false;
}

void FrameCheck::run(VkImage image) {
  const auto started = std::chrono::steady_clock::now();

  const talus::image::Bitmap frame = read(image);

  if (!m_capture.empty()) {
    if (!m_stream.is_open()) {
      m_stream.open(m_capture + "/frames.raw", std::ios::binary);

      if (!m_stream) {
        throw std::runtime_error("Cannot write the recording to " + m_capture);
      }

      m_width = frame.width;
      m_height = frame.height;
    }

    m_stream.write(
      reinterpret_cast<const char*>(frame.pixels.data()),
      static_cast<std::streamsize>(frame.pixels.size())
    );

    ++m_captured;

    m_lastCaptureMs = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - started
    )
                        .count();

    return;
  }

  if (!m_screenshot.empty()) {
    talus::image::save(frame, m_screenshot);

    std::cout << "captured " << frame.width << "x" << frame.height << " to "
              << m_screenshot << '\n';
  }

  if (!m_reference.empty()) {
    m_failed = !compareToReference(frame);
  }

  m_ran = true;
}
