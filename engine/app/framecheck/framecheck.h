#pragma once

#include <talus/gpu/context/context.h>
#include <app/startupoptions/startupoptions.h>
#include <talus/gpu/swapchain/swapchain.h>

#include <vulkan/vulkan.h>

#include <cstdint>

#if TALUS_TOOLS

  #include <talus/imagecompare/imagecompare.h>

  #include <fstream>
  #include <string>

class FrameCheck {
public:
  FrameCheck(
    const talus::gpu::Context& context,
    const talus::gpu::SwapChain& swapChain,
    const StartupOptions& options
  );

  FrameCheck(const FrameCheck&) = delete;
  FrameCheck& operator=(const FrameCheck&) = delete;
  FrameCheck(FrameCheck&&) = delete;
  FrameCheck& operator=(FrameCheck&&) = delete;

  ~FrameCheck();

  bool wanted() const noexcept;

  bool capturing() const noexcept;

  double lastCaptureMs() const noexcept;

  void run(VkImage image);

  bool ran() const noexcept;
  bool failed() const noexcept;

private:
  talus::image::Bitmap read(VkImage image) const;
  [[nodiscard]]
  bool compareToReference(const talus::image::Bitmap& frame) const;

  const talus::gpu::Context& m_context;
  const talus::gpu::SwapChain& m_swapChain;

  std::string m_capture;
  uint32_t m_captured = 0;
  double m_lastCaptureMs = 0.0;

  std::ofstream m_stream;
  uint32_t m_width = 0;
  uint32_t m_height = 0;

  std::string m_screenshot;
  std::string m_reference;

  bool m_ran = false;
  bool m_failed = false;
};

#else

class FrameCheck {
public:
  FrameCheck(
    const talus::gpu::Context&,
    const talus::gpu::SwapChain&,
    const StartupOptions&
  ) {}

  FrameCheck(const FrameCheck&) = delete;
  FrameCheck& operator=(const FrameCheck&) = delete;
  FrameCheck(FrameCheck&&) = delete;
  FrameCheck& operator=(FrameCheck&&) = delete;

  ~FrameCheck() = default;

  bool wanted() const noexcept {
    return false;
  }

  bool capturing() const noexcept {
    return false;
  }

  double lastCaptureMs() const noexcept {
    return 0.0;
  }

  void run(VkImage) {}

  bool ran() const noexcept {
    return false;
  }

  bool failed() const noexcept {
    return false;
  }
};

#endif
