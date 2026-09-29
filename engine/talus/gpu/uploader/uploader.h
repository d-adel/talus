#pragma once

#include <talus/gpu/buffer/buffer.h>
#include <talus/gpu/device/device.h>

#include <vulkan/vulkan.h>

#include <vector>

namespace talus::gpu {

class Uploader {
public:
  explicit Uploader(const Device& device);

  Uploader(const Uploader&) = delete;
  Uploader& operator=(const Uploader&) = delete;
  Uploader(Uploader&&) = delete;
  Uploader& operator=(Uploader&&) = delete;

  ~Uploader();

  [[nodiscard]] VkCommandBuffer begin();

  void keep(Buffer&& staging);

  void submit();

  void retire();

  [[nodiscard]] bool pending() const noexcept;

private:
  struct Batch {
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;
    std::vector<Buffer> staging;
  };

  void release(Batch& batch) noexcept;

  const Device& m_device;
  VkCommandPool m_pool = VK_NULL_HANDLE;

  Batch m_open;
  bool m_recording = false;

  std::vector<Batch> m_inFlight;
};

}
