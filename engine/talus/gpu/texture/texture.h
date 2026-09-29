#pragma once

#include <talus/gpu/device/device.h>
#include <talus/gpu/image/image.h>

#include <vulkan/vulkan.h>

#include <cstdint>
#include <string>

namespace talus::gpu {

class Texture {
public:
  Texture() = default;
  Texture(const Device& device, const std::string& texturePath);

  Texture(
    const Device& device,
    uint32_t width,
    uint32_t height,
    VkFormat format,
    const void* pixels
  );

  Texture(
    const Device& device,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    VkFormat format,
    const void* data,
    VkDeviceSize sizeBytes,
    VkImageUsageFlags extraUsage = 0
  );

  Texture(
    const Device& device,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    VkFormat format,
    VkImageUsageFlags extraUsage
  );

  const Image& image() const noexcept;

  Image& image() noexcept;

private:
  void createTextureImage(
    const Device& device,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    VkFormat format,
    const void* data,
    VkDeviceSize sizeBytes,
    VkImageUsageFlags extraUsage
  );

  Image m_textureImage;
};

}
