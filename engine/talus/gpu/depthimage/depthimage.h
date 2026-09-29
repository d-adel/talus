#pragma once

#include <talus/gpu/device/device.h>
#include <talus/gpu/image/image.h>

#include <vulkan/vulkan.h>
#include <utility>

namespace talus::gpu {

class DepthImage {
public:
  DepthImage() = default;
  DepthImage(const Device& device, const VkExtent2D& swapChainExtent);

  DepthImage(const DepthImage&) = delete;
  DepthImage& operator=(const DepthImage&) = delete;

  ~DepthImage() = default;

  VkImage vkImage() const noexcept;
  const Image& image() const noexcept;
  const VkFormat& format() const noexcept;

  void recreate(const Device& device, VkExtent2D extent);

private:
  VkFormat findSupportedFormat(
    const VkPhysicalDevice& physicalDevice,
    const std::vector<VkFormat>& candidates,
    VkImageTiling tiling,
    VkFormatFeatureFlags features
  );
  void findDepthFormat(const VkPhysicalDevice& physicalDevice);

  VkFormat m_format = VK_FORMAT_UNDEFINED;
  Image m_depthImage;
};

}
