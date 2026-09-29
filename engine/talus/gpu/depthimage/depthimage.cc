#include "depthimage.ih"

namespace talus::gpu {

DepthImage::DepthImage(
  const Device& device,
  const VkExtent2D& swapChainExtent
) {
  recreate(device, swapChainExtent);
}

const Image& DepthImage::image() const noexcept {
  return m_depthImage;
}

VkImage DepthImage::vkImage() const noexcept {
  return m_depthImage.image();
}

const VkFormat& DepthImage::format() const noexcept {
  return m_format;
}

void DepthImage::recreate(const Device& device, VkExtent2D swapChainExtent) {
  uint32_t w = swapChainExtent.width;
  uint32_t h = swapChainExtent.height;
  findDepthFormat(device.physicalDevice());
  VkImageTiling tiling = VK_IMAGE_TILING_OPTIMAL;
  VkImageUsageFlags usage =
    VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  VkMemoryPropertyFlags properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  VkImageAspectFlags aspect = VK_IMAGE_ASPECT_DEPTH_BIT;

  m_depthImage =
    Image(device, w, h, 1, m_format, tiling, usage, properties, aspect);
}

VkFormat DepthImage::findSupportedFormat(
  const VkPhysicalDevice& physicalDevice,
  const std::vector<VkFormat>& candidates,
  VkImageTiling tiling,
  VkFormatFeatureFlags features
) {
  for (VkFormat format : candidates) {
    VkFormatProperties props;
    vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

    if (
      tiling == VK_IMAGE_TILING_LINEAR
      && (props.linearTilingFeatures & features) == features
    ) {
      return format;
    } else if (
      tiling == VK_IMAGE_TILING_OPTIMAL
      && (props.optimalTilingFeatures & features) == features
    ) {
      return format;
    }
  }

  throw std::runtime_error("failed to find supported format!");
}

void DepthImage::findDepthFormat(const VkPhysicalDevice& physicalDevice) {
  m_format = findSupportedFormat(
    physicalDevice,
    {VK_FORMAT_D32_SFLOAT,
     VK_FORMAT_D32_SFLOAT_S8_UINT,
     VK_FORMAT_D24_UNORM_S8_UINT},
    VK_IMAGE_TILING_OPTIMAL,
    VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
  );
}

}
