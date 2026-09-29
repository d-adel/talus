#pragma once

#include <talus/gpu/device/device.h>
#include <talus/imagecompare/imagecompare.h>

#include <vulkan/vulkan.h>

namespace talus::capture {

talus::image::Bitmap read(
  const talus::gpu::Device& device,
  VkImage image,
  VkImageLayout layout,
  VkExtent2D extent,
  VkFormat format
);

}
