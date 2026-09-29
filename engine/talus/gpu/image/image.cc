#include "image.ih"

namespace talus::gpu {

VkImageView Image::createImageView(
  VkDevice device,
  VkImage image,
  VkFormat format,
  VkImageAspectFlags aspectFlags,
  VkImageViewType viewType
) {
  VkImageViewCreateInfo viewInfo{};
  viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  viewInfo.image = image;
  viewInfo.viewType = viewType;
  viewInfo.format = format;
  viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  viewInfo.subresourceRange.baseMipLevel = 0;
  viewInfo.subresourceRange.levelCount = 1;
  viewInfo.subresourceRange.baseArrayLayer = 0;
  viewInfo.subresourceRange.layerCount = 1;
  viewInfo.subresourceRange.aspectMask = aspectFlags;

  VkImageView imageView;

  if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
    throw std::runtime_error("failed to create image view!");
  }

  return imageView;
}

Image::Image(
  const Device& device,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkFormat format,
  VkImageTiling tiling,
  VkImageUsageFlags usage,
  VkMemoryPropertyFlags properties,
  VkImageAspectFlags aspect
)
    : m_device(device.vkDevice()) {
  createImage(device, width, height, depth, format, tiling, usage, properties);

  m_imageView = createImageView(
    device.vkDevice(),
    m_image,
    format,
    aspect,
    depth > 1 ? VK_IMAGE_VIEW_TYPE_3D : VK_IMAGE_VIEW_TYPE_2D
  );
}

Image::Image(Image&& other) noexcept
    : m_image(std::exchange(other.m_image, VK_NULL_HANDLE)),
      m_imageView(std::exchange(other.m_imageView, VK_NULL_HANDLE)),
      m_imageMemory(std::exchange(other.m_imageMemory, VK_NULL_HANDLE)),
      m_device(std::exchange(other.m_device, VK_NULL_HANDLE)) {}

Image& Image::operator=(Image&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  destroy();

  m_image = std::exchange(other.m_image, VK_NULL_HANDLE);

  m_imageMemory = std::exchange(other.m_imageMemory, VK_NULL_HANDLE);

  m_imageView = std::exchange(other.m_imageView, VK_NULL_HANDLE);

  m_device = std::exchange(other.m_device, VK_NULL_HANDLE);

  return *this;
}

Image::~Image() {
  destroy();
}

void Image::destroy() noexcept {
  if (m_device == VK_NULL_HANDLE) {
    return;
  }

  vkDestroyImageView(m_device, m_imageView, nullptr);

  vkDestroyImage(m_device, m_image, nullptr);
  vkFreeMemory(m_device, m_imageMemory, nullptr);
}

VkImage Image::image() const noexcept {
  return m_image;
}

VkImageView Image::imageView() const noexcept {
  return m_imageView;
}

VkDeviceMemory Image::memory() const noexcept {
  return m_imageMemory;
}

void Image::createImage(
  const Device& device,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkFormat format,
  VkImageTiling tiling,
  VkImageUsageFlags usage,
  VkMemoryPropertyFlags properties
) {
  VkImageCreateInfo imageInfo{};
  imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageInfo.imageType = depth > 1 ? VK_IMAGE_TYPE_3D : VK_IMAGE_TYPE_2D;
  imageInfo.extent.width = static_cast<uint32_t>(width);
  imageInfo.extent.height = static_cast<uint32_t>(height);
  imageInfo.extent.depth = depth;
  imageInfo.mipLevels = 1;
  imageInfo.arrayLayers = 1;
  imageInfo.format = format;
  imageInfo.tiling = tiling;
  imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  imageInfo.usage = usage;
  imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
  imageInfo.flags = 0;

  if (
    vkCreateImage(device.vkDevice(), &imageInfo, nullptr, &m_image)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create image");
  }

  VkMemoryRequirements memRequirements;
  vkGetImageMemoryRequirements(device.vkDevice(), m_image, &memRequirements);

  VkMemoryAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocInfo.allocationSize = memRequirements.size;
  allocInfo.memoryTypeIndex =
    device.findMemoryType(memRequirements.memoryTypeBits, properties);

  if (
    vkAllocateMemory(device.vkDevice(), &allocInfo, nullptr, &m_imageMemory)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to allocate image memory");
  }

  if (
    vkBindImageMemory(device.vkDevice(), m_image, m_imageMemory, 0)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to bind image memory");
  }
}

void Image::transition(
  const Device& device,
  VkImageLayout oldLayout,
  VkImageLayout newLayout
) {
  OneShotCommand cmd;
  cmd.start(device);

  recordTransition(cmd.cmdBuffer(), oldLayout, newLayout);

  cmd.submit(device);
}

void Image::recordTransition(
  VkCommandBuffer commandBuffer,
  VkImageLayout oldLayout,
  VkImageLayout newLayout
) {
  VkImageMemoryBarrier2 barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  barrier.oldLayout = oldLayout;
  barrier.newLayout = newLayout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = m_image;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.baseMipLevel = 0;
  barrier.subresourceRange.levelCount = 1;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = 1;

  if (
    oldLayout == VK_IMAGE_LAYOUT_UNDEFINED
    && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
  ) {
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
    barrier.srcAccessMask = 0;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
  } else if (
    oldLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
    && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
  ) {
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                           | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
  } else if (
    oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
    && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  ) {
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                           | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
  } else {
    throw std::invalid_argument("Unsupported layout transition!");
  }

  VkDependencyInfo depInfo{};
  depInfo.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  depInfo.pImageMemoryBarriers = &barrier;
  depInfo.imageMemoryBarrierCount = 1;

  vkCmdPipelineBarrier2(commandBuffer, &depInfo);
}

void Image::copyFrom(
  const Device& device,
  Buffer& buffer,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkOffset3D offset
) {
  OneShotCommand cmd;
  cmd.start(device);

  recordCopyFrom(cmd.cmdBuffer(), buffer, width, height, depth, offset);

  cmd.submit(device);
}

void Image::recordCopyFrom(
  VkCommandBuffer commandBuffer,
  const Buffer& buffer,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkOffset3D offset
) {
  VkBufferImageCopy region{};
  region.bufferOffset = 0;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;

  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.layerCount = 1;

  region.imageOffset = offset;
  region.imageExtent = {width, height, depth};

  vkCmdCopyBufferToImage(
    commandBuffer,
    buffer.buffer(),
    m_image,
    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
    1,
    &region
  );
}

}
