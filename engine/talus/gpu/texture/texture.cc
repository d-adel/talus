#include "texture.ih"

namespace talus::gpu {

Texture::Texture(const Device& device, const std::string& texturePath) {
  int texWidth, texHeight, texChannels;
  stbi_uc* pixels = stbi_load(
    texturePath.c_str(),
    &texWidth,
    &texHeight,
    &texChannels,
    STBI_rgb_alpha
  );

  if (!pixels) {
    throw std::runtime_error("Failed to load texture image");
  }

  const uint32_t width = static_cast<uint32_t>(texWidth);
  const uint32_t height = static_cast<uint32_t>(texHeight);

  createTextureImage(
    device,
    width,
    height,
    1,
    VK_FORMAT_R8G8B8A8_SRGB,
    pixels,
    static_cast<VkDeviceSize>(width) * height * 4,
    0
  );

  stbi_image_free(pixels);
}

Texture::Texture(
  const Device& device,
  uint32_t width,
  uint32_t height,
  VkFormat format,
  const void* pixels
) {
  createTextureImage(
    device,
    width,
    height,
    1,
    format,
    pixels,
    static_cast<VkDeviceSize>(width) * height * 4,
    0
  );
}

Texture::Texture(
  const Device& device,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkFormat format,
  const void* data,
  VkDeviceSize sizeBytes,
  VkImageUsageFlags extraUsage
) {
  createTextureImage(
    device,
    width,
    height,
    depth,
    format,
    data,
    sizeBytes,
    extraUsage
  );
}

Texture::Texture(
  const Device& device,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkFormat format,
  VkImageUsageFlags extraUsage
) {
  m_textureImage = Image(
    device,
    width,
    height,
    depth,
    format,
    VK_IMAGE_TILING_OPTIMAL,
    VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | extraUsage,
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
    VK_IMAGE_ASPECT_COLOR_BIT
  );
}

const Image& Texture::image() const noexcept {
  return m_textureImage;
}

Image& Texture::image() noexcept {
  return m_textureImage;
}

void Texture::createTextureImage(
  const Device& device,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  VkFormat format,
  const void* data,
  VkDeviceSize sizeBytes,
  VkImageUsageFlags extraUsage
) {
  VkDeviceSize imageSize = sizeBytes;

  Buffer stagingBuffer(
    device,
    imageSize,
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  void* mapped;
  if (
    vkMapMemory(
      device.vkDevice(),
      stagingBuffer.memory(),
      0,
      imageSize,
      0,
      &mapped
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map texture staging memory");
  }

  memcpy(mapped, data, static_cast<size_t>(imageSize));
  vkUnmapMemory(device.vkDevice(), stagingBuffer.memory());

  VkImageTiling tiling = VK_IMAGE_TILING_OPTIMAL;
  VkMemoryPropertyFlags properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  VkImageUsageFlags usage =
    VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | extraUsage;
  VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT;

  m_textureImage = Image(
    device,
    width,
    height,
    depth,
    format,
    tiling,
    usage,
    properties,
    aspect
  );

  m_textureImage.transition(
    device,
    VK_IMAGE_LAYOUT_UNDEFINED,
    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
  );

  m_textureImage.copyFrom(device, stagingBuffer, width, height, depth);
  m_textureImage.transition(
    device,
    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  );
}

}
