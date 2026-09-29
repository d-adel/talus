#include "scenetarget.ih"

namespace talus::renderer {

namespace {

constexpr float MIN_SCALE = 1.0f;
constexpr float MAX_SCALE = 4.0f;
constexpr uint32_t MAX_SIDE = 8192u;

VkExtent2D scaled(VkExtent2D extent, float scale) {
  const auto axis = [scale](uint32_t side) {
    const auto grown =
      static_cast<uint32_t>(std::lround(static_cast<float>(side) * scale));

    return std::clamp(grown, 1u, MAX_SIDE);
  };

  return {axis(extent.width), axis(extent.height)};
}

}

SceneTarget::SceneTarget(
  const talus::gpu::Device& device,
  VkFormat format,
  VkExtent2D extent,
  float scale
)
    : m_device(device),
      m_format(format),
      m_windowExtent(extent),
      m_scale(std::clamp(scale, MIN_SCALE, MAX_SCALE)) {
  build();
}

void SceneTarget::build() {
  m_extent = scaled(m_windowExtent, m_scale);

  m_colour = talus::gpu::Image(
    m_device,
    m_extent.width,
    m_extent.height,
    1u,
    m_format,
    VK_IMAGE_TILING_OPTIMAL,
    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
    VK_IMAGE_ASPECT_COLOR_BIT
  );

  m_depth.recreate(m_device, m_extent);
  m_selection.recreate(m_device, m_extent);
}

VkImage SceneTarget::selectionDepthImage() const noexcept {
  return m_selection.vkImage();
}

VkImageView SceneTarget::selectionDepthView() const noexcept {
  return m_selection.image().imageView();
}

void SceneTarget::resize(VkExtent2D extent) {
  if (extent.width == 0u || extent.height == 0u) {
    return;
  }

  m_windowExtent = extent;

  build();
}

void SceneTarget::setScale(float scale) {
  const float wanted = std::clamp(scale, MIN_SCALE, MAX_SCALE);

  if (wanted == m_scale) {
    return;
  }

  m_scale = wanted;

  build();
}

float SceneTarget::scale() const noexcept {
  return m_scale;
}

VkExtent2D SceneTarget::extent() const noexcept {
  return m_extent;
}

VkImage SceneTarget::colourImage() const noexcept {
  return m_colour.image();
}

VkImageView SceneTarget::colourView() const noexcept {
  return m_colour.imageView();
}

VkImageView SceneTarget::depthView() const noexcept {
  return m_depth.image().imageView();
}

VkImage SceneTarget::depthImage() const noexcept {
  return m_depth.vkImage();
}

void SceneTarget::blitTo(
  VkCommandBuffer commands,
  VkImage destination,
  VkExtent2D destinationExtent
) const {
  VkImageBlit region{};
  region.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 0u, 1u};
  region.srcOffsets[0] = {0, 0, 0};
  region.srcOffsets[1] = {
    static_cast<int32_t>(m_extent.width),
    static_cast<int32_t>(m_extent.height),
    1
  };
  region.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0u, 0u, 1u};
  region.dstOffsets[0] = {0, 0, 0};
  region.dstOffsets[1] = {
    static_cast<int32_t>(destinationExtent.width),
    static_cast<int32_t>(destinationExtent.height),
    1
  };

  vkCmdBlitImage(
    commands,
    m_colour.image(),
    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
    destination,
    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
    1u,
    &region,
    VK_FILTER_LINEAR
  );
}

}
