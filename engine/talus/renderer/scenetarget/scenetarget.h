#pragma once

#include <talus/gpu/depthimage/depthimage.h>
#include <talus/gpu/device/device.h>
#include <talus/gpu/image/image.h>

#include <vulkan/vulkan.h>

namespace talus::renderer {

class SceneTarget {
public:
  SceneTarget(
    const talus::gpu::Device& device,
    VkFormat format,
    VkExtent2D extent,
    float scale
  );

  SceneTarget(const SceneTarget&) = delete;
  SceneTarget& operator=(const SceneTarget&) = delete;
  SceneTarget(SceneTarget&&) = delete;
  SceneTarget& operator=(SceneTarget&&) = delete;

  void resize(VkExtent2D extent);

  void setScale(float scale);

  [[nodiscard]] float scale() const noexcept;

  [[nodiscard]] VkExtent2D extent() const noexcept;

  [[nodiscard]] VkImage colourImage() const noexcept;

  [[nodiscard]] VkImageView colourView() const noexcept;

  [[nodiscard]] VkImageView depthView() const noexcept;

  [[nodiscard]] VkImage depthImage() const noexcept;

  [[nodiscard]] VkImage selectionDepthImage() const noexcept;

  [[nodiscard]] VkImageView selectionDepthView() const noexcept;

  void blitTo(
    VkCommandBuffer commands,
    VkImage destination,
    VkExtent2D destinationExtent
  ) const;

private:
  void build();

  const talus::gpu::Device& m_device;

  VkFormat m_format;
  VkExtent2D m_windowExtent;
  VkExtent2D m_extent{};
  float m_scale;

  talus::gpu::Image m_colour;
  talus::gpu::DepthImage m_depth;
  talus::gpu::DepthImage m_selection;
};

}
