#pragma once

#include <talus/gpu/depthimage/depthimage.h>
#include <talus/gpu/context/context.h>
#include <vector>

namespace talus::gpu {

class SwapChain {
public:
  explicit SwapChain(const Context& context);
  SwapChain(SwapChain const&) = delete;
  SwapChain& operator=(SwapChain const&) = delete;
  ~SwapChain();

  void recreate();
  void release();
  void setUncapped(bool uncapped);
  bool uncapped() const noexcept;

  const VkSwapchainKHR& handle() const noexcept;
  const DepthImage& depthImage() const noexcept;
  const VkFormat& imageFormat() const noexcept;
  const VkExtent2D& extent() const noexcept;
  const VkImage& image(uint32_t i) const noexcept;
  const VkImageView& imageView(uint32_t i) const noexcept;
  uint32_t imageCount() const noexcept;

private:
  void createSwapChain();
  void createImageViews();
  void cleanup();
  VkSurfaceFormatKHR chooseSwapSurfaceFormat(
    const std::vector<VkSurfaceFormatKHR>& availableFormats
  );
  VkPresentModeKHR chooseSwapPresentMode(
    const std::vector<VkPresentModeKHR>& availablePresentModes
  );
  VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);

  const Context& m_context;
  SwapChainSupportDetails m_swapChainSupport;
  VkSwapchainKHR m_swapChain = VK_NULL_HANDLE;
  VkFormat m_imageFormat = VK_FORMAT_UNDEFINED;
  VkExtent2D m_extent;
  DepthImage m_depthImage;
  std::vector<VkImage> m_images;
  std::vector<VkImageView> m_imageViews;
  bool m_uncapped = false;
};

}
