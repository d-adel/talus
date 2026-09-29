#include "swapchain.ih"

namespace talus::gpu {

SwapChain::SwapChain(const Context& context)
    : m_context(context),
      m_swapChainSupport(querySwapChainSupport(
        m_context.device().physicalDevice(),
        m_context.surface().handle()
      )),
      m_extent(chooseSwapExtent(m_swapChainSupport.capabilities)),
      m_depthImage(context.device(), m_extent) {
  createSwapChain();
  createImageViews();
}

SwapChain::~SwapChain() {
  cleanup();
}

void SwapChain::cleanup() {
  for (auto imageView : m_imageViews) {
    vkDestroyImageView(m_context.device().vkDevice(), imageView, nullptr);
  }
  vkDestroySwapchainKHR(m_context.device().vkDevice(), m_swapChain, nullptr);
}

void SwapChain::setUncapped(bool uncapped) {
  if (m_uncapped == uncapped) {
    return;
  }

  m_uncapped = uncapped;
  recreate();
}

bool SwapChain::uncapped() const noexcept {
  return m_uncapped;
}

const VkSwapchainKHR& SwapChain::handle() const noexcept {
  return m_swapChain;
}

const DepthImage& SwapChain::depthImage() const noexcept {
  return m_depthImage;
}

const VkFormat& SwapChain::imageFormat() const noexcept {
  return m_imageFormat;
}

const VkExtent2D& SwapChain::extent() const noexcept {
  return m_extent;
}

const VkImage& SwapChain::image(uint32_t i) const noexcept {
  return m_images.at(i);
}

const VkImageView& SwapChain::imageView(uint32_t i) const noexcept {
  return m_imageViews.at(i);
}

uint32_t SwapChain::imageCount() const noexcept {
  return static_cast<uint32_t>(m_imageViews.size());
}

void SwapChain::createSwapChain() {
  m_swapChainSupport = querySwapChainSupport(
    m_context.device().physicalDevice(),
    m_context.surface().handle()
  );

  VkSurfaceFormatKHR surfaceFormat =
    chooseSwapSurfaceFormat(m_swapChainSupport.formats);
  VkPresentModeKHR presentMode =
    chooseSwapPresentMode(m_swapChainSupport.presentModes);
  VkExtent2D extent = chooseSwapExtent(m_swapChainSupport.capabilities);

  uint32_t imageCount = m_swapChainSupport.capabilities.minImageCount + 1;
  if (
    m_swapChainSupport.capabilities.maxImageCount > 0
    && imageCount > m_swapChainSupport.capabilities.maxImageCount
  ) {
    imageCount = m_swapChainSupport.capabilities.maxImageCount;
  }

  VkSwapchainCreateInfoKHR createInfo{};
  createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  createInfo.surface = m_context.surface().handle();
  createInfo.minImageCount = imageCount;
  createInfo.imageFormat = surfaceFormat.format;
  createInfo.imageColorSpace = surfaceFormat.colorSpace;
  createInfo.imageExtent = extent;
  createInfo.imageArrayLayers = 1;
  if (
    (m_swapChainSupport.capabilities.supportedUsageFlags
     & VK_IMAGE_USAGE_TRANSFER_SRC_BIT)
    == 0
  ) {
    throw std::runtime_error(
      "This surface cannot be read back: no TRANSFER_SRC usage"
    );
  }

  createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                          | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                          | VK_IMAGE_USAGE_TRANSFER_DST_BIT;

  QueueFamilyIndices indices = findQueueFamilies(
    m_context.device().physicalDevice(),
    m_context.surface().handle()
  );
  uint32_t queueFamilyIndices[] = {
    indices.graphicsFamily.value(),
    indices.presentFamily.value()
  };

  if (indices.graphicsFamily != indices.presentFamily) {
    createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
    createInfo.queueFamilyIndexCount = 2;
    createInfo.pQueueFamilyIndices = queueFamilyIndices;
  } else {
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }

  createInfo.preTransform = m_swapChainSupport.capabilities.currentTransform;

  createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;

  createInfo.presentMode = presentMode;
  createInfo.clipped = VK_TRUE;
  createInfo.oldSwapchain = m_swapChain;

  if (
    vkCreateSwapchainKHR(
      m_context.device().vkDevice(),
      &createInfo,
      nullptr,
      &m_swapChain
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create swap chain");
  }

  if (
    vkGetSwapchainImagesKHR(
      m_context.device().vkDevice(),
      m_swapChain,
      &imageCount,
      nullptr
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed retrieving image count");
  }

  m_images.resize(imageCount);
  if (
    vkGetSwapchainImagesKHR(
      m_context.device().vkDevice(),
      m_swapChain,
      &imageCount,
      m_images.data()
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed setting the swap chain images");
  }

  m_extent = extent;
  m_imageFormat = surfaceFormat.format;
}

void SwapChain::createImageViews() {
  m_imageViews.resize(m_images.size());

  for (size_t i = 0; i < m_images.size(); i++) {
    m_imageViews[i] = Image::createImageView(
      m_context.device().vkDevice(),
      m_images[i],
      m_imageFormat,
      VK_IMAGE_ASPECT_COLOR_BIT
    );
  }
}

void SwapChain::release() {
  vkDeviceWaitIdle(m_context.device().vkDevice());

  cleanup();

  m_imageViews.clear();
  m_images.clear();
  m_swapChain = VK_NULL_HANDLE;
}

void SwapChain::recreate() {
  m_context.windowSystem().waitForNonZeroExtent();

  vkDeviceWaitIdle(m_context.device().vkDevice());

  cleanup();
  m_swapChain = VK_NULL_HANDLE;

  createSwapChain();
  createImageViews();

  m_depthImage.recreate(m_context.device(), m_extent);
}

VkSurfaceFormatKHR SwapChain::chooseSwapSurfaceFormat(
  const std::vector<VkSurfaceFormatKHR>& availableFormats
) {
  for (const auto& availableFormat : availableFormats) {
    if (
      availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB
      && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
    ) {
      return availableFormat;
    }
  }

  return availableFormats[0];
}

VkPresentModeKHR SwapChain::chooseSwapPresentMode(
  const std::vector<VkPresentModeKHR>& availablePresentModes
) {
  if (m_uncapped) {
    for (const auto& availablePresentMode : availablePresentModes) {
      if (availablePresentMode == VK_PRESENT_MODE_IMMEDIATE_KHR) {
        return availablePresentMode;
      }
    }
  }

  return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D SwapChain::chooseSwapExtent(
  const VkSurfaceCapabilitiesKHR& capabilities
) {
  if (
    capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()
  ) {
    return capabilities.currentExtent;
  } else {
    VkExtent2D actualExtent = m_context.windowSystem().framebufferExtent();

    actualExtent.width = std::clamp(
      actualExtent.width,
      capabilities.minImageExtent.width,
      capabilities.maxImageExtent.width
    );
    actualExtent.height = std::clamp(
      actualExtent.height,
      capabilities.minImageExtent.height,
      capabilities.maxImageExtent.height
    );

    return actualExtent;
  }
}

}
