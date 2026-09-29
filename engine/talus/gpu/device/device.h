#pragma once

#include <vulkan/vulkan.h>

#include <optional>
#include <optional>
#include <vector>

namespace talus::gpu {

const std::vector<const char*> deviceExtensions = {
  VK_KHR_SWAPCHAIN_EXTENSION_NAME
};

struct SwapChainSupportDetails {
  VkSurfaceCapabilitiesKHR capabilities;
  std::vector<VkSurfaceFormatKHR> formats;
  std::vector<VkPresentModeKHR> presentModes;
};

struct QueueFamilyIndices {
  std::optional<uint32_t> graphicsFamily;
  std::optional<uint32_t> presentFamily;

  bool isComplete() const {
    return graphicsFamily.has_value() && presentFamily.has_value();
  }
};

QueueFamilyIndices findQueueFamilies(
  VkPhysicalDevice device,
  VkSurfaceKHR surface
);
std::optional<SwapChainSupportDetails> trySwapChainSupport(
  VkPhysicalDevice device,
  VkSurfaceKHR surface
);

SwapChainSupportDetails querySwapChainSupport(
  VkPhysicalDevice device,
  VkSurfaceKHR surface
);

class Device {
public:
  Device(const VkInstance& instance, const VkSurfaceKHR& surface);
  Device(Device const&) = delete;
  Device& operator=(Device const&) = delete;
  ~Device();

  const VkDevice& vkDevice() const noexcept;
  const VkPhysicalDevice& physicalDevice() const noexcept;
  const VkQueue& graphicsQueue() const noexcept;
  const VkQueue& presentQueue() const noexcept;
  void waitIdle() const;

  uint32_t findMemoryType(
    uint32_t typeFilter,
    VkMemoryPropertyFlags properties
  ) const;
  uint32_t graphicsFamilyIndex() const;

private:
  void setup(VkSurfaceKHR surface);
  void pickPhysicalDevice(VkSurfaceKHR surface);
  void createLogicalDevice(VkSurfaceKHR surface);
  int rateDeviceSuitability(VkPhysicalDevice device);
  bool checkDeviceExtensionSupport(VkPhysicalDevice device);
  bool checkDeviceFeatureSupport(VkPhysicalDevice device);

  VkInstance m_instance = VK_NULL_HANDLE;
  VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
  VkDevice m_device = VK_NULL_HANDLE;
  VkQueue m_graphicsQueue = VK_NULL_HANDLE;
  VkQueue m_presentQueue = VK_NULL_HANDLE;

  QueueFamilyIndices m_indices;
};

}
