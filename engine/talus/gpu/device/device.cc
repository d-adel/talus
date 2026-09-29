#include "device.ih"

namespace talus::gpu {

std::optional<SwapChainSupportDetails> trySwapChainSupport(
  VkPhysicalDevice device,
  VkSurfaceKHR surface
) {
  SwapChainSupportDetails details;

  if (
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(
      device,
      surface,
      &details.capabilities
    )
    != VK_SUCCESS
  ) {
    return std::nullopt;
  }

  uint32_t formatCount;
  if (
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr)
    != VK_SUCCESS
  ) {
    return std::nullopt;
  }

  if (formatCount != 0) {
    details.formats.resize(formatCount);

    if (
      vkGetPhysicalDeviceSurfaceFormatsKHR(
        device,
        surface,
        &formatCount,
        details.formats.data()
      )
      != VK_SUCCESS
    ) {
      return std::nullopt;
    }
  }

  uint32_t presentModeCount;
  if (
    vkGetPhysicalDeviceSurfacePresentModesKHR(
      device,
      surface,
      &presentModeCount,
      nullptr
    )
    != VK_SUCCESS
  ) {
    return std::nullopt;
  }

  if (presentModeCount != 0) {
    details.presentModes.resize(presentModeCount);

    if (
      vkGetPhysicalDeviceSurfacePresentModesKHR(
        device,
        surface,
        &presentModeCount,
        details.presentModes.data()
      )
      != VK_SUCCESS
    ) {
      return std::nullopt;
    }
  }

  return details;
}

SwapChainSupportDetails querySwapChainSupport(
  VkPhysicalDevice device,
  VkSurfaceKHR surface
) {
  std::optional<SwapChainSupportDetails> details =
    trySwapChainSupport(device, surface);

  if (!details.has_value()) {
    throw std::runtime_error(
      "The chosen device cannot present to this surface"
    );
  }

  return *details;
}

QueueFamilyIndices findQueueFamilies(
  VkPhysicalDevice device,
  VkSurfaceKHR surface
) {
  QueueFamilyIndices indices = {};

  uint32_t queueFamilyCount = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

  std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
  vkGetPhysicalDeviceQueueFamilyProperties(
    device,
    &queueFamilyCount,
    queueFamilies.data()
  );

  uint32_t i = 0;
  for (const auto& queueFamily : queueFamilies) {
    VkBool32 presentSupport = false;
    VkResult err =
      vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);

    if (err != VK_SUCCESS) {
      throw std::runtime_error("Failed to verify surface support");
    }

    if (presentSupport) {
      indices.presentFamily = i;
    }

    if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      indices.graphicsFamily = i;
    }

    if (indices.isComplete()) {
      break;
    }
    i++;
  }
  return indices;
}

Device::Device(const VkInstance& instance, const VkSurfaceKHR& surface)
    : m_instance(instance) {
  setup(surface);
}

Device::~Device() {
  vkDestroyDevice(m_device, nullptr);
}

const VkDevice& Device::vkDevice() const noexcept {
  return m_device;
}

const VkPhysicalDevice& Device::physicalDevice() const noexcept {
  return m_physicalDevice;
}

const VkQueue& Device::graphicsQueue() const noexcept {
  return m_graphicsQueue;
}

const VkQueue& Device::presentQueue() const noexcept {
  return m_presentQueue;
}

void Device::waitIdle() const {
  if (vkDeviceWaitIdle(m_device) != VK_SUCCESS) {
    throw std::runtime_error("Failed waiting for the device to go idle");
  }
}

uint32_t Device::findMemoryType(
  uint32_t typeFilter,
  VkMemoryPropertyFlags properties
) const {
  VkPhysicalDeviceMemoryProperties memProperties;
  vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);

  for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
    if (
      (typeFilter & (1 << i))
      && (memProperties.memoryTypes[i].propertyFlags & properties) == properties
    ) {
      return i;
    }
  }

  throw std::runtime_error("Failed to find suitable memory type");
}

uint32_t Device::graphicsFamilyIndex() const {
  return m_indices.graphicsFamily.value();
}

void Device::setup(VkSurfaceKHR surface) {
  pickPhysicalDevice(surface);
  createLogicalDevice(surface);
}

void Device::pickPhysicalDevice(VkSurfaceKHR surface) {
  uint32_t deviceCount = 0;
  VkResult err = vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);

  if (err != VK_SUCCESS) {
    throw std::runtime_error(
      "Error enumerating the physical devices accessible to the Vulkan instance"
    );
  }

  if (deviceCount == 0) {
    throw std::runtime_error("Failed to find GPUs with Vulkan support!");
  }

  std::vector<VkPhysicalDevice> devices(deviceCount);
  err = vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

  if (err != VK_SUCCESS) {
    throw std::runtime_error(
      "Error enumerating the physical devices accessible to the Vulkan instance"
    );
  }

  int bestScore = 0;
  VkPhysicalDevice bestDevice = VK_NULL_HANDLE;

  for (const auto& device : devices) {
    const QueueFamilyIndices indices = findQueueFamilies(device, surface);
    bool extensionsSupported = checkDeviceExtensionSupport(device);
    bool featuresSupported = checkDeviceFeatureSupport(device);
    bool swapChainAdequate = false;

    if (extensionsSupported && featuresSupported) {
      const std::optional<SwapChainSupportDetails> support =
        trySwapChainSupport(device, surface);

      swapChainAdequate = support.has_value() && !support->formats.empty()
                          && !support->presentModes.empty();
    }

    if (
      !indices.isComplete() || !extensionsSupported || !featuresSupported
      || !swapChainAdequate
    ) {
      continue;
    }

    const int score = rateDeviceSuitability(device);

    if (score > bestScore) {
      bestScore = score;
      bestDevice = device;
      m_indices = indices;
    }
  }

  if (bestDevice == VK_NULL_HANDLE) {
    throw std::runtime_error("Failed to find a suitable GPU");
  }

  m_physicalDevice = bestDevice;

  VkPhysicalDeviceProperties deviceProperties;
  vkGetPhysicalDeviceProperties(m_physicalDevice, &deviceProperties);

  std::cout << deviceProperties.deviceName << '\n';
}

int Device::rateDeviceSuitability(VkPhysicalDevice device) {
  VkPhysicalDeviceProperties deviceProperties;
  vkGetPhysicalDeviceProperties(device, &deviceProperties);

  VkPhysicalDeviceFeatures deviceFeatures;
  vkGetPhysicalDeviceFeatures(device, &deviceFeatures);

  int score = 0;

  if (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
    score += 1000;
  }

  score += deviceProperties.limits.maxImageDimension2D;

  if (!deviceFeatures.geometryShader) {
    return 0;
  }

  return score;
}

void Device::createLogicalDevice(VkSurfaceKHR surface) {
  m_indices = findQueueFamilies(m_physicalDevice, surface);

  std::set<uint32_t> uniqueQueueFamilies = {
    m_indices.graphicsFamily.value(),
    m_indices.presentFamily.value()
  };

  std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;

  float queuePriority = 1.0f;
  for (uint32_t queueFamily : uniqueQueueFamilies) {
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = queueFamily;
    queueCreateInfo.queueCount = 1;
    queueCreateInfo.pQueuePriorities = &queuePriority;
    queueCreateInfos.push_back(queueCreateInfo);
  }

  VkPhysicalDeviceFeatures deviceFeatures{};
  deviceFeatures.samplerAnisotropy = VK_TRUE;
  deviceFeatures.independentBlend = VK_TRUE;
  deviceFeatures.shaderSampledImageArrayDynamicIndexing = VK_TRUE;
  deviceFeatures.shaderStorageImageArrayDynamicIndexing = VK_TRUE;

  VkPhysicalDeviceVulkan12Features features12{};
  features12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  features12.descriptorBindingPartiallyBound = VK_TRUE;
  features12.descriptorBindingSampledImageUpdateAfterBind = VK_TRUE;

  VkPhysicalDeviceVulkan13Features features13{};
  features13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  features13.dynamicRendering = VK_TRUE;
  features13.synchronization2 = VK_TRUE;
  features13.pNext = &features12;

  VkDeviceCreateInfo createInfo{};
  createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  createInfo.pNext = &features13;
  createInfo.queueCreateInfoCount =
    static_cast<uint32_t>(queueCreateInfos.size());
  createInfo.pQueueCreateInfos = queueCreateInfos.data();

  createInfo.pEnabledFeatures = &deviceFeatures;
  createInfo.enabledLayerCount = 0;

  createInfo.enabledExtensionCount =
    static_cast<uint32_t>(deviceExtensions.size());
  createInfo.ppEnabledExtensionNames = deviceExtensions.data();

  if (
    vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create logical device");
  }

  vkGetDeviceQueue(
    m_device,
    m_indices.graphicsFamily.value(),
    0,
    &m_graphicsQueue
  );
  assert(m_graphicsQueue != VK_NULL_HANDLE);
  vkGetDeviceQueue(
    m_device,
    m_indices.presentFamily.value(),
    0,
    &m_presentQueue
  );
  assert(m_presentQueue != VK_NULL_HANDLE);
}

bool Device::checkDeviceExtensionSupport(VkPhysicalDevice device) {
  uint32_t extensionCount;
  VkResult result = vkEnumerateDeviceExtensionProperties(
    device,
    nullptr,
    &extensionCount,
    nullptr
  );
  if (result != VK_SUCCESS) {
    throw std::runtime_error("Failed enumerating device extension properties");
  }

  std::vector<VkExtensionProperties> availableExtensions(extensionCount);
  result = vkEnumerateDeviceExtensionProperties(
    device,
    nullptr,
    &extensionCount,
    availableExtensions.data()
  );
  if (result != VK_SUCCESS) {
    throw std::runtime_error("Failed enumerating device extension properties");
  }

  std::set<std::string> requiredExtensions(
    deviceExtensions.begin(),
    deviceExtensions.end()
  );

  for (const auto& extension : availableExtensions) {
    requiredExtensions.erase(extension.extensionName);
  }

  return requiredExtensions.empty();
}

bool Device::checkDeviceFeatureSupport(VkPhysicalDevice device) {
  VkPhysicalDeviceFeatures2 physicalDeviceFeatures = {};
  physicalDeviceFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;

  VkPhysicalDeviceVulkan12Features features12{};
  features12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;

  VkPhysicalDeviceVulkan13Features features13 = {};
  features13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  features13.pNext = &features12;

  physicalDeviceFeatures.pNext = &features13;
  vkGetPhysicalDeviceFeatures2(device, &physicalDeviceFeatures);

  return features13.dynamicRendering == VK_TRUE
         && features13.synchronization2 == VK_TRUE
         && physicalDeviceFeatures.features.samplerAnisotropy == VK_TRUE
         && physicalDeviceFeatures.features.independentBlend == VK_TRUE
         && physicalDeviceFeatures.features
                .shaderSampledImageArrayDynamicIndexing
              == VK_TRUE
         && physicalDeviceFeatures.features
                .shaderStorageImageArrayDynamicIndexing
              == VK_TRUE
         && features12.descriptorBindingPartiallyBound == VK_TRUE
         && features12.descriptorBindingSampledImageUpdateAfterBind == VK_TRUE;
}

}
