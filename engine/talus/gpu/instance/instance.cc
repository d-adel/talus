#include "instance.ih"

namespace talus::gpu {

Instance::Instance(
  const std::vector<char const*>& validationLayers,
  const std::vector<const char*>& windowExtensions
) {
  setup(validationLayers, windowExtensions);
}

Instance::~Instance() {
  if (m_debugMessenger != VK_NULL_HANDLE) {
    auto destroyFunction =
      reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT")
      );

    if (destroyFunction != nullptr) {
      destroyFunction(m_instance, m_debugMessenger, nullptr);
    }

    m_debugMessenger = VK_NULL_HANDLE;
  }

  vkDestroyInstance(m_instance, nullptr);
}

const VkInstance& Instance::handle() const noexcept {
  return m_instance;
}

VKAPI_ATTR VkBool32 VKAPI_CALL Instance::debugCallback(
  VkDebugUtilsMessageSeverityFlagBitsEXT,
  VkDebugUtilsMessageTypeFlagsEXT,
  const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
  void*
) {
  std::cerr << "Validation layer: " << callbackData->pMessage << '\n';

  return VK_FALSE;
}

std::vector<VkLayerProperties> Instance::enumerateInstanceLayerProperties() {
  uint32_t layerCount = 0;

  VkResult err = vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

  if (err != VK_SUCCESS) {
    throw std::runtime_error(
      "vkEnumerateInstanceLayerProperties layer count query failed"
    );
  }

  std::vector<VkLayerProperties> layerProperties(layerCount);

  err = vkEnumerateInstanceLayerProperties(&layerCount, layerProperties.data());

  if (err != VK_SUCCESS) {
    throw std::runtime_error(
      "vkEnumerateInstanceLayerProperties layer fill failed"
    );
  }

  return layerProperties;
}

void Instance::setup(
  const std::vector<char const*>& validationLayers,
  const std::vector<const char*>& windowExtensions
) {
  createVkInstance(validationLayers, windowExtensions);

  if (enableValidationLayers) {
    setupDebugMessenger();
  }
}

void Instance::createVkInstance(
  const std::vector<char const*>& validationLayers,
  const std::vector<const char*>& windowExtensions
) {
  VkApplicationInfo appInfo{};
  appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  appInfo.pApplicationName = "Vulkan Renderer";
  appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
  appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
  appInfo.apiVersion = VK_API_VERSION_1_4;

  VkInstanceCreateInfo createInfo{};
  createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  createInfo.pApplicationInfo = &appInfo;

  std::vector<char const*> requiredLayers;
  if (enableValidationLayers) {
    requiredLayers.assign(validationLayers.begin(), validationLayers.end());
  }

  auto layerProperties = enumerateInstanceLayerProperties();
  auto unsupportedLayerIt = std::ranges::find_if(
    requiredLayers,
    [&layerProperties](auto const& requiredLayer) {
      return std::ranges::none_of(
        layerProperties,
        [requiredLayer](auto const& layerProperty) {
          return strcmp(layerProperty.layerName, requiredLayer) == 0;
        }
      );
    }
  );

  if (unsupportedLayerIt != requiredLayers.end()) {
    throw std::runtime_error(
      "Required layer not supported: " + std::string(*unsupportedLayerIt)
    );
  }

  if (windowExtensions.empty()) {
    throw std::runtime_error(
      "The window system required no instance extensions"
    );
  }

  std::vector<const char*> requiredExtensions(windowExtensions);

  if (enableValidationLayers) {
    requiredExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  }

  createInfo.enabledExtensionCount =
    static_cast<uint32_t>(requiredExtensions.size());
  createInfo.ppEnabledExtensionNames = requiredExtensions.data();
  createInfo.enabledLayerCount = static_cast<uint32_t>(requiredLayers.size());
  createInfo.ppEnabledLayerNames = requiredLayers.data();

  if (vkCreateInstance(&createInfo, nullptr, &m_instance) != VK_SUCCESS) {
    throw std::runtime_error("failed to create instance");
  }
}

void Instance::setupDebugMessenger() {
  VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};
  debugCreateInfo.sType =
    VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;

  debugCreateInfo.messageSeverity =
    VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
    | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;

  debugCreateInfo.messageType =
    VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
    | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
    | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;

  debugCreateInfo.pfnUserCallback = debugCallback;

  auto function = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
    vkGetInstanceProcAddr(m_instance, "vkCreateDebugUtilsMessengerEXT")
  );

  if (function == nullptr) {
    throw std::runtime_error("Required Debug Messenger Extension not present");
  }

  if (
    function(m_instance, &debugCreateInfo, nullptr, &m_debugMessenger)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create debug messenger");
  }
}

}
