#pragma once

#include <vulkan/vulkan.h>
#include <vector>

namespace talus::gpu {

#ifdef NDEBUG
constexpr bool enableValidationLayers = false;
#else
constexpr bool enableValidationLayers = true;
#endif

class Instance {
public:
  Instance(
    const std::vector<char const*>& validationLayers,
    const std::vector<const char*>& windowExtensions
  );
  Instance(Instance const&) = delete;
  Instance& operator=(Instance const&) = delete;
  ~Instance();

  const VkInstance& handle() const noexcept;

private:
  void setup(
    const std::vector<char const*>& validationLayers,
    const std::vector<const char*>& windowExtensions
  );
  void createVkInstance(
    const std::vector<char const*>& validationLayers,
    const std::vector<const char*>& windowExtensions
  );
  void setupDebugMessenger();

  std::vector<VkLayerProperties> enumerateInstanceLayerProperties();

  VkInstance m_instance = VK_NULL_HANDLE;
  VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;

  static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
    void* userData
  );
};

}
