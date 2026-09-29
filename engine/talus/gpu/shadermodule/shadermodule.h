#pragma once

#include <vulkan/vulkan.h>

#include <string>
#include <utility>
#include <vector>

namespace talus::gpu {

class ShaderModule {
public:
  ShaderModule(const VkDevice& device, const std::vector<char>& code);

  ShaderModule(const VkDevice& device, const std::string& path);

  ShaderModule(const ShaderModule&) = delete;
  ShaderModule& operator=(const ShaderModule&) = delete;

  ShaderModule(ShaderModule&& other) noexcept;
  ShaderModule& operator=(ShaderModule&& other) noexcept;

  ~ShaderModule();

  VkShaderModule handle() const noexcept;

private:
  void destroy() noexcept;

  VkShaderModule m_module = VK_NULL_HANDLE;
  VkDevice m_device = VK_NULL_HANDLE;
};

}
