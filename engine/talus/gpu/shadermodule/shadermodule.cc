#include "shadermodule.ih"

namespace talus::gpu {

namespace {

std::vector<char> readSpirv(const std::string& path) {
  std::ifstream file(path, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file");
  }

  const size_t size = static_cast<size_t>(file.tellg());
  std::vector<char> buffer(size);

  file.seekg(0);
  file.read(buffer.data(), size);

  return buffer;
}

}

ShaderModule::ShaderModule(const VkDevice& device, const std::string& path)
    : ShaderModule(device, readSpirv(path)) {}

ShaderModule::ShaderModule(
  const VkDevice& device,
  const std::vector<char>& code
)
    : m_device(device) {
  VkShaderModuleCreateInfo createInfo{};
  createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  createInfo.codeSize = code.size();
  createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

  if (
    vkCreateShaderModule(m_device, &createInfo, nullptr, &m_module)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create shader module");
  }
}

ShaderModule::ShaderModule(ShaderModule&& other) noexcept
    : m_module(std::exchange(other.m_module, VK_NULL_HANDLE)),
      m_device(std::exchange(other.m_device, VK_NULL_HANDLE)) {}

ShaderModule& ShaderModule::operator=(ShaderModule&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  destroy();

  m_module = std::exchange(other.m_module, VK_NULL_HANDLE);

  m_device = std::exchange(other.m_device, VK_NULL_HANDLE);

  return *this;
}

ShaderModule::~ShaderModule() {
  destroy();
}

VkShaderModule ShaderModule::handle() const noexcept {
  return m_module;
}

void ShaderModule::destroy() noexcept {
  if (m_device == VK_NULL_HANDLE) {
    return;
  }

  vkDestroyShaderModule(m_device, m_module, nullptr);

  m_module = VK_NULL_HANDLE;
}

}
