#pragma once

#include <talus/gpu/device/device.h>

#include <vulkan/vulkan.h>

#include <utility>

namespace talus::gpu {

struct SamplerSettings {
  VkFilter filter = VK_FILTER_LINEAR;
  VkSamplerAddressMode addressMode = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  bool anisotropy = true;
};

class Sampler {
public:
  Sampler(const Device& device, const SamplerSettings& settings = {});

  Sampler(const Sampler&) = delete;
  Sampler& operator=(const Sampler&) = delete;

  Sampler(Sampler&& other) noexcept;
  Sampler& operator=(Sampler&& other) noexcept;

  ~Sampler();

  VkSampler handle() const noexcept;

private:
  void destroy() noexcept;

  VkSampler m_sampler = VK_NULL_HANDLE;
  VkDevice m_device = VK_NULL_HANDLE;
};

}
