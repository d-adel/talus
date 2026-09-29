#pragma once

#include <talus/gpu/device/device.h>

#include <vulkan/vulkan.h>

#include <cstdint>

namespace talus::gpu {

class QueryPool {
public:
  QueryPool() = default;
  QueryPool(const Device& device, uint32_t queries);

  QueryPool(const QueryPool&) = delete;
  QueryPool& operator=(const QueryPool&) = delete;

  QueryPool(QueryPool&& other) noexcept;
  QueryPool& operator=(QueryPool&& other) noexcept;

  ~QueryPool();

  [[nodiscard]] VkQueryPool handle() const noexcept;
  [[nodiscard]] uint32_t queries() const noexcept;
  [[nodiscard]] double period() const noexcept;
  [[nodiscard]] bool usable() const noexcept;

private:
  void destroy() noexcept;

  VkQueryPool m_pool = VK_NULL_HANDLE;
  VkDevice m_device = VK_NULL_HANDLE;
  uint32_t m_queries = 0u;
  double m_period = 0.0;
  bool m_usable = false;
};

}
