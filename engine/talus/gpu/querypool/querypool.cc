#include "querypool.ih"

namespace talus::gpu {

QueryPool::QueryPool(const Device& device, uint32_t queries)
    : m_device(device.vkDevice()), m_queries(queries) {
  VkPhysicalDeviceProperties held{};

  vkGetPhysicalDeviceProperties(device.physicalDevice(), &held);

  m_period = double(held.limits.timestampPeriod);

  uint32_t families = 0u;

  vkGetPhysicalDeviceQueueFamilyProperties(
    device.physicalDevice(),
    &families,
    nullptr
  );

  std::vector<VkQueueFamilyProperties> family(families);

  vkGetPhysicalDeviceQueueFamilyProperties(
    device.physicalDevice(),
    &families,
    family.data()
  );

  const uint32_t graphics = device.graphicsFamilyIndex();

  m_usable = m_period > 0.0 && graphics < families
             && family[graphics].timestampValidBits != 0u;

  if (!m_usable) {
    return;
  }

  VkQueryPoolCreateInfo info{};

  info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
  info.queryType = VK_QUERY_TYPE_TIMESTAMP;
  info.queryCount = queries;

  if (vkCreateQueryPool(m_device, &info, nullptr, &m_pool) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create the timestamp query pool");
  }
}

QueryPool::QueryPool(QueryPool&& other) noexcept
    : m_pool(std::exchange(other.m_pool, VK_NULL_HANDLE)),
      m_device(std::exchange(other.m_device, VK_NULL_HANDLE)),
      m_queries(std::exchange(other.m_queries, 0u)),
      m_period(std::exchange(other.m_period, 0.0)),
      m_usable(std::exchange(other.m_usable, false)) {}

QueryPool& QueryPool::operator=(QueryPool&& other) noexcept {
  if (this != &other) {
    destroy();

    m_pool = std::exchange(other.m_pool, VK_NULL_HANDLE);
    m_device = std::exchange(other.m_device, VK_NULL_HANDLE);
    m_queries = std::exchange(other.m_queries, 0u);
    m_period = std::exchange(other.m_period, 0.0);
    m_usable = std::exchange(other.m_usable, false);
  }

  return *this;
}

QueryPool::~QueryPool() {
  destroy();
}

void QueryPool::destroy() noexcept {
  if (m_pool != VK_NULL_HANDLE && m_device != VK_NULL_HANDLE) {
    vkDestroyQueryPool(m_device, m_pool, nullptr);
  }

  m_pool = VK_NULL_HANDLE;
  m_device = VK_NULL_HANDLE;
  m_queries = 0u;
  m_usable = false;
}

VkQueryPool QueryPool::handle() const noexcept {
  return m_pool;
}

uint32_t QueryPool::queries() const noexcept {
  return m_queries;
}

double QueryPool::period() const noexcept {
  return m_period;
}

bool QueryPool::usable() const noexcept {
  return m_usable && m_pool != VK_NULL_HANDLE;
}

}
