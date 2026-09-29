#pragma once

#include <talus/gpu/buffer/buffer.h>
#include <talus/renderer/vertex/vertex.h>

#include <vulkan/vulkan.h>
#include <vector>
#include <string>

namespace talus::renderer {

class MeshBuffers {
public:
  MeshBuffers(const talus::gpu::Device& device, std::string modelPath);

  MeshBuffers(const MeshBuffers&) = delete;
  MeshBuffers& operator=(const MeshBuffers&) = delete;

  MeshBuffers(MeshBuffers&&) noexcept = default;
  MeshBuffers& operator=(MeshBuffers&&) noexcept = default;

  size_t indexCount() const noexcept;
  const talus::gpu::Buffer& vertexBuffer() const noexcept;
  const talus::gpu::Buffer& indexBuffer() const noexcept;

  ~MeshBuffers() = default;

private:
  void loadModel(const talus::gpu::Device& device, std::string modelPath);

  talus::gpu::Buffer m_vertexBuffer;
  talus::gpu::Buffer m_indexBuffer;
  uint32_t m_indexCount = 0;
};

}
