#pragma once

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <array>
#include <cstddef>

namespace talus::renderer {

struct Vertex {
  glm::vec3 pos;
  glm::vec3 color;
  glm::vec2 texCoord;
  glm::vec3 normal;

  bool operator==(const Vertex& other) const {
    return pos == other.pos && color == other.color
           && texCoord == other.texCoord && normal == other.normal;
  }

  static VkVertexInputBindingDescription getBindingDescription() {
    VkVertexInputBindingDescription bindingDescription{};
    bindingDescription.binding = 0;
    bindingDescription.stride = sizeof(Vertex);
    bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    return bindingDescription;
  }

  static std::array<VkVertexInputAttributeDescription, 4>
  getAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 4> attributeDescriptions{};

    attributeDescriptions[0].binding = 0;
    attributeDescriptions[0].location = 0;
    attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[0].offset = offsetof(Vertex, pos);

    attributeDescriptions[1].binding = 0;
    attributeDescriptions[1].location = 1;
    attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[1].offset = offsetof(Vertex, color);

    attributeDescriptions[2].binding = 0;
    attributeDescriptions[2].location = 2;
    attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
    attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

    attributeDescriptions[3].binding = 0;
    attributeDescriptions[3].location = 3;
    attributeDescriptions[3].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[3].offset = offsetof(Vertex, normal);

    return attributeDescriptions;
  }
};

static_assert(
  offsetof(Vertex, pos) == 0,
  "Vertex must keep the stride the mesh pipeline binds"
);
static_assert(
  offsetof(Vertex, color) == 16,
  "Vertex must keep the stride the mesh pipeline binds"
);
static_assert(
  offsetof(Vertex, texCoord) == 32,
  "Vertex must keep the stride the mesh pipeline binds"
);
static_assert(
  offsetof(Vertex, normal) == 48,
  "Vertex must keep the stride the mesh pipeline binds"
);
static_assert(
  sizeof(Vertex) == 64,
  "Vertex must keep the stride the mesh pipeline binds"
);

template<typename T>
void hashHelper(std::size_t& seed, const T& value) {
  seed = std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

}

namespace std {
template<>
struct hash<talus::renderer::Vertex> {
  size_t operator()(talus::renderer::Vertex const& vertex) const {
    std::size_t seed = 0;
    talus::renderer::hashHelper(seed, vertex.pos);
    talus::renderer::hashHelper(seed, vertex.color);
    talus::renderer::hashHelper(seed, vertex.texCoord);
    talus::renderer::hashHelper(seed, vertex.normal);

    return seed;
  }
};
}
