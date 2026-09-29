#pragma once

#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/trait/requires/requires.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>
#include <talus/objectmodel/trait/transient/transient.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <memory>
#include <vector>

namespace talus::objectmodel {

enum class VolumeElementType : uint32_t {
  Hex,
  Tet
};

struct DeformableBody {
  std::optional<VolumeElementType> element{};
};

template<>
inline constexpr bool selfContained<DeformableBody> = true;

template<>
inline constexpr bool authored<DeformableBody> = true;

template<>
struct Requires<DeformableBody> {
  using List = ComponentList<SdfGrid>;
};

struct PreparedTet {
  glm::uvec4 nodes{0u};
  glm::mat3 restInverse{1.0f};
  float volume = 0.0f;
  uint32_t material = 0u;
};

struct PreparedHex {
  std::array<uint32_t, 8> nodes{};
  glm::vec3 halfExtents{0.0f};
  float volume = 0.0f;
  uint32_t material = 0u;
};

struct PreparedMesh {
  std::vector<glm::vec3> restPositions;
  std::vector<float> nodeMass;
  std::vector<uint8_t> anchored;
  std::vector<PreparedTet> tets;
  std::vector<PreparedHex> hexes;
  std::vector<uint32_t> surfaceNodes;
  std::vector<glm::uvec2> surfaceEdges;
  std::vector<glm::uvec3> surfaceTriangles;
  std::vector<float> surfaceThickness;
  std::vector<uint32_t> surfaceLane;
  glm::vec3 voxelSize{1.0f};
};

struct PlasticPoint {
  glm::mat3 strain{0.0f};
  float equivalent = 0.0f;
};

struct CommittedTet {
  glm::mat3 rotation{1.0f};
};

struct DeformablePrepared {
  std::shared_ptr<const PreparedMesh> mesh;
};

struct DeformableState {
  std::vector<glm::vec3> position;
  std::vector<glm::vec3> velocity;
  std::vector<glm::vec3> acceleration;
  std::vector<CommittedTet> committed;
  std::vector<PlasticPoint> plastic;
};

template<>
inline constexpr bool transient<DeformablePrepared> = true;

template<>
inline constexpr bool transient<DeformableState> = true;

}
