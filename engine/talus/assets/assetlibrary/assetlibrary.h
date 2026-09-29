#pragma once

#include <talus/core/asset/assetdatabase.h>
#include <talus/serialize/materialfile/materialfile.h>
#include <talus/gpu/context/context.h>
#include <talus/gpu/sampler/sampler.h>
#include <talus/gpu/texture/texture.h>
#include <talus/objectmodel/world/world.h>
#include <talus/renderer/frame/frame.h>
#include <talus/renderer/materialrecord/materialrecord.h>
#include <talus/renderer/meshbuffers/meshbuffers.h>
#include <talus/renderer/meshrecord/meshrecord.h>

#include <filesystem>
#include <vector>

namespace talus::assets {

class AssetLibrary {
public:
  AssetLibrary(
    const talus::gpu::Context& context,
    const std::filesystem::path& projectRoot
  );

  AssetLibrary(const AssetLibrary&) = delete;
  AssetLibrary& operator=(const AssetLibrary&) = delete;
  AssetLibrary(AssetLibrary&&) = delete;
  AssetLibrary& operator=(AssetLibrary&&) = delete;

  [[nodiscard]] const std::vector<talus::renderer::MeshBuffers>&
  meshBuffers() const noexcept;

  [[nodiscard]] const std::vector<talus::renderer::MaterialRecord>&
  materials() const noexcept;

  [[nodiscard]] const std::vector<talus::objectmodel::PhysicalMaterial>&
  physicalMaterials() const noexcept;

  [[nodiscard]] const std::vector<talus::gpu::Texture>&
  textures() const noexcept;

  [[nodiscard]] const std::vector<talus::gpu::Sampler>&
  samplers() const noexcept;

  [[nodiscard]] const talus::core::AssetDatabase& assets() const noexcept;

  [[nodiscard]] uint32_t materialIndex(talus::core::AssetId id) const;

  uint32_t addMaterial(
    talus::core::AssetId id,
    const talus::serialize::StoredMaterial& stored
  );

  void setPhysicalMaterial(
    uint32_t index,
    const talus::objectmodel::PhysicalMaterial& physical
  );

private:
  void loadProject(
    const talus::gpu::Device& device,
    const std::filesystem::path& projectRoot
  );

  uint32_t loadMesh(
    const talus::gpu::Device& device,
    talus::core::AssetId id,
    const char* path
  );

  uint32_t loadTexture(
    const talus::gpu::Device& device,
    talus::core::AssetId id,
    const char* path
  );

  static std::vector<talus::gpu::Texture> createTextures(
    const talus::gpu::Device& device
  );

  static std::vector<talus::gpu::Sampler> createSamplers(
    const talus::gpu::Device& device
  );

  talus::core::AssetDatabase m_assets;

  std::vector<talus::renderer::MaterialRecord> m_materials;
  std::vector<talus::objectmodel::PhysicalMaterial> m_physical;
  std::vector<talus::gpu::Sampler> m_samplers;
  std::vector<talus::gpu::Texture> m_textures;
  std::vector<talus::renderer::MeshBuffers> m_meshes;
};

}
