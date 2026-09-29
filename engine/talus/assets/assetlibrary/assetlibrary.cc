#include "assetlibrary.ih"

namespace talus::assets {

AssetLibrary::AssetLibrary(
  const talus::gpu::Context& context,
  const std::filesystem::path& projectRoot
)
    : m_samplers(createSamplers(context.device())),
      m_textures(createTextures(context.device())) {
  loadProject(context.device(), projectRoot);
}

void AssetLibrary::loadProject(
  const talus::gpu::Device& device,
  const std::filesystem::path& projectRoot
) {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(projectRoot);

  if (!project.has_value()) {
    throw std::runtime_error(
      "AssetLibrary: no project at " + projectRoot.string()
    );
  }

  for (const talus::serialize::AssetEntry* entry : project->ofType("texture")) {
    const std::string path = (project->root() / entry->path).string();

    static_cast<void>(loadTexture(device, entry->id, path.c_str()));
  }

  for (const talus::serialize::AssetEntry* entry :
       project->ofType("material")) {
    const std::optional<std::string> text =
      talus::serialize::readText(project->root() / entry->path);

    if (!text.has_value()) {
      throw std::runtime_error("AssetLibrary: cannot read " + entry->path);
    }

    talus::serialize::StoredMaterial stored{};

    if (!talus::serialize::readMaterial(*text, stored)) {
      throw std::runtime_error("AssetLibrary: cannot parse " + entry->path);
    }

    static_cast<void>(addMaterial(entry->id, stored));
  }

  for (const talus::serialize::AssetEntry* entry : project->ofType("mesh")) {
    const std::string path = (project->root() / entry->path).string();

    static_cast<void>(loadMesh(device, entry->id, path.c_str()));
  }
}

uint32_t AssetLibrary::loadTexture(
  const talus::gpu::Device& device,
  talus::core::AssetId id,
  const char* path
) {
  const uint32_t index = static_cast<uint32_t>(m_textures.size());

  m_assets.bind<talus::gpu::Texture>(id, index);

  m_textures.emplace_back(talus::gpu::Texture(device, path));

  return index;
}

uint32_t AssetLibrary::addMaterial(
  talus::core::AssetId id,
  const talus::serialize::StoredMaterial& stored
) {
  const auto slot =
    [this](const talus::serialize::StoredTextureSlot& held, uint32_t fallback) {
      if (held.texture == talus::core::INVALID_ASSET) {
        return fallback;
      }

      const uint32_t index = m_assets.index<talus::gpu::Texture>(held.texture);

      if (index == talus::core::UNBOUND_ASSET) {
        throw std::runtime_error(
          "AssetLibrary: a material names a texture the project has not bound"
        );
      }

      return index;
    };

  talus::renderer::MaterialRecord record{};

  record.baseColor = stored.baseColor;
  record.emissive = stored.emissive;

  record.metallic = stored.metallic;
  record.roughness = stored.roughness;
  record.normalScale = stored.normalScale;
  record.occlusionStrength = stored.occlusionStrength;
  record.alphaCutoff = stored.alphaCutoff;
  record.flags = stored.flags;

  record.baseColorTexture =
    slot(stored.baseColorTexture, talus::core::TEXTURE_WHITE);
  record.baseColorSampler = stored.baseColorTexture.sampler;

  record.metallicRoughnessTexture =
    slot(stored.metallicRoughnessTexture, talus::core::TEXTURE_WHITE);
  record.metallicRoughnessSampler = stored.metallicRoughnessTexture.sampler;

  record.normalTexture =
    slot(stored.normalTexture, talus::core::TEXTURE_FLAT_NORMAL);
  record.normalSampler = stored.normalTexture.sampler;

  record.occlusionTexture =
    slot(stored.occlusionTexture, talus::core::TEXTURE_WHITE);
  record.occlusionSampler = stored.occlusionTexture.sampler;

  record.emissiveTexture =
    slot(stored.emissiveTexture, talus::core::TEXTURE_BLACK);
  record.emissiveSampler = stored.emissiveTexture.sampler;

  const uint32_t index = static_cast<uint32_t>(m_materials.size());

  m_assets.bind<talus::objectmodel::Material>(id, index);

  m_materials.push_back(record);
  m_physical.push_back(talus::serialize::physicalMaterial(stored));

  return index;
}

void AssetLibrary::setPhysicalMaterial(
  uint32_t index,
  const talus::objectmodel::PhysicalMaterial& physical
) {
  if (index >= m_physical.size()) {
    throw std::runtime_error("AssetLibrary: no material at that index");
  }

  m_physical[index] = physical;
}

uint32_t AssetLibrary::loadMesh(
  const talus::gpu::Device& device,
  talus::core::AssetId id,
  const char* path
) {
  const uint32_t index = static_cast<uint32_t>(m_meshes.size());

  m_assets.bind<talus::objectmodel::Mesh>(id, index);

  m_meshes.emplace_back(talus::renderer::MeshBuffers(device, path));

  return index;
}

const std::vector<talus::renderer::MeshBuffers>& AssetLibrary::
  meshBuffers() const noexcept {
  return m_meshes;
}

const std::vector<talus::renderer::MaterialRecord>& AssetLibrary::
  materials() const noexcept {
  return m_materials;
}

const std::vector<talus::objectmodel::PhysicalMaterial>& AssetLibrary::
  physicalMaterials() const noexcept {
  return m_physical;
}

const std::vector<talus::gpu::Texture>& AssetLibrary::
  textures() const noexcept {
  return m_textures;
}

const std::vector<talus::gpu::Sampler>& AssetLibrary::
  samplers() const noexcept {
  return m_samplers;
}

std::vector<talus::gpu::Sampler> AssetLibrary::createSamplers(
  const talus::gpu::Device& device
) {
  std::vector<talus::gpu::Sampler> samplers;
  samplers.reserve(3);

  samplers.emplace_back(device);

  samplers.emplace_back(
    device,
    talus::gpu::SamplerSettings{
      VK_FILTER_LINEAR,
      VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
      false
    }
  );
  samplers.emplace_back(
    device,
    talus::gpu::SamplerSettings{
      VK_FILTER_NEAREST,
      VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
      false
    }
  );

  return samplers;
}

std::vector<talus::gpu::Texture> AssetLibrary::createTextures(
  const talus::gpu::Device& device
) {
  const std::array<uint8_t, 4> white{255, 255, 255, 255};
  const std::array<uint8_t, 4> flatNormal{128, 128, 255, 255};
  const std::array<uint8_t, 4> black{0, 0, 0, 255};

  std::vector<talus::gpu::Texture> textures;
  textures.reserve(talus::core::RESERVED_TEXTURES);

  textures.emplace_back(device, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, white.data());
  textures.emplace_back(
    device,
    1,
    1,
    VK_FORMAT_R8G8B8A8_UNORM,
    flatNormal.data()
  );
  textures.emplace_back(device, 1, 1, VK_FORMAT_R8G8B8A8_UNORM, black.data());

  return textures;
}

uint32_t AssetLibrary::materialIndex(talus::core::AssetId id) const {
  const uint32_t index = m_assets.index<talus::objectmodel::Material>(id);

  if (index == talus::core::UNBOUND_ASSET) {
    throw std::runtime_error(
      "AssetLibrary: no material is bound under that asset id"
    );
  }

  return index;
}

const talus::core::AssetDatabase& AssetLibrary::assets() const noexcept {
  return m_assets;
}

}
