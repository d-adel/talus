#include "project.ih"

namespace talus::serialize {

namespace {

struct ProjectOptions : glz::opts {
  bool new_lines_in_arrays = false;
  uint8_t indentation_width = 2;
};

std::filesystem::path worldPath(
  const std::filesystem::path& root,
  std::string_view name
) {
  return root / WORLD_FOLDER / (std::string(name) + ".world.json");
}

}

talus::core::AssetId mintAssetId() {
  static std::mt19937_64 source(std::random_device{}());

  std::uniform_int_distribution<uint64_t> spread(
    1ull,
    std::numeric_limits<uint64_t>::max()
  );

  return spread(source);
}

Project::Project(std::filesystem::path root, ProjectManifest manifest)
    : m_root(std::move(root)), m_manifest(std::move(manifest)) {}

std::optional<Project> Project::open(const std::filesystem::path& root) {
  const std::optional<std::string> text = readText(root / MANIFEST_NAME);

  if (!text.has_value()) {
    return std::nullopt;
  }

  ProjectManifest manifest{};

  if (glz::read<glz::opts{.error_on_unknown_keys = false}>(manifest, *text)) {
    return std::nullopt;
  }

  if (manifest.version > PROJECT_VERSION) {
    return std::nullopt;
  }

  return Project(root, std::move(manifest));
}

std::optional<Project> Project::create(
  const std::filesystem::path& root,
  std::string name
) {
  std::error_code failed;

  std::filesystem::create_directories(root / ASSET_FOLDER, failed);

  if (failed) {
    return std::nullopt;
  }

  std::filesystem::create_directories(root / WORLD_FOLDER, failed);

  if (failed) {
    return std::nullopt;
  }

  ProjectManifest manifest{};
  manifest.name = std::move(name);

  Project project(root, std::move(manifest));

  if (!project.save()) {
    return std::nullopt;
  }

  return project;
}

bool Project::save() const {
  std::string text;

  if (glz::write<ProjectOptions{{.prettify = true}}>(m_manifest, text)) {
    return false;
  }

  return writeText(m_root / MANIFEST_NAME, text);
}

const ProjectManifest& Project::manifest() const noexcept {
  return m_manifest;
}

const std::filesystem::path& Project::root() const noexcept {
  return m_root;
}

talus::core::AssetId Project::importAsset(
  const std::filesystem::path& file,
  std::string type,
  talus::core::AssetId id
) {
  std::error_code failed;

  if (!std::filesystem::exists(file, failed) || failed) {
    return talus::core::INVALID_ASSET;
  }

  const std::filesystem::path relative =
    std::filesystem::path(ASSET_FOLDER) / file.filename();

  std::filesystem::create_directories(m_root / ASSET_FOLDER, failed);

  std::filesystem::copy_file(
    file,
    m_root / relative,
    std::filesystem::copy_options::overwrite_existing,
    failed
  );

  if (failed) {
    return talus::core::INVALID_ASSET;
  }

  AssetEntry entry{};

  entry.id = id == talus::core::INVALID_ASSET ? mintAssetId() : id;
  entry.type = std::move(type);
  entry.path = relative.generic_string();

  m_manifest.assets.push_back(std::move(entry));

  return m_manifest.assets.back().id;
}

const AssetEntry* Project::find(talus::core::AssetId id) const noexcept {
  for (const AssetEntry& entry : m_manifest.assets) {
    if (entry.id == id) {
      return &entry;
    }
  }

  return nullptr;
}

std::vector<const AssetEntry*> Project::ofType(std::string_view type) const {
  std::vector<const AssetEntry*> found;

  for (const AssetEntry& entry : m_manifest.assets) {
    if (entry.type == type) {
      found.push_back(&entry);
    }
  }

  return found;
}

bool Project::exportTo(
  const std::filesystem::path& root,
  std::span<const talus::core::AssetId> wanted
) const {
  std::optional<Project> away = Project::create(root, m_manifest.name);

  if (!away.has_value()) {
    return false;
  }

  std::error_code failed;

  for (const talus::core::AssetId id : wanted) {
    const AssetEntry* entry = find(id);

    if (entry == nullptr) {
      return false;
    }

    std::filesystem::create_directories(
      (root / entry->path).parent_path(),
      failed
    );

    std::filesystem::copy_file(
      m_root / entry->path,
      root / entry->path,
      std::filesystem::copy_options::overwrite_existing,
      failed
    );

    if (failed) {
      return false;
    }

    away->m_manifest.assets.push_back(*entry);
  }

  return away->save();
}

bool Project::saveWorld(
  std::string_view name,
  const StoredWorld& stored
) const {
  const std::string text = writeWorld(stored);

  if (text.empty()) {
    return false;
  }

  std::error_code failed;

  std::filesystem::create_directories(m_root / WORLD_FOLDER, failed);

  return writeText(worldPath(m_root, name), text);
}

bool Project::loadWorld(std::string_view name, StoredWorld& stored) const {
  const std::optional<std::string> text = readText(worldPath(m_root, name));

  if (!text.has_value()) {
    return false;
  }

  return readWorld(*text, stored);
}

std::vector<std::string> Project::worldNames() const {
  const std::filesystem::path folder = m_root / WORLD_FOLDER;

  std::vector<std::string> names;

  std::error_code ignored;

  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(folder, ignored)) {
    const std::string file = entry.path().filename().string();

    const size_t suffix = file.rfind(".world.json");

    if (suffix != std::string::npos && suffix + 11u == file.size()) {
      names.push_back(file.substr(0, suffix));
    }
  }

  std::ranges::sort(names);

  return names;
}

}
