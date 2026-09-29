#pragma once

#include <talus/core/asset/asset.h>
#include <talus/serialize/worldfile/worldfile.h>

#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talus::serialize {

inline constexpr uint32_t PROJECT_VERSION = 1u;

inline constexpr std::string_view MANIFEST_NAME = "project.talus";
inline constexpr std::string_view ASSET_FOLDER = "assets";
inline constexpr std::string_view WORLD_FOLDER = "worlds";

struct AssetEntry {
  talus::core::AssetId id = talus::core::INVALID_ASSET;
  std::string type;
  std::string path;
};

struct FilterEntry {
  std::string name;
  std::vector<std::string> components;
};

struct ProjectManifest {
  uint32_t version = PROJECT_VERSION;
  std::string name;
  std::vector<AssetEntry> assets;
  std::vector<FilterEntry> filters;
};

class Project {
public:
  [[nodiscard]] static std::optional<Project> open(
    const std::filesystem::path& root
  );

  [[nodiscard]] static std::optional<Project> create(
    const std::filesystem::path& root,
    std::string name
  );

  [[nodiscard]] bool save() const;

  [[nodiscard]] const ProjectManifest& manifest() const noexcept;

  [[nodiscard]] const std::filesystem::path& root() const noexcept;

  [[nodiscard]] talus::core::AssetId importAsset(
    const std::filesystem::path& file,
    std::string type,
    talus::core::AssetId id = talus::core::INVALID_ASSET
  );

  [[nodiscard]] const AssetEntry* find(talus::core::AssetId id) const noexcept;

  [[nodiscard]] std::vector<const AssetEntry*> ofType(
    std::string_view type
  ) const;

  [[nodiscard]] bool exportTo(
    const std::filesystem::path& root,
    std::span<const talus::core::AssetId> wanted
  ) const;

  [[nodiscard]] bool saveWorld(
    std::string_view name,
    const StoredWorld& stored
  ) const;

  [[nodiscard]] bool loadWorld(
    std::string_view name,
    StoredWorld& stored
  ) const;

  [[nodiscard]] std::vector<std::string> worldNames() const;

private:
  Project(std::filesystem::path root, ProjectManifest manifest);

  std::filesystem::path m_root;
  ProjectManifest m_manifest;
};

[[nodiscard]] talus::core::AssetId mintAssetId();

}
