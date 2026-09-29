#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace talus::serialize {

inline constexpr std::string_view GAME_MANIFEST_NAME = "game.json";

struct GameManifest {
  std::string project;
  std::vector<std::string> worlds;
};

[[nodiscard]] std::optional<GameManifest> readGameManifest(
  const std::filesystem::path& beside
);

}
