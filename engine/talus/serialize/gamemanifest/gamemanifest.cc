#include "gamemanifest.ih"

namespace talus::serialize {

std::optional<GameManifest> readGameManifest(
  const std::filesystem::path& beside
) {
  const std::filesystem::path file = beside / GAME_MANIFEST_NAME;

  std::ifstream stream(file);

  if (!stream) {
    return std::nullopt;
  }

  std::ostringstream text;
  text << stream.rdbuf();

  GameManifest manifest{};

  if (glz::read<glz::opts{.error_on_unknown_keys = false}>(
        manifest,
        text.str()
      )) {
    return std::nullopt;
  }

  if (manifest.project.empty() || manifest.worlds.empty()) {
    return std::nullopt;
  }

  return manifest;
}

}
