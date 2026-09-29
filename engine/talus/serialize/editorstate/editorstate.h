#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace talus::serialize {

inline constexpr uint32_t EDITOR_STATE_VERSION = 1u;

inline constexpr std::string_view EDITOR_STATE_NAME = ".talus-editor.json";

struct EditorState {
  uint32_t version = EDITOR_STATE_VERSION;
  std::string lastWorld;
  bool autoSaveBeforeRun = true;
};

struct RecentProjects {
  uint32_t version = EDITOR_STATE_VERSION;
  std::vector<std::string> paths;
};

[[nodiscard]] bool saveEditorState(
  const std::filesystem::path& projectRoot,
  const EditorState& state
);

[[nodiscard]] bool loadEditorState(
  const std::filesystem::path& projectRoot,
  EditorState& state
);

[[nodiscard]] bool saveRecentProjects(
  const std::filesystem::path& file,
  const RecentProjects& recent
);

[[nodiscard]] bool loadRecentProjects(
  const std::filesystem::path& file,
  RecentProjects& recent
);

void rememberProject(RecentProjects& recent, const std::filesystem::path& root);

}
