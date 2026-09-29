#include "editorstate.ih"

namespace talus::serialize {

namespace {

constexpr size_t MAX_RECENT = 12;

}

bool saveEditorState(
  const std::filesystem::path& projectRoot,
  const EditorState& state
) {
  std::string text;

  if (glz::write<glz::opts{.prettify = true}>(state, text)) {
    return false;
  }

  return writeText(projectRoot / EDITOR_STATE_NAME, text);
}

bool loadEditorState(
  const std::filesystem::path& projectRoot,
  EditorState& state
) {
  const std::optional<std::string> text =
    readText(projectRoot / EDITOR_STATE_NAME);

  if (!text.has_value()) {
    return false;
  }

  return !glz::read<glz::opts{.error_on_unknown_keys = false}>(state, *text);
}

bool saveRecentProjects(
  const std::filesystem::path& file,
  const RecentProjects& recent
) {
  std::string text;

  if (glz::write<glz::opts{.prettify = true}>(recent, text)) {
    return false;
  }

  return writeText(file, text);
}

bool loadRecentProjects(
  const std::filesystem::path& file,
  RecentProjects& recent
) {
  const std::optional<std::string> text = readText(file);

  if (!text.has_value()) {
    return false;
  }

  return !glz::read<glz::opts{.error_on_unknown_keys = false}>(recent, *text);
}

void rememberProject(
  RecentProjects& recent,
  const std::filesystem::path& root
) {
  const std::string entry = root.generic_string();

  std::erase(recent.paths, entry);

  recent.paths.insert(recent.paths.begin(), entry);

  if (recent.paths.size() > MAX_RECENT) {
    recent.paths.resize(MAX_RECENT);
  }
}

}
