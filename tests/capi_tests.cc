#include <catch2/catch_test_macros.hpp>

#include <capi/talus_c.h>
#include <talus/serialize/editorstate/editorstate.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

namespace {

std::string slurp(const std::filesystem::path& path) {
  std::ifstream file(path);

  INFO("path " << path.string());
  REQUIRE(file.is_open());

  std::ostringstream text;
  text << file.rdbuf();

  return text.str();
}

std::set<std::string> namesMatching(
  const std::string& text,
  const std::regex& pattern
) {
  std::set<std::string> names;

  for (auto at = std::sregex_iterator(text.begin(), text.end(), pattern);
       at != std::sregex_iterator();
       ++at) {
    names.insert((*at)[1].str());
  }

  return names;
}

std::set<std::string> declared() {
  const std::string header =
    slurp(std::filesystem::path(TALUS_CAPI_DIR) / "talus_c.h");

  return namesMatching(
    header,
    std::regex(R"(TALUS_C_API[^;{]*?\b(talus_[a-z0-9_]+)\s*\()")
  );
}

std::set<std::string> defined() {
  const std::string source =
    slurp(std::filesystem::path(TALUS_CAPI_DIR) / "talus_c.cc");

  return namesMatching(
    source,
    std::regex(R"(^[A-Za-z_][^;\n]*?\b(talus_[a-z0-9_]+)\s*\()")
  );
}

std::filesystem::path rootFor(std::string_view leaf) {
  return std::filesystem::temp_directory_path() / "talus_capi_tests" / leaf;
}

std::filesystem::path scratch(std::string_view leaf) {
  const std::filesystem::path root = rootFor(leaf);

  std::error_code failed;
  std::filesystem::remove_all(root, failed);
  std::filesystem::create_directories(root.parent_path(), failed);

  return root;
}

TalusProject* createProject(std::string_view name) {
  const std::filesystem::path root = scratch(name);

  TalusProject* project =
    talus_project_create(root.string().c_str(), std::string(name).c_str());
  REQUIRE(project != nullptr);

  return project;
}

}

TEST_CASE("every exported entry point is declared in the C header", "[capi]") {
  const std::set<std::string> inHeader = declared();
  const std::set<std::string> inSource = defined();

  REQUIRE_FALSE(inHeader.empty());
  REQUIRE_FALSE(inSource.empty());

  for (const std::string& name : inSource) {
    INFO("defined in talus_c.cc but not declared in talus_c.h: " << name);
    REQUIRE(inHeader.count(name) == 1);
  }
}

TEST_CASE("every declared entry point is defined", "[capi]") {
  const std::set<std::string> inHeader = declared();
  const std::set<std::string> inSource = defined();

  for (const std::string& name : inHeader) {
    INFO("declared in talus_c.h but not defined in talus_c.cc: " << name);
    REQUIRE(inSource.count(name) == 1);
  }
}

TEST_CASE("editor state defaults to auto save when unwritten", "[capi]") {
  TalusProject* project = createProject("editor_state_default");

  REQUIRE(!std::filesystem::exists(
    rootFor("editor_state_default") / ".talus-editor.json"
  ));

  int32_t autoSaveBeforeRun = 0;
  REQUIRE(talus_project_editor_state_get(project, &autoSaveBeforeRun) == 0);
  REQUIRE(autoSaveBeforeRun == 1);

  talus_project_close(project);
}

TEST_CASE("editor state set persists off across a reload", "[capi]") {
  const std::filesystem::path root = rootFor("editor_state_off");
  TalusProject* project = createProject("editor_state_off");

  REQUIRE(talus_project_editor_state_set(project, 0) == 0);
  talus_project_close(project);

  TalusProject* reopened = talus_project_open(root.string().c_str());
  REQUIRE(reopened != nullptr);

  int32_t autoSaveBeforeRun = 1;
  REQUIRE(talus_project_editor_state_get(reopened, &autoSaveBeforeRun) == 0);
  REQUIRE(autoSaveBeforeRun == 0);

  talus_project_close(reopened);
}

TEST_CASE("editor state set keeps the last world", "[capi]") {
  const std::filesystem::path root = rootFor("editor_state_lastworld");
  TalusProject* created = createProject("editor_state_lastworld");
  talus_project_close(created);

  talus::serialize::EditorState written{};
  written.lastWorld = "arena";
  REQUIRE(talus::serialize::saveEditorState(root, written));

  TalusProject* project = talus_project_open(root.string().c_str());
  REQUIRE(project != nullptr);

  REQUIRE(talus_project_editor_state_set(project, 0) == 0);
  talus_project_close(project);

  talus::serialize::EditorState read{};
  REQUIRE(talus::serialize::loadEditorState(root, read));
  REQUIRE(read.lastWorld == "arena");
  REQUIRE(!read.autoSaveBeforeRun);
}

TEST_CASE("editor state calls reject a null project", "[capi]") {
  int32_t autoSaveBeforeRun = 0;
  REQUIRE(talus_project_editor_state_get(nullptr, &autoSaveBeforeRun) != 0);
  REQUIRE(talus_project_editor_state_set(nullptr, 1) != 0);
}

TEST_CASE("editor state get rejects a null out pointer", "[capi]") {
  TalusProject* project = createProject("editor_state_null_out");

  REQUIRE(talus_project_editor_state_get(project, nullptr) != 0);

  talus_project_close(project);
}
