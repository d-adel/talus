#include <catch2/catch_test_macros.hpp>

#include <talus/serialize/project/project.h>
#include <talus/serialize/editorstate/editorstate.h>

#include <filesystem>
#include <fstream>

using talus::core::AssetDatabase;
using talus::objectmodel::World;
using talus::serialize::captureWorld;
using talus::serialize::mintAssetId;
using talus::serialize::Project;
using talus::serialize::restoreWorld;
using talus::serialize::StoredWorld;

namespace {

std::filesystem::path scratch(std::string_view leaf) {
  const std::filesystem::path root =
    std::filesystem::temp_directory_path() / "talus_project_tests" / leaf;

  std::error_code failed;
  std::filesystem::remove_all(root, failed);
  std::filesystem::create_directories(root.parent_path(), failed);

  return root;
}

std::filesystem::path writeSource(
  const std::filesystem::path& root,
  std::string_view name,
  std::string_view body
) {
  std::error_code failed;
  std::filesystem::create_directories(root, failed);

  const std::filesystem::path file = root / name;

  std::ofstream out(file, std::ios::binary | std::ios::trunc);
  out << body;

  return file;
}

talus::objectmodel::Entity authorBox(World& world) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(65u);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(4.0f, 5.0f, 6.0f);

  const talus::objectmodel::Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    talus::field::makeBox(glm::vec3(0.0f), glm::vec3(0.45f))
  ));

  return entity;
}

}

TEST_CASE("a fresh project has the folders and reopens", "[project]") {
  const std::filesystem::path root = scratch("fresh");

  const auto made = Project::create(root, "Fresh");
  REQUIRE(made.has_value());

  REQUIRE(std::filesystem::exists(root / "project.talus"));
  REQUIRE(std::filesystem::is_directory(root / "assets"));
  REQUIRE(std::filesystem::is_directory(root / "worlds"));

  const auto reopened = Project::open(root);
  REQUIRE(reopened.has_value());
  REQUIRE(reopened->manifest().name == "Fresh");
}

TEST_CASE("opening a folder with no manifest fails", "[project]") {
  const std::filesystem::path root = scratch("empty");

  std::error_code failed;
  std::filesystem::create_directories(root, failed);

  REQUIRE(!Project::open(root).has_value());
}

TEST_CASE("an imported asset is copied and given an id", "[project]") {
  const std::filesystem::path root = scratch("import");

  auto project = Project::create(root, "Import");
  REQUIRE(project.has_value());

  const std::filesystem::path source =
    writeSource(scratch("import_src"), "brick.mat", "roughness 0.8");

  const talus::core::AssetId id = project->importAsset(source, "material");

  REQUIRE(id != talus::core::INVALID_ASSET);
  REQUIRE(std::filesystem::exists(root / "assets" / "brick.mat"));
  REQUIRE(project->find(id) != nullptr);
  REQUIRE(project->find(id)->type == "material");
  REQUIRE(project->ofType("material").size() == 1);
  REQUIRE(project->ofType("mesh").empty());
}

TEST_CASE("an asset can be imported under the id it already has", "[project]") {
  const std::filesystem::path root = scratch("adopt");

  auto project = Project::create(root, "Adopt");
  REQUIRE(project.has_value());

  const std::filesystem::path source =
    writeSource(scratch("adopt_src"), "room.obj", "v 0 0 0");

  constexpr talus::core::AssetId KNOWN = 0x0123456789ABCDEFULL;

  const talus::core::AssetId id = project->importAsset(source, "mesh", KNOWN);

  REQUIRE(id == KNOWN);
  REQUIRE(project->find(KNOWN) != nullptr);
  REQUIRE(project->find(KNOWN)->path == "assets/room.obj");

  REQUIRE(project->save());

  auto reopened = Project::open(root);
  REQUIRE(reopened.has_value());
  REQUIRE(reopened->find(KNOWN) != nullptr);
  REQUIRE(reopened->ofType("mesh").size() == 1);
}

TEST_CASE("importing a file that is not there fails", "[project]") {
  const std::filesystem::path root = scratch("missing");

  auto project = Project::create(root, "Missing");
  REQUIRE(project.has_value());

  REQUIRE(
    project->importAsset(root / "nothing.mat", "material")
    == talus::core::INVALID_ASSET
  );
}

TEST_CASE("a new asset type needs no new code", "[project]") {
  const std::filesystem::path root = scratch("kinds");

  auto project = Project::create(root, "Kinds");
  REQUIRE(project.has_value());

  const std::filesystem::path source =
    writeSource(scratch("kinds_src"), "boom.wav", "riff");

  REQUIRE(project->importAsset(source, "sound") != talus::core::INVALID_ASSET);
  REQUIRE(project->ofType("sound").size() == 1);
}

TEST_CASE("ids survive closing and reopening a project", "[project]") {
  const std::filesystem::path root = scratch("persist");

  talus::core::AssetId id = talus::core::INVALID_ASSET;

  {
    auto project = Project::create(root, "Persist");
    REQUIRE(project.has_value());

    const std::filesystem::path source =
      writeSource(scratch("persist_src"), "stone.mat", "grey");

    id = project->importAsset(source, "material");
    REQUIRE(project->save());
  }

  const auto reopened = Project::open(root);
  REQUIRE(reopened.has_value());
  REQUIRE(reopened->find(id) != nullptr);
  REQUIRE(reopened->find(id)->path == "assets/stone.mat");
}

TEST_CASE("minted ids do not collide", "[project]") {
  std::vector<talus::core::AssetId> minted;

  for (int i = 0; i != 512; ++i) {
    const talus::core::AssetId id = mintAssetId();

    REQUIRE(id != talus::core::INVALID_ASSET);
    REQUIRE(std::find(minted.begin(), minted.end(), id) == minted.end());

    minted.push_back(id);
  }
}

TEST_CASE("a world saves into a project and loads back", "[project]") {
  const std::filesystem::path root = scratch("world");

  auto project = Project::create(root, "World");
  REQUIRE(project.has_value());

  World authored;
  static_cast<void>(authorBox(authored));

  AssetDatabase assets;
  assets.bind<talus::objectmodel::Material>(mintAssetId(), 0u);

  REQUIRE(project->saveWorld("arena", captureWorld(authored, assets)));
  REQUIRE(std::filesystem::exists(root / "worlds" / "arena.world.json"));

  StoredWorld stored{};
  REQUIRE(project->loadWorld("arena", stored));

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));
  REQUIRE(loaded.sdfGrids().size() == 1);
  REQUIRE(
    loaded.registry()
      .get<talus::objectmodel::Pose>(loaded.sdfGrids()[0])
      .position.x
    == 4.0f
  );
}

TEST_CASE("loading a world that is not there fails", "[project]") {
  const std::filesystem::path root = scratch("noworld");

  const auto project = Project::create(root, "NoWorld");
  REQUIRE(project.has_value());

  StoredWorld stored{};
  REQUIRE(!project->loadWorld("absent", stored));
}

TEST_CASE("export carries only what was asked for", "[project]") {
  const std::filesystem::path root = scratch("source");
  const std::filesystem::path away = scratch("shipped");

  auto project = Project::create(root, "Source");
  REQUIRE(project.has_value());

  const std::filesystem::path first =
    writeSource(scratch("src_a"), "keep.mat", "kept");
  const std::filesystem::path second =
    writeSource(scratch("src_b"), "drop.mat", "dropped");

  const talus::core::AssetId kept = project->importAsset(first, "material");
  const talus::core::AssetId dropped = project->importAsset(second, "material");

  REQUIRE(project->save());

  const std::array<talus::core::AssetId, 1> wanted{kept};
  REQUIRE(project->exportTo(away, wanted));

  const auto shipped = Project::open(away);
  REQUIRE(shipped.has_value());

  REQUIRE(shipped->find(kept) != nullptr);
  REQUIRE(shipped->find(dropped) == nullptr);
  REQUIRE(std::filesystem::exists(away / "assets" / "keep.mat"));
  REQUIRE(!std::filesystem::exists(away / "assets" / "drop.mat"));
}

TEST_CASE("exporting an id the project never had fails", "[project]") {
  const std::filesystem::path root = scratch("badexport");
  const std::filesystem::path away = scratch("badexport_out");

  const auto project = Project::create(root, "BadExport");
  REQUIRE(project.has_value());

  const std::array<talus::core::AssetId, 1> wanted{mintAssetId()};

  REQUIRE(!project->exportTo(away, wanted));
}

TEST_CASE("a project remembers the world last opened", "[project]") {
  const std::filesystem::path root = scratch("laststate");

  const auto project = Project::create(root, "LastState");
  REQUIRE(project.has_value());

  talus::serialize::EditorState written{};
  written.lastWorld = "arena";

  REQUIRE(talus::serialize::saveEditorState(root, written));

  talus::serialize::EditorState read{};
  REQUIRE(talus::serialize::loadEditorState(root, read));
  REQUIRE(read.lastWorld == "arena");
}

TEST_CASE("editor state is absent until it is written", "[project]") {
  const std::filesystem::path root = scratch("nostate");

  const auto project = Project::create(root, "NoState");
  REQUIRE(project.has_value());

  talus::serialize::EditorState read{};
  REQUIRE(!talus::serialize::loadEditorState(root, read));
}

TEST_CASE("the recent list puts the newest first", "[project]") {
  talus::serialize::RecentProjects recent{};

  talus::serialize::rememberProject(recent, "/one");
  talus::serialize::rememberProject(recent, "/two");

  REQUIRE(recent.paths.size() == 2);
  REQUIRE(recent.paths[0] == "/two");

  talus::serialize::rememberProject(recent, "/one");

  REQUIRE(recent.paths.size() == 2);
  REQUIRE(recent.paths[0] == "/one");
}

TEST_CASE("the recent list round trips through a file", "[project]") {
  const std::filesystem::path root = scratch("recent");

  std::error_code failed;
  std::filesystem::create_directories(root, failed);

  talus::serialize::RecentProjects written{};
  talus::serialize::rememberProject(written, "/games/first");

  REQUIRE(talus::serialize::saveRecentProjects(root / "recent.json", written));

  talus::serialize::RecentProjects read{};
  REQUIRE(talus::serialize::loadRecentProjects(root / "recent.json", read));
  REQUIRE(read.paths.size() == 1);
  REQUIRE(read.paths[0] == "/games/first");
}

TEST_CASE("a project names its filters as data", "[project]") {
  const std::filesystem::path root = scratch("filtered");

  REQUIRE(Project::create(root, "Filtered").has_value());

  {
    std::ofstream manifest(root / "project.talus", std::ios::trunc);

    manifest
      << R"({"version": 1, "name": "Filtered", "assets": [], "filters": [)"
      << R"({"name": "Doors", "components": ["Door", "Pose"]}, )"
      << R"({"name": "Movable", "components": ["RigidBody"]}]})";
  }

  const auto reopened = Project::open(root);
  REQUIRE(reopened.has_value());
  REQUIRE(reopened->manifest().filters.size() == 2u);
  REQUIRE(reopened->manifest().filters[0].name == "Doors");
  REQUIRE(reopened->manifest().filters[0].components.size() == 2u);
  REQUIRE(reopened->manifest().filters[0].components[1] == "Pose");
  REQUIRE(reopened->manifest().filters[1].components[0] == "RigidBody");
  REQUIRE(reopened->save());

  const auto again = Project::open(root);
  REQUIRE(again.has_value());
  REQUIRE(again->manifest().filters.size() == 2u);
}
