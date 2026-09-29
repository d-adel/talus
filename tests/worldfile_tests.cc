#include <catch2/catch_test_macros.hpp>

#include <talus/serialize/worldfile/worldfile.h>

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/component/bakedvolume/bakedvolume.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/deformed/deformed.h>
#include <talus/objectmodel/component/rigidbody/rigidbody.h>
#include <talus/objectmodel/component/support/support.h>
#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/name/name.h>

#include <array>
#include <cmath>
#include <cstring>
#include <vector>

using talus::core::AssetDatabase;
using talus::objectmodel::World;
using talus::serialize::captureWorld;
using talus::serialize::readWorld;
using talus::serialize::restoreWorld;
using talus::serialize::StoredWorld;
using talus::serialize::WORLD_VERSION;
using talus::serialize::writeWorld;

namespace {

constexpr talus::core::AssetId STONE = 0xA1B2C3D4E5F60718ULL;
constexpr talus::core::AssetId TIMBER = 0x1807F6E5D4C3B2A1ULL;

AssetDatabase inOrder() {
  AssetDatabase assets;
  assets.bind<talus::objectmodel::Material>(STONE, 0u);
  assets.bind<talus::objectmodel::Material>(TIMBER, 1u);

  return assets;
}

AssetDatabase reordered() {
  AssetDatabase assets;
  assets.bind<talus::objectmodel::Material>(STONE, 1u);
  assets.bind<talus::objectmodel::Material>(TIMBER, 0u);

  return assets;
}

talus::objectmodel::Entity authorBox(World& world, uint32_t material) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(65u);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(1.0f, 2.0f, 3.0f);

  const talus::objectmodel::Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    talus::field::makeBox(
      glm::vec3(0.0f),
      glm::vec3(0.45f),
      0.0f,
      glm::vec3(0.0f, 1.0f, 0.0f),
      material
    )
  ));

  return entity;
}

}

TEST_CASE("entities keep the order they were authored in", "[worldfile]") {
  World authored;

  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(65u);

  std::vector<float> heights;

  for (uint32_t index = 0u; index != 5u; ++index) {
    talus::objectmodel::Pose pose{};
    pose.position = glm::vec3(0.0f, float(index), 0.0f);

    const talus::objectmodel::Entity entity =
      authored.createSdfGrid(pose, grid);

    REQUIRE(authored.addPrimitive(
      entity,
      talus::field::makeBox(glm::vec3(0.0f), glm::vec3(0.45f))
    ));

    heights.push_back(float(index));
  }

  const StoredWorld stored = captureWorld(authored, inOrder());

  REQUIRE(stored.entities.size() == heights.size());

  for (size_t at = 0u; at != heights.size(); ++at) {
    REQUIRE(stored.entities[at].pose.has_value());
    REQUIRE(stored.entities[at].pose->position.y == heights[at]);
  }

  World reloaded;
  REQUIRE(restoreWorld(stored, reloaded, inOrder()));

  const StoredWorld again = captureWorld(reloaded, inOrder());

  REQUIRE(again.entities.size() == stored.entities.size());

  for (size_t at = 0u; at != again.entities.size(); ++at) {
    REQUIRE(again.entities[at].pose->position.y == heights[at]);
  }
}

TEST_CASE("a world survives a round trip through text", "[worldfile]") {
  World authored;

  const talus::objectmodel::Entity box = authorBox(authored, 0u);
  authored.addStaticBody(box);
  authored.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    box,
    talus::objectmodel::DeformableBody{}
  );
  authored.emplaceOrReplace<talus::objectmodel::MassScale>(
    box,
    talus::objectmodel::MassScale{0.25f}
  );
  authored.emplaceOrReplace<talus::objectmodel::Name>(
    box,
    talus::objectmodel::Name{talus::objectmodel::Text::of("crate")}
  );
  REQUIRE(authored.collect("Warehouse", box));
  REQUIRE(authored.createCollection("Empty"));

  const AssetDatabase assets = inOrder();

  const std::string text = writeWorld(captureWorld(authored, assets));

  REQUIRE(!text.empty());

  StoredWorld stored{};
  REQUIRE(readWorld(text, stored));

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  REQUIRE(loaded.sdfGrids().size() == 1);

  const talus::objectmodel::Entity back = loaded.sdfGrids()[0];
  const entt::registry& registry = loaded.registry();

  REQUIRE(registry.get<talus::objectmodel::Pose>(back).position.y == 2.0f);
  REQUIRE(registry.all_of<talus::objectmodel::StaticBody>(back));
  REQUIRE(registry.all_of<talus::objectmodel::DeformableBody>(back));
  REQUIRE(registry.get<talus::objectmodel::MassScale>(back).factor == 0.25f);
  REQUIRE(registry.get<talus::objectmodel::Name>(back).text.view() == "crate");
  REQUIRE(loaded.collections().size() == 2u);
  REQUIRE(loaded.collection("Warehouse")->members.size() == 1u);
  REQUIRE(loaded.collection("Warehouse")->members[0] == back);
  REQUIRE(loaded.collection("Empty")->members.empty());
  REQUIRE(loaded.primitiveCount(back) == 1u);
}

TEST_CASE(
  "a RigidBody and a Support reach the file under their own keys",
  "[worldfile]"
) {
  World authored;

  const talus::objectmodel::Entity ball = authorBox(authored, 0u);
  authored.emplaceAuthored<talus::objectmodel::RigidBody>(
    ball,
    talus::objectmodel::RigidBody{}
  );

  const talus::objectmodel::Entity anchor = authorBox(authored, 0u);
  authored.emplaceOrReplace<talus::objectmodel::Support>(
    anchor,
    talus::objectmodel::Support{glm::vec3(1.0f, 2.0f, 3.0f)}
  );

  const AssetDatabase assets = inOrder();

  const std::string text = writeWorld(captureWorld(authored, assets));

  REQUIRE(text.find("\"rigidBody\"") != std::string::npos);
  REQUIRE(text.find("\"support\"") != std::string::npos);

  StoredWorld stored{};
  REQUIRE(readWorld(text, stored));

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  REQUIRE(loaded.sdfGrids().size() == 2u);

  const entt::registry& registry = loaded.registry();

  bool sawRigidBody = false;
  bool sawSupport = false;

  for (const talus::objectmodel::Entity entity : loaded.sdfGrids()) {
    if (registry.all_of<talus::objectmodel::RigidBody>(entity)) {
      sawRigidBody = true;
    }

    if (const auto* support =
          registry.try_get<talus::objectmodel::Support>(entity)) {
      sawSupport = true;
      REQUIRE(support->halfExtents == glm::vec3(1.0f, 2.0f, 3.0f));
    }
  }

  REQUIRE(sawRigidBody);
  REQUIRE(sawSupport);
}

TEST_CASE(
  "a material follows its id when the palette reorders",
  "[worldfile]"
) {
  World authored;

  static_cast<void>(authorBox(authored, 0u));

  const AssetDatabase writing = inOrder();
  const AssetDatabase readingBack = reordered();

  const StoredWorld stored = captureWorld(authored, writing);

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, readingBack));

  const talus::objectmodel::Entity back = loaded.sdfGrids()[0];

  REQUIRE(loaded.primitives(back)[0].shapeConfig[1] == 1u);
}

TEST_CASE("an unknown material refuses to load", "[worldfile]") {
  World authored;

  static_cast<void>(authorBox(authored, 0u));

  const AssetDatabase writing = inOrder();

  const StoredWorld stored = captureWorld(authored, writing);

  AssetDatabase empty;
  empty.bind<talus::objectmodel::Material>(TIMBER, 0u);

  World loaded;
  REQUIRE(!restoreWorld(stored, loaded, empty));
}

TEST_CASE("a file from another version refuses to load", "[worldfile]") {
  StoredWorld stored{};
  stored.version = talus::serialize::WORLD_VERSION + 1u;

  World loaded;
  const AssetDatabase none;

  REQUIRE(!restoreWorld(stored, loaded, none));
}

TEST_CASE("the grid is refitted on load, not read", "[worldfile]") {
  World authored;

  static_cast<void>(authorBox(authored, 0u));

  const AssetDatabase assets = inOrder();

  const std::string text = writeWorld(captureWorld(authored, assets));

  REQUIRE(text.find("voxelSize") == std::string::npos);
  REQUIRE(text.find("origin") == std::string::npos);

  StoredWorld stored{};
  REQUIRE(readWorld(text, stored));

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  const talus::objectmodel::Entity back = loaded.sdfGrids()[0];

  const talus::objectmodel::SdfGrid& grid =
    loaded.registry().get<talus::objectmodel::SdfGrid>(back);

  REQUIRE(grid.resolution == glm::uvec3(65u));
  REQUIRE(grid.voxelSize.x > 0.0f);
}

TEST_CASE("a mesh and a field share one entity list", "[worldfile]") {
  World authored;

  static_cast<void>(authorBox(authored, 0u));

  AssetDatabase assets = inOrder();
  assets.bind<talus::objectmodel::Mesh>(TIMBER, 4u);

  static_cast<void>(authored.createMesh(
    talus::objectmodel::Pose{glm::vec3(5.0f, 6.0f, 7.0f), glm::quat()},
    talus::objectmodel::Mesh{4u},
    talus::objectmodel::Material{1u}
  ));

  const std::string text = writeWorld(captureWorld(authored, assets));

  StoredWorld stored{};
  REQUIRE(readWorld(text, stored));

  REQUIRE(stored.entities.size() == 2);

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  REQUIRE(loaded.sdfGrids().size() == 1);
  REQUIRE(loaded.meshes().size() == 1);

  const talus::objectmodel::Entity mesh = loaded.meshes()[0];

  REQUIRE(loaded.registry().get<talus::objectmodel::Mesh>(mesh).index == 4u);
  REQUIRE(
    loaded.registry().get<talus::objectmodel::Pose>(mesh).position.z == 7.0f
  );
}

TEST_CASE("authored and self contained are separate questions", "[worldfile]") {
  static_assert(talus::objectmodel::Authored<talus::objectmodel::SdfGrid>);
  static_assert(
    !talus::objectmodel::SelfContained<talus::objectmodel::SdfGrid>
  );

  static_assert(talus::objectmodel::Authored<talus::objectmodel::StaticBody>);
  static_assert(
    !talus::objectmodel::SelfContained<talus::objectmodel::StaticBody>
  );

  SUCCEED("checked at compile time");
}

TEST_CASE("derived state cannot reach a world file", "[worldfile]") {
  static_assert(!talus::objectmodel::Authored<talus::objectmodel::BodyId>);
  static_assert(!talus::objectmodel::Authored<talus::objectmodel::BakedVolume>);
  static_assert(!talus::objectmodel::Authored<talus::objectmodel::Deformed>);
  static_assert(
    !talus::objectmodel::Authored<talus::objectmodel::SdfPrimitiveRange>
  );

  SUCCEED("checked at compile time");
}

TEST_CASE("one entity can be both a field and a mesh", "[worldfile]") {
  World authored;

  const talus::objectmodel::Entity both = authorBox(authored, 0u);

  AssetDatabase assets = inOrder();
  assets.bind<talus::objectmodel::Mesh>(TIMBER, 4u);

  authored.emplaceOrReplace<talus::objectmodel::Mesh>(
    both,
    talus::objectmodel::Mesh{4u}
  );
  authored.emplaceOrReplace<talus::objectmodel::Material>(
    both,
    talus::objectmodel::Material{1u}
  );

  const StoredWorld stored = captureWorld(authored, assets);

  REQUIRE(stored.entities.size() == 1);

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  REQUIRE(loaded.sdfGrids().size() == 1);
  REQUIRE(loaded.meshes().size() == 1);
  REQUIRE(loaded.sdfGrids()[0] == loaded.meshes()[0]);
}

TEST_CASE("a file written by an older build still loads", "[worldfile]") {
  World authored;

  static_cast<void>(authorBox(authored, 0u));

  const AssetDatabase assets = inOrder();

  StoredWorld stored = captureWorld(authored, assets);
  stored.version = WORLD_VERSION - 1u;

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));
  REQUIRE(loaded.sdfGrids().size() == 1);
}

TEST_CASE("a key this build never heard of is skipped", "[worldfile]") {
  const std::string text = R"({
    "version": 1,
    "entities": [{
      "pose": { "position": [1, 2, 3], "rotation": [1, 0, 0, 0] },
      "grid": { "resolution": [65, 65, 65] },
      "spookiness": { "level": 11 },
      "primitives": []
    }]
  })";

  StoredWorld stored{};
  REQUIRE(readWorld(text, stored));

  World loaded;
  const AssetDatabase assets = inOrder();

  REQUIRE(restoreWorld(stored, loaded, assets));
  REQUIRE(loaded.sdfGrids().size() == 1);
}

TEST_CASE("a camera is a pose and a lens", "[worldfile]") {
  World authored;

  const talus::objectmodel::Entity eye = authored.createAuthored();

  authored.emplaceAuthored<talus::objectmodel::Pose>(
    eye,
    talus::objectmodel::Pose{glm::vec3(0.0f, 3.0f, 12.0f), glm::quat()}
  );

  authored.emplaceAuthored<talus::objectmodel::Lens>(
    eye,
    talus::objectmodel::Lens{55.0f, 0.05f, 500.0f}
  );

  const AssetDatabase assets = inOrder();

  const std::string text = writeWorld(captureWorld(authored, assets));

  StoredWorld stored{};
  REQUIRE(readWorld(text, stored));
  REQUIRE(stored.entities.size() == 1);

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  const auto view =
    loaded.registry()
      .view<const talus::objectmodel::Lens, const talus::objectmodel::Pose>();

  REQUIRE(view.size_hint() == 1);

  for (const talus::objectmodel::Entity entity : view) {
    const auto& lens = view.get<const talus::objectmodel::Lens>(entity);
    const auto& pose = view.get<const talus::objectmodel::Pose>(entity);

    REQUIRE(lens.verticalFov == 55.0f);
    REQUIRE(lens.nearPlane == 0.05f);
    REQUIRE(lens.farPlane == 500.0f);
    REQUIRE(pose.position.z == 12.0f);
  }
}

TEST_CASE(
  "a runtime-declared component survives a save and load",
  "[worldfile]"
) {
  talus::objectmodel::World authored;

  const talus::objectmodel::ComponentType type = authored.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Wager",
      20u,
      {talus::objectmodel::FieldSpec{
         "count",
         talus::objectmodel::FieldKind::UInt,
         0u
       },
       talus::objectmodel::FieldSpec{
         "at",
         talus::objectmodel::FieldKind::Vec3,
         4u
       },
       talus::objectmodel::FieldSpec{
         "ready",
         talus::objectmodel::FieldKind::Bool,
         16u
       }}
    }
  );

  REQUIRE(type != talus::objectmodel::INVALID_COMPONENT_TYPE);

  const talus::objectmodel::Entity entity = authored.createSdfGrid({}, {});

  std::array<std::byte, 20> value{};

  const uint32_t count = 7u;
  const std::array<float, 3> at{1.5f, -2.5f, 3.25f};
  const uint8_t ready = 1u;

  std::memcpy(value.data(), &count, 4u);
  std::memcpy(value.data() + 4, at.data(), 12u);
  std::memcpy(value.data() + 16, &ready, 1u);

  REQUIRE(authored.dynamic().emplace(type, entity, value));

  talus::core::AssetDatabase assets;

  const talus::serialize::StoredWorld stored =
    talus::serialize::captureWorld(authored, assets);

  REQUIRE(stored.componentTypes.size() == 1u);
  REQUIRE(stored.componentTypes[0].name == "Wager");
  REQUIRE(stored.componentTypes[0].fields.size() == 3u);
  REQUIRE(stored.entities.size() == 1u);
  REQUIRE(stored.entities[0].dynamic.size() == 1u);
  REQUIRE(stored.entities[0].dynamic[0].values.size() == 5u);

  const std::string text = talus::serialize::writeWorld(stored);

  talus::serialize::StoredWorld read;

  REQUIRE(talus::serialize::readWorld(text, read));

  talus::objectmodel::World restored;

  REQUIRE(talus::serialize::restoreWorld(read, restored, assets));

  const talus::objectmodel::ComponentType back =
    restored.dynamic().find("Wager");

  REQUIRE(back != talus::objectmodel::INVALID_COMPONENT_TYPE);
  REQUIRE(restored.dynamic().count(back) == 1u);

  const std::byte* stored_at = restored.dynamic().get(back, entity);

  REQUIRE(stored_at != nullptr);

  uint32_t readCount = 0u;
  std::array<float, 3> readAt{};
  uint8_t readReady = 0u;

  std::memcpy(&readCount, stored_at, 4u);
  std::memcpy(readAt.data(), stored_at + 4, 12u);
  std::memcpy(&readReady, stored_at + 16, 1u);

  REQUIRE(readCount == 7u);
  REQUIRE(readAt[1] == -2.5f);
  REQUIRE(readAt[2] == 3.25f);
  REQUIRE(readReady == 1u);
}

TEST_CASE(
  "a float reaches the file and comes back bit for bit",
  "[worldfile]"
) {
  World authored;

  const talus::objectmodel::Entity box = authorBox(authored, 0u);

  const glm::vec3 awkward(
    1.0f / 3.0f,
    2.40485382f,
    std::nextafter(6.24189568f, 100.0f)
  );

  const glm::quat turn(0.98480773f, -0.173648179f, 1.0f / 7.0f, 1.0e-8f);

  authored.patch<talus::objectmodel::Pose>(box, [&](auto& pose) {
    pose.position = awkward;
    pose.rotation = turn;
  });

  authored.emplaceOrReplace<talus::objectmodel::Lens>(
    box,
    talus::objectmodel::Lens{69.999992f, 0.10000001f, 9999.9990f}
  );

  const AssetDatabase assets = inOrder();

  StoredWorld stored{};
  REQUIRE(readWorld(writeWorld(captureWorld(authored, assets)), stored));

  World loaded;
  REQUIRE(restoreWorld(stored, loaded, assets));

  REQUIRE(loaded.sdfGrids().size() == 1);

  const entt::registry& registry = loaded.registry();
  const talus::objectmodel::Entity back = loaded.sdfGrids()[0];

  const talus::objectmodel::Pose& pose =
    registry.get<talus::objectmodel::Pose>(back);

  const auto same = [](float wrote, float read) {
    return std::memcmp(&wrote, &read, sizeof(float)) == 0;
  };

  REQUIRE(same(awkward.x, pose.position.x));
  REQUIRE(same(awkward.y, pose.position.y));
  REQUIRE(same(awkward.z, pose.position.z));

  REQUIRE(same(turn.w, pose.rotation.w));
  REQUIRE(same(turn.x, pose.rotation.x));
  REQUIRE(same(turn.y, pose.rotation.y));
  REQUIRE(same(turn.z, pose.rotation.z));

  const talus::objectmodel::Lens& lens =
    registry.get<talus::objectmodel::Lens>(back);

  REQUIRE(same(69.999992f, lens.verticalFov));
  REQUIRE(same(0.10000001f, lens.nearPlane));
  REQUIRE(same(9999.9990f, lens.farPlane));
}

TEST_CASE(
  "a spelling with spare digits reads as the float it names",
  "[worldfile]"
) {
  const std::string longer =
    R"({"version":1,"entities":[{"pose":{"position":[0,2.40485382,6.24189568],)"
    R"("rotation":[0.98480773,-0.173648179,0,0]},"primitives":[]}]})";

  const std::string shorter =
    R"({"version":1,"entities":[{"pose":{"position":[0,2.4048538,6.2418957],)"
    R"("rotation":[0.9848077,-0.17364818,0,0]},"primitives":[]}]})";

  StoredWorld first{};
  StoredWorld second{};

  REQUIRE(readWorld(longer, first));
  REQUIRE(readWorld(shorter, second));

  const AssetDatabase assets = inOrder();

  World a;
  World b;

  REQUIRE(restoreWorld(first, a, assets));
  REQUIRE(restoreWorld(second, b, assets));

  const auto poseOf = [](const World& world) {
    const entt::registry& registry = world.registry();

    for (const talus::objectmodel::Entity entity :
         registry.view<const talus::objectmodel::Pose>()) {
      return registry.get<const talus::objectmodel::Pose>(entity);
    }

    return talus::objectmodel::Pose{};
  };

  const talus::objectmodel::Pose left = poseOf(a);
  const talus::objectmodel::Pose right = poseOf(b);

  REQUIRE(std::memcmp(&left, &right, sizeof(talus::objectmodel::Pose)) == 0);
}

TEST_CASE(
  "an unsigned vector field reaches the file and comes back unsigned",
  "[worldfile]"
) {
  talus::objectmodel::World authored;
  const talus::objectmodel::ComponentType type = authored.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Cells",
      12u,
      {talus::objectmodel::FieldSpec{
        "extent",
        talus::objectmodel::FieldKind::UVec3,
        0u
      }}
    }
  );
  REQUIRE(type != talus::objectmodel::INVALID_COMPONENT_TYPE);
  const talus::objectmodel::Entity entity = authored.createSdfGrid({}, {});
  const std::array<uint32_t, 3> extent{3u, 5u, UINT32_MAX};
  std::array<std::byte, 12> value{};
  std::memcpy(value.data(), extent.data(), 12u);
  REQUIRE(authored.dynamic().emplace(type, entity, value));
  talus::core::AssetDatabase assets;
  const std::string text = talus::serialize::writeWorld(
    talus::serialize::captureWorld(authored, assets)
  );
  talus::serialize::StoredWorld read;
  REQUIRE(talus::serialize::readWorld(text, read));
  talus::objectmodel::World restored;
  REQUIRE(talus::serialize::restoreWorld(read, restored, assets));
  const talus::objectmodel::ComponentType back =
    restored.dynamic().find("Cells");
  REQUIRE(back != talus::objectmodel::INVALID_COMPONENT_TYPE);
  const std::byte* at = restored.dynamic().get(back, entity);
  REQUIRE(at != nullptr);
  std::array<uint32_t, 3> readBack{};
  std::memcpy(readBack.data(), at, 12u);
  REQUIRE(readBack == extent);
}
