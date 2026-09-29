#include <catch2/catch_test_macros.hpp>

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/maincamera/maincamera.h>
#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/mesh/mesh.h>
#include <talus/objectmodel/component/name/name.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/world/world.h>
#include <talus/serialize/componentschema/componentschema.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <set>
#include <string>

namespace {

using talus::objectmodel::DeformableBody;
using talus::objectmodel::Entity;
using talus::objectmodel::FIELD_OPTIONAL;
using talus::objectmodel::FieldKind;
using talus::objectmodel::MassScale;
using talus::objectmodel::Mesh;
using talus::objectmodel::Pose;
using talus::objectmodel::SdfGrid;
using talus::objectmodel::World;
using talus::serialize::COMPONENT_ASSET_BACKED;
using talus::serialize::COMPONENT_WRITABLE;
using talus::serialize::componentBytes;
using talus::serialize::componentSchema;
using talus::serialize::writeField;
using talus::serialize::WriteStatus;

SdfGrid blank() {
  SdfGrid grid{};
  grid.resolution = glm::uvec3(8u);
  return grid;
}

Entity placed(World& world) {
  Pose pose{};
  pose.position = glm::vec3(1.5f, -2.25f, 0.5f);
  return world.createSdfGrid(pose, blank());
}

float floatAt(const std::vector<std::byte>& bytes, size_t offset) {
  float held = 0.0f;
  std::memcpy(&held, bytes.data() + offset, 4u);
  return held;
}

uint32_t uintAt(const std::vector<std::byte>& bytes, size_t offset) {
  uint32_t held = 0u;
  std::memcpy(&held, bytes.data() + offset, 4u);
  return held;
}

std::vector<std::byte> lanes(std::initializer_list<float> values) {
  std::vector<std::byte> bytes(values.size() * 4u);
  size_t at = 0u;
  for (const float value : values) {
    std::memcpy(bytes.data() + at, &value, 4u);
    at += 4u;
  }
  return bytes;
}

const Pose& poseOf(const World& world, Entity entity) {
  return world.registry().get<Pose>(entity);
}

}

TEST_CASE(
  "a pose schema is two packed fields with a unit default rotation",
  "[componentschema]"
) {
  World world;
  const auto schema = componentSchema(world, "Pose");
  REQUIRE(schema);
  REQUIRE(schema->spec.name == "Pose");
  REQUIRE(schema->spec.fields.size() == 2u);
  REQUIRE(schema->spec.fields[0].name == "position");
  REQUIRE(schema->spec.fields[0].kind == FieldKind::Vec3);
  REQUIRE(schema->spec.fields[0].offset == 0u);
  REQUIRE(schema->spec.fields[0].flags == 0u);
  REQUIRE(schema->spec.fields[1].name == "rotation");
  REQUIRE(schema->spec.fields[1].kind == FieldKind::Quat);
  REQUIRE(schema->spec.fields[1].offset == 12u);
  REQUIRE(schema->spec.size == 28u);
  REQUIRE(schema->flags == COMPONENT_WRITABLE);
  REQUIRE(schema->defaults.size() == 28u);
  REQUIRE(floatAt(schema->defaults, 12u) == 0.0f);
  REQUIRE(floatAt(schema->defaults, 24u) == 1.0f);
}

TEST_CASE(
  "a grid schema is its three authored fields and carries presence",
  "[componentschema]"
) {
  World world;
  const auto schema = componentSchema(world, "SdfGrid");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.size() == 3u);
  REQUIRE(schema->spec.fields[0].name == "resolution");
  REQUIRE(schema->spec.fields[0].kind == FieldKind::UVec3);
  REQUIRE(schema->spec.fields[0].offset == 0u);
  REQUIRE(schema->spec.fields[1].name == "margin");
  REQUIRE(schema->spec.fields[1].kind == FieldKind::Float);
  REQUIRE(schema->spec.fields[1].offset == 12u);
  REQUIRE(schema->spec.fields[1].flags == FIELD_OPTIONAL);
  REQUIRE(schema->spec.fields[2].name == "shadowCullMargin");
  REQUIRE(schema->spec.fields[2].offset == 17u);
  REQUIRE(schema->spec.fields[2].flags == FIELD_OPTIONAL);
  REQUIRE(schema->spec.size == 22u);
  REQUIRE(schema->flags == 0u);
  REQUIRE(std::to_integer<uint8_t>(schema->defaults[12]) == 0u);
  REQUIRE(std::to_integer<uint8_t>(schema->defaults[17]) == 0u);
}

TEST_CASE(
  "an empty component has no fields and is not writable",
  "[componentschema]"
) {
  World world;
  const auto schema = componentSchema(world, "StaticBody");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.empty());
  REQUIRE(schema->spec.size == 0u);
  REQUIRE(schema->flags == 0u);
  REQUIRE(schema->defaults.empty());
}

TEST_CASE("a deformable body authors its element type", "[componentschema]") {
  World world;
  const auto schema = componentSchema(world, "DeformableBody");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.size() == 1u);
  REQUIRE(schema->spec.fields[0].name == "element");
  REQUIRE(schema->spec.fields[0].kind == FieldKind::UInt);
  REQUIRE(schema->spec.fields[0].offset == 0u);
  REQUIRE(schema->spec.fields[0].flags == FIELD_OPTIONAL);
  REQUIRE(schema->spec.size == 5u);
  REQUIRE(schema->flags == COMPONENT_WRITABLE);
  REQUIRE(std::to_integer<uint8_t>(schema->defaults[0]) == 0u);
}

TEST_CASE(
  "a rigid body marker is empty, like a static body",
  "[componentschema]"
) {
  World world;
  const auto schema = componentSchema(world, "RigidBody");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.empty());
  REQUIRE(schema->spec.size == 0u);
  REQUIRE(schema->flags == 0u);
  REQUIRE(schema->defaults.empty());
}

TEST_CASE(
  "a support schema is one writable half-extents field",
  "[componentschema]"
) {
  World world;
  const auto schema = componentSchema(world, "Support");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.size() == 1u);
  REQUIRE(schema->spec.fields[0].name == "halfExtents");
  REQUIRE(schema->spec.fields[0].kind == FieldKind::Vec3);
  REQUIRE(schema->spec.fields[0].offset == 0u);
  REQUIRE(schema->spec.size == 12u);
  REQUIRE(schema->flags == COMPONENT_WRITABLE);
  REQUIRE(floatAt(schema->defaults, 0u) == 0.5f);
}

TEST_CASE(
  "an asset-backed component is described but not writable",
  "[componentschema]"
) {
  World world;
  const auto schema = componentSchema(world, "Mesh");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.size() == 1u);
  REQUIRE(schema->spec.fields[0].name == "index");
  REQUIRE(schema->spec.fields[0].kind == FieldKind::UInt);
  REQUIRE(schema->flags == COMPONENT_ASSET_BACKED);
}

TEST_CASE("every authored component derives a schema", "[componentschema]") {
  const auto schemas = talus::serialize::authoredSchemas();
  REQUIRE(schemas.size() == 14u);
  std::set<std::string> names;
  for (const auto& schema : schemas) {
    names.insert(schema.spec.name);
    uint32_t expected = 0u;
    for (const auto& field : schema.spec.fields) {
      REQUIRE(
        talus::objectmodel::isFieldKind(static_cast<uint32_t>(field.kind))
      );
      REQUIRE(field.offset == expected);
      expected += talus::serialize::fieldByteSize(field.kind, field.flags);
    }
    REQUIRE(schema.spec.size == expected);
    REQUIRE(schema.defaults.size() == expected);
  }
  REQUIRE(names.size() == 14u);
  REQUIRE(names.contains("Name"));
  REQUIRE(names.contains("Pose"));
  REQUIRE(names.contains("Environment"));
  REQUIRE(names.contains("RigidBody"));
  REQUIRE(names.contains("Support"));
}

TEST_CASE(
  "a component reads as the bytes its schema describes",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  const auto bytes = componentBytes(world, entity, "Pose");
  REQUIRE(bytes);
  REQUIRE(bytes->size() == 28u);
  REQUIRE(floatAt(*bytes, 0u) == 1.5f);
  REQUIRE(floatAt(*bytes, 4u) == -2.25f);
  REQUIRE(floatAt(*bytes, 8u) == 0.5f);
  REQUIRE(floatAt(*bytes, 12u) == 0.0f);
  REQUIRE(floatAt(*bytes, 24u) == 1.0f);
  const auto grid = componentBytes(world, entity, "SdfGrid");
  REQUIRE(grid);
  REQUIRE(grid->size() == 22u);
  REQUIRE(uintAt(*grid, 0u) == 8u);
  REQUIRE(uintAt(*grid, 8u) == 8u);
}

TEST_CASE(
  "a component the entity lacks or the world never authored is refused",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  REQUIRE_FALSE(componentBytes(world, entity, "MassScale"));
  REQUIRE_FALSE(componentBytes(world, entity, "BakedVolume"));
  REQUIRE_FALSE(componentSchema(world, "BakedVolume"));
  REQUIRE_FALSE(
    componentBytes(world, talus::objectmodel::INVALID_ENTITY, "Pose")
  );
}

TEST_CASE(
  "writing a field changes that field alone and publishes the entity",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  world.clearTouched();
  REQUIRE(
    writeField(
      world,
      entity,
      "Pose",
      0u,
      FieldKind::Vec3,
      lanes({4.0f, 5.0f, 6.0f})
    )
    == WriteStatus::Written
  );
  REQUIRE(poseOf(world, entity).position == glm::vec3(4.0f, 5.0f, 6.0f));
  REQUIRE(poseOf(world, entity).rotation.w == 1.0f);
  REQUIRE(world.touched().size() == 1u);
  REQUIRE(world.touched()[0] == entity);
  REQUIRE(
    writeField(
      world,
      entity,
      "Pose",
      1u,
      FieldKind::Quat,
      lanes({0.0f, 1.0f, 0.0f, 0.0f})
    )
    == WriteStatus::Written
  );
  REQUIRE(poseOf(world, entity).rotation.y == 1.0f);
  REQUIRE(poseOf(world, entity).rotation.w == 0.0f);
  REQUIRE(poseOf(world, entity).position.x == 4.0f);
}

TEST_CASE(
  "a write with the wrong kind, size, field or target leaves the component "
  "alone",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  const Pose before = poseOf(world, entity);
  REQUIRE(
    writeField(
      world,
      entity,
      "Pose",
      0u,
      FieldKind::Vec4,
      lanes({1.0f, 2.0f, 3.0f, 4.0f})
    )
    == WriteStatus::KindMismatch
  );
  REQUIRE(
    writeField(world, entity, "Pose", 0u, FieldKind::Vec3, lanes({1.0f, 2.0f}))
    == WriteStatus::SizeMismatch
  );
  REQUIRE(
    writeField(
      world,
      entity,
      "Pose",
      5u,
      FieldKind::Vec3,
      lanes({1.0f, 2.0f, 3.0f})
    )
    == WriteStatus::NoField
  );
  REQUIRE(
    writeField(
      world,
      entity,
      "Wager",
      0u,
      FieldKind::Vec3,
      lanes({1.0f, 2.0f, 3.0f})
    )
    == WriteStatus::NoComponent
  );
  REQUIRE(
    writeField(world, entity, "MassScale", 0u, FieldKind::Float, lanes({1.0f}))
    == WriteStatus::NoComponent
  );
  REQUIRE(
    writeField(
      world,
      talus::objectmodel::INVALID_ENTITY,
      "Pose",
      0u,
      FieldKind::Vec3,
      lanes({1.0f, 2.0f, 3.0f})
    )
    == WriteStatus::NoEntity
  );
  REQUIRE(poseOf(world, entity).position == before.position);
  REQUIRE(poseOf(world, entity).rotation == before.rotation);
}

TEST_CASE(
  "a write that breaks the component's rules is refused",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  const Pose before = poseOf(world, entity);
  REQUIRE(
    writeField(
      world,
      entity,
      "Pose",
      1u,
      FieldKind::Quat,
      lanes({0.0f, 0.0f, 0.0f, 2.0f})
    )
    == WriteStatus::Invalid
  );
  const float nan = std::numeric_limits<float>::quiet_NaN();
  REQUIRE(
    writeField(
      world,
      entity,
      "Pose",
      0u,
      FieldKind::Vec3,
      lanes({nan, 0.0f, 0.0f})
    )
    == WriteStatus::Invalid
  );
  REQUIRE(poseOf(world, entity).position == before.position);
  REQUIRE(poseOf(world, entity).rotation == before.rotation);
}

TEST_CASE("read-only components refuse writes", "[componentschema]") {
  World world;
  const Entity entity = placed(world);
  std::vector<std::byte> extent(12u);
  REQUIRE(
    writeField(world, entity, "SdfGrid", 0u, FieldKind::UVec3, extent)
    == WriteStatus::ReadOnly
  );
  world.emplaceAuthored<Mesh>(entity, Mesh{});
  std::vector<std::byte> index(4u);
  REQUIRE(
    writeField(world, entity, "Mesh", 0u, FieldKind::UInt, index)
    == WriteStatus::ReadOnly
  );
}

TEST_CASE("an optional field reads present or absent", "[componentschema]") {
  World world;
  SdfGrid grid = blank();
  grid.margin = 0.25f;
  const Entity entity = world.createSdfGrid(Pose{}, grid);
  const auto bytes = componentBytes(world, entity, "SdfGrid");
  REQUIRE(bytes);
  REQUIRE(std::to_integer<uint8_t>((*bytes)[12]) == 1u);
  REQUIRE(floatAt(*bytes, 13u) == 0.25f);
  REQUIRE(std::to_integer<uint8_t>((*bytes)[17]) == 0u);
  REQUIRE(floatAt(*bytes, 18u) == 0.0f);
}

TEST_CASE(
  "a dynamic component shares the schema, the bytes and the write path",
  "[componentschema]"
) {
  World world;
  const auto type = world.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Wager",
      16u,
      {talus::objectmodel::FieldSpec{"count", FieldKind::UInt, 0u},
       talus::objectmodel::FieldSpec{"at", FieldKind::Vec3, 4u}}
    }
  );
  REQUIRE(type != talus::objectmodel::INVALID_COMPONENT_TYPE);
  const Entity entity = placed(world);
  std::array<std::byte, 16> value{};
  const uint32_t count = 7u;
  std::memcpy(value.data(), &count, 4u);
  REQUIRE(world.dynamic().emplace(type, entity, value));
  const auto schema = componentSchema(world, "Wager");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.size() == 2u);
  REQUIRE(schema->flags == COMPONENT_WRITABLE);
  REQUIRE(schema->defaults.size() == 16u);
  const auto before = componentBytes(world, entity, "Wager");
  REQUIRE(before);
  REQUIRE(uintAt(*before, 0u) == 7u);
  world.clearTouched();
  REQUIRE(
    writeField(
      world,
      entity,
      "Wager",
      1u,
      FieldKind::Vec3,
      lanes({1.0f, 2.0f, 3.0f})
    )
    == WriteStatus::Written
  );
  REQUIRE(world.touched().size() == 1u);
  const auto after = componentBytes(world, entity, "Wager");
  REQUIRE(after);
  REQUIRE(uintAt(*after, 0u) == 7u);
  REQUIRE(floatAt(*after, 8u) == 2.0f);
  REQUIRE(
    writeField(
      world,
      entity,
      "Wager",
      1u,
      FieldKind::Vec4,
      lanes({1.0f, 2.0f, 3.0f, 4.0f})
    )
    == WriteStatus::KindMismatch
  );
  REQUIRE(
    writeField(world, entity, "Wager", 0u, FieldKind::UInt, lanes({1.0f, 2.0f}))
    == WriteStatus::SizeMismatch
  );
}

TEST_CASE(
  "the component names are the authored list plus every declared dynamic",
  "[componentschema]"
) {
  World world;
  REQUIRE(talus::serialize::componentNames(world).size() == 14u);
  const auto type = world.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Wager",
      4u,
      {talus::objectmodel::FieldSpec{"count", FieldKind::UInt, 0u}}
    }
  );
  REQUIRE(type != talus::objectmodel::INVALID_COMPONENT_TYPE);
  const auto names = talus::serialize::componentNames(world);
  REQUIRE(names.size() == 15u);
  REQUIRE(names.back() == "Wager");
  REQUIRE(std::find(names.begin(), names.end(), "Pose") != names.end());
}

TEST_CASE(
  "adding a component by name gives it its defaults and publishes",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  const auto type = world.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Wager",
      4u,
      {talus::objectmodel::FieldSpec{"count", FieldKind::UInt, 0u}}
    }
  );
  REQUIRE(type != talus::objectmodel::INVALID_COMPONENT_TYPE);
  world.clearTouched();
  REQUIRE(talus::serialize::addComponent(world, entity, "MassScale"));
  REQUIRE(world.registry().all_of<MassScale>(entity));
  const auto scale = componentBytes(world, entity, "MassScale");
  REQUIRE(scale);
  REQUIRE(floatAt(*scale, 0u) == 1.0f);
  REQUIRE(world.touched().size() == 1u);
  REQUIRE_FALSE(talus::serialize::addComponent(world, entity, "MassScale"));
  REQUIRE(talus::serialize::addComponent(world, entity, "StaticBody"));
  REQUIRE(world.registry().all_of<talus::objectmodel::StaticBody>(entity));
  world.clearTouched();
  REQUIRE(talus::serialize::addComponent(world, entity, "Wager"));
  const auto wager = componentBytes(world, entity, "Wager");
  REQUIRE(wager);
  REQUIRE(uintAt(*wager, 0u) == 0u);
  REQUIRE(world.touched().size() == 1u);
  REQUIRE_FALSE(talus::serialize::addComponent(world, entity, "Wager"));
  REQUIRE_FALSE(talus::serialize::addComponent(world, entity, "Nope"));
  REQUIRE_FALSE(
    talus::serialize::addComponent(
      world,
      talus::objectmodel::INVALID_ENTITY,
      "MassScale"
    )
  );
}

TEST_CASE(
  "a component's requirements gate adding it and removing what it needs",
  "[componentschema]"
) {
  World world;
  const Entity bare = world.createAuthored();
  const Entity grid = placed(world);
  REQUIRE_FALSE(talus::serialize::addComponent(world, bare, "DeformableBody"));
  REQUIRE_FALSE(talus::serialize::addComponent(world, bare, "StaticBody"));
  REQUIRE_FALSE(talus::serialize::addComponent(world, bare, "MainCamera"));
  REQUIRE(talus::serialize::addComponent(world, grid, "DeformableBody"));
  REQUIRE(talus::serialize::addComponent(world, bare, "Lens"));
  REQUIRE(talus::serialize::addComponent(world, bare, "MainCamera"));
  REQUIRE(world.registry().all_of<talus::objectmodel::MainCamera>(bare));
  const Entity other = world.createAuthored();
  REQUIRE(talus::serialize::addComponent(world, other, "Lens"));
  world.clearTouched();
  REQUIRE(talus::serialize::addComponent(world, other, "MainCamera"));
  REQUIRE(world.registry().all_of<talus::objectmodel::MainCamera>(other));
  REQUIRE_FALSE(world.registry().all_of<talus::objectmodel::MainCamera>(bare));
  REQUIRE(world.touched().size() == 2u);
  REQUIRE_FALSE(talus::serialize::removeComponent(world, other, "Lens"));
  REQUIRE(talus::serialize::removeComponent(world, other, "MainCamera"));
  REQUIRE(talus::serialize::removeComponent(world, other, "Lens"));
  REQUIRE_FALSE(world.registry().all_of<talus::objectmodel::Lens>(other));
  REQUIRE_FALSE(talus::serialize::removeComponent(world, other, "Lens"));
  REQUIRE_FALSE(talus::serialize::removeComponent(world, grid, "SdfGrid"));
  REQUIRE_FALSE(talus::serialize::removeComponent(world, grid, "Pose"));
  world.clearTouched();
  REQUIRE(talus::serialize::removeComponent(world, grid, "DeformableBody"));
  REQUIRE_FALSE(world.registry().all_of<DeformableBody>(grid));
  REQUIRE(world.touched().size() == 1u);
}

TEST_CASE(
  "a script component's named requirements gate it the same way",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  const auto type = world.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Needy",
      4u,
      {talus::objectmodel::FieldSpec{"count", FieldKind::UInt, 0u}},
      {"DeformableBody"}
    }
  );
  REQUIRE(type != talus::objectmodel::INVALID_COMPONENT_TYPE);
  REQUIRE_FALSE(talus::serialize::addComponent(world, entity, "Needy"));
  REQUIRE(talus::serialize::addComponent(world, entity, "DeformableBody"));
  REQUIRE(talus::serialize::addComponent(world, entity, "Needy"));
  REQUIRE_FALSE(
    talus::serialize::removeComponent(world, entity, "DeformableBody")
  );
  REQUIRE(talus::serialize::removeComponent(world, entity, "Needy"));
  REQUIRE(talus::serialize::removeComponent(world, entity, "DeformableBody"));
  const auto again = world.dynamic().declare(
    talus::objectmodel::ComponentSpec{
      "Needy",
      4u,
      {talus::objectmodel::FieldSpec{"count", FieldKind::UInt, 0u}},
      {}
    }
  );
  REQUIRE(again == type);
  REQUIRE(talus::serialize::addComponent(world, entity, "Needy"));
}

TEST_CASE(
  "a name is a text field: derived, written, and always terminated",
  "[componentschema]"
) {
  World world;
  const Entity entity = placed(world);
  REQUIRE(talus::serialize::addComponent(world, entity, "Name"));
  const auto schema = talus::serialize::componentSchema(world, "Name");
  REQUIRE(schema);
  REQUIRE(schema->spec.fields.size() == 1u);
  REQUIRE(schema->spec.fields[0].name == "text");
  REQUIRE(schema->spec.fields[0].kind == FieldKind::Text);
  REQUIRE(schema->spec.size == 64u);
  REQUIRE((schema->flags & talus::serialize::COMPONENT_WRITABLE) != 0u);
  std::vector<std::byte> bytes(64u);
  const std::string_view wanted = "crate";
  std::memcpy(bytes.data(), wanted.data(), wanted.size());
  REQUIRE(
    talus::serialize::writeField(
      world,
      entity,
      "Name",
      0u,
      FieldKind::Text,
      bytes
    )
    == talus::serialize::WriteStatus::Written
  );
  REQUIRE(
    world.registry().get<talus::objectmodel::Name>(entity).text.view()
    == "crate"
  );
  const std::vector<std::byte> unterminated(64u, std::byte{'x'});
  REQUIRE(
    talus::serialize::writeField(
      world,
      entity,
      "Name",
      0u,
      FieldKind::Text,
      unterminated
    )
    == talus::serialize::WriteStatus::Invalid
  );
  REQUIRE(
    world.registry().get<talus::objectmodel::Name>(entity).text.view()
    == "crate"
  );
  const auto held = componentBytes(world, entity, "Name");
  REQUIRE(held);
  REQUIRE(held->size() == 64u);
}

TEST_CASE(
  "a second body cannot be added beside the first",
  "[componentschema]"
) {
  World world;
  const Entity fixed = placed(world);
  const Entity moving = placed(world);
  REQUIRE(talus::serialize::addComponent(world, fixed, "StaticBody"));
  REQUIRE(talus::serialize::addComponent(world, moving, "RigidBody"));
  REQUIRE_FALSE(talus::serialize::canAddComponent(world, fixed, "RigidBody"));
  REQUIRE_FALSE(
    talus::serialize::canAddComponent(world, fixed, "DeformableBody")
  );
  REQUIRE_FALSE(talus::serialize::canAddComponent(world, moving, "StaticBody"));
  REQUIRE_FALSE(
    talus::serialize::canAddComponent(world, moving, "DeformableBody")
  );
  REQUIRE_FALSE(
    talus::serialize::addComponent(world, moving, "DeformableBody")
  );
  REQUIRE(talus::serialize::removeComponent(world, moving, "RigidBody"));
  REQUIRE(talus::serialize::canAddComponent(world, moving, "DeformableBody"));
}

TEST_CASE(
  "a refused removal names the component that requires it",
  "[componentschema]"
) {
  World world;
  const Entity grid = placed(world);
  REQUIRE(talus::serialize::addComponent(world, grid, "RigidBody"));
  REQUIRE(
    talus::serialize::removalBlocker(world, grid, "SdfGrid") == "RigidBody"
  );
  REQUIRE(talus::serialize::removalBlocker(world, grid, "RigidBody").empty());
  REQUIRE(talus::serialize::removeComponent(world, grid, "RigidBody"));
  REQUIRE(talus::serialize::removalBlocker(world, grid, "SdfGrid").empty());
}
