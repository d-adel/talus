#include <catch2/catch_test_macros.hpp>

#include <talus/objectmodel/dynamiccomponents/dynamiccomponents.h>

#include <array>
#include <cstring>
#include <vector>

using talus::objectmodel::ComponentSpec;
using talus::objectmodel::ComponentType;
using talus::objectmodel::DynamicComponents;
using talus::objectmodel::Entity;
using talus::objectmodel::FieldKind;
using talus::objectmodel::FieldSpec;
using talus::objectmodel::INVALID_COMPONENT_TYPE;

namespace {

ComponentSpec budgetSpec() {
  return ComponentSpec{
    "Budget",
    8u,
    {FieldSpec{"allowance", FieldKind::Float, 0u},
     FieldSpec{"spent", FieldKind::Float, 4u}}
  };
}

std::array<std::byte, 8> budget(float allowance, float spent) {
  std::array<std::byte, 8> value{};

  std::memcpy(value.data(), &allowance, sizeof(float));
  std::memcpy(value.data() + 4, &spent, sizeof(float));

  return value;
}

float allowanceOf(const std::byte* at) {
  float value = 0.0f;
  std::memcpy(&value, at, sizeof(float));

  return value;
}

Entity entityAt(uint32_t index) {
  return static_cast<Entity>(entt::entity{index});
}

}

TEST_CASE(
  "a declared type is found by name and reports its layout",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  const ComponentType type = components.declare(budgetSpec());

  REQUIRE(type != INVALID_COMPONENT_TYPE);
  REQUIRE(components.types() == 1u);
  REQUIRE(components.find("Budget") == type);
  REQUIRE(components.find("Absent") == INVALID_COMPONENT_TYPE);

  const ComponentSpec* spec = components.spec(type);

  REQUIRE(spec != nullptr);
  REQUIRE(spec->size == 8u);
  REQUIRE(spec->fields.size() == 2u);
  REQUIRE(spec->fields[1].name == "spent");
  REQUIRE(spec->fields[1].offset == 4u);
}

TEST_CASE(
  "redeclaring the same layout is the same type, a different one is refused",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  const ComponentType first = components.declare(budgetSpec());

  REQUIRE(components.declare(budgetSpec()) == first);
  REQUIRE(components.types() == 1u);

  ComponentSpec changed = budgetSpec();
  changed.fields[1].offset = 0u;

  REQUIRE(components.declare(changed) == INVALID_COMPONENT_TYPE);
  REQUIRE(components.types() == 1u);
}

TEST_CASE(
  "a nameless type, a zero size and a field past the end are refused",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  REQUIRE(
    components.declare(ComponentSpec{"", 4u, {}}) == INVALID_COMPONENT_TYPE
  );
  REQUIRE(
    components.declare(ComponentSpec{"Empty", 0u, {}}) == INVALID_COMPONENT_TYPE
  );

  const ComponentSpec overrun{
    "Overrun",
    4u,
    {FieldSpec{"value", FieldKind::Vec3, 0u}}
  };

  REQUIRE(components.declare(overrun) == INVALID_COMPONENT_TYPE);
  REQUIRE(components.types() == 0u);
}

TEST_CASE("a value written is the value read back", "[dynamiccomponents]") {
  DynamicComponents components;

  const ComponentType type = components.declare(budgetSpec());
  const Entity entity = entityAt(3u);

  const auto value = budget(12.5f, 1.5f);

  REQUIRE(components.emplace(type, entity, value));
  REQUIRE(components.contains(type, entity));
  REQUIRE(components.count(type) == 1u);

  const std::byte* stored = components.get(type, entity);

  REQUIRE(stored != nullptr);
  REQUIRE(allowanceOf(stored) == 12.5f);
}

TEST_CASE(
  "emplacing twice overwrites rather than duplicating",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  const ComponentType type = components.declare(budgetSpec());
  const Entity entity = entityAt(1u);

  REQUIRE(components.emplace(type, entity, budget(1.0f, 0.0f)));
  REQUIRE(components.emplace(type, entity, budget(9.0f, 0.0f)));

  REQUIRE(components.count(type) == 1u);
  REQUIRE(allowanceOf(components.get(type, entity)) == 9.0f);
}

TEST_CASE("a wrong-sized value is refused", "[dynamiccomponents]") {
  DynamicComponents components;

  const ComponentType type = components.declare(budgetSpec());

  const std::array<std::byte, 4> small{};

  REQUIRE_FALSE(components.emplace(type, entityAt(0u), small));
  REQUIRE_FALSE(
    components.emplace(INVALID_COMPONENT_TYPE, entityAt(0u), budget(1.0f, 0.0f))
  );
  REQUIRE(components.count(type) == 0u);
}

TEST_CASE(
  "removing from the middle keeps entities and bytes in step",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  const ComponentType type = components.declare(budgetSpec());

  for (uint32_t index = 0u; index < 5u; ++index) {
    REQUIRE(components.emplace(
      type,
      entityAt(index),
      budget(static_cast<float>(index), 0.0f)
    ));
  }

  REQUIRE(components.remove(type, entityAt(1u)));
  REQUIRE_FALSE(components.remove(type, entityAt(1u)));

  REQUIRE(components.count(type) == 4u);
  REQUIRE_FALSE(components.contains(type, entityAt(1u)));

  const std::span<const Entity> entities = components.entities(type);
  const std::span<const std::byte> bytes = components.data(type);

  REQUIRE(entities.size() == 4u);
  REQUIRE(bytes.size() == 4u * 8u);

  for (size_t slot = 0u; slot < entities.size(); ++slot) {
    const float stored = allowanceOf(bytes.data() + slot * 8u);
    const float expected =
      static_cast<float>(entt::to_integral(entities[slot]));

    REQUIRE(stored == expected);
  }

  for (uint32_t index : {0u, 2u, 3u, 4u}) {
    REQUIRE(
      allowanceOf(components.get(type, entityAt(index)))
      == static_cast<float>(index)
    );
  }
}

TEST_CASE(
  "the store is one contiguous array a script can span",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  const ComponentType type = components.declare(budgetSpec());

  for (uint32_t index = 0u; index < 2000u; ++index) {
    REQUIRE(components.emplace(
      type,
      entityAt(index),
      budget(static_cast<float>(index), 0.0f)
    ));
  }

  const std::span<const std::byte> bytes = components.data(type);

  REQUIRE(bytes.size() == 2000u * 8u);

  for (uint32_t index = 0u; index < 2000u; ++index) {
    REQUIRE(
      allowanceOf(bytes.data() + index * 8u) == static_cast<float>(index)
    );
  }

  REQUIRE(components.get(type, entityAt(1999u)) == bytes.data() + 1999u * 8u);
}

TEST_CASE(
  "clearing one entity drops it from every type",
  "[dynamiccomponents]"
) {
  DynamicComponents components;

  const ComponentType budgetType = components.declare(budgetSpec());
  const ComponentType tagType =
    components.declare(ComponentSpec{"Payload", 1u, {}});

  const Entity entity = entityAt(7u);
  const std::array<std::byte, 1> tag{};

  REQUIRE(components.emplace(budgetType, entity, budget(1.0f, 0.0f)));
  REQUIRE(components.emplace(tagType, entity, tag));

  components.clear(entity);

  REQUIRE_FALSE(components.contains(budgetType, entity));
  REQUIRE_FALSE(components.contains(tagType, entity));
  REQUIRE(components.types() == 2u);
}

TEST_CASE(
  "field kinds report the size a script must match",
  "[dynamiccomponents]"
) {
  REQUIRE(talus::objectmodel::fieldSize(FieldKind::Float) == 4u);
  REQUIRE(talus::objectmodel::fieldSize(FieldKind::Bool) == 1u);
  REQUIRE(talus::objectmodel::fieldSize(FieldKind::Vec3) == 12u);
  REQUIRE(talus::objectmodel::fieldSize(FieldKind::Quat) == 16u);
  REQUIRE(talus::objectmodel::fieldKindName(FieldKind::Vec3) == "vec3");
}

TEST_CASE("every field kind agrees with itself", "[dynamiccomponents]") {
  using talus::objectmodel::FieldKind;

  for (uint32_t raw = 0u; raw < static_cast<uint32_t>(FieldKind::Count);
       ++raw) {
    const auto kind = static_cast<FieldKind>(raw);
    const auto& info = talus::objectmodel::fieldKindInfo(kind);

    INFO("kind " << talus::objectmodel::fieldKindName(kind));

    REQUIRE(talus::objectmodel::isFieldKind(raw));
    REQUIRE_FALSE(info.name.empty());
    REQUIRE(info.lanes >= 1u);
    REQUIRE(talus::objectmodel::fieldSize(kind) == info.lanes * info.laneBytes);
    REQUIRE(talus::objectmodel::fieldLanes(kind) == info.lanes);
  }

  REQUIRE_FALSE(
    talus::objectmodel::isFieldKind(static_cast<uint32_t>(FieldKind::Count))
  );
}

TEST_CASE(
  "clearing entities keeps the ids already handed out valid",
  "[dynamic]"
) {
  DynamicComponents dynamic;

  const ComponentType budgetType = dynamic.declare(budgetSpec());

  REQUIRE(budgetType != INVALID_COMPONENT_TYPE);

  const auto value = budget(5.0f, 1.0f);

  REQUIRE(dynamic.emplace(budgetType, entityAt(0), value));
  REQUIRE(dynamic.count(budgetType) == 1);

  dynamic.clearEntities();

  REQUIRE(dynamic.types() == 1);
  REQUIRE(dynamic.count(budgetType) == 0);
  REQUIRE(dynamic.spec(budgetType) != nullptr);
  REQUIRE(dynamic.declare(budgetSpec()) == budgetType);

  REQUIRE(dynamic.emplace(budgetType, entityAt(0), value));
  REQUIRE(allowanceOf(dynamic.get(budgetType, entityAt(0))) == 5.0f);
}

TEST_CASE("clearing entities keeps every declaration in place", "[dynamic]") {
  DynamicComponents dynamic;

  ComponentSpec other = budgetSpec();
  other.name = "Payload";

  const ComponentType first = dynamic.declare(budgetSpec());
  const ComponentType second = dynamic.declare(other);

  REQUIRE(first != second);

  dynamic.clearEntities();

  REQUIRE(dynamic.declare(budgetSpec()) == first);
  REQUIRE(dynamic.declare(other) == second);
}

TEST_CASE(
  "an unsigned vector kind is three unsigned lanes",
  "[dynamiccomponents]"
) {
  using talus::objectmodel::FieldLane;
  REQUIRE(talus::objectmodel::fieldSize(FieldKind::UVec3) == 12u);
  REQUIRE(talus::objectmodel::fieldLanes(FieldKind::UVec3) == 3u);
  REQUIRE(talus::objectmodel::fieldLane(FieldKind::UVec3) == FieldLane::UInt);
  REQUIRE(talus::objectmodel::fieldLane(FieldKind::Vec3) == FieldLane::Float);
  REQUIRE(talus::objectmodel::fieldLane(FieldKind::Bool) == FieldLane::Bool);
  REQUIRE(talus::objectmodel::fieldKindName(FieldKind::UVec3) == "uvec3");
}

TEST_CASE(
  "layouts that differ only in flags are different",
  "[dynamiccomponents]"
) {
  const ComponentSpec plain{
    "Margin",
    5u,
    {FieldSpec{"value", FieldKind::Float, 1u, 0u}}
  };
  const ComponentSpec optional{
    "Margin",
    5u,
    {FieldSpec{
      "value",
      FieldKind::Float,
      1u,
      talus::objectmodel::FIELD_OPTIONAL
    }}
  };
  REQUIRE(talus::objectmodel::sameLayout(plain, plain));
  REQUIRE_FALSE(talus::objectmodel::sameLayout(plain, optional));
}
