#pragma once

#include <talus/objectmodel/dynamiccomponents/dynamiccomponents.h>
#include <talus/objectmodel/entity/entity.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talus::objectmodel {
class World;
}

namespace talus::serialize {

inline constexpr uint32_t COMPONENT_WRITABLE = 1u;
inline constexpr uint32_t COMPONENT_ASSET_BACKED = 2u;

struct ComponentSchema {
  talus::objectmodel::ComponentSpec spec;
  uint32_t flags = 0u;
  std::vector<std::byte> defaults;
};

enum class WriteStatus : uint8_t {
  Written,
  NoEntity,
  NoComponent,
  NoField,
  KindMismatch,
  SizeMismatch,
  ReadOnly,
  Invalid
};

[[nodiscard]] std::string_view describe(WriteStatus status) noexcept;

[[nodiscard]] uint32_t fieldByteSize(
  talus::objectmodel::FieldKind kind,
  uint32_t flags
) noexcept;

[[nodiscard]] std::vector<ComponentSchema> authoredSchemas();

[[nodiscard]] std::vector<std::string> componentNames(
  const talus::objectmodel::World& world
);

[[nodiscard]] bool canAddComponent(
  const talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  std::string_view component
);

[[nodiscard]] bool addComponent(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  std::string_view component
);

[[nodiscard]] bool removeComponent(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  std::string_view component
);

[[nodiscard]] std::string removalBlocker(
  const talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  std::string_view component
);

[[nodiscard]] std::optional<ComponentSchema> componentSchema(
  const talus::objectmodel::World& world,
  std::string_view component
);

[[nodiscard]] std::optional<std::vector<std::byte>> componentBytes(
  const talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  std::string_view component
);

[[nodiscard]] WriteStatus writeField(
  talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  std::string_view component,
  uint32_t field,
  talus::objectmodel::FieldKind kind,
  std::span<const std::byte> bytes
);

}
