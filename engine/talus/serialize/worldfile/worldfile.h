#pragma once

#include <talus/core/asset/assetdatabase.h>
#include <talus/field/field.h>
#include <talus/objectmodel/trait/authoredcomponents/authoredcomponents.h>
#include <talus/objectmodel/trait/assetbacked/assetbacked.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/directionallight/directionallight.h>
#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/maincamera/maincamera.h>
#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/mesh/mesh.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/rigidbody/rigidbody.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/environment/environment.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/component/support/support.h>
#include <talus/objectmodel/dynamiccomponents/dynamiccomponents.h>
#include <talus/objectmodel/world/world.h>

#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

namespace talus::serialize {

inline constexpr uint32_t WORLD_VERSION = 1u;

template<typename T>
using StoredType = std::
  conditional_t<talus::objectmodel::assetBacked<T>, talus::core::AssetId, T>;

struct StoredPrimitive {
  talus::field::Primitive primitive{};
  talus::core::AssetId material = talus::core::INVALID_ASSET;
};

struct StoredField {
  std::string name;
  uint32_t kind = 0u;
  uint32_t offset = 0u;
};

struct StoredComponentType {
  std::string name;
  uint32_t size = 0u;
  std::vector<StoredField> fields;
};

struct StoredDynamic {
  std::string type;
  std::vector<double> values;
};

struct StoredEntity {
  std::optional<talus::objectmodel::Pose> pose;
  std::optional<talus::objectmodel::SdfGrid> grid;
  std::optional<talus::objectmodel::Lens> lens;
  std::optional<talus::objectmodel::MainCamera> mainCamera;
  std::optional<talus::objectmodel::MassScale> massScale;
  std::optional<talus::objectmodel::StaticBody> staticBody;
  std::optional<talus::objectmodel::RigidBody> rigidBody;
  std::optional<talus::objectmodel::DeformableBody> deformableBody;
  std::optional<talus::objectmodel::Support> support;

  std::optional<talus::objectmodel::DirectionalLight> directionalLight;
  std::optional<talus::objectmodel::Environment> environment;
  std::optional<talus::objectmodel::Name> name;

  std::optional<talus::core::AssetId> mesh;
  std::optional<talus::core::AssetId> material;

  std::vector<StoredPrimitive> primitives;

  std::vector<StoredDynamic> dynamic;
};

template<talus::objectmodel::Authored T>
struct PortableComponent {
  using Component = T;

  std::optional<StoredType<T>> StoredEntity::* slot;
};

inline constexpr auto PORTABLE_COMPONENTS = std::tuple{
  PortableComponent<talus::objectmodel::Pose>{&StoredEntity::pose},
  PortableComponent<talus::objectmodel::SdfGrid>{&StoredEntity::grid},
  PortableComponent<talus::objectmodel::Lens>{&StoredEntity::lens},
  PortableComponent<talus::objectmodel::MainCamera>{&StoredEntity::mainCamera},
  PortableComponent<talus::objectmodel::MassScale>{&StoredEntity::massScale},
  PortableComponent<talus::objectmodel::StaticBody>{&StoredEntity::staticBody},
  PortableComponent<talus::objectmodel::RigidBody>{&StoredEntity::rigidBody},
  PortableComponent<talus::objectmodel::DeformableBody>{&StoredEntity::deformableBody},
  PortableComponent<talus::objectmodel::Support>{&StoredEntity::support},
  PortableComponent<talus::objectmodel::Mesh>{&StoredEntity::mesh},
  PortableComponent<talus::objectmodel::Material>{&StoredEntity::material},
  PortableComponent<talus::objectmodel::DirectionalLight>{
    &StoredEntity::directionalLight
  },
  PortableComponent<talus::objectmodel::Environment>{&StoredEntity::environment},
  PortableComponent<talus::objectmodel::Name>{&StoredEntity::name}
};

template<typename... Ts>
consteval bool bindsEvery(talus::objectmodel::ComponentList<Ts...>) {
  return std::apply(
    [](auto... bound) {
      return std::is_same_v<
        std::tuple<Ts...>,
        std::tuple<typename decltype(bound)::Component...>>;
    },
    PORTABLE_COMPONENTS
  );
}

static_assert(bindsEvery(talus::objectmodel::AuthoredComponents{}));

struct StoredCollection {
  std::string name;
  std::vector<uint32_t> members;
};

struct StoredWorld {
  uint32_t version = WORLD_VERSION;
  std::vector<StoredEntity> entities;

  std::vector<StoredComponentType> componentTypes;

  std::vector<StoredCollection> collections;
};

[[nodiscard]] StoredWorld captureWorld(
  const talus::objectmodel::World& world,
  const talus::core::AssetDatabase& assets
);

[[nodiscard]] bool restoreWorld(
  const StoredWorld& stored,
  talus::objectmodel::World& world,
  const talus::core::AssetDatabase& assets
);

[[nodiscard]] std::string writeWorld(const StoredWorld& stored);

[[nodiscard]] bool readWorld(std::string_view text, StoredWorld& stored);

}
