#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>

#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/mesh/mesh.h>
#include <talus/objectmodel/component/bakedvolume/bakedvolume.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/deformed/deformed.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/trait/authored/authored.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/dynamiccomponents/dynamiccomponents.h>
#include <talus/objectmodel/sdfprimitivestore/sdfprimitivestore.h>
#include <talus/objectmodel/trait/selfcontained/selfcontained.h>
#include <talus/objectmodel/component/rigidbody/rigidbody.h>
#include <talus/objectmodel/component/sharedfield/sharedfield.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>

#include <talus/objectmodel/collection/collection.h>

#include <entt/core/hashed_string.hpp>
#include <entt/entity/registry.hpp>

#include <type_traits>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace talus::objectmodel {

class World {
public:
  World();

  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = delete;
  World& operator=(World&&) = delete;

  const entt::registry& registry() const noexcept;

  [[nodiscard]] DynamicComponents& dynamic() noexcept;
  [[nodiscard]] const DynamicComponents& dynamic() const noexcept;

  void clear();

  Entity createAuthored();

  [[nodiscard]]
  bool createAuthoredAt(Entity hint);

  template<Authored T>
  void emplaceAuthored(Entity entity, const T& value) {
    if constexpr (std::is_empty_v<T>) {
      m_registry.emplace_or_replace<T>(entity);
    } else {
      m_registry.emplace_or_replace<T>(entity, value);
    }
  }

  template<Authored T>
  void emplaceSingle(Entity entity, const T& value) {
    for (const Entity holder : m_registry.view<T>()) {
      touch(holder);
    }

    m_registry.clear<T>();
    emplaceAuthored(entity, value);
    touch(entity);
  }

  template<Authored T>
  bool removeAuthored(Entity entity) {
    if (!m_registry.all_of<T>(entity)) {
      return false;
    }

    m_registry.erase<T>(entity);
    touch(entity);

    return true;
  }

  Entity createSdfGrid(const Pose& pose, const SdfGrid& grid);

  [[nodiscard]]
  bool createSdfGridAt(Entity hint, const Pose& pose, const SdfGrid& grid);

  [[nodiscard]]
  bool destroy(Entity entity);

  [[nodiscard]] std::span<const Entity> fields() const noexcept;

  [[nodiscard]] std::span<const Entity> sdfGrids() const noexcept;

  [[nodiscard]]
  bool emplacePrimitiveRange(Entity entity);

  [[nodiscard]]
  bool addPrimitive(Entity entity, const talus::field::Primitive& primitive);

  [[nodiscard]]
  bool insertPrimitive(
    Entity entity,
    uint32_t index,
    const talus::field::Primitive& primitive
  );

  [[nodiscard]]
  bool setPrimitive(
    Entity entity,
    uint32_t index,
    const talus::field::Primitive& primitive
  );

  [[nodiscard]]
  bool removePrimitive(Entity entity, uint32_t index);

  void addStaticBody(Entity entity);

  void addRigidBody(Entity entity);

  [[nodiscard]]
  bool setMainCamera(Entity entity);

  [[nodiscard]] std::span<const Collection> collections() const noexcept;

  [[nodiscard]]
  const Collection* collection(std::string_view name) const noexcept;

  [[nodiscard]] bool createCollection(std::string_view name);

  [[nodiscard]] bool collect(std::string_view name, Entity entity);

  [[nodiscard]] bool uncollect(std::string_view name, Entity entity);

  [[nodiscard]] bool dropCollection(std::string_view name);

  std::span<const talus::field::Primitive> primitives(Entity entity) const;

  uint32_t primitiveCount(Entity entity) const;

  std::span<const talus::field::Primitive> pool() const noexcept;

  template<SelfContained T, typename Fn>
  void patch(Entity entity, Fn&& fn) {
    m_registry.patch<T>(entity, std::forward<Fn>(fn));
    touch(entity);
  }

  template<typename Fn>
  void patchSampledSdf(Entity entity, Fn&& fn) {
    if (!m_registry.any_of<DeformableBody, RigidBody, StaticBody>(entity)) {
      throw std::runtime_error(
        "Only a body's lattice may be written in place"
      );
    }

    SharedSdf& held = m_registry.get<SharedSdf>(entity);

    if (held.field.use_count() > 1) {
      held.field = std::make_shared<talus::field::SampledSdf>(*held.field);
    }

    fn(*held.field);

    m_registry.emplace_or_replace<Deformed>(entity);
  }

  template<SelfContained T>
  void emplaceOrReplace(Entity entity, const T& value) {
    m_registry.emplace_or_replace<T>(entity, value);
    touch(entity);
  }

  template<SelfContained T>
  void replace(Entity entity, const T& value) {
    m_registry.replace<T>(entity, value);
    touch(entity);
  }

  template<SelfContained T>
  void syncMany(std::span<const std::pair<Entity, T>> values) {
    for (const auto& entity : values) {
      if (m_registry.all_of<T>(entity.first)) {
        m_registry.storage<T>().get(entity.first) = entity.second;
      } else {
        m_registry.emplace<T>(entity.first, entity.second);
      }
    }
  }

  [[nodiscard]] std::vector<Entity> takePoseEdits();

  void touch(Entity entity);

  [[nodiscard]] std::span<const Entity> touched() const noexcept;

  [[nodiscard]] bool touchedOverflowed() const noexcept;

  void clearTouched() noexcept;

  void markSdfDirty(Entity entity, const talus::field::SampleBox& box);

  [[nodiscard]] std::span<const std::pair<Entity, talus::field::SampleBox>>
  sdfDirty() const noexcept;

  void clearSdfDirty() noexcept;

  void setDeformablePrepared(
    Entity entity,
    std::shared_ptr<const PreparedMesh> mesh
  );

  void setDeformableState(Entity entity, DeformableState state);

  template<typename Fn>
  void patchDeformableState(Entity entity, Fn&& fn) {
    auto* held = m_registry.try_get<DeformableState>(entity);

    if (held == nullptr) {
      throw std::runtime_error("Stepping an entity with no deformable state");
    }

    fn(*held);
  }

  [[nodiscard]]
  const PreparedMesh* deformablePrepared(Entity entity) const noexcept;

  [[nodiscard]]
  const DeformableState* deformableState(Entity entity) const noexcept;

  void setBakedVolume(Entity entity, uint32_t index);

  void setBodyId(Entity entity, uint32_t index);

  void clearBodyId(Entity entity);

  void setSampledSdf(Entity entity, talus::field::SampledSdf field);

  void shareSampledSdf(Entity donor, Entity taker);

  void adoptSampledSdf(Entity entity, const SharedSdf& held);

  [[nodiscard]]
  const talus::field::SampledSdf* sampledSdf(Entity entity) const;

  [[nodiscard]]
  const talus::field::SampledSdf* currentSampledSdf(Entity entity) const;

  [[nodiscard]]
  bool hasSampledSdf(Entity entity) const noexcept;

  [[nodiscard]]
  long sampledSdfUsers(Entity entity) const noexcept;

  void clearBakedVolume(Entity entity);

  void clearBakedVolumes();

  void onBakedVolumeReleased(std::function<void(uint32_t)> release);

  bool needsBake(Entity entity) const noexcept;
  void markBaked(Entity entity);

  bool needsResample(Entity entity) const noexcept;

  Entity createMesh(
    const Pose& pose,
    const Mesh& mesh,
    const Material& material
  );

  [[nodiscard]]
  bool createMeshAt(
    Entity hint,
    const Pose& pose,
    const Mesh& mesh,
    const Material& material
  );

  std::span<const Entity> meshes() const noexcept;

  [[nodiscard]] std::span<const PhysicalMaterial> materials() const noexcept;

  void setMaterials(std::vector<PhysicalMaterial> materials);

  [[nodiscard]] const PhysicalMaterial& material(uint32_t index) const noexcept;

private:
  void bakedVolumeDestroyed(entt::registry& registry, Entity entity);

  void markTouched(entt::registry& registry, Entity entity);

  std::function<void(uint32_t)> m_onBakedVolumeReleased;

  std::vector<Entity> m_touched;

  bool m_touchedOverflow = false;

  SdfPrimitiveStore m_primitiveStore;

  DynamicComponents m_dynamic;

  std::vector<Collection> m_collections;

  std::vector<std::pair<Entity, talus::field::SampleBox>> m_sdfDirty;

  std::vector<PhysicalMaterial> m_materials;

  entt::registry m_registry;
};

}
