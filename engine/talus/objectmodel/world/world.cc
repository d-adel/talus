#include "world.ih"

namespace talus::objectmodel {

namespace {

constexpr entt::id_type BAKE_QUEUE = entt::hashed_string{"bake"};

constexpr entt::id_type RESAMPLE_QUEUE = entt::hashed_string{"resample"};
constexpr entt::id_type POSE_EDITS = entt::hashed_string{"poseedits"};

constexpr size_t MAX_TOUCHED = 1024;

}

World::World() {
  m_primitiveStore.connect(m_registry);

  m_registry.on_destroy<BakedVolume>().connect<&World::bakedVolumeDestroyed>(
    *this
  );

  m_registry.storage<entt::reactive>(BAKE_QUEUE)
    .on_construct<SdfGrid>()
    .on_update<SdfGrid>();

  m_registry.storage<entt::reactive>(RESAMPLE_QUEUE)
    .on_construct<SdfGrid>()
    .on_update<SdfGrid>();
  m_registry.storage<entt::reactive>(POSE_EDITS).on_update<Pose>();

  AuthoredComponents::each([this]<typename T>() {
    m_registry.on_construct<T>().template connect<&World::markTouched>(*this);
    m_registry.on_destroy<T>().template connect<&World::markTouched>(*this);
  });

  TransientComponents::each([this]<typename T>() {
    m_registry.on_construct<T>().template connect<&World::markTouched>(*this);
    m_registry.on_destroy<T>().template connect<&World::markTouched>(*this);
  });
}

void World::touch(Entity entity) {
  markTouched(m_registry, entity);
}

std::vector<Entity> World::takePoseEdits() {
  auto& edits = m_registry.storage<entt::reactive>(POSE_EDITS);
  std::vector<Entity> taken(edits.begin(), edits.end());
  edits.clear();
  return taken;
}

void World::markTouched(entt::registry&, Entity entity) {
  if (m_touchedOverflow) {
    return;
  }

  if (!m_touched.empty() && m_touched.back() == entity) {
    return;
  }

  if (m_touched.size() >= MAX_TOUCHED) {
    m_touchedOverflow = true;
    m_touched.clear();

    return;
  }

  m_touched.push_back(entity);
}

std::span<const Entity> World::touched() const noexcept {
  return m_touched;
}

bool World::touchedOverflowed() const noexcept {
  return m_touchedOverflow;
}

void World::clearTouched() noexcept {
  m_touched.clear();
  m_touchedOverflow = false;
}

const entt::registry& World::registry() const noexcept {
  return m_registry;
}

void World::clear() {
  m_sdfDirty.clear();
  m_collections.clear();
  m_dynamic.clearEntities();
  m_registry.clear();

  m_touched.clear();
  m_touchedOverflow = true;
}

DynamicComponents& World::dynamic() noexcept {
  return m_dynamic;
}

const DynamicComponents& World::dynamic() const noexcept {
  return m_dynamic;
}

Entity World::createAuthored() {
  const Entity entity = m_registry.create();

  markTouched(m_registry, entity);

  return entity;
}

bool World::createAuthoredAt(Entity hint) {
  const Entity entity = m_registry.create(hint);

  if (entity != hint) {
    m_registry.destroy(entity);
    return false;
  }

  markTouched(m_registry, entity);

  return true;
}

Entity World::createSdfGrid(const Pose& pose, const SdfGrid& grid) {
  const Entity entity = m_registry.create();

  m_registry.emplace<Pose>(entity, pose);
  m_registry.emplace<SdfGrid>(entity, grid);

  return entity;
}

bool World::createSdfGridAt(
  Entity hint,
  const Pose& pose,
  const SdfGrid& grid
) {
  const Entity entity = m_registry.create(hint);

  if (entity != hint) {
    m_registry.destroy(entity);
    return false;
  }

  m_registry.emplace<Pose>(entity, pose);
  m_registry.emplace<SdfGrid>(entity, grid);

  return true;
}

bool World::destroy(Entity entity) {
  if (!m_registry.valid(entity)) {
    return false;
  }

  m_dynamic.clear(entity);

  for (Collection& held : m_collections) {
    std::erase(held.members, entity);
  }

  m_registry.destroy(entity);

  return true;
}

std::span<const Entity> World::fields() const noexcept {
  return sdfGrids();
}

std::span<const Entity> World::sdfGrids() const noexcept {
  const auto* storage = m_registry.storage<SdfGrid>();

  if (storage == nullptr) {
    return {};
  }

  return {storage->data(), storage->size()};
}

bool World::emplacePrimitiveRange(Entity entity) {
  return m_primitiveStore.emplaceRange(m_registry, entity);
}

bool World::addPrimitive(
  Entity entity,
  const talus::field::Primitive& primitive
) {
  return m_primitiveStore.append(m_registry, entity, primitive);
}

bool World::insertPrimitive(
  Entity entity,
  uint32_t index,
  const talus::field::Primitive& primitive
) {
  return m_primitiveStore.insert(m_registry, entity, index, primitive);
}

bool World::setPrimitive(
  Entity entity,
  uint32_t index,
  const talus::field::Primitive& primitive
) {
  return m_primitiveStore.set(m_registry, entity, index, primitive);
}

bool World::removePrimitive(Entity entity, uint32_t index) {
  return m_primitiveStore.remove(m_registry, entity, index);
}

void World::addStaticBody(Entity entity) {
  m_registry.emplace_or_replace<StaticBody>(entity);
}

void World::addRigidBody(Entity entity) {
  m_registry.emplace_or_replace<RigidBody>(entity);
}

bool World::setMainCamera(Entity entity) {
  if (!m_registry.valid(entity) || !m_registry.all_of<Lens>(entity)) {
    return false;
  }

  m_registry.clear<MainCamera>();
  m_registry.emplace<MainCamera>(entity);

  return true;
}

std::span<const Collection> World::collections() const noexcept {
  return m_collections;
}

const Collection* World::collection(std::string_view name) const noexcept {
  for (const Collection& held : m_collections) {
    if (held.name == name) {
      return &held;
    }
  }

  return nullptr;
}

bool World::createCollection(std::string_view name) {
  if (name.empty() || collection(name) != nullptr) {
    return false;
  }

  m_collections.push_back(Collection{std::string(name), {}});

  return true;
}

bool World::collect(std::string_view name, Entity entity) {
  if (name.empty() || !m_registry.valid(entity)) {
    return false;
  }

  if (collection(name) == nullptr) {
    m_collections.push_back(Collection{std::string(name), {}});
  }

  for (Collection& held : m_collections) {
    if (held.name != name) {
      continue;
    }

    if (std::find(held.members.begin(), held.members.end(), entity)
        != held.members.end()) {
      return false;
    }

    held.members.push_back(entity);
    touch(entity);

    return true;
  }

  return false;
}

bool World::uncollect(std::string_view name, Entity entity) {
  for (Collection& held : m_collections) {
    if (held.name != name) {
      continue;
    }

    const auto at = std::find(held.members.begin(), held.members.end(), entity);

    if (at == held.members.end()) {
      return false;
    }

    held.members.erase(at);
    touch(entity);

    return true;
  }

  return false;
}

bool World::dropCollection(std::string_view name) {
  for (auto at = m_collections.begin(); at != m_collections.end(); ++at) {
    if (at->name != name) {
      continue;
    }

    for (const Entity member : at->members) {
      touch(member);
    }

    m_collections.erase(at);

    return true;
  }

  return false;
}

std::span<const talus::field::Primitive> World::primitives(
  Entity entity
) const {
  return m_primitiveStore.primitives(m_registry, entity);
}

uint32_t World::primitiveCount(Entity entity) const {
  return m_primitiveStore.count(m_registry, entity);
}

std::span<const talus::field::Primitive> World::pool() const noexcept {
  return m_primitiveStore.pool();
}

void World::setDeformablePrepared(
  Entity entity,
  std::shared_ptr<const PreparedMesh> mesh
) {
  if (mesh == nullptr) {
    throw std::runtime_error("Preparing a deformable with no mesh");
  }

  m_registry.emplace_or_replace<DeformablePrepared>(entity, std::move(mesh));
}

void World::setDeformableState(Entity entity, DeformableState state) {
  m_registry.emplace_or_replace<DeformableState>(entity, std::move(state));
}

const PreparedMesh* World::deformablePrepared(Entity entity) const noexcept {
  const auto* held = m_registry.try_get<DeformablePrepared>(entity);

  return held == nullptr ? nullptr : held->mesh.get();
}

const DeformableState* World::deformableState(Entity entity) const noexcept {
  return m_registry.try_get<DeformableState>(entity);
}

void World::setBakedVolume(Entity entity, uint32_t index) {
  m_registry.remove<BakedVolume>(entity);
  m_registry.emplace<BakedVolume>(entity, index);
}

void World::setBodyId(Entity entity, uint32_t index) {
  m_registry.emplace_or_replace<BodyId>(entity, index);
}

void World::clearBodyId(Entity entity) {
  m_registry.remove<BodyId>(entity);
}

void World::setSampledSdf(Entity entity, talus::field::SampledSdf field) {
  m_registry.emplace_or_replace<SharedSdf>(
    entity,
    std::make_shared<talus::field::SampledSdf>(std::move(field))
  );

  m_registry.storage<entt::reactive>(RESAMPLE_QUEUE).remove(entity);

  m_registry.remove<Deformed>(entity);
}

void World::shareSampledSdf(Entity donor, Entity taker) {
  const auto* held = m_registry.try_get<SharedSdf>(donor);

  if (held == nullptr) {
    throw std::runtime_error("Sharing a lattice from an object without one");
  }

  adoptSampledSdf(taker, SharedSdf{held->field});
}

void World::adoptSampledSdf(Entity entity, const SharedSdf& held) {
  if (held.field == nullptr) {
    throw std::runtime_error("Adopting an empty lattice handle");
  }

  m_registry.emplace_or_replace<SharedSdf>(entity, held);

  m_registry.storage<entt::reactive>(RESAMPLE_QUEUE).remove(entity);
}

const talus::field::SampledSdf* World::sampledSdf(Entity entity) const {
  const auto* held = m_registry.try_get<SharedSdf>(entity);

  return held == nullptr ? nullptr : held->field.get();
}

const talus::field::SampledSdf* World::currentSampledSdf(Entity entity) const {
  return needsResample(entity) ? nullptr : sampledSdf(entity);
}

bool World::hasSampledSdf(Entity entity) const noexcept {
  return m_registry.all_of<SharedSdf>(entity);
}

long World::sampledSdfUsers(Entity entity) const noexcept {
  const auto* held = m_registry.try_get<SharedSdf>(entity);

  return held == nullptr ? 0 : held->field.use_count();
}

void World::clearBakedVolume(Entity entity) {
  m_registry.remove<BakedVolume>(entity);
}

void World::clearBakedVolumes() {
  for (const Entity entity : std::vector<Entity>(
         m_registry.view<BakedVolume>().begin(),
         m_registry.view<BakedVolume>().end()
       )) {
    m_registry.remove<BakedVolume>(entity);
  }
}

void World::onBakedVolumeReleased(std::function<void(uint32_t)> release) {
  m_onBakedVolumeReleased = std::move(release);
}

void World::bakedVolumeDestroyed(entt::registry& registry, Entity entity) {
  if (m_onBakedVolumeReleased) {
    m_onBakedVolumeReleased(registry.get<BakedVolume>(entity).index);
  }
}

bool World::needsBake(Entity entity) const noexcept {
  const auto* queue = m_registry.storage<entt::reactive>(BAKE_QUEUE);

  return queue != nullptr && queue->contains(entity);
}

bool World::needsResample(Entity entity) const noexcept {
  const auto* queue = m_registry.storage<entt::reactive>(RESAMPLE_QUEUE);

  return queue != nullptr && queue->contains(entity);
}

void World::markBaked(Entity entity) {
  m_registry.storage<entt::reactive>(BAKE_QUEUE).remove(entity);
}

Entity World::createMesh(
  const Pose& pose,
  const Mesh& mesh,
  const Material& material
) {
  const Entity entity = m_registry.create();

  m_registry.emplace<Pose>(entity, pose);
  m_registry.emplace<Mesh>(entity, mesh);
  m_registry.emplace<Material>(entity, material);

  return entity;
}

bool World::createMeshAt(
  Entity hint,
  const Pose& pose,
  const Mesh& mesh,
  const Material& material
) {
  const Entity entity = m_registry.create(hint);

  if (entity != hint) {
    m_registry.destroy(entity);
    return false;
  }

  m_registry.emplace<Pose>(entity, pose);
  m_registry.emplace<Mesh>(entity, mesh);
  m_registry.emplace<Material>(entity, material);

  return true;
}

std::span<const Entity> World::meshes() const noexcept {
  const auto* storage = m_registry.storage<Mesh>();

  if (storage == nullptr) {
    return {};
  }

  return {storage->data(), storage->size()};
}

std::span<const PhysicalMaterial> World::materials() const noexcept {
  return m_materials;
}

void World::setMaterials(std::vector<PhysicalMaterial> materials) {
  m_materials = std::move(materials);
}

const PhysicalMaterial& World::material(uint32_t index) const noexcept {
  static const PhysicalMaterial DEFAULT{};

  return index < m_materials.size() ? m_materials[index] : DEFAULT;
}

void World::markSdfDirty(Entity entity, const talus::field::SampleBox& box) {
  const auto found =
    std::find_if(m_sdfDirty.begin(), m_sdfDirty.end(), [entity](const auto& e) {
      return e.first == entity;
    });

  if (found == m_sdfDirty.end()) {
    m_sdfDirty.emplace_back(entity, box);
  } else {
    found->second = talus::field::merge(found->second, box);
  }
}

std::span<const std::pair<Entity, talus::field::SampleBox>> World::
  sdfDirty() const noexcept {
  return m_sdfDirty;
}

void World::clearSdfDirty() noexcept {
  m_sdfDirty.clear();
}

}
