#include "api.ih"

namespace talus::script {

namespace {

using talus::objectmodel::ComponentSpec;
using talus::objectmodel::ComponentType;
using talus::objectmodel::Entity;
using talus::objectmodel::FieldKind;
using talus::objectmodel::FieldSpec;
using talus::objectmodel::INVALID_COMPONENT_TYPE;
using talus::objectmodel::World;

World* worldOf(void* handle) noexcept {
  return static_cast<World*>(handle);
}

Entity entityOf(uint32_t value) noexcept {
  return static_cast<Entity>(entt::entity{value});
}

uint32_t declareComponent(
  void* world,
  const char* name,
  uint32_t size,
  const FieldDescriptor* fields,
  uint32_t fieldCount,
  const char* const* requirements,
  uint32_t requirementCount
) {
  World* target = worldOf(world);

  if (target == nullptr || name == nullptr) {
    return INVALID_COMPONENT_TYPE;
  }

  ComponentSpec spec;
  spec.name = name;
  spec.size = size;

  for (uint32_t index = 0u; index < fieldCount; ++index) {
    const FieldDescriptor& field = fields[index];

    if (field.name == nullptr || !talus::objectmodel::isFieldKind(field.kind)) {
      return INVALID_COMPONENT_TYPE;
    }

    spec.fields.push_back(
      FieldSpec{field.name, static_cast<FieldKind>(field.kind), field.offset}
    );
  }

  for (uint32_t index = 0u; index < requirementCount; ++index) {
    if (requirements == nullptr || requirements[index] == nullptr) {
      return INVALID_COMPONENT_TYPE;
    }

    spec.requirements.emplace_back(requirements[index]);
  }

  return target->dynamic().declare(std::move(spec));
}

uint32_t findComponent(void* world, const char* name) {
  World* target = worldOf(world);

  if (target == nullptr || name == nullptr) {
    return INVALID_COMPONENT_TYPE;
  }

  return target->dynamic().find(name);
}

int32_t setComponent(
  void* world,
  uint32_t type,
  uint32_t entity,
  const void* value
) {
  World* target = worldOf(world);

  if (target == nullptr || value == nullptr) {
    return 0;
  }

  const ComponentSpec* spec = target->dynamic().spec(type);

  if (spec == nullptr) {
    return 0;
  }

  const std::span<const std::byte> bytes(
    static_cast<const std::byte*>(value),
    spec->size
  );

  return target->dynamic().emplace(type, entityOf(entity), bytes) ? 1 : 0;
}

void* getComponent(void* world, uint32_t type, uint32_t entity) {
  World* target = worldOf(world);

  return target == nullptr ? nullptr
                           : target->dynamic().get(type, entityOf(entity));
}

int32_t removeComponent(void* world, uint32_t type, uint32_t entity) {
  World* target = worldOf(world);

  return target != nullptr && target->dynamic().remove(type, entityOf(entity))
           ? 1
           : 0;
}

uint32_t componentCount(void* world, uint32_t type) {
  World* target = worldOf(world);

  return target == nullptr
           ? 0u
           : static_cast<uint32_t>(target->dynamic().count(type));
}

const uint32_t* componentEntities(void* world, uint32_t type) {
  World* target = worldOf(world);

  if (target == nullptr) {
    return nullptr;
  }

  const std::span<const Entity> entities = target->dynamic().entities(type);

  return entities.empty() ? nullptr
                          : reinterpret_cast<const uint32_t*>(entities.data());
}

void* componentData(void* world, uint32_t type) {
  World* target = worldOf(world);

  if (target == nullptr) {
    return nullptr;
  }

  const std::span<std::byte> bytes = target->dynamic().data(type);

  return bytes.empty() ? nullptr : bytes.data();
}

LogSink g_sink = nullptr;

float sampleSdf(void* world, uint32_t entity, const float* point) {
  World* target = worldOf(world);

  if (target == nullptr || point == nullptr) {
    return 0.0f;
  }

  const Entity subject = entityOf(entity);
  const talus::field::SampledSdf* field = target->sampledSdf(subject);

  if (
    field == nullptr
    || !target->registry().all_of<talus::objectmodel::Pose>(subject)
  ) {
    return 0.0f;
  }

  const glm::mat4 toLocal = glm::inverse(
    talus::objectmodel::model(
      target->registry().get<talus::objectmodel::Pose>(subject)
    )
  );

  const glm::vec3 local =
    glm::vec3(toLocal * glm::vec4(point[0], point[1], point[2], 1.0f));

  return talus::field::distance(*field, local);
}

int32_t getPose(void* world, uint32_t entity, float* pose) {
  World* target = worldOf(world);

  if (target == nullptr || pose == nullptr) {
    return 0;
  }

  const Entity subject = entityOf(entity);

  if (!target->registry().all_of<talus::objectmodel::Pose>(subject)) {
    return 0;
  }

  const talus::objectmodel::Pose& held =
    target->registry().get<talus::objectmodel::Pose>(subject);

  pose[0] = held.position.x;
  pose[1] = held.position.y;
  pose[2] = held.position.z;
  pose[3] = held.rotation.x;
  pose[4] = held.rotation.y;
  pose[5] = held.rotation.z;
  pose[6] = held.rotation.w;

  return 1;
}

int32_t setPose(void* world, uint32_t entity, const float* pose) {
  World* target = worldOf(world);

  if (target == nullptr || pose == nullptr) {
    return 0;
  }

  const Entity subject = entityOf(entity);

  if (!target->registry().all_of<talus::objectmodel::Pose>(subject)) {
    return 0;
  }

  talus::objectmodel::Pose held{};
  held.position = glm::vec3(pose[0], pose[1], pose[2]);
  held.rotation = glm::quat(pose[6], pose[3], pose[4], pose[5]);

  target->replace<talus::objectmodel::Pose>(subject, held);

  return 1;
}

uint32_t entities(void* world, uint32_t* buffer, uint32_t capacity) {
  World* target = worldOf(world);

  if (target == nullptr) {
    return 0u;
  }

  const std::vector<Entity> live = talus::objectmodel::liveEntities(*target);

  for (uint32_t slot = 0u; slot < live.size() && slot < capacity; ++slot) {
    buffer[slot] = static_cast<uint32_t>(entt::to_integral(live[slot]));
  }

  return static_cast<uint32_t>(live.size());
}

int32_t hasComponent(void* world, uint32_t entity, const char* name) {
  World* target = worldOf(world);

  if (target == nullptr || name == nullptr) {
    return 0;
  }

  const std::vector<std::string> carried =
    talus::objectmodel::componentNames(*target, entityOf(entity));

  return std::ranges::find(carried, name) == carried.end() ? 0 : 1;
}

int32_t bounds(void* world, uint32_t entity, float* minimum, float* maximum) {
  World* target = worldOf(world);

  if (target == nullptr || minimum == nullptr || maximum == nullptr) {
    return 0;
  }

  const talus::objectmodel::WorldExtent extent =
    talus::objectmodel::entityExtent(*target, entityOf(entity));

  if (extent.empty) {
    return 0;
  }

  for (uint32_t axis = 0u; axis < 3u; ++axis) {
    minimum[axis] = extent.minimum[axis];
    maximum[axis] = extent.maximum[axis];
  }

  return 1;
}

void logMessage(const char* message) {
  if (message == nullptr) {
    return;
  }

  if (g_sink != nullptr) {
    g_sink(message);
    return;
  }

  std::cout << "[script] " << message << std::endl;
}

size_t indexOf(
  const talus::field::SampledSdf& field,
  const talus::field::SampleBox& box,
  uint32_t x,
  uint32_t y,
  uint32_t z
) {
  const uint32_t px = box.minimum.x + x;
  const uint32_t py = box.minimum.y + y;
  const uint32_t pz = box.minimum.z + z;

  return static_cast<size_t>(pz) * field.resolution.y * field.resolution.x
         + static_cast<size_t>(py) * field.resolution.x + px;
}

const PhysicsVerbs* g_physics = nullptr;
void* g_physicsHost = nullptr;

}

talus::field::Primitive primitiveFor(
  const World& world,
  Entity entity,
  const SdfEditDescriptor& edit
) {
  const talus::objectmodel::Pose& pose =
    world.registry().get<talus::objectmodel::Pose>(entity);

  const glm::mat4 toLocal = glm::inverse(talus::objectmodel::model(pose));

  const glm::vec3 position(
    edit.position[0],
    edit.position[1],
    edit.position[2]
  );
  const glm::vec3 local = glm::vec3(toLocal * glm::vec4(position, 1.0f));

  const glm::quat wanted(
    edit.rotation[3],
    edit.rotation[0],
    edit.rotation[1],
    edit.rotation[2]
  );

  const glm::quat turn = glm::normalize(glm::conjugate(pose.rotation) * wanted);

  const float angle = glm::angle(turn);
  const glm::vec3 axis =
    angle > 1e-6f ? glm::axis(turn) : glm::vec3(0.0f, 1.0f, 0.0f);

  switch (edit.kind) {
    case 1u:
      return talus::field::makeBox(
        local,
        glm::vec3(edit.size[0], edit.size[1], edit.size[2]),
        angle,
        axis,
        edit.material,
        edit.operation,
        edit.blend
      );

    case 2u:
      return talus::field::makePlane(local, edit.material, edit.operation);

    case 3u:
      return talus::field::makeCylinder(
        local,
        edit.size[0],
        edit.size[1],
        angle,
        axis,
        edit.material,
        edit.operation,
        edit.blend
      );

    default:
      return talus::field::makeSphere(
        local,
        edit.size[0],
        edit.material,
        edit.operation,
        edit.blend
      );
  }
}

namespace {

int32_t deformSdf(
  void* world,
  uint32_t entity,
  const SdfEditDescriptor* edit,
  SdfDeformSummary* result
) {
  World* target = worldOf(world);

  if (target == nullptr || edit == nullptr) {
    return 0;
  }

  const Entity subject = entityOf(entity);

  if (
    !target->hasSampledSdf(subject)
    || !target->registry().all_of<talus::objectmodel::Pose>(subject)
  ) {
    return 0;
  }

  const talus::field::Primitive shape = primitiveFor(*target, subject, *edit);

  std::optional<talus::objectmodel::SdfDeformOutcome> outcome;

  try {
    outcome = talus::objectmodel::deformSampledSdf(*target, subject, shape);
  } catch (const std::exception&) {
    return 0;
  }

  if (!outcome.has_value()) {
    return 0;
  }

  SdfDeformSummary filled{};
  filled.cellsRemoved = outcome->cellsRemoved;
  filled.volumeRemoved = outcome->volumeRemoved;

  for (uint32_t axis = 0u; axis < 3u; ++axis) {
    filled.brickBegin[axis] = outcome->write.brickBegin[axis];
    filled.brickEnd[axis] = outcome->write.brickEnd[axis];
    filled.sampleMinimum[axis] = outcome->write.samples.minimum[axis];
    filled.sampleExtent[axis] = outcome->write.samples.extent[axis];
  }

  if (result != nullptr) {
    *result = filled;
  }

  return 1;
}

uint32_t materialsUnderSdf(
  void* world,
  uint32_t entity,
  const SdfEditDescriptor* edit,
  uint32_t* materials,
  uint32_t capacity
) {
  World* target = worldOf(world);

  if (target == nullptr || edit == nullptr) {
    return 0u;
  }

  const Entity subject = entityOf(entity);
  const talus::field::SampledSdf* field = target->sampledSdf(subject);

  if (
    field == nullptr
    || !target->registry().all_of<talus::objectmodel::Pose>(subject)
  ) {
    return 0u;
  }

  const talus::field::Primitive shape = primitiveFor(*target, subject, *edit);

  talus::field::Primitive probe = shape;
  probe.shapeConfig.z = talus::core::FIELD_OP_UNION;

  const talus::field::SampleBox box =
    talus::field::affectedBox(*field, shape, 0u);

  std::vector<uint32_t> found;

  for (uint32_t z = 0u; z < box.extent.z; ++z) {
    for (uint32_t y = 0u; y < box.extent.y; ++y) {
      for (uint32_t x = 0u; x < box.extent.x; ++x) {
        const size_t index = indexOf(*field, box, x, y, z);

        if (field->distances[index] >= 0.0f) {
          continue;
        }

        const glm::vec3 at =
          glm::vec3(field->origin)
          + field->voxelSize * glm::vec3(box.minimum + glm::uvec3(x, y, z));

        if (talus::field::sample({&probe, 1}, at).distance > 0.0f) {
          continue;
        }

        const uint32_t material = field->materials[index];

        if (std::ranges::find(found, material) == found.end()) {
          found.push_back(material);
        }
      }
    }
  }

  for (uint32_t slot = 0u; slot < found.size() && slot < capacity; ++slot) {
    materials[slot] = found[slot];
  }

  return static_cast<uint32_t>(found.size());
}

int32_t removeSystem(void* schedule, const char* name) {
  auto* target = static_cast<talus::systems::Schedule*>(schedule);

  if (target == nullptr || name == nullptr) {
    return 0;
  }

  return target->remove(name) ? 1 : 0;
}

int32_t addSystem(
  void* schedule,
  uint32_t phase,
  int32_t order,
  const char* name,
  SystemCallback callback,
  void* user
) {
  auto* target = static_cast<talus::systems::Schedule*>(schedule);

  const std::optional<talus::systems::Phase> known =
    talus::systems::phaseFrom(phase);

  if (
    target == nullptr || name == nullptr || callback == nullptr
    || !known.has_value()
  ) {
    return 0;
  }

  const auto run = [callback, user, records = std::vector<ImpactRecord>()](
                     talus::systems::Context& context
                   ) mutable {
    records.resize(context.impacts.size());

    for (size_t index = 0u; index < records.size(); ++index) {
      const talus::objectmodel::Impact& impact = context.impacts[index];
      ImpactRecord& record = records[index];

      record.entity = static_cast<uint32_t>(impact.entity);
      record.other = static_cast<uint32_t>(impact.other);
      record.point[0] = impact.point.x;
      record.point[1] = impact.point.y;
      record.point[2] = impact.point.z;
      record.outward[0] = impact.outward.x;
      record.outward[1] = impact.outward.y;
      record.outward[2] = impact.outward.z;
      record.impulse = impact.impulse;
      record.speed = impact.speed;
      record.material = impact.material;
      record.otherMaterial = impact.otherMaterial;
    }

    const SystemContext handed{
      static_cast<uint32_t>(sizeof(SystemContext)),
      &context.world,
      const_cast<talus::systems::InputState*>(&context.input),
      records.data(),
      static_cast<uint32_t>(records.size()),
      context.deltaSeconds,
      context.frameIndex,
      context.debug,
      context.queries
    };

    callback(user, &handed);
  };

  return target->add(*known, order, name, run) ? 1 : 0;
}

uint32_t createSdfGrid(
  void* world,
  const float* pose,
  const uint32_t* resolution,
  float margin
) {
  World* target = worldOf(world);

  if (target == nullptr || pose == nullptr || resolution == nullptr) {
    return UINT32_MAX;
  }

  talus::objectmodel::Pose seat{};
  seat.position = glm::vec3(pose[0], pose[1], pose[2]);
  seat.rotation = glm::quat(pose[6], pose[3], pose[4], pose[5]);

  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(resolution[0], resolution[1], resolution[2]);

  if (margin > 0.0f) {
    grid.margin = margin;
  }

  try {
    return static_cast<uint32_t>(
      entt::to_integral(target->createSdfGrid(seat, grid))
    );
  } catch (const std::exception&) {
    return UINT32_MAX;
  }
}

int32_t destroy(void* world, uint32_t entity) {
  if (g_physics != nullptr && g_physics->destroy != nullptr) {
    return g_physics->destroy(g_physicsHost, world, entity);
  }

  World* target = worldOf(world);

  return target != nullptr && target->destroy(entityOf(entity)) ? 1 : 0;
}

int32_t addPrimitive(
  void* world,
  uint32_t entity,
  const SdfEditDescriptor* shape
) {
  World* target = worldOf(world);

  if (target == nullptr || shape == nullptr) {
    return 0;
  }

  const Entity subject = entityOf(entity);

  if (!target->registry().all_of<talus::objectmodel::Pose>(subject)) {
    return 0;
  }

  try {
    return target->addPrimitive(subject, primitiveFor(*target, subject, *shape))
             ? 1
             : 0;
  } catch (const std::exception&) {
    return 0;
  }
}

uint32_t mainCamera(void* world) {
  if (world == nullptr) {
    return UINT32_MAX;
  }

  const talus::objectmodel::Entity eye = talus::objectmodel::mainCamera(
    *static_cast<talus::objectmodel::World*>(world)
  );

  return eye == talus::objectmodel::INVALID_ENTITY
           ? UINT32_MAX
           : static_cast<uint32_t>(entt::to_integral(eye));
}

int32_t shareSdf(void* world, uint32_t donor, uint32_t taker) {
  World* target = worldOf(world);

  if (target == nullptr) {
    return 0;
  }

  try {
    target->shareSampledSdf(entityOf(donor), entityOf(taker));
  } catch (const std::exception&) {
    return 0;
  }

  return 1;
}

int32_t restart(void* world) {
  if (g_physics == nullptr || g_physics->restart == nullptr) {
    return 0;
  }

  return g_physics->restart(g_physicsHost, world);
}

int32_t material(void* world, uint32_t index, PhysicalMaterial* out) {
  World* target = worldOf(world);

  if (target == nullptr || out == nullptr) {
    return 0;
  }

  const talus::objectmodel::PhysicalMaterial& found = target->material(index);

  out->friction = found.friction;
  out->restitution = found.restitution;
  out->density = found.density;
  out->strength = found.strength;

  return index < target->materials().size() ? 1 : 0;
}

int32_t setRigidBody(void* world, uint32_t entity, float mass) {
  if (g_physics == nullptr || g_physics->setRigidBody == nullptr) {
    return 0;
  }

  return g_physics->setRigidBody(g_physicsHost, world, entity, mass);
}

int32_t setVelocity(void* world, uint32_t entity, const float* velocity) {
  if (
    g_physics == nullptr || g_physics->setVelocity == nullptr
    || velocity == nullptr
  ) {
    return 0;
  }

  return g_physics->setVelocity(g_physicsHost, world, entity, velocity);
}

int32_t screenPointToRay(
  void* world,
  uint32_t camera,
  const float* screen,
  const float* viewport,
  float* ray
) {
  World* target = worldOf(world);

  if (
    target == nullptr || screen == nullptr || viewport == nullptr
    || ray == nullptr
  ) {
    return 0;
  }

  const std::optional<talus::field::Ray> found =
    talus::objectmodel::screenPointToRay(
      *target,
      entityOf(camera),
      glm::vec2(screen[0], screen[1]),
      glm::vec2(viewport[0], viewport[1])
    );

  if (!found.has_value()) {
    return 0;
  }

  ray[0] = found->origin.x;
  ray[1] = found->origin.y;
  ray[2] = found->origin.z;
  ray[3] = found->direction.x;
  ray[4] = found->direction.y;
  ray[5] = found->direction.z;

  return 1;
}

const talus::systems::InputState* asInput(void* input) {
  return static_cast<const talus::systems::InputState*>(input);
}

int32_t keyHeld(void* input, uint32_t key) {
  return input != nullptr
             && asInput(input)->held(static_cast<talus::systems::Key>(key))
           ? 1
           : 0;
}

int32_t keyPressed(void* input, uint32_t key) {
  return input != nullptr
             && asInput(input)->pressed(static_cast<talus::systems::Key>(key))
           ? 1
           : 0;
}

int32_t keyReleased(void* input, uint32_t key) {
  return input != nullptr
             && asInput(input)->released(static_cast<talus::systems::Key>(key))
           ? 1
           : 0;
}

int32_t mouseHeld(void* input, uint32_t button) {
  return input != nullptr
             && asInput(input)->held(
               static_cast<talus::systems::MouseButton>(button)
             )
           ? 1
           : 0;
}

int32_t mousePressed(void* input, uint32_t button) {
  return input != nullptr
             && asInput(input)->pressed(
               static_cast<talus::systems::MouseButton>(button)
             )
           ? 1
           : 0;
}

void cursor(void* input, float* xy) {
  if (input == nullptr || xy == nullptr) {
    return;
  }

  xy[0] = asInput(input)->cursorX();
  xy[1] = asInput(input)->cursorY();
}

void viewport(void* input, float* wh) {
  if (input == nullptr || wh == nullptr) {
    return;
  }

  wh[0] = asInput(input)->viewportWidth();
  wh[1] = asInput(input)->viewportHeight();
}

int32_t drawLine(
  void* debug,
  const float* from,
  const float* to,
  const float* rgba,
  float seconds,
  const char* category,
  int32_t depthTested
) {
  auto* target = static_cast<talus::debugdraw::DebugDraw*>(debug);

  if (
    target == nullptr || from == nullptr || to == nullptr || rgba == nullptr
    || category == nullptr
  ) {
    return 0;
  }

  return target->line(
           glm::vec3(from[0], from[1], from[2]),
           glm::vec3(to[0], to[1], to[2]),
           glm::vec4(rgba[0], rgba[1], rgba[2], rgba[3]),
           seconds,
           category,
           depthTested != 0
         )
           ? 1
           : 0;
}

int32_t showDebug(void* debug, const char* category, int32_t shown) {
  auto* target = static_cast<talus::debugdraw::DebugDraw*>(debug);

  if (target == nullptr || category == nullptr) {
    return 0;
  }

  target->show(category, shown != 0);

  return 1;
}

int32_t raycast(
  const void* queries,
  const float* ray,
  float maxDistance,
  RaycastHit* result
) {
  if (result == nullptr) {
    return -1;
  }
  *result = {};
  result->entity = UINT32_MAX;
  if (queries == nullptr || ray == nullptr) {
    return -1;
  }
  const auto& context = *static_cast<const talus::query::Context*>(queries);
  try {
    const auto hit = talus::query::raycast(
      context,
      {glm::vec3(ray[0], ray[1], ray[2]), glm::vec3(ray[3], ray[4], ray[5])},
      maxDistance
    );
    if (!hit) {
      return 0;
    }
    result->entity = static_cast<uint32_t>(hit->entity);
    for (int axis = 0; axis < 3; ++axis) {
      result->position[axis] = hit->position[axis];
      result->normal[axis] = hit->normal[axis];
    }
    result->distance = hit->distance;
    result->material = hit->material;
    return 1;
  } catch (const std::exception&) {
    return -1;
  }
}

static_assert(std::is_standard_layout_v<RaycastHit>);
static_assert(sizeof(RaycastHit) == 36u);
static_assert(offsetof(RaycastHit, position) == 4u);
static_assert(offsetof(RaycastHit, normal) == 16u);
static_assert(offsetof(RaycastHit, distance) == 28u);
static_assert(offsetof(RaycastHit, material) == 32u);

static_assert(std::is_standard_layout_v<ImpactRecord>);
static_assert(sizeof(ImpactRecord) == 48u);
static_assert(offsetof(ImpactRecord, material) == 40u);
static_assert(offsetof(ImpactRecord, otherMaterial) == 44u);

static_assert(sizeof(PhysicalMaterial) == 16u);

constexpr Api TABLE{
  static_cast<uint32_t>(sizeof(Api)),
  API_VERSION,
  &declareComponent,
  &findComponent,
  &setComponent,
  &getComponent,
  &removeComponent,
  &componentCount,
  &componentEntities,
  &componentData,
  &deformSdf,
  &materialsUnderSdf,
  &sampleSdf,
  &getPose,
  &setPose,
  &entities,
  &hasComponent,
  &bounds,
  &logMessage,
  &addSystem,
  &keyHeld,
  &keyPressed,
  &keyReleased,
  &mouseHeld,
  &mousePressed,
  &cursor,
  &createSdfGrid,
  &destroy,
  &addPrimitive,
  &shareSdf,
  &mainCamera,
  &setRigidBody,
  &setVelocity,
  &screenPointToRay,
  &viewport,
  &drawLine,
  &showDebug,
  &raycast,
  &removeSystem,
  &restart,
  &material
};

}

void setPhysicsVerbs(const PhysicsVerbs* verbs, void* host) noexcept {
  g_physics = verbs;
  g_physicsHost = host;
}

PhysicsScope::PhysicsScope(const PhysicsVerbs* verbs, void* host) noexcept
    : m_previous(g_physics), m_previousHost(g_physicsHost), m_active(true) {
  setPhysicsVerbs(verbs, host);
}

PhysicsScope::PhysicsScope(PhysicsScope&& other) noexcept
    : m_previous(other.m_previous),
      m_previousHost(other.m_previousHost),
      m_active(other.m_active) {
  other.m_active = false;
}

PhysicsScope& PhysicsScope::operator=(PhysicsScope&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  restore();

  m_previous = other.m_previous;
  m_previousHost = other.m_previousHost;
  m_active = other.m_active;

  other.m_active = false;

  return *this;
}

PhysicsScope::~PhysicsScope() {
  restore();
}

void PhysicsScope::restore() noexcept {
  if (!m_active) {
    return;
  }

  setPhysicsVerbs(m_previous, m_previousHost);

  m_active = false;
}

void setLogSink(LogSink sink) noexcept {
  g_sink = sink;
}

const Api& api() noexcept {
  return TABLE;
}

}
