#pragma once

#include <cstdint>

#include <talus/field/field.h>
#include <talus/objectmodel/entity/entity.h>

namespace talus::objectmodel {
class World;
}

namespace talus::script {

extern "C" {

inline constexpr uint32_t API_VERSION = 16u;

struct FieldDescriptor {
  const char* name;
  uint32_t kind;
  uint32_t offset;
};

struct SdfEditDescriptor {
  uint32_t kind;
  uint32_t material;
  uint32_t operation;
  float blend;
  float position[3];
  float size[3];
  float rotation[4];
};

struct SdfDeformSummary {
  uint32_t cellsRemoved;
  float volumeRemoved;
  uint32_t brickBegin[3];
  uint32_t brickEnd[3];
  uint32_t sampleMinimum[3];
  uint32_t sampleExtent[3];
};

struct PhysicsVerbs {
  int32_t (*setRigidBody)(void* host, void* world, uint32_t entity, float mass);

  int32_t (*setVelocity)(
    void* host,
    void* world,
    uint32_t entity,
    const float* velocity
  );

  int32_t (*destroy)(void* host, void* world, uint32_t entity);

  int32_t (*restart)(void* host, void* world);
};

struct ImpactRecord {
  uint32_t entity;
  uint32_t other;
  float point[3];
  float outward[3];
  float impulse;
  float speed;
  uint32_t material;
  uint32_t otherMaterial;
};

struct RaycastHit {
  uint32_t entity;
  float position[3];
  float normal[3];
  float distance;
  uint32_t material;
};

struct PhysicalMaterial {
  float friction;
  float restitution;
  float density;
  float strength;
};

struct SystemContext {
  uint32_t size;
  void* world;
  void* input;
  const ImpactRecord* impacts;
  uint32_t impactCount;
  float deltaSeconds;
  uint32_t frame;
  void* debug;
  const void* queries;
};

using SystemCallback = void (*)(void* user, const SystemContext* context);

struct Api {
  uint32_t size;
  uint32_t version;

  uint32_t (*declareComponent)(
    void* world,
    const char* name,
    uint32_t size,
    const FieldDescriptor* fields,
    uint32_t fieldCount,
    const char* const* requirements,
    uint32_t requirementCount
  );

  uint32_t (*findComponent)(void* world, const char* name);

  int32_t (*setComponent)(
    void* world,
    uint32_t type,
    uint32_t entity,
    const void* value
  );

  void* (*getComponent)(void* world, uint32_t type, uint32_t entity);

  int32_t (*removeComponent)(void* world, uint32_t type, uint32_t entity);

  uint32_t (*componentCount)(void* world, uint32_t type);

  const uint32_t* (*componentEntities)(void* world, uint32_t type);

  void* (*componentData)(void* world, uint32_t type);

  int32_t (*deformSdf)(
    void* world,
    uint32_t entity,
    const SdfEditDescriptor* edit,
    SdfDeformSummary* result
  );

  uint32_t (*materialsUnderSdf)(
    void* world,
    uint32_t entity,
    const SdfEditDescriptor* edit,
    uint32_t* materials,
    uint32_t capacity
  );

  float (*sampleSdf)(void* world, uint32_t entity, const float* point);

  int32_t (*getPose)(void* world, uint32_t entity, float* pose);

  int32_t (*setPose)(void* world, uint32_t entity, const float* pose);

  uint32_t (*entities)(void* world, uint32_t* buffer, uint32_t capacity);

  int32_t (*hasComponent)(void* world, uint32_t entity, const char* name);

  int32_t (*bounds)(
    void* world,
    uint32_t entity,
    float* minimum,
    float* maximum
  );

  void (*log)(const char* message);

  int32_t (*addSystem)(
    void* schedule,
    uint32_t phase,
    int32_t order,
    const char* name,
    SystemCallback callback,
    void* user
  );

  int32_t (*keyHeld)(void* input, uint32_t key);

  int32_t (*keyPressed)(void* input, uint32_t key);

  int32_t (*keyReleased)(void* input, uint32_t key);

  int32_t (*mouseHeld)(void* input, uint32_t button);

  int32_t (*mousePressed)(void* input, uint32_t button);

  void (*cursor)(void* input, float* xy);

  uint32_t (*createSdfGrid)(
    void* world,
    const float* pose,
    const uint32_t* resolution,
    float margin
  );

  int32_t (*destroy)(void* world, uint32_t entity);

  int32_t (*addPrimitive)(
    void* world,
    uint32_t entity,
    const SdfEditDescriptor* shape
  );

  int32_t (*shareSdf)(void* world, uint32_t donor, uint32_t taker);

  uint32_t (*mainCamera)(void* world);

  int32_t (*setRigidBody)(void* world, uint32_t entity, float mass);

  int32_t (*setVelocity)(void* world, uint32_t entity, const float* velocity);

  int32_t (*screenPointToRay)(
    void* world,
    uint32_t camera,
    const float* screen,
    const float* viewport,
    float* ray
  );

  void (*viewport)(void* input, float* wh);

  int32_t (*drawLine)(
    void* debug,
    const float* from,
    const float* to,
    const float* rgba,
    float seconds,
    const char* category,
    int32_t depthTested
  );

  int32_t (*showDebug)(void* debug, const char* category, int32_t shown);

  int32_t (*raycast)(
    const void* queries,
    const float* ray,
    float maxDistance,
    RaycastHit* hit
  );

  int32_t (*removeSystem)(void* schedule, const char* name);

  int32_t (*restart)(void* world);

  int32_t (*material)(void* world, uint32_t index, PhysicalMaterial* out);
};
}

using LogSink = void (*)(const char* message);

void setLogSink(LogSink sink) noexcept;

void setPhysicsVerbs(const PhysicsVerbs* verbs, void* host) noexcept;

class PhysicsScope {
public:
  PhysicsScope() noexcept = default;

  PhysicsScope(const PhysicsVerbs* verbs, void* host) noexcept;

  PhysicsScope(const PhysicsScope&) = delete;
  PhysicsScope& operator=(const PhysicsScope&) = delete;

  PhysicsScope(PhysicsScope&& other) noexcept;
  PhysicsScope& operator=(PhysicsScope&& other) noexcept;

  ~PhysicsScope();

private:
  void restore() noexcept;

  const PhysicsVerbs* m_previous = nullptr;
  void* m_previousHost = nullptr;
  bool m_active = false;
};

[[nodiscard]] const Api& api() noexcept;

[[nodiscard]] talus::field::Primitive primitiveFor(
  const talus::objectmodel::World& world,
  talus::objectmodel::Entity entity,
  const SdfEditDescriptor& edit
);

}
