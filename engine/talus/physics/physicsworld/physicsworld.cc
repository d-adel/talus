#include "physicsworld.ih"

namespace {

constexpr uint MAX_BODIES = 262144;
constexpr uint NUM_BODY_MUTEXES = 0;
constexpr uint MAX_BODY_PAIRS = 262144;
constexpr uint MAX_CONTACT_CONSTRAINTS = 262144;

constexpr uint NUM_VELOCITY_STEPS = 40;

constexpr uint TEMP_ALLOCATOR_SIZE = 256 * 1024 * 1024;

constexpr float IMPACT_THRESHOLD = 3.0f;

uint workerCount() {
  const unsigned int count = std::thread::hardware_concurrency();
  return count > 1 ? count - 1 : 1;
}

}

namespace talus::physics {

void ActivationListener::OnBodyActivated(const BodyID& body, uint64) {
  const std::lock_guard<std::mutex> guard(m_mutex);

  m_activated.push_back(body.GetIndexAndSequenceNumber());
}

void ActivationListener::OnBodyDeactivated(const BodyID&, uint64) {}

void ActivationListener::drain(std::vector<uint32_t>& out) {
  const std::lock_guard<std::mutex> guard(m_mutex);

  out.clear();
  out.swap(m_activated);
}

PhysicsWorld::PhysicsWorld()
    : m_tempAllocator(TEMP_ALLOCATOR_SIZE),
      m_jobSystem(cMaxPhysicsJobs, cMaxPhysicsBarriers, workerCount()),
      m_impacts(IMPACT_THRESHOLD, MAX_BODIES) {
  m_system.Init(
    MAX_BODIES,
    NUM_BODY_MUTEXES,
    MAX_BODY_PAIRS,
    MAX_CONTACT_CONSTRAINTS,
    m_broadPhaseLayerInterface,
    m_objectVsBroadPhaseLayerFilter,
    m_objectLayerPairFilter
  );

  PhysicsSettings settings = m_system.GetPhysicsSettings();

  settings.mNumVelocitySteps = NUM_VELOCITY_STEPS;

  m_system.SetPhysicsSettings(settings);

  m_system.SetGravity(JPH::Vec3(0.0f, -GRAVITY, 0.0f));

  m_system.SetContactListener(&m_impacts);

  m_system.SetBodyActivationListener(&m_activations);
}

PhysicsWorld::~PhysicsWorld() = default;

void PhysicsWorld::step() {
  m_impacts.beginStep();

  const JPH::EPhysicsUpdateError error =
    m_system.Update(TIME_STEP, 1, &m_tempAllocator, &m_jobSystem);

  if (error != JPH::EPhysicsUpdateError::None && !m_updateErrorReported) {
    m_updateErrorReported = true;

    std::cout << "Physics update error, bitmask "
              << static_cast<uint32_t>(error)
              << " (1 = manifold cache full, 2 = body pair cache full, "
                 "4 = contact constraints full)\n";
  }
}

void PhysicsWorld::optimizeBroadPhase() {
  m_system.OptimizeBroadPhase();

  ++m_broadPhaseOptimizations;
}

uint32_t PhysicsWorld::broadPhaseOptimizations() const noexcept {
  return m_broadPhaseOptimizations;
}

BodyInterface& PhysicsWorld::bodies() {
  return m_system.GetBodyInterface();
}

const BodyInterface& PhysicsWorld::bodies() const {
  return m_system.GetBodyInterface();
}

PhysicsSystem& PhysicsWorld::system() {
  return m_system;
}

const PhysicsSystem& PhysicsWorld::system() const {
  return m_system;
}

ImpactListener& PhysicsWorld::impacts() noexcept {
  return m_impacts;
}

ActivationListener& PhysicsWorld::activations() noexcept {
  return m_activations;
}

}
