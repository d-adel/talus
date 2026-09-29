#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>

#include <talus/physics/impact/impact.h>
#include <talus/physics/layers/layers.h>

#include <cstdint>
#include <mutex>
#include <vector>

namespace talus::physics {

class ActivationListener final : public JPH::BodyActivationListener {
public:
  void OnBodyActivated(const JPH::BodyID& body, JPH::uint64) override;

  void OnBodyDeactivated(const JPH::BodyID&, JPH::uint64) override;

  void drain(std::vector<uint32_t>& out);

private:
  std::mutex m_mutex;
  std::vector<uint32_t> m_activated;
};

class PhysicsWorld {
public:
  static constexpr float TIME_STEP = 1.0f / 60.0f;

  static constexpr float GRAVITY = 9.81f;

  PhysicsWorld();
  ~PhysicsWorld();

  PhysicsWorld(const PhysicsWorld&) = delete;
  PhysicsWorld& operator=(const PhysicsWorld&) = delete;
  PhysicsWorld(PhysicsWorld&&) = delete;
  PhysicsWorld& operator=(PhysicsWorld&&) = delete;

  void step();
  void optimizeBroadPhase();

  [[nodiscard]] uint32_t broadPhaseOptimizations() const noexcept;

  JPH::BodyInterface& bodies();
  const JPH::BodyInterface& bodies() const;

  JPH::PhysicsSystem& system();
  const JPH::PhysicsSystem& system() const;

  ImpactListener& impacts() noexcept;

  ActivationListener& activations() noexcept;

private:
  BroadPhaseLayerInterface m_broadPhaseLayerInterface;
  ObjectVsBroadPhaseLayerFilter m_objectVsBroadPhaseLayerFilter;
  ObjectLayerPairFilter m_objectLayerPairFilter;

  JPH::TempAllocatorImpl m_tempAllocator;
  JPH::JobSystemThreadPool m_jobSystem;

  ImpactListener m_impacts;

  ActivationListener m_activations;

  bool m_updateErrorReported = false;

  uint32_t m_broadPhaseOptimizations = 0u;

  JPH::PhysicsSystem m_system;
};

}
