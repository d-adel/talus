#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ContactListener.h>

#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/impact/impact.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

namespace talus::physics {

class ImpactListener : public JPH::ContactListener {
public:
  explicit ImpactListener(float threshold, uint32_t bodies);

  void OnContactAdded(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    JPH::ContactSettings& settings
  ) override;

  void OnContactPersisted(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    JPH::ContactSettings& settings
  ) override;

  void drain(std::vector<talus::objectmodel::Impact>& out);

  void beginStep() noexcept;

  [[nodiscard]] bool contacted(JPH::BodyID body) const noexcept;

  [[nodiscard]] bool contactedMoving(JPH::BodyID body) const noexcept;

  void setThreshold(float speed) noexcept;
  [[nodiscard]] float threshold() const noexcept;

  void setMaterials(
    std::span<const talus::objectmodel::PhysicalMaterial> materials
  ) noexcept;

  [[nodiscard]] static uint32_t materialAt(
    const JPH::Body& body,
    JPH::RVec3Arg worldPoint
  );

private:
  struct Contact {
    uint32_t point1 = 0u;
    uint32_t point2 = 0u;
    uint32_t material1 = 0u;
    uint32_t material2 = 0u;
  };

  Contact combine(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    JPH::ContactSettings& settings,
    bool strongest
  ) const;

  [[nodiscard]] static float estimateImpulse(
    const JPH::Body& body1,
    const JPH::Body& body2,
    const JPH::ContactManifold& manifold,
    const JPH::ContactSettings& settings
  );

  [[nodiscard]] const talus::objectmodel::PhysicalMaterial& material(
    uint32_t index
  ) const noexcept;

  void touch(const JPH::Body& body1, const JPH::Body& body2) noexcept;

  std::mutex m_mutex;
  std::vector<talus::objectmodel::Impact> m_impacts;
  std::atomic<float> m_threshold;
  std::span<const talus::objectmodel::PhysicalMaterial> m_materials;
  std::vector<std::atomic<uint32_t>> m_touched;
  std::vector<std::atomic<uint32_t>> m_touchedMoving;
  uint32_t m_step = 0u;
};

}
