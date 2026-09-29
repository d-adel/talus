#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

#include <array>

namespace talus::physics {

namespace ObjectLayers {

inline constexpr JPH::ObjectLayer NON_MOVING = 0;
inline constexpr JPH::ObjectLayer MOVING = 1;
inline constexpr JPH::ObjectLayer NUM_LAYERS = 2;

}

namespace BroadPhaseLayers {

inline constexpr JPH::BroadPhaseLayer NON_MOVING{0};
inline constexpr JPH::BroadPhaseLayer MOVING{1};
inline constexpr JPH::uint NUM_LAYERS = 2;

}

class ObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
public:
  bool ShouldCollide(
    JPH::ObjectLayer layer1,
    JPH::ObjectLayer layer2
  ) const override;
};

class BroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface {
public:
  BroadPhaseLayerInterface();

  JPH::uint GetNumBroadPhaseLayers() const override;

  JPH::BroadPhaseLayer GetBroadPhaseLayer(
    JPH::ObjectLayer layer
  ) const override;

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override;
#endif

private:
  std::array<JPH::BroadPhaseLayer, ObjectLayers::NUM_LAYERS>
    m_objectToBroadPhase;
};

class ObjectVsBroadPhaseLayerFilter final
    : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
  bool ShouldCollide(
    JPH::ObjectLayer objectLayer,
    JPH::BroadPhaseLayer broadPhaseLayer
  ) const override;
};

}
