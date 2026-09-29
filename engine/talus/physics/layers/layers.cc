#include "layers.ih"

namespace talus::physics {

bool ObjectLayerPairFilter::ShouldCollide(
  JPH::ObjectLayer layer1,
  JPH::ObjectLayer layer2
) const {
  if (layer1 == ObjectLayers::NON_MOVING) {
    return layer2 == ObjectLayers::MOVING;
  }

  if (layer1 == ObjectLayers::MOVING) {
    return true;
  }

  JPH_ASSERT(false);
  return false;
}

BroadPhaseLayerInterface::BroadPhaseLayerInterface() {
  m_objectToBroadPhase[ObjectLayers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;

  m_objectToBroadPhase[ObjectLayers::MOVING] = BroadPhaseLayers::MOVING;
}

JPH::uint BroadPhaseLayerInterface::GetNumBroadPhaseLayers() const {
  return BroadPhaseLayers::NUM_LAYERS;
}

JPH::BroadPhaseLayer BroadPhaseLayerInterface::GetBroadPhaseLayer(
  JPH::ObjectLayer layer
) const {
  JPH_ASSERT(layer < ObjectLayers::NUM_LAYERS);

  return m_objectToBroadPhase[layer];
}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)

const char* BroadPhaseLayerInterface::GetBroadPhaseLayerName(
  JPH::BroadPhaseLayer layer
) const {
  if (layer == BroadPhaseLayers::NON_MOVING) {
    return "NON_MOVING";
  }

  if (layer == BroadPhaseLayers::MOVING) {
    return "MOVING";
  }

  JPH_ASSERT(false);
  return "INVALID";
}

#endif

bool ObjectVsBroadPhaseLayerFilter::ShouldCollide(
  JPH::ObjectLayer objectLayer,
  JPH::BroadPhaseLayer broadPhaseLayer
) const {
  if (objectLayer == ObjectLayers::NON_MOVING) {
    return broadPhaseLayer == BroadPhaseLayers::MOVING;
  }

  if (objectLayer == ObjectLayers::MOVING) {
    return true;
  }

  JPH_ASSERT(false);
  return false;
}

}
