#include "sdfdeform.ih"

namespace talus::objectmodel {

std::optional<SdfDeformOutcome> deformSampledSdf(
  World& world,
  Entity entity,
  const talus::field::Primitive& localPrimitive
) {
  if (
    !world.hasSampledSdf(entity)
    || !world.registry().any_of<DeformableBody, RigidBody, StaticBody>(entity)
  ) {
    return std::nullopt;
  }

  SdfDeformOutcome outcome{};

  world.patchSampledSdf(entity, [&](talus::field::SampledSdf& field) {
    const talus::field::DeformReport report =
      talus::field::deformAndRepair(field, localPrimitive);

    const float cellVolume =
      field.voxelSize.x * field.voxelSize.y * field.voxelSize.z;

    outcome.write = report.write;
    outcome.cellsRemoved = report.cellsRemoved;
    outcome.volumeRemoved =
      static_cast<float>(report.cellsRemoved) * cellVolume;
  });

  world.markSdfDirty(entity, outcome.write.samples);

  return outcome;
}

}
