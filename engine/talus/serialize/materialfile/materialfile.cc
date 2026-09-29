#include "materialfile.ih"

namespace talus::serialize {

std::string writeMaterial(const StoredMaterial& stored) {
  std::string text;

  if (glz::write<glz::opts{.prettify = true}>(stored, text)) {
    return {};
  }

  return text;
}

bool readMaterial(std::string_view text, StoredMaterial& stored) {
  StoredMaterial read{};

  if (glz::read<glz::opts{.error_on_unknown_keys = false}>(read, text)) {
    return false;
  }

  if (read.version > MATERIAL_VERSION) {
    return false;
  }

  stored = read;

  return true;
}

objectmodel::PhysicalMaterial physicalMaterial(const StoredMaterial& stored) {
  objectmodel::PhysicalMaterial physical{};

  physical.friction = stored.friction;
  physical.restitution = stored.restitution;
  physical.density = stored.density;
  physical.strength = stored.strength.value_or(
    std::numeric_limits<float>::infinity()
  );
  physical.youngsModulus = stored.youngsModulus.value_or(0.0f);
  physical.poissonsRatio = stored.poissonsRatio.value_or(0.3f);
  physical.stiffnessDamping = stored.stiffnessDamping.value_or(0.0f);
  physical.yieldStress = stored.yieldStress.value_or(
    std::numeric_limits<float>::infinity()
  );
  physical.hardeningModulus = stored.hardeningModulus.value_or(0.0f);

  return physical;
}

void setPhysicalMaterial(
  StoredMaterial& stored,
  const objectmodel::PhysicalMaterial& physical
) {
  stored.friction = physical.friction;
  stored.restitution = physical.restitution;
  stored.density = physical.density;

  stored.strength = std::isinf(physical.strength)
    ? std::nullopt
    : std::optional<float>(physical.strength);

  stored.youngsModulus = physical.youngsModulus > 0.0f
    ? std::optional<float>(physical.youngsModulus)
    : std::nullopt;

  stored.poissonsRatio = physical.poissonsRatio == 0.3f
    ? std::nullopt
    : std::optional<float>(physical.poissonsRatio);

  stored.stiffnessDamping = physical.stiffnessDamping > 0.0f
    ? std::optional<float>(physical.stiffnessDamping)
    : std::nullopt;

  stored.yieldStress = std::isinf(physical.yieldStress)
    ? std::nullopt
    : std::optional<float>(physical.yieldStress);

  stored.hardeningModulus = physical.hardeningModulus > 0.0f
    ? std::optional<float>(physical.hardeningModulus)
    : std::nullopt;
}

}
