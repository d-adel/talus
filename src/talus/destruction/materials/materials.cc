#include "materials.h"

#include <cmath>
#include <stdexcept>

namespace talus::destruction {

namespace {

ModeLimits modes(double compression, double tension, double shear) {
  return ModeLimits{compression, tension, shear};
}

bool positive(const ModeLimits& limits) {
  return limits.compression > 0.0 && limits.tension > 0.0 && limits.shear > 0.0
         && std::isfinite(limits.compression) && std::isfinite(limits.tension)
         && std::isfinite(limits.shear);
}

bool ordered(const ModeLimits& elastic, const ModeLimits& fatal) {
  return elastic.compression <= fatal.compression && elastic.tension <= fatal.tension
         && elastic.shear <= fatal.shear;
}

bool positiveFinite(double value) {
  return value > 0.0 && std::isfinite(value);
}

}

StructuralMaterial concrete() {
  StructuralMaterial material;

  material.density = 2450.0;
  material.elastic = modes(33.0e6, 2.6e6, 0.45 * 2.6e6);
  material.fatal = material.elastic;
  material.shearRule = ShearRule::Coulomb;
  material.friction = 0.7;
  material.variation = 0.30;
  material.fracture = Fracture::Blocks;
  material.pieces = 2u;

  return material;
}

StructuralMaterial reinforcedConcrete(double ratio) {
  const StructuralMaterial bars = steel();
  StructuralMaterial material = concrete();

  material.density = 2550.0;
  material.elastic.tension = ratio * bars.elastic.tension;
  material.fatal.tension = ratio * bars.fatal.tension;

  return material;
}

StructuralMaterial brickMasonry() {
  StructuralMaterial material;

  material.density = 1850.0;
  material.elastic = modes(8.0e6, 0.25e6, 0.35e6);
  material.fatal = material.elastic;
  material.shearRule = ShearRule::Coulomb;
  material.friction = 0.7;
  material.anisotropy = Anisotropy::Courses;
  material.headJoints = 2.0;
  material.variation = 0.30;
  material.fracture = Fracture::Blocks;
  material.pieces = 2u;

  return material;
}

StructuralMaterial timber() {
  StructuralMaterial material;

  material.density = 420.0;
  material.elastic = modes(25.0e6, 22.0e6, 4.0e6);
  material.fatal = material.elastic;
  material.acrossElastic = modes(2.5e6, 0.4e6, 0.8e6);
  material.acrossFatal = material.acrossElastic;
  material.friction = 0.4;
  material.anisotropy = Anisotropy::Grain;
  material.variation = 0.25;
  material.fracture = Fracture::Splinters;
  material.pieces = 3u;

  return material;
}

StructuralMaterial glass() {
  StructuralMaterial material;

  material.density = 2500.0;
  material.elastic = modes(500.0e6, 45.0e6, 45.0e6);
  material.fatal = material.elastic;
  material.friction = 0.5;
  material.variation = 0.30;
  material.chunkSize = 0.25;
  material.fracture = Fracture::Tiles;
  material.pieces = 2u;

  return material;
}

StructuralMaterial steel() {
  StructuralMaterial material;

  material.density = 7850.0;
  material.elastic = modes(380.0e6, 380.0e6, 380.0e6 / std::sqrt(3.0));
  material.fatal = modes(560.0e6, 560.0e6, 560.0e6 / std::sqrt(3.0));
  material.friction = 0.2;
  material.variation = 0.07;

  return material;
}

void validate(const StructuralMaterial& material) {
  if (!positiveFinite(material.density) || !positiveFinite(material.chunkSize)) {
    throw std::invalid_argument("A structural material needs a positive density and chunk size");
  }

  if (!positive(material.elastic) || !positive(material.fatal)
      || !ordered(material.elastic, material.fatal)) {
    throw std::invalid_argument(
      "A structural material needs positive limits, each fatal limit at or above its elastic limit"
    );
  }

  if (material.anisotropy == Anisotropy::Grain
      && (!positive(material.acrossElastic) || !positive(material.acrossFatal)
          || !ordered(material.acrossElastic, material.acrossFatal))) {
    throw std::invalid_argument("A grained material needs positive limits across its grain");
  }

  if (!(material.friction >= 0.0) || !std::isfinite(material.friction)
      || !positiveFinite(material.headJoints)) {
    throw std::invalid_argument("A structural material needs a friction of at least zero and a positive head joint factor");
  }

  if (!(material.variation >= 0.0 && material.variation <= 0.5)) {
    throw std::invalid_argument("A structural material's coefficient of variation lies between 0 and 0.5");
  }

  if (material.fracture == Fracture::None ? material.pieces != 1u : (material.pieces < 2u || material.pieces > 8u)) {
    throw std::invalid_argument("A structural material's finer level needs 2 to 8 pieces per axis, and 1 without a fracture kind");
  }
}

}
