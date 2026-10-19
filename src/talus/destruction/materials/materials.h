#pragma once

#include <cstdint>

namespace talus::destruction {

struct ModeLimits {
  double compression = 0.0;
  double tension = 0.0;
  double shear = 0.0;
};

enum class ShearRule : uint8_t { Fixed, Coulomb };

enum class Anisotropy : uint8_t { None, Grain, Courses };

enum class Fracture : uint8_t { None, Blocks, Splinters, Tiles };

struct StructuralMaterial {
  double density = 0.0;
  ModeLimits elastic;
  ModeLimits fatal;
  ModeLimits acrossElastic;
  ModeLimits acrossFatal;
  ShearRule shearRule = ShearRule::Fixed;
  double friction = 0.0;
  Anisotropy anisotropy = Anisotropy::None;
  double headJoints = 1.0;
  double variation = 0.0;
  double chunkSize = 0.5;
  Fracture fracture = Fracture::None;
  uint32_t pieces = 1u;
};

[[nodiscard]] StructuralMaterial concrete();
[[nodiscard]] StructuralMaterial reinforcedConcrete(double ratio = 0.01);
[[nodiscard]] StructuralMaterial brickMasonry();
[[nodiscard]] StructuralMaterial timber();
[[nodiscard]] StructuralMaterial glass();
[[nodiscard]] StructuralMaterial steel();

void validate(const StructuralMaterial& material);

}
