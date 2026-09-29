#pragma once

#include <cstdint>

namespace talus::field {

enum class ShapeKind : uint32_t {
  Sphere = TALUS_SHAPE_SPHERE,
  Box = TALUS_SHAPE_BOX,
  Plane = TALUS_SHAPE_PLANE,
  Cylinder = TALUS_SHAPE_CYLINDER
};

enum class CsgOperation : uint32_t {
  Union = TALUS_CSG_UNION,
  SmoothUnion = TALUS_CSG_SMOOTH_UNION,
  Intersection = TALUS_CSG_INTERSECTION,
  Subtraction = TALUS_CSG_SUBTRACTION,
  SmoothSubtraction = TALUS_CSG_SMOOTH_SUBTRACTION
};

[[nodiscard]] constexpr ShapeKind shapeKindOf(uint32_t stored) noexcept {
  return static_cast<ShapeKind>(stored);
}

[[nodiscard]] constexpr CsgOperation operationOf(uint32_t stored) noexcept {
  return static_cast<CsgOperation>(stored);
}

[[nodiscard]] constexpr bool removesMaterial(CsgOperation operation) noexcept {
  switch (operation) {
    case CsgOperation::Subtraction:
    case CsgOperation::SmoothSubtraction:
      return true;
    case CsgOperation::Union:
    case CsgOperation::SmoothUnion:
    case CsgOperation::Intersection:
      return false;
  }

  return false;
}

[[nodiscard]] constexpr bool addsMaterial(CsgOperation operation) noexcept {
  switch (operation) {
    case CsgOperation::Union:
    case CsgOperation::SmoothUnion:
      return true;
    case CsgOperation::Intersection:
    case CsgOperation::Subtraction:
    case CsgOperation::SmoothSubtraction:
      return false;
  }

  return false;
}

}
