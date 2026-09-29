#pragma once

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <talus/field/crossings/crossings.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace talus::physics {

struct FieldSeed {
  glm::vec3 point;
  uint32_t brick;
  glm::vec3 normal;
};

struct SurfaceBrick {
  uint32_t brick = 0u;
  uint32_t begin = 0u;
  uint32_t end = 0u;
  glm::vec3 minimum{0.0f};
  glm::vec3 maximum{0.0f};
};

struct ReachCells {
  glm::uvec3 minimum{UINT32_MAX};
  glm::uvec3 maximum{0u};
};

struct SolidIntegral {
  double mass = 0.0;
  glm::dvec3 firstMoment{0.0};
  glm::dmat3 secondMoment{0.0};
};

struct SliceHulls {
  bool convex = false;
  std::vector<uint32_t> vertices;
};

SliceHulls sliceHulls(
  const talus::field::SampledSdf& field,
  const talus::field::LatticeSurface& surface
);

JPH::RefConst<JPH::ConvexHullShape> hullThrough(
  const talus::field::SampledSdf& field,
  const talus::field::LatticeSurface& surface,
  std::span<const uint32_t> vertices
);

bool hullAgrees(
  const talus::field::SampledSdf& field,
  std::span<const talus::field::Crossing> crossings,
  const JPH::ConvexHullShape& hull
);

struct FieldHull {
  JPH::RefConst<JPH::ConvexHullShape> shape;
  size_t crossings = 0u;
  size_t inside = 0u;
  size_t points = 0u;
  bool built = false;
};

FieldHull convexHullOf(const talus::field::SampledSdf& field);

JPH::RefConst<JPH::ConvexHullShape> displacedHull(
  const talus::field::SampledSdf& field,
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::vec3> shape
);

struct HullAxis {
  JPH::Vec3 axis = JPH::Vec3::sZero();
  float overlap = 0.0f;
  size_t candidates = 0u;
};

[[nodiscard]] HullAxis hullSeparatingAxis(
  const JPH::ConvexHullShape& first,
  JPH::Mat44Arg firstAt,
  const JPH::ConvexHullShape& second,
  JPH::Mat44Arg secondAt,
  JPH::Vec3Arg reported
);

class FieldShape final : public JPH::Shape {
public:
  explicit FieldShape(const talus::field::SampledSdf& field);

  FieldShape(
    const FieldShape& authored,
    JPH::RefConst<JPH::ConvexHullShape> hull
  );

  FieldShape(
    const talus::field::SampledSdf& field,
    const FieldShape& previous,
    const glm::uvec3& changedBegin,
    const glm::uvec3& changedEnd
  );

  const talus::field::SampledSdf& field() const noexcept;

  const glm::vec3& centerOfMass() const noexcept;

  bool uniformMaterial() const noexcept;

  const JPH::ConvexHullShape* hull() const noexcept;

  std::span<const FieldSeed> seeds() const noexcept;

  std::span<const SurfaceBrick> surfaceBricks() const noexcept;

  bool beyondReach(JPH::Vec3Arg point, float separation) const noexcept;

  JPH::AABox GetLocalBounds() const override;
  JPH::Vec3 GetCenterOfMass() const override;
  JPH::uint GetSubShapeIDBitsRecursive() const override;
  float GetInnerRadius() const override;
  JPH::MassProperties GetMassProperties() const override;

  const JPH::PhysicsMaterial* GetMaterial(
    const JPH::SubShapeID& subShapeID
  ) const override;

  JPH::Vec3 GetSurfaceNormal(
    const JPH::SubShapeID& subShapeID,
    JPH::Vec3Arg localSurfacePosition
  ) const override;

  void GetSubmergedVolume(
    JPH::Mat44Arg centerOfMassTransform,
    JPH::Vec3Arg scale,
    const JPH::Plane& surface,
    float& totalVolume,
    float& submergedVolume,
    JPH::Vec3& centerOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
    ,
    JPH::RVec3Arg baseOffset
#endif
  ) const override;

#ifdef JPH_DEBUG_RENDERER
  void Draw(
    JPH::DebugRenderer* renderer,
    JPH::RMat44Arg centerOfMassTransform,
    JPH::Vec3Arg scale,
    JPH::ColorArg color,
    bool useMaterialColors,
    bool drawWireframe
  ) const override;
#endif

  bool CastRay(
    const JPH::RayCast& ray,
    const JPH::SubShapeIDCreator& subShapeIDCreator,
    JPH::RayCastResult& hit
  ) const override;

  void CastRay(
    const JPH::RayCast& ray,
    const JPH::RayCastSettings& rayCastSettings,
    const JPH::SubShapeIDCreator& subShapeIDCreator,
    JPH::CastRayCollector& collector,
    const JPH::ShapeFilter& shapeFilter
  ) const override;

  void CollidePoint(
    JPH::Vec3Arg point,
    const JPH::SubShapeIDCreator& subShapeIDCreator,
    JPH::CollidePointCollector& collector,
    const JPH::ShapeFilter& shapeFilter
  ) const override;

  void CollideSoftBodyVertices(
    JPH::Mat44Arg centerOfMassTransform,
    JPH::Vec3Arg scale,
    const JPH::CollideSoftBodyVertexIterator& vertices,
    JPH::uint numVertices,
    int collidingShapeIndex
  ) const override;

  void GetTrianglesStart(
    GetTrianglesContext& context,
    const JPH::AABox& box,
    JPH::Vec3Arg positionCOM,
    JPH::QuatArg rotation,
    JPH::Vec3Arg scale
  ) const override;

  int GetTrianglesNext(
    GetTrianglesContext& context,
    int maxTrianglesRequested,
    JPH::Float3* triangleVertices,
    const JPH::PhysicsMaterial** materials
  ) const override;

  Stats GetStats() const override;
  float GetVolume() const override;

private:
  FieldShape(
    const talus::field::SampledSdf& field,
    const FieldShape* previous,
    const glm::uvec3& changedBegin,
    const glm::uvec3& changedEnd,
    JPH::RefConst<JPH::ConvexHullShape> hull
  );

  const talus::field::SampledSdf* m_field;

  std::vector<SolidIntegral> m_brickIntegral;
  std::vector<uint32_t> m_brickSlot;

  std::vector<SurfaceBrick> m_surfaceBricks;
  std::vector<FieldSeed> m_seeds;

  std::vector<ReachCells> m_brickReach;

  JPH::RefConst<JPH::ConvexHullShape> m_hull;

  glm::vec3 m_centerOfMass{0.0f};
  bool m_uniformMaterial = false;
  uint8_t m_material = 0u;
  JPH::MassProperties m_massProperties;

  JPH::AABox m_bounds;
  JPH::AABox m_grid;
  JPH::AABox m_reach;
  float m_reachDistance = 0.0f;
  float m_innerRadius = 0.0f;
  JPH::uint m_subShapeBits = 0u;
};

void registerFieldShape();

}
