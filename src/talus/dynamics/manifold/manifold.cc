#include "manifold.h"

#include <talus/dynamics/manifold/boxhull.h>

#include <cmath>
#include <limits>

namespace talus::dynamics {

Manifold collideSpheres(
  const glm::dvec3& centreA,
  double radiusA,
  const glm::dvec3& centreB,
  double radiusB,
  double speculative
) {
  Manifold manifold;
  const glm::dvec3 offset = centreA - centreB;
  const double distance = glm::length(offset);
  const double separation = distance - (radiusA + radiusB);

  if (separation > speculative) {
    return manifold;
  }

  manifold.normal =
    distance > 0.0 ? offset / distance : glm::dvec3(0.0, 1.0, 0.0);
  manifold.points[0].point =
    centreB + manifold.normal * (radiusB + 0.5 * separation);
  manifold.points[0].separation = separation;
  manifold.points[0].feature = 0u;
  manifold.count = 1u;

  return manifold;
}

Manifold collideBoxAndSphere(
  const glm::dvec3& sphereCentre,
  double radius,
  const BoxPose& box,
  double speculative
) {
  Manifold manifold;
  const glm::dmat3 turn = glm::mat3_cast(box.rotation);
  const glm::dvec3 local = glm::transpose(turn) * (sphereCentre - box.centre);
  const glm::dvec3 closest = glm::clamp(local, -box.half, box.half);
  const glm::dvec3 offset = local - closest;
  const double distance = glm::length(offset);
  glm::dvec3 normal(0.0);
  glm::dvec3 surface = closest;
  double separation = 0.0;

  if (distance > 0.0) {
    normal = offset / distance;
    separation = distance - radius;
  } else {
    int axis = 0;
    double best = -std::numeric_limits<double>::infinity();

    for (int index = 0; index != 3; ++index) {
      const double depth = std::abs(local[index]) - box.half[index];

      if (depth > best) {
        best = depth;
        axis = index;
      }
    }

    normal[axis] = local[axis] >= 0.0 ? 1.0 : -1.0;
    surface = local;
    surface[axis] = normal[axis] * box.half[axis];
    separation = best - radius;
  }

  if (separation > speculative) {
    return manifold;
  }

  manifold.normal = turn * normal;
  manifold.points[0].point =
    box.centre + turn * surface + manifold.normal * (0.5 * separation);
  manifold.points[0].separation = separation;
  manifold.points[0].feature = 0u;
  manifold.count = 1u;

  return manifold;
}

Manifold collideBoxes(
  const BoxPose& a,
  const BoxPose& b,
  double speculative
) {
  return collideBoxHulls(a, b, speculative);
}

}
