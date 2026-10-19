#include "neohookean.h"

#include <algorithm>

namespace talus::dynamics {

namespace {

constexpr double GAUSS_OFFSET = 0.5773502691896258;

glm::dvec3 cornerSign(uint32_t corner) {
  return glm::dvec3(
    (corner & 1u) != 0u ? 1.0 : -1.0,
    (corner & 2u) != 0u ? 1.0 : -1.0,
    (corner & 4u) != 0u ? 1.0 : -1.0
  );
}

glm::dmat3 cofactorOf(const glm::dmat3& m) {
  return glm::dmat3(
    glm::cross(m[1], m[2]),
    glm::cross(m[2], m[0]),
    glm::cross(m[0], m[1])
  );
}

double traceOf(const glm::dmat3& m) {
  return m[0][0] + m[1][1] + m[2][2];
}

double contract(const glm::dmat3& a, const glm::dmat3& b) {
  double sum = 0.0;

  for (int column = 0; column != 3; ++column) {
    sum += glm::dot(a[column], b[column]);
  }

  return sum;
}

double volumeChange(const glm::dmat3& offset) {
  const double trace = traceOf(offset);

  return trace
         + 0.5 * (trace * trace - contract(offset, glm::transpose(offset)))
         + glm::determinant(offset);
}

Corners displacementOf(const Corners& deformed, const Corners& rest) {
  Corners displacement{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    displacement[corner] = deformed[corner] - rest[corner];
  }

  return displacement;
}

glm::dmat3 offsetOf(
  const Corners& displacement,
  const std::array<glm::dvec3, 8>& gradient
) {
  glm::dmat3 offset(0.0);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    offset += glm::outerProduct(displacement[corner], gradient[corner]);
  }

  return offset;
}

glm::dmat3 offsetAt(
  const Corners& deformed,
  const Corners& rest,
  const std::array<glm::dvec3, 8>& gradient
) {
  return offsetOf(displacementOf(deformed, rest), gradient);
}

}

Moduli moduliOf(const talus::PhysicalMaterial& material) {
  const double nu =
    std::clamp(static_cast<double>(material.poissonsRatio), 0.0, 0.49);
  const double youngs = material.youngsModulus;
  const double first = youngs * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
  const double shear = youngs / (2.0 * (1.0 + nu));

  return {shear, first + shear};
}

std::array<glm::dvec3, 8> shapeGradients(
  const glm::dvec3& at,
  const glm::dvec3& halfExtents
) {
  std::array<glm::dvec3, 8> gradient{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 sign = cornerSign(corner);

    gradient[corner] = glm::dvec3(
      0.125 * sign.x * (1.0 + sign.y * at.y) * (1.0 + sign.z * at.z)
        / halfExtents.x,
      0.125 * sign.y * (1.0 + sign.x * at.x) * (1.0 + sign.z * at.z)
        / halfExtents.y,
      0.125 * sign.z * (1.0 + sign.x * at.x) * (1.0 + sign.y * at.y)
        / halfExtents.z
    );
  }

  return gradient;
}

GaussGradients gaussGradients(const glm::dvec3& halfExtents) {
  GaussGradients gradients{};

  for (uint32_t point = 0u; point != 8u; ++point) {
    gradients[point] =
      shapeGradients(GAUSS_OFFSET * cornerSign(point), halfExtents);
  }

  return gradients;
}

double cellEnergy(
  const Corners& deformed,
  const Corners& rest,
  const glm::dvec3& halfExtents,
  double volume,
  const Moduli& moduli
) {
  const double weight = 0.125 * volume;
  double energy = 0.0;

  for (uint32_t point = 0u; point != 8u; ++point) {
    const std::array<glm::dvec3, 8> gradient =
      shapeGradients(GAUSS_OFFSET * cornerSign(point), halfExtents);
    const glm::dmat3 offset = offsetAt(deformed, rest, gradient);
    const double change = volumeChange(offset);

    energy += weight
              * (0.5 * moduli.shear * contract(offset, offset)
                 - moduli.shear * (change - traceOf(offset))
                 + 0.5 * moduli.shifted * change * change);
  }

  return energy;
}

NodeTerms nodeTerms(
  const Corners& deformed,
  const Corners& rest,
  const glm::dvec3& halfExtents,
  double volume,
  const Moduli& moduli,
  uint32_t corner
) {
  return nodeTerms(
    displacementOf(deformed, rest),
    gaussGradients(halfExtents),
    volume,
    moduli,
    corner
  );
}

NodeTerms nodeTerms(
  const Corners& displacement,
  const GaussGradients& gradients,
  double volume,
  const Moduli& moduli,
  uint32_t corner
) {
  const double weight = 0.125 * volume;
  NodeTerms terms;

  for (uint32_t point = 0u; point != 8u; ++point) {
    const std::array<glm::dvec3, 8>& gradient = gradients[point];
    const glm::dmat3 offset = offsetOf(displacement, gradient);
    const glm::dmat3 shape = glm::dmat3(1.0) + offset;
    const glm::dmat3 cofactor = cofactorOf(shape);
    const double change = volumeChange(offset);
    const glm::dmat3 stress =
      moduli.shear * (shape - cofactor) + (moduli.shifted * change) * cofactor;
    const glm::dvec3 g = gradient[corner];
    const glm::dvec3 b = cofactor * g;

    terms.gradient += weight * (stress * g);
    terms.hessian +=
      weight
      * (moduli.shear * glm::dot(g, g) * glm::dmat3(1.0)
         + moduli.shifted * glm::outerProduct(b, b));
  }

  return terms;
}

std::array<std::array<glm::dmat3, 8>, 8> restStiffness(
  const glm::dvec3& halfExtents,
  double volume,
  const Moduli& moduli
) {
  const double weight = 0.125 * volume;
  std::array<std::array<glm::dmat3, 8>, 8> stiffness{};

  for (std::array<glm::dmat3, 8>& row : stiffness) {
    row.fill(glm::dmat3(0.0));
  }

  for (uint32_t point = 0u; point != 8u; ++point) {
    const std::array<glm::dvec3, 8> gradient =
      shapeGradients(GAUSS_OFFSET * cornerSign(point), halfExtents);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      for (uint32_t other = 0u; other != 8u; ++other) {
        const glm::dvec3& a = gradient[corner];
        const glm::dvec3& b = gradient[other];

        stiffness[corner][other] +=
          weight
          * (moduli.shear * glm::dot(a, b) * glm::dmat3(1.0)
             + moduli.shear * glm::outerProduct(b, a)
             + (moduli.shifted - moduli.shear) * glm::outerProduct(a, b));
      }
    }
  }

  return stiffness;
}

double volumeRatio(
  const Corners& deformed,
  const Corners& rest,
  const glm::dvec3& halfExtents
) {
  const glm::dmat3 offset =
    offsetAt(deformed, rest, shapeGradients(glm::dvec3(0.0), halfExtents));

  return 1.0 + volumeChange(offset);
}

}
