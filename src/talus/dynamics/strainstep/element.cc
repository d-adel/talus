#include "element.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace talus::dynamics {

namespace {

constexpr double DAMPING_RATIO = 1.0;
constexpr double MAX_CORRECTIVE_SPEED = 3.0;
constexpr uint32_t ROTATION_ITERATIONS = 16u;
constexpr double ROTATION_TOLERANCE = 1.0e-9;
constexpr double INFINITE = std::numeric_limits<double>::infinity();

struct Spring {
  double rate = 0.0;
  double softness = 0.0;
};

Spring springOf(double omega, double zeta, double h, double theta) {
  const double b = h * omega;
  const double a = theta * b * b + 2.0 * zeta * b;

  return a > 0.0 ? Spring{h * omega * omega / a, 1.0 / a} : Spring{};
}

double rowDamping(const ElementDamping& damping, double omega) {
  if (!std::isfinite(damping.target) || !std::isfinite(damping.lowest)
      || !(damping.lowest > 0.0)) {
    return DAMPING_RATIO;
  }

  return std::clamp(
    damping.target * omega / damping.lowest - damping.own * omega / 2.0,
    0.0,
    DAMPING_RATIO
  );
}

ElementInverse symmetricInverse(
  const ElementInverse& block,
  const std::array<bool, ELEMENT_ROWS>& inert
) {
  constexpr uint32_t SIZE = ELEMENT_ROWS;
  ElementInverse m = block;
  ElementInverse out{};

  for (uint32_t row = 0u; row != SIZE; ++row) {
    if (inert[row]) {
      for (uint32_t k = 0u; k != SIZE; ++k) {
        m[SIZE * row + k] = 0.0;
        m[SIZE * k + row] = 0.0;
      }

      m[(SIZE + 1u) * row] = 1.0;
    }

    out[(SIZE + 1u) * row] = 1.0;
  }

  for (uint32_t column = 0u; column != SIZE; ++column) {
    const double pivot = m[(SIZE + 1u) * column];

    if (!(pivot > 0.0)) {
      return {};
    }

    for (uint32_t k = 0u; k != SIZE; ++k) {
      m[SIZE * column + k] /= pivot;
      out[SIZE * column + k] /= pivot;
    }

    for (uint32_t row = 0u; row != SIZE; ++row) {
      const double factor = m[SIZE * row + column];

      if (row == column || factor == 0.0) {
        continue;
      }

      for (uint32_t k = 0u; k != SIZE; ++k) {
        m[SIZE * row + k] -= factor * m[SIZE * column + k];
        out[SIZE * row + k] -= factor * out[SIZE * column + k];
      }
    }
  }

  for (uint32_t row = 0u; row != SIZE; ++row) {
    if (inert[row]) {
      for (uint32_t k = 0u; k != SIZE; ++k) {
        out[SIZE * row + k] = 0.0;
        out[SIZE * k + row] = 0.0;
      }
    }
  }

  return out;
}

const std::array<std::array<double, 4>, 8> SIGNS = [] {
  std::array<std::array<double, 4>, 8> table{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    for (uint32_t mode = 0u; mode != 4u; ++mode) {
      table[corner][mode] = hourglassSign(mode, corner);
    }
  }

  return table;
}();

void addRows(
  std::array<double, ELEMENT_ROWS>& rows,
  const StrainElement& element,
  uint32_t corner,
  const glm::dvec3& local
) {
  const glm::dvec3& c = element.gradient[corner];

  rows[0] += local.x * c.x;
  rows[1] += local.y * c.y;
  rows[2] += local.z * c.z;
  rows[3] += 0.5 * (local.x * c.y + local.y * c.x);
  rows[4] += 0.5 * (local.y * c.z + local.z * c.y);
  rows[5] += 0.5 * (local.z * c.x + local.x * c.z);

  for (uint32_t mode = 0u; mode != 4u; ++mode) {
    const double sign = SIGNS[corner][mode] * element.bend;

    rows[6u + 3u * mode] += sign * local.x;
    rows[7u + 3u * mode] += sign * local.y;
    rows[8u + 3u * mode] += sign * local.z;
  }
}

glm::dvec3 rowForce(
  const StrainElement& element,
  uint32_t corner,
  std::span<const double, ELEMENT_ROWS> impulse
) {
  const glm::dvec3& c = element.gradient[corner];
  glm::dvec3 local(
    impulse[0] * c.x + 0.5 * (impulse[3] * c.y + impulse[5] * c.z),
    impulse[1] * c.y + 0.5 * (impulse[3] * c.x + impulse[4] * c.z),
    impulse[2] * c.z + 0.5 * (impulse[4] * c.y + impulse[5] * c.x)
  );

  for (uint32_t mode = 0u; mode != 4u; ++mode) {
    const double sign = SIGNS[corner][mode] * element.bend;

    local.x += sign * impulse[6u + 3u * mode];
    local.y += sign * impulse[7u + 3u * mode];
    local.z += sign * impulse[8u + 3u * mode];
  }

  return local;
}

}

glm::dvec3 cornerSigns(uint32_t corner) {
  return glm::dvec3(
    (corner & 1u) != 0u ? 1.0 : -1.0,
    (corner & 2u) != 0u ? 1.0 : -1.0,
    (corner & 4u) != 0u ? 1.0 : -1.0
  );
}

double hourglassSign(uint32_t mode, uint32_t corner) {
  const glm::dvec3 sign = cornerSigns(corner);

  switch (mode) {
    case 0u:
      return sign.x * sign.y;
    case 1u:
      return sign.y * sign.z;
    case 2u:
      return sign.z * sign.x;
    default:
      return sign.x * sign.y * sign.z;
  }
}

ElementMaterial elementMaterial(const talus::PhysicalMaterial& material) {
  const double nu = std::clamp(static_cast<double>(material.poissonsRatio), 0.0, 0.499);
  const double youngs = std::max(0.0, static_cast<double>(material.youngsModulus));

  return {youngs / (2.0 * (1.0 + nu)), youngs * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))};
}

double cellFrequency(const talus::PhysicalMaterial& material, double cell) {
  const double nu = std::clamp(static_cast<double>(material.poissonsRatio), 0.0, 0.499);
  const double youngs = std::max(0.0, static_cast<double>(material.youngsModulus));

  return std::sqrt(youngs / ((1.0 + nu) * static_cast<double>(material.density))) / cell;
}

ElementRest cornerRest(std::span<const glm::dvec3, 8> corners, double volume) {
  ElementRest rest;

  rest.shape = glm::dmat3(0.0);
  rest.volume = volume;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    rest.shape += glm::outerProduct(corners[corner], cornerSigns(corner)) / 8.0;

    for (uint32_t mode = 0u; mode != 4u; ++mode) {
      rest.hourglass[mode] += hourglassSign(mode, corner) * corners[corner];
    }
  }

  return rest;
}

glm::dvec3 rowJacobian(const StrainElement& element, uint32_t corner, uint32_t row) {
  const glm::dvec3& c = element.gradient[corner];

  switch (row) {
    case 0u:
      return glm::dvec3(c.x, 0.0, 0.0);
    case 1u:
      return glm::dvec3(0.0, c.y, 0.0);
    case 2u:
      return glm::dvec3(0.0, 0.0, c.z);
    case 3u:
      return 0.5 * glm::dvec3(c.y, c.x, 0.0);
    case 4u:
      return 0.5 * glm::dvec3(0.0, c.z, c.y);
    case 5u:
      return 0.5 * glm::dvec3(c.z, 0.0, c.x);
    default:
      break;
  }

  glm::dvec3 direction(0.0);

  direction[static_cast<glm::length_t>((row - 6u) % 3u)] =
    element.bend * SIGNS[corner][(row - 6u) / 3u];

  return direction;
}

std::optional<StrainElement> makeElement(
  uint32_t cell,
  std::span<const uint32_t, 8> nodes,
  std::span<const double, 8> inverseMass,
  const ElementRest& rest,
  const ElementMaterial& material,
  double h,
  const ElementDamping& damping
) {
  const double determinant = glm::determinant(rest.shape);

  if (!(std::abs(determinant) > 0.0) || !(material.shear > 0.0) || !(rest.volume > 0.0)) {
    return std::nullopt;
  }

  StrainElement element;
  const glm::dmat3 inverseTranspose = glm::transpose(glm::inverse(rest.shape));

  element.cell = cell;
  element.size = std::cbrt(8.0 * std::abs(determinant));
  element.bend = 1.0 / (4.0 * element.size);
  element.restHourglass = rest.hourglass;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    element.nodes[corner] = nodes[corner];
    element.inverseMass[corner] = inverseMass[corner];
    element.gradient[corner] = inverseTranspose * cornerSigns(corner) / 8.0;
  }

  ElementInverse block{};
  double volumeWeight = 0.0;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const double weight = element.inverseMass[corner];

    if (weight == 0.0) {
      continue;
    }

    volumeWeight += weight * glm::dot(element.gradient[corner], element.gradient[corner]);

    for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
      const glm::dvec3 left = rowJacobian(element, corner, row);

      for (uint32_t column = 0u; column != ELEMENT_ROWS; ++column) {
        block[ELEMENT_ROWS * row + column] +=
          weight * glm::dot(left, rowJacobian(element, corner, column));
      }
    }
  }

  if (!(volumeWeight > 0.0)) {
    return std::nullopt;
  }

  const double hourglass = material.shear * (3.0 * material.first + 2.0 * material.shear)
                           / (3.0 * (material.first + material.shear));
  std::array<bool, ELEMENT_ROWS> inert{};

  for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
    const double modulus =
      row < 3u ? 2.0 * material.shear : row < 6u ? 4.0 * material.shear : hourglass;
    const double weight = block[(ELEMENT_ROWS + 1u) * row];
    const double omega = std::sqrt(modulus * rest.volume * weight);

    element.stiffness[row] = modulus * rest.volume;

    if (omega > 0.0) {
      const Spring spring = springOf(omega, rowDamping(damping, omega), h, damping.theta);

      element.rate[row] = spring.rate;
      element.softness[row] = weight * spring.softness;
      block[(ELEMENT_ROWS + 1u) * row] += element.softness[row];
      element.cap[row] = MAX_CORRECTIVE_SPEED / (spring.rate * element.size);
    } else {
      inert[row] = true;
      element.cap[row] = INFINITE;
    }
  }

  element.inverse = std::make_shared<const ElementInverse>(symmetricInverse(block, inert));
  element.volumeStiffness = material.first * rest.volume;
  element.volumeCap = INFINITE;

  const double volumeOmega = std::sqrt(element.volumeStiffness * volumeWeight);

  if (volumeOmega > 0.0) {
    const Spring spring =
      springOf(volumeOmega, rowDamping(damping, volumeOmega), h, damping.theta);

    element.volumeRate = spring.rate;
    element.volumeSoftness = spring.softness;
    element.volumeCap =
      spring.rate > 0.0 ? MAX_CORRECTIVE_SPEED / (spring.rate * element.size) : INFINITE;
    element.volumetric = true;
  }

  return element;
}

glm::dquat rotationOf(const glm::dmat3& deformation, const glm::dquat& guess) {
  glm::dquat rotation = glm::normalize(guess);

  for (uint32_t iteration = 0u; iteration != ROTATION_ITERATIONS; ++iteration) {
    const glm::dmat3 turn = glm::mat3_cast(rotation);
    const glm::dvec3 torque = glm::cross(turn[0], deformation[0])
                              + glm::cross(turn[1], deformation[1])
                              + glm::cross(turn[2], deformation[2]);
    const double scale = std::abs(
                           glm::dot(turn[0], deformation[0]) + glm::dot(turn[1], deformation[1])
                           + glm::dot(turn[2], deformation[2])
                         )
                         + 1.0e-9;
    const glm::dvec3 omega = torque / scale;
    const double angle = glm::length(omega);

    if (!(angle > 1.0e-12)) {
      break;
    }

    rotation = glm::normalize(glm::angleAxis(angle, omega / angle) * rotation);

    if (angle <= ROTATION_TOLERANCE) {
      break;
    }
  }

  return rotation;
}

void updateElement(StrainElement& element, std::span<const glm::dvec3> positions) {
  glm::dmat3 deformation(0.0);
  std::array<glm::dvec3, 4> hourglass{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3& at = positions[element.nodes[corner]];

    deformation += glm::outerProduct(at, element.gradient[corner]);

    for (uint32_t mode = 0u; mode != 4u; ++mode) {
      hourglass[mode] += SIGNS[corner][mode] * at;
    }
  }

  element.rotation = rotationOf(deformation, element.rotation);
  element.turn = glm::mat3_cast(element.rotation);

  const glm::dmat3 back = glm::transpose(element.turn);
  const glm::dmat3 local = back * deformation;
  const double determinant = glm::determinant(deformation);
  const bool inverted = determinant < 0.0;
  std::array<double, ELEMENT_ROWS> raw{};

  raw[0] = local[0][0] - 1.0;
  raw[1] = local[1][1] - 1.0;
  raw[2] = local[2][2] - 1.0;
  raw[3] = 0.5 * (local[1][0] + local[0][1]);
  raw[4] = 0.5 * (local[2][1] + local[1][2]);
  raw[5] = 0.5 * (local[0][2] + local[2][0]);

  for (uint32_t mode = 0u; mode != 4u; ++mode) {
    const glm::dvec3 offset =
      (back * hourglass[mode] - element.restHourglass[mode]) * element.bend;

    raw[6u + 3u * mode] = offset.x;
    raw[7u + 3u * mode] = offset.y;
    raw[8u + 3u * mode] = offset.z;
  }

  for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
    const double cap = inverted ? element.cap[row] : std::max(element.cap[row], 1.0);

    element.strain[row] = std::clamp(raw[row], -cap, cap);
  }

  const glm::dmat3 cofactor(
    glm::cross(deformation[1], deformation[2]),
    glm::cross(deformation[2], deformation[0]),
    glm::cross(deformation[0], deformation[1])
  );
  double volumeWeight = 0.0;
  const ElementInverse& inverse = *element.inverse;

  element.coupling.fill(0.0);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 gradient = cofactor * element.gradient[corner];
    const double weight = element.inverseMass[corner];

    element.volumeGradient[corner] = gradient;
    volumeWeight += weight * glm::dot(gradient, gradient);
    addRows(element.coupling, element, corner, back * gradient * weight);
  }

  const double volumeCap = inverted ? element.volumeCap : std::max(element.volumeCap, 1.0);
  const double volume = std::clamp(determinant - 1.0, -volumeCap, volumeCap);
  double coupled = 0.0;

  for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
    double total = 0.0;

    for (uint32_t column = 0u; column != ELEMENT_ROWS; ++column) {
      total += inverse[ELEMENT_ROWS * row + column] * element.coupling[column];
    }

    element.image[row] = total;
    coupled += element.coupling[row] * total;
  }

  element.gain = volumeWeight * element.volumeSoftness;

  const double schur = volumeWeight + element.gain - coupled;

  element.schur = element.volumetric && schur > 0.0 ? 1.0 / schur : 0.0;
  element.rhs = volume * element.volumeRate;
}

void applyElement(
  const StrainElement& element,
  std::span<glm::dvec3> velocities,
  std::span<const double, ELEMENT_ROWS> impulse,
  double volumeImpulse
) {
  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const double weight = element.inverseMass[corner];

    if (weight == 0.0) {
      continue;
    }

    velocities[element.nodes[corner]] -=
      (element.turn * rowForce(element, corner, impulse)
       + element.volumeGradient[corner] * volumeImpulse)
      * weight;
  }
}

void solveElement(StrainElement& element, std::span<glm::dvec3> velocities) {
  const glm::dmat3 back = glm::transpose(element.turn);
  std::array<double, ELEMENT_ROWS> residual{};
  double volumeSpeed = 0.0;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3& velocity = velocities[element.nodes[corner]];

    volumeSpeed += glm::dot(element.volumeGradient[corner], velocity);
    addRows(residual, element, corner, back * velocity);
  }

  for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
    residual[row] += element.rate[row] * element.strain[row]
                     - element.softness[row] * element.impulse[row];
  }

  std::array<double, ELEMENT_ROWS> solution{};
  const ElementInverse& inverse = *element.inverse;
  double coupled = 0.0;

  for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
    double total = 0.0;

    for (uint32_t column = 0u; column != ELEMENT_ROWS; ++column) {
      total += inverse[ELEMENT_ROWS * row + column] * residual[column];
    }

    solution[row] = total;
    coupled += element.coupling[row] * total;
  }

  const double volume = element.rhs - element.gain * element.volumeImpulse + volumeSpeed;
  const double change = element.schur * (volume - coupled);

  for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
    solution[row] -= element.image[row] * change;
    element.impulse[row] += solution[row];
  }

  element.volumeImpulse += change;
  applyElement(element, velocities, solution, change);
}

}
