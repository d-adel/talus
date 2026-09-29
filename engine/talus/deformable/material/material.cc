#include "material.ih"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace talus::deformable {

const objectmodel::PhysicalMaterial& laneMaterial(
  std::span<const objectmodel::PhysicalMaterial> materials,
  uint32_t lane
) {
  static const objectmodel::PhysicalMaterial DEFAULT{};

  return lane < materials.size() ? materials[lane] : DEFAULT;
}

namespace {

constexpr uint32_t POLAR_ITERATIONS = 24u;

constexpr float POLAR_TOLERANCE = 1.0e-6f;

glm::mat3 elementRotation(const glm::mat3& gradient, ElementRotation rotation) {
  if (rotation == ElementRotation::Polar || glm::determinant(gradient) > 0.0f) {
    return polarRotation(gradient);
  }

  Eigen::Matrix3d held;

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      held(row, column) = static_cast<double>(gradient[column][row]);
    }
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> decomposed(
    held,
    Eigen::ComputeFullU | Eigen::ComputeFullV
  );

  Eigen::Matrix3d left = decomposed.matrixU();
  Eigen::Matrix3d right = decomposed.matrixV();

  if (left.determinant() < 0.0) {
    left.col(2) *= -1.0;
  }

  if (right.determinant() < 0.0) {
    right.col(2) *= -1.0;
  }

  const Eigen::Matrix3d turned = left * right.transpose();

  glm::mat3 result(1.0f);

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      result[column][row] = static_cast<float>(turned(row, column));
    }
  }

  return result;
}

struct Lame {
  float first = 0.0f;
  float second = 0.0f;
};

Lame lame(const ElasticParams& params) {
  const float nu = glm::clamp(params.poissonsRatio, 0.0f, 0.49f);

  return {
    params.youngsModulus * nu / ((1.0f + nu) * (1.0f - 2.0f * nu)),
    params.youngsModulus / (2.0f * (1.0f + nu))
  };
}

glm::mat3 block(
  const glm::vec3& first,
  const glm::vec3& second,
  const Lame& moduli,
  float volume
) {
  return volume
       * (moduli.first * glm::outerProduct(first, second)
          + moduli.second * glm::outerProduct(second, first)
          + moduli.second * glm::dot(first, second) * glm::mat3(1.0f));
}

std::array<glm::vec3, 4> corotatedOffsets(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const glm::mat3& rotation
) {
  const glm::mat3 inverse = glm::transpose(rotation);

  const glm::vec3 deformedCentre =
    0.25f * (deformed[0] + deformed[1] + deformed[2] + deformed[3]);
  const glm::vec3 restCentre = 0.25f * (rest[0] + rest[1] + rest[2] + rest[3]);

  return {
    inverse * (deformed[0] - deformedCentre) - (rest[0] - restCentre),
    inverse * (deformed[1] - deformedCentre) - (rest[1] - restCentre),
    inverse * (deformed[2] - deformedCentre) - (rest[2] - restCentre),
    inverse * (deformed[3] - deformedCentre) - (rest[3] - restCentre)
  };
}

std::array<glm::vec3, 4> unrotatedResponse(
  const TetStiffness& stiffness,
  const std::array<glm::vec3, 4>& offsets
) {
  std::array<glm::vec3, 4> response{};

  for (uint32_t row = 0u; row != 4u; ++row) {
    glm::vec3 sum(0.0f);

    for (uint32_t column = 0u; column != 4u; ++column) {
      sum += stiffness.block[row * 4u + column] * offsets[column];
    }

    response[row] = sum;
  }

  return response;
}

struct TetStrain {
  glm::mat3 rotation{1.0f};
  std::array<glm::vec3, 4> offsets{};
  glm::mat3 strain{0.0f};
};

TetStrain tetStrain(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
  const std::array<glm::vec3, 4>& gradient,
  ElementRotation rotation
) {
  TetStrain measured;

  const glm::mat3 deformation = glm::mat3(
                                  deformed[1] - deformed[0],
                                  deformed[2] - deformed[0],
                                  deformed[3] - deformed[0]
                                )
                                * tet.restInverse;

  measured.rotation = elementRotation(deformation, rotation);
  measured.offsets = corotatedOffsets(deformed, rest, measured.rotation);

  glm::mat3 displacement(0.0f);

  for (uint32_t node = 0u; node != 4u; ++node) {
    displacement += glm::outerProduct(measured.offsets[node], gradient[node]);
  }

  measured.strain = 0.5f * (displacement + glm::transpose(displacement));

  return measured;
}

}

glm::mat3 polarRotation(const glm::mat3& gradient) {
  if (std::abs(glm::determinant(gradient)) < 1.0e-12f) {
    return glm::mat3(1.0f);
  }

  glm::mat3 rotation = gradient;

  for (uint32_t pass = 0u; pass != POLAR_ITERATIONS; ++pass) {
    const glm::mat3 next =
      0.5f * (rotation + glm::transpose(glm::inverse(rotation)));

    const glm::mat3 change = next - rotation;

    rotation = next;

    float largest = 0.0f;

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        largest = glm::max(largest, std::abs(change[column][row]));
      }
    }

    if (largest < POLAR_TOLERANCE) {
      break;
    }
  }

  return rotation;
}

std::array<glm::vec3, 4> shapeGradients(const glm::mat3& restInverse) {
  const glm::mat3 rows = glm::transpose(restInverse);

  const glm::vec3 first = rows[0];
  const glm::vec3 second = rows[1];
  const glm::vec3 third = rows[2];

  return {-(first + second + third), first, second, third};
}

TetStiffness tetStiffness(
  const objectmodel::PreparedTet& tet,
  const ElasticParams& params
) {
  TetStiffness stiffness;

  stiffness.gradient = shapeGradients(tet.restInverse);

  const Lame moduli = lame(params);

  stiffness.first = moduli.first;
  stiffness.second = moduli.second;

  for (uint32_t row = 0u; row != 4u; ++row) {
    for (uint32_t column = 0u; column != 4u; ++column) {
      stiffness.block[row * 4u + column] = block(
        stiffness.gradient[row],
        stiffness.gradient[column],
        moduli,
        tet.volume
      );
    }
  }

  return stiffness;
}

TrialState trial(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
  const TetStiffness& stiffness,
  ElementRotation rotation
) {
  TrialState state;

  const TetStrain measured =
    tetStrain(deformed, rest, tet, stiffness.gradient, rotation);
  const glm::mat3& strain = measured.strain;

  state.rotation = measured.rotation;

  state.stress = 2.0f * stiffness.second * strain
                 + stiffness.first
                     * (strain[0][0] + strain[1][1] + strain[2][2])
                     * glm::mat3(1.0f);

  const std::array<glm::vec3, 4> response =
    unrotatedResponse(stiffness, measured.offsets);

  for (uint32_t row = 0u; row != 4u; ++row) {
    state.energy += 0.5f * glm::dot(measured.offsets[row], response[row]);
    state.force[row] = state.rotation * -response[row];
  }

  return state;
}

void applyStiffness(
  const std::array<glm::vec3, 4>& direction,
  const glm::mat3& rotation,
  const TetStiffness& stiffness,
  std::array<glm::vec3, 4>& result
) {
  const glm::mat3 inverse = glm::transpose(rotation);

  const std::array<glm::vec3, 4> unrotated{
    inverse * direction[0],
    inverse * direction[1],
    inverse * direction[2],
    inverse * direction[3]
  };

  const std::array<glm::vec3, 4> response =
    unrotatedResponse(stiffness, unrotated);

  for (uint32_t row = 0u; row != 4u; ++row) {
    result[row] = rotation * response[row];
  }
}

std::array<glm::mat3, 4> stiffnessDiagonal(
  const TetStiffness& stiffness,
  const glm::mat3& rotation
) {
  std::array<glm::mat3, 4> diagonal{};
  const glm::mat3 inverse = glm::transpose(rotation);

  for (uint32_t node = 0u; node != 4u; ++node) {
    const glm::mat3& held = stiffness.block[node * 4u + node];

    diagonal[node] = rotation * held * inverse;
  }

  return diagonal;
}

objectmodel::CommittedTet commit(const TrialState& state) {
  return objectmodel::CommittedTet{state.rotation};
}

namespace {

constexpr float GAUSS_OFFSET = 0.5773502691896258f;

glm::vec3 hexSign(uint32_t corner) {
  return glm::vec3(
    (corner & 1u) != 0u ? 1.0f : -1.0f,
    (corner & 2u) != 0u ? 1.0f : -1.0f,
    (corner & 4u) != 0u ? 1.0f : -1.0f
  );
}

std::array<glm::vec3, 8> hexGradients(
  const glm::vec3& at,
  const glm::vec3& halfExtents
) {
  std::array<glm::vec3, 8> gradient{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::vec3 sign = hexSign(corner);

    gradient[corner] = glm::vec3(
      0.125f * sign.x * (1.0f + sign.y * at.y) * (1.0f + sign.z * at.z)
        / halfExtents.x,
      0.125f * sign.y * (1.0f + sign.x * at.x) * (1.0f + sign.z * at.z)
        / halfExtents.y,
      0.125f * sign.z * (1.0f + sign.x * at.x) * (1.0f + sign.y * at.y)
        / halfExtents.z
    );
  }

  return gradient;
}

std::array<glm::vec3, 8> centredOffsets(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const glm::mat3& rotation
) {
  const glm::mat3 inverse = glm::transpose(rotation);

  glm::vec3 deformedCentre(0.0f);
  glm::vec3 restCentre(0.0f);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    deformedCentre += deformed[corner];
    restCentre += rest[corner];
  }

  deformedCentre *= 0.125f;
  restCentre *= 0.125f;

  std::array<glm::vec3, 8> offset{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    offset[corner] = inverse * (deformed[corner] - deformedCentre)
                     - (rest[corner] - restCentre);
  }

  return offset;
}

void accumulateHex(
  const std::array<glm::vec3, 8>& gradient,
  float weight,
  float lambda,
  float mu,
  std::array<glm::mat3, 64>& block
) {
  for (uint32_t row = 0u; row != 8u; ++row) {
    for (uint32_t column = 0u; column != 8u; ++column) {
      const glm::vec3& a = gradient[row];
      const glm::vec3& b = gradient[column];

      glm::mat3 local(0.0f);

      for (int i = 0; i != 3; ++i) {
        for (int j = 0; j != 3; ++j) {
          local[j][i] = lambda * a[i] * b[j] + mu * a[j] * b[i];
        }

        local[i][i] += mu * glm::dot(a, b);
      }

      block[row * 8u + column] += weight * local;
    }
  }
}

}

HexStiffness hexStiffness(
  const objectmodel::PreparedHex& hex,
  const ElasticParams& params
) {
  HexStiffness stiffness;

  const Lame moduli = lame(params);
  const float bulk = moduli.first + 2.0f * moduli.second / 3.0f;
  const float deviatoric = -2.0f * moduli.second / 3.0f;
  const float share = hex.volume * 0.125f;

  for (uint32_t point = 0u; point != 8u; ++point) {
    const glm::vec3 at = GAUSS_OFFSET * hexSign(point);

    accumulateHex(
      hexGradients(at, hex.halfExtents),
      share,
      deviatoric,
      moduli.second,
      stiffness.block
    );
  }

  accumulateHex(
    hexGradients(glm::vec3(0.0f), hex.halfExtents),
    hex.volume,
    bulk,
    0.0f,
    stiffness.block
  );

  return stiffness;
}

glm::mat3 hexRotation(
  const std::array<glm::vec3, 8>& deformed,
  const glm::vec3& halfExtents,
  ElementRotation rotation
) {
  const std::array<glm::vec3, 8> gradient =
    hexGradients(glm::vec3(0.0f), halfExtents);

  glm::mat3 gradientOf(0.0f);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    gradientOf += glm::outerProduct(deformed[corner], gradient[corner]);
  }

  return elementRotation(gradientOf, rotation);
}

void applyHexStiffness(
  const std::array<glm::vec3, 8>& direction,
  const glm::mat3& rotation,
  const HexStiffness& stiffness,
  std::array<glm::vec3, 8>& result
) {
  const glm::mat3 inverse = glm::transpose(rotation);

  std::array<glm::vec3, 8> unrotated{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    unrotated[corner] = inverse * direction[corner];
  }

  for (uint32_t row = 0u; row != 8u; ++row) {
    glm::vec3 sum(0.0f);

    for (uint32_t column = 0u; column != 8u; ++column) {
      sum += stiffness.block[row * 8u + column] * unrotated[column];
    }

    result[row] = rotation * sum;
  }
}

std::array<glm::mat3, 8> hexStiffnessDiagonal(
  const HexStiffness& stiffness,
  const glm::mat3& rotation
) {
  std::array<glm::mat3, 8> diagonal{};

  const glm::mat3 inverse = glm::transpose(rotation);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    diagonal[corner] =
      rotation * stiffness.block[corner * 8u + corner] * inverse;
  }

  return diagonal;
}

HexTrial hexTrial(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const objectmodel::PreparedHex& hex,
  const HexStiffness& stiffness,
  ElementRotation rotation
) {
  HexTrial trial;

  trial.rotation = hexRotation(deformed, hex.halfExtents, rotation);

  const std::array<glm::vec3, 8> offset =
    centredOffsets(deformed, rest, trial.rotation);

  for (uint32_t row = 0u; row != 8u; ++row) {
    glm::vec3 sum(0.0f);

    for (uint32_t column = 0u; column != 8u; ++column) {
      sum += stiffness.block[row * 8u + column] * offset[column];
    }

    trial.energy += 0.5f * glm::dot(offset[row], sum);
    trial.force[row] = -(trial.rotation * sum);
  }

  return trial;
}

namespace {

float traceOf(const glm::mat3& m) {
  return m[0][0] + m[1][1] + m[2][2];
}

glm::mat3 deviatorOf(const glm::mat3& m) {
  return m - (traceOf(m) / 3.0f) * glm::mat3(1.0f);
}

float contract(const glm::mat3& a, const glm::mat3& b) {
  return glm::dot(a[0], b[0]) + glm::dot(a[1], b[1]) + glm::dot(a[2], b[2]);
}

struct PointLaw {
  float bulk = 0.0f;
  float shear = 0.0f;
  float yield = 0.0f;
  float hardening = 0.0f;
};

struct PointResponse {
  glm::mat3 stress{0.0f};
  glm::mat3 normal{0.0f};
  float theta = 1.0f;
  float thetaBar = 0.0f;
  float plasticWork = 0.0f;
  float excess = 0.0f;
  bool plastic = false;
  objectmodel::PlasticPoint next{};
};

PointResponse returnMap(
  const glm::mat3& strain,
  const objectmodel::PlasticPoint& committed,
  const PointLaw& law,
  bool returnMapping
) {
  PointResponse response;

  response.next = committed;

  const glm::mat3 elastic = strain - committed.strain;
  const float pressure = law.bulk * traceOf(elastic);
  const glm::mat3 trial = 2.0f * law.shear * deviatorOf(elastic);
  const float norm = std::sqrt(contract(trial, trial));
  const float equivalent = std::sqrt(1.5f) * norm;
  const float radius = law.yield + law.hardening * committed.equivalent;

  if (!returnMapping || !(equivalent > radius) || !(norm > 0.0f)) {
    response.stress = trial + pressure * glm::mat3(1.0f);

    return response;
  }

  const float gamma =
    (equivalent - radius) / (3.0f * law.shear + law.hardening);
  const glm::mat3 normal = trial / norm;
  const float theta = 1.0f - 3.0f * law.shear * gamma / equivalent;

  response.stress = theta * trial + pressure * glm::mat3(1.0f);
  response.normal = normal;
  response.theta = theta;
  response.thetaBar =
    3.0f * law.shear / (3.0f * law.shear + law.hardening) - (1.0f - theta);
  response.plasticWork = (radius + law.hardening * gamma) * gamma;
  response.excess = (equivalent - radius) / radius;
  response.plastic = true;
  response.next.strain = committed.strain + std::sqrt(1.5f) * gamma * normal;
  response.next.equivalent = committed.equivalent + gamma;

  return response;
}

glm::mat3 tangentOf(
  const PointResponse& response,
  const PointLaw& law,
  const glm::mat3& strain
) {
  glm::mat3 stress = law.bulk * traceOf(strain) * glm::mat3(1.0f)
                     + 2.0f * law.shear * response.theta * deviatorOf(strain);

  if (response.plastic) {
    stress -= 2.0f * law.shear * response.thetaBar
              * contract(response.normal, strain) * response.normal;
  }

  return stress;
}

glm::vec3 pointForce(
  const glm::mat3& deviator,
  float mean,
  const glm::vec3& gradient,
  const glm::vec3& centre
) {
  return deviator * gradient + mean * centre;
}

glm::mat3 unitStrain(
  uint32_t axis,
  const glm::vec3& gradient,
  const glm::vec3& centre
) {
  glm::vec3 direction(0.0f);
  direction[static_cast<int>(axis)] = 1.0f;

  const glm::mat3 displacement = glm::outerProduct(direction, gradient);

  return deviatorOf(0.5f * (displacement + glm::transpose(displacement)))
         + (centre[static_cast<int>(axis)] / 3.0f) * glm::mat3(1.0f);
}

PointLaw pointLaw(const Lame& moduli, const PlasticParams& plastic) {
  return PointLaw{
    moduli.first + 2.0f * moduli.second / 3.0f,
    moduli.second,
    yields(plastic) ? plastic.yieldStress
                    : std::numeric_limits<float>::infinity(),
    glm::max(plastic.hardeningModulus, 0.0f)
  };
}

struct Spread {
  glm::mat3 inverse{0.0f};
  bool held = false;
};

Spread spreadOf(const glm::mat3& deformation, const glm::mat3& rotation) {
  Spread made;

  if (std::abs(glm::determinant(deformation)) < 1.0e-12f) {
    return made;
  }

  const glm::mat3 stretch = glm::transpose(rotation) * deformation;
  const glm::mat3 symmetric = 0.5f * (stretch + glm::transpose(stretch));
  const glm::mat3 spread = traceOf(symmetric) * glm::mat3(1.0f) - symmetric;

  if (!(glm::determinant(spread) > 0.0f)) {
    return made;
  }

  made.inverse = glm::inverse(spread);
  made.held = true;

  return made;
}

template<size_t CORNERS>
void addRotationTerm(
  const std::array<glm::vec3, CORNERS>& deformed,
  const std::array<glm::vec3, CORNERS>& centre,
  const glm::mat3& rotation,
  std::array<glm::vec3, CORNERS>& force
) {
  glm::dvec3 middle(0.0);

  for (const glm::vec3& at : deformed) {
    middle += glm::dvec3(at);
  }

  middle /= static_cast<double>(CORNERS);

  glm::mat3 deformation(0.0f);
  glm::dvec3 moment(0.0);

  for (size_t corner = 0u; corner != CORNERS; ++corner) {
    const glm::dvec3 arm = glm::dvec3(deformed[corner]) - middle;

    deformation += glm::outerProduct(glm::vec3(arm), centre[corner]);
    moment += glm::cross(arm, glm::dvec3(force[corner]));
  }

  const Spread spread = spreadOf(deformation, rotation);

  if (!spread.held) {
    return;
  }

  const float handed = glm::determinant(rotation) < 0.0f ? -1.0f : 1.0f;
  const glm::vec3 turn =
    handed * (spread.inverse * (glm::transpose(rotation) * glm::vec3(moment)));

  for (size_t corner = 0u; corner != CORNERS; ++corner) {
    force[corner] -= rotation * glm::cross(turn, centre[corner]);
  }
}
}

bool yields(const PlasticParams& params) noexcept {
  return std::isfinite(params.yieldStress) && params.yieldStress > 0.0f;
}

HexPointTrial hexPointTrial(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const objectmodel::PreparedHex& hex,
  const ElasticParams& elastic,
  const PlasticParams& plastic,
  const std::array<objectmodel::PlasticPoint, 8>& committed,
  float margin,
  ElementRotation rotation,
  bool returnMapping,
  bool tangent,
  bool rotationDerivative
) {
  HexPointTrial trial;

  trial.rotation = hexRotation(deformed, hex.halfExtents, rotation);

  const std::array<glm::vec3, 8> offset =
    centredOffsets(deformed, rest, trial.rotation);

  const PointLaw law = pointLaw(lame(elastic), plastic);

  const std::array<glm::vec3, 8> centre =
    hexGradients(glm::vec3(0.0f), hex.halfExtents);

  glm::mat3 centreDisplacement(0.0f);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    centreDisplacement += glm::outerProduct(offset[corner], centre[corner]);
  }

  const float volumetric = traceOf(centreDisplacement) / 3.0f;
  const float weight = hex.volume * 0.125f;

  std::array<glm::vec3, 8> internal{};

  for (uint32_t point = 0u; point != 8u; ++point) {
    const std::array<glm::vec3, 8> gradient =
      hexGradients(GAUSS_OFFSET * hexSign(point), hex.halfExtents);

    glm::mat3 displacement(0.0f);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      displacement += glm::outerProduct(offset[corner], gradient[corner]);
    }

    const glm::mat3 strain =
      deviatorOf(0.5f * (displacement + glm::transpose(displacement)))
      + volumetric * glm::mat3(1.0f);

    const PointResponse response =
      returnMap(strain, committed[point], law, returnMapping);

    trial.points[point] = response.next;
    trial.energy +=
      weight * 0.5f * contract(response.stress, strain - response.next.strain);
    trial.plasticWork += weight * response.plasticWork;
    trial.yielding += response.plastic ? 1u : 0u;

    const bool loading = response.plastic && response.excess > margin;

    trial.loading += loading ? 1u : 0u;

    const PointResponse linearized =
      loading || !response.plastic ? response : PointResponse{};

    const glm::mat3 responseDeviator = deviatorOf(response.stress);
    const float responseMean = traceOf(response.stress) / 3.0f;

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      internal[corner] += weight
                          * pointForce(
                            responseDeviator,
                            responseMean,
                            gradient[corner],
                            centre[corner]
                          );
    }

    if (!tangent) {
      continue;
    }

    for (uint32_t column = 0u; column != 8u; ++column) {
      for (uint32_t axis = 0u; axis != 3u; ++axis) {
        const glm::mat3 stress = tangentOf(
          linearized,
          law,
          unitStrain(axis, gradient[column], centre[column])
        );
        const glm::mat3 tangentDeviator = deviatorOf(stress);
        const float tangentMean = traceOf(stress) / 3.0f;

        for (uint32_t row = 0u; row != 8u; ++row) {
          trial.tangent.block[row * 8u + column][static_cast<int>(axis)] +=
            weight
            * pointForce(
              tangentDeviator,
              tangentMean,
              gradient[row],
              centre[row]
            );
        }
      }
    }
  }

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    trial.force[corner] = -(trial.rotation * internal[corner]);
  }

  if (rotationDerivative) {
    addRotationTerm(deformed, centre, trial.rotation, trial.force);
  }

  return trial;
}

TetPointTrial tetPointTrial(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
  const ElasticParams& elastic,
  const PlasticParams& plastic,
  const objectmodel::PlasticPoint& committed,
  float margin,
  ElementRotation rotation,
  bool returnMapping,
  bool tangent
) {
  TetPointTrial made;

  const Lame moduli = lame(elastic);
  const std::array<glm::vec3, 4> gradient = shapeGradients(tet.restInverse);

  made.tangent.gradient = gradient;
  made.tangent.first = moduli.first;
  made.tangent.second = moduli.second;

  const TetStrain measured = tetStrain(deformed, rest, tet, gradient, rotation);
  const PointLaw law = pointLaw(moduli, plastic);
  const PointResponse response =
    returnMap(measured.strain, committed, law, returnMapping);

  made.rotation = measured.rotation;
  made.point = response.next;
  made.energy =
    tet.volume * 0.5f
    * contract(response.stress, measured.strain - response.next.strain);
  made.plasticWork = tet.volume * response.plasticWork;
  made.yielding = response.plastic ? 1u : 0u;

  const bool loading = response.plastic && response.excess > margin;

  made.loading = loading ? 1u : 0u;

  const PointResponse linearized =
    loading || !response.plastic ? response : PointResponse{};

  const glm::mat3 responseDeviator = deviatorOf(response.stress);
  const float responseMean = traceOf(response.stress) / 3.0f;

  for (uint32_t row = 0u; row != 4u; ++row) {
    const glm::vec3 perVolume =
      pointForce(responseDeviator, responseMean, gradient[row], gradient[row]);

    made.force[row] = -(made.rotation * (tet.volume * perVolume));
  }

  if (!tangent) {
    return made;
  }

  for (uint32_t column = 0u; column != 4u; ++column) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      const glm::mat3 stress = tangentOf(
        linearized,
        law,
        unitStrain(axis, gradient[column], gradient[column])
      );
      const glm::mat3 tangentDeviator = deviatorOf(stress);
      const float tangentMean = traceOf(stress) / 3.0f;

      for (uint32_t row = 0u; row != 4u; ++row) {
        made.tangent.block[row * 4u + column][static_cast<int>(axis)] =
          tet.volume
          * pointForce(
            tangentDeviator,
            tangentMean,
            gradient[row],
            gradient[row]
          );
      }
    }
  }

  return made;
}

namespace {

glm::mat3 crossMatrix(const glm::vec3& v) {
  return glm::mat3(
    glm::vec3(0.0f, v.z, -v.y),
    glm::vec3(-v.z, 0.0f, v.x),
    glm::vec3(v.y, -v.x, 0.0f)
  );
}

glm::mat3 timesCross(const glm::mat3& m, const glm::vec3& v) {
  return glm::mat3(
    v.z * m[1] - v.y * m[2],
    v.x * m[2] - v.z * m[0],
    v.y * m[0] - v.x * m[1]
  );
}

template<size_t CORNERS>
InitialStress rotationDerivative(
  const std::array<glm::vec3, CORNERS>& deformed,
  const std::array<glm::vec3, CORNERS>& gradient,
  const std::array<glm::vec3, CORNERS>& offset,
  const std::array<glm::mat3, CORNERS * CORNERS>& block,
  const glm::mat3& rotation,
  const std::array<glm::vec3, CORNERS>& force
) {
  InitialStress made;

  glm::mat3 deformation(0.0f);

  for (size_t corner = 0u; corner != CORNERS; ++corner) {
    deformation += glm::outerProduct(deformed[corner], gradient[corner]);
  }

  const Spread spread = spreadOf(deformation, rotation);

  if (!spread.held) {
    return made;
  }

  const glm::mat3 inverse = glm::transpose(rotation);
  const glm::mat3 spin = rotation * spread.inverse * inverse;
  const float handed = glm::determinant(rotation) < 0.0f ? -1.0f : 1.0f;

  for (size_t row = 0u; row != CORNERS; ++row) {
    glm::mat3 twist(0.0f);

    for (size_t column = 0u; column != CORNERS; ++column) {
      twist += timesCross(block[row * CORNERS + column], offset[column]);
    }

    made.twist[row] =
      rotation * twist * inverse + handed * crossMatrix(force[row]);
    made.turn[row] = timesCross(spin, handed * (rotation * gradient[row]));
  }

  return made;
}

}

InitialStress initialStress(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
  const TetStiffness& stiffness,
  const glm::mat3& rotation,
  const std::array<glm::vec3, 4>& force
) {
  return rotationDerivative(
    deformed,
    shapeGradients(tet.restInverse),
    corotatedOffsets(deformed, rest, rotation),
    stiffness.block,
    rotation,
    force
  );
}

InitialStress initialStress(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const objectmodel::PreparedHex& hex,
  const HexStiffness& stiffness,
  const glm::mat3& rotation,
  const std::array<glm::vec3, 8>& force
) {
  return rotationDerivative(
    deformed,
    hexGradients(glm::vec3(0.0f), hex.halfExtents),
    centredOffsets(deformed, rest, rotation),
    stiffness.block,
    rotation,
    force
  );
}

void applyInitialStress(
  const InitialStress& stress,
  std::span<const glm::vec3> direction,
  std::span<glm::vec3> result
) noexcept {
  glm::vec3 turned(0.0f);
  glm::vec3 twisted(0.0f);

  for (size_t corner = 0u; corner != direction.size(); ++corner) {
    turned += stress.turn[corner] * direction[corner];
    twisted += direction[corner] * stress.twist[corner];
  }

  for (size_t corner = 0u; corner != result.size(); ++corner) {
    result[corner] +=
      0.5f * (stress.twist[corner] * turned + twisted * stress.turn[corner]);
  }
}

}
