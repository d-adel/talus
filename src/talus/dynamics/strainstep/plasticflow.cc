#include "plasticflow.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace talus::dynamics {

namespace {

constexpr double GAUSS_OFFSET = 0.5773502691896258;

const talus::PhysicalMaterial& laneOf(const LatticeBody& lattice, uint32_t cell) {
  const size_t lane = lattice.mesh->hexes[cell].material;

  return lattice.materials[std::min(lane, lattice.materials.size() - 1u)];
}

Eigen::Matrix3d leftStretch(const glm::dmat3& elastic) {
  Eigen::Matrix3d left;

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      double total = 0.0;

      for (int k = 0; k != 3; ++k) {
        total += elastic[k][row] * elastic[k][column];
      }

      left(row, column) = total;
    }
  }

  return left;
}

double frobenius(const glm::dmat3& m) {
  double total = 0.0;

  for (int column = 0; column != 3; ++column) {
    total += glm::dot(m[column], m[column]);
  }

  return std::sqrt(total);
}

double oneNorm(const glm::dmat3& m) {
  double total = 0.0;

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      total += std::abs(m[column][row]);
    }
  }

  return total;
}

ElementRest authoredRest(const LatticeBody& lattice, uint32_t cell) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  const talus::PreparedHex& hex = mesh.hexes[cell];
  std::array<glm::dvec3, 8> corners{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    corners[corner] = glm::dvec3(mesh.restPositions[hex.nodes[corner]]);
  }

  return cornerRest(corners, static_cast<double>(hex.volume));
}

std::array<glm::dvec3, 4> hourglassSlopes(const glm::dvec3& at) {
  return {
    glm::dvec3(at.y, at.x, 0.0),
    glm::dvec3(0.0, at.z, at.y),
    glm::dvec3(at.z, 0.0, at.x),
    glm::dvec3(at.y * at.z, at.x * at.z, at.x * at.y)
  };
}

glm::dmat3 fittedJacobian(const ElementRest& rest, uint32_t point) {
  const std::array<glm::dvec3, 4> slopes = hourglassSlopes(GAUSS_OFFSET * cornerSigns(point));
  glm::dmat3 jacobian = rest.shape;

  for (uint32_t mode = 0u; mode != 4u; ++mode) {
    jacobian += glm::outerProduct(rest.hourglass[mode], slopes[mode]) / 8.0;
  }

  return jacobian;
}

}

double vonMises(const glm::dmat3& elastic, double shear) {
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(leftStretch(elastic));
  const Eigen::Vector3d values = eigen.eigenvalues();

  if (!(values.minCoeff() > 0.0)) {
    return 0.0;
  }

  const Eigen::Vector3d strain = 0.5 * values.array().log();
  const Eigen::Vector3d deviator = strain.array() - strain.mean();

  return std::sqrt(1.5) * 2.0 * shear * deviator.norm();
}

std::optional<ReturnMap> returnMap(const glm::dmat3& elastic, double shear, double yieldStress) {
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(leftStretch(elastic));

  if (eigen.info() != Eigen::Success) {
    return std::nullopt;
  }

  const Eigen::Vector3d values = eigen.eigenvalues();

  if (!(values.minCoeff() > 0.0)) {
    return std::nullopt;
  }

  const Eigen::Vector3d strain = 0.5 * values.array().log();
  const Eigen::Vector3d deviator = strain.array() - strain.mean();
  const double size = deviator.norm();
  const double stress = std::sqrt(1.5) * 2.0 * shear * size;

  if (!(stress > yieldStress) || size <= 1.0e-15) {
    return std::nullopt;
  }

  const Eigen::Vector3d change = deviator * (yieldStress / stress - 1.0);
  const Eigen::Matrix3d correction =
    eigen.eigenvectors() * change.array().exp().matrix().asDiagonal() * eigen.eigenvectors().transpose();
  glm::dmat3 stretch(1.0);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      stretch[column][row] = correction(row, column);
    }
  }

  ReturnMap mapped;

  mapped.elastic = stretch * elastic;
  mapped.trial = stress;
  mapped.strain = (stress - yieldStress) / (3.0 * shear);

  return mapped;
}

double flowLattice(LatticeBody& lattice) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  bool yields = false;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    yields = yields || std::isfinite(laneOf(lattice, cell).yieldStress);
  }

  if (!yields) {
    return 0.0;
  }

  const size_t points = 8u * mesh.hexes.size();

  if (lattice.plastic.size() != points) {
    lattice.plastic.assign(points, glm::dmat3(1.0));
    lattice.hardening.assign(points, 0.0);
  }

  if (lattice.hardening.size() != points) {
    return 0.0;
  }

  double largest = 0.0;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PhysicalMaterial& material = laneOf(lattice, cell);

    if (!std::isfinite(material.yieldStress)) {
      continue;
    }

    const ElementMaterial moduli = elementMaterial(material);
    const double bulk = moduli.first + 2.0 * moduli.shear / 3.0;
    const double ratio = static_cast<double>(material.hardeningModulus) / (3.0 * moduli.shear);
    const bool crushable = std::isfinite(material.crushStress) && material.crushLimit < 1.0f;
    const double crush = static_cast<double>(material.crushStress) / bulk;
    const GaussGradients& gradients = lattice.gaussGradients[lattice.gaussGradientsOf[cell]];
    const talus::PreparedHex& hex = mesh.hexes[cell];

    for (uint32_t point = 0u; point != 8u; ++point) {
      const size_t at = 8u * cell + point;
      glm::dmat3 deformation(0.0);

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        deformation += glm::outerProduct(lattice.position[hex.nodes[corner]], gradients[point][corner]);
      }

      if (!(glm::determinant(deformation) > 0.0)) {
        continue;
      }

      const glm::dmat3 plastic = lattice.plastic[at];
      const glm::dmat3 elastic = deformation * plastic;
      const double start = static_cast<double>(material.yieldStress)
                           + static_cast<double>(material.hardeningModulus) * lattice.hardening[at];
      const double room = crushable
                            ? -std::log(glm::determinant(plastic) * static_cast<double>(material.crushLimit))
                            : 0.0;
      const bool squeezed =
        crushable && room > 0.0 && glm::determinant(elastic) < std::exp(-crush);
      const std::optional<ReturnMap> trial = returnMap(elastic, moduli.shear, start);
      const double yieldNow = trial ? (start + ratio * trial->trial) / (1.0 + ratio) : start;
      const std::optional<ReturnMap> mapped =
        trial ? returnMap(elastic, moduli.shear, yieldNow) : std::nullopt;
      const glm::dmat3 after = mapped ? mapped->elastic : elastic;
      const double volume = glm::determinant(after);
      const double compaction =
        squeezed && volume > 0.0 ? std::min(-crush - std::log(volume), room) : 0.0;

      if (!mapped && !(compaction > 0.0)) {
        continue;
      }

      const double grow = compaction > 0.0 ? std::exp(compaction / 3.0) : 1.0;
      glm::dmat3 next = glm::inverse(deformation) * (grow * after);
      const double kept = glm::determinant(next);
      const double cap = material.plasticMax;

      if (std::isfinite(cap) && kept > 0.0) {
        const glm::dmat3 stretch = glm::inverse(next) - glm::dmat3(1.0);
        const double size = frobenius(stretch);

        if (size > cap) {
          const glm::dmat3 capped = glm::dmat3(1.0) + (cap / size) * stretch;
          const double cappedVolume = glm::determinant(capped);

          if (!(cappedVolume > 1.0e-6)) {
            continue;
          }

          next = std::cbrt(kept * cappedVolume) * glm::inverse(capped);
        }
      }

      const double change = oneNorm(next - plastic);

      if (!(change > 1.0e-12)) {
        continue;
      }

      largest = std::max(largest, change);
      lattice.plastic[at] = next;
      lattice.hardening[at] += mapped ? mapped->strain : 0.0;
    }
  }

  return largest;
}

bool flowed(const LatticeBody& lattice) {
  return std::any_of(lattice.plastic.begin(), lattice.plastic.end(), [](const glm::dmat3& inverse) {
    return inverse != glm::dmat3(1.0);
  });
}

ElementRest plasticRest(const LatticeBody& lattice, uint32_t cell) {
  const ElementRest authored = authoredRest(lattice, cell);

  if (lattice.plastic.size() < 8u * (static_cast<size_t>(cell) + 1u)) {
    return authored;
  }

  std::array<glm::dmat3, 8> stretch{};
  glm::dmat3 mean(0.0);
  double volume = 0.0;

  for (uint32_t point = 0u; point != 8u; ++point) {
    stretch[point] = glm::inverse(lattice.plastic[8u * cell + point]);
    mean += stretch[point] / 8.0;
    volume += glm::determinant(stretch[point]) / 8.0;
  }

  ElementRest rest;
  const double factor = std::sqrt(3.0) / 2.0;

  rest.shape = mean * authored.shape;
  rest.volume = authored.volume * volume;
  rest.hourglass = {};

  for (uint32_t point = 0u; point != 8u; ++point) {
    const glm::dmat3 target = stretch[point] * authored.shape;
    const glm::dvec3 s = cornerSigns(point);

    rest.hourglass[0] += factor * (target * glm::dvec3(s.y, s.x, 0.0));
    rest.hourglass[1] += factor * (target * glm::dvec3(0.0, s.z, s.y));
    rest.hourglass[2] += factor * (target * glm::dvec3(s.z, 0.0, s.x));
    rest.hourglass[3] += target * glm::dvec3(s.y * s.z, s.x * s.z, s.x * s.y);
  }

  double fitted = 0.0;

  for (uint32_t point = 0u; point != 8u; ++point) {
    fitted += glm::determinant(fittedJacobian(rest, point)) / 8.0;
  }

  const double ratio = volume * glm::determinant(authored.shape) / fitted;
  const double size = std::isfinite(ratio) && ratio > 0.0 ? std::cbrt(ratio) : 1.0;

  rest.shape *= size;

  for (glm::dvec3& hourglass : rest.hourglass) {
    hourglass *= size;
  }

  return rest;
}

bool fitHolds(const ElementRest& rest) {
  for (uint32_t point = 0u; point != 8u; ++point) {
    if (!(glm::determinant(fittedJacobian(rest, point)) > 0.0)) {
      return false;
    }
  }

  return true;
}

void writeFitted(LatticeBody& lattice, uint32_t cell, const ElementRest& rest) {
  if (lattice.plastic.size() < 8u * (static_cast<size_t>(cell) + 1u)) {
    return;
  }

  const ElementRest authored = authoredRest(lattice, cell);

  for (uint32_t point = 0u; point != 8u; ++point) {
    lattice.plastic[8u * cell + point] = authored.shape * glm::inverse(fittedJacobian(rest, point));
  }
}

glm::dquat latticeTurn(const LatticeBody& lattice) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  glm::dvec3 centre(0.0);
  glm::dvec3 restCentre(0.0);
  double total = 0.0;

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const double mass = mesh.nodeMass[node];

    centre += mass * lattice.position[node];
    restCentre += mass * glm::dvec3(mesh.restPositions[node]);
    total += mass;
  }

  if (!(total > 0.0)) {
    return glm::dquat(1.0, 0.0, 0.0, 0.0);
  }

  centre /= total;
  restCentre /= total;

  Eigen::Matrix3d moment = Eigen::Matrix3d::Zero();

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const double mass = mesh.nodeMass[node];
    const glm::dvec3 now = lattice.position[node] - centre;
    const glm::dvec3 then = glm::dvec3(mesh.restPositions[node]) - restCentre;

    for (int row = 0; row != 3; ++row) {
      for (int column = 0; column != 3; ++column) {
        moment(row, column) += mass * now[row] * then[column];
      }
    }
  }

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(moment, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d left = svd.matrixU();
  Eigen::Matrix3d turn = left * svd.matrixV().transpose();

  if (turn.determinant() < 0.0) {
    left.col(2) = -left.col(2);
    turn = left * svd.matrixV().transpose();
  }

  glm::dmat3 rotation(1.0);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      rotation[column][row] = turn(row, column);
    }
  }

  return glm::normalize(glm::quat_cast(rotation));
}

std::vector<glm::dvec3> plasticShape(const LatticeBody& lattice, uint32_t iterations) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::vector<std::array<glm::dvec3, 8>> shapes(mesh.hexes.size());
  std::vector<glm::dquat> turns(mesh.hexes.size(), latticeTurn(lattice));

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const ElementRest rest = plasticRest(lattice, cell);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      glm::dvec3 offset = rest.shape * cornerSigns(corner);

      for (uint32_t mode = 0u; mode != 4u; ++mode) {
        offset += hourglassSign(mode, corner) * rest.hourglass[mode] / 8.0;
      }

      shapes[cell][corner] = offset;
    }
  }

  std::vector<glm::dvec3> shape = lattice.position;
  std::vector<glm::dvec3> sum(shape.size());
  std::vector<double> count(shape.size());

  for (uint32_t iteration = 0u; iteration != iterations; ++iteration) {
    std::fill(sum.begin(), sum.end(), glm::dvec3(0.0));
    std::fill(count.begin(), count.end(), 0.0);

    for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
      const talus::PreparedHex& hex = mesh.hexes[cell];
      glm::dvec3 centre(0.0);
      glm::dmat3 moment(0.0);

      for (const uint32_t node : hex.nodes) {
        centre += shape[node] / 8.0;
      }

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        moment += glm::outerProduct(shape[hex.nodes[corner]] - centre, shapes[cell][corner]);
      }

      turns[cell] = rotationOf(moment, turns[cell]);

      const glm::dmat3 turn = glm::mat3_cast(turns[cell]);

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        sum[hex.nodes[corner]] += centre + turn * shapes[cell][corner];
        count[hex.nodes[corner]] += 1.0;
      }
    }

    for (size_t node = 0u; node != shape.size(); ++node) {
      if (count[node] > 0.0) {
        shape[node] = sum[node] / count[node];
      }
    }
  }

  return shape;
}

}
