#include "lowestmode.h"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace talus::dynamics {

namespace {

constexpr uint32_t ITERATIONS = 300u;
constexpr double TOLERANCE = 1.0e-12;

double massDot(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& mass) {
  return (a.array() * mass.array() * b.array()).sum();
}

void deflate(Eigen::VectorXd& x, const std::vector<Eigen::VectorXd>& rigid, const Eigen::VectorXd& mass) {
  for (const Eigen::VectorXd& mode : rigid) {
    x -= massDot(mode, x, mass) * mode;
  }
}

}

double lowestAngular(
  std::span<const StrainElement> elements,
  std::span<const double> mass,
  std::span<const uint8_t> pinned,
  std::span<const glm::dvec3> rest
) {
  const size_t nodes = mass.size();
  std::vector<Eigen::Index> index(3u * nodes, -1);
  std::vector<double> weights;
  std::vector<size_t> owner;
  bool held = false;

  for (size_t node = 0u; node != nodes; ++node) {
    if (pinned[node] != 0u) {
      held = true;
      continue;
    }

    if (!(mass[node] > 0.0)) {
      continue;
    }

    for (size_t axis = 0u; axis != 3u; ++axis) {
      index[3u * node + axis] = static_cast<Eigen::Index>(weights.size());
      weights.push_back(mass[node]);
      owner.push_back(3u * node + axis);
    }
  }

  const Eigen::Index size = static_cast<Eigen::Index>(weights.size());

  if (size == 0) {
    return std::numeric_limits<double>::infinity();
  }

  std::vector<Eigen::Triplet<double>> entries;

  for (const StrainElement& element : elements) {
    for (uint32_t row = 0u; row != ELEMENT_ROWS + 1u; ++row) {
      const double stiffness =
        row < ELEMENT_ROWS ? element.stiffness[row] : element.volumeStiffness;

      if (!(stiffness > 0.0)) {
        continue;
      }

      std::array<double, 24> line{};

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        const glm::dvec3 slope =
          row < ELEMENT_ROWS ? rowJacobian(element, corner, row) : element.gradient[corner];

        line[3u * corner] = slope.x;
        line[3u * corner + 1u] = slope.y;
        line[3u * corner + 2u] = slope.z;
      }

      for (uint32_t i = 0u; i != 24u; ++i) {
        const Eigen::Index p = index[3u * element.nodes[i / 3u] + i % 3u];

        if (p < 0 || line[i] == 0.0) {
          continue;
        }

        for (uint32_t j = 0u; j != 24u; ++j) {
          const Eigen::Index q = index[3u * element.nodes[j / 3u] + j % 3u];

          if (q >= 0 && line[j] != 0.0) {
            entries.emplace_back(p, q, stiffness * line[i] * line[j]);
          }
        }
      }
    }
  }

  Eigen::SparseMatrix<double> stiffness(size, size);

  stiffness.setFromTriplets(entries.begin(), entries.end());

  const Eigen::VectorXd massVector =
    Eigen::Map<const Eigen::VectorXd>(weights.data(), size);
  std::vector<Eigen::VectorXd> rigid;

  if (!held) {
    glm::dvec3 centre(0.0);
    double total = 0.0;

    for (Eigen::Index i = 0; i < size; i += 3) {
      const size_t node = owner[static_cast<size_t>(i)] / 3u;

      centre += weights[static_cast<size_t>(i)] * rest[node];
      total += weights[static_cast<size_t>(i)];
    }

    centre /= total;

    for (int mode = 0; mode != 6; ++mode) {
      Eigen::VectorXd shape(size);

      for (Eigen::Index i = 0; i < size; ++i) {
        const size_t node = owner[static_cast<size_t>(i)] / 3u;
        const int axis = static_cast<int>(owner[static_cast<size_t>(i)] % 3u);
        glm::dvec3 direction(0.0);

        if (mode < 3) {
          direction[mode] = 1.0;
        } else {
          glm::dvec3 spin(0.0);

          spin[mode - 3] = 1.0;
          direction = glm::cross(spin, rest[node] - centre);
        }

        shape[i] = direction[axis];
      }

      deflate(shape, rigid, massVector);

      const double norm = std::sqrt(massDot(shape, shape, massVector));

      if (norm > 0.0) {
        rigid.push_back(shape / norm);
      }
    }
  }

  double traceK = 0.0;

  for (Eigen::Index i = 0; i < size; ++i) {
    traceK += stiffness.coeff(i, i);
  }

  const double shift = held ? 0.0 : 1.0e-6 * traceK / massVector.sum();
  Eigen::SparseMatrix<double> shifted = stiffness;

  for (Eigen::Index i = 0; i < size; ++i) {
    shifted.coeffRef(i, i) += shift * massVector[i];
  }

  const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor(shifted);

  if (factor.info() != Eigen::Success) {
    return 0.0;
  }

  Eigen::VectorXd x(size);

  for (Eigen::Index i = 0; i < size; ++i) {
    x[i] = std::sin(1.7 * static_cast<double>(i) + 0.3) + 0.5;
  }

  double previous = std::numeric_limits<double>::infinity();
  double value = std::numeric_limits<double>::infinity();

  for (uint32_t iteration = 0u; iteration != ITERATIONS; ++iteration) {
    deflate(x, rigid, massVector);
    x /= std::sqrt(massDot(x, x, massVector));

    Eigen::VectorXd z = factor.solve(x.cwiseProduct(massVector));

    deflate(z, rigid, massVector);
    value = z.dot(stiffness * z) / massDot(z, z, massVector);
    x = z;

    if (std::abs(value - previous) <= TOLERANCE * std::abs(value)) {
      break;
    }

    previous = value;
  }

  return std::sqrt(std::max(value, 0.0));
}

}
