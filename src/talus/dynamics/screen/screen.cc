#include "screen.h"

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include <algorithm>
#include <cmath>
#include <limits>

namespace talus::dynamics {

struct ScreenFactor {
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  std::vector<uint8_t> held;
  bool anchored = false;
};

namespace {

constexpr double screenShift = 1.0e-9;
constexpr double anchorScale = 1.0e6;

}

RestBody restBodyOf(const talus::PreparedMesh& mesh) {
  RestBody rest;
  glm::dvec3 low(std::numeric_limits<double>::infinity());
  glm::dvec3 high(-std::numeric_limits<double>::infinity());

  rest.mass.resize(mesh.restPositions.size());

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::dvec3 position(mesh.restPositions[node]);

    rest.mass[node] = mesh.nodeMass[node];
    rest.totalMass += rest.mass[node];
    rest.centre += rest.mass[node] * position;
    low = glm::min(low, position);
    high = glm::max(high, position);
  }

  rest.centre /= rest.totalMass;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rest.centre;

    rest.inertia +=
      rest.mass[node]
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  Eigen::Matrix3d tensor;

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      tensor(row, column) = rest.inertia[column][row];
    }
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solved(tensor);
  Eigen::Matrix3d axes = solved.eigenvectors();

  if (axes.determinant() < 0.0) {
    axes.col(2) = -axes.col(2);
  }

  glm::dmat3 turn(1.0);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      turn[column][row] = axes(row, column);
    }
  }

  const glm::dvec3 size = high - low;

  rest.axes = glm::quat_cast(turn);
  rest.moments = glm::dvec3(
    solved.eigenvalues()(0),
    solved.eigenvalues()(1),
    solved.eigenvalues()(2)
  );
  rest.thickness = std::min({size.x, size.y, size.z});

  return rest;
}

std::shared_ptr<const ScreenFactor> factorScreen(
  const talus::PreparedMesh& mesh,
  std::span<const Moduli> moduli,
  std::span<const uint32_t> held
) {
  const size_t nodes = mesh.restPositions.size();
  const Eigen::Index size = static_cast<Eigen::Index>(3u * nodes);
  std::vector<Eigen::Triplet<double>> entries;
  std::vector<double> diagonal(3u * nodes, 0.0);

  entries.reserve(mesh.hexes.size() * 576u + 3u * nodes);

  for (size_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    const auto stiffness =
      restStiffness(glm::dvec3(hex.halfExtents), hex.volume, moduli[cell]);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      for (uint32_t other = 0u; other != 8u; ++other) {
        for (int column = 0; column != 3; ++column) {
          for (int row = 0; row != 3; ++row) {
            const int i = static_cast<int>(3u * hex.nodes[corner]) + row;
            const int j = static_cast<int>(3u * hex.nodes[other]) + column;
            const double value = stiffness[corner][other][column][row];

            entries.emplace_back(i, j, value);

            if (i == j) {
              diagonal[static_cast<size_t>(i)] += value;
            }
          }
        }
      }
    }
  }

  const double largest = *std::max_element(diagonal.begin(), diagonal.end());
  double heaviest = 0.0;

  for (const float mass : mesh.nodeMass) {
    heaviest = std::max(heaviest, static_cast<double>(mass));
  }

  auto factor = std::make_shared<ScreenFactor>();

  factor->held.assign(nodes, 0u);

  for (size_t node = 0u; node != nodes; ++node) {
    if (!mesh.anchored.empty() && mesh.anchored[node] != 0u) {
      factor->held[node] = 1u;
    }
  }

  for (const uint32_t node : held) {
    factor->held[node] = 1u;
  }

  for (size_t node = 0u; node != nodes; ++node) {
    const bool anchored = factor->held[node] != 0u;

    factor->anchored = factor->anchored || anchored;

    for (int axis = 0; axis != 3; ++axis) {
      const int i = static_cast<int>(3u * node) + axis;

      entries.emplace_back(
        i,
        i,
        anchored ? anchorScale * largest
                 : screenShift * largest / heaviest * mesh.nodeMass[node]
      );
    }
  }

  Eigen::SparseMatrix<double> matrix(size, size);

  matrix.setFromTriplets(entries.begin(), entries.end());
  factor->solver.compute(matrix);

  return factor;
}

ScreenResult screenLoad(
  const ScreenFactor& factor,
  const RestBody& rest,
  const talus::PreparedMesh& mesh,
  std::span<const glm::dvec3> nodeForces
) {
  const size_t nodes = rest.mass.size();
  glm::dvec3 force(0.0);
  glm::dvec3 torque(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rest.centre;

    force += nodeForces[node];
    torque += glm::cross(arm, nodeForces[node]);
  }

  const glm::dvec3 linear =
    factor.anchored ? glm::dvec3(0.0) : force / rest.totalMass;
  const glm::dvec3 angular =
    factor.anchored ? glm::dvec3(0.0) : glm::inverse(rest.inertia) * torque;
  Eigen::VectorXd load(static_cast<Eigen::Index>(3u * nodes));

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rest.centre;
    const bool anchored = factor.held[node] != 0u;
    const glm::dvec3 relieved =
      anchored ? glm::dvec3(0.0)
               : nodeForces[node]
                   - rest.mass[node] * (linear + glm::cross(angular, arm));

    for (int axis = 0; axis != 3; ++axis) {
      load(static_cast<Eigen::Index>(3u * node) + axis) = relieved[axis];
    }
  }

  const Eigen::VectorXd solved = factor.solver.solve(load);
  ScreenResult result;

  result.displacement.resize(nodes);

  for (size_t node = 0u; node != nodes; ++node) {
    const Eigen::Index base = static_cast<Eigen::Index>(3u * node);

    result.displacement[node] =
      glm::dvec3(solved(base), solved(base + 1), solved(base + 2));
  }

  if (!factor.anchored) {
    glm::dvec3 shift(0.0);
    glm::dvec3 moment(0.0);

    for (size_t node = 0u; node != nodes; ++node) {
      shift += rest.mass[node] * result.displacement[node];
    }

    shift /= rest.totalMass;

    for (size_t node = 0u; node != nodes; ++node) {
      const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rest.centre;

      moment +=
        rest.mass[node] * glm::cross(arm, result.displacement[node] - shift);
    }

    const glm::dvec3 turn = glm::inverse(rest.inertia) * moment;

    for (size_t node = 0u; node != nodes; ++node) {
      const glm::dvec3 arm = glm::dvec3(mesh.restPositions[node]) - rest.centre;

      result.displacement[node] -= shift + glm::cross(turn, arm);
    }
  }

  for (const glm::dvec3& displacement : result.displacement) {
    result.dent = std::max(result.dent, glm::length(displacement));
  }

  result.dent /= rest.thickness;

  return result;
}

}
