#include "jacobi.h"

#include <array>
#include <cmath>
#include <cstdint>

namespace talus::dynamics {

namespace {

constexpr uint32_t sweeps = 8u;
constexpr std::array<std::array<int, 2>, 3> planes{{{0, 1}, {0, 2}, {1, 2}}};

}

SymmetricEigen jacobiEigen(const glm::dmat3& matrix) {
  std::array<std::array<double, 3>, 3> a{};
  std::array<std::array<double, 3>, 3> v{};

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      a[row][column] = 0.5 * (matrix[column][row] + matrix[row][column]);
      v[row][column] = row == column ? 1.0 : 0.0;
    }
  }

  for (uint32_t sweep = 0u; sweep != sweeps; ++sweep) {
    for (const std::array<int, 2>& plane : planes) {
      const int p = plane[0];
      const int q = plane[1];
      const double apq = a[p][q];

      if (apq == 0.0) {
        continue;
      }

      const double theta = (a[q][q] - a[p][p]) / (2.0 * apq);
      const double t = (theta >= 0.0 ? 1.0 : -1.0)
                       / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
      const double c = 1.0 / std::sqrt(t * t + 1.0);
      const double s = t * c;
      const int r = 3 - p - q;
      const double arp = a[r][p];
      const double arq = a[r][q];

      a[p][p] -= t * apq;
      a[q][q] += t * apq;
      a[p][q] = 0.0;
      a[q][p] = 0.0;
      a[r][p] = c * arp - s * arq;
      a[p][r] = a[r][p];
      a[r][q] = s * arp + c * arq;
      a[q][r] = a[r][q];

      for (int row = 0; row != 3; ++row) {
        const double vp = v[row][p];
        const double vq = v[row][q];

        v[row][p] = c * vp - s * vq;
        v[row][q] = s * vp + c * vq;
      }
    }
  }

  SymmetricEigen result;

  for (int column = 0; column != 3; ++column) {
    result.values[column] = a[column][column];

    for (int row = 0; row != 3; ++row) {
      result.vectors[column][row] = v[row][column];
    }
  }

  return result;
}

}
