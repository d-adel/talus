#include <catch2/catch_test_macros.hpp>

#include <talus/destruction/stress/factor.h>

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

namespace {

using talus::destruction::BlockPattern;
using talus::destruction::MatrixBlock;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

double uniform(std::mt19937_64& engine) {
  return static_cast<double>(engine() >> 11u) * 0x1.0p-52 - 1.0;
}

Matrix6 randomMatrix(std::mt19937_64& engine) {
  Matrix6 matrix;

  for (int column = 0; column != 6; ++column) {
    for (int row = 0; row != 6; ++row) {
      matrix(row, column) = uniform(engine);
    }
  }

  return matrix;
}

MatrixBlock blockOf(const Matrix6& matrix) {
  MatrixBlock block{};

  Eigen::Map<Matrix6>(block.data()) = matrix;

  return block;
}

struct System {
  int32_t columns = 0;
  std::vector<std::array<int32_t, 2>> links;
  std::vector<Matrix6> coupling;
  Eigen::MatrixXd dense;
};

System randomSystem(int32_t columns, size_t links, int32_t parts, uint64_t seed) {
  std::mt19937_64 engine(seed);
  System system{columns, {}, {}, Eigen::MatrixXd::Zero(6 * columns, 6 * columns)};
  const int32_t width = columns / parts;

  for (int32_t column = 0; column != columns; ++column) {
    const Matrix6 spread = randomMatrix(engine);

    system.dense.block<6, 6>(6 * column, 6 * column) += spread.transpose() * spread + Matrix6::Identity();
  }

  while (system.links.size() != links) {
    const int32_t part = static_cast<int32_t>(engine() % static_cast<uint64_t>(parts));
    const int32_t first = part * width + static_cast<int32_t>(engine() % static_cast<uint64_t>(width));
    const int32_t second = part * width + static_cast<int32_t>(engine() % static_cast<uint64_t>(width));

    if (first == second) {
      continue;
    }

    const Matrix6 left = randomMatrix(engine);
    const Matrix6 right = randomMatrix(engine);

    system.links.push_back({first, second});
    system.coupling.push_back(left.transpose() * right);
    system.dense.block<6, 6>(6 * first, 6 * first) += left.transpose() * left;
    system.dense.block<6, 6>(6 * second, 6 * second) += right.transpose() * right;
    system.dense.block<6, 6>(6 * first, 6 * second) += left.transpose() * right;
    system.dense.block<6, 6>(6 * second, 6 * first) += right.transpose() * left;
  }

  return system;
}

std::vector<double> valuesOf(const System& system, const BlockPattern& pattern) {
  std::vector<double> values(static_cast<size_t>(pattern.panel.back()), 0.0);

  for (int32_t column = 0; column != system.columns; ++column) {
    const int32_t place = pattern.place[static_cast<size_t>(column)];

    talus::destruction::addBlock(pattern, values, place, place, blockOf(system.dense.block<6, 6>(6 * column, 6 * column)));
  }

  for (size_t link = 0u; link != system.links.size(); ++link) {
    talus::destruction::addBlock(
      pattern,
      values,
      pattern.place[static_cast<size_t>(system.links[link][0])],
      pattern.place[static_cast<size_t>(system.links[link][1])],
      blockOf(system.coupling[link])
    );
  }

  return values;
}

}

TEST_CASE("a block factorization solves random symmetric positive definite block systems as a dense Cholesky does", "[destruction]") {
  struct Case {
    int32_t columns;
    size_t links;
    int32_t parts;
    uint64_t seed;
  };

  for (const Case& test : {
         Case{1, 0u, 1, 11u},
         Case{2, 1u, 1, 12u},
         Case{2, 4u, 1, 13u},
         Case{9, 12u, 1, 14u},
         Case{24, 40u, 1, 15u},
         Case{40, 30u, 4, 16u},
         Case{60, 150u, 1, 17u},
         Case{60, 400u, 2, 18u},
       }) {
    INFO("columns " << test.columns << " links " << test.links << " parts " << test.parts);

    const System system = randomSystem(test.columns, test.links, test.parts, test.seed);
    const BlockPattern pattern = talus::destruction::blockPattern(system.columns, system.links);
    std::vector<double> values = valuesOf(system, pattern);
    std::mt19937_64 engine(test.seed + 1000u);
    Eigen::VectorXd load(6 * test.columns);

    for (Eigen::Index entry = 0; entry != load.size(); ++entry) {
      load(entry) = uniform(engine);
    }

    REQUIRE(pattern.place.size() == static_cast<size_t>(test.columns));
    REQUIRE(!talus::destruction::factorBlocks(pattern, values).has_value());

    std::vector<double> solution(6u * static_cast<size_t>(test.columns), 0.0);

    for (int32_t column = 0; column != test.columns; ++column) {
      for (int32_t axis = 0; axis != 6; ++axis) {
        solution[6u * static_cast<size_t>(pattern.place[static_cast<size_t>(column)]) + static_cast<size_t>(axis)] =
          load(6 * column + axis);
      }
    }

    talus::destruction::solveBlocks(pattern, values, solution);

    const Eigen::VectorXd reference = system.dense.llt().solve(load);
    Eigen::VectorXd found(6 * test.columns);

    for (int32_t column = 0; column != test.columns; ++column) {
      for (int32_t axis = 0; axis != 6; ++axis) {
        found(6 * column + axis) =
          solution[6u * static_cast<size_t>(pattern.place[static_cast<size_t>(column)]) + static_cast<size_t>(axis)];
      }
    }

    CHECK((found - reference).norm() <= 1.0e-12 * reference.norm());
  }
}

TEST_CASE("a block factorization refuses a matrix that is not positive definite and names the first block it cannot factorize", "[destruction]") {
  const System system = randomSystem(6, 8u, 1, 21u);
  const BlockPattern pattern = talus::destruction::blockPattern(system.columns, system.links);

  for (const double scale : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
    INFO("column 3's diagonal block scaled by " << scale);

    System changed = system;

    changed.dense.block<6, 6>(18, 18) *= scale;

    std::vector<double> values = valuesOf(changed, pattern);
    const std::optional<int32_t> failed = talus::destruction::factorBlocks(pattern, values);

    REQUIRE(failed.has_value());
    CHECK(*failed == pattern.place[3]);
  }
}
