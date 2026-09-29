#include "factor.ih"

namespace talus::deformable {

namespace {

const objectmodel::PhysicalMaterial& laneOf(
  std::span<const objectmodel::PhysicalMaterial> materials,
  uint32_t lane
) {
  static const objectmodel::PhysicalMaterial DEFAULT{};

  return lane < materials.size() ? materials[lane] : DEFAULT;
}

struct PreciseLame {
  double first = 0.0;
  double second = 0.0;
};

PreciseLame preciseLame(const objectmodel::PhysicalMaterial& material) {
  const double young = static_cast<double>(material.youngsModulus);
  const double nu =
    std::clamp(static_cast<double>(material.poissonsRatio), 0.0, 0.49);

  return {
    young * nu / ((1.0 + nu) * (1.0 - 2.0 * nu)),
    young / (2.0 * (1.0 + nu))
  };
}

template<size_t Count>
std::array<glm::dmat3, Count * Count> preciseTurned(
  std::array<glm::dmat3, Count * Count> blocks,
  const glm::mat3& rotation,
  float step
) {
  const glm::dmat3 turn(rotation);
  const glm::dmat3 back = glm::transpose(turn);
  const double scale = static_cast<double>(step) * static_cast<double>(step);

  for (glm::dmat3& block : blocks) {
    block = scale * (turn * block * back);
  }

  return blocks;
}

std::array<glm::dmat3, 16> preciseTetBlocks(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::PreparedTet& tet,
  const objectmodel::PhysicalMaterial& material,
  const glm::mat3& rotation,
  float step
) {
  const glm::dvec3 first(mesh.restPositions[tet.nodes[0]]);
  const glm::dmat3 shape(
    glm::dvec3(mesh.restPositions[tet.nodes[1]]) - first,
    glm::dvec3(mesh.restPositions[tet.nodes[2]]) - first,
    glm::dvec3(mesh.restPositions[tet.nodes[3]]) - first
  );
  const double volume = glm::determinant(shape) / 6.0;
  const glm::dmat3 rows = glm::transpose(glm::inverse(shape));
  const std::array<glm::dvec3, 4> gradient{
    -(rows[0] + rows[1] + rows[2]),
    rows[0],
    rows[1],
    rows[2]
  };
  const PreciseLame moduli = preciseLame(material);

  std::array<glm::dmat3, 16> blocks{};

  for (size_t row = 0u; row != 4u; ++row) {
    for (size_t column = 0u; column != 4u; ++column) {
      const glm::dvec3& a = gradient[row];
      const glm::dvec3& b = gradient[column];

      blocks[row * 4u + column] =
        volume
        * (moduli.first * glm::outerProduct(a, b)
           + moduli.second * glm::outerProduct(b, a)
           + moduli.second * glm::dot(a, b) * glm::dmat3(1.0));
    }
  }

  return preciseTurned<4u>(blocks, rotation, step);
}

std::array<glm::dvec3, 8> preciseHexGradients(
  const glm::dvec3& at,
  const glm::dvec3& half
) {
  std::array<glm::dvec3, 8> gradient{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 sign(
      (corner & 1u) != 0u ? 1.0 : -1.0,
      (corner & 2u) != 0u ? 1.0 : -1.0,
      (corner & 4u) != 0u ? 1.0 : -1.0
    );

    gradient[corner] = glm::dvec3(
      0.125 * sign.x * (1.0 + sign.y * at.y) * (1.0 + sign.z * at.z) / half.x,
      0.125 * sign.y * (1.0 + sign.x * at.x) * (1.0 + sign.z * at.z) / half.y,
      0.125 * sign.z * (1.0 + sign.x * at.x) * (1.0 + sign.y * at.y) / half.z
    );
  }

  return gradient;
}

void preciseAccumulate(
  const std::array<glm::dvec3, 8>& gradient,
  double weight,
  double lambda,
  double mu,
  std::array<glm::dmat3, 64>& blocks
) {
  for (uint32_t row = 0u; row != 8u; ++row) {
    for (uint32_t column = 0u; column != 8u; ++column) {
      const glm::dvec3& a = gradient[row];
      const glm::dvec3& b = gradient[column];

      glm::dmat3 local(0.0);

      for (int i = 0; i != 3; ++i) {
        for (int j = 0; j != 3; ++j) {
          local[j][i] = lambda * a[i] * b[j] + mu * a[j] * b[i];
        }

        local[i][i] += mu * glm::dot(a, b);
      }

      blocks[row * 8u + column] += weight * local;
    }
  }
}

std::array<glm::dmat3, 64> preciseHexBlocks(
  const objectmodel::PreparedHex& hex,
  const objectmodel::PhysicalMaterial& material,
  const glm::mat3& rotation,
  float step
) {
  constexpr double GAUSS = 0.5773502691896258;

  const glm::dvec3 half(hex.halfExtents);
  const double volume = 8.0 * half.x * half.y * half.z;
  const PreciseLame moduli = preciseLame(material);
  const double bulk = moduli.first + 2.0 * moduli.second / 3.0;
  const double deviatoric = -2.0 * moduli.second / 3.0;

  std::array<glm::dmat3, 64> blocks{};

  for (uint32_t point = 0u; point != 8u; ++point) {
    const glm::dvec3 at(
      (point & 1u) != 0u ? GAUSS : -GAUSS,
      (point & 2u) != 0u ? GAUSS : -GAUSS,
      (point & 4u) != 0u ? GAUSS : -GAUSS
    );

    preciseAccumulate(
      preciseHexGradients(at, half),
      volume * 0.125,
      deviatoric,
      moduli.second,
      blocks
    );
  }

  preciseAccumulate(
    preciseHexGradients(glm::dvec3(0.0), half),
    volume,
    bulk,
    0.0,
    blocks
  );

  return preciseTurned<8u>(blocks, rotation, step);
}

glm::dmat3 pseudoInverse(const glm::dmat3& held, double floor) {
  constexpr uint32_t SWEEPS = 64u;
  constexpr std::array<std::array<int, 2>, 3> PAIRS{{{0, 1}, {0, 2}, {1, 2}}};

  std::array<std::array<double, 3>, 3> reduced{};
  std::array<std::array<double, 3>, 3> basis{};

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      reduced[row][column] = held[column][row];
      basis[row][column] = row == column ? 1.0 : 0.0;
    }
  }

  for (uint32_t sweep = 0u; sweep != SWEEPS; ++sweep) {
    double off = 0.0;
    double size = 0.0;

    for (int row = 0; row != 3; ++row) {
      size += reduced[row][row] * reduced[row][row];

      for (int column = row + 1; column != 3; ++column) {
        off += reduced[row][column] * reduced[row][column];
      }
    }

    if (!(off > 1.0e-30 * size)) {
      break;
    }

    for (const std::array<int, 2>& pair : PAIRS) {
      const int p = pair[0];
      const int q = pair[1];

      if (reduced[p][q] == 0.0) {
        continue;
      }

      const double theta =
        (reduced[q][q] - reduced[p][p]) / (2.0 * reduced[p][q]);
      const double tangent =
        (theta >= 0.0 ? 1.0 : -1.0)
        / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
      const double cosine = 1.0 / std::sqrt(tangent * tangent + 1.0);
      const double sine = tangent * cosine;

      for (int k = 0; k != 3; ++k) {
        const double first = reduced[k][p];
        const double second = reduced[k][q];

        reduced[k][p] = cosine * first - sine * second;
        reduced[k][q] = sine * first + cosine * second;
      }

      for (int k = 0; k != 3; ++k) {
        const double first = reduced[p][k];
        const double second = reduced[q][k];

        reduced[p][k] = cosine * first - sine * second;
        reduced[q][k] = sine * first + cosine * second;
      }

      for (int k = 0; k != 3; ++k) {
        const double first = basis[k][p];
        const double second = basis[k][q];

        basis[k][p] = cosine * first - sine * second;
        basis[k][q] = sine * first + cosine * second;
      }
    }
  }

  glm::dmat3 made(0.0);

  for (int axis = 0; axis != 3; ++axis) {
    const double value = reduced[axis][axis];

    if (!(value > floor)) {
      continue;
    }

    for (int row = 0; row != 3; ++row) {
      for (int column = 0; column != 3; ++column) {
        made[column][row] += basis[row][axis] * basis[column][axis] / value;
      }
    }
  }

  return made;
}

template<size_t Count, typename Stiffness>
std::array<glm::mat3, Count * Count> blocksOf(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const Stiffness& stiffness
) {
  std::array<glm::mat3, Count * Count> blocks{};
  std::array<glm::vec3, Count> unit{};
  std::array<glm::vec3, Count> column{};

  for (size_t corner = 0u; corner != Count; ++corner) {
    for (int axis = 0; axis != 3; ++axis) {
      unit.fill(glm::vec3(0.0f));
      unit[corner][axis] = 1.0f;

      applyDamping(fit, rotation, stiffness, unit, column);

      for (size_t row = 0u; row != Count; ++row) {
        blocks[row * Count + corner][axis] = column[row];
      }
    }
  }

  return blocks;
}

}

struct SystemFactor::Held {
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Lower,
                       Eigen::AMDOrdering<int>>
    solver;
  Eigen::SparseMatrix<double> system;
  std::vector<Eigen::Triplet<double>> entries;
  uint32_t nodes = 0u;
  uint64_t stored = 0u;
  bool analyzed = false;
  uint64_t born = 0u;
  bool ready = false;
  FactorReference reference;
};

namespace {

void gather(
  SystemFactor::Held& held,
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState* state,
  std::span<const objectmodel::PhysicalMaterial> materials,
  float step,
  float inertia,
  float damping
) {
  const uint32_t nodes = static_cast<uint32_t>(mesh.restPositions.size());

  held.entries.clear();
  held.entries.reserve(
    mesh.tets.size() * 144u + mesh.hexes.size() * 576u + nodes * 3u
  );

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const objectmodel::PreparedTet& tet = mesh.tets[element];

    glm::mat3 rotation(1.0f);
    TetStiffness stiffness{};
    float weight = 0.0f;
    std::array<glm::mat3, 16> damped{};
    std::array<glm::dmat3, 16> precise{};

    if (state != nullptr) {
      const objectmodel::PhysicalMaterial& held2 =
        laneOf(materials, tet.material);

      stiffness = tetStiffness(
        tet,
        ElasticParams{held2.youngsModulus, held2.poissonsRatio, held2.density}
      );
      rotation = state->committed[element].rotation;
      weight = damping * stiffnessDamping(held2);
      precise = preciseTetBlocks(mesh, tet, held2, rotation, step);
    }

    if (weight != 0.0f) {
      const std::array<glm::vec3, 4> corners{
        state->position[tet.nodes[0]],
        state->position[tet.nodes[1]],
        state->position[tet.nodes[2]],
        state->position[tet.nodes[3]]
      };

      damped = dampingBlocks(rigidFit(corners), rotation, stiffness);
    }

    for (int a = 0; a != 4; ++a) {
      const uint32_t first = tet.nodes[a];

      if (mesh.anchored[first] != 0u) {
        continue;
      }

      for (int b = 0; b != 4; ++b) {
        const uint32_t second = tet.nodes[b];

        if (mesh.anchored[second] != 0u) {
          continue;
        }

        glm::dmat3 block = precise[static_cast<size_t>(a * 4 + b)];

        if (weight != 0.0f) {
          block += glm::dmat3(weight * damped[static_cast<size_t>(a * 4 + b)]);
        }

        for (int r = 0; r != 3; ++r) {
          for (int c = 0; c != 3; ++c) {
            held.entries.emplace_back(
              static_cast<int>(first) * 3 + r,
              static_cast<int>(second) * 3 + c,
              block[c][r]
            );
          }
        }
      }
    }
  }

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const objectmodel::PreparedHex& hex = mesh.hexes[element];

    glm::mat3 rotation(1.0f);
    HexStiffness stiffness{};
    float weight = 0.0f;
    std::array<glm::mat3, 64> damped{};
    std::array<glm::dmat3, 64> precise{};

    if (state != nullptr) {
      const objectmodel::PhysicalMaterial& picked =
        laneOf(materials, hex.material);

      stiffness = hexStiffness(
        hex,
        ElasticParams{
          picked.youngsModulus, picked.poissonsRatio, picked.density
        }
      );
      rotation = state->committed[element].rotation;
      weight = damping * stiffnessDamping(picked);
      precise = preciseHexBlocks(hex, picked, rotation, step);
    }

    if (weight != 0.0f) {
      std::array<glm::vec3, 8> corners{};

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        corners[corner] = state->position[hex.nodes[corner]];
      }

      damped = dampingBlocks(rigidFit(corners), rotation, stiffness);
    }

    for (uint32_t a = 0u; a != 8u; ++a) {
      const uint32_t first = hex.nodes[a];

      if (mesh.anchored[first] != 0u) {
        continue;
      }

      for (uint32_t b = 0u; b != 8u; ++b) {
        const uint32_t second = hex.nodes[b];

        if (mesh.anchored[second] != 0u) {
          continue;
        }

        glm::dmat3 block = precise[a * 8u + b];

        if (weight != 0.0f) {
          block += glm::dmat3(weight * damped[a * 8u + b]);
        }

        for (int r = 0; r != 3; ++r) {
          for (int c = 0; c != 3; ++c) {
            held.entries.emplace_back(
              static_cast<int>(first) * 3 + r,
              static_cast<int>(second) * 3 + c,
              block[c][r]
            );
          }
        }
      }
    }
  }

  for (uint32_t node = 0u; node != nodes; ++node) {
    const double diagonal =
      mesh.anchored[node] != 0u || state == nullptr
        ? 1.0
        : static_cast<double>(inertia * mesh.nodeMass[node]);

    for (int axis = 0; axis != 3; ++axis) {
      held.entries.emplace_back(
        static_cast<int>(node) * 3 + axis,
        static_cast<int>(node) * 3 + axis,
        diagonal
      );
    }
  }

  held.system.setFromTriplets(held.entries.begin(), held.entries.end());
  held.system.makeCompressed();
}

}

SystemFactor::SystemFactor() : m_held(std::make_unique<Held>()) {}

uint32_t SystemFactor::rows() const noexcept {
  return m_held->nodes * 3u;
}

uint64_t SystemFactor::systemNonZeros() const noexcept {
  return static_cast<uint64_t>(m_held->system.nonZeros());
}

uint64_t SystemFactor::factorNonZeros() const noexcept {
  return m_held->ready
           ? static_cast<uint64_t>(
               m_held->solver.matrixL().nestedExpression().nonZeros()
             )
           : 0u;
}

double SystemFactor::entry(uint32_t row, uint32_t column) const {
  if (m_held == nullptr || row >= rows() || column >= rows()) {
    return 0.0;
  }

  return m_held->system.coeff(
    static_cast<Eigen::Index>(row),
    static_cast<Eigen::Index>(column)
  );
}

SystemFactor::~SystemFactor() = default;

SystemFactor::SystemFactor(SystemFactor&&) noexcept = default;

SystemFactor& SystemFactor::operator=(SystemFactor&&) noexcept = default;

bool SystemFactor::analyzed() const noexcept {
  return m_held != nullptr && m_held->analyzed;
}

bool SystemFactor::empty() const noexcept {
  return m_held == nullptr || !m_held->ready;
}

uint32_t SystemFactor::nodes() const noexcept {
  return m_held == nullptr ? 0u : m_held->nodes;
}

uint64_t SystemFactor::stored() const noexcept {
  return m_held == nullptr ? 0u : m_held->stored;
}

uint64_t SystemFactor::generation() const noexcept {
  return m_held == nullptr ? 0u : m_held->born;
}

const FactorReference& SystemFactor::reference() const noexcept {
  static const FactorReference EMPTY{};

  return m_held == nullptr ? EMPTY : m_held->reference;
}

void analyzeSystem(
  SystemFactor& factor,
  const objectmodel::PreparedMesh& mesh
) {
  const uint32_t nodes = static_cast<uint32_t>(mesh.restPositions.size());
  const int rows = static_cast<int>(nodes) * 3;

  factor.m_held->nodes = nodes;
  factor.m_held->system.resize(rows, rows);

  gather(*factor.m_held, mesh, nullptr, {}, 0.0f, 1.0f, 0.0f);

  factor.m_held->solver.analyzePattern(factor.m_held->system);
  factor.m_held->analyzed =
    factor.m_held->solver.info() == Eigen::Success;
  factor.m_held->ready = false;
}

void refreshFactor(
  SystemFactor& factor,
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& state,
  std::span<const objectmodel::PhysicalMaterial> materials,
  float step,
  float inertia,
  float damping
) {
  if (factor.m_held->analyzed
      && factor.m_held->nodes
           != static_cast<uint32_t>(mesh.restPositions.size())) {
    factor.m_held->analyzed = false;
  }

  if (!factor.m_held->analyzed) {
    analyzeSystem(factor, mesh);
  }

  gather(*factor.m_held, mesh, &state, materials, step, inertia, damping);

  factor.m_held->solver.factorize(factor.m_held->system);
  factor.m_held->ready = factor.m_held->solver.info() == Eigen::Success;
  factor.m_held->stored = static_cast<uint64_t>(
    factor.m_held->solver.matrixL().nestedExpression().nonZeros()
  );

  ++factor.m_held->born;

  factor.m_held->reference.step = step;
  factor.m_held->reference.inertia = inertia;
  factor.m_held->reference.damping = laneDamping(materials, damping);
  factor.m_held->reference.positions = state.position;
  factor.m_held->reference.centroid = glm::vec3(0.0f);

  float total = 0.0f;

  for (uint32_t node = 0u; node != factor.m_held->nodes; ++node) {
    total += mesh.nodeMass[node];
    factor.m_held->reference.centroid +=
      mesh.nodeMass[node] * state.position[node];
  }

  if (total > 0.0f) {
    factor.m_held->reference.centroid /= total;
  }
}

glm::mat3 bodyRotation(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& state,
  const FactorReference& reference
) {
  if (reference.positions.size() != state.position.size()) {
    return glm::mat3(1.0f);
  }

  glm::vec3 centre(0.0f);
  float total = 0.0f;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    total += mesh.nodeMass[node];
    centre += mesh.nodeMass[node] * state.position[node];
  }

  if (total > 0.0f) {
    centre /= total;
  }

  glm::mat3 correlation(0.0f);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 now = state.position[node] - centre;
    const glm::vec3 before = reference.positions[node] - reference.centroid;

    correlation += mesh.nodeMass[node] * glm::outerProduct(now, before);
  }

  return polarRotation(correlation);
}

void applyFactor(
  const SystemFactor& factor,
  const glm::mat3& frame,
  const objectmodel::PreparedMesh& mesh,
  std::vector<glm::vec3>& vector,
  FactorTiming* timing
) {
  if (factor.empty()) {
    return;
  }

  const auto packBegan = std::chrono::steady_clock::now();

  const glm::mat3 inverse = glm::transpose(frame);
  const uint32_t nodes = factor.m_held->nodes;

  Eigen::VectorXd packed(static_cast<int>(nodes) * 3);

  for (uint32_t node = 0u; node != nodes; ++node) {
    const glm::vec3 turned = inverse * vector[node];
    const int base = static_cast<int>(node) * 3;

    packed[base] = static_cast<double>(turned.x);
    packed[base + 1] = static_cast<double>(turned.y);
    packed[base + 2] = static_cast<double>(turned.z);
  }

  const auto solveBegan = std::chrono::steady_clock::now();

  const Eigen::VectorXd solved = factor.m_held->solver.solve(packed);

  const auto unpackBegan = std::chrono::steady_clock::now();

  for (uint32_t node = 0u; node != nodes; ++node) {
    const int base = static_cast<int>(node) * 3;
    const glm::vec3 raw(
      static_cast<float>(solved[base]),
      static_cast<float>(solved[base + 1]),
      static_cast<float>(solved[base + 2])
    );

    vector[node] =
      mesh.anchored[node] != 0u ? glm::vec3(0.0f) : frame * raw;
  }

  if (timing != nullptr) {
    const auto done = std::chrono::steady_clock::now();

    timing->packSeconds +=
      std::chrono::duration<double>(solveBegan - packBegan).count();
    timing->solveSeconds +=
      std::chrono::duration<double>(unpackBegan - solveBegan).count();
    timing->unpackSeconds +=
      std::chrono::duration<double>(done - unpackBegan).count();
    ++timing->applications;
  }
}

void applyFactorBatch(
  const SystemFactor& factor,
  const glm::mat3& frame,
  const objectmodel::PreparedMesh& mesh,
  std::span<std::vector<glm::vec3>> vectors
) {
  if (factor.empty() || vectors.empty()) {
    return;
  }

  const glm::mat3 inverse = glm::transpose(frame);
  const uint32_t nodes = factor.m_held->nodes;
  const int rows = static_cast<int>(nodes) * 3;
  const int columns = static_cast<int>(vectors.size());

  Eigen::MatrixXd packed(rows, columns);

  for (int column = 0; column != columns; ++column) {
    const std::vector<glm::vec3>& given =
      vectors[static_cast<size_t>(column)];

    for (uint32_t node = 0u; node != nodes; ++node) {
      const glm::vec3 turned = inverse * given[node];
      const int base = static_cast<int>(node) * 3;

      packed(base, column) = static_cast<double>(turned.x);
      packed(base + 1, column) = static_cast<double>(turned.y);
      packed(base + 2, column) = static_cast<double>(turned.z);
    }
  }

  const Eigen::MatrixXd solved = factor.m_held->solver.solve(packed);

  for (int column = 0; column != columns; ++column) {
    std::vector<glm::vec3>& given = vectors[static_cast<size_t>(column)];

    for (uint32_t node = 0u; node != nodes; ++node) {
      const int base = static_cast<int>(node) * 3;
      const glm::vec3 raw(
        static_cast<float>(solved(base, column)),
        static_cast<float>(solved(base + 1, column)),
        static_cast<float>(solved(base + 2, column))
      );

      given[node] = mesh.anchored[node] != 0u ? glm::vec3(0.0f) : frame * raw;
    }
  }
}

float stiffnessDamping(const objectmodel::PhysicalMaterial& material) noexcept {
  const float held = material.stiffnessDamping;

  return std::isfinite(held) && held > 0.0f ? held : 0.0f;
}

std::vector<float> laneDamping(
  std::span<const objectmodel::PhysicalMaterial> materials,
  float scale
) {
  std::vector<float> made;

  if (scale == 0.0f) {
    return made;
  }

  made.reserve(materials.size());

  for (const objectmodel::PhysicalMaterial& material : materials) {
    made.push_back(scale * stiffnessDamping(material));
  }

  return made;
}

RigidFit rigidFit(std::span<const glm::vec3> corners) {
  RigidFit fit;

  const size_t count = std::min(corners.size(), fit.arms.size());

  if (count == 0u) {
    return fit;
  }

  glm::dvec3 centre(0.0);

  for (size_t corner = 0u; corner != count; ++corner) {
    centre += glm::dvec3(corners[corner]);
  }

  centre /= static_cast<double>(count);

  glm::dmat3 inertia(0.0);

  for (size_t corner = 0u; corner != count; ++corner) {
    fit.arms[corner] = glm::vec3(glm::dvec3(corners[corner]) - centre);

    const glm::dvec3 arm(fit.arms[corner]);

    inertia +=
      glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm);
  }

  const double third = (inertia[0][0] + inertia[1][1] + inertia[2][2]) / 3.0;

  fit.inverse = glm::mat3(
    glm::determinant(inertia) > 1.0e-6 * third * third * third
      ? glm::inverse(inertia)
      : pseudoInverse(inertia, 1.0e-6 * third)
  );

  return fit;
}

void removeRigid(const RigidFit& fit, std::span<glm::vec3> values) {
  const size_t count = std::min(values.size(), fit.arms.size());

  if (count == 0u) {
    return;
  }

  glm::vec3 mean(0.0f);
  glm::vec3 moment(0.0f);

  for (size_t corner = 0u; corner != count; ++corner) {
    mean += values[corner];
    moment += glm::cross(fit.arms[corner], values[corner]);
  }

  mean /= static_cast<float>(count);

  const glm::vec3 spin = fit.inverse * moment;

  for (size_t corner = 0u; corner != count; ++corner) {
    values[corner] -= mean + glm::cross(spin, fit.arms[corner]);
  }
}

void applyDamping(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const TetStiffness& stiffness,
  const std::array<glm::vec3, 4>& direction,
  std::array<glm::vec3, 4>& result
) {
  std::array<glm::vec3, 4> relative = direction;

  removeRigid(fit, relative);
  applyStiffness(relative, rotation, stiffness, result);
  removeRigid(fit, result);
}

namespace {

using Lanes = std::array<__m128, 8>;

struct Columns {
  __m128 x;
  __m128 y;
  __m128 z;
};

static_assert(sizeof(glm::vec3) == 16u && alignof(glm::vec3) == 16u);
static_assert(sizeof(glm::mat3) == 48u);

__m128 loadThree(const glm::vec3& value) {
  return _mm_and_ps(
    _mm_load_ps(&value.x),
    _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1))
  );
}

void storeThree(glm::vec3& value, __m128 lanes) {
  _mm_store_ps(&value.x, lanes);
}

Columns columnsOf(const glm::mat3& matrix) {
  return Columns{
    loadThree(matrix[0]),
    loadThree(matrix[1]),
    loadThree(matrix[2])
  };
}

__m128 times(const Columns& matrix, __m128 value) {
  return _mm_add_ps(
    _mm_add_ps(
      _mm_mul_ps(
        matrix.x,
        _mm_shuffle_ps(value, value, _MM_SHUFFLE(0, 0, 0, 0))
      ),
      _mm_mul_ps(
        matrix.y,
        _mm_shuffle_ps(value, value, _MM_SHUFFLE(1, 1, 1, 1))
      )
    ),
    _mm_mul_ps(matrix.z, _mm_shuffle_ps(value, value, _MM_SHUFFLE(2, 2, 2, 2)))
  );
}

__m128 crossOf(__m128 first, __m128 second) {
  const __m128 firstYzx = _mm_shuffle_ps(first, first, _MM_SHUFFLE(3, 0, 2, 1));
  const __m128 firstZxy = _mm_shuffle_ps(first, first, _MM_SHUFFLE(3, 1, 0, 2));
  const __m128 secondYzx =
    _mm_shuffle_ps(second, second, _MM_SHUFFLE(3, 0, 2, 1));
  const __m128 secondZxy =
    _mm_shuffle_ps(second, second, _MM_SHUFFLE(3, 1, 0, 2));

  return _mm_sub_ps(
    _mm_mul_ps(firstYzx, secondZxy),
    _mm_mul_ps(secondYzx, firstZxy)
  );
}

void removeRigidLanes(const RigidFit& fit, Lanes& values) {
  Lanes arms{};
  __m128 mean = _mm_setzero_ps();
  __m128 moment = _mm_setzero_ps();

  for (size_t corner = 0u; corner != 8u; ++corner) {
    arms[corner] = loadThree(fit.arms[corner]);
    mean = _mm_add_ps(mean, values[corner]);
    moment = _mm_add_ps(moment, crossOf(arms[corner], values[corner]));
  }

  mean = _mm_div_ps(mean, _mm_set1_ps(8.0f));

  const __m128 spin = times(columnsOf(fit.inverse), moment);

  for (size_t corner = 0u; corner != 8u; ++corner) {
    values[corner] =
      _mm_sub_ps(values[corner], _mm_add_ps(mean, crossOf(spin, arms[corner])));
  }
}

}

void applyDamping(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const HexStiffness& stiffness,
  const std::array<glm::vec3, 8>& direction,
  std::array<glm::vec3, 8>& result
) {
  const Columns turn = columnsOf(rotation);
  const Columns back = columnsOf(glm::transpose(rotation));

  Lanes relative{};

  for (size_t corner = 0u; corner != 8u; ++corner) {
    relative[corner] = loadThree(direction[corner]);
  }

  removeRigidLanes(fit, relative);

  Lanes unrotated{};

  for (size_t corner = 0u; corner != 8u; ++corner) {
    unrotated[corner] = times(back, relative[corner]);
  }

  Lanes produced{};

  for (size_t row = 0u; row != 8u; ++row) {
    __m128 sum = _mm_setzero_ps();

    for (size_t column = 0u; column != 8u; ++column) {
      sum = _mm_add_ps(
        sum,
        times(columnsOf(stiffness.block[row * 8u + column]), unrotated[column])
      );
    }

    produced[row] = times(turn, sum);
  }

  removeRigidLanes(fit, produced);

  for (size_t corner = 0u; corner != 8u; ++corner) {
    storeThree(result[corner], produced[corner]);
  }
}

void applyStiffnessDamping(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const HexStiffness& stiffness,
  const std::array<glm::vec3, 8>& direction,
  std::array<glm::vec3, 8>& stiff,
  std::array<glm::vec3, 8>& damped
) {
  const Columns turn = columnsOf(rotation);
  const Columns back = columnsOf(glm::transpose(rotation));

  Lanes loaded{};
  Lanes relative{};

  for (size_t corner = 0u; corner != 8u; ++corner) {
    loaded[corner] = loadThree(direction[corner]);
    relative[corner] = loaded[corner];
  }

  removeRigidLanes(fit, relative);

  Lanes unrotated{};
  Lanes projected{};

  for (size_t corner = 0u; corner != 8u; ++corner) {
    unrotated[corner] = times(back, loaded[corner]);
    projected[corner] = times(back, relative[corner]);
  }

  Lanes viscous{};

  for (size_t row = 0u; row != 8u; ++row) {
    __m128 sum = _mm_setzero_ps();
    __m128 held = _mm_setzero_ps();

    for (size_t column = 0u; column != 8u; ++column) {
      const Columns block = columnsOf(stiffness.block[row * 8u + column]);

      sum = _mm_add_ps(sum, times(block, unrotated[column]));
      held = _mm_add_ps(held, times(block, projected[column]));
    }

    storeThree(stiff[row], times(turn, sum));
    viscous[row] = times(turn, held);
  }

  removeRigidLanes(fit, viscous);

  for (size_t corner = 0u; corner != 8u; ++corner) {
    storeThree(damped[corner], viscous[corner]);
  }
}

std::array<glm::mat3, 16> dampingBlocks(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const TetStiffness& stiffness
) {
  return blocksOf<4u>(fit, rotation, stiffness);
}

std::array<glm::mat3, 64> dampingBlocks(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const HexStiffness& stiffness
) {
  return blocksOf<8u>(fit, rotation, stiffness);
}

}
