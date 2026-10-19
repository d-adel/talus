#include "solver.ih"

namespace talus::deformable {

namespace {

constexpr uint32_t NO_NODE = UINT32_MAX;

constexpr float FLOATING_TOLERANCE = 1.0e-5f;

constexpr uint32_t ANCHORED_READS = 3u;

constexpr uint32_t INVERTED_SUBSTEPS = 4u;

constexpr uint32_t ARC_SOLVES = 8u;

constexpr std::array<std::array<uint32_t, 4>, 6> KUHN{
  {{0u, 1u, 3u, 7u},
   {0u, 3u, 2u, 7u},
   {0u, 2u, 6u, 7u},
   {0u, 6u, 4u, 7u},
   {0u, 4u, 5u, 7u},
   {0u, 5u, 1u, 7u}}
};

uint32_t latticeIndex(const talus::field::SampledSdf& lattice, glm::uvec3 at) {
  return at.x + lattice.resolution.x * (at.y + lattice.resolution.y * at.z);
}

const talus::PhysicalMaterial& laneOf(
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t lane
) {
  static const talus::PhysicalMaterial DEFAULT{};

  return lane < materials.size() ? materials[lane] : DEFAULT;
}

ElasticParams elasticOf(
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t lane
) {
  const talus::PhysicalMaterial& held = laneOf(materials, lane);

  return {held.youngsModulus, held.poissonsRatio, held.density};
}

PlasticParams plasticOf(
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t lane
) {
  const talus::PhysicalMaterial& held = laneOf(materials, lane);

  return {held.yieldStress, held.hardeningModulus};
}

bool anyYields(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials
) {
  for (const talus::PreparedHex& hex : mesh.hexes) {
    if (yields(plasticOf(materials, hex.material))) {
      return true;
    }
  }

  for (const talus::PreparedTet& tet : mesh.tets) {
    if (yields(plasticOf(materials, tet.material))) {
      return true;
    }
  }

  return false;
}

glm::uvec3 cornerOffset(uint32_t corner) {
  return glm::uvec3(corner & 1u, (corner >> 1u) & 1u, (corner >> 2u) & 1u);
}

constexpr uint32_t FILL_SAMPLES = 4u;
float cellFill(
  const talus::field::SampledSdf& lattice,
  const glm::uvec3& cell
) {
  std::array<float, 8> corner{};
  bool touched = false;

  for (uint32_t at = 0u; at != 8u; ++at) {
    corner[at] =
      lattice.distances[latticeIndex(lattice, cell + cornerOffset(at))];
    touched = touched || corner[at] < 0.0f;
  }

  if (!touched) {
    return 0.0f;
  }

  uint32_t inside = 0u;

  for (uint32_t k = 0u; k != FILL_SAMPLES; ++k) {
    for (uint32_t j = 0u; j != FILL_SAMPLES; ++j) {
      for (uint32_t i = 0u; i != FILL_SAMPLES; ++i) {
        const glm::vec3 at =
          (glm::vec3(i, j, k) + 0.5f) / static_cast<float>(FILL_SAMPLES);

        float distance = 0.0f;

        for (uint32_t c = 0u; c != 8u; ++c) {
          const glm::vec3 side = glm::vec3(cornerOffset(c));
          const glm::vec3 weight = side * at + (1.0f - side) * (1.0f - at);

          distance += weight.x * weight.y * weight.z * corner[c];
        }

        inside += distance < 0.0f ? 1u : 0u;
      }
    }
  }

  return static_cast<float>(inside)
         / static_cast<float>(FILL_SAMPLES * FILL_SAMPLES * FILL_SAMPLES);
}

uint32_t cellLane(
  const talus::field::SampledSdf& lattice,
  const glm::uvec3& cell
) {
  std::array<uint32_t, 8> seen{};
  std::array<uint32_t, 8> counted{};

  uint32_t distinct = 0u;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const uint32_t lane =
      lattice.materials[latticeIndex(lattice, cell + cornerOffset(corner))];

    uint32_t slot = 0u;

    while (slot != distinct && seen[slot] != lane) {
      ++slot;
    }

    if (slot == distinct) {
      seen[distinct] = lane;
      counted[distinct] = 0u;
      ++distinct;
    }

    ++counted[slot];
  }

  uint32_t best = 0u;

  for (uint32_t slot = 1u; slot != distinct; ++slot) {
    if (counted[slot] > counted[best]) {
      best = slot;
    }
  }

  return seen[best];
}

bool supported(
  const glm::vec3& rest,
  const glm::mat4& localToWorld,
  std::span<const SupportVolume> supports
) {
  const glm::vec3 world = glm::vec3(localToWorld * glm::vec4(rest, 1.0f));

  for (const SupportVolume& volume : supports) {
    const glm::vec3 local =
      glm::vec3(volume.worldToSupport * glm::vec4(world, 1.0f));

    if (glm::all(glm::lessThanEqual(glm::abs(local), volume.halfExtents))) {
      return true;
    }
  }

  return false;
}

glm::vec3 samplePosition(
  const talus::field::SampledSdf& lattice,
  const glm::uvec3& at
) {
  return lattice.origin + lattice.voxelSize * glm::vec3(at);
}

constexpr std::array<std::array<int, 3>, 4> TET_FACES{
  {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}}
};

constexpr std::array<std::array<int, 2>, 3> FACE_EDGES{
  {{0, 1}, {0, 2}, {1, 2}}
};

bool beforeFace(const glm::uvec3& left, const glm::uvec3& right) {
  return std::tie(left.x, left.y, left.z) < std::tie(right.x, right.y, right.z);
}

bool sameFace(const glm::uvec3& left, const glm::uvec3& right) {
  return left == right;
}

bool beforeEdge(const glm::uvec2& left, const glm::uvec2& right) {
  return std::tie(left.x, left.y) < std::tie(right.x, right.y);
}

struct TetFace {
  glm::uvec3 sorted{0u};
  glm::uvec3 ordered{0u};
  uint32_t opposite = 0u;
  uint32_t lane = 0u;
};

bool beforeTetFace(const TetFace& left, const TetFace& right) {
  return beforeFace(left.sorted, right.sorted);
}

void collectSurface(
  talus::PreparedMesh& mesh,
  const talus::field::SampledSdf& lattice
) {
  std::vector<TetFace> faces;

  faces.reserve(mesh.tets.size() * TET_FACES.size() + mesh.hexes.size() * 12u);

  for (const talus::PreparedTet& tet : mesh.tets) {
    for (const std::array<int, 3>& pick : TET_FACES) {
      TetFace face;

      face.ordered =
        glm::uvec3(tet.nodes[pick[0]], tet.nodes[pick[1]], tet.nodes[pick[2]]);
      face.opposite = tet.nodes[6 - pick[0] - pick[1] - pick[2]];
      face.lane = tet.material;

      std::array<uint32_t, 3> triple{
        face.ordered.x,
        face.ordered.y,
        face.ordered.z
      };

      std::sort(triple.begin(), triple.end());

      face.sorted = glm::uvec3(triple[0], triple[1], triple[2]);

      faces.push_back(face);
    }
  }

  constexpr std::array<std::array<int, 5>, 6> HEX_FACES{
    {{0, 2, 6, 4, 1},
     {1, 3, 7, 5, 0},
     {0, 1, 5, 4, 2},
     {2, 3, 7, 6, 0},
     {0, 1, 3, 2, 4},
     {4, 5, 7, 6, 0}}
  };

  for (const talus::PreparedHex& hex : mesh.hexes) {
    for (const std::array<int, 5>& pick : HEX_FACES) {
      const std::array<uint32_t, 4> quad{
        hex.nodes[static_cast<size_t>(pick[0])],
        hex.nodes[static_cast<size_t>(pick[1])],
        hex.nodes[static_cast<size_t>(pick[2])],
        hex.nodes[static_cast<size_t>(pick[3])]
      };

      size_t lowest = 0u;

      for (size_t corner = 1u; corner != 4u; ++corner) {
        if (quad[corner] < quad[lowest]) {
          lowest = corner;
        }
      }

      const std::array<std::array<size_t, 3>, 2> split{
        {{lowest, (lowest + 1u) % 4u, (lowest + 2u) % 4u},
         {lowest, (lowest + 2u) % 4u, (lowest + 3u) % 4u}}
      };

      for (const std::array<size_t, 3>& triangle : split) {
        TetFace face;

        face.ordered =
          glm::uvec3(quad[triangle[0]], quad[triangle[1]], quad[triangle[2]]);
        face.opposite = hex.nodes[static_cast<size_t>(pick[4])];
        face.lane = hex.material;

        std::array<uint32_t, 3> triple{
          face.ordered.x,
          face.ordered.y,
          face.ordered.z
        };

        std::sort(triple.begin(), triple.end());

        face.sorted = glm::uvec3(triple[0], triple[1], triple[2]);

        faces.push_back(face);
      }
    }
  }

  std::sort(faces.begin(), faces.end(), beforeTetFace);

  for (size_t at = 0u; at != faces.size();) {
    size_t next = at + 1u;

    while (next != faces.size()
           && sameFace(faces[at].sorted, faces[next].sorted)) {
      ++next;
    }

    if (next - at == 1u) {
      const TetFace& face = faces[at];

      for (const std::array<int, 2>& pick : FACE_EDGES) {
        const uint32_t first = face.sorted[pick[0]];
        const uint32_t second = face.sorted[pick[1]];

        mesh.surfaceEdges.push_back(
          glm::uvec2(std::min(first, second), std::max(first, second))
        );
      }

      const glm::vec3& a = mesh.restPositions[face.ordered.x];
      const glm::vec3& b = mesh.restPositions[face.ordered.y];
      const glm::vec3& c = mesh.restPositions[face.ordered.z];

      const bool inward = glm::dot(
                            glm::cross(b - a, c - a),
                            mesh.restPositions[face.opposite] - a
                          )
                          > 0.0f;

      mesh.surfaceTriangles.push_back(
        inward ? glm::uvec3(face.ordered.x, face.ordered.z, face.ordered.y)
               : face.ordered
      );

      float thickness = 0.0f;

      for (int corner = 0; corner != 3; ++corner) {
        thickness -= talus::field::distance(
          lattice,
          mesh.restPositions[face.ordered[corner]]
        );
      }

      mesh.surfaceThickness.push_back(thickness / 3.0f);
      mesh.surfaceLane.push_back(face.lane);
    }

    at = next;
  }

  std::sort(mesh.surfaceEdges.begin(), mesh.surfaceEdges.end(), beforeEdge);

  mesh.surfaceEdges.erase(
    std::unique(mesh.surfaceEdges.begin(), mesh.surfaceEdges.end()),
    mesh.surfaceEdges.end()
  );
}

constexpr uint32_t EQUILIBRIUM_PASSES = 20u;

constexpr double FREE_MODE_GAP = 1.0e-9;

constexpr double FREE_AXIS_FLOOR = 1.0e-6;

constexpr float LOADING_MARGIN = 1.0e-3f;

constexpr double PLASTIC_REDUCTION = 1.0e-2;

bool plasticReduced(double squared, double opening) {
  return squared <= PLASTIC_REDUCTION * PLASTIC_REDUCTION * opening;
}

constexpr float PLASTIC_RESOLUTION = 8.0f;

constexpr uint32_t CONTACT_PLASTIC_PASSES = 40u;

struct Damping {
  std::vector<float> tau;
  std::vector<RigidFit> fits;
  std::vector<glm::mat3> blocks;
  std::span<const TetStiffness> tets;
  std::span<const HexStiffness> hexes;
  float scale = 0.0f;
  mutable std::vector<std::array<glm::vec3, 8>> produced;
};

struct Scratch {
  std::vector<glm::vec3> force;
  std::vector<glm::vec3> previous;
  std::vector<glm::vec3> right;
  std::vector<glm::mat3> diagonal;
  std::vector<glm::mat3> preconditioner;
  std::vector<glm::vec3> residual;
  std::vector<glm::vec3> direction;
  std::vector<glm::vec3> applied;
  std::vector<glm::vec3> conditioned;
  std::vector<glm::vec3> increment;
  std::vector<glm::vec3> restart;
  std::vector<glm::vec3> shifted;
  std::vector<glm::vec3> weighted;
  std::vector<glm::vec3> carried;
  std::vector<InitialStress> initialStress;
  std::span<const TetStiffness> stiffness;
  std::span<const HexStiffness> hexStiffnesses;
  std::vector<TetStiffness> tetWorking;
  std::vector<HexStiffness> hexWorking;
  bool stressed = false;
  Damping damping;
};

struct OperatorTiming {
  uint32_t applications = 0u;
  double massSeconds = 0.0;
  double tetSeconds = 0.0;
  double anchorSeconds = 0.0;
};

struct CgResult {
  uint32_t iterations = 0u;
  float relativeResidual = 0.0f;
  float startResidual = 0.0f;
  bool capped = false;
  bool resolved = false;
  double conditionSeconds = 0.0;
  FactorTiming factorTiming;
  OperatorTiming operatorTiming;
};

void sizeScratch(Scratch& scratch, size_t nodes) {
  scratch.force.assign(nodes, glm::vec3(0.0f));
  scratch.previous.assign(nodes, glm::vec3(0.0f));
  scratch.right.assign(nodes, glm::vec3(0.0f));
  scratch.diagonal.assign(nodes, glm::mat3(0.0f));
  scratch.preconditioner.assign(nodes, glm::mat3(0.0f));
  scratch.residual.assign(nodes, glm::vec3(0.0f));
  scratch.direction.assign(nodes, glm::vec3(0.0f));
  scratch.applied.assign(nodes, glm::vec3(0.0f));
  scratch.conditioned.assign(nodes, glm::vec3(0.0f));
  scratch.increment.assign(nodes, glm::vec3(0.0f));
  scratch.restart.assign(nodes, glm::vec3(0.0f));
  scratch.shifted.assign(nodes, glm::vec3(0.0f));
  scratch.weighted.assign(nodes, glm::vec3(0.0f));
  scratch.carried.assign(nodes, glm::vec3(0.0f));
}

bool refreshStiffness(
  ElementStiffness& held,
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials
) {
  bool same = held.mesh == static_cast<const void*>(&mesh)
              && held.tets == mesh.tets.size()
              && held.hexes == mesh.hexes.size()
              && held.nodes == mesh.restPositions.size()
              && held.elastic.size() == materials.size();

  for (size_t lane = 0u; same && lane != materials.size(); ++lane) {
    const glm::vec3 now(
      materials[lane].youngsModulus,
      materials[lane].poissonsRatio,
      materials[lane].density
    );

    same = held.elastic[lane] == now;
  }

  if (same) {
    return false;
  }

  held.mesh = static_cast<const void*>(&mesh);
  held.tets = mesh.tets.size();
  held.hexes = mesh.hexes.size();
  held.nodes = mesh.restPositions.size();
  held.elastic.assign(materials.size(), glm::vec3(0.0f));

  for (size_t lane = 0u; lane != materials.size(); ++lane) {
    held.elastic[lane] = glm::vec3(
      materials[lane].youngsModulus,
      materials[lane].poissonsRatio,
      materials[lane].density
    );
  }

  held.tet.clear();
  held.tet.reserve(mesh.tets.size());

  for (const talus::PreparedTet& tet : mesh.tets) {
    held.tet.push_back(tetStiffness(tet, elasticOf(materials, tet.material)));
  }

  held.hex.clear();
  held.hex.reserve(mesh.hexes.size());

  for (const talus::PreparedHex& hex : mesh.hexes) {
    held.hex.push_back(hexStiffness(hex, elasticOf(materials, hex.material)));
  }

  return true;
}

void bindStiffness(
  Scratch& scratch,
  const ElementStiffness& store,
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  bool consistent
) {
  if (!consistent || !anyYields(mesh, materials)) {
    scratch.tetWorking.clear();
    scratch.hexWorking.clear();
    scratch.stiffness = store.tet;
    scratch.hexStiffnesses = store.hex;

    return;
  }

  scratch.tetWorking = store.tet;
  scratch.hexWorking = store.hex;
  scratch.stiffness = scratch.tetWorking;
  scratch.hexStiffnesses = scratch.hexWorking;
}

struct IntegratorRule {
  float alphaMass = 0.0f;
  float alphaForce = 0.0f;
  float beta = 0.25f;
  float gamma = 0.5f;
};

IntegratorRule integratorRule(float spectral) {
  const float held = glm::clamp(spectral, 0.0f, 1.0f);

  IntegratorRule made;

  made.alphaMass = (2.0f * held - 1.0f) / (held + 1.0f);
  made.alphaForce = held / (held + 1.0f);
  made.gamma = 0.5f - made.alphaMass + made.alphaForce;
  made.beta = 0.25f * (1.0f - made.alphaMass + made.alphaForce)
              * (1.0f - made.alphaMass + made.alphaForce);

  return made;
}

struct EffectiveSystem {
  float inertia = 1.0f;
  float step = 0.0f;
  float mobility = 1.0f;
};

EffectiveSystem effectiveSystem(const IntegratorRule& rule, float step) {
  EffectiveSystem made;

  made.inertia = 1.0f - rule.alphaMass;
  made.step = std::sqrt((1.0f - rule.alphaForce) * rule.beta) * step;
  made.mobility = rule.beta;

  return made;
}

bool predictorInverts(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const IntegratorRule& rule,
  float inner,
  std::vector<glm::vec3>& weighted
) {
  const float carryScale = (0.5f - rule.beta) * inner * inner;

  for (size_t node = 0u; node != weighted.size(); ++node) {
    const glm::vec3 shifted = state.position[node]
                              + inner * state.velocity[node]
                              + carryScale * state.acceleration[node];

    weighted[node] = (1.0f - rule.alphaForce) * shifted
                     + rule.alphaForce * state.position[node];
  }

  for (const talus::PreparedTet& tet : mesh.tets) {
    const float det = glm::determinant(
      glm::mat3(
        weighted[tet.nodes[1]] - weighted[tet.nodes[0]],
        weighted[tet.nodes[2]] - weighted[tet.nodes[0]],
        weighted[tet.nodes[3]] - weighted[tet.nodes[0]]
      )
      * tet.restInverse
    );

    if (det <= 0.0f) {
      return true;
    }
  }

  for (const talus::PreparedHex& hex : mesh.hexes) {
    glm::mat3 gradientOf(0.0f);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::vec3 sign(
        (corner & 1u) != 0u ? 1.0f : -1.0f,
        (corner & 2u) != 0u ? 1.0f : -1.0f,
        (corner & 4u) != 0u ? 1.0f : -1.0f
      );

      gradientOf += glm::outerProduct(
        weighted[hex.nodes[corner]],
        0.125f * sign / hex.halfExtents
      );
    }

    if (glm::determinant(gradientOf) <= 0.0f) {
      return true;
    }
  }

  return false;
}

bool bindDamping(
  Damping& damping,
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const ElementStiffness& store
) {
  damping.tau.clear();
  damping.fits.clear();
  damping.blocks.clear();
  damping.tets = store.tet;
  damping.hexes = store.hex;

  if (
    std::none_of(
      materials.begin(),
      materials.end(),
      [](const talus::PhysicalMaterial& held) {
        return stiffnessDamping(held) > 0.0f;
      }
    )
  ) {
    return false;
  }

  const size_t tets = mesh.tets.size();

  damping.tau.assign(tets + mesh.hexes.size(), 0.0f);

  bool any = false;

  for (size_t element = 0u; element != tets; ++element) {
    damping.tau[element] =
      stiffnessDamping(laneOf(materials, mesh.tets[element].material));
    any = any || damping.tau[element] > 0.0f;
  }

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    damping.tau[tets + element] =
      stiffnessDamping(laneOf(materials, mesh.hexes[element].material));
    any = any || damping.tau[tets + element] > 0.0f;
  }

  if (!any) {
    damping.tau.clear();

    return false;
  }

  damping.fits.assign(damping.tau.size(), RigidFit{});

  return true;
}

void fitDamping(
  Damping& damping,
  const talus::PreparedMesh& mesh,
  const std::vector<glm::vec3>& at
) {
  const size_t tets = mesh.tets.size();

  for (size_t element = 0u; element != tets; ++element) {
    if (damping.tau[element] == 0.0f) {
      continue;
    }

    const glm::uvec4& held = mesh.tets[element].nodes;
    const std::array<glm::vec3, 4> corners{
      at[held[0]],
      at[held[1]],
      at[held[2]],
      at[held[3]]
    };

    damping.fits[element] = rigidFit(corners);
  }

  std::array<glm::vec3, 8> corners{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    if (damping.tau[tets + element] == 0.0f) {
      continue;
    }

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      corners[corner] = at[mesh.hexes[element].nodes[corner]];
    }

    damping.fits[tets + element] = rigidFit(corners);
  }
}

void addDamping(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const Damping& damping,
  float weight,
  const std::vector<glm::vec3>& input,
  std::vector<glm::vec3>& output,
  bool hexes = true
) {
  const size_t tets = mesh.tets.size();

  std::array<glm::vec3, 4> gathered{};
  std::array<glm::vec3, 4> produced{};

  for (size_t element = 0u; element != tets; ++element) {
    const float tau = damping.tau[element];

    if (tau == 0.0f) {
      continue;
    }

    const glm::uvec4& corners = mesh.tets[element].nodes;

    for (int corner = 0; corner != 4; ++corner) {
      gathered[static_cast<size_t>(corner)] = input[corners[corner]];
    }

    applyDamping(
      damping.fits[element],
      state.committed[element].rotation,
      damping.tets[element],
      gathered,
      produced
    );

    const float scaled = weight * tau;

    for (int corner = 0; corner != 4; ++corner) {
      output[corners[corner]] += scaled * produced[static_cast<size_t>(corner)];
    }
  }

  if (!hexes) {
    return;
  }

  std::array<glm::vec3, 8> gatheredHex{};
  std::array<glm::vec3, 8> producedHex{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const float tau = damping.tau[tets + element];

    if (tau == 0.0f) {
      continue;
    }

    const std::array<uint32_t, 8>& corners = mesh.hexes[element].nodes;

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      gatheredHex[corner] = input[corners[corner]];
    }

    applyDamping(
      damping.fits[tets + element],
      state.committed[element].rotation,
      damping.hexes[element],
      gatheredHex,
      producedHex
    );

    const float scaled = weight * tau;

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      output[corners[corner]] += scaled * producedHex[corner];
    }
  }
}

size_t dampingBase(
  const talus::PreparedMesh& mesh,
  bool hex,
  size_t element
) {
  return hex ? 16u * mesh.tets.size() + 64u * element : 16u * element;
}

void blockDamping(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  Damping& damping
) {
  const size_t tets = mesh.tets.size();

  damping.blocks.assign(16u * tets + 64u * mesh.hexes.size(), glm::mat3(0.0f));

  for (size_t element = 0u; element != tets; ++element) {
    const float weight = damping.scale * damping.tau[element];

    if (weight == 0.0f) {
      continue;
    }

    const std::array<glm::mat3, 16> made = dampingBlocks(
      damping.fits[element],
      state.committed[element].rotation,
      damping.tets[element]
    );
    const size_t base = dampingBase(mesh, false, element);

    for (size_t at = 0u; at != made.size(); ++at) {
      damping.blocks[base + at] = weight * made[at];
    }
  }

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const float weight = damping.scale * damping.tau[tets + element];

    if (weight == 0.0f) {
      continue;
    }

    const std::array<glm::mat3, 64> made = dampingBlocks(
      damping.fits[tets + element],
      state.committed[element].rotation,
      damping.hexes[element]
    );
    const size_t base = dampingBase(mesh, true, element);

    for (size_t at = 0u; at != made.size(); ++at) {
      damping.blocks[base + at] = weight * made[at];
    }
  }
}

void addDampingDiagonal(
  const talus::PreparedMesh& mesh,
  const Damping& damping,
  std::vector<glm::mat3>& diagonal
) {
  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    if (damping.tau[element] == 0.0f) {
      continue;
    }

    const size_t base = dampingBase(mesh, false, element);

    for (int corner = 0; corner != 4; ++corner) {
      diagonal[mesh.tets[element].nodes[corner]] +=
        damping.blocks[base + static_cast<size_t>(corner * 4 + corner)];
    }
  }

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    if (damping.tau[mesh.tets.size() + element] == 0.0f) {
      continue;
    }

    const size_t base = dampingBase(mesh, true, element);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      diagonal[mesh.hexes[element].nodes[corner]] +=
        damping.blocks[base + corner * 8u + corner];
    }
  }
}

void turnDamping(Damping& damping, const glm::mat3& turn) {
  const glm::mat3 back = glm::transpose(turn);

  for (size_t element = 0u; element != damping.tau.size(); ++element) {
    if (damping.tau[element] == 0.0f) {
      continue;
    }

    RigidFit& fit = damping.fits[element];

    for (glm::vec3& arm : fit.arms) {
      arm = turn * arm;
    }

    fit.inverse = turn * fit.inverse * back;
  }

  for (glm::mat3& block : damping.blocks) {
    block = turn * block * back;
  }
}

void applySystem(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const Scratch& scratch,
  float inertia,
  float step,
  const std::vector<glm::vec3>& input,
  std::vector<glm::vec3>& output,
  OperatorTiming* timing = nullptr
) {
  const auto massBegan = std::chrono::steady_clock::now();

  for (size_t node = 0u; node != input.size(); ++node) {
    output[node] = inertia * mesh.nodeMass[node] * input[node];
  }

  if (timing != nullptr) {
    timing->massSeconds += std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - massBegan
    )
                             .count();
  }

  const float scale = step * step;

  std::array<glm::vec3, 4> gathered{};
  std::array<glm::vec3, 4> produced{};
  const auto tetBegan = std::chrono::steady_clock::now();

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const glm::uvec4& corners = mesh.tets[element].nodes;

    for (int corner = 0; corner != 4; ++corner) {
      gathered[static_cast<size_t>(corner)] = input[corners[corner]];
    }

    applyStiffness(
      gathered,
      state.committed[element].rotation,
      scratch.stiffness[element],
      produced
    );

    if (scratch.stressed) {
      applyInitialStress(scratch.initialStress[element], gathered, produced);
    }

    for (int corner = 0; corner != 4; ++corner) {
      output[corners[corner]] += scale * produced[static_cast<size_t>(corner)];
    }
  }

  std::array<glm::vec3, 8> gatheredHex{};
  std::array<glm::vec3, 8> producedHex{};

  const Damping& damping = scratch.damping;
  const bool fused = !damping.tau.empty() && !scratch.stressed
                     && !mesh.hexes.empty()
                     && scratch.hexStiffnesses.data() == damping.hexes.data();

  if (fused) {
    damping.produced.resize(mesh.hexes.size());
  }

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const std::array<uint32_t, 8>& corners = mesh.hexes[element].nodes;

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      gatheredHex[corner] = input[corners[corner]];
    }

    if (fused && damping.tau[mesh.tets.size() + element] != 0.0f) {
      applyStiffnessDamping(
        damping.fits[mesh.tets.size() + element],
        state.committed[element].rotation,
        scratch.hexStiffnesses[element],
        gatheredHex,
        producedHex,
        damping.produced[element]
      );
    } else {
      applyHexStiffness(
        gatheredHex,
        state.committed[element].rotation,
        scratch.hexStiffnesses[element],
        producedHex
      );
    }

    if (scratch.stressed) {
      applyInitialStress(
        scratch.initialStress[element],
        gatheredHex,
        producedHex
      );
    }

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      output[corners[corner]] += scale * producedHex[corner];
    }
  }

  if (!damping.tau.empty()) {
    addDamping(mesh, state, damping, damping.scale, input, output, !fused);

    if (fused) {
      for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
        const float tau = damping.tau[mesh.tets.size() + element];

        if (tau == 0.0f) {
          continue;
        }

        const std::array<uint32_t, 8>& corners = mesh.hexes[element].nodes;
        const float scaled = damping.scale * tau;

        for (uint32_t corner = 0u; corner != 8u; ++corner) {
          output[corners[corner]] += scaled * damping.produced[element][corner];
        }
      }
    }
  }

  if (timing != nullptr) {
    timing->tetSeconds +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - tetBegan)
        .count();
  }

  const auto anchorBegan = std::chrono::steady_clock::now();

  for (size_t node = 0u; node != output.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      output[node] = glm::vec3(0.0f);
    }
  }

  if (timing != nullptr) {
    ++timing->applications;
    timing->anchorSeconds += std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - anchorBegan
    )
                               .count();
  }
}

struct Incidence {
  std::vector<uint32_t> start;
  std::vector<uint32_t> element;
  uint32_t corners = 4u;
  bool hex = false;

  [[nodiscard]] uint32_t node(
    const talus::PreparedMesh& mesh,
    uint32_t held,
    uint32_t local
  ) const {
    return hex ? mesh.hexes[held].nodes[local] : mesh.tets[held].nodes[local];
  }
};

Incidence buildIncidence(const talus::PreparedMesh& mesh) {
  Incidence built;

  built.hex = !mesh.hexes.empty();
  built.corners = built.hex ? 8u : 4u;

  const size_t nodes = mesh.restPositions.size();
  const size_t elements = built.hex ? mesh.hexes.size() : mesh.tets.size();

  built.start.assign(nodes + 1u, 0u);

  for (size_t held = 0u; held != elements; ++held) {
    for (uint32_t local = 0u; local != built.corners; ++local) {
      ++built.start[built.node(mesh, static_cast<uint32_t>(held), local) + 1u];
    }
  }

  for (size_t node = 0u; node != nodes; ++node) {
    built.start[node + 1u] += built.start[node];
  }

  built.element.assign(built.start[nodes], 0u);

  std::vector<uint32_t> cursor(built.start.begin(), built.start.end() - 1);

  for (size_t held = 0u; held != elements; ++held) {
    for (uint32_t local = 0u; local != built.corners; ++local) {
      const uint32_t node =
        built.node(mesh, static_cast<uint32_t>(held), local);

      built.element[cursor[node]] = static_cast<uint32_t>(held);
      ++cursor[node];
    }
  }

  return built;
}

void gaussSeidelSweep(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const Scratch& scratch,
  float step,
  const Incidence& incidence,
  const std::vector<glm::vec3>& right,
  std::vector<glm::vec3>& solution
) {
  const float scale = step * step;
  const size_t nodes = solution.size();

  for (size_t node = 0u; node != nodes; ++node) {
    if (mesh.anchored[node] != 0u) {
      solution[node] = glm::vec3(0.0f);

      continue;
    }

    glm::vec3 sum = right[node];

    for (uint32_t at = incidence.start[node]; at != incidence.start[node + 1u];
         ++at) {
      const uint32_t held = incidence.element[at];
      const glm::mat3& rotation = state.committed[held].rotation;
      const glm::mat3 inverse = glm::transpose(rotation);
      const bool damped =
        !scratch.damping.blocks.empty()
        && scratch.damping.tau[incidence.hex ? mesh.tets.size() + held : held]
             != 0.0f;
      const size_t base = dampingBase(mesh, incidence.hex, held);

      uint32_t row = incidence.corners;

      for (uint32_t local = 0u; local != incidence.corners; ++local) {
        if (incidence.node(mesh, held, local) == node) {
          row = local;

          break;
        }
      }

      if (row == incidence.corners) {
        continue;
      }

      for (uint32_t column = 0u; column != incidence.corners; ++column) {
        if (column == row) {
          continue;
        }

        const uint32_t other = incidence.node(mesh, held, column);

        if (mesh.anchored[other] != 0u) {
          continue;
        }

        const glm::mat3& coupling =
          incidence.hex ? scratch.hexStiffnesses[held].block[row * 8u + column]
                        : scratch.stiffness[held].block[row * 4u + column];

        sum -= scale * (rotation * coupling * inverse) * solution[other];

        if (damped) {
          sum -= scratch.damping.blocks[base + row * incidence.corners + column]
                 * solution[other];
        }
      }
    }

    solution[node] = scratch.preconditioner[node] * sum;
  }
}

size_t tetPoint(const talus::PreparedMesh& mesh, size_t element) {
  return 8u * mesh.hexes.size() + element;
}

void sizePlastic(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state
) {
  const size_t points = 8u * mesh.hexes.size() + mesh.tets.size();

  if (state.plastic.size() != points) {
    state.plastic.assign(points, talus::PlasticPoint{});
  }
}

TetPointTrial plasticTrial(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  size_t element,
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  float margin,
  ElementRotation rotation,
  bool returnMapping = true,
  bool tangent = true
) {
  sizePlastic(mesh, state);

  const talus::PreparedTet& tet = mesh.tets[element];

  return tetPointTrial(
    deformed,
    rest,
    tet,
    elasticOf(materials, tet.material),
    plasticOf(materials, tet.material),
    state.plastic[tetPoint(mesh, element)],
    margin,
    rotation,
    returnMapping,
    tangent
  );
}

HexPointTrial plasticTrial(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  size_t element,
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  float margin,
  ElementRotation rotation,
  bool returnMapping = true,
  bool tangent = true
) {
  sizePlastic(mesh, state);

  const talus::PreparedHex& hex = mesh.hexes[element];

  std::array<talus::PlasticPoint, 8> committed{};

  std::copy_n(
    state.plastic.begin() + static_cast<std::ptrdiff_t>(8u * element),
    8u,
    committed.begin()
  );

  return hexPointTrial(
    deformed,
    rest,
    hex,
    elasticOf(materials, hex.material),
    plasticOf(materials, hex.material),
    committed,
    margin,
    rotation,
    returnMapping,
    tangent
  );
}

bool commitPlastic(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  ElementRotation rotation
) {
  std::array<glm::vec3, 8> deformed{};
  std::array<glm::vec3, 8> rest{};

  bool grew = false;

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::PreparedHex& hex = mesh.hexes[element];

    if (!yields(plasticOf(materials, hex.material))) {
      continue;
    }

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      deformed[corner] = state.position[hex.nodes[corner]];
      rest[corner] = mesh.restPositions[hex.nodes[corner]];
    }

    const HexPointTrial computed = plasticTrial(
      mesh,
      state,
      materials,
      element,
      deformed,
      rest,
      0.0f,
      rotation,
      true,
      false
    );

    for (size_t point = 0u; point != computed.points.size(); ++point) {
      grew = grew
             || computed.points[point].equivalent
                  > state.plastic[8u * element + point].equivalent;
    }

    std::copy(
      computed.points.begin(),
      computed.points.end(),
      state.plastic.begin() + static_cast<std::ptrdiff_t>(8u * element)
    );
  }

  std::array<glm::vec3, 4> gathered{};
  std::array<glm::vec3, 4> restTet{};

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const talus::PreparedTet& tet = mesh.tets[element];

    if (!yields(plasticOf(materials, tet.material))) {
      continue;
    }

    for (int corner = 0; corner != 4; ++corner) {
      gathered[static_cast<size_t>(corner)] = state.position[tet.nodes[corner]];
      restTet[static_cast<size_t>(corner)] =
        mesh.restPositions[tet.nodes[corner]];
    }

    const TetPointTrial computed = plasticTrial(
      mesh,
      state,
      materials,
      element,
      gathered,
      restTet,
      0.0f,
      rotation,
      true,
      false
    );

    grew = grew
           || computed.point.equivalent
                > state.plastic[tetPoint(mesh, element)].equivalent;

    state.plastic[tetPoint(mesh, element)] = computed.point;
  }

  return grew;
}

struct Assembly {
  float energy = 0.0f;
  uint32_t loading = 0u;
  uint32_t yielding = 0u;
};

Assembly assembleElastic(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state,
  Scratch& scratch,
  float step,
  const std::vector<glm::vec3>& at,
  std::span<const talus::PhysicalMaterial> materials,
  bool keep,
  float margin,
  ElementRotation rotation,
  bool returnMapping = true
) {
  std::fill(scratch.force.begin(), scratch.force.end(), glm::vec3(0.0f));
  std::fill(scratch.diagonal.begin(), scratch.diagonal.end(), glm::mat3(0.0f));

  if (scratch.stressed) {
    scratch.initialStress.resize(state.committed.size());
  }

  const float scale = step * step;

  float elastic = 0.0f;
  uint32_t loading = 0u;
  uint32_t yielding = 0u;

  std::array<glm::vec3, 4> gathered{};
  std::array<glm::vec3, 4> rest{};

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const talus::PreparedTet& tet = mesh.tets[element];

    for (int corner = 0; corner != 4; ++corner) {
      gathered[static_cast<size_t>(corner)] = at[tet.nodes[corner]];
      rest[static_cast<size_t>(corner)] = mesh.restPositions[tet.nodes[corner]];
    }

    if (yields(plasticOf(materials, tet.material))) {
      const TetPointTrial computed = plasticTrial(
        mesh,
        state,
        materials,
        element,
        gathered,
        rest,
        margin,
        rotation,
        returnMapping,
        !scratch.tetWorking.empty()
      );

      state.committed[element] = talus::CommittedTet{computed.rotation};

      elastic += computed.energy;
      loading += computed.loading;
      yielding += computed.yielding;

      if (!scratch.tetWorking.empty()) {
        scratch.tetWorking[element] = computed.tangent;
      }

      if (keep) {
        state.plastic[tetPoint(mesh, element)] = computed.point;
      }

      const std::array<glm::mat3, 4> diagonal =
        stiffnessDiagonal(scratch.stiffness[element], computed.rotation);

      for (int corner = 0; corner != 4; ++corner) {
        scratch.force[tet.nodes[corner]] +=
          computed.force[static_cast<size_t>(corner)];
        scratch.diagonal[tet.nodes[corner]] +=
          scale * diagonal[static_cast<size_t>(corner)];
      }

      if (scratch.stressed) {
        scratch.initialStress[element] = initialStress(
          gathered,
          rest,
          tet,
          scratch.stiffness[element],
          computed.rotation,
          computed.force
        );
      }

      continue;
    }

    const TetStiffness& stiffness = scratch.stiffness[element];

    const TrialState computed = trial(gathered, rest, tet, stiffness, rotation);

    state.committed[element] = commit(computed);

    elastic += computed.energy;

    const std::array<glm::mat3, 4> diagonal =
      stiffnessDiagonal(stiffness, computed.rotation);

    for (int corner = 0; corner != 4; ++corner) {
      scratch.force[tet.nodes[corner]] +=
        computed.force[static_cast<size_t>(corner)];
      scratch.diagonal[tet.nodes[corner]] +=
        scale * diagonal[static_cast<size_t>(corner)];
    }

    if (scratch.stressed) {
      scratch.initialStress[element] = initialStress(
        gathered,
        rest,
        tet,
        stiffness,
        computed.rotation,
        computed.force
      );
    }
  }

  std::array<glm::vec3, 8> gatheredHex{};
  std::array<glm::vec3, 8> restHex{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::PreparedHex& hex = mesh.hexes[element];

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      gatheredHex[corner] = at[hex.nodes[corner]];
      restHex[corner] = mesh.restPositions[hex.nodes[corner]];
    }

    if (
      const PlasticParams plastic = plasticOf(materials, hex.material);
      yields(plastic)
    ) {
      const HexPointTrial computed = plasticTrial(
        mesh,
        state,
        materials,
        element,
        gatheredHex,
        restHex,
        margin,
        rotation,
        returnMapping,
        !scratch.hexWorking.empty()
      );

      state.committed[element] = talus::CommittedTet{computed.rotation};

      elastic += computed.energy;
      loading += computed.loading;
      yielding += computed.yielding;

      if (!scratch.hexWorking.empty()) {
        scratch.hexWorking[element] = computed.tangent;
      }

      if (keep) {
        std::copy(
          computed.points.begin(),
          computed.points.end(),
          state.plastic.begin() + static_cast<std::ptrdiff_t>(8u * element)
        );
      }

      const std::array<glm::mat3, 8> diagonal = hexStiffnessDiagonal(
        scratch.hexStiffnesses[element],
        computed.rotation
      );

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        scratch.force[hex.nodes[corner]] += computed.force[corner];
        scratch.diagonal[hex.nodes[corner]] += scale * diagonal[corner];
      }

      if (scratch.stressed) {
        scratch.initialStress[element] = initialStress(
          gatheredHex,
          restHex,
          hex,
          scratch.hexStiffnesses[element],
          computed.rotation,
          computed.force
        );
      }

      continue;
    }

    const HexStiffness& stiffness = scratch.hexStiffnesses[element];

    const HexTrial computed =
      hexTrial(gatheredHex, restHex, hex, stiffness, rotation);

    state.committed[element] = talus::CommittedTet{computed.rotation};

    elastic += computed.energy;

    const std::array<glm::mat3, 8> diagonal =
      hexStiffnessDiagonal(stiffness, computed.rotation);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      scratch.force[hex.nodes[corner]] += computed.force[corner];
      scratch.diagonal[hex.nodes[corner]] += scale * diagonal[corner];
    }

    if (scratch.stressed) {
      scratch.initialStress[element] = initialStress(
        gatheredHex,
        restHex,
        hex,
        stiffness,
        computed.rotation,
        computed.force
      );
    }
  }

  return Assembly{elastic, loading, yielding};
}

void guardPreconditioner(
  const talus::PreparedMesh& mesh,
  Scratch& scratch
) {
  for (size_t node = 0u; node != scratch.preconditioner.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      scratch.preconditioner[node] = glm::mat3(1.0f);

      continue;
    }

    if (!(glm::determinant(scratch.diagonal[node]) > 0.0f)) {
      scratch.preconditioner[node] = glm::mat3(1.0f);

      continue;
    }

    scratch.preconditioner[node] = glm::inverse(scratch.diagonal[node]);
  }
}

struct FreeModes {
  uint32_t count = 0u;
  bool translates = false;
  glm::dvec3 pivot{0.0};
  double arm = 0.0;
  std::vector<glm::dvec3> axes;
  std::vector<glm::dvec3> shape;
};

std::vector<glm::dvec3> orthonormalAxes(std::span<const glm::dvec3> turns) {
  std::vector<glm::dvec3> axes;

  for (const glm::dvec3& turn : turns) {
    glm::dvec3 axis = turn;

    for (const glm::dvec3& earlier : axes) {
      axis -= glm::dot(earlier, axis) * earlier;
    }

    const double length = glm::length(axis);

    if (length > FREE_AXIS_FLOOR * glm::length(turn)) {
      axes.push_back(axis / length);
    }
  }

  return axes;
}

FreeModes freeModes(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state
) {
  const size_t nodes = mesh.restPositions.size();

  FreeModes free;

  double total = 0.0;
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    total += mass;
    centre += mass * glm::dvec3(state.position[node]);
  }

  if (!(total > 0.0)) {
    return free;
  }

  centre /= total;

  for (size_t node = 0u; node != nodes; ++node) {
    free.arm = std::max(
      free.arm,
      glm::length(glm::dvec3(state.position[node]) - centre)
    );
  }

  if (!(free.arm > 0.0)) {
    return free;
  }

  Eigen::Matrix<double, 6, 6> gram = Eigen::Matrix<double, 6, 6>::Zero();
  size_t anchors = 0u;

  free.pivot = centre;

  for (size_t node = 0u; node != nodes; ++node) {
    if (mesh.anchored[node] == 0u) {
      continue;
    }

    const glm::dvec3 reach =
      (glm::dvec3(state.position[node]) - centre) / free.arm;

    const Eigen::Vector3d lever(reach.x, reach.y, reach.z);

    Eigen::Matrix<double, 3, 6> row;

    row.leftCols<3>().setIdentity();

    for (Eigen::Index axis = 0; axis != 3; ++axis) {
      row.col(3 + axis) = Eigen::Vector3d::Unit(axis).cross(lever);
    }

    gram += row.transpose() * row;

    if (anchors == 0u) {
      free.pivot = glm::dvec3(state.position[node]);
    }

    ++anchors;
  }

  free.translates = anchors == 0u;

  Eigen::Matrix<double, 6, Eigen::Dynamic> basis(6, 0);

  if (free.translates) {
    basis = Eigen::Matrix<double, 6, 6>::Identity();
  } else {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solved(
      gram
    );
    const double largest = solved.eigenvalues()(5);

    for (Eigen::Index mode = 0; mode != 6; ++mode) {
      if (solved.eigenvalues()(mode) <= FREE_MODE_GAP * largest) {
        basis.conservativeResize(Eigen::NoChange, basis.cols() + 1);
        basis.col(basis.cols() - 1) = solved.eigenvectors().col(mode);
      }
    }
  }

  std::vector<glm::dvec3> turns;

  for (Eigen::Index mode = 0; mode != basis.cols(); ++mode) {
    const glm::dvec3 shift(basis(0, mode), basis(1, mode), basis(2, mode));
    const glm::dvec3 turn(basis(3, mode), basis(4, mode), basis(5, mode));

    turns.push_back(turn);

    std::vector<glm::dvec3> vector(nodes, glm::dvec3(0.0));

    for (size_t node = 0u; node != nodes; ++node) {
      if (mesh.anchored[node] != 0u) {
        continue;
      }

      vector[node] = shift
                     + glm::cross(
                       turn,
                       (glm::dvec3(state.position[node]) - centre) / free.arm
                     );
    }

    for (uint32_t earlier = 0u; earlier != free.count; ++earlier) {
      const glm::dvec3* held = free.shape.data() + earlier * nodes;

      double along = 0.0;

      for (size_t node = 0u; node != nodes; ++node) {
        along += static_cast<double>(mesh.nodeMass[node])
                 * glm::dot(held[node], vector[node]);
      }

      for (size_t node = 0u; node != nodes; ++node) {
        vector[node] -= along * held[node];
      }
    }

    double norm = 0.0;

    for (size_t node = 0u; node != nodes; ++node) {
      norm += static_cast<double>(mesh.nodeMass[node])
              * glm::dot(vector[node], vector[node]);
    }

    norm = std::sqrt(norm);

    if (!(norm > 0.0)) {
      continue;
    }

    for (glm::dvec3& entry : vector) {
      free.shape.push_back(entry / norm);
    }

    ++free.count;
  }

  free.axes = orthonormalAxes(turns);

  return free;
}

void projectMotion(
  const FreeModes& free,
  const talus::PreparedMesh& mesh,
  std::vector<glm::vec3>& vector
) {
  const size_t nodes = vector.size();

  for (uint32_t mode = 0u; mode != free.count; ++mode) {
    const glm::dvec3* shape = free.shape.data() + mode * nodes;

    double along = 0.0;

    for (size_t node = 0u; node != nodes; ++node) {
      along += static_cast<double>(mesh.nodeMass[node])
               * glm::dot(shape[node], glm::dvec3(vector[node]));
    }

    for (size_t node = 0u; node != nodes; ++node) {
      vector[node] = glm::vec3(glm::dvec3(vector[node]) - along * shape[node]);
    }
  }
}

void projectForce(
  const FreeModes& free,
  const talus::PreparedMesh& mesh,
  std::vector<glm::vec3>& vector
) {
  const size_t nodes = vector.size();

  for (uint32_t mode = 0u; mode != free.count; ++mode) {
    const glm::dvec3* shape = free.shape.data() + mode * nodes;

    double along = 0.0;

    for (size_t node = 0u; node != nodes; ++node) {
      along += glm::dot(shape[node], glm::dvec3(vector[node]));
    }

    for (size_t node = 0u; node != nodes; ++node) {
      vector[node] = glm::vec3(
        glm::dvec3(vector[node])
        - along * static_cast<double>(mesh.nodeMass[node]) * shape[node]
      );
    }
  }
}

struct Balance {
  float force = 0.0f;
  float torque = 0.0f;
  bool unbalanced = false;
};

Balance weigh(
  const FreeModes& free,
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const std::vector<glm::vec3>& right,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load
) {
  const double pull = static_cast<double>(glm::length(gravity));

  double scale = 0.0;
  glm::dvec3 force(0.0);
  glm::dvec3 moment(0.0);

  for (size_t node = 0u; node != right.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    scale += static_cast<double>(mesh.nodeMass[node]) * pull;

    if (node < load.size()) {
      scale += static_cast<double>(glm::length(load[node]));
    }

    const glm::dvec3 pushed(right[node]);

    force += pushed;
    moment += glm::cross(glm::dvec3(state.position[node]) - free.pivot, pushed);
  }

  if (!(scale > 0.0)) {
    return Balance{};
  }

  glm::dvec3 turning(0.0);

  for (const glm::dvec3& axis : free.axes) {
    turning += glm::dot(axis, moment) * axis;
  }

  const double forceRatio = free.translates ? glm::length(force) / scale : 0.0;
  const double torqueRatio = glm::length(turning) / (scale * free.arm);
  const double limit = static_cast<double>(EQUILIBRIUM_BALANCE);

  return Balance{
    static_cast<float>(forceRatio),
    static_cast<float>(torqueRatio),
    !(forceRatio <= limit) || !(torqueRatio <= limit)
  };
}

float equilibriumShift(
  const talus::PreparedMesh& mesh,
  const Scratch& scratch
) {
  double stiffness = 0.0;
  double mass = 0.0;

  for (size_t node = 0u; node != scratch.diagonal.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    const glm::mat3& block = scratch.diagonal[node];

    stiffness += static_cast<double>(block[0][0] + block[1][1] + block[2][2]);
    mass += 3.0 * static_cast<double>(mesh.nodeMass[node]);
  }

  if (!(mass > 0.0)) {
    return 0.0f;
  }

  return static_cast<float>(EQUILIBRIUM_SHIFT * stiffness / mass);
}

CgResult conjugateGradient(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  Scratch& scratch,
  float inertia,
  float step,
  uint32_t limit,
  float tolerance,
  std::vector<glm::vec3>& solution,
  const SystemFactor* prepared,
  const glm::mat3& frame,
  const FreeModes* free = nullptr
) {
  const size_t nodes = solution.size();
  CgResult result;

  const auto condition = [&]() {
    const auto conditionBegan = std::chrono::steady_clock::now();

    if (prepared != nullptr && !prepared->empty()) {
      scratch.conditioned = scratch.residual;

      if (free != nullptr) {
        projectForce(*free, mesh, scratch.conditioned);
      }

      applyFactor(
        *prepared,
        frame,
        mesh,
        scratch.conditioned,
        &result.factorTiming
      );
    } else {
      for (size_t node = 0u; node != nodes; ++node) {
        scratch.conditioned[node] =
          scratch.preconditioner[node] * scratch.residual[node];
      }
    }

    if (free != nullptr) {
      projectMotion(*free, mesh, scratch.conditioned);
    }

    result.conditionSeconds +=
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - conditionBegan
      )
        .count();
  };

  float reference = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    reference += glm::dot(scratch.right[node], scratch.right[node]);
  }

  reference = std::sqrt(reference);

  applySystem(
    mesh,
    state,
    scratch,
    inertia,
    step,
    solution,
    scratch.applied,
    &result.operatorTiming
  );

  if (free != nullptr) {
    projectForce(*free, mesh, scratch.applied);
  }

  float squared = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    scratch.residual[node] = mesh.anchored[node] != 0u
                               ? glm::vec3(0.0f)
                               : scratch.right[node] - scratch.applied[node];
  }

  condition();

  for (size_t node = 0u; node != nodes; ++node) {
    scratch.direction[node] = scratch.conditioned[node];

    squared += glm::dot(scratch.residual[node], scratch.conditioned[node]);
  }

  const float referenceFloor =
    glm::max(reference, std::numeric_limits<float>::min());
  const float target = tolerance * referenceFloor;

  float opening = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    opening += glm::dot(scratch.residual[node], scratch.residual[node]);
  }

  result.startResidual = std::sqrt(opening) / referenceFloor;

  uint32_t taken = 0u;

  for (uint32_t iteration = 0u; iteration != limit; ++iteration) {
    float length = 0.0f;

    for (size_t node = 0u; node != nodes; ++node) {
      length += glm::dot(scratch.residual[node], scratch.residual[node]);
    }

    if (std::sqrt(length) <= target) {
      break;
    }

    applySystem(
      mesh,
      state,
      scratch,
      inertia,
      step,
      scratch.direction,
      scratch.applied,
      &result.operatorTiming
    );

    if (free != nullptr) {
      projectForce(*free, mesh, scratch.applied);
    }

    float curvature = 0.0f;

    for (size_t node = 0u; node != nodes; ++node) {
      curvature += glm::dot(scratch.direction[node], scratch.applied[node]);
    }

    const bool broken = scratch.stressed
                          ? curvature == 0.0f || !std::isfinite(curvature)
                          : !(curvature > 0.0f);

    if (broken || !(squared > 0.0f)) {
      break;
    }

    const float stride = squared / curvature;

    for (size_t node = 0u; node != nodes; ++node) {
      solution[node] += stride * scratch.direction[node];
      scratch.residual[node] -= stride * scratch.applied[node];
    }

    float next = 0.0f;

    condition();

    for (size_t node = 0u; node != nodes; ++node) {
      next += glm::dot(scratch.residual[node], scratch.conditioned[node]);
    }

    const float blend = next / squared;

    squared = next;

    for (size_t node = 0u; node != nodes; ++node) {
      scratch.direction[node] =
        scratch.conditioned[node] + blend * scratch.direction[node];
    }

    ++taken;
  }

  float residualSquared = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    residualSquared += glm::dot(scratch.residual[node], scratch.residual[node]);
  }

  const float residual = std::sqrt(residualSquared);

  result.iterations = taken;
  result.relativeResidual = residual / referenceFloor;
  result.capped = taken == limit && residual > target;
  result.resolved = residual <= target;

  return result;
}
}

namespace {

constexpr uint32_t SEED_SAMPLES = 2u;
constexpr uint32_t SEED_PROJECTIONS = 4u;
constexpr float SEED_SURFACE_SHARE = 1.0e-4f;
constexpr float SEED_SPACING_SHARE = 0.25f;

void appendSeeds(
  const talus::field::SampledSdf& lattice,
  const glm::uvec3& cell,
  uint32_t hex,
  talus::PreparedMesh& mesh
) {
  const glm::vec3 low = samplePosition(lattice, cell);
  const glm::vec3 size = lattice.voxelSize;
  const float spacing = SEED_SPACING_SHARE * glm::compMin(size);
  const float onSurface = SEED_SURFACE_SHARE * glm::compMin(size);
  const size_t first = mesh.surfaceSeeds.size();

  for (uint32_t k = 0u; k != SEED_SAMPLES; ++k) {
    for (uint32_t j = 0u; j != SEED_SAMPLES; ++j) {
      for (uint32_t i = 0u; i != SEED_SAMPLES; ++i) {
        glm::vec3 point =
          low
          + size * (glm::vec3(i, j, k) + 0.5f)
              / static_cast<float>(SEED_SAMPLES);

        float distance = 0.0f;

        for (uint32_t step = 0u; step != SEED_PROJECTIONS; ++step) {
          const talus::field::FieldProbe probed =
            talus::field::probe(lattice, point);

          distance = probed.distance;
          point -= distance * probed.normal;
        }

        distance = talus::field::distance(lattice, point);

        const glm::vec3 local = (point - low) / size;

        if (
          std::abs(distance) > onSurface
          || glm::any(glm::lessThan(local, glm::vec3(0.0f)))
          || glm::any(glm::greaterThan(local, glm::vec3(1.0f)))
        ) {
          continue;
        }

        bool crowded = false;

        for (size_t seed = first; seed != mesh.surfaceSeeds.size(); ++seed) {
          crowded =
            crowded || glm::distance(mesh.surfaceSeeds[seed], point) < spacing;
        }

        if (crowded) {
          continue;
        }

        std::array<float, 8> shape{};

        for (uint32_t corner = 0u; corner != 8u; ++corner) {
          const glm::vec3 side = glm::vec3(cornerOffset(corner));
          const glm::vec3 weight =
            side * local + (1.0f - side) * (1.0f - local);

          shape[corner] = weight.x * weight.y * weight.z;
        }

        mesh.surfaceSeeds.push_back(point);
        mesh.surfaceSeedHexes.push_back(hex);
        mesh.surfaceSeedShapes.push_back(shape);
      }
    }
  }
}

}

void rebuildSurface(
  talus::PreparedMesh& mesh,
  const talus::field::SampledSdf& lattice
) {
  mesh.surfaceEdges.clear();
  mesh.surfaceTriangles.clear();
  mesh.surfaceThickness.clear();
  mesh.surfaceLane.clear();

  collectSurface(mesh, lattice);
  clusterSurface(mesh);
}

std::shared_ptr<const talus::PreparedMesh> buildMesh(
  const talus::field::SampledSdf& lattice,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::mat4& localToWorld,
  std::span<const SupportVolume> supports,
  VolumeElementType element
) {
  const bool hexElements = element == VolumeElementType::Hex;
  if (
    lattice.resolution.x < 2u || lattice.resolution.y < 2u
    || lattice.resolution.z < 2u
  ) {
    return nullptr;
  }

  auto mesh = std::make_shared<talus::PreparedMesh>();

  mesh->voxelSize = lattice.voxelSize;

  const glm::uvec3 cells = lattice.resolution - glm::uvec3(1u);

  const size_t samples = static_cast<size_t>(lattice.resolution.x)
                         * lattice.resolution.y * lattice.resolution.z;

  std::vector<uint32_t> nodeOf(samples, NO_NODE);
  std::vector<uint8_t> solid(
    static_cast<size_t>(cells.x) * cells.y * cells.z,
    0u
  );
  std::vector<float> fill(solid.size(), 0.0f);
  std::vector<uint8_t> partial(samples, 0u);

  for (uint32_t z = 0u; z != cells.z; ++z) {
    for (uint32_t y = 0u; y != cells.y; ++y) {
      for (uint32_t x = 0u; x != cells.x; ++x) {
        const glm::uvec3 cell(x, y, z);
        const float filled = cellFill(lattice, cell);

        if (!(filled > 0.0f)) {
          continue;
        }

        solid[x + cells.x * (y + cells.y * z)] = 1u;
        fill[x + cells.x * (y + cells.y * z)] = filled;

        for (uint32_t corner = 0u; corner != 8u; ++corner) {
          if (filled < 1.0f) {
            partial[latticeIndex(lattice, cell + cornerOffset(corner))] = 1u;
          }
        }

        for (uint32_t corner = 0u; corner != 8u; ++corner) {
          const glm::uvec3 at = cell + cornerOffset(corner);
          const uint32_t sample = latticeIndex(lattice, at);

          if (nodeOf[sample] != NO_NODE) {
            continue;
          }

          nodeOf[sample] = static_cast<uint32_t>(mesh->restPositions.size());
          mesh->restPositions.push_back(samplePosition(lattice, at));
          mesh->restSurfaceDistance.push_back(0.0f);
        }
      }
    }
  }

  if (mesh->restPositions.empty()) {
    return nullptr;
  }

  for (size_t sample = 0u; sample != samples; ++sample) {
    if (nodeOf[sample] != NO_NODE && partial[sample] != 0u) {
      mesh->restSurfaceDistance[nodeOf[sample]] = lattice.distances[sample];
    }
  }

  mesh->nodeMass.assign(mesh->restPositions.size(), 0.0f);
  mesh->anchored.assign(mesh->restPositions.size(), 0u);

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    mesh->anchored[node] =
      supported(mesh->restPositions[node], localToWorld, supports) ? 1u : 0u;
  }

  std::vector<uint8_t> onSurface(mesh->restPositions.size(), 0u);

  for (uint32_t z = 0u; z != cells.z; ++z) {
    for (uint32_t y = 0u; y != cells.y; ++y) {
      for (uint32_t x = 0u; x != cells.x; ++x) {
        if (solid[x + cells.x * (y + cells.y * z)] == 0u) {
          continue;
        }

        const glm::uvec3 cell(x, y, z);

        std::array<uint32_t, 8> corners{};

        for (uint32_t corner = 0u; corner != 8u; ++corner) {
          corners[corner] =
            nodeOf[latticeIndex(lattice, cell + cornerOffset(corner))];
        }

        const uint32_t lane = cellLane(lattice, cell);
        const float density = laneOf(materials, lane).density;
        const float filled = fill[x + cells.x * (y + cells.y * z)];

        if (hexElements) {
          talus::PreparedHex hex;

          hex.nodes = corners;
          hex.halfExtents = 0.5f * lattice.voxelSize;
          hex.volume = filled * lattice.voxelSize.x * lattice.voxelSize.y
                       * lattice.voxelSize.z;
          hex.material = lane;

          const float share = density * hex.volume * 0.125f;

          for (uint32_t corner = 0u; corner != 8u; ++corner) {
            mesh->nodeMass[hex.nodes[corner]] += share;
          }

          if (filled < 1.0f) {
            appendSeeds(
              lattice,
              cell,
              static_cast<uint32_t>(mesh->hexes.size()),
              *mesh
            );
          }

          mesh->hexes.push_back(hex);
        }

        for (const std::array<uint32_t, 4>& pick : KUHN) {
          if (hexElements) {
            break;
          }

          talus::PreparedTet tet;

          tet.nodes = glm::uvec4(
            corners[pick[0]],
            corners[pick[1]],
            corners[pick[2]],
            corners[pick[3]]
          );
          tet.material = lane;

          const glm::vec3& first = mesh->restPositions[tet.nodes[0]];

          glm::mat3 shape(
            mesh->restPositions[tet.nodes[1]] - first,
            mesh->restPositions[tet.nodes[2]] - first,
            mesh->restPositions[tet.nodes[3]] - first
          );

          if (glm::determinant(shape) < 0.0f) {
            std::swap(tet.nodes[1], tet.nodes[2]);

            shape = glm::mat3(
              mesh->restPositions[tet.nodes[1]] - first,
              mesh->restPositions[tet.nodes[2]] - first,
              mesh->restPositions[tet.nodes[3]] - first
            );
          }

          tet.volume = filled * glm::determinant(shape) / 6.0f;

          if (tet.volume <= 0.0f) {
            continue;
          }

          tet.restInverse = glm::inverse(shape);

          const float share = density * tet.volume * 0.25f;

          for (int corner = 0; corner != 4; ++corner) {
            mesh->nodeMass[tet.nodes[corner]] += share;
          }

          mesh->tets.push_back(tet);
        }

        for (uint32_t axis = 0u; axis != 3u; ++axis) {
          for (uint32_t side = 0u; side != 2u; ++side) {
            const int along = static_cast<int>(axis);

            bool outside =
              side == 0u ? cell[along] == 0u : cell[along] + 1u == cells[along];

            if (!outside) {
              glm::uvec3 neighbour = cell;

              neighbour[along] =
                side == 0u ? cell[along] - 1u : cell[along] + 1u;

              outside = solid
                          [neighbour.x
                           + cells.x * (neighbour.y + cells.y * neighbour.z)]
                        == 0u;
            }

            if (!outside) {
              continue;
            }

            for (uint32_t corner = 0u; corner != 8u; ++corner) {
              if (((corner >> axis) & 1u) != side) {
                continue;
              }

              onSurface[corners[corner]] = 1u;
            }
          }
        }
      }
    }
  }

  if (mesh->tets.empty() && mesh->hexes.empty()) {
    return nullptr;
  }

  for (uint32_t node = 0u; node != onSurface.size(); ++node) {
    if (onSurface[node] != 0u) {
      mesh->surfaceNodes.push_back(node);
    }
  }

  rebuildSurface(*mesh, lattice);

  return mesh;
}

void restStateInto(
  const talus::PreparedMesh& mesh,
  const talus::Pose& pose,
  talus::DeformableState& state
) {
  const glm::mat4 placement = talus::model(pose);
  const talus::CommittedTet placed{polarRotation(glm::mat3(placement))};

  state.position.resize(mesh.restPositions.size());

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    state.position[node] =
      glm::vec3(placement * glm::vec4(mesh.restPositions[node], 1.0f));
  }

  state.velocity.assign(mesh.restPositions.size(), glm::vec3(0.0f));
  state.acceleration.assign(mesh.restPositions.size(), glm::vec3(0.0f));
  state.committed.assign(
    mesh.hexes.empty() ? mesh.tets.size() : mesh.hexes.size(),
    placed
  );
  state.plastic.clear();
}

talus::DeformableState restState(
  const talus::PreparedMesh& mesh,
  const talus::Pose& pose
) {
  talus::DeformableState state;

  restStateInto(mesh, pose, state);

  return state;
}

MeshFrame meshFrame(
  const talus::PreparedMesh& mesh,
  std::span<const glm::vec3> position
) {
  MeshFrame fit;

  double mass = 0.0;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);

    fit.restCentre += weight * glm::dvec3(mesh.restPositions[node]);
    fit.centre += weight * glm::dvec3(position[node]);
    mass += weight;
  }

  if (!(mass > 0.0)) {
    return fit;
  }

  fit.restCentre /= mass;
  fit.centre /= mass;

  glm::dmat3 covariance(0.0);

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    covariance += static_cast<double>(mesh.nodeMass[node])
                  * glm::outerProduct(
                    glm::dvec3(mesh.restPositions[node]) - fit.restCentre,
                    glm::dvec3(position[node]) - fit.centre
                  );
  }

  glm::dmat3 toRest = covariance;

  for (uint32_t pass = 0u; pass != 100u; ++pass) {
    const glm::dmat3 next =
      0.5 * (toRest + glm::transpose(glm::inverse(toRest)));
    const glm::dmat3 change = next - toRest;

    toRest = next;

    double largest = 0.0;

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        largest = std::max(largest, std::abs(change[column][row]));
      }
    }

    if (largest < 1.0e-14) {
      break;
    }
  }

  fit.rotation = glm::transpose(toRest);

  return fit;
}

Unloading unloadedState(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& from,
  std::span<const talus::PhysicalMaterial> materials,
  const Settings& settings,
  float deltaTime
) {
  std::vector<talus::PhysicalMaterial> held(
    materials.begin(),
    materials.end()
  );

  for (talus::PhysicalMaterial& lane : held) {
    if (yields(PlasticParams{lane.yieldStress, lane.hardeningModulus})) {
      lane.yieldStress = std::numeric_limits<float>::max();
    }
  }

  const MeshFrame before = meshFrame(mesh, from.position);

  Unloading unload{from};
  DeformableSolverState relaxing;

  const glm::vec3 placed(before.centre);

  for (glm::vec3& at : unload.state.position) {
    at -= placed;
  }

  const auto still = [&unload]() {
    std::fill(
      unload.state.velocity.begin(),
      unload.state.velocity.end(),
      glm::vec3(0.0f)
    );
    std::fill(
      unload.state.acceleration.begin(),
      unload.state.acceleration.end(),
      glm::vec3(0.0f)
    );
  };

  while (unload.steps != UNLOAD_STEPS && !(unload.speed < UNLOAD_SPEED)) {
    still();

    StepReport relaxed;

    step(
      mesh,
      unload.state,
      held,
      settings,
      glm::vec3(0.0f),
      deltaTime,
      {},
      {},
      &relaxing,
      relaxed
    );

    ++unload.steps;
    unload.speed = 0.0f;

    for (const glm::vec3& moving : unload.state.velocity) {
      unload.speed = std::max(unload.speed, glm::length(moving));
    }
  }

  still();

  const MeshFrame after = meshFrame(mesh, unload.state.position);
  const glm::dmat3 turn = before.rotation * glm::transpose(after.rotation);

  for (glm::vec3& at : unload.state.position) {
    at = glm::vec3(turn * (glm::dvec3(at) - after.centre) + before.centre);
  }

  return unload;
}

namespace {

struct NewtonStep {
  float moved = 0.0f;
  float residual = 0.0f;
  bool capped = false;
  bool solved = false;
  uint32_t iterations = 0u;
  bool factored = false;
  float shift = 0.0f;
  Balance balance{};
};

NewtonStep staticNewtonStep(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load,
  Scratch& scratch,
  SystemFactor& prepared,
  bool refactor,
  float ceiling,
  const FreeModes& free
) {
  const size_t nodes = mesh.restPositions.size();

  scratch.stressed = true;

  assembleElastic(
    mesh,
    state,
    scratch,
    1.0f,
    state.position,
    materials,
    false,
    0.0f,
    ElementRotation::Polar
  );

  for (size_t node = 0u; node != nodes; ++node) {
    scratch.increment[node] = glm::vec3(0.0f);

    scratch.right[node] =
      mesh.anchored[node] != 0u
        ? glm::vec3(0.0f)
        : scratch.force[node] + mesh.nodeMass[node] * gravity
            + (node < load.size() ? load[node] : glm::vec3(0.0f));
  }

  double squared = 0.0;

  for (const glm::vec3& right : scratch.right) {
    squared += static_cast<double>(glm::dot(right, right));
  }

  const float residual = static_cast<float>(std::sqrt(squared));

  if (!(residual <= ceiling)) {
    return NewtonStep{0.0f, residual, false, false};
  }

  guardPreconditioner(mesh, scratch);

  const bool projected = free.count != 0u;

  Balance balance;
  float shift = 0.0f;

  if (projected) {
    balance = weigh(free, mesh, state, scratch.right, gravity, load);
    shift = equilibriumShift(mesh, scratch);

    projectForce(free, mesh, scratch.right);
  }

  if (refactor) {
    refreshFactor(prepared, mesh, state, materials, 1.0f, shift);
  }

  const CgResult result = conjugateGradient(
    mesh,
    state,
    scratch,
    0.0f,
    1.0f,
    settings.cgIterations,
    settings.tolerance,
    scratch.increment,
    prepared.empty() ? nullptr : &prepared,
    glm::mat3(1.0f),
    projected ? &free : nullptr
  );

  if (projected) {
    projectMotion(free, mesh, scratch.increment);
  }

  float moved = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    moved = glm::max(moved, glm::length(scratch.increment[node]));
  }

  return NewtonStep{
    moved,
    residual,
    !result.resolved,
    true,
    result.iterations,
    !prepared.empty(),
    shift,
    balance
  };
}
}

talus::DeformableState solveEquilibrium(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const talus::Pose& pose,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load
) {
  return solveEquilibrium(
    mesh,
    materials,
    restState(mesh, pose),
    settings,
    gravity,
    load
  );
}

talus::DeformableState solveEquilibrium(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const talus::DeformableState& start,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load
) {
  talus::DeformableState state = start;

  const size_t nodes = mesh.restPositions.size();

  Scratch scratch;
  sizeScratch(scratch, nodes);

  ElementStiffness store;

  refreshStiffness(store, mesh, materials);

  bindStiffness(scratch, store, mesh, materials, true);

  SystemFactor prepared;

  std::vector<glm::vec3> last(nodes, glm::vec3(0.0f));

  const bool backtracks = anyYields(mesh, materials);

  float accepted = std::numeric_limits<float>::infinity();

  for (uint32_t pass = 0u; pass != EQUILIBRIUM_PASSES; ++pass) {
    const NewtonStep taken = staticNewtonStep(
      mesh,
      state,
      materials,
      settings,
      gravity,
      load,
      scratch,
      prepared,
      pass == 0u,
      accepted,
      FreeModes{}
    );

    if (!taken.solved) {
      for (size_t node = 0u; node != nodes; ++node) {
        last[node] *= 0.5f;
        state.position[node] -= last[node];
      }

      continue;
    }

    if (backtracks) {
      accepted = taken.residual;
    }

    for (size_t node = 0u; node != nodes; ++node) {
      state.position[node] += scratch.increment[node];
      last[node] = scratch.increment[node];
    }

    if (!taken.capped && taken.moved < EQUILIBRIUM_MOTION) {
      break;
    }
  }

  scratch.stressed = false;

  assembleElastic(
    mesh,
    state,
    scratch,
    1.0f,
    state.position,
    materials,
    true,
    0.0f,
    ElementRotation::Polar
  );

  std::fill(state.velocity.begin(), state.velocity.end(), glm::vec3(0.0f));
  std::fill(
    state.acceleration.begin(),
    state.acceleration.end(),
    glm::vec3(0.0f)
  );

  return state;
}

EquilibriumCheck checkEquilibrium(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& current,
  std::span<const talus::PhysicalMaterial> materials,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load
) {
  talus::DeformableState probed = current;

  Scratch scratch;
  sizeScratch(scratch, mesh.restPositions.size());

  ElementStiffness store;

  refreshStiffness(store, mesh, materials);

  bindStiffness(scratch, store, mesh, materials, true);

  SystemFactor prepared;

  const FreeModes free = freeModes(mesh, probed);

  const NewtonStep taken = staticNewtonStep(
    mesh,
    probed,
    materials,
    settings,
    gravity,
    load,
    scratch,
    prepared,
    true,
    std::numeric_limits<float>::infinity(),
    free
  );

  return EquilibriumCheck{
    taken.moved,
    taken.capped,
    taken.balance.unbalanced,
    taken.factored,
    free.count,
    taken.iterations,
    taken.balance.force,
    taken.balance.torque,
    taken.shift
  };
}

namespace {

struct Contact {
  glm::uvec3 nodes{0u};
  glm::vec3 weights{0.0f};
  size_t source = 0u;
  size_t partner = 0u;
  bool dynamic = false;
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
  glm::vec3 tangent{1.0f, 0.0f, 0.0f};
  glm::vec3 bitangent{0.0f, 0.0f, 1.0f};
  glm::vec3 point{0.0f};
  glm::vec3 arm{0.0f};
  glm::vec3 rigidNormalLinear{0.0f};
  glm::vec3 rigidNormalAngular{0.0f};
  float gap = 0.0f;
  float bias = 0.0f;
  float friction = 0.5f;
  float area = 0.0f;
  float normalTarget = 0.0f;
  glm::vec3 lambda{0.0f};

  [[nodiscard]] const glm::vec3& direction(size_t row) const {
    return row == 0u ? normal : row == 1u ? tangent : bitangent;
  }

  [[nodiscard]] glm::vec3 row(int corner, size_t axis) const {
    return weights[corner] * direction(axis);
  }

  [[nodiscard]] glm::vec3 rigidLinear(size_t axis) const {
    return axis == 0u ? rigidNormalLinear : -area * direction(axis);
  }

  [[nodiscard]] glm::vec3 rigidAngular(size_t axis) const {
    return axis == 0u ? rigidNormalAngular
                      : -glm::cross(arm, area * direction(axis));
  }

  [[nodiscard]] float target(size_t axis) const {
    return axis == 0u ? normalTarget : 0.0f;
  }
};

void tangentsOf(
  const glm::vec3& normal,
  glm::vec3& tangent,
  glm::vec3& bitangent
) {
  const glm::vec3 helper = std::abs(normal.x) < 0.9f
                             ? glm::vec3(1.0f, 0.0f, 0.0f)
                             : glm::vec3(0.0f, 1.0f, 0.0f);

  tangent = glm::normalize(glm::cross(normal, helper));
  bitangent = glm::cross(normal, tangent);
}

glm::quat spun(
  const glm::quat& rotation,
  const glm::vec3& angular,
  float elapsed
) {
  const float spin = glm::length(angular);

  return spin > 0.0f ? glm::angleAxis(spin * elapsed, angular / spin) * rotation
                     : rotation;
}

struct FloatingFrame {
  glm::vec3 centre{0.0f};
  glm::vec3 velocity{0.0f};
  glm::vec3 acceleration{0.0f};
  glm::vec3 nextCentre{0.0f};
  glm::vec3 nextVelocity{0.0f};
  glm::vec3 nextAcceleration{0.0f};
  glm::vec3 momentum{0.0f};
  glm::vec3 torque{0.0f};
  glm::vec3 spin{0.0f};
  glm::vec3 spinRate{0.0f};
  glm::vec3 weightedSpin{0.0f};
  glm::vec3 weightedRate{0.0f};
  glm::vec3 endSpin{0.0f};
  glm::vec3 endRate{0.0f};
  glm::mat3 turn{1.0f};
  glm::mat3 weightedTurn{1.0f};
};

bool floats(const talus::PreparedMesh& mesh, const Settings& settings) {
  return settings.rigidSplit != RigidSplit::Off && !mesh.nodeMass.empty()
         && std::none_of(
           mesh.anchored.begin(),
           mesh.anchored.end(),
           [](uint8_t held) { return held != 0u; }
         );
}

glm::dmat3 skew(const glm::dvec3& axis) {
  return glm::dmat3(
    glm::dvec3(0.0, axis.z, -axis.y),
    glm::dvec3(-axis.z, 0.0, axis.x),
    glm::dvec3(axis.y, -axis.x, 0.0)
  );
}

}

RigidMotion rigidRate(const glm::dmat3& inverse, const RigidMotion& at) {
  const glm::dvec3 spin = inverse * at.momentum;

  return RigidMotion{glm::cross(at.momentum, spin), at.turn * skew(spin)};
}

RigidMotion rigidAdvance(
  const glm::dmat3& inverse,
  RigidMotion motion,
  double span,
  uint32_t count
) {
  const double part = span / static_cast<double>(count);

  const auto moved =
    [](const RigidMotion& from, const RigidMotion& rate, double by) {
      return RigidMotion{
        from.momentum + by * rate.momentum,
        from.turn + by * rate.turn
      };
    };

  for (uint32_t at = 0u; at != count; ++at) {
    const RigidMotion first = rigidRate(inverse, motion);
    const RigidMotion second =
      rigidRate(inverse, moved(motion, first, 0.5 * part));
    const RigidMotion third =
      rigidRate(inverse, moved(motion, second, 0.5 * part));
    const RigidMotion fourth = rigidRate(inverse, moved(motion, third, part));

    motion.momentum +=
      part / 6.0
      * (first.momentum + 2.0 * second.momentum + 2.0 * third.momentum + fourth.momentum);
    motion.turn +=
      part / 6.0
      * (first.turn + 2.0 * second.turn + 2.0 * third.turn + fourth.turn);
  }

  for (uint32_t pass = 0u; pass != 3u; ++pass) {
    motion.turn =
      0.5 * (motion.turn + glm::transpose(glm::inverse(motion.turn)));
  }

  return motion;
}

namespace {

FloatingFrame floatingFrame(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const IntegratorRule& rule,
  float inner,
  const glm::vec3& gravity
) {
  glm::dvec3 centre(0.0);
  glm::dvec3 velocity(0.0);
  glm::dvec3 acceleration(0.0);
  double total = 0.0;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    total += mass;
    centre += mass * glm::dvec3(state.position[node]);
    velocity += mass * glm::dvec3(state.velocity[node]);
    acceleration += mass * glm::dvec3(state.acceleration[node]);
  }

  centre /= total;
  velocity /= total;
  acceleration /= total;

  glm::dmat3 inertia(0.0);
  glm::dvec3 momentum(0.0);
  glm::dvec3 torque(0.0);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3 arm = glm::dvec3(state.position[node]) - centre;

    inertia +=
      mass
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
    momentum +=
      mass * glm::cross(arm, glm::dvec3(state.velocity[node]) - velocity);
    torque +=
      mass
      * glm::cross(arm, glm::dvec3(state.acceleration[node]) - acceleration);
  }

  FloatingFrame made;

  const double h = static_cast<double>(inner);
  const double alphaMass = static_cast<double>(rule.alphaMass);
  const double beta = static_cast<double>(rule.beta);
  const double gamma = static_cast<double>(rule.gamma);
  const glm::dvec3 next =
    (glm::dvec3(gravity) - alphaMass * acceleration) / (1.0 - alphaMass);

  made.centre = glm::vec3(centre);
  made.velocity = glm::vec3(velocity);
  made.acceleration = glm::vec3(acceleration);
  made.nextAcceleration = glm::vec3(next);
  made.nextCentre = glm::vec3(
    centre + h * velocity + h * h * ((0.5 - beta) * acceleration + beta * next)
  );
  made.nextVelocity =
    glm::vec3(velocity + h * ((1.0 - gamma) * acceleration + gamma * next));
  made.momentum = glm::vec3(
    momentum
    + h * (1.0 - gamma - gamma * alphaMass / (1.0 - alphaMass)) * torque
  );
  made.torque = glm::vec3(-alphaMass / (1.0 - alphaMass) * torque);

  const double determinant = glm::determinant(inertia);
  const double scale = inertia[0][0] + inertia[1][1] + inertia[2][2];

  if (
    !(determinant > 1.0e-12 * scale * scale * scale)
    || glm::dot(momentum, momentum) == 0.0
  ) {
    return made;
  }

  const glm::dmat3 inverse = glm::inverse(inertia);
  const double weighted = (1.0 - static_cast<double>(rule.alphaForce)) * h;

  const RigidMotion start{momentum, glm::dmat3(1.0)};
  const RigidMotion middle =
    rigidAdvance(inverse, start, weighted, RIGID_PARTS);
  const RigidMotion end =
    rigidAdvance(inverse, middle, h - weighted, RIGID_PARTS);

  const auto spinOf = [&](const RigidMotion& at) {
    return inverse * at.momentum;
  };

  const auto rateOf = [&](const RigidMotion& at) {
    return inverse * glm::cross(at.momentum, inverse * at.momentum);
  };

  made.spin = glm::vec3(spinOf(start));
  made.spinRate = glm::vec3(rateOf(start));
  made.weightedSpin = glm::vec3(spinOf(middle));
  made.weightedRate = glm::vec3(rateOf(middle));
  made.endSpin = glm::vec3(end.turn * spinOf(end));
  made.endRate = glm::vec3(end.turn * rateOf(end));
  made.turn = glm::mat3(end.turn);
  made.weightedTurn = glm::mat3(middle.turn);

  return made;
}

struct RigidPart {
  glm::dvec3 centre{0.0};
  glm::dvec3 spin{0.0};
  bool posed = false;
};

RigidPart rigidPart(
  const talus::PreparedMesh& mesh,
  const std::vector<glm::vec3>& at,
  const std::vector<glm::vec3>& moved
) {
  RigidPart part;
  glm::dvec3 mean(0.0);
  double total = 0.0;

  for (size_t node = 0u; node != at.size(); ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    total += mass;
    part.centre += mass * glm::dvec3(at[node]);
    mean += mass * glm::dvec3(moved[node]);
  }

  part.centre /= total;
  mean /= total;

  glm::dmat3 inertia(0.0);
  glm::dvec3 momentum(0.0);

  for (size_t node = 0u; node != at.size(); ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3 arm = glm::dvec3(at[node]) - part.centre;

    inertia +=
      mass
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
    momentum += mass * glm::cross(arm, glm::dvec3(moved[node]) - mean);
  }

  if (!(glm::determinant(inertia) > 0.0)) {
    return part;
  }

  part.spin = glm::inverse(inertia) * momentum;
  part.posed = glm::length(part.spin) > 0.0;

  return part;
}

glm::dmat3 rigidTurn(const RigidPart& part, double span) {
  const double length = glm::length(part.spin);

  return glm::mat3_cast(glm::angleAxis(span * length, part.spin / length));
}

glm::vec3 linearPart(
  const RigidPart& part,
  const glm::vec3& at,
  const glm::vec3& moved
) {
  return moved - glm::vec3(glm::cross(part.spin, glm::dvec3(at) - part.centre));
}

std::vector<glm::dvec3> arcDeparture(
  const talus::PreparedMesh& mesh,
  const IntegratorRule& rule,
  float inner,
  const RigidPart& part,
  const std::vector<glm::vec3>& freeEnd,
  const std::vector<glm::vec3>& correction,
  const Scratch& scratch,
  const talus::DeformableState& state
) {
  const size_t nodes = state.position.size();
  const double h = static_cast<double>(inner);
  const double spanned = static_cast<double>(rule.beta) * h * h;

  std::vector<glm::dvec3> departure(nodes, glm::dvec3(0.0));
  std::vector<glm::dvec3> reached(nodes);
  std::vector<glm::dvec3> chord(nodes);

  double total = 0.0;
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3 start(scratch.shifted[node]);
    const glm::dvec3 pull(state.acceleration[node]);
    const glm::dvec3 pushed(correction[node]);
    const glm::dvec3 linear =
      pushed - glm::cross(part.spin, glm::dvec3(freeEnd[node]) - part.centre);

    reached[node] = start + spanned * (pull + linear / h);
    chord[node] = start + spanned * (pull + pushed / h);
    total += mass;
    centre += mass * reached[node];
  }

  centre /= total;

  const glm::dmat3 turn = rigidTurn(part, static_cast<double>(rule.beta) * h);

  for (size_t node = 0u; node != nodes; ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    departure[node] =
      (centre + turn * (reached[node] - centre) - chord[node]) / h;
  }

  return departure;
}

bool poseRigid(
  const talus::PreparedMesh& mesh,
  const IntegratorRule& rule,
  float inner,
  const RigidPart& part,
  const std::vector<glm::vec3>& freeEnd,
  const std::vector<glm::vec3>& correction,
  bool damped,
  Scratch& scratch,
  talus::DeformableState& state,
  std::vector<glm::vec3>& velocities,
  std::vector<glm::vec3>& posedEnd,
  double& kineticChange
) {
  const size_t nodes = state.position.size();
  const double h = static_cast<double>(inner);

  std::vector<glm::dvec3> arm(nodes);
  std::vector<glm::dvec3> drift(nodes);

  double total = 0.0;
  glm::dvec3 centre(0.0);
  glm::dvec3 velocity(0.0);
  glm::dvec3 acceleration(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);
    const glm::vec3 linear = linearPart(part, freeEnd[node], correction[node]);

    arm[node] = glm::dvec3(
      scratch.shifted[node]
      + rule.beta * inner * inner * (state.acceleration[node] + linear / inner)
    );
    total += mass;
    centre += mass * arm[node];
    velocity += mass * glm::dvec3(velocities[node]);
    acceleration += mass * glm::dvec3(state.acceleration[node]);
  }

  centre /= total;
  velocity /= total;
  acceleration /= total;

  glm::dmat3 inertia(0.0);
  glm::dvec3 momentum(0.0);
  glm::dvec3 torque(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    arm[node] -= centre;
    inertia +=
      mass
      * (glm::dot(arm[node], arm[node]) * glm::dmat3(1.0) - glm::outerProduct(arm[node], arm[node]));
    momentum +=
      mass * glm::cross(arm[node], glm::dvec3(velocities[node]) - velocity);
    torque += mass
              * glm::cross(
                arm[node],
                glm::dvec3(state.acceleration[node]) - acceleration
              );
  }

  if (!(glm::determinant(inertia) > 0.0)) {
    return false;
  }

  const glm::dmat3 inverse = glm::inverse(inertia);
  const glm::dvec3 spin = inverse * momentum;

  glm::dvec3 gyroscopic(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    drift[node] =
      glm::dvec3(velocities[node]) - velocity - glm::cross(spin, arm[node]);
    gyroscopic += mass
                  * glm::cross(
                    arm[node],
                    glm::cross(spin, glm::cross(spin, arm[node]))
                      + 2.0 * glm::cross(spin, drift[node])
                  );
  }

  const glm::dvec3 rate = inverse * (torque - gyroscopic);
  const glm::dmat3 turn = rigidTurn(part, static_cast<double>(rule.beta) * h);
  const glm::dmat3 turnedInverse = turn * inverse * glm::transpose(turn);
  const glm::dvec3 reposed = turnedInverse * momentum;

  glm::dvec3 reposedGyroscopic(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3 placed = turn * arm[node];
    const glm::dvec3 moving = turn * drift[node];

    reposedGyroscopic += mass
                         * glm::cross(
                           placed,
                           glm::cross(reposed, glm::cross(reposed, placed))
                             + 2.0 * glm::cross(reposed, moving)
                         );
  }

  const glm::dvec3 reposedRate = turnedInverse * (torque - reposedGyroscopic);

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::dvec3 placed = turn * arm[node];
    const glm::dvec3 moving = turn * drift[node];
    const glm::dvec3 deforming = glm::dvec3(state.acceleration[node])
                                 - acceleration - glm::cross(rate, arm[node])
                                 - glm::cross(spin, glm::cross(spin, arm[node]))
                                 - 2.0 * glm::cross(spin, drift[node]);

    posedEnd[node] = glm::vec3(centre + placed);
    velocities[node] =
      glm::vec3(velocity + glm::cross(reposed, placed) + moving);
    state.acceleration[node] = glm::vec3(
      acceleration + glm::cross(reposedRate, placed)
      + glm::cross(reposed, glm::cross(reposed, placed))
      + 2.0 * glm::cross(reposed, moving) + turn * deforming
    );
  }

  kineticChange = 0.5 * glm::dot(momentum, reposed - spin);

  const glm::mat3 held(turn);

  for (talus::CommittedTet& committed : state.committed) {
    committed.rotation = held * committed.rotation;
  }

  if (damped) {
    turnDamping(scratch.damping, held);
  }

  return true;
}

void captureSystem(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const Scratch& scratch,
  const EffectiveSystem& effective,
  const Incidence& incidence,
  const std::vector<glm::vec3>& read,
  const glm::mat3& turn,
  StepReport& report
) {
  const size_t nodes = mesh.restPositions.size();
  const size_t rows = 3u * nodes;

  report.systemPositions.assign(nodes, glm::vec3(0.0f));

  for (size_t node = 0u; node != nodes; ++node) {
    report.systemPositions[node] = turn * read[node];
  }

  report.systemRotations.clear();

  for (const talus::CommittedTet& held : state.committed) {
    report.systemRotations.push_back(held.rotation);
  }

  report.systemDiagonal = scratch.diagonal;
  report.systemOperator.assign(rows * rows, 0.0f);
  report.systemDamping.assign(rows * rows, 0.0f);
  report.systemSweep.assign(rows * rows, 0.0f);

  std::vector<glm::vec3> unit(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> produced(nodes, glm::vec3(0.0f));

  const auto keep = [&](std::vector<float>& matrix, size_t column) {
    for (size_t node = 0u; node != nodes; ++node) {
      for (int axis = 0; axis != 3; ++axis) {
        matrix[(3u * node + static_cast<size_t>(axis)) * rows + column] =
          produced[node][axis];
      }
    }
  };

  for (size_t node = 0u; node != nodes; ++node) {
    for (int axis = 0; axis != 3; ++axis) {
      const size_t column = 3u * node + static_cast<size_t>(axis);

      unit[node][axis] = 1.0f;

      applySystem(
        mesh,
        state,
        scratch,
        effective.inertia,
        effective.step,
        unit,
        produced
      );
      keep(report.systemOperator, column);

      applySystem(mesh, state, scratch, 0.0f, 0.0f, unit, produced);
      keep(report.systemDamping, column);

      std::fill(produced.begin(), produced.end(), glm::vec3(0.0f));
      gaussSeidelSweep(
        mesh,
        state,
        scratch,
        effective.step,
        incidence,
        unit,
        produced
      );
      keep(report.systemSweep, column);

      unit[node][axis] = 0.0f;
    }
  }
}

}

void step(
  const talus::PreparedMesh& mesh,
  talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  const Settings& settings,
  const glm::vec3& gravity,
  float deltaTime,
  std::span<const SurfaceContact> contacts,
  std::span<RigidPartner> partners,
  DeformableSolverState* solver,
  StepReport& report
) {
  const auto began = std::chrono::steady_clock::now();

  const uint32_t configured = glm::max(settings.substeps, 1u);
  const IntegratorRule rule = integratorRule(settings.spectralRadius);
  const bool floating = floats(mesh, settings);

  const auto setupBegan = std::chrono::steady_clock::now();

  const size_t nodes = mesh.restPositions.size();

  if (state.acceleration.size() != nodes) {
    state.acceleration.assign(nodes, glm::vec3(0.0f));
  }

  const auto scratchBegan = std::chrono::steady_clock::now();

  Scratch scratch;
  sizeScratch(scratch, nodes);

  const auto stiffnessBegan = std::chrono::steady_clock::now();

  report.setupScratchSeconds +=
    std::chrono::duration<double>(stiffnessBegan - scratchBegan).count();

  ElementStiffness local;
  ElementStiffness& store = solver != nullptr ? solver->stiffness : local;

  if (refreshStiffness(store, mesh, materials)) {
    ++report.stiffnessRebuilds;
  }

  const bool newton = settings.plasticPasses > 1u && anyYields(mesh, materials);
  const bool contactStep =
    std::ranges::any_of(contacts, [&](const SurfaceContact& held) {
      return held.partner >= 0
             && static_cast<size_t>(held.partner) < partners.size();
    });
  const uint32_t plasticCap =
    contactStep ? CONTACT_PLASTIC_PASSES : settings.plasticPasses;

  bindStiffness(scratch, store, mesh, materials, false);

  const bool damped = bindDamping(scratch.damping, mesh, materials, store);

  const auto incidenceBegan = std::chrono::steady_clock::now();

  report.setupStiffnessSeconds +=
    std::chrono::duration<double>(incidenceBegan - stiffnessBegan).count();

  const Incidence incidence = buildIncidence(mesh);

  const auto allocateBegan = std::chrono::steady_clock::now();

  report.setupIncidenceSeconds +=
    std::chrono::duration<double>(allocateBegan - incidenceBegan).count();

  std::vector<glm::vec3> unconstrained(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> advance(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> pushed(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> correction(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> contactVelocity(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> delivered;
  std::vector<float> matrix;
  std::vector<float> free;
  std::vector<float> current;
  std::vector<float> lambda;
  std::vector<Contact> active;
  std::vector<Contact> pressing;
  std::vector<glm::vec3> lambdas(contacts.size(), glm::vec3(0.0f));
  std::vector<glm::vec3> partnerLinear(partners.size(), glm::vec3(0.0f));
  std::vector<glm::vec3> partnerAngular(partners.size(), glm::vec3(0.0f));
  std::vector<glm::vec3> partnerCenter(partners.size(), glm::vec3(0.0f));
  std::vector<glm::quat> partnerRotation(partners.size());
  std::vector<glm::vec3> total(newton ? nodes : 0u, glm::vec3(0.0f));
  std::vector<glm::vec3> evaluated(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> lowestIterate(newton ? nodes : 0u, glm::vec3(0.0f));
  const float tolerance = floating
                            ? glm::min(settings.tolerance, FLOATING_TOLERANCE)
                            : settings.tolerance;
  std::vector<glm::vec3> drive(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> relative(floating ? nodes : 0u, glm::vec3(0.0f));

  active.reserve(contacts.size());

  report.setupAllocateSeconds +=
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - allocateBegan
    )
      .count();

  const uint32_t substeps = !floating
                                && predictorInverts(
                                  mesh,
                                  state,
                                  rule,
                                  deltaTime / static_cast<float>(configured),
                                  scratch.weighted
                                )
                              ? glm::max(configured, INVERTED_SUBSTEPS)
                              : configured;
  const float inner = deltaTime / static_cast<float>(substeps);
  const EffectiveSystem effective = effectiveSystem(rule, inner);
  const float carryScale = (0.5f - rule.beta) * inner * inner;
  const ElementRotation rotation =
    floating ? ElementRotation::Polar : ElementRotation::Invertible;

  scratch.damping.scale = (1.0f - rule.alphaForce) * rule.gamma * inner;

  const float dampingStep = damped ? scratch.damping.scale : 0.0f;
  const std::vector<float> dampingKey = laneDamping(materials, dampingStep);
  std::vector<glm::vec3> viscous(damped ? nodes : 0u, glm::vec3(0.0f));
  std::vector<glm::vec3> viscousVelocity(damped ? nodes : 0u, glm::vec3(0.0f));

  report.setupSeconds +=
    std::chrono::duration<double>(std::chrono::steady_clock::now() - setupBegan)
      .count();

  for (size_t partner = 0u; partner != partners.size(); ++partner) {
    partnerCenter[partner] = partners[partner].centerOfMass;
    partnerRotation[partner] = partners[partner].rotation;
  }

  if (solver != nullptr) {
    report.factorRows = solver->factor.rows();
    report.factorNonZeros = solver->factor.factorNonZeros();
    report.systemNonZeros = solver->factor.systemNonZeros();
  }

  report.nodes += static_cast<uint32_t>(nodes);
  report.elements += static_cast<uint32_t>(
    mesh.hexes.empty() ? mesh.tets.size() : mesh.hexes.size()
  );

  float elastic = 0.0f;

  const uint32_t OUTER_PASSES = glm::max(settings.contactIterations, 1u);
  constexpr float RELAXATION = 0.8f;
  ContactSolve solve;

  const auto recordCg = [&](const CgResult& result) {
    report.cgIterations += result.iterations;
    report.cgCappedSolves += result.capped ? 1u : 0u;
    report.primaryCgOperatorApplications += result.operatorTiming.applications;
    report.primaryCgMassSeconds += result.operatorTiming.massSeconds;
    report.primaryCgTetSeconds += result.operatorTiming.tetSeconds;
    report.primaryCgAnchorSeconds += result.operatorTiming.anchorSeconds;
    report.primaryCgConditionSeconds += result.conditionSeconds;
    report.factorPackSeconds += result.factorTiming.packSeconds;
    report.factorSolveSeconds += result.factorTiming.solveSeconds;
    report.factorUnpackSeconds += result.factorTiming.unpackSeconds;
    report.factorApplications += result.factorTiming.applications;
    report.cgWorstRelativeResidual =
      glm::max(report.cgWorstRelativeResidual, result.relativeResidual);
  };

  if (solver != nullptr
      && (solver->factor.empty()
          || solver->factor.reference().step != effective.step
          || solver->factor.reference().inertia != effective.inertia
          || solver->factor.reference().damping != dampingKey)) {
    refreshFactor(
      solver->factor,
      mesh,
      state,
      materials,
      effective.step,
      effective.inertia,
      dampingStep
    );
    solver->stats.factorAge = 0u;
    ++solver->stats.refactorCount;
  }

  for (uint32_t pass = 0u; pass != substeps; ++pass) {
    const FloatingFrame body =
      floating ? floatingFrame(mesh, state, rule, inner, gravity)
               : FloatingFrame{};

    if (floating) {
      const glm::vec3 spin = body.spin;
      const glm::vec3 held = body.weightedSpin;
      const float lag = (1.0f - rule.alphaForce) * (1.0f - rule.gamma) * inner;

      for (size_t node = 0u; node != nodes; ++node) {
        const glm::vec3 arm = state.position[node] - body.centre;
        const glm::vec3 moving =
          state.velocity[node] - body.velocity - glm::cross(spin, arm);
        const glm::vec3 turning = state.acceleration[node] - body.acceleration
                                  - glm::cross(body.spinRate, arm)
                                  - glm::cross(spin, glm::cross(spin, arm))
                                  - 2.0f * glm::cross(spin, moving);
        const float mass = mesh.nodeMass[node];

        relative[node] = moving;
        scratch.carried[node] = turning;
        scratch.shifted[node] = arm + inner * moving + carryScale * turning;
        scratch.weighted[node] =
          (1.0f - rule.alphaForce) * scratch.shifted[node]
          + rule.alphaForce * arm;
        drive[node] =
          -mass
          * (glm::cross(held, glm::cross(held, scratch.weighted[node]))
             + glm::cross(body.weightedRate, scratch.weighted[node])
             + 2.0f * glm::cross(held, moving + lag * turning));
        state.acceleration[node] = turning;
      }
    } else {
      for (size_t node = 0u; node != nodes; ++node) {
        scratch.carried[node] = state.acceleration[node];
        scratch.shifted[node] = state.position[node]
                                + inner * state.velocity[node]
                                + carryScale * state.acceleration[node];
        scratch.weighted[node] =
          (1.0f - rule.alphaForce) * scratch.shifted[node]
          + rule.alphaForce * state.position[node];
        drive[node] = mesh.nodeMass[node] * gravity;
      }
    }

    const std::vector<glm::vec3>* assembledAt = &scratch.weighted;

    const auto assemble = [&](
                            const std::vector<glm::vec3>& at,
                            float margin,
                            bool returnMapping = true
                          ) {
      const auto elasticBegan = std::chrono::steady_clock::now();

      const Assembly assembled = assembleElastic(
        mesh,
        state,
        scratch,
        effective.step,
        at,
        materials,
        false,
        margin,
        rotation,
        returnMapping
      );

      assembledAt = &at;

      if (damped) {
        fitDamping(scratch.damping, mesh, at);

        if (solver == nullptr || solver->factor.empty()) {
          blockDamping(mesh, state, scratch.damping);
          addDampingDiagonal(mesh, scratch.damping, scratch.diagonal);
        } else {
          scratch.damping.blocks.clear();
        }
      }

      report.elasticSeconds += std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - elasticBegan
      )
                                 .count();

      return assembled;
    };

    const auto balance = [&](const std::vector<glm::vec3>* taken) {
      if (damped) {
        const float lag = (1.0f - rule.alphaForce) * inner;

        for (size_t node = 0u; node != nodes; ++node) {
          if (mesh.anchored[node] != 0u) {
            viscousVelocity[node] = glm::vec3(0.0f);

            continue;
          }

          viscousVelocity[node] =
            (floating ? relative[node] : state.velocity[node])
            + lag * (1.0f - rule.gamma) * scratch.carried[node];

          if (taken != nullptr) {
            viscousVelocity[node] += lag * rule.gamma * (*taken)[node];
          }
        }

        std::fill(viscous.begin(), viscous.end(), glm::vec3(0.0f));

        addDamping(
          mesh,
          state,
          scratch.damping,
          1.0f,
          viscousVelocity,
          viscous
        );
      }

      for (size_t node = 0u; node != nodes; ++node) {
        scratch.previous[node] = state.velocity[node];

        if (mesh.anchored[node] != 0u) {
          state.velocity[node] = glm::vec3(0.0f);
          state.acceleration[node] = glm::vec3(0.0f);
          scratch.right[node] = glm::vec3(0.0f);

          continue;
        }

        const float mass = mesh.nodeMass[node];

        scratch.right[node] = scratch.force[node] + drive[node]
                              - rule.alphaMass * mass * scratch.carried[node];

        if (taken != nullptr) {
          scratch.right[node] -= effective.inertia * mass * (*taken)[node];
        }

        if (damped) {
          scratch.right[node] -= viscous[node];
        }

        scratch.diagonal[node] += glm::mat3(effective.inertia * mass);
      }

      guardPreconditioner(mesh, scratch);
    };

    const SystemFactor* prepared =
      solver != nullptr && !solver->factor.empty() ? &solver->factor : nullptr;

    glm::mat3 frame = prepared != nullptr
                        ? bodyRotation(mesh, state, prepared->reference())
                        : glm::mat3(1.0f);

    const uint32_t budget =
      prepared != nullptr && settings.factorBudget != 0u
        ? glm::min(settings.factorBudget, settings.cgIterations)
        : settings.cgIterations;

    const auto solvePrimary =
      [&](std::vector<glm::vec3>& solution, bool refreshes) {
        const auto primaryCgBegan = std::chrono::steady_clock::now();

        scratch.restart = solution;

        CgResult primaryResult = conjugateGradient(
          mesh,
          state,
          scratch,
          effective.inertia,
          effective.step,
          budget,
          tolerance,
          solution,
          prepared,
          frame
        );

        if (primaryResult.capped && prepared != nullptr && refreshes) {
          ++report.refreshRequests;
          report.refreshedAtAge = solver->stats.substepsSinceRefresh;
          report.spentBeforeRefresh = primaryResult.iterations;
          report.wastedIterations += primaryResult.iterations;
          solver->stats.refreshRequested = true;

          solution = scratch.restart;

          const auto refactorBegan = std::chrono::steady_clock::now();

          refreshFactor(
            solver->factor,
            mesh,
            state,
            materials,
            effective.step,
            effective.inertia,
            dampingStep
          );

          if (damped && solver->factor.empty()) {
            blockDamping(mesh, state, scratch.damping);
            addDampingDiagonal(mesh, scratch.damping, scratch.diagonal);
            guardPreconditioner(mesh, scratch);
          }

          report.refactorSeconds +=
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - refactorBegan
            )
              .count();

          ++report.refactors;
          ++report.retries;
          ++solver->stats.refactorCount;
          ++solver->stats.retryCount;
          solver->stats.factorAge = 0u;
          solver->stats.substepsSinceRefresh = 0u;
          frame = glm::mat3(1.0f);

          primaryResult = conjugateGradient(
            mesh,
            state,
            scratch,
            effective.inertia,
            effective.step,
            settings.cgIterations,
            tolerance,
            solution,
            &solver->factor,
            frame
          );

          report.spentAfterRefresh = primaryResult.iterations;

          if (primaryResult.capped) {
            ++report.retriesFailed;
            solver->stats.freshRetryFailed = true;
          }
        }

        report.primaryCgSeconds +=
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - primaryCgBegan
          )
            .count();

        recordCg(primaryResult);

        return primaryResult;
      };

    CgResult primaryResult;

    if (!newton) {
      elastic = assemble(scratch.weighted, 0.0f).energy;

      balance(nullptr);

      primaryResult = solvePrimary(state.acceleration, true);

      const float reach = effective.step * effective.step;

      for (uint32_t read = 1u; read != ANCHORED_READS; ++read) {
        for (size_t node = 0u; node != nodes; ++node) {
          evaluated[node] =
            scratch.weighted[node] + reach * state.acceleration[node];
        }

        elastic = assemble(evaluated, 0.0f).energy;

        balance(&state.acceleration);

        std::fill(
          scratch.increment.begin(),
          scratch.increment.end(),
          glm::vec3(0.0f)
        );

        const CgResult again = solvePrimary(scratch.increment, false);

        for (size_t node = 0u; node != nodes; ++node) {
          if (mesh.anchored[node] == 0u) {
            state.acceleration[node] += scratch.increment[node];
          }
        }

        primaryResult.iterations += again.iterations;
        primaryResult.relativeResidual = again.relativeResidual;
        primaryResult.capped = primaryResult.capped || again.capped;
      }
    } else {
      const float reach = effective.step * effective.step;
      const float travel = rule.beta * inner * inner;

      uint32_t spent = 0u;
      bool linearAllowed = true;

      for (;;) {
        std::fill(total.begin(), total.end(), glm::vec3(0.0f));

        double accepted = std::numeric_limits<double>::infinity();
        double opening = 0.0;
        double lowest = std::numeric_limits<double>::infinity();
        uint32_t loadingBefore = 0u;
        uint32_t solves = 0u;
        bool converged = false;
        bool lowestIsLast = false;
        bool predictorUnloaded = false;
        bool handedOff = false;

        for (uint32_t attempt = 0u; attempt != plasticCap; ++attempt) {
          if (attempt < 2u) {
            bindStiffness(scratch, store, mesh, materials, attempt != 0u);
          }

          for (size_t node = 0u; node != nodes; ++node) {
            evaluated[node] = scratch.weighted[node] + reach * total[node];
          }

          const Assembly assembled = assemble(evaluated, LOADING_MARGIN);

          elastic = assembled.energy;
          ++report.plasticEvaluations;

          balance(&total);

          double squared = 0.0;

          for (const glm::vec3& right : scratch.right) {
            squared += static_cast<double>(glm::dot(right, right));
          }

          if (attempt != 0u) {
            const double ranked = std::isfinite(squared)
                                    ? squared
                                    : std::numeric_limits<double>::infinity();

            lowestIsLast = attempt == 1u || ranked < lowest;

            if (lowestIsLast) {
              lowest = ranked;
              std::copy(total.begin(), total.end(), lowestIterate.begin());
            }
          }

          if (attempt == 0u) {
            predictorUnloaded = assembled.loading == 0u;
          }

          if (
            attempt == 1u && linearAllowed && predictorUnloaded
            && std::isfinite(squared) && squared > accepted
          ) {
            ++report.plasticLinearReads;

            bindStiffness(scratch, store, mesh, materials, false);

            for (uint32_t read = 1u; read != ANCHORED_READS; ++read) {
              for (size_t node = 0u; node != nodes; ++node) {
                evaluated[node] = scratch.weighted[node] + reach * total[node];
              }

              const Assembly reread = assemble(evaluated, LOADING_MARGIN);

              elastic = reread.energy;
              ++report.plasticEvaluations;

              if (reread.loading != 0u) {
                handedOff = true;

                break;
              }

              balance(&total);

              std::fill(
                scratch.increment.begin(),
                scratch.increment.end(),
                glm::vec3(0.0f)
              );

              const CgResult solved = solvePrimary(scratch.increment, false);

              primaryResult.relativeResidual = solved.relativeResidual;
              spent += solved.iterations;

              ++solves;

              for (size_t node = 0u; node != nodes; ++node) {
                if (mesh.anchored[node] == 0u) {
                  total[node] += scratch.increment[node];
                }
              }
            }

            converged = !handedOff;

            break;
          }

          if (attempt != 0u && !(squared <= accepted)) {
            for (size_t node = 0u; node != nodes; ++node) {
              scratch.increment[node] *= 0.5f;
              total[node] -= scratch.increment[node];
            }

            ++report.plasticBacktracks;

            continue;
          }

          if (attempt != 0u && loadingBefore == 0u && assembled.loading == 0u) {
            converged = true;

            break;
          }

          opening = attempt == 0u ? squared : opening;
          accepted = squared;
          loadingBefore = assembled.loading;

          if (attempt == 0u) {
            scratch.increment = state.acceleration;
          } else {
            std::fill(
              scratch.increment.begin(),
              scratch.increment.end(),
              glm::vec3(0.0f)
            );
          }

          const CgResult solved =
            solvePrimary(scratch.increment, attempt == 0u);

          primaryResult.startResidual =
            attempt == 0u ? solved.startResidual : primaryResult.startResidual;
          primaryResult.relativeResidual = solved.relativeResidual;
          spent += solved.iterations;

          ++solves;

          float moved = 0.0f;
          float extent = 0.0f;

          for (size_t node = 0u; node != nodes; ++node) {
            if (mesh.anchored[node] != 0u) {
              scratch.increment[node] = glm::vec3(0.0f);

              continue;
            }

            total[node] += scratch.increment[node];
            moved = glm::max(moved, glm::length(scratch.increment[node]));
            extent = glm::max(extent, glm::length(evaluated[node]));
          }

          if (
            solves > 1u
            && travel * moved <= PLASTIC_RESOLUTION
                                   * std::numeric_limits<float>::epsilon()
                                   * extent
          ) {
            converged = true;

            break;
          }
        }

        report.plasticSolves += solves;

        if (handedOff) {
          ++report.plasticHandoffs;
          linearAllowed = false;

          continue;
        }

        if (!converged) {
          bindStiffness(scratch, store, mesh, materials, false);

          std::fill(total.begin(), total.end(), glm::vec3(0.0f));

          bool fellBack = true;

          for (uint32_t read = 0u; read != ANCHORED_READS; ++read) {
            for (size_t node = 0u; node != nodes; ++node) {
              evaluated[node] = scratch.weighted[node] + reach * total[node];
            }

            const Assembly reread =
              assemble(evaluated, LOADING_MARGIN, read != 0u);

            elastic = reread.energy;
            ++report.plasticEvaluations;

            if (reread.loading != 0u) {
              fellBack = false;

              break;
            }

            balance(&total);

            if (read == 0u) {
              scratch.increment = state.acceleration;
            } else {
              std::fill(
                scratch.increment.begin(),
                scratch.increment.end(),
                glm::vec3(0.0f)
              );
            }

            const CgResult solved = solvePrimary(scratch.increment, read == 0u);

            primaryResult.relativeResidual = solved.relativeResidual;
            spent += solved.iterations;

            ++report.plasticSolves;

            for (size_t node = 0u; node != nodes; ++node) {
              if (mesh.anchored[node] == 0u) {
                total[node] += scratch.increment[node];
              }
            }
          }

          if (fellBack) {
            for (size_t node = 0u; node != nodes; ++node) {
              evaluated[node] = scratch.weighted[node] + reach * total[node];
            }

            const Assembly reread = assemble(evaluated, LOADING_MARGIN);

            elastic = reread.energy;
            ++report.plasticEvaluations;

            balance(&total);

            double squared = 0.0;

            for (const glm::vec3& right : scratch.right) {
              squared += static_cast<double>(glm::dot(right, right));
            }

            fellBack = reread.loading == 0u && plasticReduced(squared, opening);
          }

          if (fellBack) {
            ++report.plasticFallbacks;

            converged = true;
          } else {
            ++report.plasticFallbackRejections;

            bindStiffness(scratch, store, mesh, materials, true);

            std::copy(
              lowestIterate.begin(),
              lowestIterate.end(),
              total.begin()
            );

            for (size_t node = 0u; node != nodes; ++node) {
              evaluated[node] = scratch.weighted[node] + reach * total[node];
            }

            elastic = assemble(evaluated, LOADING_MARGIN).energy;
            ++report.plasticEvaluations;

            balance(&total);

            const double remaining = std::sqrt(lowest);

            if (remaining >= static_cast<double>(report.plasticWorstResidual)) {
              report.plasticWorstResidual = static_cast<float>(remaining);
              report.plasticWorstRelativeResidual =
                opening > 0.0 ? static_cast<float>(std::sqrt(lowest / opening))
                              : 0.0f;
            }
          }
        }

        report.plasticCapped += converged ? 0u : 1u;
        primaryResult.iterations = spent;
        state.acceleration = total;

        break;
      }
    }

    report.worstSubstep =
      glm::max(report.worstSubstep, primaryResult.iterations);
    report.cgPerSubstep.push_back(primaryResult.iterations);
    report.cgStartResidual.push_back(primaryResult.startResidual);
    report.cgEndResidual.push_back(primaryResult.relativeResidual);

    if (solver != nullptr) {
      ++solver->stats.substepsSinceRefresh;
      solver->stats.meanIterations =
        0.9f * solver->stats.meanIterations
        + 0.1f * static_cast<float>(primaryResult.iterations);
    }

    if (floating) {
      glm::dvec3 stray(0.0);
      double weight = 0.0;

      for (size_t node = 0u; node != nodes; ++node) {
        weight += static_cast<double>(mesh.nodeMass[node]);
        stray += static_cast<double>(mesh.nodeMass[node])
                 * glm::dvec3(state.acceleration[node]);
      }

      const glm::vec3 mean(stray / weight);
      const glm::vec3 spin = body.endSpin;

      glm::dmat3 inertia(0.0);
      glm::dvec3 momentum(0.0);
      glm::dvec3 torque(0.0);

      for (size_t node = 0u; node != nodes; ++node) {
        const glm::vec3 turned = state.acceleration[node] - mean;
        const glm::vec3 placed =
          body.turn
          * (scratch.shifted[node] + rule.beta * inner * inner * turned);
        const glm::vec3 moved =
          body.turn
          * (relative[node] + inner * ((1.0f - rule.gamma) * scratch.carried[node] + rule.gamma * turned));
        const glm::vec3 reached = body.nextCentre + placed;
        const glm::vec3 acceleration =
          body.nextAcceleration + glm::cross(body.endRate, placed)
          + glm::cross(spin, glm::cross(spin, placed))
          + 2.0f * glm::cross(spin, moved) + body.turn * turned;

        unconstrained[node] =
          body.nextVelocity + glm::cross(spin, placed) + moved;
        advance[node] = (reached - state.position[node]) / inner;
        state.acceleration[node] = acceleration;
        scratch.shifted[node] =
          reached - rule.beta * inner * inner * acceleration;
        relative[node] = placed;

        const double mass = static_cast<double>(mesh.nodeMass[node]);
        const glm::dvec3 arm(placed);

        inertia +=
          mass
          * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
        momentum += mass
                    * glm::cross(
                      arm,
                      glm::dvec3(unconstrained[node] - body.nextVelocity)
                    );
        torque +=
          mass
          * glm::cross(arm, glm::dvec3(acceleration - body.nextAcceleration));
      }

      if (
        settings.rigidSplit == RigidSplit::Projected
        && glm::determinant(inertia) > 0.0
      ) {
        const glm::dmat3 inverse = glm::inverse(inertia);
        const glm::vec3 restore(
          inverse * (glm::dvec3(body.momentum) - momentum)
        );
        const glm::vec3 settle(inverse * (glm::dvec3(body.torque) - torque));

        for (size_t node = 0u; node != nodes; ++node) {
          const glm::vec3 turning = glm::cross(settle, relative[node]);

          unconstrained[node] += glm::cross(restore, relative[node]);
          state.acceleration[node] += turning;
          scratch.shifted[node] -= rule.beta * inner * inner * turning;
        }
      }

      for (size_t node = 0u; node != nodes; ++node) {
        state.velocity[node] = unconstrained[node];
      }

      for (talus::CommittedTet& held : state.committed) {
        held.rotation = body.turn * held.rotation;
      }

      if (damped) {
        turnDamping(scratch.damping, body.turn);
      }
    } else {
      for (size_t node = 0u; node != nodes; ++node) {
        if (mesh.anchored[node] != 0u) {
          unconstrained[node] = glm::vec3(0.0f);
          advance[node] = glm::vec3(0.0f);

          continue;
        }

        unconstrained[node] =
          state.velocity[node]
          + inner
              * ((1.0f - rule.gamma) * scratch.carried[node] + rule.gamma * state.acceleration[node]);
        advance[node] =
          state.velocity[node]
          + inner
              * ((0.5f - rule.beta) * scratch.carried[node] + rule.beta * state.acceleration[node]);
        state.velocity[node] = unconstrained[node];
      }
    }

    if (settings.reportSystem) {
      captureSystem(
        mesh,
        state,
        scratch,
        effective,
        incidence,
        *assembledAt,
        floating ? body.turn : glm::mat3(1.0f),
        report
      );
    }

    for (size_t partner = 0u; partner != partners.size(); ++partner) {
      partnerLinear[partner] = partners[partner].linearVelocity;
      partnerAngular[partner] = partners[partner].angularVelocity;
    }

    const auto contactBegan = std::chrono::steady_clock::now();

    bool corrected = false;
    bool posed = false;
    std::vector<glm::vec3> posedEnd;

    active.clear();

    for (size_t source = 0u; source != contacts.size(); ++source) {
      const SurfaceContact& held = contacts[source];

      if (
        held.partner < 0 || static_cast<size_t>(held.partner) >= partners.size()
      ) {
        continue;
      }

      const size_t index = static_cast<size_t>(held.partner);
      const RigidPartner& partner = partners[index];

      Contact contact;
      contact.nodes = held.nodes;
      contact.weights = held.weights;
      contact.source = source;
      contact.lambda = lambdas[source];
      contact.partner = index;
      contact.dynamic = partner.dynamic && partner.inverseMass > 0.0f;
      contact.normal = held.normal;
      contact.point =
        partnerCenter[index] + partnerRotation[index] * held.localPoint;
      contact.arm = contact.point - partnerCenter[index];
      contact.bias = held.bias;
      contact.friction = held.friction;

      tangentsOf(contact.normal, contact.tangent, contact.bitangent);

      glm::vec3 meshPoint(0.0f);

      for (int corner = 0; corner != 3; ++corner) {
        meshPoint += held.weights[corner] * state.position[held.nodes[corner]];
      }

      contact.gap =
        glm::dot(contact.normal, meshPoint - contact.point) - held.bias;
      contact.normalTarget = -glm::max(contact.gap, 0.0f) / inner;
      contact.rigidNormalLinear = -contact.normal;
      contact.rigidNormalAngular = -glm::cross(contact.arm, contact.normal);

      for (int corner = 0; corner != 3; ++corner) {
        contact.area += held.weights[corner];
      }

      report.maxPenetration =
        glm::max(report.maxPenetration, -(contact.gap + held.bias));

      active.push_back(contact);
    }

    if (!active.empty()) {
      corrected = true;

      const bool checking =
        settings.reportContactSystem || settings.reportContactDiagnostics;
      const bool lands = floating && prepared != nullptr;

      talus::DeformableState freeEnd;
      glm::mat3 contactFrame = frame;
      std::vector<float> slipDiagonal(lands ? active.size() : 0u, 0.0f);

      if (lands) {
        freeEnd.position.resize(nodes);

        for (size_t node = 0u; node != nodes; ++node) {
          freeEnd.position[node] =
            scratch.shifted[node]
            + rule.beta * inner * inner * state.acceleration[node];
        }

        contactFrame = bodyRotation(mesh, freeEnd, prepared->reference());
      }

      std::vector<float> slipCompliance(checking ? active.size() : 0u, 0.0f);
      std::vector<glm::vec2> solverSlip(
        checking ? active.size() : 0u,
        glm::vec2(0.0f)
      );
      float checkedScale = 0.0f;
      float checkedResidual = 0.0f;

      const auto solveContacts = [&](
                                   std::vector<Contact>& solving,
                                   float mobility,
                                   const std::vector<glm::vec3>& base,
                                   bool primary,
                                   const glm::mat3& facing,
                                   const std::vector<float>& departure
                                 ) {
        const size_t count = solving.size();
        const size_t rows = 3u * count;
        const bool opening = departure.empty();

        std::vector<uint32_t> contactNodes;

        contactNodes.reserve(3u * count);

        for (const Contact& contact : solving) {
          contactNodes.push_back(contact.nodes.x);
          contactNodes.push_back(contact.nodes.y);
          contactNodes.push_back(contact.nodes.z);
        }

        std::sort(contactNodes.begin(), contactNodes.end());
        contactNodes.erase(
          std::unique(contactNodes.begin(), contactNodes.end()),
          contactNodes.end()
        );

        ++report.contactSweeps;
        report.contactSweepNodes += static_cast<uint32_t>(contactNodes.size());

        if (primary && opening) {
          solve.contacts =
            glm::max(solve.contacts, static_cast<uint32_t>(count));
          solve.rows = glm::max(solve.rows, static_cast<uint32_t>(rows));
          report.contactRows += static_cast<uint32_t>(rows);
        }

        const auto matrixBegan = std::chrono::steady_clock::now();

        const bool unbuilt =
          settings.contactOperator == ContactOperator::Unbuilt
          && prepared != nullptr;

        if (!unbuilt) {
          matrix.assign(rows * rows, 0.0f);
          current.assign(rows, 0.0f);
        }

        free.assign(rows, 0.0f);
        lambda.assign(rows, 0.0f);

        const auto basisBegan = std::chrono::steady_clock::now();

        std::vector<std::vector<glm::vec3>> fallback;
        ResponseBasis* responseBasis = nullptr;
        std::vector<float> normalCompliance;
        std::vector<float> tangentCompliance;
        std::vector<glm::vec3> unbuiltResponse;
        std::vector<glm::vec3> unbuiltPartnerLinear;
        std::vector<glm::vec3> unbuiltPartnerAngular;

        const double matrixSeconds =
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - matrixBegan
          )
            .count();

        if (prepared != nullptr) {
          ResponseBasis& cache = solver->basis;

          responseBasis = &cache;

          if (primary && opening) {
            if (cache.generation != prepared->generation()) {
              cache.column.clear();
              cache.generation = prepared->generation();
            }

            solve.cacheColumnsBefore =
              static_cast<uint32_t>(cache.column.size());

            std::vector<uint32_t> wanted;

            for (const Contact& held : solving) {
              for (int corner = 0; corner != 3; ++corner) {
                const uint32_t node = held.nodes[corner];

                if (
                  mesh.anchored[node] == 0u
                  && cache.column.find(node) == cache.column.end()
                ) {
                  wanted.push_back(node);
                }
              }
            }

            std::sort(wanted.begin(), wanted.end());

            wanted.erase(
              std::unique(wanted.begin(), wanted.end()),
              wanted.end()
            );

            if (!wanted.empty()) {
              std::vector<std::vector<glm::vec3>> fresh(wanted.size() * 3u);

              for (size_t at = 0u; at != wanted.size(); ++at) {
                for (size_t axis = 0u; axis != 3u; ++axis) {
                  std::vector<glm::vec3> seed(nodes, glm::vec3(0.0f));

                  seed[wanted[at]][static_cast<int>(axis)] = 1.0f;
                  fresh[at * 3u + axis] = std::move(seed);
                }
              }

              applyFactorBatch(*prepared, glm::mat3(1.0f), mesh, fresh);

              for (size_t at = 0u; at != wanted.size(); ++at) {
                std::array<std::vector<glm::vec3>, 3> held;

                for (size_t axis = 0u; axis != 3u; ++axis) {
                  held[axis] = std::move(fresh[at * 3u + axis]);
                }

                cache.column.emplace(wanted[at], std::move(held));
              }
            }

            const uint32_t newBasisSolves =
              static_cast<uint32_t>(wanted.size() * 3u);

            report.basisSolves += newBasisSolves;
            solve.cacheMisses += static_cast<uint32_t>(wanted.size());
            solve.newBasisSolves += newBasisSolves;
            solve.cacheColumnsAfter =
              static_cast<uint32_t>(cache.column.size());

            const double responseCacheSeconds =
              std::chrono::duration<double>(
                std::chrono::steady_clock::now() - basisBegan
              )
                .count();

            report.responseCacheSeconds += responseCacheSeconds;
            solve.responseCacheSeconds += responseCacheSeconds;
          }

          const glm::mat3 unturn = glm::transpose(facing);

          const auto delassusBegan = std::chrono::steady_clock::now();

          if (unbuilt) {
            normalCompliance.assign(count, 0.0f);
            tangentCompliance.assign(count, 0.0f);
            unbuiltResponse.assign(nodes, glm::vec3(0.0f));
            unbuiltPartnerLinear.assign(partners.size(), glm::vec3(0.0f));
            unbuiltPartnerAngular.assign(partners.size(), glm::vec3(0.0f));

            const auto localCompliance =
              [&](const Contact& contact, size_t axis) {
                float entry = 0.0f;

                for (int sourceCorner = 0; sourceCorner != 3; ++sourceCorner) {
                  const uint32_t source = contact.nodes[sourceCorner];

                  if (mesh.anchored[source] != 0u) {
                    continue;
                  }

                  const std::array<std::vector<glm::vec3>, 3>& held =
                    cache.column.at(source);
                  const glm::vec3 turned =
                    unturn * contact.row(sourceCorner, axis);

                  for (int targetCorner = 0; targetCorner != 3;
                       ++targetCorner) {
                    const uint32_t target = contact.nodes[targetCorner];

                    if (mesh.anchored[target] != 0u) {
                      continue;
                    }

                    const glm::vec3 along =
                      unturn * contact.row(targetCorner, axis);
                    float reach = 0.0f;

                    for (size_t column = 0u; column != 3u; ++column) {
                      reach += turned[static_cast<int>(column)]
                               * glm::dot(along, held[column][target]);
                    }

                    entry += reach;
                  }
                }

                entry *= mobility;

                if (contact.dynamic) {
                  const RigidPartner& partner = partners[contact.partner];

                  entry += partner.inverseMass
                             * glm::dot(
                               contact.rigidLinear(axis),
                               contact.rigidLinear(axis)
                             )
                           + glm::dot(
                             contact.rigidAngular(axis),
                             partner.inverseInertia * contact.rigidAngular(axis)
                           );
                }

                return entry;
              };

            for (size_t i = 0u; i != count; ++i) {
              normalCompliance[i] = localCompliance(solving[i], 0u);
              tangentCompliance[i] =
                0.5f
                * (localCompliance(solving[i], 1u) + localCompliance(solving[i], 2u));
            }
          } else {
            for (size_t r = 0u; r != rows; ++r) {
              const Contact& pusher = solving[r / 3u];

              for (size_t s = 0u; s != rows; ++s) {
                const Contact& felt = solving[s / 3u];

                float entry = 0.0f;

                for (int sourceCorner = 0; sourceCorner != 3; ++sourceCorner) {
                  const uint32_t source = pusher.nodes[sourceCorner];

                  if (mesh.anchored[source] != 0u) {
                    continue;
                  }

                  const std::array<std::vector<glm::vec3>, 3>& held =
                    cache.column.at(source);
                  const glm::vec3 turned =
                    unturn * pusher.row(sourceCorner, r % 3u);

                  for (int targetCorner = 0; targetCorner != 3;
                       ++targetCorner) {
                    const uint32_t target = felt.nodes[targetCorner];

                    if (mesh.anchored[target] != 0u) {
                      continue;
                    }

                    const glm::vec3 along =
                      unturn * felt.row(targetCorner, s % 3u);
                    float reach = 0.0f;

                    for (size_t axis = 0u; axis != 3u; ++axis) {
                      reach += turned[static_cast<int>(axis)]
                               * glm::dot(along, held[axis][target]);
                    }
                    entry += reach;
                  }
                }

                entry *= mobility;

                if (
                  pusher.dynamic && felt.dynamic
                  && pusher.partner == felt.partner
                ) {
                  const RigidPartner& partner = partners[pusher.partner];

                  entry +=
                    partner.inverseMass
                      * glm::dot(
                        pusher.rigidLinear(r % 3u),
                        felt.rigidLinear(s % 3u)
                      )
                    + glm::dot(
                      felt.rigidAngular(s % 3u),
                      partner.inverseInertia * pusher.rigidAngular(r % 3u)
                    );
                }

                matrix[s * rows + r] = entry;
              }
            }
          }

          const double delassusSeconds =
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - delassusBegan
            )
              .count();

          if (primary) {
            report.delassusSeconds += matrixSeconds + delassusSeconds;
            solve.delassusSeconds += matrixSeconds + delassusSeconds;
          }
        } else {
          const auto delassusBegan = std::chrono::steady_clock::now();

          fallback.assign(rows, std::vector<glm::vec3>());

          for (size_t r = 0u; r != rows; ++r) {
            const Contact& pusher = solving[r / 3u];

            std::vector<glm::vec3> load(nodes, glm::vec3(0.0f));

            for (int corner = 0; corner != 3; ++corner) {
              const uint32_t source = pusher.nodes[corner];

              if (mesh.anchored[source] != 0u) {
                continue;
              }

              load[source] += pusher.row(corner, r % 3u);
            }

            fallback[r].assign(nodes, glm::vec3(0.0f));

            gaussSeidelSweep(
              mesh,
              state,
              scratch,
              effective.step,
              incidence,
              load,
              fallback[r]
            );
          }

          for (size_t r = 0u; r != rows; ++r) {
            const Contact& pusher = solving[r / 3u];

            for (size_t s = 0u; s != rows; ++s) {
              const Contact& felt = solving[s / 3u];

              float entry = 0.0f;

              for (int corner = 0; corner != 3; ++corner) {
                const uint32_t target = felt.nodes[corner];

                if (mesh.anchored[target] != 0u) {
                  continue;
                }

                entry +=
                  glm::dot(felt.row(corner, s % 3u), fallback[r][target]);
              }

              entry *= mobility;

              if (
                pusher.dynamic && felt.dynamic && pusher.partner == felt.partner
              ) {
                const RigidPartner& partner = partners[pusher.partner];

                entry += partner.inverseMass
                           * glm::dot(
                             pusher.rigidLinear(r % 3u),
                             felt.rigidLinear(s % 3u)
                           )
                         + glm::dot(
                           felt.rigidAngular(s % 3u),
                           partner.inverseInertia * pusher.rigidAngular(r % 3u)
                         );
              }

              matrix[s * rows + r] = entry;
            }
          }

          const double delassusSeconds =
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - delassusBegan
            )
              .count();

          if (primary) {
            report.delassusSeconds += matrixSeconds + delassusSeconds;
            solve.delassusSeconds += matrixSeconds + delassusSeconds;
          }
        }

        if (
          primary && settings.reportContactDiagnostics && solve.samples.empty()
        ) {
          solve.samples.reserve(solving.size());

          for (const Contact& contact : solving) {
            solve.samples.push_back(
              ContactSample{
                contacts[contact.source].triangle,
                partners[contact.partner].body,
                contact.nodes,
                contact.point
              }
            );
          }

          solve.uniqueNodes = static_cast<uint32_t>(contactNodes.size());
        }

        if (primary) {
          report.basisSeconds += std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - basisBegan
          )
                                   .count();
        }

        for (size_t i = 0u; i != count; ++i) {
          const Contact& contact = solving[i];

          for (size_t axis = 0u; axis != 3u; ++axis) {
            lambda[3u * i + axis] = contact.lambda[static_cast<int>(axis)];
          }

          for (size_t axis = 0u; axis != 3u; ++axis) {
            float velocity = 0.0f;

            for (int corner = 0; corner != 3; ++corner) {
              const uint32_t node = contact.nodes[corner];

              if (mesh.anchored[node] == 0u) {
                velocity += glm::dot(contact.row(corner, axis), base[node]);
              }
            }

            velocity += glm::dot(
              contact.rigidLinear(axis),
              partnerLinear[contact.partner]
            );
            velocity += glm::dot(
              contact.rigidAngular(axis),
              partnerAngular[contact.partner]
            );

            free[3u * i + axis] =
              opening ? velocity : velocity + departure[3u * i + axis];
          }
        }

        const auto unbuiltResponseAt =
          [&](const Contact& contact, size_t axis) {
            float response = 0.0f;

            for (int corner = 0; corner != 3; ++corner) {
              const uint32_t node = contact.nodes[corner];

              if (mesh.anchored[node] == 0u) {
                response +=
                  glm::dot(contact.row(corner, axis), unbuiltResponse[node]);
              }
            }

            if (contact.dynamic) {
              response += glm::dot(
                contact.rigidLinear(axis),
                unbuiltPartnerLinear[contact.partner]
              );
              response += glm::dot(
                contact.rigidAngular(axis),
                unbuiltPartnerAngular[contact.partner]
              );
            }

            return response;
          };

        const auto applyUnbuiltResponse =
          [&](const Contact& contact, const glm::vec3& change) {
            for (size_t axis = 0u; axis != 3u; ++axis) {
              const float value = change[static_cast<int>(axis)];

              if (value == 0.0f) {
                continue;
              }

              for (int corner = 0; corner != 3; ++corner) {
                const uint32_t source = contact.nodes[corner];

                if (mesh.anchored[source] != 0u) {
                  continue;
                }

                const std::array<std::vector<glm::vec3>, 3>& columns =
                  responseBasis->column.at(source);
                const glm::vec3 turned =
                  glm::transpose(facing) * (value * contact.row(corner, axis));

                for (const uint32_t node : contactNodes) {
                  unbuiltResponse[node] += mobility
                                       * (facing
                                          * (turned.x * columns[0][node]
                                             + turned.y * columns[1][node]
                                             + turned.z * columns[2][node]));
                }
              }
            }

            if (!contact.dynamic) {
              return;
            }

            const RigidPartner& partner = partners[contact.partner];

            for (size_t axis = 0u; axis != 3u; ++axis) {
              const float value = change[static_cast<int>(axis)];

              unbuiltPartnerLinear[contact.partner] +=
                value * partner.inverseMass * contact.rigidLinear(axis);
              unbuiltPartnerAngular[contact.partner] +=
                partner.inverseInertia * (value * contact.rigidAngular(axis));
            }
          };

        const auto rebuildUnbuiltResponse = [&]() {
          for (const uint32_t node : contactNodes) {
            unbuiltResponse[node] = glm::vec3(0.0f);
          }

          std::fill(
            unbuiltPartnerLinear.begin(),
            unbuiltPartnerLinear.end(),
            glm::vec3(0.0f)
          );
          std::fill(
            unbuiltPartnerAngular.begin(),
            unbuiltPartnerAngular.end(),
            glm::vec3(0.0f)
          );

          for (size_t i = 0u; i != count; ++i) {
            const size_t row = 3u * i;

            applyUnbuiltResponse(
              solving[i],
              glm::vec3(lambda[row], lambda[row + 1u], lambda[row + 2u])
            );
          }
        };

        if (unbuilt) {
          rebuildUnbuiltResponse();
        }

        if (primary && settings.reportContactSystem) {
          if (unbuilt) {
            report.contactOperator.clear();
          } else {
            report.contactOperator = matrix;
          }
          report.contactRight.assign(rows, 0.0f);
          report.contactFriction.assign(count, 0.0f);
          report.contactFrames.assign(count, ContactFrame{});

          for (size_t i = 0u; i != count; ++i) {
            const Contact& contact = solving[i];

            report.contactFriction[i] = contact.friction;
            report.contactFrames[i].nodes = contact.nodes;
            report.contactFrames[i].weights = contact.weights;
            report.contactFrames[i].normal = contact.normal;
            report.contactFrames[i].tangent = contact.tangent;
            report.contactFrames[i].bitangent = contact.bitangent;
            report.contactFrames[i].gap = contact.gap;
            report.contactFrames[i].friction = contact.friction;
            report.contactFrames[i].dynamic = contact.dynamic ? 1u : 0u;

            for (size_t axis = 0u; axis != 3u; ++axis) {
              report.contactRight[3u * i + axis] =
                contact.target(axis) - free[3u * i + axis];
            }
          }
        }

        float reference = 0.0f;
        float standing = 0.0f;

        for (size_t i = 0u; i != count; ++i) {
          const Contact& contact = solving[i];
          const size_t row = 3u * i;
          const float straight =
            unbuilt ? normalCompliance[i] : matrix[row * rows + row];
          const float shared =
            unbuilt
              ? tangentCompliance[i]
              : 0.5f
                  * (matrix[(row + 1u) * rows + row + 1u] + matrix[(row + 2u) * rows + row + 2u]);
          const float aim = contact.target(0u);

          const glm::vec3 scaled(
            straight > 0.0f ? (aim - free[row]) / straight : 0.0f,
            shared > 0.0f ? -free[row + 1u] / shared : 0.0f,
            shared > 0.0f ? -free[row + 2u] / shared : 0.0f
          );

          reference += glm::dot(scaled, scaled);

          if (primary && lands) {
            slipDiagonal[i] = shared;
          }
        }

        reference = std::sqrt(reference);

        float settled = 0.0f;

        uint32_t taken = 0u;

        std::vector<uint8_t> regime(count, 0u);
        std::vector<uint8_t> shifted(count, 255u);

        uint32_t settleAt = 0u;
        uint32_t tightAt = 0u;

        const auto measureResidual = [&]() {
          float missed = 0.0f;
          float carried = 0.0f;

          for (size_t i = 0u; i != count; ++i) {
            const Contact& contact = solving[i];
            const size_t row = 3u * i;
            const float straight =
              unbuilt ? normalCompliance[i] : matrix[row * rows + row];
            const float shared =
              unbuilt
                ? tangentCompliance[i]
                : 0.5f
                    * (matrix[(row + 1u) * rows + row + 1u] + matrix[(row + 2u) * rows + row + 2u]);
            const float aim = contact.target(0u);

            const glm::vec3 seen(
              lambda[row],
              lambda[row + 1u],
              lambda[row + 2u]
            );

            glm::vec3 wanted(
              seen.x
                + (straight > 0.0f ? (aim - free[row] - (unbuilt ? unbuiltResponseAt(contact, 0u) : current[row])) / straight : 0.0f),
              seen.y
                + (shared > 0.0f ? (-free[row + 1u] - (unbuilt ? unbuiltResponseAt(contact, 1u) : current[row + 1u])) / shared : 0.0f),
              seen.z
                + (shared > 0.0f ? (-free[row + 2u] - (unbuilt ? unbuiltResponseAt(contact, 2u) : current[row + 2u])) / shared : 0.0f)
            );

            wanted.x = glm::max(0.0f, wanted.x);

            const float reach = contact.friction * wanted.x;
            const float across = glm::length(glm::vec2(wanted.y, wanted.z));

            if (across > reach) {
              const float shrink = across > 0.0f ? reach / across : 0.0f;

              wanted.y *= shrink;
              wanted.z *= shrink;
            }

            missed += glm::dot(wanted - seen, wanted - seen);
            carried += glm::dot(seen, seen);

            if (checking && primary) {
              slipCompliance[i] = shared;
              solverSlip[i] = glm::vec2(
                free[row + 1u]
                  + (unbuilt ? unbuiltResponseAt(contact, 1u) : current[row + 1u]),
                free[row + 2u]
                  + (unbuilt ? unbuiltResponseAt(contact, 2u) : current[row + 2u])
              );
            }
          }

          return glm::vec2(std::sqrt(missed), std::sqrt(carried));
        };

        float finalScale = reference;
        float finalResidual = 0.0f;
        bool met = false;

        std::vector<float> passStart(rows, 0.0f);
        std::vector<float> gradient(rows, 0.0f);
        std::vector<float> direction(rows, 0.0f);
        std::vector<float> projected(rows, 0.0f);
        double previousGradient = 0.0;
        double watchdog = 0.0;
        bool restarted = false;

        const auto sweepBegan = std::chrono::steady_clock::now();

        for (uint32_t outer = 0u; outer != OUTER_PASSES; ++outer) {
          ++taken;

          std::copy(lambda.begin(), lambda.end(), passStart.begin());

          if (!unbuilt) {
            std::fill(current.begin(), current.end(), 0.0f);

            for (size_t r = 0u; r != rows; ++r) {
              for (size_t c = 0u; c != rows; ++c) {
                current[r] += matrix[r * rows + c] * lambda[c];
              }
            }
          }

          float moved = 0.0f;
          float reached = 0.0f;

          for (size_t i = 0u; i != count; ++i) {
            const Contact& contact = solving[i];
            const size_t row = 3u * i;

            glm::vec3 next(lambda[row], lambda[row + 1u], lambda[row + 2u]);

            const float shared =
              unbuilt
                ? tangentCompliance[i]
                : 0.5f
                    * (matrix[(row + 1u) * rows + row + 1u] + matrix[(row + 2u) * rows + row + 2u]);

            for (size_t axis = 0u; axis != 3u; ++axis) {
              const float diagonal =
                axis == 0u
                  ? (unbuilt ? normalCompliance[i] : matrix[row * rows + row])
                  : shared;

              if (!(diagonal > 0.0f)) {
                continue;
              }

              const float target = contact.target(axis);
              const float violation =
                target - free[row + axis]
                - (unbuilt ? unbuiltResponseAt(contact, axis) : current[row + axis]);

              next[static_cast<int>(axis)] += RELAXATION * violation / diagonal;
            }

            next.x = glm::max(0.0f, next.x);

            const float cap = contact.friction * next.x;
            const float slide = glm::length(glm::vec2(next.y, next.z));

            if (slide > cap) {
              const float scale = slide > 0.0f ? cap / slide : 0.0f;

              next.y *= scale;
              next.z *= scale;
            }

            const glm::vec3 change =
              next - glm::vec3(lambda[row], lambda[row + 1u], lambda[row + 2u]);

            for (size_t axis = 0u; axis != 3u; ++axis) {
              const float value = change[static_cast<int>(axis)];

              moved += value * value;
              reached +=
                next[static_cast<int>(axis)] * next[static_cast<int>(axis)];

              if (value == 0.0f) {
                continue;
              }

              lambda[row + axis] = next[static_cast<int>(axis)];

              if (!unbuilt) {
                for (size_t r = 0u; r != rows; ++r) {
                  current[r] += matrix[r * rows + row + axis] * value;
                }
              }
            }

            if (unbuilt) {
              applyUnbuiltResponse(contact, change);
            }
          }

          settled = reached > 0.0f ? std::sqrt(moved / reached) : 0.0f;

          glm::vec2 measured = measureResidual();

          standing =
            measured.x / glm::max(glm::max(measured.y, reference), 1.0e-30f);

          for (size_t i = 0u; i != count; ++i) {
            const size_t row = 3u * i;
            const float normal = lambda[row];
            const float across =
              glm::length(glm::vec2(lambda[row + 1u], lambda[row + 2u]));
            const float cap = solving[i].friction * normal;

            regime[i] = normal <= 0.0f
                          ? 0u
                          : (across >= cap * (1.0f - 1.0e-4f) ? 2u : 1u);
          }

          if (regime != shifted) {
            shifted = regime;
            settleAt = taken;
          }

          finalScale = glm::max(measured.y, reference);
          finalResidual = measured.x;

          if (unbuilt && measured.x <= settings.contactTolerance * finalScale) {
            rebuildUnbuiltResponse();

            measured = measureResidual();
            finalScale = glm::max(measured.y, reference);
            finalResidual = measured.x;
            standing = measured.x / glm::max(finalScale, 1.0e-30f);
          }

          if (tightAt == 0u && standing <= 1.0e-5f) {
            tightAt = taken;
          }

          if (measured.x <= settings.contactTolerance * finalScale) {
            met = true;

            break;
          }

          double gradientNorm = 0.0;
          double turn = 0.0;

          for (size_t r = 0u; r != rows; ++r) {
            gradient[r] = passStart[r] - lambda[r];
            gradientNorm += static_cast<double>(gradient[r])
                            * static_cast<double>(gradient[r]);
            turn +=
              static_cast<double>(gradient[r])
              * (static_cast<double>(lambda[r]) - static_cast<double>(projected[r]));
            projected[r] = lambda[r];
          }

          if (outer == 0u || restarted) {
            watchdog = gradientNorm;
            restarted = false;
          }

          if (outer == 0u) {
            for (size_t r = 0u; r != rows; ++r) {
              direction[r] = -gradient[r];
            }
          } else if (
            !(previousGradient > 0.0) || turn > 0.0 || gradientNorm > watchdog
          ) {
            restarted = true;
            std::fill(direction.begin(), direction.end(), 0.0f);
          } else {
            const float beta = static_cast<float>(
              std::min(gradientNorm / previousGradient, 1.0)
            );

            if (outer + 1u != OUTER_PASSES) {
              for (size_t i = 0u; i != count; ++i) {
                const size_t row = 3u * i;
                const glm::vec3 change(
                  beta * direction[row],
                  beta * direction[row + 1u],
                  beta * direction[row + 2u]
                );

                for (size_t axis = 0u; axis != 3u; ++axis) {
                  lambda[row + axis] += change[static_cast<int>(axis)];
                }

                if (unbuilt) {
                  applyUnbuiltResponse(solving[i], change);
                }
              }
            }

            for (size_t r = 0u; r != rows; ++r) {
              direction[r] = beta * direction[r] - gradient[r];
            }
          }

          previousGradient = gradientNorm;
        }

        if (unbuilt && !met) {
          rebuildUnbuiltResponse();

          const glm::vec2 measured = measureResidual();

          finalScale = glm::max(measured.y, reference);
          finalResidual = measured.x;
          standing = measured.x / glm::max(finalScale, 1.0e-30f);
        }

        const double pgsSeconds =
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - sweepBegan
          )
            .count();

        if (primary) {
          report.contactSweepSeconds += pgsSeconds;
          solve.pgsSeconds += pgsSeconds;

          report.contactWorstRelativeChange =
            glm::max(report.contactWorstRelativeChange, settled);
          report.contactPasses = glm::max(report.contactPasses, taken);
          report.contactTotalPasses += taken;
          solve.pgsPasses += taken;
          report.contactSettlePass =
            glm::max(report.contactSettlePass, settleAt);
          report.contactTightPass = glm::max(report.contactTightPass, tightAt);
          report.contactResidual = glm::max(report.contactResidual, standing);

          checkedScale = finalScale;
          checkedResidual = finalResidual;

          report.contactWarmStart.clear();

          for (const Contact& held : solving) {
            report.contactWarmStart.push_back(held.lambda);
          }
        } else {
          report.velocityContactSweepSeconds += pgsSeconds;
          report.velocityContactPasses =
            glm::max(report.velocityContactPasses, taken);
          report.velocityContactTotalPasses += taken;
          report.velocityContactResidual =
            glm::max(report.velocityContactResidual, standing);

          if (!met) {
            ++report.velocityContactCapped;
          }
        }

        for (size_t i = 0u; i != count; ++i) {
          Contact& contact = solving[i];

          contact.lambda =
            glm::vec3(lambda[3u * i], lambda[3u * i + 1u], lambda[3u * i + 2u]);

          if (primary) {
            lambdas[contact.source] = contact.lambda;
          }
        }
      };

      const auto respond = [&](
                             const std::vector<Contact>& solved,
                             std::vector<glm::vec3>& response,
                             bool primary,
                             const glm::mat3& facing
                           ) {
        std::fill(pushed.begin(), pushed.end(), glm::vec3(0.0f));

        for (const Contact& contact : solved) {
          for (int corner = 0; corner != 3; ++corner) {
            const uint32_t node = contact.nodes[corner];

            if (mesh.anchored[node] == 0u) {
              for (size_t axis = 0u; axis != 3u; ++axis) {
                pushed[node] += contact.lambda[static_cast<int>(axis)]
                                * contact.row(corner, axis);
              }
            }
          }
        }

        response = pushed;

        const auto responseApplyBegan = std::chrono::steady_clock::now();

        if (prepared != nullptr) {
          applyFactor(*prepared, facing, mesh, response);
        } else {
          std::fill(response.begin(), response.end(), glm::vec3(0.0f));

          gaussSeidelSweep(
            mesh,
            state,
            scratch,
            effective.step,
            incidence,
            pushed,
            response
          );
        }

        const double responseApplySeconds =
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - responseApplyBegan
          )
            .count();

        if (primary) {
          report.responseApplySeconds += responseApplySeconds;
          solve.responseApplySeconds += responseApplySeconds;
        } else {
          report.velocityResponseApplySeconds += responseApplySeconds;
        }
      };

      std::vector<float> arcOffset;
      RigidPart part;
      uint32_t arcSolves = 0u;
      bool arcMet = true;

      for (;;) {
        solveContacts(
          active,
          effective.mobility,
          advance,
          true,
          contactFrame,
          arcOffset
        );
        respond(active, correction, true, contactFrame);

        if (!lands) {
          break;
        }

        part = rigidPart(mesh, freeEnd.position, correction);

        if (!part.posed) {
          break;
        }

        ++arcSolves;

        const std::vector<glm::dvec3> arcNodes = arcDeparture(
          mesh,
          rule,
          inner,
          part,
          freeEnd.position,
          correction,
          scratch,
          state
        );

        std::vector<float> arcNext(3u * active.size(), 0.0f);
        double arcChange = 0.0;

        for (size_t i = 0u; i != active.size(); ++i) {
          const Contact& contact = active[i];

          for (size_t axis = 1u; axis != 3u; ++axis) {
            const size_t row = 3u * i + axis;

            double arcValue = 0.0;

            for (int corner = 0; corner != 3; ++corner) {
              const uint32_t node = contact.nodes[corner];

              if (mesh.anchored[node] == 0u) {
                arcValue += glm::dot(
                  glm::dvec3(contact.row(corner, axis)),
                  arcNodes[node]
                );
              }
            }

            arcNext[row] = static_cast<float>(arcValue);

            const double arcDiagonal = static_cast<double>(slipDiagonal[i]);

            if (!(arcDiagonal > 0.0)) {
              continue;
            }

            const double arcApplied =
              arcOffset.empty() ? 0.0 : static_cast<double>(arcOffset[row]);
            const double arcMoved = (arcValue - arcApplied) / arcDiagonal;

            arcChange += arcMoved * arcMoved;
          }
        }

        arcMet =
          std::sqrt(arcChange) <= static_cast<double>(settings.contactTolerance)
                                    * static_cast<double>(checkedScale);

        if (arcMet || arcSolves == ARC_SOLVES) {
          break;
        }

        arcOffset = std::move(arcNext);
      }

      report.contactImpulses.clear();

      for (const Contact& held : active) {
        report.contactImpulses.push_back(held.lambda);
      }

      if (settings.reportContactSystem) {
        std::vector<glm::vec3> reproduced(nodes, glm::vec3(0.0f));

        applySystem(
          mesh,
          state,
          scratch,
          effective.inertia,
          effective.step,
          correction,
          reproduced
        );

        float apart = 0.0f;
        float size = 0.0f;

        for (size_t node = 0u; node != nodes; ++node) {
          apart = glm::max(apart, glm::length(reproduced[node] - pushed[node]));
          size = glm::max(size, glm::length(pushed[node]));
        }

        report.contactResponseResidual = glm::max(
          report.contactResponseResidual,
          size > 0.0f ? apart / size : 0.0f
        );
      }

      pressing.clear();

      for (const Contact& contact : active) {
        if (contact.lambda.x > 0.0f) {
          Contact held = contact;

          held.normalTarget = 0.0f;
          held.lambda =
            contact.lambda * (effective.mobility / effective.inertia);
          pressing.push_back(held);
        }
      }

      if (delivered.empty()) {
        delivered.assign(nodes, glm::vec3(0.0f));
      }

      std::fill(
        contactVelocity.begin(),
        contactVelocity.end(),
        glm::vec3(0.0f)
      );

      glm::mat3 velocityFrame = frame;

      if (lands && part.posed) {
        posedEnd.resize(nodes);

        double reposedKinetic = 0.0;

        posed = poseRigid(
          mesh,
          rule,
          inner,
          part,
          freeEnd.position,
          correction,
          damped,
          scratch,
          state,
          unconstrained,
          posedEnd,
          reposedKinetic
        );

        if (posed) {
          talus::DeformableState posedAt;

          posedAt.position = posedEnd;
          velocityFrame = bodyRotation(mesh, posedAt, prepared->reference());

          ++report.posedSubsteps;
          report.posedSolves += arcSolves;
          report.posedCapped += arcMet ? 0u : 1u;
          report.posedKineticChange += reposedKinetic;
        }
      }

      if (!pressing.empty()) {
        solveContacts(
          pressing,
          effective.inertia,
          unconstrained,
          false,
          velocityFrame,
          {}
        );
        respond(pressing, contactVelocity, false, velocityFrame);

        for (size_t node = 0u; node != nodes; ++node) {
          delivered[node] += pushed[node];
        }
      }

      for (size_t node = 0u; node != nodes; ++node) {
        if (mesh.anchored[node] != 0u) {
          state.velocity[node] = glm::vec3(0.0f);

          continue;
        }

        state.velocity[node] =
          unconstrained[node] + effective.inertia * contactVelocity[node];
      }

      for (const Contact& contact : pressing) {
        if (!contact.dynamic) {
          continue;
        }

        const RigidPartner& partner = partners[contact.partner];

        for (size_t axis = 0u; axis != 3u; ++axis) {
          const float held = contact.lambda[static_cast<int>(axis)];

          partnerLinear[contact.partner] +=
            held * partner.inverseMass * contact.rigidLinear(axis);
          partnerAngular[contact.partner] +=
            partner.inverseInertia * (held * contact.rigidAngular(axis));
        }
      }

      if (checking) {
        report.contactChecks.clear();

        for (size_t i = 0u; i != active.size(); ++i) {
          const Contact& contact = active[i];

          glm::vec3 meshAdvance(0.0f);
          glm::vec3 meshVelocity(0.0f);
          glm::vec3 freeAdvance(0.0f);
          glm::vec3 freeVelocity(0.0f);

          for (int corner = 0; corner != 3; ++corner) {
            const uint32_t node = contact.nodes[corner];

            if (mesh.anchored[node] != 0u) {
              continue;
            }

            const float weight = contact.weights[corner] / contact.area;
            const glm::vec3 travelled =
              posed ? (posedEnd[node] - state.position[node]) / inner
                    : advance[node] + rule.beta * correction[node];

            meshAdvance += weight * travelled;
            meshVelocity += weight * state.velocity[node];
            freeAdvance += weight * advance[node];
            freeVelocity += weight * unconstrained[node];
          }

          const glm::vec3 moving =
            partnerLinear[contact.partner]
            + glm::cross(partnerAngular[contact.partner], contact.arm);
          const glm::vec3 before = partners[contact.partner].linearVelocity
                                   + glm::cross(
                                     partners[contact.partner].angularVelocity,
                                     contact.arm
                                   );

          const glm::vec3 slipAdvance = meshAdvance - moving;
          const glm::vec3 slipVelocity = meshVelocity - moving;

          const glm::vec2 pull(contact.lambda.y, contact.lambda.z);
          const glm::vec2 alongAdvance(
            glm::dot(contact.tangent, slipAdvance),
            glm::dot(contact.bitangent, slipAdvance)
          );
          const glm::vec2 alongVelocity(
            glm::dot(contact.tangent, slipVelocity),
            glm::dot(contact.bitangent, slipVelocity)
          );

          ContactCheck held;

          held.normalImpulse = contact.lambda.x;
          held.tangentImpulse = glm::length(pull);
          held.friction = contact.friction;
          held.gap = contact.gap;
          held.advanceNormal = glm::dot(contact.normal, slipAdvance);
          held.advanceTangent = glm::length(alongAdvance);
          held.freeAdvanceTangent = glm::length(
            glm::vec2(
              glm::dot(contact.tangent, freeAdvance - before),
              glm::dot(contact.bitangent, freeAdvance - before)
            )
          );
          held.freeVelocityTangent = glm::length(
            glm::vec2(
              glm::dot(contact.tangent, freeVelocity - before),
              glm::dot(contact.bitangent, freeVelocity - before)
            )
          );
          held.velocityNormal = glm::dot(contact.normal, slipVelocity);
          held.velocityTangent = glm::length(alongVelocity);
          held.dynamic = contact.dynamic ? 1u : 0u;
          held.frictionWork = glm::dot(pull, alongAdvance);
          held.area = contact.area;
          held.tangentCompliance = slipCompliance[i];
          held.tangentAdvance = alongAdvance;
          held.solverTangentSlip = solverSlip[i];
          held.residualScale = checkedScale;
          held.residualNorm = checkedResidual;

          if (held.tangentImpulse > 0.0f && held.advanceTangent > 0.0f) {
            held.advanceOpposition = glm::dot(
              pull / held.tangentImpulse,
              -alongAdvance / held.advanceTangent
            );
          }

          if (held.tangentImpulse > 0.0f && held.velocityTangent > 0.0f) {
            held.velocityOpposition = glm::dot(
              pull / held.tangentImpulse,
              -alongVelocity / held.velocityTangent
            );
          }

          report.contactChecks.push_back(held);
        }

        solve.checks = report.contactChecks;
      }

      for (size_t partner = 0u; partner != partners.size(); ++partner) {
        RigidPartner& held = partners[partner];

        if (!held.dynamic || held.inverseMass <= 0.0f) {
          continue;
        }

        const glm::vec3 impulse =
          (partnerLinear[partner] - held.linearVelocity) / held.inverseMass;

        held.impulse += impulse;
        held.nodeImpulse -= impulse;
        held.linearVelocity = partnerLinear[partner];
        held.angularVelocity = partnerAngular[partner];
      }
    }

    const double contactSeconds =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - contactBegan
      )
        .count();

    report.contactSeconds += contactSeconds;
    solve.totalContactSeconds += contactSeconds;

    for (size_t node = 0u; node != nodes; ++node) {
      if (mesh.anchored[node] != 0u) {
        state.velocity[node] = glm::vec3(0.0f);
        state.acceleration[node] = glm::vec3(0.0f);

        continue;
      }

      if (posed) {
        state.position[node] = posedEnd[node];
      } else if (corrected) {
        state.position[node] =
          scratch.shifted[node]
          + rule.beta * inner * inner
              * (state.acceleration[node] + correction[node] / inner);
      } else {
        state.position[node] =
          scratch.shifted[node]
          + rule.beta * inner * inner * state.acceleration[node];
      }
    }

    for (size_t partner = 0u; partner != partners.size(); ++partner) {
      partnerCenter[partner] += partners[partner].linearVelocity * inner;
      partnerRotation[partner] = spun(
        partnerRotation[partner],
        partners[partner].angularVelocity,
        inner
      );
    }

    report.plasticGrowth =
      commitPlastic(mesh, state, materials, rotation) || report.plasticGrowth;
  }

  report.contactLoad.clear();

  if (deltaTime > 0.0f) {
    report.contactLoad.reserve(delivered.size());

    for (const glm::vec3& impulse : delivered) {
      report.contactLoad.push_back(impulse / deltaTime);
    }
  }

  float kinetic = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    kinetic += 0.5f * mesh.nodeMass[node]
               * glm::dot(state.velocity[node], state.velocity[node]);
  }

  report.kineticEnergy += kinetic;
  report.elasticEnergy += elastic;

  if (settings.reportContactDiagnostics) {
    report.contactSolves.push_back(std::move(solve));
  }

  for (const RigidPartner& partner : partners) {
    if (partner.impulse == glm::vec3(0.0f)) {
      continue;
    }

    report.exchange.push_back(
      RigidExchange{partner.body, partner.impulse, partner.nodeImpulse}
    );
  }

  report.cpuSeconds +=
    std::chrono::duration<double>(std::chrono::steady_clock::now() - began)
      .count();
}

}
