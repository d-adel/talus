#include "strainstep.h"

#include <talus/dynamics/strainstep/contactrows.h>
#include <talus/dynamics/strainstep/element.h>
#include <talus/dynamics/strainstep/elementcache.h>
#include <talus/dynamics/strainstep/inversepool.h>
#include <talus/dynamics/strainstep/lowestmode.h>
#include <talus/dynamics/strainstep/plasticflow.h>
#include <talus/dynamics/strainstep/rigidrows.h>
#include <talus/dynamics/strainstep/rigidstate.h>
#include <talus/dynamics/strainstep/tearing.h>

#include <Eigen/Dense>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace talus::dynamics {

struct StrainLattice {
  std::shared_ptr<const talus::PreparedMesh> mesh;
  double h = 0.0;
  bool exact = false;
  double lowest = 0.0;
  std::vector<glm::dmat3> plastic;
  std::vector<ElementRest> rests;
  std::vector<StrainElement> elements;
};

struct StrainstepArena {
  std::unordered_map<uint32_t, StrainLattice> lattices;
  InversePool inverses;
  ElementCache cache;
  uint32_t builds = 0u;
  uint32_t reuses = 0u;
  uint32_t parallelBatches = 0u;
};

namespace {

constexpr double MAX_SPEED = 400.0;
constexpr double MIDPOINT = 0.5;
constexpr uint32_t PASSES = 4u;

bool pinned(const LatticeBody& lattice, size_t node) {
  return !(lattice.mesh->nodeMass[node] > 0.0f)
         || (!lattice.mesh->anchored.empty() && lattice.mesh->anchored[node] != 0u);
}

void integrateNodes(LatticeBody& lattice, const glm::dvec3& gravity, double h) {
  for (size_t node = 0u; node != lattice.velocity.size(); ++node) {
    lattice.velocity[node] =
      pinned(lattice, node) ? glm::dvec3(0.0) : lattice.velocity[node] + h * gravity;
  }
}

void advanceNodes(LatticeBody& lattice, double h) {
  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    glm::dvec3& velocity = lattice.velocity[node];
    const double speed = glm::length(velocity);

    if (speed > MAX_SPEED) {
      velocity *= MAX_SPEED / speed;
    }

    lattice.position[node] += h * velocity;
  }
}

const talus::PhysicalMaterial& materialOf(const LatticeBody& lattice, uint32_t cell) {
  if (lattice.materials.empty()) {
    throw std::runtime_error("A strainstep lattice needs its material lanes");
  }

  const size_t lane = lattice.mesh->hexes[cell].material;

  return lattice.materials[std::min(lane, lattice.materials.size() - 1u)];
}

double stiffestFrequency(const LatticeBody& lattice) {
  double frequency = 0.0;

  for (uint32_t cell = 0u; cell != lattice.mesh->hexes.size(); ++cell) {
    frequency = std::max(frequency, cellFrequency(materialOf(lattice, cell), lattice.cell));
  }

  return frequency;
}

bool tooStiff(const LatticeBody& lattice, double h) {
  return h * stiffestFrequency(lattice) > RESOLVED;
}

uint32_t neededSubsteps(const LatticeBody& lattice, double dt) {
  const double frequency = stiffestFrequency(lattice);

  if (!(frequency > 0.0) || !std::isfinite(frequency)) {
    return 1u;
  }

  uint32_t needed = static_cast<uint32_t>(
    std::clamp(std::ceil(dt * frequency / RESOLVED), 1.0, static_cast<double>(MOST_SUBSTEPS))
  );

  while (needed < MOST_SUBSTEPS && dt / static_cast<double>(needed) * frequency > RESOLVED) {
    ++needed;
  }

  return needed;
}

ElementRest restOf(const LatticeBody& lattice, uint32_t cell, std::span<const ElementRest> kept) {
  const ElementRest rest = plasticRest(lattice, cell);

  if (!fitHolds(rest) && cell < kept.size() && kept[cell].volume > 0.0) {
    return kept[cell];
  }

  return rest;
}

std::vector<double> inverseMasses(const LatticeBody& lattice) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::vector<double> inverse(mesh.nodeMass.size(), 0.0);

  for (size_t node = 0u; node != inverse.size(); ++node) {
    if (!pinned(lattice, node) && mesh.nodeMass[node] > 0.0f) {
      inverse[node] = 1.0 / static_cast<double>(mesh.nodeMass[node]);
    }
  }

  return inverse;
}

ElementDamping cellDamping(const LatticeBody& lattice, uint32_t cell, double h, const StrainLattice& built) {
  const double target = materialOf(lattice, cell).dampingRatio;
  ElementDamping damping{target, std::isfinite(built.lowest) ? built.lowest : 0.0, 1.0, 0.0};

  if (built.exact && built.lowest > 0.0 && std::isfinite(built.lowest) && std::isfinite(target)) {
    damping.theta = std::clamp(2.0 * target / (h * built.lowest), MIDPOINT, 1.0);
    damping.own = damping.theta * h;
  }

  return damping;
}

template <typename DampingOf>
std::vector<StrainElement> elementsOf(
  const LatticeBody& lattice,
  double h,
  std::span<const double> inverse,
  const glm::dquat& turn,
  std::span<const ElementRest> rests,
  const DampingOf& dampingOf,
  InversePool* pool
) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::vector<StrainElement> elements;

  elements.reserve(mesh.hexes.size());

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    std::array<double, 8> weights{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      weights[corner] = inverse[hex.nodes[corner]];
    }

    std::optional<StrainElement> element = makeElement(
      cell,
      hex.nodes,
      weights,
      rests[cell],
      elementMaterial(materialOf(lattice, cell)),
      h,
      dampingOf(cell)
    );

    if (element) {
      element->rotation = turn;

      if (pool != nullptr) {
        element->inverse = pool->intern(std::move(element->inverse));
      }

      elements.push_back(std::move(*element));
    }
  }

  return elements;
}

template <typename Value>
void put(std::string& key, const Value& value) {
  key.append(reinterpret_cast<const char*>(&value), sizeof(Value));
}

std::string buildKey(const LatticeBody& lattice, double h) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::string key;

  put(key, h);
  put(key, lattice.cell);
  put(key, mesh.restPositions.size());
  put(key, mesh.hexes.size());

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    put(key, mesh.restPositions[node].x);
    put(key, mesh.restPositions[node].y);
    put(key, mesh.restPositions[node].z);
    put(key, mesh.nodeMass[node]);
    put(key, mesh.anchored.empty() ? uint8_t{0u} : mesh.anchored[node]);
  }

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    const talus::PhysicalMaterial& material = materialOf(lattice, cell);

    put(key, hex.nodes);
    put(key, hex.volume);
    put(key, material.youngsModulus);
    put(key, material.poissonsRatio);
    put(key, material.density);
    put(key, material.dampingRatio);
  }

  return key;
}

StrainLattice reusedElements(const LatticeBody& lattice, double h, const BuiltElements& found) {
  StrainLattice built;
  const glm::dquat turn = latticeTurn(lattice);

  built.mesh = lattice.mesh;
  built.h = h;
  built.exact = found.exact;
  built.lowest = found.lowest;
  built.rests = found.rests;
  built.elements = found.elements;

  for (StrainElement& element : built.elements) {
    element.rotation = turn;
  }

  return built;
}

StrainLattice buildElements(
  const LatticeBody& lattice,
  double h,
  const StrainLattice& previous,
  StrainstepArena& arena
) {
  const bool fresh = previous.mesh == nullptr && lattice.plastic.empty() && !lattice.materials.empty();
  std::string key;

  if (fresh) {
    key = buildKey(lattice, h);

    if (const BuiltElements* found = arena.cache.find(key); found != nullptr) {
      ++arena.reuses;

      return reusedElements(lattice, h, *found);
    }
  }

  ++arena.builds;

  const talus::PreparedMesh& mesh = *lattice.mesh;
  const std::vector<double> inverse = inverseMasses(lattice);
  const glm::dquat turn = latticeTurn(lattice);
  std::span<const ElementRest> kept;
  std::vector<ElementRest> rests;
  bool damped = false;

  if (previous.rests.size() == mesh.hexes.size()) {
    kept = previous.rests;
  }

  rests.reserve(mesh.hexes.size());

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    rests.push_back(restOf(lattice, cell, kept));
    damped = damped || std::isfinite(materialOf(lattice, cell).dampingRatio);
  }

  StrainLattice built;

  built.mesh = lattice.mesh;
  built.h = h;

  if (damped && previous.lowest > 0.0 && std::isfinite(previous.lowest)) {
    built.lowest = previous.lowest;
  } else if (damped) {
    const std::vector<StrainElement> plain = elementsOf(
      lattice,
      h,
      inverse,
      turn,
      rests,
      [](uint32_t) { return ElementDamping{}; },
      nullptr
    );
    const std::vector<double> mass(mesh.nodeMass.begin(), mesh.nodeMass.end());
    std::vector<uint8_t> held(mesh.restPositions.size(), 0u);
    std::vector<glm::dvec3> rest(mesh.restPositions.size());

    for (size_t node = 0u; node != rest.size(); ++node) {
      held[node] = pinned(lattice, node) ? 1u : 0u;
      rest[node] = glm::dvec3(mesh.restPositions[node]);
    }

    built.lowest = lowestAngular(plain, mass, held, rest);
  }

  built.exact = !tooStiff(lattice, h);
  built.elements = elementsOf(
    lattice,
    h,
    inverse,
    turn,
    rests,
    [&](uint32_t cell) { return cellDamping(lattice, cell, h, built); },
    &arena.inverses
  );
  built.plastic = lattice.plastic;
  built.rests = std::move(rests);

  for (ElementRest& rest : built.rests) {
    if (!fitHolds(rest)) {
      rest = ElementRest{};
    }
  }

  if (fresh) {
    arena.cache.store(std::move(key), BuiltElements{built.lowest, built.exact, built.rests, built.elements});
  }

  return built;
}

double remaining(std::span<const double> before, std::span<const double> after) {
  double was = 0.0;
  double now = 0.0;

  for (size_t row = 0u; row != before.size(); ++row) {
    was += before[row] * before[row];
    now += after[row] * after[row];
  }

  return was > 0.0 ? std::min(1.0, std::sqrt(now / was)) : 1.0;
}

void refreshFlowed(LatticeBody& lattice, StrainLattice& cached, double h, InversePool& pool) {
  if (lattice.plastic.size() != 8u * lattice.mesh->hexes.size()) {
    return;
  }

  if (cached.plastic.size() != lattice.plastic.size()) {
    cached.plastic.assign(lattice.plastic.size(), glm::dmat3(1.0));
  }

  const talus::PreparedMesh& mesh = *lattice.mesh;
  const std::vector<double> inverse = inverseMasses(lattice);

  for (StrainElement& old : cached.elements) {
    const uint32_t cell = old.cell;
    bool moved = false;

    for (uint32_t point = 0u; point != 8u; ++point) {
      moved = moved || lattice.plastic[8u * cell + point] != cached.plastic[8u * cell + point];
    }

    if (!moved) {
      continue;
    }

    const talus::PreparedHex& hex = mesh.hexes[cell];
    std::array<double, 8> weights{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      weights[corner] = inverse[hex.nodes[corner]];
    }

    const ElementRest rest = plasticRest(lattice, cell);

    if (!fitHolds(rest)) {
      continue;
    }

    std::optional<StrainElement> fresh = makeElement(
      cell,
      hex.nodes,
      weights,
      rest,
      elementMaterial(materialOf(lattice, cell)),
      h,
      cellDamping(lattice, cell, h, cached)
    );

    if (!fresh) {
      continue;
    }

    fresh->inverse = pool.intern(std::move(fresh->inverse));
    fresh->rotation = old.rotation;
    fresh->impulse = old.impulse;
    fresh->volumeImpulse = old.volumeImpulse;
    updateElement(old, lattice.position);
    updateElement(*fresh, lattice.position);

    const double strain = remaining(
      std::span<const double>(old.strain.data(), 6u),
      std::span<const double>(fresh->strain.data(), 6u)
    );
    const double bend = remaining(
      std::span<const double>(old.strain.data() + 6u, ELEMENT_ROWS - 6u),
      std::span<const double>(fresh->strain.data() + 6u, ELEMENT_ROWS - 6u)
    );

    for (uint32_t row = 0u; row != ELEMENT_ROWS; ++row) {
      fresh->impulse[row] *= row < 6u ? strain : bend;
    }

    writeFitted(lattice, cell, rest);

    for (uint32_t point = 0u; point != 8u; ++point) {
      cached.plastic[8u * cell + point] = lattice.plastic[8u * cell + point];
    }

    cached.rests[cell] = rest;
    old = *fresh;
  }
}

std::vector<uint32_t> islandSlots(
  const BodySet& bodies,
  const ManifoldCache& cache,
  const Island& island,
  std::span<const uint32_t> stateOf
) {
  std::vector<uint32_t> slots;
  const auto inIsland = [&](const BodyRef& end) {
    return end.side == Side::Static
           || (end.side == Side::Rigid
               && (stateOf[end.index] != 0u || !(bodies.rigids[end.index].mass > 0.0)));
  };

  for (const BodyRef& ref : island.bodies) {
    if (ref.side != Side::Rigid) {
      continue;
    }

    for (const uint32_t slot : cache.pairsOf(bodies.rigids[ref.index].id)) {
      const CachedBodyPair& pair = cache.at(slot);

      if (pair.live && !pair.points.empty() && inIsland(pair.first) && inIsland(pair.second)) {
        slots.push_back(slot);
      }
    }
  }

  std::sort(slots.begin(), slots.end());
  slots.erase(std::unique(slots.begin(), slots.end()), slots.end());

  return slots;
}

}

std::shared_ptr<StrainstepArena> makeStrainstepArena() {
  return std::make_shared<StrainstepArena>();
}

StrainstepStats strainstepStats(const StrainstepArena& arena) {
  return {arena.inverses.size(), arena.builds, arena.reuses, arena.cache.cells(), arena.parallelBatches};
}

void forgetLattices(StrainstepArena& arena, const BodySet& bodies) {
  std::erase_if(arena.lattices, [&](const auto& entry) {
    return std::none_of(bodies.lattices.begin(), bodies.lattices.end(), [&](const LatticeBody& lattice) {
      return lattice.id == entry.first;
    });
  });
}

uint32_t islandSubsteps(
  const BodySet& bodies,
  const Island& island,
  const IntegrationParameters& parameters
) {
  uint32_t substeps = std::max(parameters.softstepSubsteps, 1u);

  for (const BodyRef& ref : island.bodies) {
    if (ref.side == Side::Lattice) {
      const LatticeBody& lattice = bodies.lattices[ref.index];

      substeps = std::max({substeps, lattice.substeps, neededSubsteps(lattice, parameters.dt)});
    }
  }

  return substeps;
}

uint32_t solveStrainstep(
  BodySet& bodies,
  ManifoldCache& cache,
  std::span<Contact> contacts,
  const Island& island,
  const IntegrationParameters& parameters,
  StrainstepArena& arena,
  const Workers& workers
) {
  const uint32_t substeps = islandSubsteps(bodies, island, parameters);
  const double h = parameters.dt / static_cast<double>(substeps);
  const double invH = 1.0 / h;
  const double maxAngularSpeed = 0.25 * std::numbers::pi / parameters.dt;
  std::vector<uint32_t> lattices;
  std::vector<StrainLattice*> sets;
  std::vector<RigidState> states(1u);
  std::vector<uint32_t> stateOf(bodies.rigids.size(), 0u);
  std::vector<uint8_t> stiff(bodies.lattices.size(), 0u);

  for (const BodyRef& ref : island.bodies) {
    if (ref.side == Side::Lattice) {
      LatticeBody& lattice = bodies.lattices[ref.index];
      StrainLattice& cached = arena.lattices[lattice.id];

      if (cached.mesh != lattice.mesh || cached.h != h) {
        StrainLattice built = buildElements(lattice, h, cached, arena);

        if (cached.h == h && cached.elements.size() == built.elements.size()) {
          for (size_t at = 0u; at != built.elements.size(); ++at) {
            if (built.elements[at].cell == cached.elements[at].cell) {
              built.elements[at].impulse = cached.elements[at].impulse;
              built.elements[at].volumeImpulse = cached.elements[at].volumeImpulse;
              built.elements[at].rotation = cached.elements[at].rotation;
            }
          }
        }

        cached = std::move(built);
      } else {
        refreshFlowed(lattice, cached, h, arena.inverses);
      }

      lattice.previousVelocity = lattice.velocity;
      lattice.initial = lattice.position;
      stiff[ref.index] = tooStiff(lattice, h) ? 1u : 0u;
      lattices.push_back(ref.index);
      sets.push_back(&cached);
    } else if (ref.side == Side::Rigid) {
      stateOf[ref.index] = static_cast<uint32_t>(states.size());
      states.push_back(gatherRigid(bodies.rigids[ref.index], ref.index));
    }
  }

  const RowBodies motion{bodies.lattices, states};
  const RowSettings settings{h, parameters.contactHertz, parameters.contactDampingRatio};
  std::vector<ContactRow> rows;

  for (const uint32_t index : island.contacts) {
    std::optional<ContactRow> row =
      makeContactRow(contacts[index], index, motion, stateOf, stiff, settings);

    if (row) {
      rows.push_back(*row);
    } else {
      contacts[index].memory = ContactMemory{};
    }
  }

  const double hertz = std::min(parameters.contactHertz, 0.125 * invH);
  const ManifoldSettings manifoldSettings{
    h,
    parameters.speculativeDistance,
    makeSoft(hertz, parameters.contactDampingRatio, h),
    makeSoft(2.0 * hertz, 0.5 * parameters.contactDampingRatio, h)
  };
  const std::vector<uint32_t> slots = islandSlots(bodies, cache, island, stateOf);
  std::vector<ManifoldRow> manifolds;
  std::vector<ManifoldRowPoint> points;

  prepareManifolds(bodies, cache, slots, states, stateOf, manifoldSettings, manifolds, points);

  const bool restitution = std::any_of(manifolds.begin(), manifolds.end(), [](const ManifoldRow& row) {
    return row.restitution > 0.0;
  });
  size_t cells = 0u;

  for (const StrainLattice* set : sets) {
    cells += set->elements.size();
  }

  const bool parallel = workers.count > 1u && sets.size() > 1u && cells >= PARALLEL_CELLS;
  const auto overSets = [&](const auto& visit) {
    if (!parallel) {
      visit(0u, static_cast<uint32_t>(sets.size()));

      return;
    }

    ++arena.parallelBatches;
    parallelFor(workers, static_cast<uint32_t>(sets.size()), 1u, [&](uint32_t begin, uint32_t end, uint32_t) {
      visit(begin, end);
    });
  };

  for (uint32_t substep = 0u; substep != substeps; ++substep) {
    for (const uint32_t index : lattices) {
      integrateNodes(bodies.lattices[index], parameters.gravity, h);
    }

    for (size_t item = 1u; item < states.size(); ++item) {
      integrateRigidVelocity(states[item], parameters.gravity, h);
    }

    for (ContactRow& row : rows) {
      refreshContactRow(row, motion, invH, parameters.contactSpeed, false);
      warmStartContactRow(row, motion);
    }

    for (const ManifoldRow& row : manifolds) {
      warmStartManifold(states, row, points);
    }

    overSets([&](uint32_t first, uint32_t last) {
      for (uint32_t set = first; set != last; ++set) {
        if (!sets[set]->exact) {
          continue;
        }

        LatticeBody& lattice = bodies.lattices[lattices[set]];

        for (StrainElement& element : sets[set]->elements) {
          updateElement(element, lattice.position);
          applyElement(element, lattice.velocity, element.impulse, element.volumeImpulse);
        }
      }
    });

    for (uint32_t pass = 0u; pass != PASSES; ++pass) {
      overSets([&](uint32_t first, uint32_t last) {
        for (uint32_t set = first; set != last; ++set) {
          LatticeBody& lattice = bodies.lattices[lattices[set]];
          const bool exact = sets[set]->exact;

          for (StrainElement& element : sets[set]->elements) {
            if (pass == 0u && !exact) {
              updateElement(element, lattice.position);
              applyElement(element, lattice.velocity, element.impulse, element.volumeImpulse);
            }

            solveElement(element, lattice.velocity);
          }
        }
      });

      for (ContactRow& row : rows) {
        solveContactRow(row, motion, true);
      }

      for (const ManifoldRow& row : manifolds) {
        pushManifold(states, row, points, invH, parameters.contactSpeed);
      }
    }

    for (const uint32_t index : lattices) {
      advanceNodes(bodies.lattices[index], h);
    }

    for (size_t item = 1u; item < states.size(); ++item) {
      integrateRigidPosition(states[item], h, parameters.maxLinearSpeed, maxAngularSpeed);
    }

    for (ContactRow& row : rows) {
      if (row.soft && row.gap > 0.0) {
        continue;
      }

      refreshContactRow(row, motion, invH, parameters.contactSpeed, true);
      solveContactRow(row, motion, false);
    }

    for (ManifoldRow& row : manifolds) {
      relaxManifold(states, row, points, invH);
    }

    for (ContactRow& row : rows) {
      row.total += row.normalImpulse;
    }
  }

  if (restitution) {
    for (uint32_t iteration = 0u; iteration != parameters.restitutionIterations; ++iteration) {
      for (const ManifoldRow& row : manifolds) {
        restituteManifold(states, row, points, invH, parameters.restitutionThreshold);
      }
    }
  }

  for (const ContactRow& row : rows) {
    storeContactRow(row, contacts[row.contact], h);
  }

  storeManifolds(cache, manifolds, points, h);

  for (const uint32_t index : lattices) {
    LatticeBody& lattice = bodies.lattices[index];

    (void)flowLattice(lattice);

    const std::vector<uint32_t> torn = overstretched(lattice);

    if (!torn.empty()) {
      (void)tearCells(lattice, torn);
    }
  }

  for (const uint32_t index : lattices) {
    bodies.lattices[index].inertial = bodies.lattices[index].position;
  }

  for (size_t item = 1u; item < states.size(); ++item) {
    storeRigid(states[item], bodies.rigids[states[item].body]);
  }

  return substeps;
}

}
