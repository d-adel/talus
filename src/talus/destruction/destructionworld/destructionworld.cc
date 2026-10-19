#include "destructionworld.h"

#include "loadqueue.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/bounds/bounds.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/scheduler/stopwatch.h>
#include <talus/dynamics/tiers/tiers.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace talus::destruction {

namespace {

constexpr uint32_t noPiece = 0xFFFFFFFFu;
constexpr uint32_t noIndex = 0xFFFFFFFFu;
constexpr uint32_t gatherRange = 64u;
constexpr uint32_t loadRange = 64u;
constexpr uint32_t queueRange = 128u;
constexpr uint32_t scanRange = 256u;
constexpr double hostMargin = 1.0e-6;
constexpr uint64_t guardSteps = 60u;
constexpr size_t freezeLimit = 2048u;
constexpr size_t removalLimit = 256u;

void rethrowFirst(std::span<const std::exception_ptr> failures) {
  for (const std::exception_ptr& failure : failures) {
    if (failure) {
      std::rethrow_exception(failure);
    }
  }
}

bool finitePoint(const glm::dvec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double kineticEnergy(const dynamics::RigidBody& body) {
  return 0.5 * body.mass * glm::dot(body.velocity, body.velocity)
         + 0.5 * glm::dot(body.angularVelocity, dynamics::worldInertia(body) * body.angularVelocity);
}

glm::dvec3 pathAt(const std::vector<PathPoint>& path, uint64_t step) {
  size_t at = 0u;

  while (at + 1u != path.size() && path[at + 1u].step <= step) {
    ++at;
  }

  if (at + 1u == path.size()) {
    return path.back().position;
  }

  const PathPoint& from = path[at];
  const PathPoint& to = path[at + 1u];
  const double along = static_cast<double>(step - from.step) / static_cast<double>(to.step - from.step);

  return from.position + along * (to.position - from.position);
}

glm::dvec3 entryPoint(const glm::dvec3& from, const Chunk& chunk) {
  const glm::dvec3 span = chunk.centre - from;
  double enter = 0.0;

  for (int axis = 0; axis != 3; ++axis) {
    if (span[axis] != 0.0) {
      const double low = (chunk.centre[axis] - chunk.halfExtents[axis] - from[axis]) / span[axis];
      const double high = (chunk.centre[axis] + chunk.halfExtents[axis] - from[axis]) / span[axis];

      enter = std::max(enter, std::min(low, high));
    }
  }

  return from + std::min(enter, 1.0) * span;
}

uint32_t parentOf(const FinerLevel& fine, uint32_t offset) {
  const auto above = std::upper_bound(fine.start.begin(), fine.start.end(), offset);

  return static_cast<uint32_t>(above - fine.start.begin()) - 1u;
}

struct End {
  dynamics::Side side = dynamics::Side::Static;
  uint32_t body = 0u;
  uint32_t child = dynamics::noChild;
  glm::dvec3 local{0.0};
};

std::vector<uint32_t> indexById(const dynamics::World& world) {
  std::vector<uint32_t> index(world.bodies.nextId, noIndex);

  for (uint32_t at = 0u; at != world.bodies.rigids.size(); ++at) {
    index[world.bodies.rigids[at].id] = at;
  }

  return index;
}

uint32_t rigidIndex(const std::vector<uint32_t>& index, uint32_t id) {
  if (id >= index.size() || index[id] == noIndex) {
    throw std::logic_error("A destruction piece lost its body");
  }

  return index[id];
}

bool unchanged(
  const StressState& stress,
  const StructureState& structure,
  std::span<const ChunkLoad> loads,
  const glm::dvec3& gravity
) {
  const auto same = [](const ChunkLoad& a, const ChunkLoad& b) {
    return a.chunk == b.chunk && a.force == b.force && a.torque == b.torque;
  };

  return stress.solved && !stress.eroding && stress.revision == structure.revision && stress.gravity == gravity
         && std::equal(stress.loads.begin(), stress.loads.end(), loads.begin(), loads.end(), same);
}

bool falling(const dynamics::RigidBody& body, const StressState& stress, std::span<const ChunkLoad> loads) {
  return body.mass > 0.0 && loads.empty() && stress.loads.empty();
}

bool bondless(const StructureState& structure) {
  return std::all_of(structure.broken.begin(), structure.broken.end(), [](uint8_t broken) { return broken != 0u; });
}

bool anchoredOnly(const ChunkedBuilding& chunked, const StructureState& structure) {
  for (uint32_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
    if (structure.alive[chunk] != 0u && !chunked.chunks[chunk].anchored) {
      return false;
    }
  }

  return true;
}

bool fracturable(const FinerLevel& fine, std::span<const uint32_t> ids, const StructureState& structure) {
  const auto original = static_cast<uint32_t>(fine.cells.size());

  for (uint32_t local = 0u; local != ids.size(); ++local) {
    const uint32_t id = ids[local];

    if (structure.alive[local] != 0u && id < original && fine.start[id] != fine.start[id + 1u]) {
      return true;
    }
  }

  return false;
}

void solveWithoutBonds(
  StressState& stress,
  const StructureState& structure,
  std::vector<ChunkLoad> loads,
  const glm::dvec3& gravity
) {
  std::fill(stress.force.begin(), stress.force.end(), glm::dvec3(0.0));
  std::fill(stress.moment.begin(), stress.moment.end(), glm::dvec3(0.0));
  stress.loads = std::move(loads);
  stress.gravity = gravity;
  stress.revision = structure.revision;
  stress.solved = true;
  stress.eroding = false;
  stress.pending = false;
}

uint32_t aliveCount(const StructureState& state) {
  return static_cast<uint32_t>(std::count(state.alive.begin(), state.alive.end(), uint8_t{1}));
}

std::vector<std::vector<uint32_t>> componentsOf(const StructureState& state) {
  std::vector<uint32_t> slot(state.alive.size(), noIndex);
  std::vector<std::vector<uint32_t>> groups;

  for (uint32_t chunk = 0u; chunk != state.alive.size(); ++chunk) {
    if (state.alive[chunk] == 0u) {
      continue;
    }

    const uint32_t component = state.component[chunk];

    if (slot[component] == noIndex) {
      slot[component] = static_cast<uint32_t>(groups.size());
      groups.emplace_back();
    }

    groups[slot[component]].push_back(chunk);
  }

  return groups;
}

const DestructionSettings& checked(const DestructionSettings& settings) {
  if (!(settings.friction >= 0.0) || !std::isfinite(settings.friction)
      || !(settings.restitution >= 0.0 && settings.restitution <= 1.0)) {
    throw std::invalid_argument("A destruction world needs a friction of at least zero and a restitution from 0 to 1");
  }

    if (!(settings.target >= 0.0) || !(settings.ceiling >= 0.0) || !(settings.removalFloor >= 0.0)) {
    throw std::invalid_argument("A destruction world needs a step target and a collapse ceiling of at least zero");
  }

  if (!(settings.discardBelow < std::numeric_limits<double>::infinity())) {
    throw std::invalid_argument("A debris lower boundary must be finite, or negative infinity to disable it");
  }

  return settings;
}

}

DestructionWorld::DestructionWorld(const DestructionSettings& settings) : m_settings(checked(settings)) {}

BuildingReport DestructionWorld::addBuilding(
  dynamics::World& world,
  const Building& building,
  std::span<const StructuralMaterial> materials,
  const ChunkerSettings& chunker
) {
  ChunkedBuilding chunked = chunkBuilding(building, materials, chunker);
  const auto chunks = static_cast<uint32_t>(chunked.chunks.size());
  const auto bonds = static_cast<uint32_t>(chunked.bonds.size());

  if (chunks == 0u) {
    throw std::invalid_argument("A destruction world needs a building with at least one chunk");
  }

  FinerLevel fine = finerLevel(building, chunked, materials);
  const uint32_t parts = fine.start.back();

  if (chunks > 0xFFFFFFFFu - m_nextKey || parts > 0xFFFFFFFFu - m_nextKey - chunks) {
    throw std::length_error("A destruction world has no chunk keys left for this building");
  }

  if (!std::isfinite(building.position.x) || !std::isfinite(building.position.y) || !std::isfinite(building.position.z)
      || !(glm::length(building.rotation) > 0.0) || !std::isfinite(glm::length(building.rotation))) {
    throw std::invalid_argument("A destruction world needs a building with a finite position and a rotation of non-zero length");
  }

  Piece piece;

  piece.building = static_cast<uint32_t>(m_buildings.size());
  piece.unchecked = true;
  piece.chunks.resize(chunks);
  piece.bonds.resize(bonds);
  std::iota(piece.chunks.begin(), piece.chunks.end(), 0u);
  std::iota(piece.bonds.begin(), piece.bonds.end(), 0u);
  piece.chunked = chunked;
  piece.structure = makeStructure(piece.chunked);
  piece.stress = makeStress(piece.chunked);

  std::vector<dynamics::CompoundChild> children;

  children.reserve(chunks);

  for (uint32_t chunk = 0u; chunk != chunks; ++chunk) {
    const Chunk& held = chunked.chunks[chunk];

    children.push_back(dynamics::CompoundChild{
      held.centre,
      glm::dquat(1.0, 0.0, 0.0, 0.0),
      held.halfExtents,
      held.mass / held.volume,
      m_nextKey + chunk
    });
  }

  const uint32_t index = dynamics::addCompoundBody(
    world.bodies,
    children,
    building.position,
    building.rotation,
    glm::dvec3(0.0),
    m_settings.friction,
    m_settings.restitution,
    true
  );
  const glm::dquat frame = world.bodies.rigids[index].rotation;
  BuildingReport report;

  piece.body = world.bodies.rigids[index].id;
  report.building = piece.building;
  report.firstKey = m_nextKey;
  report.body = piece.body;

  const StressSettings settle = stressSettings(
    glm::conjugate(glm::dquat(1.0, 0.0, 0.0, 0.0)) * (glm::conjugate(frame) * world.parameters.gravity),
    0u
  );

  while (!report.settled && report.updates != m_settings.settleUpdates) {
    const StressReport stressed = updateStress(piece.chunked, piece.structure, piece.stress, {}, settle);

    report.settled = !stressed.ran;
    report.updates += stressed.ran ? 1u : 0u;
    report.rounds += stressed.rounds;
    report.factorizations += stressed.factorizations;
    report.broken.insert(report.broken.end(), stressed.broken.begin(), stressed.broken.end());
    report.cracked.insert(report.cracked.end(), stressed.cracked.begin(), stressed.cracked.end());
    static_cast<void>(updateComponents(piece.structure, piece.chunked));
  }

  Record record;

  record.firstKey = m_nextKey;
  record.position = building.position;
  record.rotation = frame;
  record.source = building;
  record.materials.assign(materials.begin(), materials.end());
  record.chunker = chunker;
  record.fine = std::move(fine);
  record.retired.assign(bonds, 0u);
  record.chunked = std::move(chunked);
  record.chunkPiece.assign(chunks + parts, noPiece);
  record.chunkLocal.assign(chunks + parts, 0u);
  record.bondPiece.assign(bonds, noPiece);
  record.bondLocal.assign(bonds, 0u);
  m_buildings.push_back(std::move(record));
  m_nextKey += chunks + parts;
  m_anchors.push_back(static_cast<uint32_t>(m_pieces.size()));
  m_anchorTurns.push_back(frame);
  place(std::move(piece));

  return report;
}

void DestructionWorld::place(Piece piece) {
  const auto index = static_cast<uint32_t>(m_pieces.size());
  Record& record = m_buildings[piece.building];

  for (uint32_t local = 0u; local != piece.chunks.size(); ++local) {
    record.chunkPiece[piece.chunks[local]] = index;
    record.chunkLocal[piece.chunks[local]] = local;
  }

  for (uint32_t local = 0u; local != piece.bonds.size(); ++local) {
    record.bondPiece[piece.bonds[local]] = index;
    record.bondLocal[piece.bonds[local]] = local;
  }

  if (piece.body >= m_pieceOfBody.size()) {
    m_pieceOfBody.resize(piece.body + 1u, noPiece);
  }

  m_pieceOfBody[piece.body] = index;
  m_pieces.push_back(std::move(piece));
  touch(index);
}

uint32_t DestructionWorld::pieceOf(uint32_t body) const {
  return body < m_pieceOfBody.size() ? m_pieceOfBody[body] : noPiece;
}

void DestructionWorld::countBodies(const dynamics::World& world, const dynamics::Workers& workers, DestructionReport& report, bool updateRest) {
  if (updateRest) {
    m_rest.update(world, workers);
  }

  report.moving = 0u;
  report.awakeChunks = 0u;
  report.awakeManifolds = 0u;

  std::vector<std::array<uint32_t, 3>> sums(workers.count, std::array<uint32_t, 3>{});

  dynamics::parallelFor(
    workers,
    static_cast<uint32_t>(world.bodies.rigids.size()),
    scanRange,
    [&](uint32_t begin, uint32_t end, uint32_t worker) {
      for (uint32_t rigid = begin; rigid != end; ++rigid) {
        const dynamics::RigidBody& body = world.bodies.rigids[rigid];

        if (!(body.mass > 0.0) || body.asleep) {
          continue;
        }

        sums[worker][0] += m_rest.resting(body.id) ? 0u : 1u;

        if (pieceOf(body.id) != noPiece && body.compound != nullptr) {
          sums[worker][1] += static_cast<uint32_t>(body.compound->children.size());
        }
      }
    }
  );

  dynamics::parallelFor(
    workers,
    static_cast<uint32_t>(world.manifolds.size()),
    scanRange,
    [&](uint32_t begin, uint32_t end, uint32_t worker) {
      for (uint32_t slot = begin; slot != end; ++slot) {
        if (!world.manifolds.live(slot) || world.manifolds.at(slot).points.empty()) {
          continue;
        }

        const dynamics::CachedBodyPair& pair = world.manifolds.at(slot);
        bool awake = false;

        for (const dynamics::BodyRef& ref : {pair.first, pair.second}) {
          if (ref.side == dynamics::Side::Rigid) {
            const dynamics::RigidBody& body = world.bodies.rigids[ref.index];

            awake = awake || (body.mass > 0.0 && !body.asleep);
          }
        }

        sums[worker][2] += awake ? 1u : 0u;
      }
    }
  );

  for (const std::array<uint32_t, 3>& sum : sums) {
    report.moving += sum[0];
    report.awakeChunks += sum[1];
    report.awakeManifolds += sum[2];
  }

  report.bodies = static_cast<uint32_t>(world.bodies.rigids.size());
}

const RestTracker& DestructionWorld::rest() const noexcept {
  return m_rest;
}

void DestructionWorld::guardPiece(uint32_t at, uint64_t until) {
  if (at >= m_pieces.size() || !m_pieces[at].alive) {
    return;
  }

  const Piece& piece = m_pieces[at];
  const uint32_t first = m_buildings[piece.building].firstKey;

  m_guard.resize(m_nextKey, 0u);

  for (uint32_t local = 0u; local != piece.chunks.size(); ++local) {
    if (piece.structure.alive[local]) {
      const uint32_t key = first + piece.chunks[local];
      m_guard[key] = std::max(m_guard[key], until);
    }
  }
}

bool DestructionWorld::guarded(uint32_t at) const {
  const Piece& piece = m_pieces[at];
  const uint32_t first = m_buildings[piece.building].firstKey;

  for (uint32_t local = 0u; local != piece.chunks.size(); ++local) {
    const uint32_t key = first + piece.chunks[local];
    if (piece.structure.alive[local] && key < m_guard.size() && m_guard[key] >= m_steps) {
      return true;
    }
  }
  return false;
}

bool DestructionWorld::guardedBody(uint32_t body) const {
  const uint32_t at = pieceOf(body);

  return at != noPiece && guarded(at);
}

void DestructionWorld::guardTouched(const dynamics::World& world) {
  for (const uint32_t tool : m_tools) {
    for (const uint32_t slot : world.manifolds.pairsOf(tool)) {
      if (!world.manifolds.live(slot) || world.manifolds.at(slot).points.empty()) {
        continue;
      }

      const dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

      guardPiece(pieceOf(pair.firstId == tool ? pair.secondId : pair.firstId), m_steps + guardSteps);
    }
  }
}

void DestructionWorld::freezeResting(dynamics::World& world, DestructionReport& report) {
  const double limit = m_settings.target;

  if (!m_settings.activeSet || !(report.cost.total > limit)) {
    return;
  }

  struct Resting {
    uint32_t steps = 0u;
    uint32_t body = 0u;
    uint32_t rigid = 0u;
  };
  std::vector<Resting> resting;

  for (uint32_t rigid = 0u; rigid != world.bodies.rigids.size(); ++rigid) {
    const dynamics::RigidBody& body = world.bodies.rigids[rigid];
    const uint32_t at = pieceOf(body.id);

    if (at == noPiece || !(body.mass > 0.0) || body.asleep || !m_rest.resting(body.id) || m_pieces[at].pending
        || !m_pieces[at].cases.empty() || guarded(at)) {
      continue;
    }

    resting.push_back(Resting{m_rest.restSteps(body.id), body.id, rigid});
  }

  std::sort(resting.begin(), resting.end(), [](const auto& a, const auto& b) {
    return a.steps != b.steps ? a.steps > b.steps : a.body < b.body;
  });

  for (size_t slot = 0u; slot != std::min(resting.size(), freezeLimit); ++slot) {
    if (!(report.cost.total > limit)) {
      break;
    }

    const dynamics::BodyRef ref{dynamics::Side::Rigid, resting[slot].rigid};

    if (dynamics::freezeBody(world, ref)) {
      // A contact or support edit can thaw it before the tracker next observes zero mass.
      // Start a fresh quiet interval so that thawing cannot immediately freeze it again.
      m_rest.reset(resting[slot].body);
      m_frozen.emplace_back(resting[slot].body, m_steps);
      report.frozen += 1u;
      const dynamics::RigidBody& body = world.bodies.rigids[ref.index];
      report.awakeChunks -= body.compound != nullptr ? static_cast<uint32_t>(body.compound->children.size()) : 0u;

      for (const uint32_t pairSlot : world.manifolds.pairsOf(body.id)) {
        const dynamics::CachedBodyPair& pair = world.manifolds.at(pairSlot);

        if (!pair.live || pair.points.empty()) {
          continue;
        }

        bool active = false;

        for (const dynamics::BodyRef& end : {pair.first, pair.second}) {
          if (end.side == dynamics::Side::Rigid) {
            const dynamics::RigidBody& other = world.bodies.rigids[end.index];
            active = active || (other.mass > 0.0 && !other.asleep);
          }
        }

        report.awakeManifolds -= active ? 0u : 1u;
      }

      report.cost = predictCost(stepCounts(report));
    }
  }
}

void DestructionWorld::removeFrozen(dynamics::World& world, DestructionReport& report, const std::vector<uint32_t>& index) {
  const double limit = m_settings.target;
  const double bodyTerms = (costModel.rigid[2] + costModel.update[5]) * static_cast<double>(world.bodies.rigids.size());

  if (!m_settings.activeSet || !(report.cost.total > limit) || !(bodyTerms > m_settings.removalFloor)) {
    return;
  }

  struct Candidate {
    double volume = 0.0;
    uint64_t since = 0u;
    uint32_t body = 0u;
    uint32_t at = 0u;
  };

  std::vector<Candidate> candidates;

  for (const auto& [body, since] : m_frozen) {
    const uint32_t at = pieceOf(body);

    if (at == noPiece || body >= index.size() || index[body] == noIndex || !dynamics::isFrozen(world.bodies.rigids[index[body]])
        || aliveCount(m_pieces[at].structure) != 1u || guarded(at)) {
      continue;
    }

    const Piece& piece = m_pieces[at];
    const auto local = static_cast<uint32_t>(
      std::find(piece.structure.alive.begin(), piece.structure.alive.end(), uint8_t{1}) - piece.structure.alive.begin()
    );
    const glm::dvec3 half = piece.chunked.chunks[local].halfExtents;

    candidates.push_back(Candidate{8.0 * half.x * half.y * half.z, since, body, at});
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    return std::tie(a.volume, a.since, a.body) < std::tie(b.volume, b.since, b.body);
  });
  candidates.resize(std::min(candidates.size(), removalLimit));

  std::vector<uint32_t> bodies;

  for (const Candidate& candidate : candidates) {
    const double remainingBodyTerms = (costModel.rigid[2] + costModel.update[5]) * report.bodies;

    if (!(report.cost.total > limit) || !(remainingBodyTerms > m_settings.removalFloor)) {
      break;
    }

    Piece& piece = m_pieces[candidate.at];
    Record& record = m_buildings[piece.building];

    for (uint32_t local = 0u; local != piece.chunks.size(); ++local) {
      if (piece.structure.alive[local] != 0u) {
        removeChunk(piece.structure, piece.chunked, local);
        record.chunkPiece[piece.chunks[local]] = noPiece;
      }
    }

    release(candidate.at);
    bodies.push_back(candidate.body);
    --report.bodies;
    report.cost = predictCost(stepCounts(report));
  }

  if (!bodies.empty()) {
    report.removedFrozen = dynamics::removeBodies(world, bodies);
  }
}

void DestructionWorld::removeBelow(dynamics::World& world, DestructionReport& report) {
  if (!std::isfinite(m_settings.discardBelow)) {
    return;
  }

  std::vector<uint32_t> bodies;

  for (const dynamics::RigidBody& body : world.bodies.rigids) {
    const uint32_t at = pieceOf(body.id);

    if (at == noPiece || !(body.mass > 0.0 || body.frozenMass > 0.0) || guarded(at)) {
      continue;
    }

    Piece& piece = m_pieces[at];
    const double reach = body.compound != nullptr ? body.compound->reach : body.radius;

    if (body.position.y + reach >= m_settings.discardBelow || piece.pending || !piece.cases.empty()) {
      continue;
    }

    Record& record = m_buildings[piece.building];

    for (uint32_t local = 0u; local != piece.chunks.size(); ++local) {
      if (piece.structure.alive[local] != 0u) {
        removeChunk(piece.structure, piece.chunked, local);
        record.chunkPiece[piece.chunks[local]] = noPiece;
      }
    }

    release(at);
    bodies.push_back(body.id);
  }

  if (!bodies.empty()) {
    report.discarded = dynamics::removeBodies(world, bodies);
  }
}

bool DestructionWorld::collapseActive(const dynamics::World& world, const std::vector<uint32_t>& index) {
  bool moving = false;

  for (const uint32_t id : m_collapse) {
    if (id < index.size() && index[id] != noIndex) {
      const dynamics::RigidBody& body = world.bodies.rigids[index[id]];

      moving = moving || (body.mass > 0.0 && !body.asleep);
    }
  }

  if (!moving) {
    m_collapse.clear();
  }

  return moving;
}

void DestructionWorld::queueCase(uint32_t at, LoadCase next, DestructionReport& report) {
  Piece& piece = m_pieces[at];
  const std::vector<ChunkLoad> loads = loadsFrom(at, next.contributions);
  const bool breaks = !breakingChunks(at, next.contributions).empty();
  const auto differs = [&](const LoadCase& a, const LoadCase& b) {
    return loadChange(loadsFrom(at, a.contributions), a.gravity, loadsFrom(at, b.contributions), b.gravity);
  };
  const double change = piece.cases.empty() ? loadChange(piece.heldLoads, piece.heldGravity, loads, next.gravity)
                                            : loadChange(loadsFrom(at, piece.cases.back().contributions), piece.cases.back().gravity, loads, next.gravity);
  const bool impulse = std::any_of(next.contributions.begin(), next.contributions.end(), [](const Contribution& contribution) {
    return contribution.impulse != glm::dvec3(0.0);
  });
  const bool near = !breaks && !impulse && withinTier(change, piece.stress.utilization) && (piece.pending || !piece.cases.empty());
  const auto merge = [](const LoadCase& older, LoadCase& newer) {
    for (Contribution contribution : older.contributions) {
      if (contribution.impulse != glm::dvec3(0.0)) {
        contribution.force = glm::dvec3(0.0);
        newer.contributions.push_back(contribution);
      }
    }

    std::sort(newer.contributions.begin(), newer.contributions.end(), earlier);
  };
  const size_t protectedCases = !piece.cases.empty() && piece.cases.front().phase != CasePhase::Queued ? 1u : 0u;

  switch (enqueueCase(piece.cases, std::move(next), near, queuedCases, differs, merge, protectedCases)) {
  case Enqueued::Dropped:
  case Enqueued::Replaced:
    report.casesReplaced += 1u;
    break;
  case Enqueued::Joined:
    report.casesQueued += 1u;
    break;
  case Enqueued::Merged:
    report.casesQueued += 1u;
    report.casesMerged += 1u;
    break;
  }
}

double DestructionWorld::roundBound(uint32_t at) const {
  const Piece& piece = m_pieces[at];
  const uint32_t alive = aliveCount(piece.structure);
  const bool large = m_settings.wideChunks != 0u && alive >= m_settings.wideChunks;
  const auto work = static_cast<double>(factorWorkBound(piece.stress, piece.structure));

  return (large ? costModel.stress[1] : costModel.stress[0]) * work + costModel.stress[2] * alive;
}

void DestructionWorld::solvePieces(
  const dynamics::Workers& workers,
  const std::vector<uint32_t>& queue,
  DestructionReport& report
) {
  const auto solves = static_cast<uint32_t>(queue.size());
  const double limit = report.collapse ? m_settings.ceiling : m_settings.target;
  const bool bounded = std::isfinite(limit) || std::isfinite(m_settings.ceiling);
  const StepCounts before = stepCounts(report);
  const double spent = rigidCost(before) + updateCost(before);
  std::vector<StressReport> stressed(solves);
  std::vector<std::exception_ptr> failures(solves);
  std::vector<uint8_t> running(solves, 0u);

  const auto absorb = [](StressReport& into, const StressReport& from) {
    into.ran = into.ran || from.ran;
    into.rounds += from.rounds;
    into.factorizations += from.factorizations;
    into.solvedChunks += from.solvedChunks;
    into.factorizedChunks += from.factorizedChunks;
    into.factorWork += from.factorWork;
    into.factorizing += from.factorizing;
    into.pending = from.pending;
    into.broken.insert(into.broken.end(), from.broken.begin(), from.broken.end());
    into.cracked.insert(into.cracked.end(), from.cracked.begin(), from.cracked.end());
  };
  const auto stressSoFar = [&]() {
    StepCounts counts;

    for (uint32_t slot = 0u; slot != solves; ++slot) {
      counts.factorWork += stressed[slot].factorWork;
      counts.solvedChunks += stressed[slot].solvedChunks;
      counts.updates += stressed[slot].ran ? 1u : 0u;
    }

    counts.wideWork = report.wideWork;

    return stressCost(counts);
  };
  const auto wave = [&](const std::vector<uint8_t>& chosen, uint32_t rounds) {
    std::vector<StressReport> part(solves);

    dynamics::parallelFor(workers, solves, 1u, [&](uint32_t begin, uint32_t end, uint32_t) {
      for (uint32_t slot = begin; slot != end; ++slot) {
        if (chosen[slot] == 0u || failures[slot]) {
          continue;
        }

        try {
          Piece& piece = m_pieces[queue[slot]];
          StressSettings settings = stressSettings(piece.heldGravity, rounds);

          settings.wideChunks = m_settings.wideChunks;
          part[slot] = updateStress(piece.chunked, piece.structure, piece.stress, piece.heldLoads, settings);
        } catch (...) {
          failures[slot] = std::current_exception();
        }
      }
    });

    for (uint32_t slot = 0u; slot != solves; ++slot) {
      if (chosen[slot] == 0u || failures[slot]) {
        continue;
      }

      if (part[slot].wide) {
        try {
          Piece& piece = m_pieces[queue[slot]];
          StressSettings settings = stressSettings(piece.heldGravity, rounds == 0u ? 0u : rounds - part[slot].rounds);

          settings.workers = workers;

          StressReport rest = updateStress(piece.chunked, piece.structure, piece.stress, piece.heldLoads, settings);

          report.wide += 1u;
          report.wideWork += rest.factorWork;
          rest.factorWork = 0u;
          absorb(part[slot], rest);
        } catch (...) {
          failures[slot] = std::current_exception();
          continue;
        }
      }

      absorb(stressed[slot], part[slot]);
    }
  };

  bool oldest = true;
  double carriedBound = 0.0;

  for (uint32_t slot = 0u; slot != solves; ++slot) {
    const Piece& piece = m_pieces[queue[slot]];

    if (m_settings.wideChunks == 0u || aliveCount(piece.structure) < m_settings.wideChunks) {
      running[slot] = 1u;
      continue;
    }

    const double bound = roundBound(queue[slot]);

    if (oldest || spent + carriedBound + bound <= m_settings.ceiling) {
      running[slot] = 1u;
      carriedBound += bound;
    } else {
      report.deferred += 1u;
    }

    oldest = false;
  }

  wave(running, bounded ? 1u : 0u);

  uint32_t firstRounds = 0u;

  for (const StressReport& done : stressed) {
    firstRounds += done.rounds;
  }

  report.roundWork = stressSoFar();

  while (bounded) {
    const double predicted = spent + stressSoFar();
    std::vector<uint8_t> chosen(solves, 0u);
    double granted = 0.0;
    bool any = false;

    for (uint32_t slot = 0u; slot != solves; ++slot) {
      if (running[slot] == 0u || failures[slot] || !stressed[slot].pending) {
        continue;
      }

      const double bound = roundBound(queue[slot]);

      if (predicted + granted + bound <= limit) {
        chosen[slot] = 1u;
        granted += bound;
        any = true;
      }
    }

    if (!any) {
      break;
    }

    wave(chosen, 1u);
  }

  const double predicted = spent + stressSoFar();

  report.room = limit - spent;

  if (bounded && predicted > limit) {
    report.overRounds = firstRounds;
    report.overWork = predicted - std::max(limit, spent);
  }

  for (uint32_t slot = 0u; slot != solves; ++slot) {
    Piece& piece = m_pieces[queue[slot]];

    if (failures[slot]) {
      piece.pending = false;
      piece.heldLoads.clear();
      piece.stress.pending = false;
      piece.stress.solved = false;
      continue;
    }

    if (running[slot] == 0u) {
      report.pending += 1u;
      continue;
    }

    const StressReport& done = stressed[slot];

    report.solved += done.ran ? 1u : 0u;
    report.rounds += done.rounds;
    report.factorizations += done.factorizations;
    report.solvedChunks += done.solvedChunks;
    report.factorizedChunks += done.factorizedChunks;
    report.factorWork += done.factorWork;
    report.profile.factorizing += done.factorizing;

    if (done.pending) {
      report.pending += 1u;
    } else {
      piece.pending = false;
      piece.heldLoads.clear();
    }

    for (const uint32_t bond : done.broken) {
      report.broken.push_back(BrokenBond{piece.building, piece.bonds[bond]});
      report.maxBreakDelay = std::max(report.maxBreakDelay, m_steps - piece.since);
    }
  }

  rethrowFirst(failures);
}

std::vector<std::vector<ChunkLoad>> DestructionWorld::queuedLoads(uint32_t body) const {
  const uint32_t at = pieceOf(body);
  std::vector<std::vector<ChunkLoad>> found;

  if (at == noPiece) {
    return found;
  }

  for (const LoadCase& queued : m_pieces[at].cases) {
    if (queued.phase != CasePhase::Queued) {
      continue;
    }

    std::vector<ChunkLoad> loads = loadsFrom(at, queued.contributions);

    for (ChunkLoad& load : loads) {
      load.chunk = m_pieces[at].chunks[load.chunk];
    }

    found.push_back(std::move(loads));
  }

  return found;
}

StepCounts stepCounts(const DestructionReport& report) {
  StepCounts counts;

  counts.moving = report.moving;
  counts.awakeChunks = report.awakeChunks;
  counts.bodies = report.bodies;
  counts.visited = report.visited;
  counts.gathered = report.gathered;
  counts.awakeManifolds = report.awakeManifolds;
  counts.fractures = static_cast<uint32_t>(report.fractured.size());
  counts.broken = static_cast<uint32_t>(report.broken.size());
  counts.factorWork = report.factorWork;
  counts.wideWork = report.wideWork;
  counts.solvedChunks = report.solvedChunks;
  counts.updates = report.solved;

  return counts;
}

void DestructionWorld::touch(uint32_t at) {
  m_touched.push_back(at);
}

void DestructionWorld::release(uint32_t at) {
  Piece& piece = m_pieces[at];

  piece.alive = false;

  if (pieceOf(piece.body) == at) {
    m_pieceOfBody[piece.body] = noPiece;
  }
}

uint32_t DestructionWorld::removeChunks(
  dynamics::World& world,
  uint32_t building,
  std::span<const uint32_t> chunks
) {
  const Record& checkedRecord = recordOf(building);

  for (const uint32_t chunk : chunks) {
    if (chunk >= checkedRecord.chunkPiece.size()) {
      throw std::out_of_range("A building has no chunk with that index");
    }
  }

  Record& record = m_buildings[building];
  std::vector<std::pair<uint32_t, uint32_t>> taken;

  for (const uint32_t chunk : chunks) {
    const uint32_t piece = record.chunkPiece[chunk];

    if (piece == noPiece) {
      continue;
    }

    Piece& holder = m_pieces[piece];

    removeChunk(holder.structure, holder.chunked, record.chunkLocal[chunk]);
    record.chunkPiece[chunk] = noPiece;
    taken.emplace_back(piece, record.firstKey + chunk);
  }

  std::sort(taken.begin(), taken.end());

  for (size_t begin = 0u; begin != taken.size();) {
    const uint32_t piece = taken[begin].first;
    std::vector<uint32_t> keys;
    size_t end = begin;

    while (end != taken.size() && taken[end].first == piece) {
      keys.push_back(taken[end].second);
      ++end;
    }

    Piece& holder = m_pieces[piece];

    touch(piece);
    guardPiece(piece, m_steps + 1u + guardSteps);

    if (aliveCount(holder.structure) == 0u) {
      if (!dynamics::removeBody(world, holder.body)) {
        throw std::logic_error("A destruction body could not be removed");
      }

      release(piece);
    } else if (!dynamics::removeCompoundChildren(world, holder.body, keys)) {
      throw std::logic_error("A destruction body refused to give up its chunks");
    }

    begin = end;
  }

  return static_cast<uint32_t>(taken.size());
}

uint32_t DestructionWorld::removeChunksAt(dynamics::World& world, const glm::dvec3& point) {
  if (!finitePoint(point)) {
    throw std::invalid_argument("Chunks can only be removed at a finite point");
  }

  std::vector<std::vector<uint32_t>> hosts(m_buildings.size());

  for (const dynamics::RigidBody& body : world.bodies.rigids) {
    const uint32_t at = pieceOf(body.id);

    if (at == noPiece || body.compound == nullptr) {
      continue;
    }

    const dynamics::CompoundShape& shape = *body.compound;
    const glm::dvec3 held = glm::conjugate(body.rotation) * (point - body.position);

    if (!dynamics::contains(dynamics::grown(shape.bounds, hostMargin), dynamics::Bounds{held, held})) {
      continue;
    }

    const Piece& piece = m_pieces[at];
    const uint32_t firstKey = m_buildings[piece.building].firstKey;

    for (uint32_t index = 0u; index != shape.children.size(); ++index) {
      const dynamics::CompoundChild& child = shape.children[index];
      const glm::dvec3 local =
        glm::conjugate(body.rotation * child.rotation) * (point - (body.position + body.rotation * child.centre));
      uint8_t touching = 0u;
      bool inside = true;

      for (int axis = 0; axis != 3 && inside; ++axis) {
        const double reach = std::abs(local[axis]);

        inside = reach <= child.half[axis];

        if (reach == child.half[axis]) {
          touching = static_cast<uint8_t>(touching | (1u << (2 * axis + (local[axis] > 0.0 ? 1 : 0))));
        }
      }

      if (inside && (touching & shape.exposed[index]) == 0u) {
        hosts[piece.building].push_back(child.key - firstKey);
      }
    }
  }

  uint32_t removed = 0u;

  for (uint32_t building = 0u; building != hosts.size(); ++building) {
    if (!hosts[building].empty()) {
      std::sort(hosts[building].begin(), hosts[building].end());
      removed += removeChunks(world, building, hosts[building]);
    }
  }

  return removed;
}

uint32_t DestructionWorld::fractureChunks(
  dynamics::World& world,
  uint32_t building,
  std::span<const uint32_t> chunks
) {
  const Record& record = recordOf(building);

  for (const uint32_t chunk : chunks) {
    if (chunk >= record.chunkPiece.size()) {
      throw std::out_of_range("A building has no chunk with that index");
    }
  }

  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());
  std::vector<std::pair<uint32_t, uint32_t>> taken;

  for (const uint32_t chunk : chunks) {
    if (chunk < original && record.fine.start[chunk] != record.fine.start[chunk + 1u] && record.chunkPiece[chunk] != noPiece) {
      taken.emplace_back(record.chunkPiece[chunk], chunk);
    }
  }

  std::sort(taken.begin(), taken.end());
  taken.erase(std::unique(taken.begin(), taken.end()), taken.end());

  for (size_t begin = 0u; begin != taken.size();) {
    const uint32_t piece = taken[begin].first;
    std::vector<uint32_t> held;
    size_t end = begin;

    while (end != taken.size() && taken[end].first == piece) {
      held.push_back(taken[end].second);
      ++end;
    }

    fracturePiece(world, piece, held);
    begin = end;
  }

  return static_cast<uint32_t>(taken.size());
}

void DestructionWorld::fracturePiece(dynamics::World& world, uint32_t at, std::span<const uint32_t> chunks) {
  const Piece& before = m_pieces[at];
  Record& record = m_buildings[before.building];
  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());
  const auto knownBonds = static_cast<uint32_t>(record.chunked.bonds.size() + record.made.size());
  std::vector<std::vector<Chunk>> parts;
  std::vector<uint32_t> firsts;
  std::vector<uint32_t> partIds;

  for (const uint32_t chunk : chunks) {
    const Chunk& whole = before.chunked.chunks[record.chunkLocal[chunk]];

    firsts.push_back(original + record.fine.start[chunk]);
    parts.push_back(finerChunks(record.source, record.materials, whole, record.fine.cells[chunk]));

    for (uint32_t cell = 0u; cell != parts.back().size(); ++cell) {
      partIds.push_back(firsts.back() + cell);
    }
  }

  std::vector<uint32_t> ids;

  ids.reserve(before.chunks.size() + partIds.size());
  std::merge(before.chunks.begin(), before.chunks.end(), partIds.begin(), partIds.end(), std::back_inserter(ids));

  const auto localOf = [&ids](uint32_t id) {
    return static_cast<uint32_t>(std::lower_bound(ids.begin(), ids.end(), id) - ids.begin());
  };
  const auto placeOf = [&chunks](uint32_t id) {
    const auto found = std::lower_bound(chunks.begin(), chunks.end(), id);

    return found != chunks.end() && *found == id ? static_cast<size_t>(found - chunks.begin()) : chunks.size();
  };

  Piece piece;

  piece.building = before.building;
  piece.body = before.body;
  piece.unchecked = true;
  piece.chunks = ids;
  piece.chunked.chunks.resize(ids.size());
  piece.cases = before.cases;

  if (before.pending && before.activeCase.has_value()) {
    piece.cases.insert(piece.cases.begin(), *before.activeCase);
  }

  for (uint32_t local = 0u; local != before.chunks.size(); ++local) {
    piece.chunked.chunks[localOf(before.chunks[local])] = before.chunked.chunks[local];
  }

  for (size_t entry = 0u; entry != chunks.size(); ++entry) {
    for (uint32_t cell = 0u; cell != parts[entry].size(); ++cell) {
      piece.chunked.chunks[localOf(firsts[entry] + cell)] = parts[entry][cell];
      m_guard.resize(m_nextKey, 0u);
      m_guard[record.firstKey + firsts[entry] + cell] = m_guard[record.firstKey + chunks[entry]];
    }
  }

  for (uint32_t bond = 0u; bond != before.chunked.bonds.size(); ++bond) {
    Bond moved = before.chunked.bonds[bond];

    moved.first = localOf(before.chunks[moved.first]);
    moved.second = localOf(before.chunks[moved.second]);
    piece.chunked.bonds.push_back(moved);
    piece.bonds.push_back(before.bonds[bond]);
  }

  struct Made {
    Bond bond;
    double health = 1.0;
    uint8_t cracked = 0u;
  };

  std::vector<Made> made;
  std::vector<uint32_t> retiring;

  for (size_t entry = 0u; entry != chunks.size(); ++entry) {
    const std::vector<Bond> inner =
      innerBonds(record.source, record.materials, record.chunker, parts[entry], record.fine.cells[chunks[entry]], firsts[entry]);

    for (const Bond& bond : inner) {
      made.push_back(Made{bond, 1.0, 0u});
    }
  }

  for (size_t entry = 0u; entry != chunks.size(); ++entry) {
    const uint32_t local = record.chunkLocal[chunks[entry]];

    for (uint32_t slot = before.structure.bondStart[local]; slot != before.structure.bondStart[local + 1u]; ++slot) {
      const uint32_t bond = before.structure.bondList[slot];

      if (before.structure.broken[bond] != 0u) {
        continue;
      }

      Bond global = before.chunked.bonds[bond];

      global.first = before.chunks[global.first];
      global.second = before.chunks[global.second];

      const size_t firstAt = placeOf(global.first);
      const size_t secondAt = placeOf(global.second);

      if (firstAt != chunks.size() && secondAt != chunks.size() && global.first != chunks[entry]) {
        continue;
      }

      std::vector<Bond> pieces{global};

      for (const size_t place : {firstAt, secondAt}) {
        if (place == chunks.size()) {
          continue;
        }

        std::vector<Bond> finer;

        for (const Bond& coarse : pieces) {
          const std::vector<Bond> split = splitBond(coarse, chunks[place], parts[place], firsts[place]);

          finer.insert(finer.end(), split.begin(), split.end());
        }

        pieces = std::move(finer);
      }

      for (const Bond& split : pieces) {
        made.push_back(Made{split, before.structure.health[bond], before.structure.cracked[bond]});
      }

      retiring.push_back(before.bonds[bond]);
    }
  }

  const auto kept = static_cast<uint32_t>(piece.chunked.bonds.size());

  for (size_t entry = 0u; entry != made.size(); ++entry) {
    Bond local = made[entry].bond;

    local.first = localOf(made[entry].bond.first);
    local.second = localOf(made[entry].bond.second);
    piece.chunked.bonds.push_back(local);
    piece.bonds.push_back(knownBonds + static_cast<uint32_t>(entry));
  }

  piece.structure = makeStructure(piece.chunked);

  for (uint32_t local = 0u; local != before.chunks.size(); ++local) {
    piece.structure.alive[localOf(before.chunks[local])] = before.structure.alive[local];
  }

  for (uint32_t bond = 0u; bond != kept; ++bond) {
    piece.structure.health[bond] = before.structure.health[bond];
    piece.structure.broken[bond] = before.structure.broken[bond];
    piece.structure.cracked[bond] = before.structure.cracked[bond];
  }

  for (const uint32_t chunk : chunks) {
    const uint32_t local = localOf(chunk);

    piece.structure.alive[local] = 0u;

    for (uint32_t slot = piece.structure.bondStart[local]; slot != piece.structure.bondStart[local + 1u]; ++slot) {
      piece.structure.broken[piece.structure.bondList[slot]] = 1u;
      piece.structure.health[piece.structure.bondList[slot]] = 0.0;
    }
  }

  for (size_t entry = 0u; entry != made.size(); ++entry) {
    piece.structure.health[kept + entry] = made[entry].health;
    piece.structure.cracked[kept + entry] = made[entry].cracked;
  }

  std::fill(piece.structure.component.begin(), piece.structure.component.end(), 0u);
  piece.structure.dirty.assign(1u, 0u);
  piece.structure.revision = before.structure.revision + 1u;
  static_cast<void>(updateComponents(piece.structure, piece.chunked));
  piece.stress = makeStressLike(piece.chunked, before.stress);

  const std::optional<dynamics::BodyRef> ref = dynamics::findBody(world.bodies, before.body);

  if (!ref.has_value() || ref->side != dynamics::Side::Rigid || world.bodies.rigids[ref->index].compound == nullptr) {
    throw std::logic_error("A destruction piece lost its body");
  }

  const dynamics::CompoundShape& shape = *world.bodies.rigids[ref->index].compound;
  std::vector<dynamics::ChildReplacement> replacements;

  for (size_t entry = 0u; entry != chunks.size(); ++entry) {
    const std::optional<uint32_t> index = dynamics::findChild(shape, record.firstKey + chunks[entry]);

    if (!index.has_value()) {
      throw std::logic_error("A destruction body does not hold the chunk that fractures");
    }

    const dynamics::CompoundChild& child = shape.children[*index];
    const Chunk& whole = before.chunked.chunks[record.chunkLocal[chunks[entry]]];
    dynamics::ChildReplacement replacement;

    replacement.key = child.key;

    for (uint32_t cell = 0u; cell != parts[entry].size(); ++cell) {
      replacement.parts.push_back(dynamics::CompoundChild{
        child.centre + child.rotation * (parts[entry][cell].centre - whole.centre),
        child.rotation,
        parts[entry][cell].halfExtents,
        child.density,
        record.firstKey + firsts[entry] + cell
      });
    }

    replacements.push_back(std::move(replacement));
  }

  if (!dynamics::replaceCompoundChildren(world, before.body, replacements)) {
    throw std::logic_error("A destruction body refused the sub-boxes of its fractured chunks");
  }

  for (const Made& entry : made) {
    record.made.push_back(entry.bond);
  }

  record.retired.resize(knownBonds + made.size(), 0u);
  record.bondPiece.resize(knownBonds + made.size(), noPiece);
  record.bondLocal.resize(knownBonds + made.size(), 0u);

  for (const uint32_t bond : retiring) {
    record.retired[bond] = 1u;
  }

  m_pieces[at] = std::move(piece);

  const Piece& placed = m_pieces[at];

  for (uint32_t local = 0u; local != placed.chunks.size(); ++local) {
    const uint32_t id = placed.chunks[local];

    if (record.chunkPiece[id] == at || std::binary_search(partIds.begin(), partIds.end(), id)) {
      record.chunkPiece[id] = at;
      record.chunkLocal[id] = local;
    }
  }

  for (const uint32_t chunk : chunks) {
    record.chunkPiece[chunk] = noPiece;
  }

  for (uint32_t local = 0u; local != placed.bonds.size(); ++local) {
    const uint32_t id = placed.bonds[local];

    if (record.bondPiece[id] == at || id >= knownBonds) {
      record.bondPiece[id] = at;
      record.bondLocal[id] = local;
    }
  }

  for (LoadCase& queued : m_pieces[at].cases) {
    queued.contributions = ontoParts(at, chunks, queued.contributions);
  }

  std::vector<Contribution> impulses;

  for (const ChunkImpulse& impulse : record.impulses) {
    impulses.push_back(Contribution{impulse.chunk, 0u, 0u, 0u, 0u, 0u, 0u, glm::dvec3(0.0), impulse.point, impulse.impulse});
  }

  record.impulses.clear();

  for (const Contribution& impulse : ontoParts(at, chunks, impulses)) {
    record.impulses.push_back(ChunkImpulse{impulse.chunk, impulse.impulse, impulse.point});
  }

  touch(at);
}

void DestructionWorld::blastAt(const Blast& blast) {
  if (blast.step <= m_steps || !std::isfinite(blast.centre.x) || !std::isfinite(blast.centre.y)
      || !std::isfinite(blast.centre.z) || !(blast.strength >= 0.0) || !std::isfinite(blast.strength)
      || !(blast.radius > 0.0) || !std::isfinite(blast.radius)) {
    throw std::invalid_argument("A blast needs a step still to come, a finite centre, a strength of at least zero and a positive radius");
  }

  m_blasts.push_back(blast);
}

void DestructionWorld::prepare(dynamics::World& world) {
  dynamics::Stopwatch clock;
  const uint64_t next = m_steps + 1u;

  world.thawOnContact = m_tools;
  applyDrives(world, next);
  applyShots(world, next);
  std::vector<Blast> waiting;

  for (const Blast& blast : m_blasts) {
    if (blast.step != next) {
      if (blast.step > next) {
        waiting.push_back(blast);
      }

      continue;
    }

    m_blastRemoved += removeChunksAt(world, blast.centre);

    for (const BlastImpulse& hit : blastImpulses(world, blast)) {
      const std::optional<dynamics::BodyRef> ref = dynamics::findBody(world.bodies, hit.body);

      if (!ref.has_value() || ref->side != dynamics::Side::Rigid) {
        continue;
      }

      if (dynamics::isFrozen(world.bodies.rigids[ref->index])) {
        static_cast<void>(dynamics::thawBody(world, hit.body));
      }

      const dynamics::RigidBody& body = world.bodies.rigids[ref->index];
      const uint32_t at = pieceOf(hit.body);
      const bool moving = body.mass > 0.0;

      if (at != noPiece) {
        guardPiece(at, next + guardSteps);
      }
      bool reached = false;

      if (at != noPiece && hit.child != dynamics::noChild) {
        const Record& record = m_buildings[m_pieces[at].building];
        const uint32_t chunk = hit.child - record.firstKey;

        if (hit.child >= record.firstKey && chunk < record.chunkPiece.size() && record.chunkPiece[chunk] == at) {
          const Layout layout = layoutOf(body, at);
          const glm::dquat back = glm::conjugate(layout.turn);
          const glm::dquat unturn = glm::conjugate(body.rotation);
          const Chunk& held = m_pieces[at].chunked.chunks[record.chunkLocal[chunk]];
          const glm::dvec3 from = back * (unturn * (blast.centre - body.position) - layout.origin);

          m_blastLoads.push_back(BlastLoad{
            m_pieces[at].building,
            chunk,
            static_cast<uint32_t>(m_blastLoads.size()),
            back * (unturn * hit.impulse),
            entryPoint(from, held),
            moving ? glm::dvec3(0.0) : hit.impulse
          });
          reached = true;
        }
      }

      if (moving && dynamics::applyImpulse(world, hit.body, hit.impulse, hit.point)) {
        reached = true;
      }

      m_blasted += reached ? 1u : 0u;
    }
  }

  m_blasts = std::move(waiting);
  m_prepared += clock.lap();
}

void DestructionWorld::drive(const Drive& scripted) {
  if (scripted.path.empty() || scripted.path.front().step <= m_steps) {
    throw std::invalid_argument("A drive needs a path that starts at a step still to come");
  }

  for (size_t at = 0u; at != scripted.path.size(); ++at) {
    if (!finitePoint(scripted.path[at].position)
        || (at != 0u && !(scripted.path[at].step > scripted.path[at - 1u].step))) {
      throw std::invalid_argument("A drive's path needs finite points at rising steps");
    }
  }

  m_drives.push_back(scripted);
  m_tools.push_back(scripted.body);
}

void DestructionWorld::fire(const Shot& shot) {
  if (shot.step <= m_steps || !finitePoint(shot.from) || !finitePoint(shot.velocity)) {
    throw std::invalid_argument("A shot needs a step still to come, a finite start and a finite velocity");
  }

  m_shots.push_back(shot);
  m_tools.push_back(shot.body);
}

double DestructionWorld::drivenWork() const noexcept {
  return m_drivenWork;
}

void DestructionWorld::applyDrives(dynamics::World& world, uint64_t next) {
  for (const Drive& scripted : m_drives) {
    if (next < scripted.path.front().step || next > scripted.path.back().step) {
      continue;
    }

    const std::optional<dynamics::BodyRef> ref = dynamics::findBody(world.bodies, scripted.body);

    if (!ref.has_value() || ref->side != dynamics::Side::Rigid) {
      continue;
    }

    if (dynamics::isFrozen(world.bodies.rigids[ref->index])) {
      static_cast<void>(dynamics::thawBody(world, scripted.body));
    }

    if (!(world.bodies.rigids[ref->index].mass > 0.0)) {
      continue;
    }

    guardPiece(pieceOf(scripted.body), next + guardSteps);
    const double before = kineticEnergy(world.bodies.rigids[ref->index]);
    const glm::dvec3 velocity =
      (pathAt(scripted.path, next) - world.bodies.rigids[ref->index].position) / world.parameters.dt;

    if (!dynamics::setVelocity(world, scripted.body, velocity)
        || !dynamics::setAngularVelocity(world, scripted.body, glm::dvec3(0.0))) {
      throw std::logic_error("A driven body refused its velocity");
    }

    m_drivenWork += kineticEnergy(world.bodies.rigids[ref->index]) - before;
    ++m_driven;
  }

  std::erase_if(m_drives, [next](const Drive& finished) {
    return finished.path.back().step <= next;
  });
}

void DestructionWorld::applyShots(dynamics::World& world, uint64_t next) {
  std::vector<Shot> waiting;

  for (const Shot& shot : m_shots) {
    if (shot.step != next) {
      if (shot.step > next) {
        waiting.push_back(shot);
      }

      continue;
    }

    const std::optional<dynamics::BodyRef> ref = dynamics::findBody(world.bodies, shot.body);

    if (!ref.has_value() || ref->side != dynamics::Side::Rigid) {
      continue;
    }

    if (dynamics::isFrozen(world.bodies.rigids[ref->index])) {
      static_cast<void>(dynamics::thawBody(world, shot.body));
    }

    if (!(world.bodies.rigids[ref->index].mass > 0.0)) {
      continue;
    }

    guardPiece(pieceOf(shot.body), next + guardSteps);
    const glm::dquat turn = world.bodies.rigids[ref->index].rotation;

    if (!dynamics::setRigidPose(world, shot.body, shot.from, turn) || !dynamics::setVelocity(world, shot.body, shot.velocity)
        || !dynamics::setAngularVelocity(world, shot.body, glm::dvec3(0.0))) {
      throw std::logic_error("A shot's body refused its pose or velocity");
    }

    ++m_fired;
  }

  m_shots = std::move(waiting);
}

void DestructionWorld::removeAt(uint64_t step, uint32_t building, std::vector<uint32_t> chunks) {
  const Record& record = recordOf(building);

  for (const uint32_t chunk : chunks) {
    if (chunk >= record.chunkPiece.size()) {
      throw std::out_of_range("A building has no chunk with that index");
    }
  }

  m_removals.push_back(Removal{step, building, std::move(chunks)});
}

void DestructionWorld::applyRemovals(dynamics::World& world, DestructionReport& report) {
  std::vector<Removal> waiting;

  for (Removal& removal : m_removals) {
    if (removal.step > m_steps) {
      waiting.push_back(std::move(removal));
      continue;
    }

    report.removed += removeChunks(world, removal.building, removal.chunks);
  }

  m_removals = std::move(waiting);
}

DestructionReport DestructionWorld::update(dynamics::World& world) {
  try {
    return updatePieces(world);
  } catch (...) {
    m_recover = true;
    throw;
  }
}

DestructionReport DestructionWorld::updatePieces(dynamics::World& world) {
  dynamics::Stopwatch clock;

  ++m_steps;

  DestructionReport report;

  report.step = m_steps;
  report.blasted = std::exchange(m_blasted, 0u);
  report.driven = std::exchange(m_driven, 0u);
  report.fired = std::exchange(m_fired, 0u);
  report.removed = std::exchange(m_blastRemoved, 0u);
  report.profile.prepare = std::exchange(m_prepared, 0.0);
  applyRemovals(world, report);
  report.profile.removals = clock.lap();

  const auto count = static_cast<uint32_t>(m_pieces.size());
  const std::vector<uint32_t> index = indexById(world);
  const std::unordered_set<uint64_t> recorded = dynamics::recordedPairs(world.contacts);
  constexpr uint64_t blastPair = 0xFFFFFFFFFFFFFFFFull;
  std::vector<PieceContribution> blasting;

  for (const BlastLoad& load : m_blastLoads) {
    const uint32_t at = m_buildings[load.building].chunkPiece[load.chunk];

    if (at == noPiece || at >= count) {
      continue;
    }

    blasting.push_back(PieceContribution{
      at,
      Contribution{load.chunk, blastPair, load.order, 0u, 0u, 0u, 2u, load.impulse / world.parameters.dt, load.point,
                   load.pendingImpulse}
    });
  }

  m_blastLoads.clear();

  std::vector<uint32_t> blastOrder(blasting.size());

  std::iota(blastOrder.begin(), blastOrder.end(), 0u);
  std::sort(blastOrder.begin(), blastOrder.end(), [&blasting](uint32_t a, uint32_t b) {
    return std::pair(blasting[a].piece, a) < std::pair(blasting[b].piece, b);
  });

  std::vector<PieceContribution> blastsByPiece;

  blastsByPiece.reserve(blasting.size());

  for (const uint32_t slot : blastOrder) {
    blastsByPiece.push_back(blasting[slot]);
  }

  const dynamics::Workers workers =
    dynamics::resolveWorkers(world.workerCount, world.enqueueTask, world.finishTask, world.userTaskContext, world.scheduler);

  countBodies(world, workers, report);
  report.collapse = collapseActive(world, index);
  guardTouched(world);
  const auto countThaws = [&](const std::vector<uint32_t>& bodyIndex) {
    std::erase_if(m_frozen, [&](const std::pair<uint32_t, uint64_t>& entry) {
      const uint32_t id = entry.first;
      const bool present = id < bodyIndex.size() && bodyIndex[id] != noIndex;
      const bool thawed = present && !dynamics::isFrozen(world.bodies.rigids[bodyIndex[id]]);

      report.thawed += thawed ? 1u : 0u;

      return !present || thawed;
    });
  };

  countThaws(index);

  const Visit visit = visitOf(world, workers, index, count, blastsByPiece);
  const auto visits = static_cast<uint32_t>(visit.pieces.size());
  std::vector<std::vector<ChunkLoad>> loads(visits);
  std::vector<std::vector<Contribution>> gathered(visits);
  std::vector<uint8_t> active(visits, 0u);
  std::vector<uint8_t> took(visits, 0u);
  std::vector<uint32_t> refused(visits, 0u);
  std::vector<std::exception_ptr> failures(visits);
  std::vector<Gather> scratch(workers.count);
  std::vector<std::pair<uint32_t, uint32_t>> blastRange(visits);
  size_t blast = 0u;

  report.visited = visits;

  for (uint32_t entry = 0u; entry != visits; ++entry) {
    while (blast != blastsByPiece.size() && blastsByPiece[blast].piece < visit.pieces[entry]) {
      ++blast;
    }

    size_t blastEnd = blast;

    while (blastEnd != blastsByPiece.size() && blastsByPiece[blastEnd].piece == visit.pieces[entry]) {
      ++blastEnd;
    }

    blastRange[entry] = {static_cast<uint32_t>(blast), static_cast<uint32_t>(blastEnd)};
  }

  dynamics::parallelFor(workers, visits, gatherRange, [&](uint32_t begin, uint32_t end, uint32_t worker) {
    for (uint32_t entry = begin; entry != end; ++entry) {
      try {
        const uint32_t at = visit.pieces[entry];
        const auto [firstBlast, endBlast] = blastRange[entry];

        if (!m_pieces[at].alive) {
          continue;
        }

        const uint32_t rigid = rigidIndex(index, m_pieces[at].body);
        const dynamics::RigidBody& body = world.bodies.rigids[rigid];
        const bool resting = visit.all ? still(world, body, at) : visit.moving[entry] == 0u;

        if (resting && quiet(world, body, at) && endBlast == firstBlast) {
          continue;
        }

        m_pieces[at].resting = resting;
        active[entry] = 1u;

        const Piece& piece = m_pieces[at];

        if (anchoredOnly(piece.chunked, piece.structure)
            && !fracturable(m_buildings[piece.building].fine, piece.chunks, piece.structure)) {
          continue;
        }

        gathered[entry] = contributionsOf(world, body, rigid, at, recorded, scratch[worker]);
        refused[entry] = scratch[worker].refused;
        took[entry] = 1u;

        if (endBlast != firstBlast) {
          for (uint32_t slot = firstBlast; slot != endBlast; ++slot) {
            gathered[entry].push_back(blastsByPiece[slot].contribution);
          }

          std::sort(gathered[entry].begin(), gathered[entry].end(), earlier);
        }
      } catch (...) {
        failures[entry] = std::current_exception();
      }
    }
  });
  rethrowFirst(failures);
  report.gathered = static_cast<uint32_t>(std::count(took.begin(), took.end(), uint8_t{1}));
  report.refused = std::accumulate(refused.begin(), refused.end(), 0u);
  report.profile.gather = clock.lap();

  bool appliedImpulse = false;
  std::vector<uint8_t> fromCase(visits, 0u);
  std::vector<glm::dvec3> caseGravity(visits, glm::dvec3(0.0));
  std::vector<uint64_t> caseSince(visits, 0u);
  std::vector<CasePhase> casePhase(visits, CasePhase::Queued);

  for (uint32_t entry = 0u; entry != visits; ++entry) {
    const uint32_t at = visit.pieces[entry];
    Piece& piece = m_pieces[at];

    if (active[entry] == 0u || (!piece.pending && piece.cases.empty())) {
      continue;
    }

    const dynamics::RigidBody& body = world.bodies.rigids[rigidIndex(index, piece.body)];

    queueCase(at, LoadCase{std::move(gathered[entry]), gravityOf(world, body, at), m_steps}, report);
    gathered[entry].clear();

    if (piece.pending) {
      continue;
    }

    LoadCase next = std::move(piece.cases.front());

    piece.cases.erase(piece.cases.begin());
    gathered[entry] = std::move(next.contributions);
    caseGravity[entry] = next.gravity;
    caseSince[entry] = next.since;
    casePhase[entry] = next.phase;
    fromCase[entry] = 1u;
    report.casesStarted += next.phase == CasePhase::Queued ? 1u : 0u;
  }

  std::vector<uint32_t> fractureOrder(visits);

  std::iota(fractureOrder.begin(), fractureOrder.end(), 0u);
  std::sort(fractureOrder.begin(), fractureOrder.end(), [&](uint32_t a, uint32_t b) {
    const uint64_t sinceA = fromCase[a] != 0u ? caseSince[a] : m_steps;
    const uint64_t sinceB = fromCase[b] != 0u ? caseSince[b] : m_steps;

    return std::pair(sinceA, visit.pieces[a]) < std::pair(sinceB, visit.pieces[b]);
  });
  size_t fractureRoom = m_lastPrediction > m_settings.target ? 4u : std::numeric_limits<size_t>::max();

  for (const uint32_t entry : fractureOrder) {
    const uint32_t at = visit.pieces[entry];

    if (active[entry] == 0u || m_pieces[at].pending) {
      continue;
    }

    std::vector<uint32_t> breaking = breakingChunks(at, gathered[entry]);

    if (breaking.empty()) {
      continue;
    }

    const size_t deferred = breaking.size() - std::min(breaking.size(), fractureRoom);

    breaking.resize(breaking.size() - deferred);
    fractureRoom -= breaking.size();

    if (!breaking.empty()) {
      fracturePiece(world, at, breaking);
      gathered[entry] = ontoParts(at, breaking, gathered[entry]);
    }

    for (const uint32_t chunk : breaking) {
      report.fractured.push_back(FracturedChunk{m_pieces[at].building, chunk});
      report.maxBreakDelay = std::max(report.maxBreakDelay, fromCase[entry] != 0u ? m_steps - caseSince[entry] : 0u);
    }

    if (deferred != 0u) {
      const dynamics::RigidBody& body = world.bodies.rigids[rigidIndex(index, m_pieces[at].body)];
      const glm::dvec3 gravity = fromCase[entry] != 0u ? caseGravity[entry] : gravityOf(world, body, at);
      const uint64_t since = fromCase[entry] != 0u ? caseSince[entry] : m_steps;
      const CasePhase phase = casePhase[entry] == CasePhase::Booked ? CasePhase::Booked : CasePhase::Fracturing;

      m_pieces[at].cases.insert(m_pieces[at].cases.begin(), LoadCase{std::move(gathered[entry]), gravity, since, phase});
      report.deferredFractures += static_cast<uint32_t>(deferred);
      report.pending += 1u;
      report.deferred += 1u;
      active[entry] = 0u;
      touch(at);
    }
  }

  report.profile.fracture = clock.lap();

  dynamics::parallelFor(workers, visits, loadRange, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t entry = begin; entry != end; ++entry) {
      if (active[entry] == 0u) {
        continue;
      }

      try {
        loads[entry] = loadsFrom(visit.pieces[entry], gathered[entry]);
      } catch (...) {
        failures[entry] = std::current_exception();
      }
    }
  });
  rethrowFirst(failures);

  for (uint32_t entry = 0u; entry != visits; ++entry) {
    const uint32_t at = visit.pieces[entry];

    if (active[entry] == 0u) {
      continue;
    }

    report.loads += static_cast<uint32_t>(loads[entry].size());

    if (!m_pieces[at].pending && casePhase[entry] != CasePhase::Booked) {
      std::vector<ChunkImpulse>& impulses = m_buildings[m_pieces[at].building].impulses;
      const dynamics::RigidBody& body = world.bodies.rigids[rigidIndex(index, m_pieces[at].body)];
      const Layout layout = layoutOf(body, at);

      for (Contribution& contribution : gathered[entry]) {
        const Record& record = m_buildings[m_pieces[at].building];

        if (contribution.chunk >= record.chunkPiece.size() || record.chunkPiece[contribution.chunk] != at) {
          continue;
        }

        if (contribution.impulse != glm::dvec3(0.0)) {
          if (body.mass > 0.0) {
            static_cast<void>(dynamics::applyImpulse(world, body.id, contribution.impulse,
              body.position + body.rotation * (layout.origin + layout.turn * contribution.point)));
            m_rest.reset(body.id);
            guardPiece(at, m_steps + guardSteps);
            appliedImpulse = true;
            contribution.impulse = glm::dvec3(0.0);
          } else {
            impulses.push_back(ChunkImpulse{contribution.chunk, contribution.impulse, contribution.point});
          }
        }
      }
    }

    if (!m_pieces[at].pending) {
      const dynamics::RigidBody& body = world.bodies.rigids[rigidIndex(index, m_pieces[at].body)];
      const glm::dvec3 gravity = fromCase[entry] != 0u ? caseGravity[entry] : gravityOf(world, body, at);
      const uint64_t since = fromCase[entry] != 0u ? caseSince[entry] : m_steps;

      m_pieces[at].activeCase = LoadCase{std::move(gathered[entry]), gravity, since, CasePhase::Booked};
    }
  }

  report.profile.loads = clock.lap();

  std::vector<uint8_t> queued(visits, 0u);
  std::vector<uint8_t> tolerated(visits, 0u);

  dynamics::parallelFor(workers, visits, queueRange, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t entry = begin; entry != end; ++entry) {
      if (active[entry] == 0u) {
        continue;
      }

      try {
        const uint32_t at = visit.pieces[entry];
        Piece& piece = m_pieces[at];
        const dynamics::RigidBody& body = world.bodies.rigids[rigidIndex(index, piece.body)];

        if (bondless(piece.structure)) {
          piece.pending = false;
          piece.heldLoads.clear();
          solveWithoutBonds(piece.stress, piece.structure, std::move(loads[entry]), gravityOf(world, body, at));
          continue;
        }

        if (!piece.pending) {
          const glm::dvec3 gravity = fromCase[entry] != 0u ? caseGravity[entry] : gravityOf(world, body, at);

          if (falling(body, piece.stress, loads[entry]) || unchanged(piece.stress, piece.structure, loads[entry], gravity)) {
            continue;
          }

          if (m_settings.tolerance && tolerates(piece.stress, piece.structure, loads[entry], gravity)) {
            tolerated[entry] = 1u;
            continue;
          }

          piece.pending = true;
          piece.since = fromCase[entry] != 0u ? caseSince[entry] : m_steps;
          piece.heldLoads = std::move(loads[entry]);
          piece.heldGravity = gravity;
        }

        queued[entry] = 1u;
      } catch (...) {
        failures[entry] = std::current_exception();
      }
    }
  });
  rethrowFirst(failures);

  std::vector<uint32_t> queue;

  report.tolerated = static_cast<uint32_t>(std::count(tolerated.begin(), tolerated.end(), uint8_t{1}));
  m_revisit.clear();

  for (uint32_t entry = 0u; entry != visits; ++entry) {
    if (active[entry] != 0u) {
      m_revisit.push_back(visit.pieces[entry]);
    }

    if (queued[entry] != 0u) {
      queue.push_back(visit.pieces[entry]);
    }
  }

  std::sort(queue.begin(), queue.end(), [this](uint32_t first, uint32_t second) {
    return std::pair(m_pieces[first].since, first) < std::pair(m_pieces[second].since, second);
  });

  solvePieces(workers, queue, report);

  report.profile.stress = clock.lap();

  dynamics::Stopwatch splitting;
  std::vector<uint32_t> checked = visit.pieces;

  for (const uint32_t at : m_touched) {
    if (at < count) {
      checked.push_back(at);
    }
  }

  std::sort(checked.begin(), checked.end());
  checked.erase(std::unique(checked.begin(), checked.end()), checked.end());

  for (const uint32_t at : checked) {
    if (!m_pieces[at].alive) {
      continue;
    }

    const bool changed = !updateComponents(m_pieces[at].structure, m_pieces[at].chunked).empty();

    if (changed || m_pieces[at].unchecked) {
      m_pieces[at].unchecked = false;
      static_cast<void>(splitting.lap());
      split(world, at, report);
      report.profile.splits += splitting.lap();
    }
  }

  report.profile.components = clock.lap() - report.profile.splits;
  m_collapse.insert(m_collapse.end(), report.created.begin(), report.created.end());
  report.collapse = report.collapse || !report.created.empty();

  const std::vector<uint32_t> splitIndex = report.created.empty() ? std::vector<uint32_t>{} : indexById(world);
  const std::vector<uint32_t>& finalIndex = report.created.empty() ? index : splitIndex;

  for (Record& record : m_buildings) {
    std::erase_if(record.impulses, [&](const ChunkImpulse& impulse) {
      const uint32_t at = record.chunkPiece[impulse.chunk];

      if (at == noPiece) {
        return true;
      }

      const Piece& piece = m_pieces[at];
      const bool restarting = !piece.cases.empty() && piece.cases.front().phase == CasePhase::Booked;

      return (!piece.pending && !restarting) || world.bodies.rigids[rigidIndex(finalIndex, piece.body)].mass > 0.0;
    });
  }

  for (Piece& piece : m_pieces) {
    if (!piece.pending) {
      piece.activeCase.reset();
    }
  }

  m_gravity = visit.gravity;
  m_anchorTurns = visit.turns;
  m_touched.erase(m_touched.begin(), m_touched.begin() + static_cast<std::ptrdiff_t>(visit.touched));
  m_recover = false;
  if (!report.created.empty() || !report.fractured.empty() || appliedImpulse) {
    countBodies(world, workers, report, false);
  }
  report.cost = predictCost(stepCounts(report));
  freezeResting(world, report);
  removeFrozen(world, report, finalIndex);
  removeBelow(world, report);

  if (report.removedFrozen != 0u || report.discarded != 0u) {
    countBodies(world, workers, report, false);
    report.cost = predictCost(stepCounts(report));
  }

  if (report.removedFrozen != 0u || report.discarded != 0u) {
    countThaws(indexById(world));
  } else {
    countThaws(finalIndex);
  }
  m_lastPrediction = report.cost.total;

  for (uint32_t at = 0u; at != m_pieces.size(); ++at) {
    report.guarded += m_pieces[at].alive && guarded(at) ? 1u : 0u;

    if (m_pieces[at].alive && m_pieces[at].pending) {
      report.maxPendingAge = std::max(report.maxPendingAge, m_steps - m_pieces[at].since);
    }

    if (m_pieces[at].alive && !m_pieces[at].cases.empty()) {
      report.maxPendingAge = std::max(report.maxPendingAge, m_steps - m_pieces[at].cases.front().since);
    }
  }

  report.profile.finish = clock.lap();
  report.profile.total = report.profile.prepare + clock.total();

  return report;
}

void DestructionWorld::split(dynamics::World& world, uint32_t at, DestructionReport& report) {
  const std::optional<dynamics::BodyRef> ref = dynamics::findBody(world.bodies, m_pieces[at].body);

  if (!ref.has_value() || ref->side != dynamics::Side::Rigid) {
    throw std::logic_error("A destruction piece lost its body");
  }

  if (dynamics::isFrozen(world.bodies.rigids[ref->index])) {
    static_cast<void>(dynamics::thawBody(world, m_pieces[at].body));
  }

  if (world.bodies.rigids[ref->index].mass > 0.0) {
    splitMoving(world, at, report);
  } else {
    splitAnchored(world, at, report);
  }
}

void DestructionWorld::splitAnchored(dynamics::World& world, uint32_t at, DestructionReport& report) {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  std::vector<std::vector<uint32_t>> leaving;

  for (std::vector<uint32_t>& group : componentsOf(piece.structure)) {
    if (!isAnchored(piece.structure, piece.structure.component[group.front()])) {
      leaving.push_back(std::move(group));
    }
  }

  if (leaving.empty()) {
    return;
  }

  std::vector<Piece> fresh;
  std::vector<uint32_t> keys;
  std::vector<uint32_t> locals;

  for (const std::vector<uint32_t>& group : leaving) {
    fresh.push_back(extract(piece, group));

    for (const uint32_t local : group) {
      keys.push_back(record.firstKey + piece.chunks[local]);
      locals.push_back(local);
    }
  }

  std::sort(keys.begin(), keys.end());

  const uint32_t body = piece.body;
  const bool emptied = keys.size() == aliveCount(piece.structure);

  if (emptied) {
    if (!dynamics::removeBody(world, body)) {
      throw std::logic_error("A destruction body could not be removed");
    }
  } else if (!dynamics::removeCompoundChildren(world, body, keys)) {
    throw std::logic_error("A destruction body refused to give up its chunks");
  }

  for (Piece& next : fresh) {
    std::vector<dynamics::CompoundChild> children;

    children.reserve(next.chunks.size());

    for (uint32_t local = 0u; local != next.chunks.size(); ++local) {
      const Chunk& chunk = next.chunked.chunks[local];

      children.push_back(dynamics::CompoundChild{
        chunk.centre,
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        chunk.halfExtents,
        chunk.mass / chunk.volume,
        record.firstKey + next.chunks[local]
      });
    }

    const uint32_t index = dynamics::addCompoundBody(
      world.bodies,
      children,
      record.position,
      record.rotation,
      glm::dvec3(0.0),
      m_settings.friction,
      m_settings.restitution,
      false
    );

    next.body = world.bodies.rigids[index].id;
    report.created.push_back(next.body);

    for (const ChunkImpulse& taken : record.impulses) {
      if (std::binary_search(next.chunks.begin(), next.chunks.end(), taken.chunk)) {
        static_cast<void>(dynamics::applyImpulse(
          world,
          next.body,
          taken.impulse,
          record.position + record.rotation * taken.point
        ));
      }
    }
  }

  forget(at, locals);

  if (emptied) {
    release(at);
  }

  for (Piece& next : fresh) {
    place(std::move(next));
  }
}

void DestructionWorld::splitMoving(dynamics::World& world, uint32_t at, DestructionReport& report) {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  const std::vector<std::vector<uint32_t>> groups = componentsOf(piece.structure);

  if (groups.size() < 2u) {
    return;
  }

  std::vector<std::vector<uint32_t>> keyed;

  for (const std::vector<uint32_t>& group : groups) {
    std::vector<uint32_t> keys;

    for (const uint32_t local : group) {
      keys.push_back(record.firstKey + piece.chunks[local]);
    }

    keyed.push_back(std::move(keys));
  }

  const std::vector<uint32_t> ids = dynamics::splitCompound(world, piece.body, keyed);

  if (ids.size() + 1u != groups.size()) {
    throw std::logic_error("A moving destruction body refused its split");
  }

  const size_t first = world.bodies.rigids.size() - ids.size();
  std::vector<Piece> fresh;
  std::vector<uint32_t> departed;

  for (size_t index = 0u; index != ids.size(); ++index) {
    const dynamics::RigidBody& body = world.bodies.rigids[first + index];

    if (body.id != ids[index]) {
      throw std::logic_error("A split appended its bodies out of order");
    }

    std::vector<uint32_t> locals;

    for (const dynamics::CompoundChild& child : body.compound->children) {
      locals.push_back(record.chunkLocal[child.key - record.firstKey]);
    }

    fresh.push_back(extract(piece, locals));
    fresh.back().body = ids[index];
    departed.insert(departed.end(), locals.begin(), locals.end());
    report.created.push_back(ids[index]);
  }

  forget(at, departed);

  for (Piece& next : fresh) {
    place(std::move(next));
  }
}

DestructionWorld::Piece DestructionWorld::extract(const Piece& parent, std::span<const uint32_t> locals) const {
  Piece piece;
  std::vector<uint32_t> renumber(parent.chunked.chunks.size(), noIndex);
  std::vector<uint32_t> source;

  piece.building = parent.building;

  for (const uint32_t local : locals) {
    renumber[local] = static_cast<uint32_t>(piece.chunks.size());
    piece.chunks.push_back(parent.chunks[local]);
    piece.chunked.chunks.push_back(parent.chunked.chunks[local]);
  }

  for (const uint32_t local : locals) {
    for (uint32_t slot = parent.structure.bondStart[local]; slot != parent.structure.bondStart[local + 1u]; ++slot) {
      const uint32_t bond = parent.structure.bondList[slot];
      const Bond& held = parent.chunked.bonds[bond];

      if (held.first == local && renumber[held.second] != noIndex && parent.structure.broken[bond] == 0u) {
        source.push_back(bond);
      }
    }
  }

  std::sort(source.begin(), source.end());

  for (const uint32_t bond : source) {
    Bond copied = parent.chunked.bonds[bond];

    copied.first = renumber[copied.first];
    copied.second = renumber[copied.second];
    piece.chunked.bonds.push_back(copied);
    piece.bonds.push_back(parent.bonds[bond]);
  }

  piece.structure = makeStructure(piece.chunked);

  for (uint32_t bond = 0u; bond != source.size(); ++bond) {
    piece.structure.health[bond] = parent.structure.health[source[bond]];
    piece.structure.cracked[bond] = parent.structure.cracked[source[bond]];
  }

  piece.stress = makeStress(piece.chunked);

  std::vector<uint32_t> held = piece.chunks;

  std::sort(held.begin(), held.end());

  const auto divide = [&](const LoadCase& queued) {
    LoadCase divided{{}, queued.gravity, queued.since, queued.phase};

    for (const Contribution& contribution : queued.contributions) {
      if (std::binary_search(held.begin(), held.end(), contribution.chunk)) {
        divided.contributions.push_back(contribution);
      }
    }

    piece.cases.push_back(std::move(divided));
  };

  if (parent.pending && parent.activeCase.has_value()) {
    divide(*parent.activeCase);
  }

  for (const LoadCase& queued : parent.cases) {
    divide(queued);
  }

  return piece;
}

DestructionWorld::Layout DestructionWorld::layoutOf(const dynamics::RigidBody& body, uint32_t at) const {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  const dynamics::CompoundChild& child = body.compound->children.front();
  const uint32_t chunk = child.key - record.firstKey;

  if (child.key < record.firstKey || chunk >= record.chunkPiece.size() || record.chunkPiece[chunk] != at) {
    throw std::logic_error("A destruction body holds a chunk its piece does not");
  }

  const glm::dvec3& centre = piece.chunked.chunks[record.chunkLocal[chunk]].centre;

  return Layout{child.rotation, child.centre - child.rotation * centre};
}

DestructionWorld::Visit DestructionWorld::visitOf(
  const dynamics::World& world,
  const dynamics::Workers& workers,
  const std::vector<uint32_t>& index,
  uint32_t count,
  std::span<const PieceContribution> blasting
) {
  const dynamics::IntegrationParameters& parameters = world.parameters;
  const uint64_t stamp = m_steps;
  bool turned = false;
  Visit visit;

  visit.turns = m_anchorTurns;

  for (size_t building = 0u; building != m_anchors.size(); ++building) {
    const Piece& anchor = m_pieces[m_anchors[building]];

    if (!anchor.alive) {
      continue;
    }

    const glm::dquat turn = world.bodies.rigids[rigidIndex(index, anchor.body)].rotation;

    turned = turned || turn != m_anchorTurns[building];
    visit.turns[building] = turn;
  }

  visit.gravity = parameters.gravity;
  visit.touched = m_touched.size();
  visit.all = m_recover || m_settings.visitAll || !parameters.sleep || !(parameters.timeToSleep > parameters.dt) || turned
              || !m_gravity.has_value() || *m_gravity != parameters.gravity;
  m_marked.resize(count, 0u);
  m_moving.resize(count, 0u);

  if (visit.all) {
    for (uint32_t at = 0u; at != count; ++at) {
      if (m_pieces[at].alive) {
        visit.pieces.push_back(at);
      }
    }

    return visit;
  }

  const auto mark = [&](uint32_t at) {
    if (at < count) {
      m_marked[at] = stamp;
    }
  };
  const auto nearOf = [&](uint32_t body, std::vector<uint32_t>& found) {
    for (const uint32_t slot : world.manifolds.pairsOf(body)) {
      if (world.manifolds.live(slot)) {
        const dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

        found.push_back(pieceOf(pair.firstId == body ? pair.secondId : pair.firstId));
      }
    }
  };

  for (const uint32_t at : m_revisit) {
    mark(at);
  }

  for (const uint32_t at : m_touched) {
    mark(at);
  }

  for (const PieceContribution& entry : blasting) {
    mark(entry.piece);
  }

  std::vector<std::vector<uint32_t>> moved(workers.count);
  std::vector<std::exception_ptr> failures(workers.count);

  dynamics::parallelFor(
    workers,
    static_cast<uint32_t>(world.bodies.rigids.size()),
    scanRange,
    [&](uint32_t begin, uint32_t end, uint32_t worker) {
      try {
        for (uint32_t rigid = begin; rigid != end; ++rigid) {
          const dynamics::RigidBody& body = world.bodies.rigids[rigid];

          if (body.mass > 0.0 && !body.asleep) {
            moved[worker].push_back(pieceOf(body.id));
            nearOf(body.id, moved[worker]);
          }
        }
      } catch (...) {
        failures[worker] = std::current_exception();
      }
    }
  );
  rethrowFirst(failures);

  for (const std::vector<uint32_t>& found : moved) {
    for (const uint32_t at : found) {
      if (at < count) {
        m_marked[at] = stamp;
        m_moving[at] = stamp;
      }
    }
  }

  for (uint32_t at = 0u; at != count; ++at) {
    if (m_marked[at] == stamp && m_pieces[at].alive) {
      visit.pieces.push_back(at);
      visit.moving.push_back(m_moving[at] == stamp ? 1u : 0u);
    }
  }

  return visit;
}

bool DestructionWorld::still(const dynamics::World& world, const dynamics::RigidBody& body, uint32_t at) const {
  if (body.mass > 0.0 && !body.asleep) {
    return false;
  }

  for (const uint32_t slot : world.manifolds.pairsOf(m_pieces[at].body)) {
    if (!world.manifolds.live(slot)) {
      continue;
    }

    const dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

    for (const dynamics::BodyRef& end : {pair.first, pair.second}) {
      if (end.side == dynamics::Side::Rigid) {
        const dynamics::RigidBody& other = world.bodies.rigids[end.index];

        if (other.mass > 0.0 && !other.asleep) {
          return false;
        }
      }
    }
  }

  return true;
}

bool DestructionWorld::quiet(const dynamics::World& world, const dynamics::RigidBody& body, uint32_t at) const {
  const Piece& piece = m_pieces[at];
  const StressState& stress = piece.stress;
  const glm::dvec3 gravity = gravityOf(world, body, at);

  if (!piece.resting || piece.pending || !piece.cases.empty() || !stress.solved || stress.eroding
      || stress.revision != piece.structure.revision) {
    return false;
  }

  return stress.gravity == gravity || (m_settings.tolerance && tolerates(stress, piece.structure, stress.loads, gravity));
}

glm::dvec3 DestructionWorld::gravityOf(
  const dynamics::World& world,
  const dynamics::RigidBody& body,
  uint32_t at
) const {
  const Layout layout = layoutOf(body, at);

  return glm::conjugate(layout.turn) * (glm::conjugate(body.rotation) * world.parameters.gravity);
}

bool DestructionWorld::earlier(const Contribution& a, const Contribution& b) {
  return std::tie(a.chunk, a.pair, a.manifold, a.key, a.first, a.second, a.side,
                 a.force.x, a.force.y, a.force.z, a.point.x, a.point.y, a.point.z, a.impulse.x, a.impulse.y, a.impulse.z)
         < std::tie(b.chunk, b.pair, b.manifold, b.key, b.first, b.second, b.side,
                    b.force.x, b.force.y, b.force.z, b.point.x, b.point.y, b.point.z, b.impulse.x, b.impulse.y, b.impulse.z);
}

std::vector<DestructionWorld::Contribution> DestructionWorld::contributionsOf(
  const dynamics::World& world,
  const dynamics::RigidBody& body,
  uint32_t rigid,
  uint32_t at,
  const std::unordered_set<uint64_t>& recorded,
  Gather& gather
) const {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  const Layout layout = layoutOf(body, at);
  const glm::dquat back = glm::conjugate(layout.turn);
  const glm::dquat unturn = glm::conjugate(body.rotation);
  const double rate = 1.0 / world.parameters.dt;
  std::vector<Contribution>& contributions = gather.contributions;
  const auto add = [&](
                        const End& a,
                        const End& b,
                        const std::array<glm::dvec3, 3>& basis,
                        const dynamics::ContactMemory& memory,
                        uint64_t pair,
                        uint32_t manifold,
                        uint64_t key
                      ) {
    if (memory.impulse == 0.0 && memory.friction == glm::dvec2(0.0)) {
      return;
    }

    const glm::dvec3 pushed = (memory.impulse * rate) * basis[0]
                              + memory.friction.x * basis[1]
                              + memory.friction.y * basis[2];

    for (uint32_t side = 0u; side != 2u; ++side) {
      const End& end = side == 0u ? a : b;

      if (end.side != dynamics::Side::Rigid || end.body != rigid || end.child == dynamics::noChild) {
        continue;
      }

      const uint32_t chunk = end.child - record.firstKey;

      if (end.child < record.firstKey || chunk >= record.chunkPiece.size()
          || (record.chunkPiece[chunk] != at && record.chunkPiece[chunk] != noPiece)) {
        throw std::logic_error("A contact names a chunk that its body's piece does not hold");
      }

      if (record.chunkPiece[chunk] == noPiece) {
        continue;
      }

      const glm::dvec3 force = back * (unturn * (side == 0u ? pushed : -pushed));
      const glm::dvec3 point = back * (end.local - layout.origin);

      if (!finitePoint(force) || !finitePoint(point)) {
        ++gather.refused;
        continue;
      }

      contributions.push_back(Contribution{chunk, pair, manifold, key, a.child, b.child, side, force, point, glm::dvec3(0.0)});
    }
  };

  contributions.clear();
  gather.refused = 0u;

  for (const dynamics::Contact& contact : world.contacts) {
    if ((contact.a.side != dynamics::Side::Rigid || contact.a.body != rigid)
        && (contact.b.side != dynamics::Side::Rigid || contact.b.body != rigid)) {
      continue;
    }

    add(
      End{contact.a.side, contact.a.body, contact.a.child, contact.a.local},
      End{contact.b.side, contact.b.body, contact.b.child, contact.b.local},
      contact.basis,
      contact.memory,
      contact.pair,
      contact.manifold,
      contact.key
    );
  }

  for (const uint32_t slot : world.manifolds.pairsOf(body.id)) {
    const uint64_t pair = world.manifolds.at(slot).pair;

    if (!recorded.empty() && recorded.contains(pair)) {
      continue;
    }

    world.manifolds.visitPoints(slot, [&](const dynamics::CachedPointView& view) {
      add(
        End{view.a.side, view.a.index, view.childA, view.point.localA},
        End{view.b.side, view.b.index, view.childB, view.point.localB},
        view.manifold.basis,
        view.source.memory,
        pair,
        view.index,
        view.point.key
      );
    });
  }

  gather.order.resize(contributions.size());
  std::iota(gather.order.begin(), gather.order.end(), 0u);
  std::sort(gather.order.begin(), gather.order.end(), [&](uint32_t left, uint32_t right) {
    const Contribution& first = contributions[left];
    const Contribution& second = contributions[right];

    if (earlier(first, second)) {
      return true;
    }

    return !earlier(second, first) && left < right;
  });

  std::vector<Contribution> sorted;

  sorted.reserve(contributions.size());

  for (const uint32_t slot : gather.order) {
    sorted.push_back(contributions[slot]);
  }

  return sorted;
}

std::vector<ChunkLoad> DestructionWorld::loadsFrom(uint32_t at, std::span<const Contribution> contributions) const {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  std::vector<ChunkLoad> loads;

  for (const Contribution& contribution : contributions) {
    if (record.chunkPiece[contribution.chunk] != at) {
      continue;
    }

    const uint32_t local = record.chunkLocal[contribution.chunk];
    const Chunk& held = piece.chunked.chunks[local];

    if (held.anchored) {
      continue;
    }

    if (loads.empty() || loads.back().chunk != local) {
      loads.push_back(ChunkLoad{local, glm::dvec3(0.0), glm::dvec3(0.0)});
    }

    loads.back().force += contribution.force;
    loads.back().torque += glm::cross(contribution.point - held.centre, contribution.force);
  }

  return loads;
}

std::vector<uint32_t> DestructionWorld::breakingChunks(uint32_t at, std::span<const Contribution> contributions) const {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());
  std::vector<uint32_t> breaking;

  for (size_t begin = 0u; begin != contributions.size();) {
    const uint32_t chunk = contributions[begin].chunk;
    glm::dvec3 force(0.0);
    size_t end = begin;

    while (end != contributions.size() && contributions[end].chunk == chunk) {
      force += contributions[end].force;
      ++end;
    }

    begin = end;

    if (chunk >= original || record.fine.start[chunk] == record.fine.start[chunk + 1u] || record.chunkPiece[chunk] != at) {
      continue;
    }

    const double size = glm::length(force);

    if (!(size > 0.0)) {
      continue;
    }

    const Chunk& held = piece.chunked.chunks[record.chunkLocal[chunk]];
    const double limit =
      fractureLoad(record.materials[held.material], record.source.members[held.member], held.halfExtents, force / size);

    if (size > limit) {
      breaking.push_back(chunk);
    }
  }

  return breaking;
}

std::vector<DestructionWorld::Contribution> DestructionWorld::ontoParts(
  uint32_t at,
  std::span<const uint32_t> fractured,
  std::span<const Contribution> contributions
) const {
  const Piece& piece = m_pieces[at];
  const Record& record = m_buildings[piece.building];
  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());
  std::vector<Contribution> moved;

  for (const Contribution& contribution : contributions) {
    if (!std::binary_search(fractured.begin(), fractured.end(), contribution.chunk)) {
      moved.push_back(contribution);
      continue;
    }

    const uint32_t first = original + record.fine.start[contribution.chunk];
    const uint32_t end = original + record.fine.start[contribution.chunk + 1u];
    std::vector<double> distances;
    double nearest = std::numeric_limits<double>::infinity();
    double size = 0.0;

    for (uint32_t id = first; id != end; ++id) {
      const Chunk& part = piece.chunked.chunks[record.chunkLocal[id]];
      const glm::dvec3 outside = glm::max(glm::abs(contribution.point - part.centre) - part.halfExtents, glm::dvec3(0.0));

      distances.push_back(glm::length(outside));
      nearest = std::min(nearest, distances.back());
      size = std::max(size, glm::length(part.halfExtents));
    }

    std::vector<uint32_t> chosen;

    for (uint32_t id = first; id != end; ++id) {
      if (distances[id - first] <= nearest + 1.0e-9 * size) {
        chosen.push_back(id);
      }
    }

    const double share = 1.0 / static_cast<double>(chosen.size());

    for (const uint32_t id : chosen) {
      Contribution part = contribution;

      part.chunk = id;
      part.force = share * contribution.force;
      part.impulse = share * contribution.impulse;
      moved.push_back(part);
    }
  }

  std::sort(moved.begin(), moved.end(), earlier);

  return moved;
}

void DestructionWorld::forget(uint32_t at, std::span<const uint32_t> locals) {
  Piece& piece = m_pieces[at];

  for (const uint32_t local : locals) {
    removeChunk(piece.structure, piece.chunked, local);
  }

  touch(at);
}

StressSettings DestructionWorld::stressSettings(const glm::dvec3& gravity, uint32_t rounds) {
  StressSettings settings;

  settings.gravity = gravity;
  settings.roundsPerCall = rounds;

  return settings;
}

void DestructionWorld::setSettings(const DestructionSettings& settings) {
  m_settings = checked(settings);
}

const DestructionSettings& DestructionWorld::settings() const noexcept {
  return m_settings;
}

uint64_t DestructionWorld::steps() const noexcept {
  return m_steps;
}

uint32_t DestructionWorld::buildings() const noexcept {
  return static_cast<uint32_t>(m_buildings.size());
}

uint32_t DestructionWorld::keys() const noexcept {
  return m_nextKey;
}

const DestructionWorld::Record& DestructionWorld::recordOf(uint32_t building) const {
  if (building >= m_buildings.size()) {
    throw std::out_of_range("A destruction world has no building with that index");
  }

  return m_buildings[building];
}

uint32_t DestructionWorld::firstKey(uint32_t building) const {
  return recordOf(building).firstKey;
}

const ChunkedBuilding& DestructionWorld::chunked(uint32_t building) const {
  return recordOf(building).chunked;
}

std::optional<uint32_t> DestructionWorld::bodyOf(uint32_t building, uint32_t chunk) const {
  const Record& record = recordOf(building);

  if (chunk >= record.chunkPiece.size()) {
    throw std::out_of_range("A building has no chunk with that index");
  }

  const uint32_t piece = record.chunkPiece[chunk];

  if (piece == noPiece) {
    return std::nullopt;
  }

  return m_pieces[piece].body;
}

BondState DestructionWorld::bondState(uint32_t building, uint32_t bond) const {
  const Record& record = recordOf(building);

  if (bond >= record.bondPiece.size()) {
    throw std::out_of_range("A building has no bond with that index");
  }

  if (record.retired[bond] != 0u) {
    return BondState::Replaced;
  }

  const uint32_t piece = record.bondPiece[bond];

  if (piece == noPiece) {
    return BondState::Broken;
  }

  const StructureState& state = m_pieces[piece].structure;
  const uint32_t local = record.bondLocal[bond];

  if (state.broken[local] != 0u) {
    return BondState::Broken;
  }

  return state.cracked[local] != 0u ? BondState::Cracked : BondState::Intact;
}

double DestructionWorld::bondHealth(uint32_t building, uint32_t bond) const {
  const Record& record = recordOf(building);

  if (bond >= record.bondPiece.size()) {
    throw std::out_of_range("A building has no bond with that index");
  }

  const uint32_t piece = record.bondPiece[bond];

  return piece == noPiece ? 0.0 : m_pieces[piece].structure.health[record.bondLocal[bond]];
}

std::vector<ChunkLoad> DestructionWorld::chunkLoads(uint32_t building) const {
  static_cast<void>(recordOf(building));

  std::vector<ChunkLoad> loads;

  for (const Piece& piece : m_pieces) {
    if (!piece.alive || piece.building != building) {
      continue;
    }

    for (const ChunkLoad& load : piece.stress.loads) {
      loads.push_back(ChunkLoad{piece.chunks[load.chunk], load.force, load.torque});
    }
  }

  std::sort(loads.begin(), loads.end(), [](const ChunkLoad& a, const ChunkLoad& b) {
    return a.chunk < b.chunk;
  });

  return loads;
}

glm::dvec3 DestructionWorld::bondForce(uint32_t building, uint32_t bond) const {
  const Record& record = recordOf(building);

  if (bond >= record.bondPiece.size()) {
    throw std::out_of_range("A building has no bond with that index");
  }

  const uint32_t piece = record.bondPiece[bond];

  return piece == noPiece ? glm::dvec3(0.0) : m_pieces[piece].stress.force[record.bondLocal[bond]];
}

glm::dvec3 DestructionWorld::bondMoment(uint32_t building, uint32_t bond) const {
  const Record& record = recordOf(building);

  if (bond >= record.bondPiece.size()) {
    throw std::out_of_range("A building has no bond with that index");
  }

  const uint32_t piece = record.bondPiece[bond];

  return piece == noPiece ? glm::dvec3(0.0) : m_pieces[piece].stress.moment[record.bondLocal[bond]];
}

uint32_t DestructionWorld::chunkCount(uint32_t building) const {
  return static_cast<uint32_t>(recordOf(building).chunkPiece.size());
}

Chunk DestructionWorld::chunkOf(uint32_t building, uint32_t chunk) const {
  const Record& record = recordOf(building);
  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());

  if (chunk >= record.chunkPiece.size()) {
    throw std::out_of_range("A building has no chunk with that index");
  }

  if (chunk < original) {
    return record.chunked.chunks[chunk];
  }

  const uint32_t parent = parentOf(record.fine, chunk - original);
  const std::vector<Chunk> parts =
    finerChunks(record.source, record.materials, record.chunked.chunks[parent], record.fine.cells[parent]);

  return parts[chunk - original - record.fine.start[parent]];
}

uint32_t DestructionWorld::materialOf(uint32_t building, uint32_t chunk) const {
  const Record& record = recordOf(building);
  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());

  if (chunk >= record.chunkPiece.size()) {
    throw std::out_of_range("A building has no chunk with that index");
  }

  return record.chunked.chunks[chunk < original ? chunk : parentOf(record.fine, chunk - original)].material;
}

std::pair<uint32_t, uint32_t> DestructionWorld::partsOf(uint32_t building, uint32_t chunk) const {
  const Record& record = recordOf(building);
  const auto original = static_cast<uint32_t>(record.chunked.chunks.size());

  if (chunk >= record.chunkPiece.size()) {
    throw std::out_of_range("A building has no chunk with that index");
  }

  if (chunk >= original) {
    return {chunk, chunk};
  }

  return {original + record.fine.start[chunk], original + record.fine.start[chunk + 1u]};
}

uint32_t DestructionWorld::bondCount(uint32_t building) const {
  const Record& record = recordOf(building);

  return static_cast<uint32_t>(record.chunked.bonds.size() + record.made.size());
}

Bond DestructionWorld::bondOf(uint32_t building, uint32_t bond) const {
  const Record& record = recordOf(building);
  const auto original = static_cast<uint32_t>(record.chunked.bonds.size());

  if (bond >= original + record.made.size()) {
    throw std::out_of_range("A building has no bond with that index");
  }

  return bond < original ? record.chunked.bonds[bond] : record.made[bond - original];
}

std::vector<uint32_t> DestructionWorld::bodies() const {
  std::vector<uint32_t> found;

  for (const Piece& piece : m_pieces) {
    if (piece.alive) {
      found.push_back(piece.body);
    }
  }

  return found;
}

}
