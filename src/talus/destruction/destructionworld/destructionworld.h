#pragma once

#include <talus/destruction/blast/blast.h>
#include <talus/destruction/budget/budget.h>
#include <talus/destruction/building/building.h>
#include <talus/destruction/chunker/chunker.h>
#include <talus/destruction/materials/materials.h>
#include <talus/destruction/stress/stress.h>
#include <talus/destruction/structure/structure.h>
#include <talus/dynamics/pipeline/pipeline.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <unordered_set>
#include <utility>
#include <vector>

namespace talus::destruction {

struct DestructionSettings {
  double target = budgetTarget;
  double ceiling = collapseCeiling;
  uint32_t wideChunks = 512u;
  double removalFloor = 1.0;
  double discardBelow = -std::numeric_limits<double>::infinity();
  bool activeSet = true;
  bool tolerance = true;
  uint32_t settleUpdates = 10000u;
  double friction = 0.6;
  double restitution = 0.0;
  bool visitAll = false;
};

struct BuildingReport {
  uint32_t building = 0u;
  uint32_t firstKey = 0u;
  uint32_t body = 0u;
  uint32_t updates = 0u;
  uint32_t rounds = 0u;
  uint32_t factorizations = 0u;
  bool settled = false;
  std::vector<uint32_t> broken;
  std::vector<uint32_t> cracked;
};

enum class BondState : uint8_t { Intact, Cracked, Broken, Replaced };

struct BrokenBond {
  uint32_t building = 0u;
  uint32_t bond = 0u;

  friend bool operator==(const BrokenBond&, const BrokenBond&) = default;
};

struct FracturedChunk {
  uint32_t building = 0u;
  uint32_t chunk = 0u;

  friend bool operator==(const FracturedChunk&, const FracturedChunk&) = default;
};

struct PathPoint {
  uint64_t step = 0u;
  glm::dvec3 position{0.0};
};

struct Drive {
  uint32_t body = 0u;
  std::vector<PathPoint> path;
};

struct Shot {
  uint32_t body = 0u;
  uint64_t step = 0u;
  glm::dvec3 from{0.0};
  glm::dvec3 velocity{0.0};
};

struct DestructionProfile {
  double prepare = 0.0;
  double removals = 0.0;
  double gather = 0.0;
  double fracture = 0.0;
  double loads = 0.0;
  double stress = 0.0;
  double factorizing = 0.0;
  double components = 0.0;
  double splits = 0.0;
  double finish = 0.0;
  double total = 0.0;
};

struct DestructionReport {
  uint64_t step = 0u;
  uint32_t removed = 0u;
  uint32_t visited = 0u;
  uint32_t gathered = 0u;
  uint32_t refused = 0u;
  uint32_t loads = 0u;
  uint32_t solved = 0u;
  uint32_t tolerated = 0u;
  uint32_t rounds = 0u;
  uint32_t factorizations = 0u;
  uint32_t solvedChunks = 0u;
  uint32_t factorizedChunks = 0u;
  uint64_t factorWork = 0u;
  uint64_t wideWork = 0u;
  uint32_t pending = 0u;
  uint32_t wide = 0u;
  uint32_t casesQueued = 0u;
  uint32_t casesReplaced = 0u;
  uint32_t casesMerged = 0u;
  uint32_t casesStarted = 0u;
  uint32_t deferred = 0u;
  uint32_t overRounds = 0u;
  double overWork = 0.0;
  double room = 0.0;
  double roundWork = 0.0;
  bool collapse = false;
  uint32_t frozen = 0u;
  uint32_t thawed = 0u;
  uint32_t guarded = 0u;
  uint32_t removedFrozen = 0u;
  uint32_t discarded = 0u;
  uint32_t deferredFractures = 0u;
  uint64_t maxPendingAge = 0u;
  uint64_t maxBreakDelay = 0u;
  uint32_t blasted = 0u;
  uint32_t driven = 0u;
  uint32_t fired = 0u;
  uint32_t moving = 0u;
  uint32_t awakeChunks = 0u;
  uint32_t bodies = 0u;
  uint32_t awakeManifolds = 0u;
  std::vector<BrokenBond> broken;
  std::vector<uint32_t> created;
  std::vector<FracturedChunk> fractured;
  StepCost cost;
  DestructionProfile profile;
};

[[nodiscard]] StepCounts stepCounts(const DestructionReport& report);

class DestructionWorld {
public:
  explicit DestructionWorld(const DestructionSettings& settings = {});

  [[nodiscard]] BuildingReport addBuilding(
    dynamics::World& world,
    const Building& building,
    std::span<const StructuralMaterial> materials,
    const ChunkerSettings& chunker = {}
  );

  [[nodiscard]] uint32_t removeChunks(
    dynamics::World& world,
    uint32_t building,
    std::span<const uint32_t> chunks
  );

  [[nodiscard]] uint32_t removeChunksAt(dynamics::World& world, const glm::dvec3& point);

  void removeAt(uint64_t step, uint32_t building, std::vector<uint32_t> chunks);

  [[nodiscard]] uint32_t fractureChunks(
    dynamics::World& world,
    uint32_t building,
    std::span<const uint32_t> chunks
  );

  void blastAt(const Blast& blast);

  void prepare(dynamics::World& world);

  void drive(const Drive& scripted);

  void fire(const Shot& shot);

  [[nodiscard]] double drivenWork() const noexcept;

  [[nodiscard]] DestructionReport update(dynamics::World& world);

  void setSettings(const DestructionSettings& settings);

  [[nodiscard]] const DestructionSettings& settings() const noexcept;
  [[nodiscard]] uint64_t steps() const noexcept;
  [[nodiscard]] uint32_t buildings() const noexcept;
  [[nodiscard]] uint32_t keys() const noexcept;
  [[nodiscard]] uint32_t firstKey(uint32_t building) const;
  [[nodiscard]] const ChunkedBuilding& chunked(uint32_t building) const;
  [[nodiscard]] std::optional<uint32_t> bodyOf(uint32_t building, uint32_t chunk) const;
  [[nodiscard]] BondState bondState(uint32_t building, uint32_t bond) const;
  [[nodiscard]] double bondHealth(uint32_t building, uint32_t bond) const;
  [[nodiscard]] std::vector<uint32_t> bodies() const;
  [[nodiscard]] std::vector<ChunkLoad> chunkLoads(uint32_t building) const;
  [[nodiscard]] glm::dvec3 bondForce(uint32_t building, uint32_t bond) const;
  [[nodiscard]] glm::dvec3 bondMoment(uint32_t building, uint32_t bond) const;
  [[nodiscard]] uint32_t chunkCount(uint32_t building) const;
  [[nodiscard]] Chunk chunkOf(uint32_t building, uint32_t chunk) const;
  [[nodiscard]] uint32_t materialOf(uint32_t building, uint32_t chunk) const;
  [[nodiscard]] std::pair<uint32_t, uint32_t> partsOf(uint32_t building, uint32_t chunk) const;
  [[nodiscard]] uint32_t bondCount(uint32_t building) const;
  [[nodiscard]] Bond bondOf(uint32_t building, uint32_t bond) const;
  [[nodiscard]] const RestTracker& rest() const noexcept;
  [[nodiscard]] std::vector<std::vector<ChunkLoad>> queuedLoads(uint32_t body) const;
  [[nodiscard]] bool guardedBody(uint32_t body) const;

private:
  struct Contribution {
    uint32_t chunk = 0u;
    uint64_t pair = 0u;
    uint32_t manifold = 0u;
    uint64_t key = 0u;
    uint32_t first = 0u;
    uint32_t second = 0u;
    uint32_t side = 0u;
    glm::dvec3 force{0.0};
    glm::dvec3 point{0.0};
    glm::dvec3 impulse{0.0};
  };

  struct ChunkImpulse {
    uint32_t chunk = 0u;
    glm::dvec3 impulse{0.0};
    glm::dvec3 point{0.0};
  };

  enum class CasePhase : uint8_t { Queued, Fracturing, Booked };

  struct LoadCase {
    std::vector<Contribution> contributions;
    glm::dvec3 gravity{0.0};
    uint64_t since = 0u;
    CasePhase phase = CasePhase::Queued;
  };

  struct Piece {
    uint32_t building = 0u;
    uint32_t body = 0u;
    bool alive = true;
    bool unchecked = false;
    bool resting = false;
    bool pending = false;
    uint64_t since = 0u;
    glm::dvec3 heldGravity{0.0};
    std::vector<ChunkLoad> heldLoads;
    std::vector<LoadCase> cases;
    std::optional<LoadCase> activeCase;
    std::vector<uint32_t> chunks;
    std::vector<uint32_t> bonds;
    ChunkedBuilding chunked;
    StructureState structure;
    StressState stress;
  };

  struct Record {
    uint32_t firstKey = 0u;
    glm::dvec3 position{0.0};
    glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
    Building source;
    std::vector<StructuralMaterial> materials;
    ChunkerSettings chunker;
    FinerLevel fine;
    std::vector<Bond> made;
    std::vector<uint8_t> retired;
    std::vector<ChunkImpulse> impulses;
    ChunkedBuilding chunked;
    std::vector<uint32_t> chunkPiece;
    std::vector<uint32_t> chunkLocal;
    std::vector<uint32_t> bondPiece;
    std::vector<uint32_t> bondLocal;
  };

  struct Removal {
    uint64_t step = 0u;
    uint32_t building = 0u;
    std::vector<uint32_t> chunks;
  };

  struct Layout {
    glm::dquat turn{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 origin{0.0};
  };

  struct BlastLoad {
    uint32_t building = 0u;
    uint32_t chunk = 0u;
    uint32_t order = 0u;
    glm::dvec3 impulse{0.0};
    glm::dvec3 point{0.0};
    glm::dvec3 pendingImpulse{0.0};
  };

  struct Gather {
    std::vector<Contribution> contributions;
    std::vector<uint32_t> order;
    uint32_t refused = 0u;
  };

  struct PieceContribution {
    uint32_t piece = 0u;
    Contribution contribution;
  };

  struct Visit {
    std::vector<uint32_t> pieces;
    std::vector<uint8_t> moving;
    bool all = false;
    glm::dvec3 gravity{0.0};
    std::vector<glm::dquat> turns;
    size_t touched = 0u;
  };

  [[nodiscard]] const Record& recordOf(uint32_t building) const;
  [[nodiscard]] DestructionReport updatePieces(dynamics::World& world);
  [[nodiscard]] Visit visitOf(
    const dynamics::World& world,
    const dynamics::Workers& workers,
    const std::vector<uint32_t>& index,
    uint32_t count,
    std::span<const PieceContribution> blasting
  );
  [[nodiscard]] uint32_t pieceOf(uint32_t body) const;
  void refreshIndex(const dynamics::World& world);
  [[nodiscard]] bool listAwake(const dynamics::World& world);
  void countBodies(const dynamics::World& world, const dynamics::Workers& workers, DestructionReport& report, bool updateRest = true);
  [[nodiscard]] bool collapseActive(const dynamics::World& world, const std::vector<uint32_t>& index);
  void guardPiece(uint32_t at, uint64_t until);
  [[nodiscard]] bool guarded(uint32_t at) const;
  void guardTouched(const dynamics::World& world);
  void freezeResting(dynamics::World& world, DestructionReport& report);
  void removeFrozen(dynamics::World& world, DestructionReport& report, const std::vector<uint32_t>& index);
  void removeBelow(dynamics::World& world, DestructionReport& report);
  void queueCase(uint32_t at, LoadCase next, DestructionReport& report);
  [[nodiscard]] double roundBound(uint32_t at) const;
  void solvePieces(
    const dynamics::Workers& workers,
    const std::vector<uint32_t>& queue,
    DestructionReport& report
  );
  void touch(uint32_t at);
  void release(uint32_t at);
  [[nodiscard]] static StressSettings stressSettings(const glm::dvec3& gravity, uint32_t rounds);
  void place(Piece piece);
  void applyRemovals(dynamics::World& world, DestructionReport& report);
  void applyDrives(dynamics::World& world, uint64_t next);
  void applyShots(dynamics::World& world, uint64_t next);
  void split(dynamics::World& world, uint32_t at, DestructionReport& report);
  void splitAnchored(dynamics::World& world, uint32_t at, DestructionReport& report);
  void splitMoving(dynamics::World& world, uint32_t at, DestructionReport& report);
  [[nodiscard]] Piece extract(const Piece& parent, std::span<const uint32_t> locals) const;
  void forget(uint32_t at, std::span<const uint32_t> locals);
  void fracturePiece(dynamics::World& world, uint32_t at, std::span<const uint32_t> chunks);
  [[nodiscard]] Layout layoutOf(const dynamics::RigidBody& body, uint32_t at) const;
  [[nodiscard]] bool still(const dynamics::World& world, const dynamics::RigidBody& body, uint32_t at) const;
  [[nodiscard]] bool quiet(const dynamics::World& world, const dynamics::RigidBody& body, uint32_t at) const;
  [[nodiscard]] glm::dvec3 gravityOf(const dynamics::World& world, const dynamics::RigidBody& body, uint32_t at) const;
  [[nodiscard]] std::vector<Contribution> contributionsOf(
    const dynamics::World& world,
    const dynamics::RigidBody& body,
    uint32_t rigid,
    uint32_t at,
    const std::unordered_set<uint64_t>& recorded,
    Gather& gather
  ) const;
  [[nodiscard]] std::vector<ChunkLoad> loadsFrom(uint32_t at, std::span<const Contribution> contributions) const;
  [[nodiscard]] std::vector<uint32_t> breakingChunks(uint32_t at, std::span<const Contribution> contributions) const;
  [[nodiscard]] std::vector<Contribution> ontoParts(
    uint32_t at,
    std::span<const uint32_t> fractured,
    std::span<const Contribution> contributions
  ) const;
  [[nodiscard]] static bool earlier(const Contribution& a, const Contribution& b);

  DestructionSettings m_settings;
  std::vector<Record> m_buildings;
  std::vector<Piece> m_pieces;
  std::vector<Removal> m_removals;
  std::vector<Blast> m_blasts;
  std::vector<BlastLoad> m_blastLoads;
  uint32_t m_blasted = 0u;
  uint32_t m_blastRemoved = 0u;
  double m_prepared = 0.0;
  std::vector<Drive> m_drives;
  std::vector<Shot> m_shots;
  double m_drivenWork = 0.0;
  uint32_t m_driven = 0u;
  uint32_t m_fired = 0u;
  uint32_t m_nextKey = 0u;
  uint64_t m_steps = 0u;
  std::vector<uint32_t> m_pieceOfBody;
  std::vector<uint32_t> m_revisit;
  std::vector<uint32_t> m_touched;
  std::vector<uint64_t> m_marked;
  std::vector<uint64_t> m_moving;
  std::vector<uint32_t> m_anchors;
  std::vector<glm::dquat> m_anchorTurns;
  std::optional<glm::dvec3> m_gravity;
  bool m_recover = false;
  RestTracker m_rest;
  std::vector<uint32_t> m_collapse;
  std::vector<uint64_t> m_guard;
  std::vector<uint32_t> m_tools;
  std::vector<std::pair<uint32_t, uint64_t>> m_frozen;
  double m_lastPrediction = 0.0;
  std::vector<uint32_t> m_index;
  uint64_t m_indexLayout = 0u;
  size_t m_indexed = 0u;
  std::vector<uint32_t> m_awake;
  bool m_rescan = false;
};

}
