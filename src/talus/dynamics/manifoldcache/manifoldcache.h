#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace talus::dynamics {

class JointSet;

inline constexpr uint32_t colourCount = 24u;
inline constexpr uint32_t overflowColour = colourCount - 1u;
inline constexpr uint32_t dynamicColourCount = colourCount - 4u;
inline constexpr uint32_t noColour = 0xFFFFFFFFu;
inline constexpr uint32_t noSlot = 0xFFFFFFFFu;
inline constexpr uint32_t noBody = 0xFFFFFFFFu;
inline constexpr double sharedNormal = 0.99;

struct CachedContactPoint {
  glm::vec3 anchorA{0.0f};
  glm::vec3 anchorB{0.0f};
  double separation = 0.0;
  ContactMemory memory;
};

struct CachedManifoldPoint {
  uint64_t key = 0u;
  glm::dvec3 localA{0.0};
  glm::dvec3 localB{0.0};
  double builtSeparation = 0.0;
  uint32_t firstChild = noChild;
  uint32_t secondChild = noChild;
};

struct CachedManifold {
  std::array<glm::dvec3, 3> basis{
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(1.0, 0.0, 0.0),
    glm::dvec3(0.0, 0.0, 1.0)
  };
  bool flipped = false;
  uint32_t start = 0u;
  uint32_t count = 0u;
  double twist = 0.0;
};

struct CachedBodyPair {
  BodyRef first;
  BodyRef second;
  uint32_t firstId = noBody;
  uint32_t secondId = noBody;
  uint64_t pair = 0u;
  double friction = 0.0;
  double restitution = 0.0;
  uint32_t colour = noColour;
  uint32_t colourIndex = noSlot;
  uint8_t marked = 0u;
  bool wide = false;
  bool live = false;
  std::vector<CachedManifold> manifolds;
  std::vector<CachedContactPoint> points;
  CachedPose pose;
  std::vector<CachedManifoldPoint> manifoldPoints;
  std::vector<CachedContactPoint> spare;
  std::vector<CachedManifoldPoint> spareManifoldPoints;
  std::vector<CachedManifold> spareManifolds;
};

[[nodiscard]] inline bool fitsWideLane(const CachedBodyPair& pair) {
  return pair.manifolds.size() == 1u && !pair.points.empty() && pair.points.size() <= 4u;
}

struct CachedPointView {
  const BodyRef& a;
  const BodyRef& b;
  uint32_t childA = noChild;
  uint32_t childB = noChild;
  uint32_t index = 0u;
  const CachedManifold& manifold;
  const CachedContactPoint& source;
  const CachedManifoldPoint& point;
};

class PairSlots {
public:
  [[nodiscard]] uint32_t find(uint64_t key) const;
  void insert(uint64_t key, uint32_t slot);
  void erase(uint64_t key);

private:
  static constexpr uint64_t empty = ~uint64_t{0};

  [[nodiscard]] size_t home(uint64_t key) const;
  void grow();

  std::vector<uint64_t> m_keys;
  std::vector<uint32_t> m_slots;
  size_t m_count = 0u;
};

class ManifoldCache {
public:
  void sync(
    const BodySet& bodies,
    std::span<const BodyPair> pairs,
    const Workers& workers = Workers{}
  );

  [[nodiscard]] uint32_t size() const;
  [[nodiscard]] bool live(uint32_t slot) const;
  [[nodiscard]] uint32_t find(uint64_t pair) const;
  [[nodiscard]] CachedBodyPair& at(uint32_t slot);
  [[nodiscard]] const CachedBodyPair& at(uint32_t slot) const;
  [[nodiscard]] std::span<const uint32_t> pairsOf(uint32_t id) const;
  [[nodiscard]] bool touchedLast(uint32_t id) const;
  [[nodiscard]] std::span<const uint32_t> touchedIndices() const;
  void keepTouched(const BodySet& bodies, std::span<const uint32_t> indices);
  [[nodiscard]] uint32_t pointCount() const;

  [[nodiscard]] std::vector<Contact> records(uint32_t slot) const;

  template <typename Visit>
  void visitPoints(uint32_t slot, Visit&& visit) const;

  void rebuildManifolds(
    const BodySet& bodies,
    uint32_t slot,
    std::span<const Contact> records
  );
  void replaceManifolds(
    const BodySet& bodies,
    uint32_t slot,
    std::span<const Contact> records
  );
  void writeMemory(uint32_t slot, std::span<const Contact> records);
  void refreshColour(const BodySet& bodies, uint32_t slot);
  void recolour(const BodySet& bodies, std::span<const uint32_t> slots);
  [[nodiscard]] std::vector<uint32_t> syncBodies(
    const BodySet& bodies,
    const Workers& workers = Workers{},
    const std::vector<uint32_t>* live = nullptr
  );
  void sleepBody(const BodySet& bodies, uint32_t id);
  [[nodiscard]] std::span<const uint32_t> widePairs(uint32_t colour) const;
  [[nodiscard]] std::span<const uint32_t> otherPairs(uint32_t colour) const;
  void touchingPairs(std::vector<uint32_t>& slots) const;

  void createPairs(
    const BodySet& bodies,
    std::span<const BodyPair> pairs,
    const JointSet* joints = nullptr
  );
  void forgetPair(uint32_t idA, uint32_t idB);
  void destroyPair(uint32_t slot);
  [[nodiscard]] uint32_t dropChildren(
    const BodySet& bodies,
    uint32_t slot,
    uint32_t id,
    std::span<const uint32_t> keys,
    bool reframed
  );
  void removeBody(uint32_t id);
  void renumber(Side side, uint32_t removed);
  void renumber(Side side, std::span<const uint32_t> moved);

private:
  uint32_t create(const BodySet& bodies, const BodyPair& pair, uint64_t key);
  void remove(uint32_t slot);
  void fill(uint32_t slot, std::span<const Contact> records, bool carry);
  void releaseColour(uint32_t slot);
  void assignColour(const BodySet& bodies, uint32_t slot);
  void enlist(uint32_t slot, bool wide);
  void delist(uint32_t slot);
  [[nodiscard]] bool usedBy(uint32_t colour, uint32_t id) const;
  void mark(uint32_t colour, uint32_t id, bool used);

  std::vector<CachedBodyPair> m_pairs;
  std::vector<uint32_t> m_free;
  PairSlots m_slot;
  std::vector<std::vector<uint32_t>> m_byBody;
  std::array<std::vector<uint8_t>, overflowColour> m_used;
  std::array<std::vector<uint32_t>, colourCount> m_wide;
  std::array<std::vector<uint32_t>, colourCount> m_other;
  std::vector<uint8_t> m_bodyState;
  std::vector<uint8_t> m_seen;
  std::vector<uint8_t> m_touched;
  std::vector<uint32_t> m_touchedIds;
  std::vector<uint32_t> m_touchedIndices;
};

template <typename Visit>
void ManifoldCache::visitPoints(uint32_t slot, Visit&& visit) const {
  const CachedBodyPair& pair = m_pairs[slot];

  for (uint32_t index = 0u; index != pair.manifolds.size(); ++index) {
    const CachedManifold& manifold = pair.manifolds[index];
    const BodyRef& a = manifold.flipped ? pair.second : pair.first;
    const BodyRef& b = manifold.flipped ? pair.first : pair.second;

    for (uint32_t point = 0u; point != manifold.count; ++point) {
      const CachedManifoldPoint& built = pair.manifoldPoints[manifold.start + point];

      visit(CachedPointView{
        a,
        b,
        manifold.flipped ? built.secondChild : built.firstChild,
        manifold.flipped ? built.firstChild : built.secondChild,
        index,
        manifold,
        pair.points[manifold.start + point],
        built
      });
    }
  }
}

}
