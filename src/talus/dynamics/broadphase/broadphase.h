#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/bounds/bounds.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/dynamictree/dynamictree.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace talus::dynamics {

struct BodyRef {
  Side side = Side::Static;
  uint32_t index = 0u;
};

struct BodyPair {
  BodyRef first;
  BodyRef second;
};

[[nodiscard]] Bounds boundsOf(
  const BodySet& bodies,
  const BodyRef& ref,
  double grow
);
[[nodiscard]] double travelOf(
  const BodySet& bodies,
  std::span<const double> travel,
  const BodyRef& ref
);
[[nodiscard]] std::vector<BodyPair> broadphase(
  const BodySet& bodies,
  double grow
);
[[nodiscard]] std::vector<BodyPair> broadphase(
  const BodySet& bodies,
  double grow,
  std::span<const double> travel,
  const Workers& workers = Workers{}
);

class ManifoldCache;

class BroadPhase {
public:
  void update(
    const BodySet& bodies,
    const IntegrationParameters& parameters,
    std::span<const double> travel,
    const Workers& workers,
    const ManifoldCache* known = nullptr,
    const std::vector<uint32_t>* live = nullptr
  );
  void markMoved(uint32_t id);
  void rebuildTrees();

  template<typename Visit>
  void rayQuery(
    const glm::dvec3& from,
    const glm::dvec3& to,
    Visit&& visit
  ) const {
    double clip = 1.0;

    clip =
      m_fixedTree.rayQuery(from, to, clip, [&visit](uint32_t id, double at) {
        return visit(true, id, at);
      });
    m_movingTree.rayQuery(from, to, clip, [&visit](uint32_t id, double at) {
      return visit(false, id, at);
    });
  }

  [[nodiscard]] uint32_t seenBodies() const;
  [[nodiscard]] uint32_t seenStatics() const;
  [[nodiscard]] std::span<const BodyPair> created() const;
  [[nodiscard]] std::span<const BodyPair> latticePairs() const;
  [[nodiscard]] std::span<const uint32_t> changedProxies() const;
  [[nodiscard]] bool fixedChanged() const;
  [[nodiscard]] bool overlaps(
    const BodySet& bodies,
    const BodyRef& a,
    const BodyRef& b
  ) const;
  [[nodiscard]] bool separated(
    const BodySet& bodies,
    const BodyRef& a,
    const BodyRef& b
  ) const;

private:
  struct Proxy {
    uint32_t node = noNode;
    BodyRef ref;
    Bounds tight;
    glm::dvec3 position{0.0};
    glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
    double grow = 0.0;
    uint32_t seen = 0u;
    uint32_t moved = 0u;
    double margin = 0.0;
    bool forced = false;
    bool kept = false;
  };

  [[nodiscard]] const Bounds* fatOf(const BodySet& bodies, const BodyRef& ref) const;
  [[nodiscard]] bool changed(const BodySet& bodies, const BodyRef& ref) const;

  std::vector<Proxy> m_moving;
  std::vector<Proxy> m_fixed;
  DynamicTree m_movingTree;
  DynamicTree m_fixedTree;
  std::vector<Bounds> m_tight;
  std::vector<uint8_t> m_changedMoving;
  std::vector<uint8_t> m_changedFixed;
  std::vector<uint32_t> m_markedMoving;
  std::vector<uint32_t> m_markedFixed;
  std::vector<uint32_t> m_forced;
  std::vector<uint32_t> m_visit;
  std::vector<uint32_t> m_pending;
  std::vector<uint8_t> m_action;
  std::vector<Bounds> m_fat;
  std::vector<uint32_t> m_ids;
  std::vector<uint32_t> m_movedMoving;
  std::vector<uint32_t> m_movedFixed;
  std::vector<uint32_t> m_changedIds;
  bool m_fixedChanged = false;
  std::vector<std::vector<std::pair<uint64_t, BodyPair>>> m_found;
  std::vector<std::pair<uint64_t, BodyPair>> m_keyed;
  std::vector<BodyPair> m_created;
  std::vector<BodyPair> m_lattice;
  uint32_t m_stamp = 0u;
  uint32_t m_movingCount = 0u;
  uint32_t m_fixedCount = 0u;
};

}
