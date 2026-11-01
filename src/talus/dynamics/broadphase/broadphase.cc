#include "broadphase.h"

#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/sleep/sleep.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace talus::dynamics {

Bounds boundsOf(const BodySet& bodies, const BodyRef& ref, double grow) {
  Bounds bounds{
    glm::dvec3(std::numeric_limits<double>::infinity()),
    glm::dvec3(-std::numeric_limits<double>::infinity())
  };

  if (ref.side == Side::Lattice) {
    for (const glm::dvec3& position : bodies.lattices[ref.index].position) {
      bounds.low = glm::min(bounds.low, position);
      bounds.high = glm::max(bounds.high, position);
    }
  } else if (ref.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[ref.index];
    const glm::dmat3 turn = glm::mat3_cast(body.rotation);

    if (body.shape == RigidShape::Box) {
      const glm::dvec3& half = body.halfExtents;
      const glm::dvec3 reach(
        std::abs(turn[0][0]) * half.x + std::abs(turn[1][0]) * half.y
          + std::abs(turn[2][0]) * half.z,
        std::abs(turn[0][1]) * half.x + std::abs(turn[1][1]) * half.y
          + std::abs(turn[2][1]) * half.z,
        std::abs(turn[0][2]) * half.x + std::abs(turn[1][2]) * half.y
          + std::abs(turn[2][2]) * half.z
      );

      bounds.low = body.position - reach;
      bounds.high = body.position + reach;
    } else if (body.shape == RigidShape::Compound) {
      const Bounds& local = body.compound->bounds;
      const glm::dvec3 middle = 0.5 * (local.low + local.high);
      const glm::dvec3 half = 0.5 * (local.high - local.low);

      bounds = boxBoundsIn(body.position + turn * middle, turn, half);
    } else if (body.shape != RigidShape::Sphere) {
      for (const glm::dvec3& sample : body.samples) {
        const glm::dvec3 world = body.position + turn * sample;

        bounds.low = glm::min(bounds.low, world);
        bounds.high = glm::max(bounds.high, world);
      }
    }

    if (body.radius > 0.0) {
      bounds.low = glm::min(bounds.low, body.position - glm::dvec3(body.radius));
      bounds.high =
        glm::max(bounds.high, body.position + glm::dvec3(body.radius));
    }
  } else {
    const StaticField& fixed = bodies.statics[ref.index];
    const field::SampledSdf& field = *fixed.field;
    const glm::dvec3 low(field.origin);
    const glm::dvec3 high =
      low
      + glm::dvec3(field.voxelSize)
          * (glm::dvec3(field.resolution) - glm::dvec3(1.0));

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::dvec3 local(
        (corner & 1u) != 0u ? high.x : low.x,
        (corner & 2u) != 0u ? high.y : low.y,
        (corner & 4u) != 0u ? high.z : low.z
      );
      const glm::dvec3 world = fixed.position + fixed.rotation * local;

      bounds.low = glm::min(bounds.low, world);
      bounds.high = glm::max(bounds.high, world);
    }
  }

  bounds.low -= glm::dvec3(grow);
  bounds.high += glm::dvec3(grow);

  return bounds;
}

double travelOf(
  const BodySet& bodies,
  std::span<const double> travel,
  const BodyRef& ref
) {
  if (ref.side == Side::Lattice) {
    return travel[ref.index];
  }

  if (ref.side == Side::Rigid) {
    return travel[bodies.lattices.size() + ref.index];
  }

  return 0.0;
}

std::vector<BodyPair> broadphase(const BodySet& bodies, double grow) {
  const std::vector<double> still(
    bodies.lattices.size() + bodies.rigids.size(),
    0.0
  );

  return broadphase(bodies, grow, still);
}

std::vector<BodyPair> broadphase(
  const BodySet& bodies,
  double grow,
  std::span<const double> travel,
  const Workers& workers
) {
  std::vector<BodyRef> refs;

  for (uint32_t index = 0u; index != bodies.lattices.size(); ++index) {
    refs.push_back({Side::Lattice, index});
  }

  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    refs.push_back({Side::Rigid, index});
  }

  for (uint32_t index = 0u; index != bodies.statics.size(); ++index) {
    refs.push_back({Side::Static, index});
  }

  const uint32_t count = static_cast<uint32_t>(refs.size());
  std::vector<Bounds> boxes(count);

  parallelFor(workers, count, 64u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t index = begin; index != end; ++index) {
      boxes[index] =
        boundsOf(bodies, refs[index], grow + travelOf(bodies, travel, refs[index]));
    }
  });

  std::vector<uint32_t> order(count);

  for (uint32_t index = 0u; index != count; ++index) {
    order[index] = index;
  }

  std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
    return boxes[a].low.x < boxes[b].low.x
           || (boxes[a].low.x == boxes[b].low.x && a < b);
  });

  const uint32_t blocks = std::max(1u, std::min(count, 4u * workers.count));
  std::vector<std::vector<BodyPair>> found(blocks);

  parallelFor(workers, blocks, 1u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t block = begin; block != end; ++block) {
      const size_t from = static_cast<size_t>(count) * block / blocks;
      const size_t to = static_cast<size_t>(count) * (block + 1u) / blocks;
      std::vector<BodyPair>& list = found[block];

      for (size_t first = from; first != to; ++first) {
        const Bounds& a = boxes[order[first]];

        for (size_t second = first + 1u; second != order.size(); ++second) {
          const Bounds& b = boxes[order[second]];

          if (b.low.x > a.high.x) {
            break;
          }

          const BodyRef& left = refs[order[first]];
          const BodyRef& right = refs[order[second]];

          if (left.side == Side::Static && right.side == Side::Static) {
            continue;
          }

          if (a.low.y <= b.high.y && b.low.y <= a.high.y && a.low.z <= b.high.z
              && b.low.z <= a.high.z) {
            list.push_back({left, right});
          }
        }
      }
    }
  });

  size_t total = 0u;

  for (const std::vector<BodyPair>& list : found) {
    total += list.size();
  }

  std::vector<BodyPair> pairs;

  pairs.reserve(total);

  for (const std::vector<BodyPair>& list : found) {
    pairs.insert(pairs.end(), list.begin(), list.end());
  }

  return pairs;
}

namespace {

double marginOf(const BodySet& bodies, const BodyRef& ref, const Bounds& tight) {
  double extent = 0.0;

  if (ref.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[ref.index];

    extent = body.radius;

    if (body.shape == RigidShape::Compound) {
      extent = std::max(extent, body.compound->reach);
    }

    for (const glm::dvec3& sample : body.samples) {
      extent = std::max(extent, glm::length(sample));
    }
  } else {
    extent = 0.5 * glm::length(tight.high - tight.low);
  }

  return std::min(0.05, 0.125 * extent);
}

uint32_t movingId(const BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Lattice ? bodies.lattices[ref.index].id
                                   : bodies.rigids[ref.index].id;
}

}

void BroadPhase::update(
  const BodySet& bodies,
  const IntegrationParameters& parameters,
  std::span<const double> travel,
  const Workers& workers,
  const ManifoldCache* known,
  const std::vector<uint32_t>* live
) {
  m_stamp += 1u;
  m_created.clear();
  m_lattice.clear();

  const double band = parameters.speculativeDistance + parameters.margin;
  const uint32_t lattices = static_cast<uint32_t>(bodies.lattices.size());
  const uint32_t moving = lattices + static_cast<uint32_t>(bodies.rigids.size());
  const uint32_t fixed = static_cast<uint32_t>(bodies.statics.size());
  const auto refOf = [lattices](uint32_t index) {
    return index < lattices ? BodyRef{Side::Lattice, index}
                            : BodyRef{Side::Rigid, index - lattices};
  };

  const uint32_t total = moving + fixed;

  m_tight.resize(total);
  m_action.assign(total, 0u);
  m_fat.resize(total);
  m_ids.resize(total);

  if (m_moving.size() < bodies.nextId) {
    m_moving.resize(bodies.nextId);
  }

  if (m_fixed.size() < bodies.nextStaticId) {
    m_fixed.resize(bodies.nextStaticId);
  }

  m_visit.clear();

  if (live != nullptr) {
    for (uint32_t index = 0u; index != lattices; ++index) {
      m_visit.push_back(index);
    }

    for (const uint32_t index : *live) {
      m_visit.push_back(lattices + index);
    }

    for (const uint32_t id : m_forced) {
      const Proxy& proxy = m_moving[id];

      if (proxy.node != noNode && proxy.ref.side == Side::Rigid
          && proxy.ref.index < bodies.rigids.size()) {
        m_visit.push_back(lattices + proxy.ref.index);
      }
    }

    std::sort(m_visit.begin() + lattices, m_visit.end());
    m_visit.erase(
      std::unique(m_visit.begin() + lattices, m_visit.end()),
      m_visit.end()
    );
  }

  m_forced.clear();

  const uint32_t visited = static_cast<uint32_t>(m_visit.size());
  const uint32_t visits = live != nullptr ? visited + fixed : total;

  parallelFor(
    workers,
    visits,
    64u,
    [&](uint32_t begin, uint32_t end, uint32_t) {
      for (uint32_t entry = begin; entry != end; ++entry) {
        const uint32_t index = live == nullptr   ? entry
                               : entry < visited ? m_visit[entry]
                                                 : moving + (entry - visited);
        const bool movable = index < moving;
        const BodyRef ref =
          movable ? refOf(index) : BodyRef{Side::Static, index - moving};
        const uint32_t id =
          movable ? movingId(bodies, ref) : bodies.statics[ref.index].id;
        const DynamicTree& tree = movable ? m_movingTree : m_fixedTree;
        Proxy& proxy = movable ? m_moving[id] : m_fixed[id];
        const double grow =
          movable ? band + travelOf(bodies, travel, ref) : band;

        if (ref.side == Side::Rigid && proxy.kept && !proxy.forced
            && proxy.node != noNode && proxy.ref.side == ref.side
            && proxy.ref.index == ref.index && proxy.grow == grow
            && proxy.position == bodies.rigids[ref.index].position
            && proxy.rotation == bodies.rigids[ref.index].rotation) {
          m_tight[index] = proxy.tight;
          m_ids[index] = id;
          proxy.seen = m_stamp;
          continue;
        }

        const Bounds tight = boundsOf(bodies, ref, grow);
        const bool posed = ref.side == Side::Rigid
                           && (!proxy.kept || proxy.position != bodies.rigids[ref.index].position
                               || proxy.rotation != bodies.rigids[ref.index].rotation);

        if (ref.side == Side::Rigid) {
          proxy.position = bodies.rigids[ref.index].position;
          proxy.rotation = bodies.rigids[ref.index].rotation;
          proxy.grow = grow;
          proxy.kept = true;
        }

        m_tight[index] = tight;
        m_ids[index] = id;
        proxy.ref = ref;
        proxy.tight = tight;
        proxy.seen = m_stamp;

        if (proxy.node == noNode) {
          m_action[index] = 3u;
          continue;
        }

        if (proxy.forced || !contains(tree.box(proxy.node), tight)) {
          if (proxy.forced || ref.side == Side::Lattice) {
            proxy.margin = ref.side == Side::Static
                             ? parameters.speculativeDistance
                             : marginOf(bodies, ref, tight);
          }

          m_fat[index] = grown(tight, proxy.margin);
          m_action[index] = 1u;
        } else if (!contains(
                     grown(tight, 2.0 * proxy.margin),
                     tree.box(proxy.node)
                   )) {
          m_fat[index] = grown(tight, proxy.margin);
          m_action[index] = 2u;
        } else if (posed) {
          m_action[index] = 4u;
        }

        proxy.forced = false;
      }
    }
  );

  const auto refresh = [&](
    DynamicTree& tree,
    std::vector<Proxy>& proxies,
    std::vector<uint8_t>& changed,
    uint32_t count,
    uint32_t first,
    uint32_t& tracked,
    std::vector<uint32_t>& moved,
    std::vector<uint32_t>& refreshed
  ) {
    moved.clear();
    refreshed.clear();

    const bool movingTree = &tree == &m_movingTree;
    std::vector<uint32_t>& marked = movingTree ? m_markedMoving : m_markedFixed;
    const std::vector<uint32_t>* walk =
      movingTree && live != nullptr ? &m_visit : nullptr;

    for (const uint32_t index : marked) {
      if (index < changed.size()) {
        changed[index] = 0u;
      }
    }

    marked.clear();
    changed.resize(count, 0u);
    m_pending.clear();

    const uint32_t steps =
      walk != nullptr ? static_cast<uint32_t>(walk->size()) : count;

    for (uint32_t step = 0u; step != steps; ++step) {
      const uint32_t index = walk != nullptr ? (*walk)[step] : step;
      const uint8_t action = m_action[first + index];

      if (action == 0u) {
        continue;
      }

      if (action == 3u) {
        m_pending.push_back(index);
        continue;
      }

      const uint32_t id = m_ids[first + index];

      if (action == 4u) {
        refreshed.push_back(id);
        continue;
      }

      Proxy& proxy = proxies[id];

      tree.resize(proxy.node, m_fat[first + index]);
      changed[index] = 1u;
      marked.push_back(index);

      if (action == 1u) {
        proxy.moved = m_stamp;
        moved.push_back(id);
      } else {
        refreshed.push_back(id);
      }
    }

    if (tracked + static_cast<uint32_t>(m_pending.size()) != count) {
      for (uint32_t id = 0u; id != proxies.size(); ++id) {
        if (proxies[id].node != noNode && proxies[id].seen != m_stamp) {
          tree.destroy(proxies[id].node);
          proxies[id] = Proxy{};
        }
      }
    }

    tracked = count;

    tree.refitFlagged();

    for (const uint32_t index : m_pending) {
      const uint32_t id = m_ids[first + index];
      const Bounds& tight = m_tight[first + index];
      Proxy& proxy = proxies[id];
      const BodyRef& ref = proxy.ref;

      proxy.margin = ref.side == Side::Static ? parameters.speculativeDistance
                                              : marginOf(bodies, ref, tight);
      proxy.node = tree.create(grown(tight, proxy.margin), id);
      proxy.moved = m_stamp;
      proxy.forced = false;
      changed[index] = 1u;
      marked.push_back(index);
      moved.push_back(id);
    }
  };

  std::vector<uint32_t> resizedFixed;

  refresh(m_movingTree, m_moving, m_changedMoving, moving, 0u, m_movingCount, m_movedMoving, m_changedIds);
  refresh(m_fixedTree, m_fixed, m_changedFixed, fixed, moving, m_fixedCount, m_movedFixed, resizedFixed);
  m_changedIds.insert(m_changedIds.end(), m_movedMoving.begin(), m_movedMoving.end());
  m_fixedChanged = !m_movedFixed.empty() || !resizedFixed.empty();


  const uint32_t movedMoving = static_cast<uint32_t>(m_movedMoving.size());
  const uint32_t movedCount = movedMoving + static_cast<uint32_t>(m_movedFixed.size());

  m_found.resize(std::max(workers.count, 1u));

  for (std::vector<std::pair<uint64_t, BodyPair>>& list : m_found) {
    list.clear();
  }

  parallelFor(workers, movedCount, 16u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
    std::vector<std::pair<uint64_t, BodyPair>>& found = m_found[worker];
    const auto keep = [&](const BodyRef& first, const BodyRef& second) {
      const uint64_t key = pairKey(bodies, first, second);

      if (known == nullptr || known->find(key) == noSlot) {
        found.emplace_back(key, BodyPair{first, second});
      }
    };

    for (uint32_t entry = begin; entry != end; ++entry) {
      if (entry < movedMoving) {
        const uint32_t id = m_movedMoving[entry];
        const Proxy& self = m_moving[id];
        const Bounds& box = m_movingTree.box(self.node);

        if (self.ref.side != Side::Rigid) {
          continue;
        }

        m_movingTree.query(box, [&](uint32_t, uint32_t other) {
          const Proxy& hit = m_moving[other];

          if (other == id || hit.ref.side != Side::Rigid
              || (hit.moved == m_stamp && other < id)) {
            return;
          }

          keep(self.ref, hit.ref);
        });
        m_fixedTree.query(box, [&](uint32_t, uint32_t other) {
          keep(self.ref, m_fixed[other].ref);
        });
      } else {
        const Proxy& self = m_fixed[m_movedFixed[entry - movedMoving]];

        m_movingTree.query(m_fixedTree.box(self.node), [&](uint32_t, uint32_t other) {
          const Proxy& hit = m_moving[other];

          if (hit.ref.side == Side::Rigid) {
            keep(hit.ref, self.ref);
          }
        });
      }
    }
  });

  m_keyed.clear();

  for (const std::vector<std::pair<uint64_t, BodyPair>>& found : m_found) {
    m_keyed.insert(m_keyed.end(), found.begin(), found.end());
  }

  std::sort(m_keyed.begin(), m_keyed.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });

  for (size_t index = 0u; index != m_keyed.size(); ++index) {
    if (index == 0u || m_keyed[index].first != m_keyed[index - 1u].first) {
      m_created.push_back(m_keyed[index].second);
    }
  }

  m_keyed.clear();

  for (uint32_t lattice = 0u; lattice != lattices; ++lattice) {
    const BodyRef ref{Side::Lattice, lattice};
    const uint32_t id = bodies.lattices[lattice].id;
    const Bounds& tight = m_tight[lattice];
    const bool rests = resting(bodies, ref);

    m_movingTree.query(tight, [&](uint32_t, uint32_t other) {
      const Proxy& hit = m_moving[other];

      if (other == id || (hit.ref.side == Side::Lattice && other < id)
          || !talus::dynamics::overlaps(tight, hit.tight)
          || (rests && resting(bodies, hit.ref))) {
        return;
      }

      m_keyed.emplace_back(pairKey(bodies, ref, hit.ref), BodyPair{ref, hit.ref});
    });

    if (rests) {
      continue;
    }

    m_fixedTree.query(tight, [&](uint32_t, uint32_t other) {
      const Proxy& hit = m_fixed[other];

      if (talus::dynamics::overlaps(tight, hit.tight)) {
        m_keyed.emplace_back(pairKey(bodies, ref, hit.ref), BodyPair{ref, hit.ref});
      }
    });
  }

  std::sort(m_keyed.begin(), m_keyed.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });

  for (const auto& [key, pair] : m_keyed) {
    m_lattice.push_back(pair);
  }
}

void BroadPhase::rebuildTrees() {
  m_movingTree.rebuild();
  m_fixedTree.rebuild();
}

uint32_t BroadPhase::seenBodies() const {
  return static_cast<uint32_t>(m_moving.size());
}

uint32_t BroadPhase::seenStatics() const {
  return static_cast<uint32_t>(m_fixed.size());
}

void BroadPhase::markMoved(uint32_t id) {
  if (id < m_moving.size()) {
    m_moving[id].forced = true;
    m_forced.push_back(id);
  }
}

std::span<const BodyPair> BroadPhase::created() const {
  return m_created;
}

std::span<const BodyPair> BroadPhase::latticePairs() const {
  return m_lattice;
}

std::span<const uint32_t> BroadPhase::changedProxies() const {
  return m_changedIds;
}

bool BroadPhase::fixedChanged() const {
  return m_fixedChanged;
}

bool BroadPhase::separated(
  const BodySet& bodies,
  const BodyRef& a,
  const BodyRef& b
) const {
  return (changed(bodies, a) || changed(bodies, b)) && !overlaps(bodies, a, b);
}

bool BroadPhase::changed(const BodySet& bodies, const BodyRef& ref) const {
  if (ref.side == Side::Static) {
    return ref.index < m_changedFixed.size() && m_changedFixed[ref.index] != 0u;
  }

  const size_t index =
    ref.side == Side::Lattice ? ref.index : bodies.lattices.size() + ref.index;

  return index < m_changedMoving.size() && m_changedMoving[index] != 0u;
}

bool BroadPhase::overlaps(
  const BodySet& bodies,
  const BodyRef& a,
  const BodyRef& b
) const {
  const Bounds* boxA = fatOf(bodies, a);
  const Bounds* boxB = fatOf(bodies, b);

  return boxA == nullptr || boxB == nullptr || talus::dynamics::overlaps(*boxA, *boxB);
}

const Bounds* BroadPhase::fatOf(const BodySet& bodies, const BodyRef& ref) const {
  if (ref.side == Side::Static) {
    const uint32_t id = bodies.statics[ref.index].id;

    if (id >= m_fixed.size() || m_fixed[id].node == noNode) {
      return nullptr;
    }

    return &m_fixedTree.box(m_fixed[id].node);
  }

  const uint32_t id = movingId(bodies, ref);

  if (id >= m_moving.size() || m_moving[id].node == noNode) {
    return nullptr;
  }

  return &m_movingTree.box(m_moving[id].node);
}

}
