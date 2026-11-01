#include "manifoldcache.h"

#include <talus/dynamics/joints/joints.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/sleep/sleep.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace talus::dynamics {

size_t PairSlots::home(uint64_t key) const {
  uint64_t mixed = key + 0x9E3779B97F4A7C15ull;

  mixed = (mixed ^ (mixed >> 30u)) * 0xBF58476D1CE4E5B9ull;
  mixed = (mixed ^ (mixed >> 27u)) * 0x94D049BB133111EBull;
  mixed ^= mixed >> 31u;

  return static_cast<size_t>(mixed) & (m_keys.size() - 1u);
}

uint32_t PairSlots::find(uint64_t key) const {
  if (m_keys.empty()) {
    return noSlot;
  }

  const size_t mask = m_keys.size() - 1u;

  for (size_t at = home(key);; at = (at + 1u) & mask) {
    if (m_keys[at] == key) {
      return m_slots[at];
    }

    if (m_keys[at] == empty) {
      return noSlot;
    }
  }
}

void PairSlots::insert(uint64_t key, uint32_t slot) {
  if (2u * (m_count + 1u) > m_keys.size()) {
    grow();
  }

  const size_t mask = m_keys.size() - 1u;

  for (size_t at = home(key);; at = (at + 1u) & mask) {
    if (m_keys[at] == key) {
      return;
    }

    if (m_keys[at] == empty) {
      m_keys[at] = key;
      m_slots[at] = slot;
      m_count += 1u;
      return;
    }
  }
}

void PairSlots::erase(uint64_t key) {
  if (m_keys.empty()) {
    return;
  }

  const size_t mask = m_keys.size() - 1u;
  size_t hole = home(key);

  while (m_keys[hole] != key) {
    if (m_keys[hole] == empty) {
      return;
    }

    hole = (hole + 1u) & mask;
  }

  for (size_t at = (hole + 1u) & mask; m_keys[at] != empty; at = (at + 1u) & mask) {
    const size_t wanted = home(m_keys[at]);
    const bool stays = hole <= at ? (hole < wanted && wanted <= at) : (hole < wanted || wanted <= at);

    if (!stays) {
      m_keys[hole] = m_keys[at];
      m_slots[hole] = m_slots[at];
      hole = at;
    }
  }

  m_keys[hole] = empty;
  m_count -= 1u;
}

void PairSlots::grow() {
  std::vector<uint64_t> keys = std::move(m_keys);
  std::vector<uint32_t> slots = std::move(m_slots);

  m_keys.assign(std::max<size_t>(64u, 2u * keys.size()), empty);
  m_slots.assign(m_keys.size(), noSlot);
  m_count = 0u;

  for (size_t at = 0u; at != keys.size(); ++at) {
    if (keys[at] != empty) {
      insert(keys[at], slots[at]);
    }
  }
}

namespace {

bool storable(const BodyPair& pair) {
  const Side a = pair.first.side;
  const Side b = pair.second.side;

  return (a == Side::Rigid && b == Side::Rigid)
         || (a == Side::Rigid && b == Side::Static)
         || (a == Side::Static && b == Side::Rigid);
}

bool moves(const BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Rigid && bodies.rigids[ref.index].mass > 0.0;
}

}

void ManifoldCache::sync(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  const Workers& workers
) {
  const uint32_t count = static_cast<uint32_t>(pairs.size());
  std::vector<uint64_t> keys(count, 0u);
  std::vector<uint32_t> slots(count, noSlot);
  std::vector<uint8_t> stored(count, 0u);

  parallelFor(workers, count, 64u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t index = begin; index != end; ++index) {
      const BodyPair& pair = pairs[index];

      if (!storable(pair)) {
        continue;
      }

      stored[index] = 1u;
      keys[index] = pairKey(bodies, pair.first, pair.second);
      slots[index] = find(keys[index]);
    }
  });

  m_seen.assign(m_pairs.size(), 0u);

  for (uint32_t index = 0u; index != count; ++index) {
    if (stored[index] == 0u) {
      continue;
    }

    uint32_t slot = slots[index];

    if (slot == noSlot) {
      slot = find(keys[index]);
    }

    if (slot == noSlot) {
      slot = create(bodies, pairs[index], keys[index]);
    }

    if (slot >= m_seen.size()) {
      m_seen.resize(slot + 1u, 0u);
    }

    m_seen[slot] = 1u;
  }

  for (uint32_t slot = 0u; slot != m_pairs.size(); ++slot) {
    if (m_pairs[slot].live && (slot >= m_seen.size() || m_seen[slot] == 0u)) {
      remove(slot);
    }
  }
}

uint32_t ManifoldCache::size() const {
  return static_cast<uint32_t>(m_pairs.size());
}

bool ManifoldCache::live(uint32_t slot) const {
  return slot < m_pairs.size() && m_pairs[slot].live;
}

uint32_t ManifoldCache::find(uint64_t pair) const {
  return m_slot.find(pair);
}

CachedBodyPair& ManifoldCache::at(uint32_t slot) {
  return m_pairs[slot];
}

const CachedBodyPair& ManifoldCache::at(uint32_t slot) const {
  return m_pairs[slot];
}

std::span<const uint32_t> ManifoldCache::pairsOf(uint32_t id) const {
  if (id >= m_byBody.size()) {
    return {};
  }

  return m_byBody[id];
}

bool ManifoldCache::touchedLast(uint32_t id) const {
  return id < m_touched.size() && m_touched[id] != 0u;
}

std::span<const uint32_t> ManifoldCache::touchedIndices() const {
  return m_touchedIndices;
}

void ManifoldCache::keepTouched(
  const BodySet& bodies,
  std::span<const uint32_t> indices
) {
  for (const uint32_t id : m_touchedIds) {
    m_touched[id] = 0u;
  }

  m_touchedIds.clear();
  m_touchedIndices.assign(indices.begin(), indices.end());

  for (const uint32_t index : indices) {
    const uint32_t id = bodies.rigids[index].id;

    if (id >= m_touched.size()) {
      m_touched.resize(id + 1u, 0u);
    }

    m_touched[id] = 1u;
    m_touchedIds.push_back(id);
  }
}

uint32_t ManifoldCache::pointCount() const {
  uint32_t count = 0u;

  for (const CachedBodyPair& pair : m_pairs) {
    if (pair.live) {
      count += static_cast<uint32_t>(pair.points.size());
    }
  }

  return count;
}

std::vector<Contact> ManifoldCache::records(uint32_t slot) const {
  const CachedBodyPair& pair = m_pairs[slot];
  std::vector<Contact> out;

  out.reserve(pair.points.size());

  visitPoints(slot, [&](const CachedPointView& view) {
    Contact contact;

    contact.a.side = view.a.side;
    contact.a.body = view.a.index;
    contact.a.child = view.childA;
    contact.a.local = view.point.localA;
    contact.b.side = view.b.side;
    contact.b.body = view.b.index;
    contact.b.child = view.childB;
    contact.b.local = view.point.localB;
    contact.basis = view.manifold.basis;
    contact.gap = view.source.separation;
    contact.friction = pair.friction;
    contact.restitution = pair.restitution;
    contact.key = view.point.key;
    contact.pair = pair.pair;
    contact.manifold = view.index;
    contact.memory = view.source.memory;
    contact.cached = pair.pose;
    contact.cached.separation = view.point.builtSeparation;
    out.push_back(contact);
  });

  return out;
}

void ManifoldCache::rebuildManifolds(
  const BodySet&,
  uint32_t slot,
  std::span<const Contact> records
) {
  fill(slot, records, true);
}

void ManifoldCache::replaceManifolds(
  const BodySet& bodies,
  uint32_t slot,
  std::span<const Contact> records
) {
  fill(slot, records, false);
  refreshColour(bodies, slot);
}

void ManifoldCache::writeMemory(uint32_t slot, std::span<const Contact> records) {
  CachedBodyPair& pair = m_pairs[slot];
  const size_t count = std::min(pair.points.size(), records.size());

  for (size_t index = 0u; index != count; ++index) {
    pair.points[index].memory = records[index].memory;
  }
}

void ManifoldCache::refreshColour(const BodySet& bodies, uint32_t slot) {
  CachedBodyPair& pair = m_pairs[slot];
  const bool wanted = pair.live && !pair.points.empty()
                      && (!resting(bodies, pair.first) || !resting(bodies, pair.second));

  if (!wanted) {
    releaseColour(slot);
    return;
  }

  if (pair.colour == noColour) {
    assignColour(bodies, slot);
  }

  const bool wide = pair.colour != overflowColour && fitsWideLane(pair);

  if (pair.colourIndex == noSlot) {
    enlist(slot, wide);
  } else if (pair.wide != wide) {
    delist(slot);
    enlist(slot, wide);
  }
}

void ManifoldCache::assignColour(const BodySet& bodies, uint32_t slot) {
  CachedBodyPair& pair = m_pairs[slot];
  const bool movesFirst = moves(bodies, pair.first);
  const bool movesSecond = moves(bodies, pair.second);

  pair.colour = overflowColour;
  pair.marked = 0u;

  if (movesFirst && movesSecond) {
    for (uint32_t colour = 0u; colour != dynamicColourCount; ++colour) {
      if (!usedBy(colour, pair.firstId) && !usedBy(colour, pair.secondId)) {
        mark(colour, pair.firstId, true);
        mark(colour, pair.secondId, true);
        pair.colour = colour;
        pair.marked = 3u;
        break;
      }
    }
  } else if (movesFirst || movesSecond) {
    const uint32_t id = movesFirst ? pair.firstId : pair.secondId;

    for (uint32_t colour = overflowColour - 1u; colour >= 1u; --colour) {
      if (!usedBy(colour, id)) {
        mark(colour, id, true);
        pair.colour = colour;
        pair.marked = static_cast<uint8_t>(movesFirst ? 1u : 2u);
        break;
      }
    }
  }
}

void ManifoldCache::enlist(uint32_t slot, bool wide) {
  CachedBodyPair& pair = m_pairs[slot];
  std::vector<uint32_t>& list = wide ? m_wide[pair.colour] : m_other[pair.colour];

  pair.wide = wide;
  pair.colourIndex = static_cast<uint32_t>(list.size());
  list.push_back(slot);
}

void ManifoldCache::delist(uint32_t slot) {
  CachedBodyPair& pair = m_pairs[slot];

  if (pair.colourIndex == noSlot) {
    return;
  }

  std::vector<uint32_t>& list = pair.wide ? m_wide[pair.colour] : m_other[pair.colour];
  const uint32_t last = list.back();

  list[pair.colourIndex] = last;
  m_pairs[last].colourIndex = pair.colourIndex;
  list.pop_back();
  pair.colourIndex = noSlot;
}

void ManifoldCache::recolour(const BodySet& bodies, std::span<const uint32_t> slots) {
  for (const uint32_t slot : slots) {
    if (live(slot)) {
      refreshColour(bodies, slot);
    }
  }
}

std::vector<uint32_t> ManifoldCache::syncBodies(
  const BodySet& bodies,
  const Workers& workers,
  const std::vector<uint32_t>* live
) {
  std::vector<uint32_t> slots;
  const auto stateOf = [](const RigidBody& body) {
    return static_cast<uint8_t>(4u | (body.asleep ? 0u : 1u) | (body.mass > 0.0 ? 2u : 0u));
  };
  std::vector<std::vector<uint32_t>> found(std::max(workers.count, 1u));
  const uint32_t count = static_cast<uint32_t>(
    live != nullptr ? live->size() : bodies.rigids.size()
  );

  parallelFor(
    workers,
    count,
    1024u,
    [&](uint32_t begin, uint32_t end, uint32_t worker) {
      for (uint32_t entry = begin; entry != end; ++entry) {
        const uint32_t index = live != nullptr ? (*live)[entry] : entry;
        const RigidBody& body = bodies.rigids[index];

        if (body.id >= m_bodyState.size()
            || m_bodyState[body.id] != stateOf(body)) {
          found[worker].push_back(index);
        }
      }
    }
  );

  std::vector<uint32_t> changed;

  for (const std::vector<uint32_t>& list : found) {
    changed.insert(changed.end(), list.begin(), list.end());
  }

  std::sort(changed.begin(), changed.end());

  for (const uint32_t index : changed) {
    const RigidBody& body = bodies.rigids[index];
    const uint8_t state = stateOf(body);

    if (body.id >= m_bodyState.size()) {
      m_bodyState.resize(body.id + 1u, 0u);
    }

    const uint8_t previous = m_bodyState[body.id];

    if (previous == state) {
      continue;
    }

    m_bodyState[body.id] = state;

    if (body.id >= m_byBody.size()) {
      continue;
    }

    const bool regroup = (previous & 4u) != 0u && ((previous ^ state) & 2u) != 0u;

    for (const uint32_t slot : m_byBody[body.id]) {
      if (regroup) {
        releaseColour(slot);
      }

      slots.push_back(slot);
    }
  }

  std::sort(slots.begin(), slots.end());
  slots.erase(std::unique(slots.begin(), slots.end()), slots.end());

  return slots;
}

void ManifoldCache::sleepBody(const BodySet& bodies, uint32_t id) {
  if (id < m_bodyState.size()) {
    m_bodyState[id] = static_cast<uint8_t>(m_bodyState[id] & ~1u);
  }

  if (id >= m_byBody.size()) {
    return;
  }

  for (const uint32_t slot : m_byBody[id]) {
    if (resting(bodies, m_pairs[slot].first) && resting(bodies, m_pairs[slot].second)) {
      releaseColour(slot);
    }
  }
}

std::span<const uint32_t> ManifoldCache::widePairs(uint32_t colour) const {
  return m_wide[colour];
}

std::span<const uint32_t> ManifoldCache::otherPairs(uint32_t colour) const {
  return m_other[colour];
}

void ManifoldCache::touchingPairs(std::vector<uint32_t>& slots) const {
  slots.clear();

  for (uint32_t colour = 0u; colour != colourCount; ++colour) {
    slots.insert(slots.end(), m_wide[colour].begin(), m_wide[colour].end());
    slots.insert(slots.end(), m_other[colour].begin(), m_other[colour].end());
  }
}

void ManifoldCache::createPairs(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  const JointSet* joints
) {
  const bool filter = joints != nullptr && !joints->empty();

  for (const BodyPair& pair : pairs) {
    if (!storable(pair)) {
      continue;
    }

    if (filter && pair.first.side == Side::Rigid
        && pair.second.side == Side::Rigid
        && joints->excludes(
          bodies.rigids[pair.first.index].id,
          bodies.rigids[pair.second.index].id
        )) {
      continue;
    }

    const uint64_t key = pairKey(bodies, pair.first, pair.second);

    if (find(key) == noSlot) {
      (void)create(bodies, pair, key);
    }
  }
}

void ManifoldCache::forgetPair(uint32_t idA, uint32_t idB) {
  if (idA >= m_byBody.size()) {
    return;
  }

  for (const uint32_t slot : m_byBody[idA]) {
    const CachedBodyPair& pair = m_pairs[slot];

    if ((pair.firstId == idA && pair.secondId == idB)
        || (pair.firstId == idB && pair.secondId == idA)) {
      remove(slot);

      return;
    }
  }
}

void ManifoldCache::destroyPair(uint32_t slot) {
  if (live(slot)) {
    remove(slot);
  }
}

uint32_t ManifoldCache::dropChildren(
  const BodySet& bodies,
  uint32_t slot,
  uint32_t id,
  std::span<const uint32_t> keys,
  bool reframed
) {
  CachedBodyPair& pair = m_pairs[slot];
  const bool onFirst = pair.firstId == id;
  std::vector<CachedManifold> manifolds;
  std::vector<CachedContactPoint> points;
  std::vector<CachedManifoldPoint> built;
  uint32_t dropped = 0u;

  for (const CachedManifold& manifold : pair.manifolds) {
    CachedManifold moved = manifold;

    moved.start = static_cast<uint32_t>(points.size());

    for (uint32_t index = manifold.start; index != manifold.start + manifold.count; ++index) {
      const CachedManifoldPoint& point = pair.manifoldPoints[index];
      const uint32_t child = onFirst ? point.firstChild : point.secondChild;

      if (std::binary_search(keys.begin(), keys.end(), child)) {
        dropped += 1u;
        continue;
      }

      points.push_back(pair.points[index]);
      built.push_back(point);
    }

    moved.count = static_cast<uint32_t>(points.size()) - moved.start;

    if (moved.count != 0u) {
      manifolds.push_back(moved);
    }
  }

  pair.manifolds = std::move(manifolds);
  pair.points = std::move(points);
  pair.manifoldPoints = std::move(built);

  if (reframed || dropped != 0u) {
    pair.pose.valid = false;
  }

  refreshColour(bodies, slot);

  return dropped;
}

void ManifoldCache::removeBody(uint32_t id) {
  if (id >= m_byBody.size()) {
    return;
  }

  const std::vector<uint32_t> slots = m_byBody[id];

  for (const uint32_t slot : slots) {
    if (m_pairs[slot].live) {
      remove(slot);
    }
  }
}

void ManifoldCache::renumber(Side side, uint32_t removed) {
  for (CachedBodyPair& pair : m_pairs) {
    if (!pair.live) {
      continue;
    }

    for (BodyRef* ref : {&pair.first, &pair.second}) {
      if (ref->side == side && ref->index > removed) {
        --ref->index;
      }
    }
  }
}

void ManifoldCache::renumber(Side side, std::span<const uint32_t> moved) {
  for (CachedBodyPair& pair : m_pairs) {
    if (!pair.live) {
      continue;
    }

    for (BodyRef* ref : {&pair.first, &pair.second}) {
      if (ref->side == side) {
        ref->index = moved[ref->index];
      }
    }
  }
}

uint32_t ManifoldCache::create(
  const BodySet& bodies,
  const BodyPair& pair,
  uint64_t key
) {
  uint32_t slot = 0u;

  if (!m_free.empty()) {
    slot = m_free.back();
    m_free.pop_back();
  } else {
    slot = static_cast<uint32_t>(m_pairs.size());
    m_pairs.emplace_back();
  }

  const bool staticFirst = pair.first.side == Side::Static;
  BodyRef first = staticFirst ? pair.second : pair.first;
  BodyRef second = staticFirst ? pair.first : pair.second;

  if (second.side == Side::Rigid
      && bodies.rigids[first.index].id > bodies.rigids[second.index].id) {
    std::swap(first, second);
  }

  CachedBodyPair& entry = m_pairs[slot];

  entry.first = first;
  entry.second = second;
  entry.firstId = bodies.rigids[first.index].id;
  entry.secondId =
    second.side == Side::Rigid ? bodies.rigids[second.index].id : noBody;
  entry.pair = key;
  entry.friction = 0.0;
  entry.restitution = 0.0;
  entry.pose = CachedPose{};
  entry.colour = noColour;
  entry.colourIndex = noSlot;
  entry.marked = 0u;
  entry.wide = false;
  entry.live = true;
  entry.manifolds.clear();
  entry.points.clear();
  entry.manifoldPoints.clear();
  m_slot.insert(key, slot);

  for (const uint32_t id : {entry.firstId, entry.secondId}) {
    if (id == noBody) {
      continue;
    }

    if (id >= m_byBody.size()) {
      m_byBody.resize(id + 1u);
    }

    m_byBody[id].push_back(slot);
  }

  return slot;
}

void ManifoldCache::remove(uint32_t slot) {
  CachedBodyPair& pair = m_pairs[slot];

  releaseColour(slot);

  for (const uint32_t id : {pair.firstId, pair.secondId}) {
    if (id == noBody || id >= m_byBody.size()) {
      continue;
    }

    std::vector<uint32_t>& list = m_byBody[id];

    list.erase(std::remove(list.begin(), list.end(), slot), list.end());
  }

  m_slot.erase(pair.pair);
  pair.live = false;
  pair.manifolds.clear();
  pair.points.clear();
  pair.manifoldPoints.clear();
  pair.spare.clear();
  pair.spareManifoldPoints.clear();
  pair.spareManifolds.clear();
  m_free.push_back(slot);
}

void ManifoldCache::fill(
  uint32_t slot,
  std::span<const Contact> records,
  bool carry
) {
  CachedBodyPair& pair = m_pairs[slot];

  pair.spare.swap(pair.points);
  pair.spareManifoldPoints.swap(pair.manifoldPoints);
  pair.spareManifolds.swap(pair.manifolds);
  pair.points.clear();
  pair.manifoldPoints.clear();
  pair.manifolds.clear();

  const auto carried = [&](const CachedManifoldPoint& point) {
    ContactMemory memory;
    uint32_t matches = 0u;

    for (size_t index = 0u; index != pair.spare.size(); ++index) {
      const CachedManifoldPoint& old = pair.spareManifoldPoints[index];

      if (old.key == point.key && old.firstChild == point.firstChild
          && old.secondChild == point.secondChild) {
        memory = pair.spare[index].memory;
        ++matches;
      }
    }

    return matches == 1u ? memory : ContactMemory{};
  };

  for (size_t first = 0u; first != records.size(); ++first) {
    const uint32_t group = records[first].manifold;
    bool seen = false;

    for (size_t earlier = 0u; earlier != first && !seen; ++earlier) {
      seen = records[earlier].manifold == group;
    }

    if (seen) {
      continue;
    }

    CachedManifold manifold;

    manifold.start = static_cast<uint32_t>(pair.points.size());
    manifold.basis = records[first].basis;
    manifold.flipped = !(records[first].a.side == pair.first.side
                         && records[first].a.body == pair.first.index);

    for (size_t index = first; index != records.size(); ++index) {
      const Contact& record = records[index];

      if (record.manifold != group) {
        continue;
      }

      CachedContactPoint point;
      CachedManifoldPoint built;

      built.key = record.key;
      built.localA = record.a.local;
      built.localB = record.b.local;
      built.builtSeparation = record.cached.separation;
      built.firstChild = manifold.flipped ? record.b.child : record.a.child;
      built.secondChild = manifold.flipped ? record.a.child : record.b.child;
      point.separation = record.gap;
      point.memory = carry ? carried(built) : record.memory;
      pair.points.push_back(point);
      pair.manifoldPoints.push_back(built);
    }

    manifold.count = static_cast<uint32_t>(pair.points.size()) - manifold.start;

    if (carry) {
      double closest = sharedNormal;

      for (const CachedManifold& old : pair.spareManifolds) {
        const double aligned = glm::dot(old.basis[0], manifold.basis[0]);

        if (old.flipped == manifold.flipped && aligned >= closest) {
          closest = aligned;
          manifold.twist = old.twist;
        }
      }
    }

    pair.manifolds.push_back(manifold);
  }

  if (records.empty()) {
    pair.pose = CachedPose{};
  } else {
    pair.pose = records.front().cached;
    pair.friction = records.front().friction;
    pair.restitution = records.front().restitution;
  }

  if (!pair.pose.valid) {
    return;
  }

  const glm::dmat3 turnA = glm::mat3_cast(pair.pose.rotationA);
  const glm::dmat3 turnB = glm::mat3_cast(pair.pose.rotationB);

  for (const CachedManifold& manifold : pair.manifolds) {
    const bool fixedB = (manifold.flipped ? pair.first : pair.second).side == Side::Static;
    const glm::dmat3& turnOfA = manifold.flipped ? turnB : turnA;
    const glm::dmat3& turnOfB = manifold.flipped ? turnA : turnB;

    for (uint32_t index = manifold.start; index != manifold.start + manifold.count; ++index) {
      CachedContactPoint& point = pair.points[index];
      const CachedManifoldPoint& built = pair.manifoldPoints[index];
      const glm::dvec3 shift = 0.5 * built.builtSeparation * manifold.basis[0];

      point.anchorB = glm::vec3(turnOfA * built.localA - shift);
      point.anchorA = fixedB ? point.anchorB : glm::vec3(turnOfB * built.localB + shift);
    }
  }
}

void ManifoldCache::releaseColour(uint32_t slot) {
  CachedBodyPair& pair = m_pairs[slot];

  delist(slot);

  if (pair.colour != noColour && pair.colour != overflowColour) {
    if ((pair.marked & 1u) != 0u) {
      mark(pair.colour, pair.firstId, false);
    }

    if ((pair.marked & 2u) != 0u) {
      mark(pair.colour, pair.secondId, false);
    }
  }

  pair.colour = noColour;
  pair.marked = 0u;
}

bool ManifoldCache::usedBy(uint32_t colour, uint32_t id) const {
  const std::vector<uint8_t>& used = m_used[colour];

  return id < used.size() && used[id] != 0u;
}

void ManifoldCache::mark(uint32_t colour, uint32_t id, bool used) {
  std::vector<uint8_t>& marks = m_used[colour];

  if (id >= marks.size()) {
    marks.resize(id + 1u, 0u);
  }

  marks[id] = static_cast<uint8_t>(used ? 1u : 0u);
}

}
