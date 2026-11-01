#include "islands.h"

#include <talus/dynamics/sleep/sleep.h>

#include <algorithm>
#include <numeric>
#include <utility>

namespace talus::dynamics {

namespace {

uint32_t findRoot(std::vector<uint32_t>& parent, uint32_t node) {
  while (parent[node] != node) {
    parent[node] = parent[parent[node]];
    node = parent[node];
  }

  return node;
}

bool heavy(const BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Rigid && bodies.rigids[ref.index].mass > 0.0;
}

bool hasAnchor(const LatticeBody& body) {
  if (body.mesh == nullptr) {
    return false;
  }

  return std::any_of(
    body.mesh->anchored.begin(),
    body.mesh->anchored.end(),
    [](uint8_t value) { return value != 0u; }
  );
}

}

std::vector<Island> islandsOf(
  const BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const Contact> contacts
) {
  std::vector<Island> islands;

  islandsOf(bodies, cache, contacts, islands);

  return islands;
}

void islandsOf(
  const BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const Contact> contacts,
  std::vector<Island>& islands
) {
  const uint32_t lattices = static_cast<uint32_t>(bodies.lattices.size());
  const uint32_t total = lattices + static_cast<uint32_t>(bodies.rigids.size());
  std::vector<uint32_t> parent(total);
  std::vector<uint8_t> external(total, 0u);

  std::iota(parent.begin(), parent.end(), 0u);

  const auto slot = [&](const ContactSide& side) {
    return side.side == Side::Lattice ? side.body : lattices + side.body;
  };

  for (const Contact& contact : contacts) {
    const bool aStatic = contact.a.side == Side::Static;
    const bool bStatic = contact.b.side == Side::Static;

    if (aStatic && !bStatic) {
      external[slot(contact.b)] = 1u;
    } else if (bStatic && !aStatic) {
      external[slot(contact.a)] = 1u;
    } else if (!aStatic && !bStatic) {
      parent[findRoot(parent, slot(contact.a))] =
        findRoot(parent, slot(contact.b));
    }
  }

  for (uint32_t entry = 0u; entry != cache.size(); ++entry) {
    if (!cache.live(entry) || cache.at(entry).points.empty()) {
      continue;
    }

    const CachedBodyPair& pair = cache.at(entry);

    if (pair.second.side == Side::Static) {
      external[lattices + pair.first.index] = 1u;
    } else if (!heavy(bodies, pair.first) || !heavy(bodies, pair.second)) {
      external[lattices + pair.first.index] = 1u;
      external[lattices + pair.second.index] = 1u;
    } else {
      parent[findRoot(parent, lattices + pair.first.index)] =
        findRoot(parent, lattices + pair.second.index);
    }
  }

  for (uint32_t lattice = 0u; lattice != lattices; ++lattice) {
    if (hasAnchor(bodies.lattices[lattice])) {
      external[lattice] = 1u;
    }
  }

  for (uint32_t rigid = 0u; rigid != bodies.rigids.size(); ++rigid) {
    if (!(bodies.rigids[rigid].mass > 0.0)) {
      external[lattices + rigid] = 1u;
    }
  }

  std::vector<int32_t> islandOf(total, -1);
  uint32_t count = 0u;

  for (uint32_t body = 0u; body != total; ++body) {
    const uint32_t root = findRoot(parent, body);

    if (islandOf[root] < 0) {
      islandOf[root] = static_cast<int32_t>(count);

      if (count == islands.size()) {
        islands.emplace_back();
      }

      Island& fresh = islands[count];

      fresh.bodies.clear();
      fresh.contacts.clear();
      fresh.pairs.clear();
      fresh.external = false;
      fresh.asleep = true;
      fresh.owner = Owner::Softstep;
      count += 1u;
    }

    Island& island = islands[static_cast<size_t>(islandOf[root])];
    const BodyRef ref = body < lattices ? BodyRef{Side::Lattice, body}
                                        : BodyRef{Side::Rigid, body - lattices};

    island.bodies.push_back(ref);
    island.external = island.external || external[body] != 0u;
    island.asleep = island.asleep && resting(bodies, ref);

    if (body < lattices) {
      island.owner = Owner::Strainstep;
    }
  }

  islands.resize(count);

  for (uint32_t index = 0u; index != contacts.size(); ++index) {
    const Contact& contact = contacts[index];
    const ContactSide& side =
      contact.a.side != Side::Static ? contact.a : contact.b;

    if (side.side == Side::Static) {
      continue;
    }

    islands[static_cast<size_t>(islandOf[findRoot(parent, slot(side))])]
      .contacts.push_back(index);
  }

  for (uint32_t entry = 0u; entry != cache.size(); ++entry) {
    if (!cache.live(entry) || cache.at(entry).points.empty()) {
      continue;
    }

    const uint32_t body = lattices + cache.at(entry).first.index;

    islands[static_cast<size_t>(islandOf[findRoot(parent, body)])]
      .pairs.push_back(entry);
  }
}

uint32_t IslandStore::create() {
  if (!m_free.empty()) {
    const uint32_t island = m_free.back();

    m_free.pop_back();
    m_members[island].clear();
    m_removed[island] = 0u;

    return island;
  }

  m_members.emplace_back();
  m_removed.push_back(0u);

  return static_cast<uint32_t>(m_members.size() - 1u);
}

void IslandStore::sync(const BodySet& bodies, const Workers& workers) {
  const uint32_t limit = bodies.nextId;

  if (m_islandOf.size() < limit) {
    m_islandOf.resize(limit, noIsland);
    m_ref.resize(limit);
    m_state.resize(limit, 0u);
  }

  m_stamp += 1u;

  if (m_present.size() < m_islandOf.size()) {
    m_present.resize(m_islandOf.size(), 0u);
  }

  uint32_t added = 0u;
  const auto add = [&](uint32_t id, const BodyRef& ref, uint8_t state) {
    if (m_ref[id].side != ref.side || m_ref[id].index != ref.index || m_state[id] != state) {
      m_ref[id] = ref;
      m_state[id] = state;
      m_dirty = true;
    }

    if (m_islandOf[id] == noIsland) {
      const uint32_t island = create();

      m_members[island].push_back(id);
      m_islandOf[id] = island;
      added += 1u;
    }
  };

  for (uint32_t index = 0u; index != bodies.lattices.size(); ++index) {
    const LatticeBody& body = bodies.lattices[index];

    add(body.id, BodyRef{Side::Lattice, index}, static_cast<uint8_t>(body.asleep ? 3u : 1u));
  }

  const auto stateOf = [](const RigidBody& body) {
    return static_cast<uint8_t>(1u | (body.asleep ? 2u : 0u) | (body.mass > 0.0 ? 4u : 0u));
  };
  std::vector<std::vector<uint32_t>> found(std::max(workers.count, 1u));

  parallelFor(workers, static_cast<uint32_t>(bodies.rigids.size()), 1024u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
    for (uint32_t index = begin; index != end; ++index) {
      const RigidBody& body = bodies.rigids[index];
      const BodyRef& known = m_ref[body.id];

      if (m_islandOf[body.id] == noIsland || known.side != Side::Rigid || known.index != index
          || m_state[body.id] != stateOf(body)) {
        found[worker].push_back(index);
      }
    }
  });

  std::vector<uint32_t> changed;

  for (const std::vector<uint32_t>& list : found) {
    changed.insert(changed.end(), list.begin(), list.end());
  }

  std::sort(changed.begin(), changed.end());

  for (const uint32_t index : changed) {
    const RigidBody& body = bodies.rigids[index];

    add(body.id, BodyRef{Side::Rigid, index}, stateOf(body));
  }

  const uint32_t total =
    static_cast<uint32_t>(bodies.lattices.size() + bodies.rigids.size());
  const bool removals = m_bodies + added != total;

  m_bodies = total;

  if (added != 0u) {
    m_dirty = true;
    m_full = true;
  }

  if (!removals) {
    return;
  }

  m_dirty = true;

  for (const LatticeBody& body : bodies.lattices) {
    m_present[body.id] = m_stamp;
  }

  for (const RigidBody& body : bodies.rigids) {
    m_present[body.id] = m_stamp;
  }

  for (uint32_t id = 0u; id != m_islandOf.size(); ++id) {
    const uint32_t island = m_islandOf[id];

    if (island == noIsland || m_present[id] == m_stamp) {
      continue;
    }

    std::vector<uint32_t>& members = m_members[island];

    members.erase(std::find(members.begin(), members.end(), id));
    m_islandOf[id] = noIsland;
    m_state[id] = 0u;
    m_removed[island] += 1u;

    if (members.empty()) {
      m_free.push_back(island);
    }
  }
}

void IslandStore::link(uint32_t a, uint32_t b) {
  uint32_t keep = m_islandOf[a];
  uint32_t drop = m_islandOf[b];

  if (keep == drop) {
    return;
  }

  if (m_members[keep].size() < m_members[drop].size()) {
    std::swap(keep, drop);
  }

  for (const uint32_t id : m_members[drop]) {
    m_islandOf[id] = keep;
  }

  m_members[keep].insert(m_members[keep].end(), m_members[drop].begin(), m_members[drop].end());
  m_members[drop].clear();
  m_dirty = true;
  m_removed[keep] += m_removed[drop];
  m_removed[drop] = 0u;
  m_free.push_back(drop);
}

void IslandStore::markRemoved(uint32_t id) {
  if (id < m_islandOf.size() && m_islandOf[id] != noIsland) {
    m_removed[m_islandOf[id]] += 1u;
    m_dirty = true;
  }
}

void IslandStore::linkPairs(const BodySet& bodies, const ManifoldCache& cache) {
  for (uint32_t colour = 0u; colour != colourCount; ++colour) {
    for (const std::span<const uint32_t> slots : {cache.widePairs(colour), cache.otherPairs(colour)}) {
      for (const uint32_t slot : slots) {
        const CachedBodyPair& pair = cache.at(slot);

        if (pair.second.side == Side::Rigid && heavy(bodies, pair.first)
            && heavy(bodies, pair.second)) {
          link(pair.firstId, pair.secondId);
        }
      }
    }
  }
}

void IslandStore::linkContacts(const BodySet& bodies, std::span<const Contact> contacts) {
  const auto idOf = [&](const ContactSide& side) {
    return side.side == Side::Rigid ? bodies.rigids[side.body].id
                                    : bodies.lattices[side.body].id;
  };

  for (const Contact& contact : contacts) {
    if (contact.a.side != Side::Static && contact.b.side != Side::Static) {
      link(idOf(contact.a), idOf(contact.b));
    }
  }
}

bool IslandStore::heavyId(const BodySet& bodies, uint32_t id) const {
  return id < m_islandOf.size() && m_islandOf[id] != noIsland
         && heavy(bodies, m_ref[id]);
}

void IslandStore::linkJoints(const BodySet& bodies, const JointSet& joints) {
  for (const Joint& joint : joints.all()) {
    if (heavyId(bodies, joint.def.bodyA) && heavyId(bodies, joint.def.bodyB)) {
      link(joint.def.bodyA, joint.def.bodyB);
    }
  }
}

std::vector<uint32_t> IslandStore::split(
  const BodySet& bodies,
  uint32_t island,
  const ManifoldCache& cache,
  std::span<const Contact> contacts,
  const JointSet& joints
) {
  const std::vector<uint32_t> members = m_members[island];
  const uint32_t count = static_cast<uint32_t>(members.size());

  m_stamp += 1u;
  m_splits += 1u;
  m_dirty = true;

  if (m_local.size() < m_islandOf.size()) {
    m_local.resize(m_islandOf.size());
    m_localStamp.resize(m_islandOf.size(), 0u);
  }

  m_parent.resize(count);

  for (uint32_t index = 0u; index != count; ++index) {
    m_local[members[index]] = index;
    m_localStamp[members[index]] = m_stamp;
    m_parent[index] = index;
  }

  const auto root = [&](uint32_t node) {
    while (m_parent[node] != node) {
      m_parent[node] = m_parent[m_parent[node]];
      node = m_parent[node];
    }

    return node;
  };
  const auto join = [&](uint32_t a, uint32_t b) {
    if (a < m_localStamp.size() && b < m_localStamp.size()
        && m_localStamp[a] == m_stamp && m_localStamp[b] == m_stamp) {
      m_parent[root(m_local[a])] = root(m_local[b]);
    }
  };

  for (const uint32_t id : members) {
    for (const uint32_t slot : cache.pairsOf(id)) {
      if (!cache.live(slot)) {
        continue;
      }

      const CachedBodyPair& pair = cache.at(slot);

      if (!pair.points.empty() && pair.second.side == Side::Rigid
          && heavy(bodies, pair.first) && heavy(bodies, pair.second)) {
        join(pair.firstId, pair.secondId);
      }
    }
  }

  const auto idOf = [&](const ContactSide& side) {
    return side.side == Side::Rigid ? bodies.rigids[side.body].id
                                    : bodies.lattices[side.body].id;
  };

  for (const Contact& contact : contacts) {
    if (contact.a.side != Side::Static && contact.b.side != Side::Static) {
      join(idOf(contact.a), idOf(contact.b));
    }
  }

  for (const Joint& joint : joints.all()) {
    if (heavyId(bodies, joint.def.bodyA) && heavyId(bodies, joint.def.bodyB)) {
      join(joint.def.bodyA, joint.def.bodyB);
    }
  }

  std::vector<uint32_t> pieces;
  std::vector<uint32_t> pieceOf(count, noIsland);

  m_members[island].clear();
  m_removed[island] = 0u;

  for (uint32_t index = 0u; index != count; ++index) {
    const uint32_t top = root(index);

    if (pieceOf[top] == noIsland) {
      pieceOf[top] = pieces.empty() ? island : create();
      pieces.push_back(pieceOf[top]);
    }

    m_members[pieceOf[top]].push_back(members[index]);
    m_islandOf[members[index]] = pieceOf[top];
  }

  return pieces;
}

void IslandStore::build(
  BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const uint32_t> changed,
  bool separated,
  std::span<const Contact> contacts,
  std::vector<Island>& islands,
  const JointSet& joints
) {
  m_splits = 0u;

  if (m_full) {
    linkPairs(bodies, cache);
    m_full = false;
  }

  for (const uint32_t slot : changed) {
    if (!cache.live(slot)) {
      continue;
    }

    const CachedBodyPair& pair = cache.at(slot);

    if (pair.second.side == Side::Static) {
      m_dirty = true;
    } else if (pair.second.side == Side::Rigid && !pair.points.empty()) {
      if (heavy(bodies, pair.first) && heavy(bodies, pair.second)) {
        link(pair.firstId, pair.secondId);
      } else {
        m_dirty = true;
      }
    }
  }

  linkContacts(bodies, contacts);
  linkJoints(bodies, joints);

  std::vector<uint8_t> settled(m_members.size(), 0u);

  for (const LatticeBody& lattice : bodies.lattices) {
    const uint32_t island = m_islandOf[lattice.id];

    if (m_members[island].size() < 2u
        || (island < settled.size() && settled[island] != 0u)) {
      continue;
    }

    for (const uint32_t piece :
         split(bodies, island, cache, contacts, joints)) {
      if (piece >= settled.size()) {
        settled.resize(piece + 1u, 0u);
      }

      settled[piece] = 1u;
    }
  }

  const bool carries = !contacts.empty();

  if (!m_dirty && !separated && !carries && !m_builtContacts && bodies.lattices.empty()) {
    return;
  }

  m_dirty = false;
  m_builtContacts = carries;

  if (m_outputStamp.size() < m_members.size()) {
    m_output.resize(m_members.size());
    m_outputStamp.resize(m_members.size(), 0u);
  }

  m_stamp += 1u;
  m_resting.clear();

  uint32_t count = 0u;
  const auto entry = [&](uint32_t id, const BodyRef& ref) -> Island& {
    const uint32_t island = m_islandOf[id];

    if (m_outputStamp[island] != m_stamp) {
      m_outputStamp[island] = m_stamp;
      m_output[island] = count;

      if (count == islands.size()) {
        islands.emplace_back();
      }

      Island& fresh = islands[count];

      fresh.bodies.clear();
      fresh.contacts.clear();
      fresh.pairs.clear();
      fresh.external = false;
      fresh.asleep = true;
      fresh.owner = Owner::Softstep;
      m_resting.push_back(0u);
      count += 1u;
    }

    const uint32_t output = m_output[island];
    Island& found = islands[output];
    const bool rests = resting(bodies, ref);

    found.bodies.push_back(ref);
    found.asleep = found.asleep && rests;
    m_resting[output] |= rests ? 1u : 0u;

    return found;
  };

  for (uint32_t index = 0u; index != bodies.lattices.size(); ++index) {
    Island& island = entry(bodies.lattices[index].id, BodyRef{Side::Lattice, index});

    island.owner = Owner::Strainstep;
    island.external = island.external || hasAnchor(bodies.lattices[index]);
  }

  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    Island& island = entry(bodies.rigids[index].id, BodyRef{Side::Rigid, index});

    island.external = island.external || !(bodies.rigids[index].mass > 0.0);
  }

  for (uint32_t output = 0u; output != count; ++output) {
    Island& island = islands[output];

    if (m_resting[output] != 0u && !island.asleep) {
      for (const BodyRef& ref : island.bodies) {
        if (resting(bodies, ref)) {
          wakeBody(bodies, ref);
        }
      }
    }
  }

  islands.resize(count);

  for (uint32_t colour = 0u; colour != colourCount; ++colour) {
    for (const std::span<const uint32_t> slots : {cache.widePairs(colour), cache.otherPairs(colour)}) {
      for (const uint32_t slot : slots) {
        const CachedBodyPair& pair = cache.at(slot);

        if (pair.second.side == Side::Static) {
          islands[m_output[m_islandOf[pair.firstId]]].external = true;
        } else if (!heavy(bodies, pair.first) || !heavy(bodies, pair.second)) {
          islands[m_output[m_islandOf[pair.firstId]]].external = true;
          islands[m_output[m_islandOf[pair.secondId]]].external = true;
        }
      }
    }
  }

  for (const Joint& joint : joints.all()) {
    const uint32_t a = joint.def.bodyA;
    const uint32_t b = joint.def.bodyB;

    if (b >= m_islandOf.size() || m_islandOf[b] == noIsland) {
      continue;
    }

    if (a == noBody) {
      islands[m_output[m_islandOf[b]]].external = true;
    } else if (a < m_islandOf.size() && m_islandOf[a] != noIsland
               && (!heavyId(bodies, a) || !heavyId(bodies, b))) {
      islands[m_output[m_islandOf[a]]].external = true;
      islands[m_output[m_islandOf[b]]].external = true;
    }
  }

  for (uint32_t index = 0u; index != contacts.size(); ++index) {
    const Contact& contact = contacts[index];
    const ContactSide& side = contact.a.side != Side::Static ? contact.a : contact.b;

    if (side.side == Side::Static) {
      continue;
    }

    const uint32_t id = side.side == Side::Rigid ? bodies.rigids[side.body].id
                                                 : bodies.lattices[side.body].id;
    Island& island = islands[m_output[m_islandOf[id]]];

    island.contacts.push_back(index);

    if (contact.a.side == Side::Static || contact.b.side == Side::Static) {
      island.external = true;
    }
  }
}

uint32_t IslandStore::splits() const {
  return m_splits;
}

void IslandStore::requestSplit(uint32_t id) {
  m_splitBody = id;
}

void IslandStore::offerSplit(uint32_t id) {
  if (m_splitBody == noBody) {
    m_splitBody = id;
  }
}

uint32_t IslandStore::takeSplit() {
  const uint32_t id = std::exchange(m_splitBody, noBody);

  if (id >= m_islandOf.size() || m_islandOf[id] == noIsland) {
    return noIsland;
  }

  const uint32_t island = m_islandOf[id];

  return m_removed[island] != 0u && m_members[island].size() > 1u ? island : noIsland;
}

uint32_t IslandStore::pendingSplit() const {
  return m_splitBody;
}

uint32_t IslandStore::islandOf(uint32_t id) const {
  return id < m_islandOf.size() ? m_islandOf[id] : noIsland;
}

std::span<const uint32_t> IslandStore::members(uint32_t island) const {
  return m_members[island];
}

BodyRef IslandStore::refOf(uint32_t id) const {
  return m_ref[id];
}

uint32_t IslandStore::removed(uint32_t island) const {
  return m_removed[island];
}

bool IslandStore::visit(uint32_t island) {
  if (m_visited.size() < m_members.size()) {
    m_visited.resize(m_members.size(), 0u);
  }

  if (m_visited[island] == m_stamp) {
    return false;
  }

  m_visited[island] = m_stamp;

  return true;
}

}
