#include <talus/dynamics/joints/joints.h>

#include <algorithm>
#include <utility>

namespace talus::dynamics {

namespace {

uint64_t pairKey(uint32_t a, uint32_t b) {
  const uint32_t low = std::min(a, b);
  const uint32_t high = std::max(a, b);
  return (static_cast<uint64_t>(low) << 32u) | high;
}

}

bool excludesPair(const JointDef& def) {
  return !def.collideConnected && def.bodyA != noBody && def.bodyB != noBody;
}

void JointSet::holdPair(const JointDef& def) {
  if (excludesPair(def)) {
    ++excluded_[pairKey(def.bodyA, def.bodyB)];
  }
}

void JointSet::releasePair(const JointDef& def) {
  if (excludesPair(def)) {
    const auto entry = excluded_.find(pairKey(def.bodyA, def.bodyB));

    if (--entry->second == 0u) {
      excluded_.erase(entry);
    }
  }
}

uint32_t JointSet::add(const JointDef& def) {
  Joint joint;
  joint.id = nextId_++;
  joint.def = def;
  joints_.push_back(joint);
  holdPair(def);

  return joint.id;
}

bool JointSet::remove(uint32_t id) {
  const auto at = std::lower_bound(
    joints_.begin(),
    joints_.end(),
    id,
    [](const Joint& joint, uint32_t key) { return joint.id < key; }
  );

  if (at == joints_.end() || at->id != id) {
    return false;
  }

  releasePair(at->def);
  joints_.erase(at);

  return true;
}

bool JointSet::rebind(uint32_t id, uint32_t bodyA, uint32_t bodyB) {
  Joint* joint = find(id);

  if (joint == nullptr) {
    return false;
  }

  releasePair(joint->def);
  joint->def.bodyA = bodyA;
  joint->def.bodyB = bodyB;
  holdPair(joint->def);

  return true;
}

Joint* JointSet::find(uint32_t id) {
  return const_cast<Joint*>(std::as_const(*this).find(id));
}

const Joint* JointSet::find(uint32_t id) const {
  const auto at = std::lower_bound(
    joints_.begin(),
    joints_.end(),
    id,
    [](const Joint& joint, uint32_t key) { return joint.id < key; }
  );

  return at != joints_.end() && at->id == id ? &*at : nullptr;
}

std::span<const Joint> JointSet::all() const {
  return joints_;
}

std::span<Joint> JointSet::all() {
  return joints_;
}

bool JointSet::excludes(uint32_t a, uint32_t b) const {
  return !excluded_.empty() && excluded_.contains(pairKey(a, b));
}

bool JointSet::empty() const {
  return joints_.empty();
}

}
