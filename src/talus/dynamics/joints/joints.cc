#include <talus/dynamics/joints/jointrecords.h>

#include <algorithm>

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

    if (entry != excluded_.end() && --entry->second == 0u) {
      excluded_.erase(entry);
    }
  }
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

bool JointSet::excludes(uint32_t a, uint32_t b) const {
  return !excluded_.empty() && excluded_.contains(pairKey(a, b));
}

bool JointSet::empty() const {
  return joints_.empty();
}

uint32_t JointRecords::add(JointSet& joints, const JointDef& def) {
  Joint joint;
  joint.id = joints.nextId_++;
  joint.def = def;
  joints.joints_.push_back(joint);
  joints.holdPair(def);

  return joint.id;
}

bool JointRecords::remove(JointSet& joints, uint32_t id) {
  const auto at = std::lower_bound(
    joints.joints_.begin(),
    joints.joints_.end(),
    id,
    [](const Joint& joint, uint32_t key) { return joint.id < key; }
  );

  if (at == joints.joints_.end() || at->id != id) {
    return false;
  }

  joints.releasePair(at->def);
  joints.joints_.erase(at);

  return true;
}

bool JointRecords::rebind(
  JointSet& joints,
  uint32_t id,
  uint32_t bodyA,
  uint32_t bodyB
) {
  Joint* joint = find(joints, id);

  if (joint == nullptr) {
    return false;
  }

  joints.releasePair(joint->def);
  joint->def.bodyA = bodyA;
  joint->def.bodyB = bodyB;
  joints.holdPair(joint->def);

  return true;
}

Joint* JointRecords::find(JointSet& joints, uint32_t id) {
  return const_cast<Joint*>(joints.find(id));
}

std::span<Joint> JointRecords::all(JointSet& joints) {
  return joints.joints_;
}

}
