#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/continuous/continuous.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/joints/joints.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <unordered_set>
#include <vector>

namespace talus::dynamics {

struct SoftstepArena;
struct StrainstepArena;

struct PairWork {
  double recycle = 0.0;
  double midphase = 0.0;
  double narrowphase = 0.0;
  double reduce = 0.0;
  double build = 0.0;
  uint32_t tested = 0u;
  uint32_t recycled = 0u;
  uint32_t childPairs = 0u;
  uint32_t redone = 0u;
};

struct StepProfile {
  double travel = 0.0;
  double broadphase = 0.0;
  double storeSync = 0.0;
  double storeUpdate = 0.0;
  double lattice = 0.0;
  double islands = 0.0;
  double solverSetup = 0.0;
  double gather = 0.0;
  double prepare = 0.0;
  double substeps = 0.0;
  double integrate = 0.0;
  double restitute = 0.0;
  double storeBack = 0.0;
  double joints = 0.0;
  double continuous = 0.0;
  double events = 0.0;
  double sleep = 0.0;
  double total = 0.0;
  PairWork pairs;
};

struct StepReport {
  uint32_t substeps = 0u;
  uint32_t softstepSubsteps = 0u;
  uint32_t jointSubsteps = 0u;
  uint32_t contacts = 0u;
  uint32_t activeContacts = 0u;
  uint32_t softstepIslands = 0u;
  uint32_t strainstepIslands = 0u;
  uint32_t widePairs = 0u;
  uint32_t jointRows = 0u;
  uint32_t limitPassRows = 0u;
  uint32_t promotions = 0u;
  uint32_t demotions = 0u;
  uint32_t thawed = 0u;
  bool nonFinite = false;
  StepProfile profile;
  std::vector<ContactHitEvent> hits;
};

struct AwakeList {
  std::vector<uint32_t> indices;
  size_t rigidCount = 0u;
  size_t latticeCount = 0u;
  uint64_t layout = 0u;
  double band = 0.0;
};

struct StepBuffers {
  std::vector<double> travel;
  std::vector<RigidPose> start;
  std::optional<std::vector<uint32_t>> anchors;
};

struct World {
  BodySet bodies;
  std::vector<Contact> contacts;
  std::vector<CarriedForce> carried;
  ManifoldCache manifolds;
  BroadPhase broadPhase;
  IslandStore islandStore;
  JointSet joints;
  uint32_t workerCount = 1u;
  EnqueueTaskCallback* enqueueTask = nullptr;
  FinishTaskCallback* finishTask = nullptr;
  void* userTaskContext = nullptr;
  std::shared_ptr<Scheduler> scheduler;
  uint32_t simdWidth = 0u;
  std::shared_ptr<SoftstepArena> softstepArena;
  std::shared_ptr<StrainstepArena> strainstepArena;
  std::vector<Island> islands;
  std::vector<uint32_t> movers;
  std::vector<uint32_t> thawOnContact;
  IntegrationParameters parameters;
  double time = 0.0;
  AwakeList awake;
  StepBuffers buffers;
};

[[nodiscard]] StepReport step(World& world);
[[nodiscard]] std::span<const uint32_t> awakeRigids(const World& world);
[[nodiscard]] bool awakeSuperset(
  const World& world,
  std::vector<uint32_t>& rigids
);
[[nodiscard]] std::unordered_set<uint64_t> recordedPairs(std::span<const Contact> contacts);
[[nodiscard]] std::vector<Contact> contactRecords(const World& world);
[[nodiscard]] std::vector<Contact> contactsOf(const World& world, const BodyRef& ref);
[[nodiscard]] std::vector<Contact> contactsOf(
  const World& world,
  const BodyRef& ref,
  const std::unordered_set<uint64_t>& recorded
);

}
