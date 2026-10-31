#include "pipeline.h"

#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/events/events.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/continuous/continuous.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/projection/projection.h>
#include <talus/dynamics/scheduler/stopwatch.h>
#include <talus/dynamics/softstep/softstep.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/strainstep/strainstep.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <unordered_set>
#include <utility>
#include <vector>

namespace talus::dynamics {

namespace {

std::vector<double> frameTravel(
  const BodySet& bodies,
  double dt,
  const Workers& workers,
  std::vector<uint32_t>& movers
) {
  const uint32_t lattices = static_cast<uint32_t>(bodies.lattices.size());
  std::vector<double> travel(bodies.lattices.size() + bodies.rigids.size(), 0.0);
  std::vector<std::vector<uint32_t>> found(std::max(workers.count, 1u));

  parallelFor(workers, static_cast<uint32_t>(travel.size()), 1024u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
    for (uint32_t index = begin; index != end; ++index) {
      if (index < lattices) {
        travel[index] = speedOf(bodies, BodyRef{Side::Lattice, index}) * dt;
        continue;
      }

      const RigidBody& body = bodies.rigids[index - lattices];
      const bool still = body.velocity == glm::dvec3(0.0) && body.angularVelocity == glm::dvec3(0.0);

      if (!body.asleep && (body.mass > 0.0 || !still)) {
        found[worker].push_back(body.id);
      }

      if (!still) {
        travel[index] = speedOf(bodies, BodyRef{Side::Rigid, index - lattices}) * dt;
      }
    }
  });

  movers.clear();

  for (const std::vector<uint32_t>& list : found) {
    movers.insert(movers.end(), list.begin(), list.end());
  }

  return travel;
}

std::optional<std::vector<uint32_t>> activeSlots(
  const World& world,
  std::span<const uint32_t> movers
) {
  if (world.broadPhase.fixedChanged()) {
    return std::nullopt;
  }

  const std::span<const uint32_t> changed = world.broadPhase.changedProxies();
  std::vector<uint64_t> seen((static_cast<size_t>(world.bodies.nextId) + 63u) / 64u, 0u);
  std::vector<uint32_t> seeds;

  for (const std::span<const uint32_t> ids : {movers, std::span<const uint32_t>(world.movers), changed}) {
    for (const uint32_t id : ids) {
      uint64_t& word = seen[id / 64u];
      const uint64_t bit = uint64_t{1} << (id % 64u);

      if ((word & bit) == 0u) {
        word |= bit;
        seeds.push_back(id);
      }
    }
  }

  if (4u * seeds.size() > world.bodies.rigids.size()) {
    return std::nullopt;
  }

  const ManifoldCache& cache = world.manifolds;
  std::vector<uint64_t> marks((static_cast<size_t>(cache.size()) + 63u) / 64u, 0u);

  for (const uint32_t id : seeds) {
    for (const uint32_t slot : cache.pairsOf(id)) {
      marks[slot / 64u] |= uint64_t{1} << (slot % 64u);
    }
  }

  std::vector<uint32_t> slots;

  for (size_t word = 0u; word != marks.size(); ++word) {
    for (uint64_t bits = marks[word]; bits != 0u; bits &= bits - 1u) {
      slots.push_back(static_cast<uint32_t>(64u * word) + static_cast<uint32_t>(std::countr_zero(bits)));
    }
  }

  return slots;
}

void shareStoredForces(World& world) {
  for (const CarriedForce& share : world.carried) {
    const uint32_t slot = world.manifolds.find(share.pair);

    if (slot == noSlot) {
      continue;
    }

    std::vector<Contact> records = world.manifolds.records(slot);

    (void)shareCarriedForces(std::span<const CarriedForce>(&share, 1u), records, world.bodies);
    world.manifolds.writeMemory(slot, records);
  }
}

bool finite(const BodySet& bodies, const Workers& workers) {
  for (const LatticeBody& body : bodies.lattices) {
    for (const glm::dvec3& position : body.position) {
      if (!std::isfinite(position.x + position.y + position.z)) {
        return false;
      }
    }
  }

  std::atomic<bool> bad{false};

  parallelFor(workers, static_cast<uint32_t>(bodies.rigids.size()), 4096u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t index = begin; index != end; ++index) {
      const RigidBody& body = bodies.rigids[index];

      if (!std::isfinite(body.position.x + body.position.y + body.position.z + body.rotation.w)) {
        bad.store(true, std::memory_order_relaxed);
        return;
      }
    }
  });

  return !bad.load(std::memory_order_relaxed);
}

std::vector<uint8_t> jointedRigids(
  const World& world,
  std::span<const Island* const> islands
) {
  if (world.joints.empty()) {
    return {};
  }

  const BodySet& bodies = world.bodies;
  std::vector<uint8_t> ends(bodies.rigids.size(), 0u);

  for (const Joint& joint : world.joints.all()) {
    for (const uint32_t id : {joint.def.bodyA, joint.def.bodyB}) {
      if (id == noBody) {
        continue;
      }

      const BodyRef ref = world.islandStore.refOf(id);

      if (ref.side == Side::Rigid && ref.index < bodies.rigids.size()
          && bodies.rigids[ref.index].id == id) {
        ends[ref.index] = 1u;
      }
    }
  }

  std::vector<uint8_t> jointed(bodies.rigids.size(), 0u);
  bool any = false;

  for (const Island* island : islands) {
    const bool linked = std::any_of(
      island->bodies.begin(),
      island->bodies.end(),
      [&](const BodyRef& ref) {
        return ref.side == Side::Rigid && ends[ref.index] != 0u;
      }
    );

    if (!linked) {
      continue;
    }

    any = true;

    for (const BodyRef& ref : island->bodies) {
      if (ref.side == Side::Rigid) {
        jointed[ref.index] = 1u;
      }
    }
  }

  return any ? jointed : std::vector<uint8_t>{};
}

std::vector<uint32_t> kinematicGuests(
  const World& world,
  std::span<const uint32_t> unjointed,
  std::span<const uint8_t> jointed
) {
  const BodySet& bodies = world.bodies;
  const ManifoldCache& cache = world.manifolds;
  const auto jointedHeavy = [&](const BodyRef& ref) {
    return ref.side == Side::Rigid && jointed[ref.index] != 0u
           && bodies.rigids[ref.index].mass > 0.0;
  };
  std::vector<uint32_t> guests;

  for (const uint32_t index : unjointed) {
    const RigidBody& body = bodies.rigids[index];

    if (body.mass > 0.0) {
      continue;
    }

    bool touches = jointed[index] != 0u;

    for (const uint32_t slot : cache.pairsOf(body.id)) {
      if (touches) {
        break;
      }

      const CachedBodyPair& pair = cache.at(slot);

      touches = cache.live(slot)
                && (jointedHeavy(pair.first) || jointedHeavy(pair.second));
    }

    if (touches) {
      guests.push_back(index);
    }
  }

  return guests;
}

void wakeMovedPartners(World& world) {
  BodySet& bodies = world.bodies;
  const auto rigidOf = [&](uint32_t id) -> std::optional<BodyRef> {
    if (id == noBody) {
      return std::nullopt;
    }

    const BodyRef ref = world.islandStore.refOf(id);

    if (ref.side != Side::Rigid || ref.index >= bodies.rigids.size()
        || bodies.rigids[ref.index].id != id) {
      return std::nullopt;
    }

    return ref;
  };
  const auto moving = [&](const std::optional<BodyRef>& ref) {
    if (!ref) {
      return false;
    }

    const RigidBody& body = bodies.rigids[ref->index];

    return !(body.mass > 0.0) && !body.asleep
           && (body.velocity != glm::dvec3(0.0) || body.angularVelocity != glm::dvec3(0.0));
  };

  for (const Joint& joint : world.joints.all()) {
    const std::optional<BodyRef> a = rigidOf(joint.def.bodyA);
    const std::optional<BodyRef> b = rigidOf(joint.def.bodyB);

    if (b && moving(a)) {
      wakeBody(bodies, *b);
    }

    if (a && moving(b)) {
      wakeBody(bodies, *a);
    }
  }
}

void addSoftstepWork(StepProfile& total, const StepProfile& part) {
  total.gather += part.gather;
  total.prepare += part.prepare;
  total.substeps += part.substeps;
  total.integrate += part.integrate;
  total.restitute += part.restitute;
  total.storeBack += part.storeBack;
  total.joints += part.joints;
}

void addTouching(SoftstepTouching& total, const SoftstepTouching& part) {
  total.points += part.points;
  total.active += part.active;
  total.hits.insert(total.hits.end(), part.hits.begin(), part.hits.end());
  std::sort(total.hits.begin(), total.hits.end());
  total.hits.erase(
    std::unique(total.hits.begin(), total.hits.end()),
    total.hits.end()
  );
}
}

std::unordered_set<uint64_t> recordedPairs(std::span<const Contact> contacts) {
  std::unordered_set<uint64_t> pairs;

  pairs.reserve(contacts.size());

  for (const Contact& contact : contacts) {
    pairs.insert(contact.pair);
  }

  return pairs;
}

std::vector<Contact> contactRecords(const World& world) {
  const std::unordered_set<uint64_t> recorded = recordedPairs(world.contacts);
  std::vector<Contact> records = world.contacts;

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot)
        || recorded.contains(world.manifolds.at(slot).pair)) {
      continue;
    }

    const std::vector<Contact> converted = world.manifolds.records(slot);

    records.insert(records.end(), converted.begin(), converted.end());
  }

  return records;
}

std::vector<Contact> contactsOf(const World& world, const BodyRef& ref) {
  return contactsOf(world, ref, recordedPairs(world.contacts));
}

std::vector<Contact> contactsOf(
  const World& world,
  const BodyRef& ref,
  const std::unordered_set<uint64_t>& recorded
) {
  std::vector<Contact> found;

  for (const Contact& contact : world.contacts) {
    if ((contact.a.side == ref.side && contact.a.body == ref.index)
        || (contact.b.side == ref.side && contact.b.body == ref.index)) {
      found.push_back(contact);
    }
  }

  if (ref.side != Side::Rigid) {
    return found;
  }

  for (const uint32_t slot : world.manifolds.pairsOf(world.bodies.rigids[ref.index].id)) {
    if (recorded.contains(world.manifolds.at(slot).pair)) {
      continue;
    }

    const std::vector<Contact> converted = world.manifolds.records(slot);

    found.insert(found.end(), converted.begin(), converted.end());
  }

  return found;
}

StepReport step(World& world) {
  Stopwatch clock;
  StepReport report;

  if (!world.parameters.sleep) {
    wakeAll(world.bodies);
  }

  switchTiers(world, report);
  BodySet& bodies = world.bodies;
  const IntegrationParameters& parameters = world.parameters;
  const Workers workers = resolveWorkers(
    world.workerCount,
    world.enqueueTask,
    world.finishTask,
    world.userTaskContext,
    world.scheduler
  );
  const uint32_t simdWidth = resolveSimdWidth(world.simdWidth);
  const double band = parameters.speculativeDistance + parameters.margin;
  std::vector<uint32_t> movers;
  const std::vector<double> travel = frameTravel(bodies, parameters.dt, workers, movers);

  report.profile.travel = clock.lap();

  world.broadPhase.update(bodies, parameters, travel, workers, &world.manifolds);
  world.manifolds.createPairs(
    bodies,
    world.broadPhase.created(),
    &world.joints
  );

  const std::span<const BodyPair> latticePairs = world.broadPhase.latticePairs();
  std::vector<Contact> sleeping;

  for (const Contact& contact : world.contacts) {
    if (resting(bodies, BodyRef{contact.a.side, contact.a.body})
        && resting(bodies, BodyRef{contact.b.side, contact.b.body})) {
      sleeping.push_back(contact);
    }
  }

  const std::optional<std::vector<uint32_t>> walked = activeSlots(world, movers);

  world.movers = std::move(movers);
  report.profile.broadphase = clock.lap();
  const PairEvents events = updateCachedPairs(
    bodies,
    world.manifolds,
    band,
    travel,
    parameters.dt,
    workers,
    &world.broadPhase,
    parameters.timePairs ? &report.profile.pairs : nullptr,
    walked ? &*walked : nullptr
  );
  report.profile.storeUpdate = clock.lap();

  std::vector<Contact> contacts =
    narrowphase(bodies, latticePairs, band, travel, workers);

  carryMemory(contacts, world.contacts);
  (void)shareCarriedForces(world.carried, contacts, bodies);
  shareStoredForces(world);
  contacts.insert(contacts.end(), sleeping.begin(), sleeping.end());
  report.thawed = thawStruck(world, parameters.thawSpeed);
  report.profile.lattice = clock.lap();

  world.islandStore.sync(bodies, workers);

  for (const uint32_t id : events.lost) {
    world.islandStore.markRemoved(id);
  }

  world.manifolds.recolour(bodies, events.changed);
  world.islandStore.build(
    bodies,
    world.manifolds,
    events.changed,
    !events.separated.empty(),
    contacts,
    world.islands,
    world.joints
  );
  world.manifolds.recolour(bodies, world.manifolds.syncBodies(bodies, workers));

  std::vector<Island>& islands = world.islands;
  std::vector<RigidPose> start;
  std::vector<uint32_t> stateOf(bodies.rigids.size(), 0u);
  uint32_t points = 0u;
  uint32_t active = 0u;

  start.resize(bodies.rigids.size());
  parallelFor(workers, static_cast<uint32_t>(bodies.rigids.size()), 1024u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t index = begin; index != end; ++index) {
      start[index] = {bodies.rigids[index].position, bodies.rigids[index].rotation};
    }
  });

  std::vector<const Island*> rigidIslands;
  std::vector<const Island*> strainIslands;
  std::vector<uint8_t> latticeOwned(bodies.rigids.size(), 0u);

  for (const Island& island : islands) {
    if (island.asleep) {
      continue;
    }

    if (island.owner == Owner::Softstep) {
      rigidIslands.push_back(&island);
      report.softstepIslands += 1u;
      continue;
    }

    strainIslands.push_back(&island);
    report.strainstepIslands += 1u;

    for (const BodyRef& ref : island.bodies) {
      if (ref.side == Side::Rigid) {
        latticeOwned[ref.index] = 1u;
      }
    }
  }

  const StartVelocities startSpeed =
    bodies.lattices.empty() ? StartVelocities{} : startVelocities(bodies);

  if (!world.softstepArena) {
    world.softstepArena = makeSoftstepArena();
  }

  std::vector<uint8_t> softstep(bodies.rigids.size(), 0u);
  std::vector<uint32_t> rigids;

  for (const Island* island : rigidIslands) {
    for (const BodyRef& ref : island->bodies) {
      if (ref.side == Side::Rigid) {
        softstep[ref.index] = 1u;
      }
    }
  }

  for (uint32_t index = 0u; index != softstep.size(); ++index) {
    if (softstep[index] != 0u) {
      rigids.push_back(index);
    }
  }
  report.profile.islands = clock.lap();
  SoftstepTouching softstepTouching;

  const uint32_t splitIsland = world.islandStore.takeSplit();
  const std::function<void()> alongside = [&] {
    if (splitIsland != noIsland) {
      (void)world.islandStore.split(
        bodies,
        splitIsland,
        world.manifolds,
        contacts,
        world.joints
      );
    }

    world.broadPhase.rebuildTrees();
  };

  const std::vector<uint8_t> jointed = jointedRigids(world, rigidIslands);
  std::vector<uint32_t> jointedSet;
  std::vector<uint32_t> freeSet;

  if (!jointed.empty()) {
    for (const uint32_t index : rigids) {
      (jointed[index] != 0u && bodies.rigids[index].mass > 0.0 ? jointedSet
                                                               : freeSet)
        .push_back(index);
    }
  }

  const bool separate = !jointedSet.empty();
  const std::vector<uint32_t> guests =
    separate ? kinematicGuests(world, freeSet, jointed)
             : std::vector<uint32_t>{};
  const std::span<const uint32_t> unjointed =
    separate ? std::span<const uint32_t>(freeSet)
             : std::span<const uint32_t>(rigids);
  StepProfile jointedProfile;
  SoftstepTouching jointedTouching;

  if (separate) {
    IntegrationParameters jointedParameters = parameters;

    jointedParameters.softstepSubsteps = parameters.jointSubsteps;

    const SoftstepResult jointedRun = solveSoftstep(
      bodies,
      world.manifolds,
      jointedSet,
      jointedParameters,
      stateOf,
      workers,
      simdWidth,
      world.softstepArena.get(),
      &jointedProfile,
      &jointedTouching,
      nullptr,
      &world.joints,
      &world.islandStore,
      guests,
      true,
      true
    );

    report.widePairs += jointedRun.widePairs;
    report.jointSubsteps = jointedRun.substeps;
    report.jointRows += jointedRun.jointRows;
    report.limitPassRows += jointedRun.limitPassRows;
  }

  const SoftstepResult freeRun = solveSoftstep(
    bodies,
    world.manifolds,
    unjointed,
    parameters,
    stateOf,
    workers,
    simdWidth,
    world.softstepArena.get(),
    &report.profile,
    &softstepTouching,
    &alongside,
    separate ? nullptr : &world.joints,
    separate ? nullptr : &world.islandStore,
    {},
    separate
  );

  report.widePairs += freeRun.widePairs;
  report.jointRows += freeRun.jointRows;
  report.limitPassRows += freeRun.limitPassRows;

  if (!unjointed.empty()) {
    report.softstepSubsteps = freeRun.substeps;
  }

  if (separate) {
    addSoftstepWork(report.profile, jointedProfile);
    addTouching(softstepTouching, jointedTouching);
  }

  const double solve = clock.lap();

  report.profile.solverSetup = solve - report.profile.gather - report.profile.prepare
                               - report.profile.substeps - report.profile.storeBack;

  if (parameters.continuous) {
    const std::vector<uint32_t> anchors = sweepAnchors(bodies, workers);

    parallelFor(workers, static_cast<uint32_t>(rigids.size()), 64u, [&](uint32_t begin, uint32_t end, uint32_t) {
      for (uint32_t index = begin; index != end; ++index) {
        solveContinuousBody(bodies, start, rigids[index], parameters, anchors);
      }
    });
  }

  report.profile.continuous = clock.lap();

  report.profile.events = clock.lap();

  std::vector<Contact> lastContacts;

  world.time += parameters.dt;

  if (world.strainstepArena) {
    forgetLattices(*world.strainstepArena, bodies);
  }

  if (!strainIslands.empty()) {
    if (!world.strainstepArena) {
      world.strainstepArena = makeStrainstepArena();
    }

    for (const Island* island : strainIslands) {
      const StrainstepResult strained = solveStrainstep(
        bodies,
        world.manifolds,
        contacts,
        *island,
        parameters,
        *world.strainstepArena,
        workers,
        &world.joints,
        &world.islandStore,
        start
      );

      report.substeps = std::max(report.substeps, strained.substeps);
      report.jointRows += strained.jointRows;
      report.limitPassRows += strained.limitPassRows;

      for (const uint32_t index : island->contacts) {
        lastContacts.push_back(contacts[index]);
      }
    }
  }

  world.contacts = std::move(lastContacts);
  report.profile.lattice += clock.lap();

  for (const Contact& contact : sleeping) {
    if (resting(bodies, BodyRef{contact.a.side, contact.a.body})
        && resting(bodies, BodyRef{contact.b.side, contact.b.body})) {
      world.contacts.push_back(contact);
    }
  }

  report.hits = gatherHits(world, startSpeed, latticeOwned, softstepTouching.hits);
  points = softstepTouching.points;
  active = softstepTouching.active;
  report.contacts = points + static_cast<uint32_t>(world.contacts.size());
  report.activeContacts += active;
  report.nonFinite = report.nonFinite || !finite(bodies, workers);
  report.profile.events += clock.lap();

  if (parameters.sleep) {
    world.islandStore.linkContacts(bodies, world.contacts);
    wakeMovedPartners(world);
    settleIslands(bodies, world.manifolds, world.islandStore, islands, start, parameters);
  }

  world.carried.clear();
  report.profile.sleep = clock.lap();
  report.profile.total = clock.total();

  return report;
}

}
