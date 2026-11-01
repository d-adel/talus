#include <catch2/catch_test_macros.hpp>

#include "bowlfixture.h"
#include "meshfixtures.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/meshbody/meshbody.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/scheduler/scheduler.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/tiers/tiers.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <utility>
#include <vector>

namespace {

using namespace talus::dynamics;
using meshfixture::Mesh;

const glm::dquat noTurn(1.0, 0.0, 0.0, 0.0);

struct Threads {
  std::shared_ptr<Scheduler> scheduler;
  Workers workers = resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);
};

struct Scanned {
  std::vector<CachedManifold> manifolds;
  std::vector<CachedManifoldPoint> built;
  std::vector<ContactMemory> memories;
};

Scanned scannedFill(
  const CachedBodyPair& old,
  std::span<const Contact> records,
  bool carry
) {
  Scanned out;
  const auto carried = [&](const CachedManifoldPoint& point) {
    ContactMemory memory;
    uint32_t matches = 0u;

    for (size_t index = 0u; index != old.points.size(); ++index) {
      const CachedManifoldPoint& before = old.manifoldPoints[index];

      if (before.key == point.key && before.firstChild == point.firstChild
          && before.secondChild == point.secondChild) {
        memory = old.points[index].memory;
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

    manifold.start = static_cast<uint32_t>(out.built.size());
    manifold.basis = records[first].basis;
    manifold.flipped =
      !(records[first].a.side == old.first.side
        && records[first].a.body == old.first.index);

    for (size_t index = first; index != records.size(); ++index) {
      const Contact& record = records[index];

      if (record.manifold != group) {
        continue;
      }

      CachedManifoldPoint built;

      built.key = record.key;
      built.localA = record.a.local;
      built.localB = record.b.local;
      built.builtSeparation = record.cached.separation;
      built.firstChild = manifold.flipped ? record.b.child : record.a.child;
      built.secondChild = manifold.flipped ? record.a.child : record.b.child;
      out.memories.push_back(carry ? carried(built) : record.memory);
      out.built.push_back(built);
    }

    manifold.count = static_cast<uint32_t>(out.built.size()) - manifold.start;

    if (carry) {
      double closest = sharedNormal;

      for (const CachedManifold& before : old.manifolds) {
        const double aligned = glm::dot(before.basis[0], manifold.basis[0]);

        if (before.flipped == manifold.flipped && aligned >= closest) {
          closest = aligned;
          manifold.twist = before.twist;
        }
      }
    }

    out.manifolds.push_back(manifold);
  }

  return out;
}

bool sameBits(double a, double b) {
  return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

bool sameBits(const glm::dvec3& a, const glm::dvec3& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z);
}

bool sameBits(const ContactMemory& a, const ContactMemory& b) {
  return sameBits(a.normal, b.normal) && sameBits(a.friction.x, b.friction.x)
         && sameBits(a.friction.y, b.friction.y)
         && sameBits(a.approach, b.approach) && sameBits(a.impulse, b.impulse);
}

uint32_t mismatches(const CachedBodyPair& pair, const Scanned& expected) {
  uint32_t wrong = 0u;

  if (pair.manifolds.size() != expected.manifolds.size()
      || pair.points.size() != expected.built.size()) {
    return 1u + static_cast<uint32_t>(pair.points.size());
  }

  for (size_t index = 0u; index != expected.manifolds.size(); ++index) {
    const CachedManifold& got = pair.manifolds[index];
    const CachedManifold& want = expected.manifolds[index];

    wrong += got.start == want.start && got.count == want.count
                 && got.flipped == want.flipped
                 && sameBits(got.basis[0], want.basis[0])
                 && sameBits(got.twist, want.twist)
               ? 0u
               : 1u;
  }

  for (size_t index = 0u; index != expected.built.size(); ++index) {
    const CachedManifoldPoint& got = pair.manifoldPoints[index];
    const CachedManifoldPoint& want = expected.built[index];

    wrong +=
      got.key == want.key && got.firstChild == want.firstChild
          && got.secondChild == want.secondChild
          && sameBits(got.localA, want.localA)
          && sameBits(got.localB, want.localB)
          && sameBits(pair.points[index].memory, expected.memories[index])
        ? 0u
        : 1u;
  }

  return wrong;
}

void label(CachedBodyPair& pair, uint32_t round) {
  for (size_t index = 0u; index != pair.points.size(); ++index) {
    const double value = static_cast<double>(index) + 1.0;
    ContactMemory& memory = pair.points[index].memory;

    memory.normal = 1.0e4 * static_cast<double>(round) + value;
    memory.friction = glm::dvec2(value + 0.25, -value - 0.5);
    memory.approach = 0.125 * value;
    memory.impulse = 2.0 * value;
  }

  for (size_t index = 0u; index != pair.manifolds.size(); ++index) {
    pair.manifolds[index].twist =
      index % 5u == 0u
        ? (index % 2u == 0u ? 0.0 : -0.0)
        : static_cast<double>(round) + 1.0e-3 * static_cast<double>(index + 1u);
  }
}

uint32_t rebuildAndCompare(
  ManifoldCache& cache,
  const BodySet& bodies,
  uint32_t slot,
  std::span<const Contact> records,
  uint32_t round,
  bool carry
) {
  label(cache.at(slot), round);

  const Scanned expected = scannedFill(cache.at(slot), records, carry);

  if (carry) {
    cache.rebuildManifolds(bodies, slot, records);
  } else {
    cache.replaceManifolds(bodies, slot, records);
  }

  return mismatches(cache.at(slot), expected);
}

double unit(std::mt19937_64& random) {
  return static_cast<double>(random() >> 11u) * 0x1.0p-53;
}

uint64_t below(std::mt19937_64& random, uint64_t count) {
  return random() % count;
}

glm::dvec3 point(std::mt19937_64& random) {
  const double x = unit(random);
  const double y = unit(random);
  const double z = unit(random);

  return glm::dvec3(x, y, z);
}

glm::dvec3 direction(std::mt19937_64& random) {
  return glm::normalize(point(random) - glm::dvec3(0.499, 0.5, 0.5));
}

std::vector<glm::dvec3> normalPool(std::mt19937_64& random) {
  std::vector<glm::dvec3> pool{
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(-0.0, 1.0, 0.0),
    glm::dvec3(0.0, 1.0, -0.0),
    glm::dvec3(0.0, -1.0, 0.0)
  };

  for (uint32_t base = 0u; base != 6u; ++base) {
    const glm::dvec3 normal = direction(random);

    pool.push_back(normal);

    for (const double angle : {1.0e-12, 1.0e-9, 1.0e-4, 0.05, 0.1405, 0.15}) {
      const glm::dvec3 axis =
        glm::normalize(glm::cross(normal, direction(random)));

      pool.push_back(glm::angleAxis(angle, axis) * normal);
    }
  }

  return pool;
}

std::vector<glm::dvec3> curvedPool(std::mt19937_64& random) {
  const glm::dvec3 axis = direction(random);
  std::vector<glm::dvec3> pool;

  for (uint32_t base = 0u; base != 1500u; ++base) {
    const glm::dvec3 across =
      glm::normalize(glm::cross(axis, direction(random)));
    const glm::dvec3 normal =
      glm::angleAxis(0.45 * unit(random), across) * axis;
    const glm::dvec3 side =
      glm::normalize(glm::cross(normal, direction(random)));

    pool.push_back(normal);

    for (const double angle : {0.1405, 0.1425}) {
      pool.push_back(glm::angleAxis(angle, side) * normal);
    }
  }

  return pool;
}

std::vector<Contact> randomRecords(
  std::mt19937_64& random,
  size_t count,
  const std::vector<glm::dvec3>& pool
) {
  constexpr std::array<uint32_t, 4> children{noChild, 0u, 1u, 7u};
  std::vector<uint32_t> ids(count / 3u + 1u);

  for (uint32_t& id : ids) {
    id = random() % 2u == 0u
           ? static_cast<uint32_t>(below(random, 2u * count + 1u))
           : static_cast<uint32_t>(random());
  }

  std::vector<Contact> records(count);

  for (Contact& record : records) {
    const bool swapped = random() % 2u == 0u;
    const glm::dvec3 normal = pool[below(random, pool.size())];

    record.a.side = Side::Rigid;
    record.a.body = swapped ? 1u : 0u;
    record.a.child = children[below(random, children.size())];
    record.a.local = point(random);
    record.b.side = Side::Rigid;
    record.b.body = swapped ? 0u : 1u;
    record.b.child = children[below(random, children.size())];
    record.b.local = point(random);
    record.basis[0] = normal;
    record.gap = unit(random) - 0.5;
    record.key = below(random, count / 2u + 2u);
    record.manifold = ids[below(random, ids.size())];
    record.memory.normal = unit(random);
    record.memory.friction = glm::dvec2(point(random));
    record.cached.separation = unit(random);
  }

  return records;
}

void shuffle(std::mt19937_64& random, std::vector<Contact>& records) {
  for (size_t index = records.size(); index > 1u; --index) {
    std::swap(records[index - 1u], records[below(random, index)]);
  }
}

BodySet twoBoxes() {
  BodySet bodies;
  CompoundChild cube;

  cube.half = glm::dvec3(0.05);
  cube.density = 1000.0;

  for (const double height : {0.05, 0.15}) {
    (void)addCompoundBody(
      bodies,
      std::vector<CompoundChild>{cube},
      glm::dvec3(0.0, height, 0.0),
      noTurn,
      glm::dvec3(0.0),
      0.5,
      0.0,
      false
    );
  }

  return bodies;
}

struct Slab {
  glm::vec3 half{0.0f};
  double mass = 0.0;
  double friction = 0.0;
  glm::dvec3 at{0.0};
  glm::dquat turn{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 velocity{0.0};
};

std::array<Slab, 3> slabs() {
  return {
    Slab{
      {0.2f, 0.04f, 0.2f},
      40.0,
      0.0,
      {-1.2, 0.04, 0.0},
      noTurn,
      {1.0, 0.0, 0.0}
    },
    Slab{
      {0.4f, 0.05f, 0.4f},
      300.0,
      0.5,
      {0.8, 0.05, 0.0},
      noTurn,
      {0.0, 0.0, 0.0}
    },
    Slab{
      {0.15f, 0.04f, 0.15f},
      10.0,
      0.02,
      {0.6, 0.141, 0.1},
      glm::angleAxis(0.3, glm::dvec3(0.0, 1.0, 0.0)),
      {0.5, 0.0, -0.2}
    }
  };
}

const std::array<MeshShape, 3>& slabShapes() {
  static const std::array<MeshShape, 3> shapes = [] {
    const Threads threads;
    std::array<MeshShape, 3> made;

    for (size_t index = 0u; index != made.size(); ++index) {
      const Slab slab = slabs()[index];
      const Mesh mesh = meshfixture::box(-slab.half, slab.half);
      MeshShapeSettings settings;

      settings.mass = slab.mass;
      settings.friction = slab.friction;
      made[index] =
        meshShape(mesh.vertices, mesh.indices, settings, threads.workers);
    }

    return made;
  }();

  return shapes;
}

void addSlidingSlabs(World& world) {
  CompoundChild floor;

  floor.half = glm::dvec3(3.0, 0.5, 3.0);
  floor.density = 1000.0;
  (void)addCompoundBody(
    world.bodies,
    std::vector<CompoundChild>{floor},
    glm::dvec3(0.0, -0.5, 0.0),
    noTurn,
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  );

  for (size_t index = 0u; index != slabs().size(); ++index) {
    const Slab slab = slabs()[index];

    (void)addMeshBody(
      world.bodies,
      slabShapes()[index],
      slab.at,
      slab.turn,
      slab.velocity
    );
  }
}

uint32_t largestPair(const World& world) {
  uint32_t largest = 0u;

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (world.manifolds.live(slot)) {
      largest = std::max(
        largest,
        static_cast<uint32_t>(world.manifolds.at(slot).points.size())
      );
    }
  }

  return largest;
}

struct Replayed {
  uint32_t compared = 0u;
  uint32_t largest = 0u;
};

Replayed replayAgainstScan(World& world, uint32_t frames) {
  std::mt19937_64 random(7u);
  ManifoldCache replay;
  Replayed replayed;

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);

    for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
      if (!world.manifolds.live(slot)
          || world.manifolds.at(slot).points.empty()) {
        continue;
      }

      const CachedBodyPair& pair = world.manifolds.at(slot);

      replay.createPairs(
        world.bodies,
        std::vector<BodyPair>{{pair.first, pair.second}}
      );

      const uint32_t mine = replay.find(pair.pair);

      REQUIRE(mine != noSlot);

      std::vector<Contact> records = world.manifolds.records(slot);

      if (frame % 2u == 1u) {
        shuffle(random, records);
      }

      replayed.compared += 1u;
      replayed.largest =
        std::max(replayed.largest, static_cast<uint32_t>(records.size()));
      CAPTURE(frame, slot, records.size());
      CHECK(
        rebuildAndCompare(
          replay,
          world.bodies,
          mine,
          records,
          replayed.compared,
          true
        )
        == 0u
      );
    }
  }

  return replayed;
}
}

TEST_CASE(
  "rebuilt manifolds carry the same memory and twist as a full scan of the old "
  "ones",
  "[dynamics][manifoldcache]"
) {
  const BodySet bodies = twoBoxes();
  const BodyRef first{Side::Rigid, 0u};
  const BodyRef second{Side::Rigid, 1u};
  std::mt19937_64 random(20261101u);
  const std::vector<glm::dvec3> pool = normalPool(random);
  ManifoldCache cache;

  cache.sync(bodies, std::vector<BodyPair>{{first, second}});

  const uint32_t slot = cache.find(pairKey(bodies, first, second));

  REQUIRE(slot != noSlot);

  constexpr std::array<size_t, 16> sizes{
    0u,
    1u,
    3u,
    8u,
    32u,
    33u,
    40u,
    300u,
    1500u,
    5u,
    32u,
    2000u,
    12u,
    0u,
    64u,
    33u
  };
  uint32_t round = 0u;

  for (uint32_t pass = 0u; pass != 4u; ++pass) {
    for (const size_t size : sizes) {
      round += 1u;

      const std::vector<Contact> records = randomRecords(random, size, pool);
      const bool carry = round % 7u != 0u;

      CAPTURE(round, size, carry);
      CHECK(
        rebuildAndCompare(cache, bodies, slot, records, round, carry) == 0u
      );
    }
  }
}

TEST_CASE(
  "rebuilt manifolds of curved contacts carry the same twist as a full scan "
  "of the old ones",
  "[dynamics][manifoldcache]"
) {
  const BodySet bodies = twoBoxes();
  const BodyRef first{Side::Rigid, 0u};
  const BodyRef second{Side::Rigid, 1u};
  std::mt19937_64 random(20261102u);
  const std::vector<glm::dvec3> pool = curvedPool(random);
  ManifoldCache cache;

  cache.sync(bodies, std::vector<BodyPair>{{first, second}});

  const uint32_t slot = cache.find(pairKey(bodies, first, second));

  REQUIRE(slot != noSlot);

  uint32_t round = 0u;

  for (const size_t size : {300u, 1500u, 2000u, 40u, 800u, 2000u}) {
    round += 1u;

    std::vector<Contact> records = randomRecords(random, size, pool);

    for (uint32_t index = 0u; index != records.size(); ++index) {
      records[index].manifold = index;
    }

    CAPTURE(round, size);
    CHECK(rebuildAndCompare(cache, bodies, slot, records, round, true) == 0u);
  }
}

TEST_CASE(
  "manifolds rebuilt from sliding mesh bodies match a full scan of the old "
  "ones",
  "[dynamics][manifoldcache]"
) {
  World world;

  addSlidingSlabs(world);

  const Replayed replayed = replayAgainstScan(world, 40u);

  CAPTURE(replayed.compared, replayed.largest);
  CHECK(replayed.compared >= 80u);
  CHECK(replayed.largest > 300u);
}

TEST_CASE(
  "manifolds rebuilt from a ball spinning in a bowl match a full scan of the "
  "old ones",
  "[dynamics][manifoldcache]"
) {
  const talus::field::SampledSdf bowl = bowlfixture::field(0.30f);
  const Threads threads;
  const Mesh sphere = meshfixture::icosphere(0.25f, 4u);
  MeshShapeSettings settings;

  settings.mass = 50.0;
  settings.friction = 0.0;

  const MeshShape ball =
    meshShape(sphere.vertices, sphere.indices, settings, threads.workers);
  const glm::dvec3 bowlAt(0.0, static_cast<double>(bowlfixture::half.y), 0.0);
  World world;

  (void)bowlfixture::addBowl(world.bodies, bowl, bowlAt, 500.0, 0.0);

  const uint32_t id = addMeshBody(
    world.bodies,
    ball,
    bowlAt + glm::dvec3(bowlfixture::cavity(0.30f))
      - glm::dvec3(0.0, 0.0505, 0.0),
    noTurn,
    glm::dvec3(0.0)
  );
  const std::optional<BodyRef> spun = findBody(world.bodies, id);

  REQUIRE(spun.has_value());
  world.bodies.rigids[spun->index].angularVelocity = glm::dvec3(0.0, 1.0, 3.0);
  world.parameters.gravity = glm::dvec3(0.0);

  const Replayed replayed = replayAgainstScan(world, 8u);

  CAPTURE(replayed.compared, replayed.largest);
  CHECK(replayed.compared >= 8u);
  CHECK(replayed.largest > 1000u);
}

TEST_CASE(
  "sliding mesh bodies step identically across workers and widths",
  "[dynamics][manifoldcache]"
) {
  const auto run = [](uint32_t workers, uint32_t width) {
    World world;
    uint32_t largest = 0u;

    world.workerCount = workers;
    world.simdWidth = width;
    addSlidingSlabs(world);

    for (uint32_t frame = 0u; frame != 30u; ++frame) {
      REQUIRE_FALSE(step(world).nonFinite);
      largest = std::max(largest, largestPair(world));
    }

    return std::pair{world.bodies.rigids, largest};
  };
  const auto [serial, largest] = run(1u, 1u);

  CAPTURE(largest);
  REQUIRE(largest > 300u);

  std::vector<uint32_t> widths{1u, 4u};

  if (hasAvx2()) {
    widths.push_back(8u);
  }

  for (const uint32_t workers : {1u, 4u, 8u}) {
    for (const uint32_t width : widths) {
      CAPTURE(workers, width);

      const std::vector<RigidBody> bodies = run(workers, width).first;

      REQUIRE(bodies.size() == serial.size());

      for (size_t index = 0u; index != serial.size(); ++index) {
        CAPTURE(index);
        CHECK(sameBits(bodies[index].position, serial[index].position));
        CHECK(sameBits(bodies[index].velocity, serial[index].velocity));
        CHECK(
          sameBits(bodies[index].angularVelocity, serial[index].angularVelocity)
        );
        CHECK(sameBits(bodies[index].rotation.x, serial[index].rotation.x));
        CHECK(sameBits(bodies[index].rotation.y, serial[index].rotation.y));
        CHECK(sameBits(bodies[index].rotation.z, serial[index].rotation.z));
        CHECK(sameBits(bodies[index].rotation.w, serial[index].rotation.w));
      }
    }
  }
}
