#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"
#include "plasticbar.h"
#include "windowmonitor.h"

#include <talus/deformable/impactwindow/impactwindow.h>
#include <talus/deformable/material/material.h>
#include <talus/deformable/solver/solver.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/world/world.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/physics/physicsworld/physicsworld.h>
#include <talus/runtime/runtime/runtime.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <vector>

using talus::deformable::ImpactWindow;
using talus::deformable::WindowGroup;
using talus::deformable::WindowMember;
using talus::deformable::WindowPartner;
using talus::deformable::WindowStep;
using talus::objectmodel::DeformableState;
using talus::objectmodel::Entity;
using talus::objectmodel::PhysicalMaterial;
using talus::objectmodel::PlasticPoint;
using talus::objectmodel::PreparedHex;
using talus::objectmodel::PreparedMesh;
using talus::objectmodel::World;
using talus::physics::JoltLibrary;
using talus::runtime::Runtime;

namespace {

constexpr double LANDING_GRAVITY = 9.81;
constexpr double LANDING_LEAD = 1.0e-3;
constexpr double LANDING_AFTER = 60.0e-3;
constexpr double LANDING_FLAT_HEIGHT = 0.5;
constexpr double RELAX_TIME = 10.0e-3;
constexpr double RELAX_DAMPING = 1800.0;
constexpr float FINE_VOXEL = 0.0125f;
constexpr float SLAB_VOXEL = 0.05f;
constexpr float SLAB_CENTRE = -0.15f;
constexpr uint32_t SLAB_BODY = 1u;
constexpr double WINDOW_EPSILON = 0.035;
constexpr double FRAME_TIME = 1.0 / 60.0;
constexpr float SPAWN_TILT = 0.6f;
constexpr float SPAWN_SPIN = 10.0f;
constexpr std::array<float, 4> SPAWN_HEIGHTS{0.1f, 0.5f, 1.5f, 3.0f};
constexpr uint64_t SPAWN_SEED = 124u;
constexpr int64_t PROBE_FRAME_STEPS = 1024;
constexpr int64_t PROBE_SEARCH_STEPS = 1 << 15;
constexpr double REFERENCE_BAND = 1.0e-3;
constexpr double SOFT_PENALTY = 4.0;
constexpr float ELASTIC_YIELD = 1.0e12f;
constexpr double FLIGHT_SPIN_SHARE = 1.0e-3;
constexpr double FLIGHT_MOMENTUM = 1.0e-6;
constexpr double FLIGHT_ANGULAR = 1.0e-4;
constexpr uint32_t FLIGHT_FRAMES = 60u;
constexpr double FLIGHT_SPIN = 10.0;
constexpr double FLIGHT_BENT_SPIN = 20.0;
constexpr double FLIGHT_UNSTABLE_SHARE = 1.5;
constexpr double ELASTIC_FLAT_BOUND = 0.025;
constexpr double ELASTIC_TUMBLE_BOUND = 0.01;
constexpr uint32_t PLASTIC_FRAMES_AFTER = 40u;

struct Spawn {
  uint32_t index = 0u;
  bool hex = true;
  float height = 0.0f;
  glm::quat turn = glm::quat(glm::vec3(0.0f));
  glm::vec3 turning{0.0f};
};

Spawn spawnOf(uint32_t target) {
  std::mt19937_64 random(SPAWN_SEED);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  std::normal_distribution<float> normal(0.0f, 1.0f);

  const auto direction = [&]() {
    for (;;) {
      const glm::vec3 drawn(normal(random), normal(random), normal(random));
      const float length = glm::length(drawn);

      if (length > 1.0e-3f) {
        return drawn / length;
      }
    }
  };

  Spawn body;

  for (uint32_t index = 0u; index <= target; ++index) {
    body.index = index;
    body.hex = index % plasticbar::ELEMENTS.size() == 0u;
    body.height = SPAWN_HEIGHTS
      [(index / plasticbar::ELEMENTS.size()) % SPAWN_HEIGHTS.size()];

    const float tilt = SPAWN_TILT * unit(random);
    const float spin = SPAWN_SPIN * unit(random);
    const float heading = 2.0f * std::numbers::pi_v<float> * unit(random);
    const float lean = 2.0f * std::numbers::pi_v<float> * unit(random);
    const glm::vec3 about = direction();

    body.turn =
      glm::angleAxis(tilt, glm::vec3(std::cos(lean), 0.0f, std::sin(lean)))
      * glm::angleAxis(heading, glm::vec3(0.0f, 1.0f, 0.0f));
    body.turning = spin * about;
  }

  return body;
}

struct Landing {
  std::string name;
  uint32_t bar = 0u;
  double height = 0.0;
  glm::dquat turn = glm::dquat(1.0, 0.0, 0.0, 0.0);
  glm::dvec3 spin{0.0};
};

Landing landingFor(const std::string& name) {
  Landing landing;

  landing.name = name;

  if (name == "F") {
    landing.height = LANDING_FLAT_HEIGHT;

    return landing;
  }

  const std::array<uint32_t, 4> bars{0u, 8u, 22u, 30u};
  const uint32_t pick = name.size() == 2u && name[0] == 'P'
                          ? static_cast<uint32_t>(name[1] - '1')
                          : 4u;

  REQUIRE(pick < bars.size());

  const Spawn spawn = spawnOf(bars[pick]);

  REQUIRE(spawn.hex);

  landing.bar = spawn.index;
  landing.height = static_cast<double>(spawn.height);
  landing.turn = glm::dquat(spawn.turn);
  landing.spin = glm::dvec3(spawn.turning);

  return landing;
}

std::array<glm::dvec3, 8> barCorners() {
  std::array<glm::dvec3, 8> corners{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    corners[corner] = glm::dvec3(
      (corner & 1u) != 0u ? plasticbar::HALF_LENGTH : -plasticbar::HALF_LENGTH,
      (corner & 2u) != 0u ? plasticbar::HALF_WIDTH : -plasticbar::HALF_WIDTH,
      (corner & 4u) != 0u ? plasticbar::HALF_WIDTH : -plasticbar::HALF_WIDTH
    );
  }

  return corners;
}

glm::dquat turnedAt(const Landing& landing, double time) {
  const double rate = glm::length(landing.spin);

  if (rate <= 0.0) {
    return landing.turn;
  }

  return glm::angleAxis(rate * time, landing.spin / rate) * landing.turn;
}

double lowestAt(const Landing& landing, double spawnCentre, double time) {
  const glm::dquat turn = turnedAt(landing, time);
  double lowest = std::numeric_limits<double>::infinity();

  for (const glm::dvec3& corner : barCorners()) {
    lowest = std::min(lowest, (turn * corner).y);
  }

  return spawnCentre - 0.5 * LANDING_GRAVITY * time * time + lowest;
}

double contactTime(const Landing& landing, double spawnCentre) {
  constexpr double SCAN = 1.0e-5;

  double before = 0.0;

  while (lowestAt(landing, spawnCentre, before + SCAN) > 0.0) {
    before += SCAN;
  }

  double after = before + SCAN;

  for (uint32_t pass = 0u; pass != 60u; ++pass) {
    const double middle = 0.5 * (before + after);

    if (lowestAt(landing, spawnCentre, middle) > 0.0) {
      before = middle;
    } else {
      after = middle;
    }
  }

  return after;
}

std::shared_ptr<const PreparedMesh> barMesh(
  float voxel,
  std::span<const PhysicalMaterial> materials
) {
  const glm::vec3 half(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );

  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * voxel));

  primitive.shapeConfig.y = plasticbar::LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(glm::round(2.0f * half / voxel)) + glm::uvec3(1u)
  );

  return talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
}

talus::field::SampledSdf landingSlab() {
  const glm::vec3 half(1.0f, 0.15f, 1.0f);
  const glm::vec3 reach = half + glm::vec3(2.0f * SLAB_VOXEL);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(glm::vec3(0.0f), half)},
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / SLAB_VOXEL)) + glm::uvec3(1u)
  );
}

WindowPartner slabPartner(const talus::field::SampledSdf& slab) {
  WindowPartner partner;

  partner.probe.field = &slab;
  partner.body = SLAB_BODY;
  partner.centerOfMass = glm::dvec3(0.0, SLAB_CENTRE, 0.0);

  return partner;
}

ImpactWindow landingWindow(const PreparedMesh& mesh, const Landing& landing) {
  double lowestCorner = std::numeric_limits<double>::infinity();

  for (const glm::dvec3& corner : barCorners()) {
    lowestCorner = std::min(lowestCorner, (landing.turn * corner).y);
  }

  const double spawnCentre = landing.height - lowestCorner;
  const double touch = contactTime(landing, spawnCentre);
  const double start = touch - LANDING_LEAD;
  const glm::dquat turn = turnedAt(landing, start);
  const glm::dvec3 origin(
    0.0,
    spawnCentre - 0.5 * LANDING_GRAVITY * start * start,
    0.0
  );
  const glm::dvec3 falling(0.0, -LANDING_GRAVITY * start, 0.0);

  const DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  ImpactWindow window = talus::deformable::openWindow(mesh, state, origin);

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    window.position[node] = turn * glm::dvec3(mesh.restPositions[node]);
    window.velocity[node] =
      falling + glm::cross(landing.spin, window.position[node]);
  }

  return window;
}

glm::dvec3 sectionOf(
  const PreparedMesh& mesh,
  std::span<const glm::dvec3> position,
  float along,
  float edge
) {
  glm::dvec3 sum(0.0);
  size_t counted = 0u;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (std::abs(mesh.restPositions[node].x - along) < edge) {
      sum += position[node];
      ++counted;
    }
  }

  return counted != 0u ? sum / static_cast<double>(counted) : sum;
}

double bendOf(const PreparedMesh& mesh, std::span<const glm::dvec3> position) {
  float first = std::numeric_limits<float>::infinity();
  float last = -std::numeric_limits<float>::infinity();
  float middle = std::numeric_limits<float>::infinity();

  for (const glm::vec3& rest : mesh.restPositions) {
    first = std::min(first, rest.x);
    last = std::max(last, rest.x);

    if (std::abs(rest.x) < std::abs(middle)) {
      middle = rest.x;
    }
  }

  const float edge = 0.25f * mesh.voxelSize.x;
  const glm::dvec3 start = sectionOf(mesh, position, first, edge);
  const glm::dvec3 end = sectionOf(mesh, position, last, edge);
  const glm::dvec3 centre = sectionOf(mesh, position, middle, edge);
  const glm::dvec3 chord = end - start;
  const double span = glm::length(chord);

  if (span <= 0.0) {
    return glm::length(centre - start);
  }

  return glm::length(glm::cross(centre - start, chord)) / span;
}

glm::dmat3 rotationOf(const glm::dmat3& covariance) {
  glm::dmat3 rotation = covariance;

  for (uint32_t pass = 0u; pass != 100u; ++pass) {
    const glm::dmat3 next =
      0.5 * (rotation + glm::transpose(glm::inverse(rotation)));
    const glm::dmat3 change = next - rotation;

    rotation = next;

    double largest = 0.0;

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        largest = std::max(largest, std::abs(change[column][row]));
      }
    }

    if (largest < 1.0e-14) {
      break;
    }
  }

  return rotation;
}

double dentOf(const PreparedMesh& mesh, std::span<const glm::dvec3> position) {
  glm::dvec3 restCentre(0.0);
  glm::dvec3 endCentre(0.0);
  double mass = 0.0;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);

    restCentre += weight * glm::dvec3(mesh.restPositions[node]);
    endCentre += weight * position[node];
    mass += weight;
  }

  restCentre /= mass;
  endCentre /= mass;

  glm::dmat3 covariance(0.0);

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    covariance += static_cast<double>(mesh.nodeMass[node])
                  * glm::outerProduct(
                    glm::dvec3(mesh.restPositions[node]) - restCentre,
                    position[node] - endCentre
                  );
  }

  const glm::dmat3 rotation = rotationOf(covariance);

  std::vector<glm::dvec3> displacement(mesh.restPositions.size());

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    displacement[node] = rotation * (position[node] - endCentre) + restCentre
                         - glm::dvec3(mesh.restPositions[node]);
  }

  double dent = 0.0;

  for (const glm::uvec3& triangle : mesh.surfaceTriangles) {
    const glm::dvec3 a(mesh.restPositions[triangle.x]);
    const glm::dvec3 b(mesh.restPositions[triangle.y]);
    const glm::dvec3 c(mesh.restPositions[triangle.z]);
    const glm::dvec3 normal = glm::cross(b - a, c - a);
    const double twiceArea = glm::length(normal);

    if (twiceArea <= 0.0) {
      continue;
    }

    const glm::dvec3 moved =
      (displacement[triangle.x] + displacement[triangle.y]
       + displacement[triangle.z])
      / 3.0;
    const double inward = -glm::dot(moved, normal / twiceArea);

    dent += 0.5 * twiceArea * std::max(0.0, inward);
  }

  return dent;
}

void hexForces(
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> materials,
  std::span<const glm::dvec3> position,
  std::span<const PlasticPoint> plastic,
  bool returnMapping,
  std::vector<glm::dvec3>& force,
  std::vector<PlasticPoint>& advanced
) {
  std::fill(force.begin(), force.end(), glm::dvec3(0.0));

  std::array<glm::vec3, 8> deformed{};
  std::array<glm::vec3, 8> rest{};
  std::array<PlasticPoint, 8> committed{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const PreparedHex& hex = mesh.hexes[element];
    const glm::dvec3 corner = position[hex.nodes[0]];
    const PhysicalMaterial& lane = materials[hex.material];

    for (size_t at = 0u; at != 8u; ++at) {
      deformed[at] = glm::vec3(position[hex.nodes[at]] - corner);
      rest[at] =
        mesh.restPositions[hex.nodes[at]] - mesh.restPositions[hex.nodes[0]];
      committed[at] = plastic[8u * element + at];
    }

    const talus::deformable::HexPointTrial trial =
      talus::deformable::hexPointTrial(
        deformed,
        rest,
        hex,
        talus::deformable::ElasticParams{
          lane.youngsModulus,
          lane.poissonsRatio,
          lane.density
        },
        talus::deformable::PlasticParams{
          lane.yieldStress,
          lane.hardeningModulus
        },
        committed,
        0.0f,
        talus::deformable::ElementRotation::Polar,
        returnMapping,
        false
      );

    for (size_t at = 0u; at != 8u; ++at) {
      force[hex.nodes[at]] += glm::dvec3(trial.force[at]);
      advanced[8u * element + at] = trial.points[at];
    }
  }
}

struct Scored {
  double work = 0.0;
  double dent = 0.0;
  double bend = 0.0;
};

Scored scoreOf(
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> materials,
  std::span<const glm::dvec3> position,
  std::span<const PlasticPoint> plastic,
  double relaxStep
) {
  Scored scored;

  scored.work = talus::deformable::windowPlasticWork(mesh, materials, plastic);

  std::vector<glm::dvec3> relaxed(position.begin(), position.end());
  std::vector<glm::dvec3> moving(relaxed.size(), glm::dvec3(0.0));
  std::vector<glm::dvec3> force(relaxed.size(), glm::dvec3(0.0));
  std::vector<PlasticPoint> frozen(plastic.begin(), plastic.end());

  const uint32_t steps =
    static_cast<uint32_t>(std::ceil(RELAX_TIME / relaxStep));
  const double keep = 1.0 - 0.5 * RELAX_DAMPING * relaxStep;
  const double scale = 1.0 / (1.0 + 0.5 * RELAX_DAMPING * relaxStep);

  for (uint32_t step = 0u; step != steps; ++step) {
    hexForces(mesh, materials, relaxed, plastic, false, force, frozen);

    for (size_t node = 0u; node != relaxed.size(); ++node) {
      const double mass = static_cast<double>(mesh.nodeMass[node]);

      moving[node] =
        scale * (keep * moving[node] + relaxStep * force[node] / mass);
      relaxed[node] += relaxStep * moving[node];
    }
  }

  scored.dent = dentOf(mesh, relaxed);
  scored.bend = bendOf(mesh, relaxed);

  return scored;
}

struct ReferenceRun {
  Scored scored;
  double stepCreation = 0.0;
  double input = 0.0;
  int64_t contact = -1;
  int64_t snapshot = 0;
  bool diverged = false;
};

WindowStep stepFor(
  WindowGroup& group,
  std::span<const PhysicalMaterial> materials,
  double frameTime,
  const talus::deformable::Settings& settings
) {
  const std::vector<WindowStep> steps = talus::deformable::stepWindows(
    group,
    materials,
    glm::dvec3(0.0, -LANDING_GRAVITY, 0.0),
    frameTime,
    settings
  );

  REQUIRE(steps.size() == 1u);

  return steps.front();
}

ReferenceRun referenceLanding(
  const std::string& name,
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> materials,
  const talus::field::SampledSdf& slab,
  const talus::deformable::Settings& settings
) {
  const Landing landing = landingFor(name);

  ImpactWindow window = landingWindow(mesh, landing);

  WindowGroup group;

  group.partners.push_back(slabPartner(slab));
  group.members.push_back(WindowMember{&mesh, &window, {0u}, 0.0f, true});

  const talus::deformable::WindowLaw law =
    talus::deformable::windowLaw(mesh, materials, FRAME_TIME, settings);
  const double step = law.stepLimit;

  WindowStep last;

  while (window.firstContact < 0 && window.substeps < PROBE_SEARCH_STEPS) {
    last = stepFor(
      group,
      materials,
      step * static_cast<double>(PROBE_FRAME_STEPS),
      settings
    );
  }

  REQUIRE(window.firstContact >= 0);

  const double contact = static_cast<double>(window.firstContact) * step;

  int64_t target = window.firstContact;

  while (static_cast<double>(target) * step < contact + LANDING_AFTER) {
    ++target;
  }

  REQUIRE(window.substeps <= target);

  int64_t remaining = target - window.substeps;

  for (int64_t chunk = PROBE_SEARCH_STEPS; chunk != 0; chunk /= 2) {
    while (remaining >= chunk) {
      last =
        stepFor(group, materials, step * static_cast<double>(chunk), settings);
      remaining -= chunk;
    }
  }

  REQUIRE(window.substeps == target);

  std::vector<glm::dvec3> world(window.position.size());

  for (size_t node = 0u; node != world.size(); ++node) {
    world[node] = window.origin + window.position[node];
  }

  std::vector<glm::dvec3> pushed(world.size(), glm::dvec3(0.0));
  std::vector<PlasticPoint> reached(window.plastic.size());

  hexForces(mesh, materials, world, window.plastic, true, pushed, reached);

  ReferenceRun run;

  run.scored = scoreOf(mesh, materials, world, reached, law.referenceStep);
  run.stepCreation = last.stepCreation;
  run.input = last.input;
  run.contact = window.firstContact;
  run.snapshot = target;
  run.diverged = window.diverged;

  return run;
}

struct LandingRun {
  WindowStep last;
  double afterContact = 0.0;
  uint32_t framesAfter = 0u;
  bool diverged = false;
};

LandingRun productionLanding(
  const std::string& name,
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> materials,
  const talus::field::SampledSdf& slab,
  const talus::deformable::Settings& settings,
  double after,
  uint32_t framesAfter
) {
  const Landing landing = landingFor(name);

  ImpactWindow window = landingWindow(mesh, landing);

  WindowGroup group;

  group.partners.push_back(slabPartner(slab));
  group.members.push_back(WindowMember{&mesh, &window, {0u}, 0.0f, true});

  LandingRun run;

  for (uint32_t frame = 0u; frame != 1000u && !window.diverged; ++frame) {
    run.last = stepFor(group, materials, FRAME_TIME, settings);

    if (window.firstContact < 0) {
      continue;
    }

    run.afterContact = window.time - window.lead;

    if (run.afterContact >= after && run.framesAfter >= framesAfter) {
      break;
    }

    ++run.framesAfter;
  }

  run.diverged = window.diverged;

  return run;
}

double apart(double value, double reference) {
  return std::abs(value / reference - 1.0);
}

struct FlightRun {
  double stepCreation = 0.0;
  bool stepCreationContact = false;
  double input = 0.0;
  double spinStart = 0.0;
  double spinCreation = 0.0;
  double linear = 0.0;
  double angular = 0.0;
  double angularDrift = 0.0;
  bool diverged = false;
  bool restored = false;
};

struct Motion {
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
};

Motion motionOf(const PreparedMesh& mesh, const ImpactWindow& window) {
  double mass = 0.0;
  glm::dvec3 centre(0.0);
  Motion motion;

  for (size_t node = 0u; node != window.position.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);

    mass += weight;
    centre += weight * window.position[node];
    motion.linear += weight * window.velocity[node];
  }

  centre /= mass;

  for (size_t node = 0u; node != window.position.size(); ++node) {
    motion.angular +=
      static_cast<double>(mesh.nodeMass[node])
      * glm::cross(window.position[node] - centre, window.velocity[node]);
  }

  return motion;
}

double relativeChange(const glm::dvec3& before, const glm::dvec3& after) {
  const double size = glm::length(before);
  const double change = glm::length(after - before);

  return size > 0.0 ? change / size : change;
}

glm::mat3 committedStrain() {
  return glm::mat3(
    glm::vec3(0.1f, 0.0f, 0.0f),
    glm::vec3(0.0f, -0.05f, 0.0f),
    glm::vec3(0.0f, 0.0f, -0.05f)
  );
}

glm::mat3 shearStrain() {
  return glm::mat3(
    glm::vec3(3.0e-4f, 4.0e-4f, -1.5e-4f),
    glm::vec3(4.0e-4f, -1.2e-4f, 2.5e-4f),
    glm::vec3(-1.5e-4f, 2.5e-4f, -1.0e-4f)
  );
}

float equivalentOf(const glm::mat3& strain) {
  float squared = 0.0f;

  for (int column = 0; column != 3; ++column) {
    squared += glm::dot(strain[column], strain[column]);
  }

  return std::sqrt(2.0f / 3.0f * squared);
}

FlightRun flyWindow(
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> materials,
  const talus::deformable::Settings& settings,
  const glm::dvec3& gravity,
  double spin,
  bool bent
) {
  const glm::dvec3 drift = bent ? glm::dvec3(0.0) : glm::dvec3(1.0, 0.5, 0.0);
  const glm::dvec3 axis = spin * glm::normalize(glm::dvec3(1.0));
  const glm::dvec3 lifted(0.0, 1.0, 0.0);

  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  const glm::mat3 stretch = glm::mat3(1.0f) + committedStrain() + shearStrain();

  if (bent) {
    for (glm::vec3& at : state.position) {
      at = stretch * at;
    }

    state.plastic.assign(
      8u * mesh.hexes.size(),
      PlasticPoint{committedStrain(), equivalentOf(committedStrain())}
    );
  }

  ImpactWindow window = talus::deformable::openWindow(mesh, state, lifted);

  for (size_t node = 0u; node != window.position.size(); ++node) {
    window.position[node] = glm::dvec3(state.position[node]);
    window.velocity[node] = drift + glm::cross(axis, window.position[node]);
  }

  const Motion start = motionOf(mesh, window);

  double mass = 0.0;

  for (const float weight : mesh.nodeMass) {
    mass += static_cast<double>(weight);
  }

  const auto centreVelocity = [&]() {
    glm::dvec3 moving(0.0);

    for (size_t node = 0u; node != window.velocity.size(); ++node) {
      moving +=
        static_cast<double>(mesh.nodeMass[node]) * window.velocity[node];
    }

    return moving / mass;
  };

  const glm::dvec3 opening = centreVelocity();

  WindowGroup group;

  group.members.push_back(WindowMember{&mesh, &window, {}, 0.0f, true});

  FlightRun run;

  for (uint32_t frame = 0u; frame != FLIGHT_FRAMES; ++frame) {
    const std::vector<WindowStep> steps = talus::deformable::stepWindows(
      group,
      materials,
      gravity,
      FRAME_TIME,
      settings
    );

    REQUIRE(steps.size() == 1u);

    const WindowStep& step = steps.front();

    run.stepCreation = step.stepCreation;
    run.stepCreationContact = step.stepCreationContact;
    run.input = step.input;
    run.angularDrift = step.angularDrift;
    run.restored = run.restored || step.restored;

    if (window.diverged) {
      break;
    }

    if (frame == 0u) {
      run.spinStart =
        window.openingTotal - window.openingGravity
        - 0.5 * mass
            * (glm::dot(opening, opening) - 0.25 * window.step * window.step * glm::dot(gravity, gravity));
    }

    const glm::dvec3 moving = centreVelocity();
    const double internal =
      step.ledger.total - step.ledger.gravity
      - 0.5 * mass * glm::dot(moving, moving - window.step * gravity);

    run.spinCreation = std::max(run.spinCreation, internal - run.spinStart);
  }

  const Motion end = motionOf(mesh, window);

  run.linear = relativeChange(start.linear, end.linear);
  run.angular = relativeChange(start.angular, end.angular);
  run.diverged = window.diverged;

  return run;
}

constexpr uint32_t THREAD_FRAMES = 30u;
constexpr uint32_t THREAD_SETTLE = 10u;
constexpr float THREAD_SKIN = 0.2f * plasticbar::COARSE_VOXEL;
constexpr float THREAD_BOX_HALF = 0.05f;
constexpr float THREAD_BOX_VOXEL = 0.025f;
constexpr float THREAD_APART = 0.35f;
constexpr uint64_t HASH_BASIS = 14695981039346656037ull;
constexpr uint64_t HASH_PRIME = 1099511628211ull;

talus::field::Primitive spanningBox(
  const glm::vec3& low,
  const glm::vec3& high,
  uint32_t lane
) {
  talus::field::Primitive primitive =
    fixture::box(0.5f * (low + high), 0.5f * (high - low));

  primitive.shapeConfig.y = lane;

  return primitive;
}

Entity threadSlab(World& world) {
  talus::objectmodel::SdfGrid grid{};

  grid.resolution = glm::uvec3(29u, 5u, 29u);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, -0.05f, 0.0f);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanningBox(
      glm::vec3(-0.6f, -0.05f, -0.6f),
      glm::vec3(0.6f, 0.05f, 0.6f),
      0u
    )
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      glm::vec3(-0.7f, -0.1f, -0.7f),
      glm::vec3(0.7f, 0.1f, 0.7f),
      grid.resolution
    )
  );

  world.addStaticBody(entity);

  return entity;
}

Entity threadBar(World& world, const talus::objectmodel::Pose& pose) {
  const float voxel = plasticbar::COARSE_VOXEL;
  const glm::vec3 half(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );
  const glm::vec3 low = -half - glm::vec3(voxel);
  const glm::vec3 high = half + glm::vec3(voxel);

  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round((high - low) / voxel)) + glm::uvec3(1u);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanningBox(
      -half - glm::vec3(THREAD_SKIN),
      half + glm::vec3(THREAD_SKIN),
      plasticbar::LANE
    )
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(world.primitives(entity), low, high, grid.resolution)
  );

  world.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    entity,
    talus::objectmodel::DeformableBody{
      talus::objectmodel::VolumeElementType::Hex
    }
  );

  return entity;
}

Entity threadBox(World& world, const glm::vec3& at) {
  const glm::vec3 reach(THREAD_BOX_HALF + 2.0f * THREAD_BOX_VOXEL);

  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round(2.0f * reach / THREAD_BOX_VOXEL)) + glm::uvec3(1u);

  talus::objectmodel::Pose pose{};
  pose.position = at;

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanningBox(glm::vec3(-THREAD_BOX_HALF), glm::vec3(THREAD_BOX_HALF), 0u)
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(world.primitives(entity), -reach, reach, grid.resolution)
  );

  world.addRigidBody(entity);

  return entity;
}

void mixHash(uint64_t& hash, uint64_t bits) {
  for (uint32_t shift = 0u; shift != 64u; shift += 8u) {
    hash ^= (bits >> shift) & 0xffu;
    hash *= HASH_PRIME;
  }
}

void mixDouble(uint64_t& hash, double value) {
  mixHash(hash, std::bit_cast<uint64_t>(value));
}

void mixVector(uint64_t& hash, const glm::dvec3& value) {
  mixDouble(hash, value.x);
  mixDouble(hash, value.y);
  mixDouble(hash, value.z);
}

void mixFloat(uint64_t& hash, float value) {
  mixHash(hash, std::bit_cast<uint32_t>(value));
}

struct ThreadRun {
  uint64_t hash = HASH_BASIS;
  uint32_t windows = 0u;
  uint32_t groups = 0u;
  uint32_t boxTouched = 0u;
  uint32_t worstBookings = 0u;
  uint32_t bookedFrames = 0u;
  uint32_t handbacks = 0u;
  uint32_t owned = 0u;
  uint32_t lines = 0u;
  double groupMass = 0.0;
  double groupStart = 0.0;
  double groupSettled = 0.0;
  double groupEnd = 0.0;
  double groupRise = 0.0;
};

ThreadRun groupedWindows(uint32_t threads) {
  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  static_cast<void>(threadSlab(source));

  const Entity box = threadBox(source, glm::vec3(0.0f, THREAD_BOX_HALF, 0.0f));

  std::vector<Entity> bars;

  for (uint32_t index = 0u; index != 3u; ++index) {
    talus::objectmodel::Pose pose{};

    pose.position =
      glm::vec3(0.0f, 2.0f * THREAD_BOX_HALF + plasticbar::HALF_WIDTH, 0.0f);
    pose.rotation = glm::angleAxis(
      static_cast<float>(index) * std::numbers::pi_v<float> / 3.0f,
      glm::vec3(0.0f, 1.0f, 0.0f)
    );

    bars.push_back(threadBar(source, pose));
  }

  for (const float side : {-THREAD_APART, THREAD_APART}) {
    talus::objectmodel::Pose pose{};

    pose.position = glm::vec3(side, plasticbar::HALF_WIDTH, 0.0f);
    pose.rotation = glm::angleAxis(
      0.5f * std::numbers::pi_v<float>,
      glm::vec3(0.0f, 1.0f, 0.0f)
    );

    bars.push_back(threadBar(source, pose));
  }

  Runtime runtime(source, library);

  talus::deformable::Settings settings = runtime.deformableSettings();

  settings.demoteAfterYield = false;
  settings.stepThreads = threads;

  runtime.setDeformableSettings(settings);

  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  runtime.optimizeBroadPhase();

  const auto* boxBody =
    runtime.world().registry().try_get<talus::objectmodel::BodyId>(box);

  REQUIRE(boxBody != nullptr);

  const JPH::BodyID boxId(boxBody->id);

  for (const Entity bar : bars) {
    REQUIRE(runtime.forcePromotion(bar));
  }

  const PreparedMesh* barMesh =
    runtime.world().deformablePrepared(bars.front());

  REQUIRE(barMesh != nullptr);

  const windowmonitor::MonitoredBody monitored =
    windowmonitor::monitoredBody(*barMesh);
  const double restingMass =
    static_cast<double>(bars.size()) * monitored.mass
    + static_cast<double>(
      runtime.physics().bodies().GetShape(boxId)->GetMassProperties().mMass
    );

  ThreadRun run;
  std::vector<std::pair<Entity, double>> booked;

  for (uint32_t frame = 0u; frame != THREAD_FRAMES; ++frame) {
    runtime.step();

    const talus::runtime::DeformableActivity& activity =
      runtime.deformableActivity();

    uint32_t bookings = 0u;

    for (const talus::runtime::WindowRecord& record : activity.windowFrames) {
      const auto held =
        std::find_if(booked.begin(), booked.end(), [&](const auto& entry) {
          return entry.first == record.entity;
        });
      const double before = held != booked.end() ? held->second : 0.0;

      bookings += record.step.ledger.partners != before ? 1u : 0u;

      if (held != booked.end()) {
        held->second = record.step.ledger.partners;
      } else {
        booked.emplace_back(record.entity, record.step.ledger.partners);
      }
    }

    run.worstBookings = std::max(run.worstBookings, bookings);
    run.bookedFrames += bookings != 0u ? 1u : 0u;

    const auto print = [&run](const std::vector<std::string>& printed) {
      for (const std::string& line : printed) {
        std::cout << line << '\n';
        ++run.lines;
      }
    };

    print(
      windowmonitor::frameViolations(
        activity.windowFrames,
        frame,
        WINDOW_EPSILON,
        [&monitored](Entity) { return monitored; },
        restingMass
      )
    );

    for (const talus::runtime::WindowRecord& record : activity.windowFrames) {
      for (const talus::runtime::PartnerHandback& handback : record.handbacks) {
        ++run.handbacks;
        run.owned += handback.owned ? 1u : 0u;
        print(
          windowmonitor::handbackViolations(handback, frame, WINDOW_EPSILON)
        );
      }
    }

    for (const talus::runtime::YieldTransition& transition :
         activity.transitions) {
      print(windowmonitor::transitionViolations(transition, frame, monitored));
    }

    if (frame == 0u) {
      run.windows = activity.windows;
      run.groups = activity.windowGroups;
    }

    for (const talus::runtime::WindowRecord& record : activity.windowFrames) {
      run.boxTouched += record.step.dynamicPartners;
    }

    uint32_t shared = 0u;
    size_t members = 0u;

    for (const talus::runtime::WindowRecord& record : activity.windowFrames) {
      const size_t count = static_cast<size_t>(std::count_if(
        activity.windowFrames.begin(),
        activity.windowFrames.end(),
        [&record](const talus::runtime::WindowRecord& other) {
          return other.group == record.group;
        }
      ));

      if (count > members) {
        members = count;
        shared = record.group;
      }
    }

    double summed = 0.0;

    for (const talus::runtime::WindowRecord& record : activity.windowFrames) {
      if (record.group == shared) {
        summed += record.step.creation;
        run.groupRise = std::max(run.groupRise, record.step.groupRise);
      }
    }

    run.groupMass =
      static_cast<double>(members) * monitored.mass
      + static_cast<double>(
        runtime.physics().bodies().GetShape(boxId)->GetMassProperties().mMass
      );
    run.groupStart = frame == 0u ? summed : run.groupStart;
    run.groupSettled = frame == THREAD_SETTLE ? summed : run.groupSettled;
    run.groupEnd = summed;
  }

  for (const Entity bar : bars) {
    const ImpactWindow* window = runtime.impactWindow(bar);

    REQUIRE(window != nullptr);

    for (const glm::dvec3& at : window->position) {
      mixVector(run.hash, at);
    }

    for (const glm::dvec3& moving : window->velocity) {
      mixVector(run.hash, moving);
    }

    mixDouble(run.hash, window->ledger.kinetic);
    mixDouble(run.hash, window->ledger.gravity);
    mixDouble(run.hash, window->ledger.elastic);
    mixDouble(run.hash, window->ledger.plastic);
    mixDouble(run.hash, window->ledger.penalty);
    mixDouble(run.hash, window->ledger.friction);
    mixDouble(run.hash, window->ledger.partners);
    mixDouble(run.hash, window->ledger.total);
  }

  const JPH::Vec3 linear = runtime.physics().bodies().GetLinearVelocity(boxId);
  const JPH::Vec3 angular =
    runtime.physics().bodies().GetAngularVelocity(boxId);

  mixFloat(run.hash, linear.GetX());
  mixFloat(run.hash, linear.GetY());
  mixFloat(run.hash, linear.GetZ());
  mixFloat(run.hash, angular.GetX());
  mixFloat(run.hash, angular.GetY());
  mixFloat(run.hash, angular.GetZ());

  return run;
}
}

TEST_CASE(
  "an impact window in free flight creates no energy and keeps its momentum",
  "[.][impactwindow]"
) {
  std::cout.precision(6);

  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(plasticbar::YIELD);
  const std::shared_ptr<const PreparedMesh> mesh =
    barMesh(plasticbar::COARSE_VOXEL, materials);

  REQUIRE(mesh != nullptr);
  REQUIRE(!mesh->hexes.empty());

  const talus::deformable::Settings settings;

  REQUIRE(settings.windowStepShare == 0.45);
  REQUIRE(settings.windowPenaltyScale == 16.0);
  REQUIRE(settings.windowRotationDerivative);

  const FlightRun still =
    flyWindow(*mesh, materials, settings, glm::dvec3(0.0), FLIGHT_SPIN, false);
  const FlightRun falling = flyWindow(
    *mesh,
    materials,
    settings,
    glm::dvec3(0.0, -LANDING_GRAVITY, 0.0),
    FLIGHT_SPIN,
    false
  );

  talus::deformable::Settings plain = settings;

  plain.windowRotationDerivative = false;

  const FlightRun bentKept = flyWindow(
    *mesh,
    materials,
    settings,
    glm::dvec3(0.0),
    FLIGHT_BENT_SPIN,
    true
  );
  const FlightRun bentPlain =
    flyWindow(*mesh, materials, plain, glm::dvec3(0.0), FLIGHT_BENT_SPIN, true);

  talus::deformable::Settings unstable = settings;

  unstable.windowStepShare = FLIGHT_UNSTABLE_SHARE;

  const FlightRun blown =
    flyWindow(*mesh, materials, unstable, glm::dvec3(0.0), FLIGHT_SPIN, false);

  const auto report = [](const char* label, const FlightRun& run) {
    std::cout << "impactwindow: flight " << label << " stepCreation "
              << run.stepCreation << " contact "
              << (run.stepCreationContact ? 1 : 0) << " input " << run.input
              << " share " << run.stepCreation / run.input << " spinStart "
              << run.spinStart << " spinCreation " << run.spinCreation
              << " spinShare " << run.spinCreation / run.spinStart << " linear "
              << run.linear << " angular " << run.angular << " angularDrift "
              << run.angularDrift << " diverged " << (run.diverged ? 1 : 0)
              << '\n';
  };

  report("gravityOff", still);
  report("gravityOn", falling);
  report("bentWithTerm", bentKept);
  report("bentWithoutTerm", bentPlain);
  report("share1.5", blown);

  CHECK(!still.diverged);
  CHECK(!falling.diverged);
  CHECK(still.spinStart > 0.0);
  CHECK(falling.spinStart > 0.0);
  CHECK(still.spinCreation <= FLIGHT_SPIN_SHARE * still.spinStart);
  CHECK(falling.spinCreation <= FLIGHT_SPIN_SHARE * falling.spinStart);
  CHECK(!still.stepCreationContact);
  CHECK(!falling.stepCreationContact);
  CHECK(still.linear <= FLIGHT_MOMENTUM);
  CHECK(still.angular <= FLIGHT_MOMENTUM);
  CHECK(!bentKept.diverged);
  CHECK(bentKept.angularDrift <= FLIGHT_ANGULAR);
  CHECK(bentPlain.angularDrift > FLIGHT_ANGULAR);
  CHECK(
    (blown.diverged || blown.spinCreation > FLIGHT_SPIN_SHARE * blown.spinStart)
  );
  CHECK(!still.restored);
  CHECK(!falling.restored);
  CHECK(blown.restored == blown.diverged);
}

TEST_CASE(
  "an elastic landing in an impact window stays within its energy bound",
  "[.][impactwindow]"
) {
  std::cout.precision(6);

  const talus::field::SampledSdf slab = landingSlab();
  const talus::deformable::Settings settings;

  const std::vector<PhysicalMaterial> elastic =
    plasticbar::barMaterials(ELASTIC_YIELD);
  const std::shared_ptr<const PreparedMesh> elasticMesh =
    barMesh(FINE_VOXEL, elastic);

  REQUIRE(elasticMesh != nullptr);

  const LandingRun flat = productionLanding(
    "F",
    *elasticMesh,
    elastic,
    slab,
    settings,
    LANDING_AFTER,
    0u
  );
  const LandingRun tumble = productionLanding(
    "P3",
    *elasticMesh,
    elastic,
    slab,
    settings,
    LANDING_AFTER,
    0u
  );

  const std::vector<PhysicalMaterial> plastic =
    plasticbar::barMaterials(plasticbar::YIELD);
  const std::shared_ptr<const PreparedMesh> plasticMesh =
    barMesh(FINE_VOXEL, plastic);

  REQUIRE(plasticMesh != nullptr);

  const LandingRun yielded = productionLanding(
    "P3",
    *plasticMesh,
    plastic,
    slab,
    settings,
    0.0,
    PLASTIC_FRAMES_AFTER
  );

  const auto report = [](const char* label, const LandingRun& run) {
    std::cout << "impactwindow: landing " << label << " afterContact "
              << run.afterContact << " framesAfter " << run.framesAfter
              << " plasticWork " << run.last.ledger.plastic << " stepCreation "
              << run.last.stepCreation << " contact "
              << (run.last.stepCreationContact ? 1 : 0) << " input "
              << run.last.input << " share "
              << run.last.stepCreation / run.last.input << " creation "
              << run.last.creation << " diverged " << (run.diverged ? 1 : 0)
              << '\n';
  };

  report("F-elastic", flat);
  report("P3-elastic", tumble);
  report("P3-plastic", yielded);

  CHECK(!flat.diverged);
  CHECK(!tumble.diverged);
  CHECK(!yielded.diverged);
  CHECK(flat.afterContact >= LANDING_AFTER);
  CHECK(tumble.afterContact >= LANDING_AFTER);
  CHECK(yielded.framesAfter >= PLASTIC_FRAMES_AFTER);
  CHECK(flat.last.stepCreationContact);
  CHECK(flat.last.ledger.plastic == 0.0);
  CHECK(tumble.last.ledger.plastic == 0.0);
  CHECK(flat.last.stepCreation <= ELASTIC_FLAT_BOUND * flat.last.input);
  CHECK(tumble.last.stepCreation <= ELASTIC_TUMBLE_BOUND * tumble.last.input);
  CHECK(yielded.last.stepCreation <= WINDOW_EPSILON * yielded.last.input);
}

TEST_CASE(
  "impact window landings match the explicit reference",
  "[.][impactwindow]"
) {
  std::cout.precision(6);

  struct Expected {
    const char* name = "";
    double work = 0.0;
    double dent = 0.0;
    double bend = 0.0;
  };

  const std::array<Expected, 4> expected{
    Expected{"F", 4.06401, 8.63231e-6, 3.86334e-5},
    Expected{"P2", 0.76066, 9.97635e-6, 1.49643e-3},
    Expected{"P3", 23.8042, 8.95024e-5, 1.08963e-2},
    Expected{"P4", 21.2091, 9.23706e-5, 1.21555e-2}
  };

  const talus::field::SampledSdf slab = landingSlab();
  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(plasticbar::YIELD);
  const std::shared_ptr<const PreparedMesh> mesh =
    barMesh(FINE_VOXEL, materials);

  REQUIRE(mesh != nullptr);

  talus::deformable::Settings reference;

  reference.windowRotationDerivative = false;
  reference.windowContactTransitions = false;

  REQUIRE(reference.windowStepShare == 0.45);
  REQUIRE(reference.windowPenaltyScale == 16.0);
  REQUIRE(materials[plasticbar::LANE].friction == 0.5f);

  talus::deformable::Settings production;

  REQUIRE(production.windowRotationDerivative);
  REQUIRE(production.windowContactTransitions);

  talus::deformable::Settings soft = reference;

  soft.windowPenaltyScale = SOFT_PENALTY;

  const double step =
    talus::deformable::windowLaw(*mesh, materials, FRAME_TIME, reference)
      .stepLimit;

  std::cout << "impactwindow: reference step " << step << " nodes "
            << mesh->restPositions.size() << " hexes " << mesh->hexes.size()
            << '\n';

  for (const Expected& held : expected) {
    const ReferenceRun matched =
      referenceLanding(held.name, *mesh, materials, slab, reference);
    const ReferenceRun turned =
      referenceLanding(held.name, *mesh, materials, slab, production);
    const ReferenceRun softened =
      referenceLanding(held.name, *mesh, materials, slab, soft);

    std::cout << "impactwindow: reference case " << held.name << " contact "
              << matched.contact << " snapshot " << matched.snapshot
              << " plasticWork " << matched.scored.work << " of " << held.work
              << " apart " << apart(matched.scored.work, held.work) << " dent "
              << matched.scored.dent << " of " << held.dent << " apart "
              << apart(matched.scored.dent, held.dent) << " bend "
              << matched.scored.bend << " of " << held.bend << " apart "
              << apart(matched.scored.bend, held.bend) << " stepCreation "
              << matched.stepCreation << " input " << matched.input << '\n';
    std::cout << "impactwindow: production case " << held.name
              << " plasticWork " << turned.scored.work << " apart "
              << apart(turned.scored.work, held.work) << " dent "
              << turned.scored.dent << " apart "
              << apart(turned.scored.dent, held.dent) << " bend "
              << turned.scored.bend << " apart "
              << apart(turned.scored.bend, held.bend) << " stepCreation "
              << turned.stepCreation << " input " << turned.input << '\n';
    std::cout << "impactwindow: soft case " << held.name << " plasticWork "
              << softened.scored.work << " apart "
              << apart(softened.scored.work, held.work) << '\n';

    CHECK(!matched.diverged);
    CHECK(apart(matched.scored.work, held.work) <= REFERENCE_BAND);
    CHECK(apart(matched.scored.dent, held.dent) <= REFERENCE_BAND);
    CHECK(apart(matched.scored.bend, held.bend) <= REFERENCE_BAND);
    CHECK(matched.stepCreation <= WINDOW_EPSILON * matched.input);
    CHECK(apart(softened.scored.work, held.work) > REFERENCE_BAND);
  }
}

TEST_CASE(
  "impact window groups step alike on one and four threads",
  "[.][impactwindow]"
) {
  const ThreadRun serial = groupedWindows(1u);
  const ThreadRun threaded = groupedWindows(4u);

  std::cout
    << "impactwindow: threads serial hash " << serial.hash << " threaded hash "
    << threaded.hash << " windows " << serial.windows << " groups "
    << serial.groups << " boxTouched " << serial.boxTouched << " worstBookings "
    << serial.worstBookings << " bookedFrames " << serial.bookedFrames
    << " handbacks " << serial.handbacks << " owned " << serial.owned
    << " lines " << serial.lines << ' ' << threaded.lines << " groupMass "
    << serial.groupMass << " groupCreation " << serial.groupStart << ' '
    << serial.groupSettled << ' ' << serial.groupEnd << " groupRise "
    << serial.groupRise << " groupRate "
    << (serial.groupEnd - serial.groupStart)
         / (static_cast<double>(THREAD_FRAMES - 1u) * serial.groupMass)
    << " settledRate "
    << (serial.groupEnd - serial.groupSettled)
         / (static_cast<double>(THREAD_FRAMES - 1u - THREAD_SETTLE) * serial.groupMass)
    << " threshold " << 2.0 * windowmonitor::RESTING_DRIFT << '\n';

  CHECK(serial.handbacks > 0u);
  CHECK(serial.lines == 0u);
  CHECK(threaded.lines == 0u);

  CHECK(serial.worstBookings == 1u);
  CHECK(threaded.worstBookings == 1u);

  REQUIRE(serial.windows == 5u);
  REQUIRE(serial.groups == 3u);
  REQUIRE(serial.boxTouched > 0u);
  REQUIRE(threaded.windows == serial.windows);
  REQUIRE(threaded.groups == serial.groups);
  REQUIRE(serial.hash == threaded.hash);
}
