#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/field/constants.h>
#include <talus/deformable/contact/contact.h>
#include <talus/deformable/deformedfield/deformedfield.h>
#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/resolution/resolution.h>
#include <talus/dynamics/strainstep/contactrows.h>
#include <talus/dynamics/strainstep/element.h>
#include <talus/dynamics/strainstep/elementcache.h>
#include <talus/dynamics/strainstep/inversepool.h>
#include <talus/dynamics/strainstep/lowestmode.h>
#include <talus/dynamics/strainstep/plasticflow.h>
#include <talus/dynamics/strainstep/rigidstate.h>
#include <talus/dynamics/strainstep/strainstep.h>
#include <talus/dynamics/strainstep/tearing.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <numeric>
#include <optional>
#include <random>
#include <span>
#include <vector>

namespace {

using talus::dynamics::World;

struct SoftShape {
  talus::field::SampledSdf field;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
};

talus::PhysicalMaterial jelly(double youngs, double density = 1000.0) {
  talus::PhysicalMaterial material;

  material.density = static_cast<float>(density);
  material.friction = 0.7f;
  material.youngsModulus = static_cast<float>(youngs);
  material.poissonsRatio = 0.4f;

  return material;
}

talus::field::SampledSdf boxField(const glm::dvec3& half, double voxel, double outset = 0.0) {
  const glm::vec3 reach = glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half) + glm::vec3(static_cast<float>(outset)))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel))) + glm::uvec3(1u)
  );
}

talus::field::SampledSdf ballField(double radius, double voxel) {
  const float reach = static_cast<float>(radius + 2.0 * voxel);
  talus::field::Primitive primitive = fixture::sphere(glm::vec3(0.0f), static_cast<float>(radius));

  primitive.shapeConfig.y = 0u;

  return talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    glm::vec3(-reach),
    glm::vec3(reach),
    glm::uvec3(static_cast<uint32_t>(std::round(2.0f * reach / static_cast<float>(voxel))) + 1u)
  );
}

talus::deformable::SupportVolume holdAt(const glm::dvec3& centre, const glm::dvec3& half) {
  return talus::deformable::SupportVolume{
    glm::inverse(glm::translate(glm::mat4(1.0f), glm::vec3(centre))),
    glm::vec3(half)
  };
}

SoftShape softBox(
  const glm::dvec3& half,
  double voxel,
  const talus::PhysicalMaterial& material,
  std::span<const talus::deformable::SupportVolume> supports = {},
  double outset = 1.0e-4
) {
  SoftShape shape;

  shape.field = boxField(half, voxel, outset);
  shape.lanes.assign(2u, material);
  shape.mesh = talus::deformable::buildMesh(
    shape.field,
    shape.lanes,
    glm::mat4(1.0f),
    supports,
    talus::deformable::VolumeElementType::Hex
  );

  return shape;
}

void useStrainstep(World& world, const glm::dvec3& gravity) {
  world.parameters.sleep = false;
  world.parameters.gravity = gravity;
  world.parameters.softstepSubsteps = 4u;
  world.parameters.speculativeDistance = 0.02;
  world.parameters.contactHertz = 30.0;
  world.parameters.contactDampingRatio = 10.0;
  world.parameters.contactSpeed = 3.0;
  world.parameters.restitutionThreshold = 1.0;
  world.parameters.restitutionIterations = 2u;
}

uint32_t addSoft(
  World& world,
  const SoftShape& shape,
  const glm::dvec3& offset,
  const glm::dvec3& velocity = glm::dvec3(0.0)
) {
  return talus::dynamics::addLattice(
    world.bodies,
    shape.mesh,
    shape.lanes,
    offset,
    velocity,
    shape.lanes.front().friction,
    &shape.field
  );
}

void checkFreeFall(uint32_t requested, uint32_t expected) {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3));
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 50.0, 0.0));

  world.bodies.lattices[index].substeps = requested;

  const std::vector<glm::dvec3> start = world.bodies.lattices[index].position;
  constexpr uint32_t STEPS = 30u;

  for (uint32_t frame = 0u; frame != STEPS; ++frame) {
    REQUIRE(talus::dynamics::step(world).substeps == expected);
  }

  const double h = world.parameters.dt / static_cast<double>(expected);
  const double n = static_cast<double>(expected * STEPS);
  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];

  for (size_t node = 0u; node != body.position.size(); ++node) {
    CHECK(std::abs(body.velocity[node].y + 9.81 * h * n) <= 1.0e-9);
    CHECK(std::abs(body.position[node].y - start[node].y + 9.81 * h * h * n * (n + 1.0) / 2.0) <= 1.0e-9);
    CHECK(std::abs(body.position[node].x - start[node].x) <= 1.0e-12);
    CHECK(std::abs(body.position[node].z - start[node].z) <= 1.0e-12);
  }
}

struct TestCell {
  std::array<glm::dvec3, 8> rest{};
  std::array<uint32_t, 8> nodes{};
  std::array<double, 8> inverseMass{};
};

TestCell testCell(double half, double mass) {
  TestCell cell;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    cell.rest[corner] = half * talus::dynamics::cornerSigns(corner);
    cell.nodes[corner] = corner;
    cell.inverseMass[corner] = 1.0 / (mass * (1.0 + 0.1 * corner));
  }

  return cell;
}

talus::dynamics::StrainElement testElement(const TestCell& cell, double youngs, double poisson, double h) {
  talus::PhysicalMaterial material;

  material.youngsModulus = static_cast<float>(youngs);
  material.poissonsRatio = static_cast<float>(poisson);

  const double side = cell.rest[7].x - cell.rest[0].x;
  const std::optional<talus::dynamics::StrainElement> element = talus::dynamics::makeElement(
    0u,
    cell.nodes,
    cell.inverseMass,
    talus::dynamics::cornerRest(cell.rest, side * side * side),
    talus::dynamics::elementMaterial(material),
    h,
    talus::dynamics::ElementDamping{}
  );

  REQUIRE(element.has_value());

  return *element;
}

double expectedStiffness(uint32_t row, double youngs, double poisson, double volume) {
  const double shear = youngs / (2.0 * (1.0 + poisson));

  return row < 3u ? 2.0 * shear * volume : row < 6u ? 4.0 * shear * volume : youngs * volume / 3.0;
}

double expectedVolumeStiffness(double youngs, double poisson, double volume) {
  return youngs * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson)) * volume;
}

double expectedPattern(uint32_t mode, uint32_t corner) {
  const glm::dvec3 sign = talus::dynamics::cornerSigns(corner);

  return mode == 0u   ? sign.x * sign.y
         : mode == 1u ? sign.y * sign.z
         : mode == 2u ? sign.z * sign.x
                      : sign.x * sign.y * sign.z;
}

std::vector<talus::dynamics::StrainElement> meshElements(
  const talus::PreparedMesh& mesh,
  const talus::PhysicalMaterial& material
) {
  std::vector<talus::dynamics::StrainElement> elements;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    std::array<glm::dvec3, 8> corners{};
    std::array<double, 8> weights{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const uint32_t node = hex.nodes[corner];

      corners[corner] = glm::dvec3(mesh.restPositions[node]);
      weights[corner] =
        mesh.anchored[node] != 0u ? 0.0 : 1.0 / static_cast<double>(mesh.nodeMass[node]);
    }

    const std::optional<talus::dynamics::StrainElement> element = talus::dynamics::makeElement(
      cell,
      hex.nodes,
      weights,
      talus::dynamics::cornerRest(corners, hex.volume),
      talus::dynamics::elementMaterial(material),
      1.0 / 240.0,
      talus::dynamics::ElementDamping{}
    );

    REQUIRE(element.has_value());
    elements.push_back(*element);
  }

  return elements;
}

double fastestNode(const talus::dynamics::LatticeBody& lattice) {
  double fastest = 0.0;

  for (const glm::dvec3& velocity : lattice.velocity) {
    fastest = std::max(fastest, glm::length(velocity));
  }

  return fastest;
}

uint32_t nearestNode(const talus::PreparedMesh& mesh, const glm::dvec3& point) {
  uint32_t nearest = 0u;

  for (uint32_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (glm::length(glm::dvec3(mesh.restPositions[node]) - point)
        < glm::length(glm::dvec3(mesh.restPositions[nearest]) - point)) {
      nearest = node;
    }
  }

  return nearest;
}

talus::PhysicalMaterial beam(double youngs, double poisson) {
  talus::PhysicalMaterial material = jelly(youngs, 5.0);

  material.friction = 0.5f;
  material.poissonsRatio = static_cast<float>(poisson);

  return material;
}

struct Cantilever {
  double sag = 0.0;
  double theory = 0.0;
  size_t cells = 0u;
  uint32_t substeps = 0u;
};

struct SubstepChange {
  uint32_t frame = 0u;
  uint32_t substeps = 0u;
};

Cantilever cantilever(
  double thickness,
  double cell,
  const talus::PhysicalMaterial& material,
  uint32_t substeps,
  uint32_t steps,
  const SubstepChange& change = SubstepChange{}
) {
  constexpr double LENGTH = 4.0;
  const std::array<talus::deformable::SupportVolume, 1> clamp{
    holdAt(glm::dvec3(-LENGTH / 2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const SoftShape shape =
    softBox(glm::dvec3(LENGTH / 2.0, thickness / 2.0, thickness / 2.0), cell, material, clamp);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));

  world.bodies.lattices[index].substeps = substeps;

  const uint32_t tip =
    nearestNode(*shape.mesh, glm::dvec3(LENGTH / 2.0, thickness / 2.0, thickness / 2.0));
  const uint32_t window = steps / 2u;
  Cantilever result;

  for (uint32_t frame = 1u; frame <= steps; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    result.substeps = report.substeps;

    if (change.substeps != 0u && frame == change.frame) {
      world.bodies.lattices[index].substeps = change.substeps;
    }

    if (frame > steps - window) {
      result.sag += (static_cast<double>(shape.mesh->restPositions[tip].y)
                     - world.bodies.lattices[index].position[tip].y)
                    / window;
    }
  }

  const double inertia = std::pow(thickness, 4.0) / 12.0;

  result.theory = material.density * 9.81 * thickness * thickness * std::pow(LENGTH, 4.0)
                  / (8.0 * material.youngsModulus * inertia);
  result.cells = shape.mesh->hexes.size();

  return result;
}

double latticeVolume(const talus::dynamics::LatticeBody& lattice) {
  double total = 0.0;

  for (const talus::PreparedHex& hex : lattice.mesh->hexes) {
    glm::dmat3 shape(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      shape += glm::outerProduct(lattice.position[hex.nodes[corner]], talus::dynamics::cornerSigns(corner)) / 8.0;
    }

    total += 8.0 * glm::determinant(shape);
  }

  return total;
}

talus::field::SampledSdf floorField() {
  return boxField(glm::dvec3(4.0, 0.2, 4.0), 0.1);
}

void addFloor(World& world, const talus::field::SampledSdf& floor, double friction) {
  (void)talus::dynamics::addStatic(world.bodies, floor, friction, 0.0, glm::dvec3(0.0, -0.2, 0.0));
}

double latticeMass(const talus::dynamics::LatticeBody& lattice) {
  double total = 0.0;

  for (const float mass : lattice.mesh->nodeMass) {
    total += mass;
  }

  return total;
}

glm::dvec3 latticeCentre(const talus::dynamics::LatticeBody& lattice) {
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    centre += static_cast<double>(lattice.mesh->nodeMass[node]) * lattice.position[node];
  }

  return centre / latticeMass(lattice);
}

glm::dvec3 latticeMomentum(const talus::dynamics::LatticeBody& lattice) {
  glm::dvec3 momentum(0.0);

  for (size_t node = 0u; node != lattice.velocity.size(); ++node) {
    momentum += static_cast<double>(lattice.mesh->nodeMass[node]) * lattice.velocity[node];
  }

  return momentum;
}

glm::dvec3 latticeAngularMomentum(const talus::dynamics::LatticeBody& lattice) {
  glm::dvec3 momentum(0.0);

  for (size_t node = 0u; node != lattice.velocity.size(); ++node) {
    momentum += static_cast<double>(lattice.mesh->nodeMass[node])
                * glm::cross(lattice.position[node], lattice.velocity[node]);
  }

  return momentum;
}

double lowestCellRatio(const talus::dynamics::LatticeBody& lattice) {
  double lowest = std::numeric_limits<double>::infinity();

  for (const talus::PreparedHex& hex : lattice.mesh->hexes) {
    glm::dmat3 now(0.0);
    glm::dmat3 rest(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::dvec3 sign = talus::dynamics::cornerSigns(corner);

      now += glm::outerProduct(lattice.position[hex.nodes[corner]], sign);
      rest += glm::outerProduct(glm::dvec3(lattice.mesh->restPositions[hex.nodes[corner]]), sign);
    }

    lowest = std::min(lowest, glm::determinant(now) / glm::determinant(rest));
  }

  return lowest;
}

double rebound(double cell, uint32_t substeps) {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial material = jelly(1.0e4, 100.0);

  material.dampingRatio = 0.2f;

  const SoftShape shape = softBox(glm::dvec3(0.3), cell, material);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.6);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 3.0, 0.0));

  world.bodies.lattices[index].substeps = substeps;

  double previous = latticeCentre(world.bodies.lattices[index]).y;
  double apex = -std::numeric_limits<double>::infinity();
  bool landed = false;

  for (uint32_t frame = 1u; frame <= 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const double y = latticeCentre(world.bodies.lattices[index]).y;

    if (!landed && y > previous && previous < 1.0) {
      landed = true;
    }

    if (landed) {
      apex = std::max(apex, y);
    }

    previous = y;
  }

  return std::sqrt(std::max(0.0, apex - 0.3) / 2.7);
}

double cachedLoad(const World& world, uint32_t first, uint32_t second) {
  double load = 0.0;
  const uint32_t id = world.bodies.rigids[first].id;
  const uint32_t other = world.bodies.rigids[second].id;

  for (const uint32_t slot : world.manifolds.pairsOf(id)) {
    const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

    if (!((pair.firstId == id && pair.secondId == other) || (pair.firstId == other && pair.secondId == id))) {
      continue;
    }

    for (const talus::dynamics::CachedContactPoint& point : pair.points) {
      load += point.memory.normal;
    }
  }

  return load;
}

bool restingOn(const World& world, uint32_t rigid) {
  const uint32_t id = world.bodies.rigids[rigid].id;

  for (const uint32_t slot : world.manifolds.pairsOf(id)) {
    const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

    if (pair.second.side != talus::dynamics::Side::Static) {
      continue;
    }

    for (const talus::dynamics::CachedContactPoint& point : pair.points) {
      if (point.memory.normal > 0.0) {
        return true;
      }
    }
  }

  return false;
}

double elasticEnergy(const talus::dynamics::LatticeBody& lattice) {
  const talus::PreparedMesh& mesh = *lattice.mesh;
  double energy = 0.0;

  for (uint32_t cell = 0u; cell != mesh.hexes.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    const talus::PhysicalMaterial& material =
      lattice.materials[std::min<size_t>(hex.material, lattice.materials.size() - 1u)];
    std::array<glm::dvec3, 8> corners{};
    std::array<double, 8> weights{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      corners[corner] = glm::dvec3(mesh.restPositions[hex.nodes[corner]]);
      weights[corner] = 1.0;
    }

    std::optional<talus::dynamics::StrainElement> element = talus::dynamics::makeElement(
      cell,
      hex.nodes,
      weights,
      talus::dynamics::cornerRest(corners, hex.volume),
      talus::dynamics::elementMaterial(material),
      1.0 / 240.0,
      talus::dynamics::ElementDamping{}
    );

    if (!element) {
      continue;
    }

    talus::dynamics::updateElement(*element, lattice.position);

    glm::dmat3 deformation(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      deformation += glm::outerProduct(lattice.position[hex.nodes[corner]], element->gradient[corner]);
    }

    for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
      energy += 0.5 * element->stiffness[row] * element->strain[row] * element->strain[row];
    }

    const double volume = glm::determinant(deformation) - 1.0;

    energy += 0.5 * element->volumeStiffness * volume * volume;
  }

  return energy;
}

double latticeEnergy(const talus::dynamics::LatticeBody& lattice) {
  double energy = elasticEnergy(lattice);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const double mass = lattice.mesh->nodeMass[node];

    energy += 0.5 * mass * glm::dot(lattice.velocity[node], lattice.velocity[node])
              + mass * 9.81 * lattice.position[node].y;
  }

  return energy;
}

struct Drop {
  double peak = 0.0;
  uint32_t substeps = 0u;
};

Drop dropStiff(double youngs) {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial stiff = jelly(youngs, 1000.0);

  stiff.poissonsRatio = 0.3f;

  const SoftShape shape = softBox(glm::dvec3(0.2, 0.6, 0.2), 0.1, stiff);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.1, 0.0));
  const double start = latticeEnergy(world.bodies.lattices[index]);
  Drop drop;

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    drop.substeps = report.substeps;
    drop.peak = std::max(drop.peak, latticeEnergy(world.bodies.lattices[index]) / start);
  }

  return drop;
}

SoftShape resolvedBox(
  const glm::dvec3& half,
  double voxel,
  const talus::PhysicalMaterial& material,
  uint32_t& coarsening,
  std::span<const talus::deformable::SupportVolume> supports = {}
) {
  SoftShape shape;
  talus::dynamics::IntegrationParameters parameters;

  shape.field = boxField(half, voxel, 1.0e-4);
  shape.lanes.assign(2u, material);

  const talus::dynamics::ResolvedMesh resolved =
    talus::dynamics::resolveMesh(shape.field, shape.lanes, glm::mat4(1.0f), supports, parameters);

  shape.mesh = resolved.mesh;
  coarsening = resolved.coarsening;

  return shape;
}

SoftShape gridPlaneBox(
  const glm::dvec3& half,
  double voxel,
  const talus::PhysicalMaterial& material
) {
  SoftShape shape;
  const talus::dynamics::IntegrationParameters parameters;

  shape.field = boxField(half, voxel);
  shape.lanes.assign(2u, material);
  shape.mesh = talus::dynamics::resolveMesh(shape.field, shape.lanes, glm::mat4(1.0f), {}, parameters).mesh;

  return shape;
}

double cellVolume(const talus::PreparedMesh& mesh) {
  double volume = 0.0;

  for (const talus::PreparedHex& hex : mesh.hexes) {
    volume += hex.volume;
  }

  return volume;
}

uint32_t flowedLattice(
  World& world,
  const SoftShape& shape,
  const glm::dmat3& flow,
  const glm::dvec3& offset = glm::dvec3(0.0, 1.0, 0.0)
) {
  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, offset, glm::dvec3(0.0), 0.5, 0.0
  );

  talus::dynamics::promote(world, id);

  const std::optional<talus::dynamics::BodyRef> soft = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(soft.has_value());
  REQUIRE(soft->side == talus::dynamics::Side::Lattice);

  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[soft->index];

  lattice.plastic.assign(8u * shape.mesh->hexes.size(), glm::inverse(flow));
  lattice.hardening.assign(lattice.plastic.size(), 0.0);
  lattice.position = talus::dynamics::plasticShape(lattice, 16u);

  return id;
}

talus::PhysicalMaterial stiffBeam(double youngs) {
  talus::PhysicalMaterial material = jelly(youngs, 1000.0);

  material.poissonsRatio = 0.3f;

  return material;
}

uint32_t nodesIn(
  const talus::PreparedMesh& mesh,
  const talus::deformable::SupportVolume& support
) {
  uint32_t count = 0u;

  for (const glm::vec3& rest : mesh.restPositions) {
    const glm::vec3 local = glm::vec3(support.worldToSupport * glm::vec4(rest, 1.0f));

    count += glm::all(glm::lessThanEqual(glm::abs(local), support.halfExtents)) ? 1u : 0u;
  }

  return count;
}

talus::PhysicalMaterial clay(double youngs, double yield) {
  talus::PhysicalMaterial material = jelly(youngs, 1000.0);

  material.poissonsRatio = 0.35f;
  material.yieldStress = static_cast<float>(yield);
  material.hardeningModulus = 0.0f;
  material.plasticMax = 1.0f;

  return material;
}

glm::dmat3 plasticStretchOf(const glm::dmat3& inverse) {
  return glm::inverse(inverse);
}

double frobenius(const glm::dmat3& m) {
  double total = 0.0;

  for (int column = 0; column != 3; ++column) {
    total += glm::dot(m[column], m[column]);
  }

  return std::sqrt(total);
}

void setAnchored(talus::dynamics::LatticeBody& lattice, std::span<const uint32_t> nodes, uint8_t value) {
  auto copy = std::make_shared<talus::PreparedMesh>(*lattice.mesh);

  if (copy->anchored.empty()) {
    copy->anchored.assign(copy->restPositions.size(), 0u);
  }

  for (const uint32_t node : nodes) {
    copy->anchored[node] = value;
  }

  lattice.mesh = copy;
}

template <typename Predicate>
std::vector<uint32_t> nodesWhere(const talus::PreparedMesh& mesh, const Predicate& predicate) {
  std::vector<uint32_t> nodes;

  for (uint32_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (predicate(glm::dvec3(mesh.restPositions[node]))) {
      nodes.push_back(node);
    }
  }

  return nodes;
}

double lean(double yieldStrain) {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial block = jelly(2.0e4, 50.0);

  block.poissonsRatio = 0.3f;
  block.friction = 2.0f;
  block.dampingRatio = 1.0f;
  block.yieldStress = yieldStrain > 0.0 ? static_cast<float>(2.0e4 * yieldStrain) : std::numeric_limits<float>::infinity();
  block.plasticMax = 1.0f;

  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(0.0, -0.75, 0.0), glm::dvec3(1.0, 0.01, 1.0)),
    holdAt(glm::dvec3(0.0, 0.75, 0.0), glm::dvec3(1.0, 0.01, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(0.75), 0.375, block, ends);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.6);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.75, 0.0));
  const std::vector<uint32_t> bottom = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.y < -0.74; });
  const std::vector<uint32_t> top = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.y > 0.74; });
  std::vector<double> start;

  for (const uint32_t node : top) {
    start.push_back(world.bodies.lattices[index].position[node].x);
  }

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    const double shift = 0.9 * std::min(static_cast<double>(frame) / 60.0, 1.0);

    for (size_t at = 0u; at != top.size(); ++at) {
      world.bodies.lattices[index].position[top[at]].x = start[at] + shift;
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  setAnchored(world.bodies.lattices[index], top, 0u);

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const auto mean = [&](const std::vector<uint32_t>& set) {
    double total = 0.0;

    for (const uint32_t node : set) {
      total += world.bodies.lattices[index].position[node].x;
    }

    return total / static_cast<double>(set.size());
  };

  return mean(top) - mean(bottom);
}

double plasticStrain(const talus::dynamics::LatticeBody& lattice) {
  double total = 0.0;

  for (const double strain : lattice.hardening) {
    total += strain / 8.0;
  }

  return total;
}

SoftShape stripShape() {
  talus::PhysicalMaterial strip = jelly(1.0e5, 10.0);

  strip.poissonsRatio = 0.3f;
  strip.friction = 0.5f;
  strip.dampingRatio = 1.0f;
  strip.yieldStress = 1.0e5f * 0.002f;
  strip.plasticMax = 1.0f;

  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };

  return softBox(glm::dvec3(1.2, 0.1, 0.1), 0.2, strip, ends);
}

uint32_t orphans(const talus::dynamics::LatticeBody& lattice) {
  uint32_t count = 0u;

  for (size_t node = 0u; node + 1u < lattice.cellStart.size(); ++node) {
    count += lattice.cellStart[node + 1u] == lattice.cellStart[node] ? 1u : 0u;
  }

  return count;
}

double longestEdge(const talus::dynamics::LatticeBody& lattice) {
  double longest = 0.0;

  for (const talus::PreparedHex& hex : lattice.mesh->hexes) {
    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      for (uint32_t axis = 0u; axis != 3u; ++axis) {
        const uint32_t other = corner ^ (1u << axis);

        if (other < corner) {
          continue;
        }

        const double now = glm::length(lattice.position[hex.nodes[corner]] - lattice.position[hex.nodes[other]]);
        const double rest = glm::length(glm::dvec3(lattice.mesh->restPositions[hex.nodes[corner]])
                                        - glm::dvec3(lattice.mesh->restPositions[hex.nodes[other]]));

        longest = std::max(longest, now / rest);
      }
    }
  }

  return longest;
}

talus::PhysicalMaterial tearable() {
  talus::PhysicalMaterial material = jelly(5.0e3, 20.0);

  material.poissonsRatio = 0.3f;
  material.tearStrain = 0.4f;

  return material;
}

struct SurfaceFlaws {
  uint32_t edgesAcross = 0u;
  uint32_t trianglesAcross = 0u;
  uint32_t cracks = 0u;
  uint32_t cracksBare = 0u;
  uint32_t nodesOff = 0u;
  uint32_t arraysOff = 0u;

  [[nodiscard]] uint32_t defects() const {
    return edgesAcross + trianglesAcross + cracksBare + nodesOff + arraysOff;
  }
};

SurfaceFlaws surfaceFlaws(const talus::PreparedMesh& mesh) {
  std::vector<uint32_t> parent(mesh.restPositions.size());

  std::iota(parent.begin(), parent.end(), 0u);

  const auto rootOf = [&](uint32_t at) {
    while (parent[at] != at) {
      parent[at] = parent[parent[at]];
      at = parent[at];
    }

    return at;
  };

  for (const talus::PreparedHex& hex : mesh.hexes) {
    const uint32_t first = rootOf(hex.nodes[0]);

    for (const uint32_t node : hex.nodes) {
      parent[rootOf(node)] = first;
    }
  }

  std::vector<uint8_t> onSurface(mesh.restPositions.size(), 0u);

  for (const uint32_t node : mesh.surfaceNodes) {
    onSurface[node] = 1u;
  }

  const size_t triangles = mesh.surfaceTriangles.size();
  SurfaceFlaws flaws;

  for (const glm::uvec2& edge : mesh.surfaceEdges) {
    flaws.edgesAcross += rootOf(edge.x) == rootOf(edge.y) ? 0u : 1u;
  }

  for (const glm::uvec3& triangle : mesh.surfaceTriangles) {
    flaws.trianglesAcross +=
      rootOf(triangle.x) == rootOf(triangle.y) && rootOf(triangle.y) == rootOf(triangle.z) ? 0u : 1u;

    for (glm::length_t corner = 0; corner != 3; ++corner) {
      flaws.nodesOff += onSurface[triangle[corner]] != 0u ? 0u : 1u;
    }
  }

  std::vector<uint8_t> listed(triangles, 0u);

  for (const uint32_t face : mesh.surfaceClusterFaces) {
    if (face < triangles && listed[face] == 0u) {
      listed[face] = 1u;
    } else {
      ++flaws.arraysOff;
    }
  }

  flaws.arraysOff += mesh.surfaceThickness.size() == triangles ? 0u : 1u;
  flaws.arraysOff += mesh.surfaceLane.size() == triangles ? 0u : 1u;
  flaws.arraysOff += mesh.surfaceClusterFaces.size() == triangles ? 0u : 1u;
  flaws.arraysOff += !mesh.surfaceClusterStarts.empty() && mesh.surfaceClusterStarts.back() == triangles ? 0u : 1u;

  for (uint32_t cell = 0u; cell != mesh.crackFaces.size(); ++cell) {
    const talus::PreparedHex& hex = mesh.hexes[cell];
    glm::dvec3 centre(0.0);

    for (const uint32_t node : hex.nodes) {
      centre += glm::dvec3(mesh.restPositions[node]) / 8.0;
    }

    for (uint32_t face = 0u; face != 6u; ++face) {
      if ((mesh.crackFaces[cell] & (1u << face)) == 0u) {
        continue;
      }

      std::array<uint32_t, 4> corners{};
      size_t held = 0u;

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        if (((corner >> (face / 2u)) & 1u) == face % 2u) {
          corners[held++] = hex.nodes[corner];
        }
      }

      const auto lies = [&](uint32_t node) { return std::find(corners.begin(), corners.end(), node) != corners.end(); };
      bool outward = false;

      for (const glm::uvec3& triangle : mesh.surfaceTriangles) {
        if (!lies(triangle.x) || !lies(triangle.y) || !lies(triangle.z)) {
          continue;
        }

        const glm::dvec3 a(mesh.restPositions[triangle.x]);
        const glm::dvec3 b(mesh.restPositions[triangle.y]);
        const glm::dvec3 c(mesh.restPositions[triangle.z]);

        outward = outward || glm::dot(glm::cross(b - a, c - a), centre - a) < 0.0;
      }

      ++flaws.cracks;
      flaws.cracksBare += outward ? 0u : 1u;
    }
  }

  return flaws;
}

SoftShape pinchedBoxes() {
  SoftShape shape;
  const glm::vec3 reach(2.0f);

  shape.field = talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(0.5f)),
      fixture::box(glm::vec3(0.5f, 0.5f, 0.0f), glm::vec3(0.5f))
    },
    -reach,
    reach,
    glm::uvec3(9u)
  );
  shape.lanes.assign(2u, tearable());
  shape.mesh = talus::deformable::buildMesh(
    shape.field,
    shape.lanes,
    glm::mat4(1.0f),
    {},
    talus::deformable::VolumeElementType::Hex
  );

  return shape;
}

}

TEST_CASE("a soft island goes to strainstep", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3));
  World world;

  world.parameters.sleep = false;
  (void)addSoft(world, shape, glm::dvec3(0.0, 5.0, 0.0));

  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  CHECK(report.strainstepIslands == 1u);
  CHECK(report.softstepIslands == 0u);
}

TEST_CASE("falls freely as semi-implicit Euler with four substeps", "[strainstep]") {
  checkFreeFall(0u, 4u);
}

TEST_CASE("runs an island at the substeps its lattices ask for", "[strainstep]") {
  checkFreeFall(8u, 8u);
}

TEST_CASE("integrates a rigid body in a soft island as semi-implicit Euler", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3));
  const talus::field::SampledSdf ball = ballField(0.1, 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  (void)addSoft(world, shape, glm::dvec3(0.0, 50.0, 0.0));

  const uint32_t rigid = talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.1,
    1.0,
    glm::dvec3(0.0, 50.41, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  const glm::dvec3 start = world.bodies.rigids[rigid].position;
  constexpr uint32_t STEPS = 30u;

  for (uint32_t frame = 0u; frame != STEPS; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(report.strainstepIslands == 1u);
    REQUIRE(report.softstepIslands == 0u);
  }

  const double h = world.parameters.dt / 4.0;
  const double n = 4.0 * STEPS;
  const talus::dynamics::RigidBody& body = world.bodies.rigids[rigid];

  CHECK(std::abs(body.velocity.y + 9.81 * h * n) <= 1.0e-9);
  CHECK(std::abs(body.position.y - start.y + 9.81 * h * h * n * (n + 1.0) / 2.0) <= 1.0e-9);
}

TEST_CASE("measures no strain in a hex element turned and moved, and gives it no impulse", "[strainstep]") {
  const TestCell cell = testCell(0.1, 1.0);
  talus::dynamics::StrainElement element = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const glm::dquat turn = glm::angleAxis(2.0, glm::normalize(glm::dvec3(1.0, 2.0, 3.0)));
  std::vector<glm::dvec3> positions(8u);
  std::vector<glm::dvec3> velocities(8u, glm::dvec3(0.0));

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    positions[corner] = glm::dvec3(0.5, -1.0, 2.0) + turn * cell.rest[corner];
  }

  element.rotation = turn;
  talus::dynamics::updateElement(element, positions);

  for (const double strain : element.strain) {
    CHECK(std::abs(strain) <= 1.0e-12);
  }

  talus::dynamics::solveElement(element, velocities);

  for (const glm::dvec3& velocity : velocities) {
    CHECK(glm::length(velocity) <= 1.0e-12);
  }
}

TEST_CASE("finds the rotation of a stretched and turned hex from a guess half a radian away", "[strainstep]") {
  const glm::dquat turn = glm::angleAxis(0.5, glm::normalize(glm::dvec3(1.0, -1.0, 2.0)));
  const glm::dmat3 stretch(
    glm::dvec3(1.1, 0.02, 0.0),
    glm::dvec3(0.02, 0.95, 0.01),
    glm::dvec3(0.0, 0.01, 1.05)
  );
  const glm::dquat found =
    talus::dynamics::rotationOf(glm::mat3_cast(turn) * stretch, glm::dquat(1.0, 0.0, 0.0, 0.0));
  const glm::dmat3 difference = glm::mat3_cast(found) - glm::mat3_cast(turn);

  for (int column = 0; column != 3; ++column) {
    CHECK(glm::length(difference[column]) <= 1.0e-6);
  }
}

TEST_CASE("keeps a deformed hex element's momentum through its solves", "[strainstep]") {
  const TestCell cell = testCell(0.1, 2.0);
  talus::dynamics::StrainElement element = testElement(cell, 1.0e5, 0.45, 1.0 / 240.0);
  const glm::dmat3 shape(
    glm::dvec3(1.08, 0.03, -0.02),
    glm::dvec3(-0.05, 0.93, 0.04),
    glm::dvec3(0.02, -0.01, 1.11)
  );
  std::vector<glm::dvec3> positions(8u);
  std::vector<glm::dvec3> velocities(8u);
  glm::dvec3 before(0.0);
  double scale = 0.0;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const double turn = static_cast<double>(corner);

    positions[corner] = shape * cell.rest[corner]
                        + 1.0e-3 * glm::dvec3(std::sin(1.0 + turn), std::cos(2.0 * turn), std::sin(3.0 * turn));
    velocities[corner] = glm::dvec3(std::cos(turn), 0.5 * std::sin(2.0 * turn), -0.3 * turn);
    before += velocities[corner] / cell.inverseMass[corner];
    scale += glm::length(velocities[corner]) / cell.inverseMass[corner];
  }

  talus::dynamics::updateElement(element, positions);
  talus::dynamics::solveElement(element, velocities);
  talus::dynamics::solveElement(element, velocities);

  glm::dvec3 after(0.0);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    after += velocities[corner] / cell.inverseMass[corner];
  }

  CHECK(glm::length(after - before) <= 1.0e-12 * scale);
}

TEST_CASE("solves a hex element's rows exactly in one block solve", "[strainstep]") {
  const TestCell cell = testCell(0.1, 1.0);
  talus::dynamics::StrainElement element = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const glm::dquat turn = glm::angleAxis(0.3, glm::normalize(glm::dvec3(0.0, 1.0, 1.0)));
  const glm::dmat3 shape(glm::dvec3(1.05, 0.0, 0.0), glm::dvec3(0.02, 0.97, 0.0), glm::dvec3(0.0, 0.01, 1.02));
  std::vector<glm::dvec3> positions(8u);
  std::vector<glm::dvec3> velocities(8u, glm::dvec3(0.0));

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    positions[corner] = turn * (shape * cell.rest[corner]);
  }

  element.rotation = turn;
  talus::dynamics::updateElement(element, positions);
  talus::dynamics::solveElement(element, velocities);

  const std::array<double, talus::dynamics::ELEMENT_ROWS> first = element.impulse;
  const double firstVolume = element.volumeImpulse;

  talus::dynamics::solveElement(element, velocities);

  for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
    CHECK(std::abs(element.impulse[row] - first[row]) <= 1.0e-9 * (1.0 + std::abs(first[row])));
  }

  CHECK(std::abs(element.volumeImpulse - firstVolume) <= 1.0e-9 * (1.0 + std::abs(firstVolume)));
  CHECK(std::abs(first[0]) > 0.0);
}

TEST_CASE("bends a hex element with the energy beam theory gives one cell of a beam", "[strainstep]") {
  constexpr double HALF = 0.1;
  constexpr double SIDE = 2.0 * HALF;
  constexpr double CURVATURE = 0.05;
  constexpr double YOUNGS = 1.0e5;
  const TestCell cell = testCell(HALF, 1.0);
  talus::dynamics::StrainElement element = testElement(cell, YOUNGS, 0.3, 1.0 / 240.0);
  std::vector<glm::dvec3> positions(8u);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3& rest = cell.rest[corner];

    positions[corner] = rest + glm::dvec3(-CURVATURE * rest.x * rest.y, 0.0, 0.0);
  }

  talus::dynamics::updateElement(element, positions);

  for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
    if (row != 6u) {
      CHECK(std::abs(element.strain[row]) <= 1.0e-12);
    }
  }

  const double energy = 0.5 * element.stiffness[6] * element.strain[6] * element.strain[6];
  const double theory = YOUNGS * CURVATURE * CURVATURE * std::pow(SIDE, 5.0) / 24.0;

  CHECK(std::abs(energy - theory) <= 1.0e-9 * theory);
}

TEST_CASE("sets a hex element's row stiffness from Young's modulus and Poisson's ratio", "[strainstep]") {
  constexpr double HALF = 0.1;
  constexpr double SIDE = 2.0 * HALF;
  constexpr double VOLUME = SIDE * SIDE * SIDE;
  constexpr double YOUNGS = 1.0e5;
  constexpr double POISSON = 0.375;
  const TestCell cell = testCell(HALF, 1.0);
  const talus::dynamics::StrainElement element = testElement(cell, YOUNGS, POISSON, 1.0 / 240.0);

  for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
    const double expected = expectedStiffness(row, YOUNGS, POISSON, VOLUME);

    CHECK(std::abs(element.stiffness[row] - expected) <= 1.0e-9 * expected);
  }

  const double expectedVolume = expectedVolumeStiffness(YOUNGS, POISSON, VOLUME);

  CHECK(std::abs(element.volumeStiffness - expectedVolume) <= 1.0e-9 * expectedVolume);
}

TEST_CASE("measures the six strain rows of a stretched and sheared hex element", "[strainstep]") {
  const TestCell cell = testCell(0.1, 1.0);
  talus::dynamics::StrainElement element = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const glm::dmat3 stretch(
    glm::dvec3(1.02, 0.03, 0.01),
    glm::dvec3(0.03, 0.99, 0.02),
    glm::dvec3(0.01, 0.02, 1.04)
  );
  const std::array<double, 6> expected{0.02, -0.01, 0.04, 0.03, 0.02, 0.01};
  std::vector<glm::dvec3> positions(8u);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    positions[corner] = stretch * cell.rest[corner];
  }

  talus::dynamics::updateElement(element, positions);

  for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
    CHECK(std::abs(element.strain[row] - (row < 6u ? expected[row] : 0.0)) <= 1.0e-12);
  }
}

TEST_CASE("measures no strain in a sheared or warped hex element that only turns and moves", "[strainstep]") {
  constexpr double HALF = 0.1;
  const glm::dmat3 shear(glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.3, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0));
  const glm::dquat turn = glm::angleAxis(1.1, glm::normalize(glm::dvec3(-1.0, 0.5, 2.0)));
  TestCell sheared = testCell(HALF, 1.0);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    sheared.rest[corner] = HALF * (shear * talus::dynamics::cornerSigns(corner));
  }

  TestCell warped = sheared;

  warped.rest[7] += glm::dvec3(0.02, 0.01, -0.015);
  warped.rest[3] += glm::dvec3(-0.01, 0.0, 0.02);

  const talus::dynamics::ElementRest shearedRest = talus::dynamics::cornerRest(sheared.rest, 1.0);
  const talus::dynamics::ElementRest warpedRest = talus::dynamics::cornerRest(warped.rest, 1.0);

  REQUIRE(std::abs(shearedRest.shape[1][0] - shearedRest.shape[0][1]) > 1.0e-3);
  REQUIRE(std::abs(warpedRest.shape[1][0] - warpedRest.shape[0][1]) > 1.0e-3);
  REQUIRE(glm::length(warpedRest.hourglass[0]) > 1.0e-3);

  for (const TestCell& cell : {sheared, warped}) {
    talus::dynamics::StrainElement element = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
    std::vector<glm::dvec3> positions(8u);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      positions[corner] = glm::dvec3(1.0, 2.0, 3.0) + turn * cell.rest[corner];
    }

    element.rotation = turn;
    talus::dynamics::updateElement(element, positions);

    for (const double strain : element.strain) {
      CHECK(std::abs(strain) <= 1.0e-12);
    }

    CHECK(std::abs(element.rhs / element.volumeRate) <= 1.0e-12);
  }
}

TEST_CASE("gives a hex element a volume gradient equal to the derivative of its volume", "[strainstep]") {
  constexpr double DELTA = 1.0e-6;
  const TestCell cell = testCell(0.1, 1.0);
  talus::dynamics::StrainElement element = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const glm::dquat turn = glm::angleAxis(0.3, glm::normalize(glm::dvec3(1.0, 2.0, 3.0)));
  const glm::dmat3 stretch(
    glm::dvec3(1.02, 0.03, 0.01),
    glm::dvec3(0.03, 0.99, 0.02),
    glm::dvec3(0.01, 0.02, 1.04)
  );
  std::vector<glm::dvec3> positions(8u);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    positions[corner] = turn * (stretch * cell.rest[corner]);
  }

  talus::dynamics::updateElement(element, positions);

  const auto volumeAt = [&element](const std::vector<glm::dvec3>& moved) {
    talus::dynamics::StrainElement copy = element;

    talus::dynamics::updateElement(copy, moved);

    return copy.rhs / copy.volumeRate + 1.0;
  };

  CHECK(std::abs(volumeAt(positions) - glm::determinant(stretch)) <= 1.0e-12);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    for (int axis = 0; axis != 3; ++axis) {
      std::vector<glm::dvec3> forward = positions;
      std::vector<glm::dvec3> backward = positions;

      forward[corner][axis] += DELTA;
      backward[corner][axis] -= DELTA;

      const double slope = (volumeAt(forward) - volumeAt(backward)) / (2.0 * DELTA);

      CHECK(std::abs(slope - element.volumeGradient[corner][axis]) <= 1.0e-7);
    }
  }
}

TEST_CASE("sets each hex element row's bias rate and softness like a critically damped spring", "[strainstep]") {
  constexpr double HALF = 0.1;
  constexpr double SIDE = 2.0 * HALF;
  constexpr double VOLUME = SIDE * SIDE * SIDE;
  constexpr double YOUNGS = 1.0e5;
  constexpr double POISSON = 0.375;
  constexpr double SUBSTEP = 1.0 / 240.0;
  constexpr double GRADIENT = 1.0 / (8.0 * HALF);
  constexpr double BEND = 1.0 / (4.0 * SIDE);
  const TestCell cell = testCell(HALF, 1.0);
  const talus::dynamics::StrainElement element = testElement(cell, YOUNGS, POISSON, SUBSTEP);
  double inverseMass = 0.0;

  for (const double weight : cell.inverseMass) {
    inverseMass += weight;
  }

  const auto expectSoft = [](double stiffness, double weight, double rate, double softness) {
    const double hertz = std::sqrt(stiffness * weight) / (2.0 * std::numbers::pi);
    const talus::dynamics::Softness soft = talus::dynamics::makeSoft(hertz, 1.0, SUBSTEP);

    CHECK(std::abs(rate - soft.biasRate) <= 1.0e-9 * soft.biasRate);
    CHECK(std::abs(softness - soft.impulseScale / soft.massScale) <= 1.0e-9 * softness);
  };

  for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
    const double jacobianSquare =
      row < 3u ? GRADIENT * GRADIENT : row < 6u ? 0.5 * GRADIENT * GRADIENT : BEND * BEND;
    const double weight = jacobianSquare * inverseMass;

    expectSoft(
      expectedStiffness(row, YOUNGS, POISSON, VOLUME),
      weight,
      element.rate[row],
      element.softness[row] / weight
    );
  }

  expectSoft(
    expectedVolumeStiffness(YOUNGS, POISSON, VOLUME),
    3.0 * GRADIENT * GRADIENT * inverseMass,
    element.volumeRate,
    element.volumeSoftness
  );
}

TEST_CASE("moves the corners of a hex element down its energy gradient in a tiny substep", "[strainstep]") {
  constexpr double HALF = 0.1;
  constexpr double SIDE = 2.0 * HALF;
  constexpr double VOLUME = SIDE * SIDE * SIDE;
  constexpr double YOUNGS = 1.0e5;
  constexpr double POISSON = 0.375;
  constexpr double SUBSTEP = 1.0e-6;
  constexpr double DELTA = 1.0e-6;
  constexpr double BEND = 1.0 / (4.0 * SIDE);
  const TestCell cell = testCell(HALF, 1.0);
  talus::dynamics::StrainElement element = testElement(cell, YOUNGS, POISSON, SUBSTEP);
  const glm::dquat turn = glm::angleAxis(0.4, glm::normalize(glm::dvec3(1.0, -2.0, 2.0)));
  const glm::dmat3 back = glm::transpose(glm::mat3_cast(turn));
  const glm::dmat3 stretch(
    glm::dvec3(1.012, 0.008, 0.007),
    glm::dvec3(0.008, 0.994, -0.005),
    glm::dvec3(0.007, -0.005, 1.009)
  );
  const std::array<glm::dvec3, 4> bends{
    glm::dvec3(1.0e-3, -0.8e-3, 0.6e-3),
    glm::dvec3(-0.7e-3, 1.2e-3, -0.5e-3),
    glm::dvec3(0.9e-3, 0.4e-3, -1.1e-3),
    glm::dvec3(-0.6e-3, 0.8e-3, 1.0e-3)
  };
  const glm::dmat3 cofactor = glm::determinant(stretch) * glm::transpose(glm::inverse(stretch));
  double restWeight = 0.0;
  double currentWeight = 0.0;
  std::vector<glm::dvec3> positions(8u);
  std::vector<glm::dvec3> velocities(8u, glm::dvec3(0.0));

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 gradient = talus::dynamics::cornerSigns(corner) / (8.0 * HALF);
    glm::dvec3 local = stretch * cell.rest[corner];

    for (uint32_t mode = 0u; mode != 4u; ++mode) {
      local += expectedPattern(mode, corner) * bends[mode];
    }

    positions[corner] = turn * local;
    restWeight += cell.inverseMass[corner] * glm::dot(gradient, gradient);
    currentWeight += cell.inverseMass[corner] * glm::dot(cofactor * gradient, cofactor * gradient);
  }

  const double volumeScale = restWeight / currentWeight;
  const auto energy = [&](const std::vector<glm::dvec3>& moved) {
    glm::dmat3 deformation(0.0);
    std::array<glm::dvec3, 4> patterns{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      deformation += glm::outerProduct(moved[corner], talus::dynamics::cornerSigns(corner) / (8.0 * HALF));

      for (uint32_t mode = 0u; mode != 4u; ++mode) {
        patterns[mode] += expectedPattern(mode, corner) * moved[corner];
      }
    }

    const glm::dmat3 local = back * deformation;
    const double change = glm::determinant(deformation) - 1.0;
    std::array<double, talus::dynamics::ELEMENT_ROWS> rows{};

    rows[0] = local[0][0] - 1.0;
    rows[1] = local[1][1] - 1.0;
    rows[2] = local[2][2] - 1.0;
    rows[3] = 0.5 * (local[1][0] + local[0][1]);
    rows[4] = 0.5 * (local[2][1] + local[1][2]);
    rows[5] = 0.5 * (local[0][2] + local[2][0]);

    for (uint32_t mode = 0u; mode != 4u; ++mode) {
      const glm::dvec3 offset = BEND * (back * patterns[mode]);

      rows[6u + 3u * mode] = offset.x;
      rows[7u + 3u * mode] = offset.y;
      rows[8u + 3u * mode] = offset.z;
    }

    double total = 0.5 * expectedVolumeStiffness(YOUNGS, POISSON, VOLUME) * volumeScale * change * change;

    for (uint32_t row = 0u; row != talus::dynamics::ELEMENT_ROWS; ++row) {
      total += 0.5 * expectedStiffness(row, YOUNGS, POISSON, VOLUME) * rows[row] * rows[row];
    }

    return total;
  };

  element.rotation = turn;
  talus::dynamics::updateElement(element, positions);
  talus::dynamics::solveElement(element, velocities);

  double scale = 0.0;
  std::array<glm::dvec3, 8> expected{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    glm::dvec3 gradient(0.0);

    for (int axis = 0; axis != 3; ++axis) {
      std::vector<glm::dvec3> forward = positions;
      std::vector<glm::dvec3> backward = positions;

      forward[corner][axis] += DELTA;
      backward[corner][axis] -= DELTA;
      gradient[axis] = (energy(forward) - energy(backward)) / (2.0 * DELTA);
    }

    expected[corner] = -SUBSTEP * cell.inverseMass[corner] * gradient;
    scale = std::max(scale, glm::length(expected[corner]));
  }

  REQUIRE(scale > 1.0e-6);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    CHECK(glm::length(velocities[corner] - expected[corner]) <= 2.0e-3 * scale);
  }
}

TEST_CASE("finds a cantilever's lowest bending mode as beam theory says", "[strainstep]") {
  constexpr double LENGTH = 4.0;
  constexpr double THICKNESS = 0.4;
  constexpr double YOUNGS = 1.0e5;
  constexpr double DENSITY = 5.0;
  const std::array<talus::deformable::SupportVolume, 1> clamp{
    holdAt(glm::dvec3(-LENGTH / 2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  talus::PhysicalMaterial material = jelly(YOUNGS, DENSITY);

  material.poissonsRatio = 0.0f;

  const SoftShape shape =
    softBox(glm::dvec3(LENGTH / 2.0, THICKNESS / 2.0, THICKNESS / 2.0), 0.2, material, clamp);
  const talus::PreparedMesh& mesh = *shape.mesh;
  std::vector<double> mass(mesh.nodeMass.begin(), mesh.nodeMass.end());
  std::vector<glm::dvec3> rest(mesh.restPositions.begin(), mesh.restPositions.end());
  const std::vector<talus::dynamics::StrainElement> elements = meshElements(mesh, material);
  const double omega = talus::dynamics::lowestAngular(elements, mass, mesh.anchored, rest);
  const double inertia = std::pow(THICKNESS, 4.0) / 12.0;
  const double area = THICKNESS * THICKNESS;
  const double theory =
    1.8751 * 1.8751 * std::sqrt(YOUNGS * inertia / (DENSITY * area * std::pow(LENGTH, 4.0)));

  CHECK(elements.size() == 80u);
  CHECK(std::abs(omega - theory) / theory < 0.1);
}

TEST_CASE("finds a free beam's lowest bending mode as beam theory says", "[strainstep]") {
  constexpr double LENGTH = 4.0;
  constexpr double THICKNESS = 0.4;
  constexpr double YOUNGS = 1.0e5;
  constexpr double DENSITY = 5.0;
  talus::PhysicalMaterial material = jelly(YOUNGS, DENSITY);

  material.poissonsRatio = 0.0f;

  const SoftShape shape =
    softBox(glm::dvec3(LENGTH / 2.0, THICKNESS / 2.0, THICKNESS / 2.0), 0.2, material);
  const talus::PreparedMesh& mesh = *shape.mesh;
  const std::vector<talus::dynamics::StrainElement> elements = meshElements(mesh, material);
  const std::vector<double> mass(mesh.nodeMass.begin(), mesh.nodeMass.end());
  const std::vector<glm::dvec3> rest(mesh.restPositions.begin(), mesh.restPositions.end());

  REQUIRE(std::all_of(
    mesh.anchored.begin(), mesh.anchored.end(), [](uint8_t held) { return held == 0u; }
  ));

  const double omega = talus::dynamics::lowestAngular(elements, mass, mesh.anchored, rest);
  const double inertia = std::pow(THICKNESS, 4.0) / 12.0;
  const double area = THICKNESS * THICKNESS;
  const double theory =
    4.7300 * 4.7300 * std::sqrt(YOUNGS * inertia / (DENSITY * area * std::pow(LENGTH, 4.0)));

  CHECK(elements.size() == 80u);
  CHECK(std::abs(omega - theory) / theory < 0.1);
}

TEST_CASE("finds a free cell's lowest mode as its hourglass mode", "[strainstep]") {
  constexpr double SIDE = 0.2;
  constexpr double YOUNGS = 1.0e5;
  constexpr double DENSITY = 5.0;
  constexpr double CORNER_MASS = DENSITY * SIDE * SIDE * SIDE / 8.0;
  TestCell cell = testCell(SIDE / 2.0, CORNER_MASS);

  cell.inverseMass.fill(1.0 / CORNER_MASS);

  const std::vector<talus::dynamics::StrainElement> elements{
    testElement(cell, YOUNGS, 0.0, 1.0 / 240.0)
  };
  const std::vector<double> mass(8u, CORNER_MASS);
  const std::vector<glm::dvec3> rest(cell.rest.begin(), cell.rest.end());
  const std::vector<uint8_t> pinned(8u, 0u);
  const double omega = talus::dynamics::lowestAngular(elements, mass, pinned, rest);
  const double expected = 2.0 / SIDE * std::sqrt(YOUNGS / (3.0 * DENSITY));

  CHECK(std::abs(omega - expected) <= 1.0e-6 * expected);
}

TEST_CASE("finds a cantilever's lowest bending mode as beam theory says with the volume row on", "[strainstep]") {
  constexpr double LENGTH = 4.0;
  constexpr double THICKNESS = 0.4;
  constexpr double YOUNGS = 1.0e5;
  constexpr double DENSITY = 5.0;
  const std::array<talus::deformable::SupportVolume, 1> clamp{
    holdAt(glm::dvec3(-LENGTH / 2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  talus::PhysicalMaterial material = jelly(YOUNGS, DENSITY);

  material.poissonsRatio = 0.4f;

  const SoftShape shape =
    softBox(glm::dvec3(LENGTH / 2.0, THICKNESS / 2.0, THICKNESS / 2.0), 0.2, material, clamp);
  const talus::PreparedMesh& mesh = *shape.mesh;
  const std::vector<talus::dynamics::StrainElement> elements = meshElements(mesh, material);
  const std::vector<double> mass(mesh.nodeMass.begin(), mesh.nodeMass.end());
  const std::vector<glm::dvec3> rest(mesh.restPositions.begin(), mesh.restPositions.end());

  REQUIRE(std::all_of(
    elements.begin(),
    elements.end(),
    [](const talus::dynamics::StrainElement& element) { return element.volumetric; }
  ));

  const double omega = talus::dynamics::lowestAngular(elements, mass, mesh.anchored, rest);
  const double inertia = std::pow(THICKNESS, 4.0) / 12.0;
  const double area = THICKNESS * THICKNESS;
  const double theory =
    1.8751 * 1.8751 * std::sqrt(YOUNGS * inertia / (DENSITY * area * std::pow(LENGTH, 4.0)));

  CHECK(elements.size() == 80u);
  CHECK(std::abs(omega - theory) / theory < 0.1);
}

TEST_CASE("returns infinity when every node is pinned", "[strainstep]") {
  const TestCell cell = testCell(0.1, 1.0);
  const std::vector<talus::dynamics::StrainElement> elements{
    testElement(cell, 1.0e5, 0.3, 1.0 / 240.0)
  };
  const std::vector<double> mass(8u, 1.0);
  const std::vector<glm::dvec3> rest(cell.rest.begin(), cell.rest.end());
  const std::vector<uint8_t> pinned(8u, 1u);

  CHECK(
    talus::dynamics::lowestAngular(elements, mass, pinned, rest)
    == std::numeric_limits<double>::infinity()
  );
}

TEST_CASE("returns zero when a free node has no element to hold it", "[strainstep]") {
  const std::vector<double> mass{1.0, 1.0};
  const std::vector<uint8_t> pinned{1u, 0u};
  const std::vector<glm::dvec3> rest{glm::dvec3(0.0), glm::dvec3(1.0, 0.0, 0.0)};

  CHECK(talus::dynamics::lowestAngular({}, mass, pinned, rest) == 0.0);
}

TEST_CASE("keeps an undisturbed jelly at rest", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3));
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(fastestNode(world.bodies.lattices[index]) < 1.0e-12);
}

TEST_CASE("keeps a jelly turned upside down at rest", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3, 0.2, 0.25), 0.1, jelly(8.0e3));
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& body = world.bodies.lattices[index];
  const glm::dquat turn = glm::angleAxis(3.14159265358979323846, glm::dvec3(1.0, 0.0, 0.0));

  for (glm::dvec3& position : body.position) {
    position = turn * position;
  }

  body.initial = body.position;
  body.inertial = body.position;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(fastestNode(world.bodies.lattices[index]) < 1.0e-9);
}

TEST_CASE("keeps a jelly turned a quarter turn about a skew axis at rest", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3, 0.2, 0.25), 0.1, jelly(8.0e3));
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& body = world.bodies.lattices[index];
  const glm::dquat turn =
    glm::angleAxis(std::numbers::pi / 2.0, glm::normalize(glm::dvec3(1.0, 2.0, 3.0)));

  for (glm::dvec3& position : body.position) {
    position = turn * position + glm::dvec3(1.0, 2.0, 3.0);
  }

  body.initial = body.position;
  body.inertial = body.position;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(fastestNode(world.bodies.lattices[index]) < 1.0e-9);
}

TEST_CASE("never moves pinned nodes", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 1> clamp{
    holdAt(glm::dvec3(-0.6, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(0.6, 0.15, 0.15), 0.15, jelly(2.0e4), clamp);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 3.0, 0.0));
  const std::vector<glm::dvec3> start = world.bodies.lattices[index].position;
  uint32_t held = 0u;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  for (size_t node = 0u; node != start.size(); ++node) {
    if (shape.mesh->anchored[node] != 0u) {
      ++held;
      CHECK(world.bodies.lattices[index].position[node] == start[node]);
    }
  }

  CHECK(held > 0u);
}

TEST_CASE("keeps a lattice with every node pinned where it is", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 1> all{holdAt(glm::dvec3(0.0), glm::dvec3(1.0))};
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3), all);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  const std::vector<glm::dvec3> start = world.bodies.lattices[index].position;

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(world.bodies.lattices[index].position == start);
}

TEST_CASE("holds a node with no mass still, as the sandbox does", "[strainstep]") {
  SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3));
  const uint32_t corner = nearestNode(*shape.mesh, glm::dvec3(0.3));
  const std::shared_ptr<talus::PreparedMesh> massless =
    std::make_shared<talus::PreparedMesh>(*shape.mesh);

  REQUIRE(shape.mesh->anchored[corner] == 0u);

  massless->nodeMass[corner] = 0.0f;
  shape.mesh = massless;

  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  const glm::dvec3 start = world.bodies.lattices[index].position[corner];

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(world.bodies.lattices[index].position[corner] == start);
}

TEST_CASE("bends a cantilever one cell thick as beam theory says for compressible and nearly incompressible materials", "[strainstep][slow]") {
  for (const double poisson : {0.0, 0.45}) {
    const Cantilever bent = cantilever(0.4, 0.4, beam(5.0e5, poisson), 48u, 600u);

    CHECK(bent.cells == 10u);
    CHECK(std::abs(bent.sag - bent.theory) / bent.theory < 0.1);
  }
}

TEST_CASE("bends a cantilever two cells thick as beam theory says", "[strainstep][slow]") {
  const Cantilever bent = cantilever(0.4, 0.2, beam(5.0e5, 0.3), 16u, 600u);

  CHECK(bent.cells == 80u);
  CHECK(std::abs(bent.sag - bent.theory) / bent.theory < 0.1);
}

TEST_CASE("keeps the volume of a nearly incompressible block squeezed under its own weight", "[strainstep]") {
  talus::PhysicalMaterial rubber = jelly(1.0e5);

  rubber.poissonsRatio = 0.45f;

  const std::array<talus::deformable::SupportVolume, 1> base{
    holdAt(glm::dvec3(0.0, -0.3, 0.0), glm::dvec3(1.0, 0.01, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, rubber, base);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  const double before = latticeVolume(world.bodies.lattices[index]);
  const uint32_t top = nearestNode(*shape.mesh, glm::dvec3(0.0, 0.3, 0.0));
  double drop = 0.0;

  for (uint32_t frame = 1u; frame <= 300u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (frame > 240u) {
      drop += (0.3 - world.bodies.lattices[index].position[top].y) / 60.0;
    }
  }

  CHECK(std::abs(latticeVolume(world.bodies.lattices[index]) - before) / before < 0.01);
  CHECK(drop > 0.005);
}

TEST_CASE("bends a cantilever as beam theory says when its material sets a damping ratio", "[strainstep][slow]") {
  talus::PhysicalMaterial damped = beam(5.0e5, 0.3);

  damped.dampingRatio = 0.2f;

  const Cantilever bent = cantilever(0.4, 0.2, damped, 16u, 600u);

  CHECK(std::abs(bent.sag - bent.theory) / bent.theory < 0.1);
}

TEST_CASE("keeps a damped cantilever as beam theory says after its substeps change", "[strainstep][slow]") {
  talus::PhysicalMaterial damped = beam(5.0e5, 0.3);

  damped.dampingRatio = 0.2f;

  const Cantilever bent = cantilever(0.4, 0.2, damped, 16u, 600u, SubstepChange{100u, 32u});

  CHECK(bent.substeps == 32u);
  CHECK(std::abs(bent.sag - bent.theory) / bent.theory < 0.1);
}

TEST_CASE("keeps the momentum of two jellies that hit head-on", "[strainstep]") {
  const SoftShape firm = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3, 10.0));
  const SoftShape gentle = softBox(glm::dvec3(0.3), 0.15, jelly(2.0e3, 10.0));
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t left = addSoft(world, firm, glm::dvec3(-0.35, 0.0, 0.0), glm::dvec3(5.0, 0.0, 0.0));
  const uint32_t right = addSoft(world, gentle, glm::dvec3(0.35, 0.0, 0.0), glm::dvec3(-5.0, 0.0, 0.0));
  const double mass = latticeMass(world.bodies.lattices[left]) + latticeMass(world.bodies.lattices[right]);
  double drift = 0.0;
  double closest = std::numeric_limits<double>::infinity();

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::LatticeBody& a = world.bodies.lattices[left];
    const talus::dynamics::LatticeBody& b = world.bodies.lattices[right];

    drift = std::max(drift, glm::length(latticeMomentum(a) + latticeMomentum(b)) / mass);
    closest = std::min(closest, latticeCentre(b).x - latticeCentre(a).x);
  }

  CHECK(drift < 1.0e-9);
  CHECK(closest > 0.3);
  CHECK(latticeCentre(world.bodies.lattices[left]).x < -0.35);
  CHECK(latticeCentre(world.bodies.lattices[right]).x > 0.35);
}

TEST_CASE("brings a jelly dropped on the ground to rest", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3, 100.0));
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.6, 0.0));
  glm::dvec3 settled(0.0);
  double fastest = 0.0;
  double drift = 0.0;
  double lowest = std::numeric_limits<double>::infinity();

  for (uint32_t frame = 1u; frame <= 480u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];

    for (const glm::dvec3& position : body.position) {
      lowest = std::min(lowest, position.y);
    }

    if (frame == 180u) {
      settled = latticeCentre(body);
    }

    if (frame > 180u) {
      fastest = std::max(fastest, fastestNode(body));
      drift = std::max(drift, glm::length(latticeCentre(body) - settled));
    }
  }

  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
  CHECK(lowest > -0.02);
}

TEST_CASE("carries a resting jelly's weight in its contacts", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3, 100.0));
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.31, 0.0));

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const double load = talus::dynamics::normalForceOn(
    world,
    talus::dynamics::BodyRef{talus::dynamics::Side::Lattice, index}
  );
  const double weight = latticeMass(world.bodies.lattices[index]) * 9.81;

  CHECK(std::abs(load - weight) <= 0.1 * weight);
}

TEST_CASE("stops a soft jelly sliding on the ground where Coulomb friction says", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial block = jelly(2.0e4, 200.0);

  block.friction = 0.5f;

  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, block);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(-2.0, 0.305, 0.0), glm::dvec3(3.0, 0.0, 0.0));
  const double start = latticeCentre(world.bodies.lattices[index]).x;

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const double coulomb = 3.0 * 3.0 / (2.0 * 0.5 * 9.81);

  CHECK(std::abs(latticeCentre(world.bodies.lattices[index]).x - start - coulomb) / coulomb < 0.1);
}

TEST_CASE("keeps both bottom corners of a box lying on a soft plank in contact", "[strainstep][slow]") {
  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  talus::PhysicalMaterial wood = jelly(2.0e7, 500.0);

  wood.friction = 0.6f;
  wood.poissonsRatio = 0.3f;

  const SoftShape plank = softBox(glm::dvec3(2.0, 0.1, 0.4), 0.1, wood, ends);
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.25), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, plank, glm::dvec3(0.0));

  world.bodies.lattices[index].substeps = 32u;

  const uint32_t crate = talus::dynamics::addRigidBox(
    world.bodies,
    crateField,
    glm::dvec3(0.25),
    50.0,
    glm::dvec3(1.0, 0.36, 0.0),
    glm::dvec3(0.0),
    0.6,
    5u
  );

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  bool left = false;
  bool right = false;

  for (const talus::dynamics::Contact& contact : world.contacts) {
    for (const talus::dynamics::ContactSide* side : {&contact.a, &contact.b}) {
      if (side->side == talus::dynamics::Side::Rigid && side->body == crate
          && contact.memory.normal > 0.0 && side->local.y < -0.2) {
        left = left || side->local.x < -0.2;
        right = right || side->local.x > 0.2;
      }
    }
  }

  CHECK(left);
  CHECK(right);
  CHECK(world.bodies.rigids[crate].position.y > 0.3);
}

TEST_CASE("rests a small box dropped off-centre on a soft cube (Rapier box_rests_on_surface_and_nothing_creeps)", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial foam = jelly(5.0e3, 20.0);

  foam.poissonsRatio = 0.35f;
  foam.friction = 0.5f;

  const SoftShape cube = softBox(glm::dvec3(0.6), 0.24, foam);
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.2), 0.04);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, cube, glm::dvec3(0.0, 0.6, 0.0));
  const uint32_t crate = talus::dynamics::addRigidBox(
    world.bodies,
    crateField,
    glm::dvec3(0.2),
    0.8,
    glm::dvec3(0.1, 2.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    5u
  );

  for (uint32_t frame = 0u; frame != 600u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::RigidBody& box = world.bodies.rigids[crate];
  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];

  CHECK(box.position.y > 1.2);
  CHECK(box.position.y < 1.7);
  CHECK(std::abs(box.position.x - 0.1) < 0.05);
  CHECK(std::abs(latticeCentre(body).x) < 0.02);
  CHECK(fastestNode(body) < 0.01);
}

TEST_CASE("bounces a jelly alike at four and eight cells across when its material sets a damping ratio", "[strainstep][slow]") {
  const double four = rebound(0.15, 16u);
  const double eight = rebound(0.075, 16u);

  CHECK(four > 0.1);
  CHECK(std::abs(eight - four) / four < 0.1);
}

TEST_CASE("brings a jelly squashed by a fast heavy box back to its shape", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(2.0e5, 1000.0));
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.25), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));
  const uint32_t crate = talus::dynamics::addRigidBox(
    world.bodies,
    crateField,
    glm::dvec3(0.25),
    50.0,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0, -10.0, 0.0),
    0.5,
    5u
  );

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(lowestCellRatio(world.bodies.lattices[index]) > 0.0);
  CHECK(world.bodies.rigids[crate].position.y > 0.6);
}

TEST_CASE("keeps a heavy crate dropped on a soft jelly from being launched", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(2.0e5, 1000.0));
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.25), 0.05);
  const std::array<glm::dvec2, 3> drops{glm::dvec2(0.86, 10.0), glm::dvec2(0.86, 20.0), glm::dvec2(1.0, 10.0)};

  for (const glm::dvec2& drop : drops) {
    DYNAMIC_SECTION("dropped from " << drop.x << " m at " << drop.y << " m/s") {
      World world;

      useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
      addFloor(world, floor, 0.5);

      const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));
      const uint32_t crate = talus::dynamics::addRigidBox(
        world.bodies,
        crateField,
        glm::dvec3(0.25),
        5000.0,
        glm::dvec3(0.0, drop.x, 0.0),
        glm::dvec3(0.0, -drop.y, 0.0),
        0.5,
        5u
      );
      const auto energy = [&] {
        const talus::dynamics::RigidBody& body = world.bodies.rigids[crate];
        const glm::dvec3 spin = glm::conjugate(body.rotation) * body.angularVelocity;

        return 0.5 * body.mass * glm::dot(body.velocity, body.velocity) + 0.5 * glm::dot(spin, body.moment * spin)
               + body.mass * 9.81 * body.position.y + latticeEnergy(world.bodies.lattices[index]);
      };
      const double start = energy();
      double highest = drop.x;
      double peak = start;

      for (uint32_t frame = 0u; frame != 300u; ++frame) {
        REQUIRE(!talus::dynamics::step(world).nonFinite);
        highest = std::max(highest, world.bodies.rigids[crate].position.y);
        peak = std::max(peak, energy());
      }

      CHECK(highest <= drop.x);
      CHECK(peak <= start * 1.02);
    }
  }
}

TEST_CASE("treats a contact point held mostly by pinned nodes as immovable without kicking its free node or taking its motion", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(2.0e5, 1000.0));
  const talus::field::SampledSdf ball = ballField(0.2, 0.04);
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  const uint32_t heavy = talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.2,
    5000.0,
    glm::dvec3(0.0, 0.5, 0.0),
    glm::dvec3(0.0, -1.0, 0.0),
    0.5,
    0.0
  );
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];
  const std::array<uint32_t, 8> nodes = lattice.mesh->hexes.front().nodes;
  const uint32_t free = nodes[7];

  setAnchored(lattice, std::span<const uint32_t>(nodes.data(), 7u), 1u);

  talus::dynamics::Contact contact;

  contact.a.side = talus::dynamics::Side::Lattice;
  contact.a.body = index;
  contact.a.nodes = nodes;
  contact.a.weights.fill(0.95 / 7.0);
  contact.a.weights[7] = 0.05;
  contact.b.side = talus::dynamics::Side::Rigid;
  contact.b.body = heavy;
  contact.b.local = glm::dvec3(0.0, -0.2, 0.0);
  contact.basis = {glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0)};
  contact.friction = 0.0;

  const double h = 1.0 / 240.0;
  std::vector<talus::dynamics::RigidState> states{
    talus::dynamics::RigidState{},
    talus::dynamics::gatherRigid(world.bodies.rigids[heavy], heavy)
  };
  std::vector<uint32_t> stateOf(world.bodies.rigids.size(), 0u);
  const std::vector<uint8_t> stiff(world.bodies.lattices.size(), 0u);
  const talus::dynamics::RowBodies bodies{world.bodies.lattices, states};

  stateOf[heavy] = 1u;

  const auto solve = [&] {
    std::optional<talus::dynamics::ContactRow> row =
      talus::dynamics::makeContactRow(contact, 0u, bodies, stateOf, stiff, talus::dynamics::RowSettings{h, 30.0, 10.0});

    REQUIRE(row.has_value());
    talus::dynamics::refreshContactRow(*row, bodies, 1.0 / h, 3.0, false);
    talus::dynamics::warmStartContactRow(*row, bodies);

    for (uint32_t pass = 0u; pass != 4u; ++pass) {
      talus::dynamics::solveContactRow(*row, bodies, true);
    }
  };

  solve();

  CHECK(glm::length(lattice.velocity[free]) <= 1.0);
  CHECK(states[1].v.y > -0.1);

  states[1].v = glm::dvec3(0.0);
  lattice.velocity[free] = glm::dvec3(0.0, 5.0, 0.0);
  solve();

  CHECK(glm::length(states[1].v) < 1.0e-12);
  CHECK(lattice.velocity[free] == glm::dvec3(0.0, 5.0, 0.0));
}

TEST_CASE("holds a heavy ball dragged into a soft bridge", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-1.5, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(1.5, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  talus::PhysicalMaterial rubber = jelly(5.0e6, 1000.0);

  rubber.poissonsRatio = 0.3f;

  const SoftShape bridge = softBox(glm::dvec3(1.5, 0.2, 0.5), 0.2, rubber, ends);
  const talus::field::SampledSdf ball = ballField(0.2, 0.04);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, bridge, glm::dvec3(0.0));
  const uint32_t heavy = talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.2,
    100.0,
    glm::dvec3(0.0, 0.41, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  double deepest = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    talus::dynamics::RigidBody& body = world.bodies.rigids[heavy];
    const double k = 20.0 * body.mass;
    const double damping = 2.0 * std::sqrt(k * body.mass);

    body.velocity.y +=
      (k * (-1.0 - body.position.y) - damping * body.velocity.y) / body.mass * world.parameters.dt;

    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::LatticeBody& plank = world.bodies.lattices[index];

    for (size_t node = 0u; node != plank.position.size(); ++node) {
      const glm::vec3& rest = bridge.mesh->restPositions[node];

      if (std::abs(rest.x) < 0.25f && std::abs(rest.z) < 0.25f && rest.y > 0.19f) {
        deepest = std::max(
          deepest,
          0.2 - glm::length(plank.position[node] - world.bodies.rigids[heavy].position)
        );
      }
    }
  }

  CHECK(deepest < 0.02);
}

TEST_CASE("brings a plank lying across two jellies on the ground to rest", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(2.0e4, 300.0));
  const talus::field::SampledSdf plankField = boxField(glm::dvec3(1.0, 0.05, 0.3), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, shape, glm::dvec3(-0.6, 0.3, 0.0));
  (void)addSoft(world, shape, glm::dvec3(0.6, 0.3, 0.0));

  const uint32_t plank = talus::dynamics::addRigidBox(
    world.bodies,
    plankField,
    glm::dvec3(1.0, 0.05, 0.3),
    10.0,
    glm::dvec3(0.0, 0.66, 0.0),
    glm::dvec3(0.0),
    0.5,
    5u
  );
  glm::dvec3 settled(0.0);
  double fastest = 0.0;
  double drift = 0.0;
  uint32_t islands = 0u;

  for (uint32_t frame = 1u; frame <= 360u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    islands = report.strainstepIslands;

    const talus::dynamics::RigidBody& body = world.bodies.rigids[plank];

    if (frame == 240u) {
      settled = body.position;
    }

    if (frame > 240u) {
      fastest = std::max(fastest, glm::length(body.velocity) + glm::length(body.angularVelocity));
      drift = std::max(drift, glm::length(body.position - settled));
    }
  }

  CHECK(islands == 1u);
  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
  CHECK(world.bodies.rigids[plank].position.y > 0.5);
}

TEST_CASE("stops a soft jelly sliding diagonally on the ground where Coulomb friction says", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial block = jelly(2.0e4, 200.0);

  block.friction = 0.5f;

  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, block);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(-1.5, 0.305, -1.5), glm::dvec3(2.1, 0.0, 2.1));
  const glm::dvec3 start = latticeCentre(world.bodies.lattices[index]);

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const glm::dvec3 moved = latticeCentre(world.bodies.lattices[index]) - start;
  const double speed = 2.1 * std::sqrt(2.0);
  const double coulomb = speed * speed / (2.0 * 0.5 * 9.81);

  CHECK(std::abs(glm::length(moved) - coulomb) / coulomb < 0.1);
  CHECK(std::abs(moved.x - moved.z) < 0.02 * coulomb);
}

TEST_CASE(
  "keeps a ball one cell wide from sinking into a soft face between its nodes",
  "[strainstep][!mayfail]"
) {
  constexpr double RADIUS = 0.2;
  constexpr double TOP = 0.4;
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial rubber = jelly(5.0e6, 1000.0);

  rubber.poissonsRatio = 0.3f;

  const SoftShape slab = softBox(glm::dvec3(0.6, TOP / 2.0, 0.6), 0.2, rubber);
  const talus::field::SampledSdf ball = ballField(RADIUS, 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, slab, glm::dvec3(0.0, TOP / 2.0, 0.0));

  const uint32_t sphere = talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    RADIUS,
    10.0,
    glm::dvec3(0.1, TOP + 0.5, 0.1),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(std::abs(world.bodies.rigids[sphere].position.y - (TOP + RADIUS)) < 0.02);
}

TEST_CASE("pushes a jelly started inside the floor out no faster than the contact speed", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3, 100.0));
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.15, 0.0));
  const double limit = world.parameters.contactSpeed * world.parameters.dt * (1.0 + 1.0e-6);
  const auto lowestNode = [&world, index]() {
    double lowest = std::numeric_limits<double>::infinity();

    for (const glm::dvec3& position : world.bodies.lattices[index].position) {
      lowest = std::min(lowest, position.y);
    }

    return lowest;
  };
  double previous = lowestNode();
  double fastestRise = -std::numeric_limits<double>::infinity();

  REQUIRE(std::abs(previous + 0.15) < 1.0e-6);

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const double now = lowestNode();

    fastestRise = std::max(fastestRise, now - previous);
    previous = now;
  }

  CHECK(fastestRise <= limit);
  CHECK(fastestNode(world.bodies.lattices[index]) < 0.05);
  CHECK(lowestNode() > -0.02);
}

TEST_CASE("stores a sliding jelly's friction on its friction circle, against its slide", "[strainstep]") {
  constexpr double MU = 0.5;
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial block = jelly(2.0e4, 200.0);

  block.friction = static_cast<float>(MU);

  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, block);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, MU);

  const uint32_t index = addSoft(world, shape, glm::dvec3(-2.0, 0.305, 0.0), glm::dvec3(3.0, 0.0, 0.0));
  const glm::dvec3 slide(1.0, 0.0, 0.0);
  const double mass = latticeMass(world.bodies.lattices[index]);
  const uint32_t frames = static_cast<uint32_t>(3.0 / (MU * 9.81) / world.parameters.dt);
  uint32_t checked = 0u;

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (glm::dot(latticeMomentum(world.bodies.lattices[index]), slide) / mass <= 0.1) {
      continue;
    }

    glm::dvec3 force(0.0);
    double normal = 0.0;

    for (const talus::dynamics::Contact& contact : world.contacts) {
      if (contact.a.side == talus::dynamics::Side::Lattice && contact.a.body == index) {
        force += contact.memory.friction.x * contact.basis[1] + contact.memory.friction.y * contact.basis[2];
        normal += contact.memory.normal;
      }
    }

    if (!(normal > 0.0)) {
      continue;
    }

    ++checked;
    CHECK(glm::dot(force, slide) < 0.0);
    CHECK(std::abs(glm::length(force) / normal - MU) <= 0.1 * MU);
  }

  REQUIRE(checked >= 10u);
}

TEST_CASE("keeps a lattice with every node pinned on the floor still and finite", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const std::array<talus::deformable::SupportVolume, 1> all{holdAt(glm::dvec3(0.0), glm::dvec3(1.0))};
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(8.0e3), all);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));
  const std::vector<glm::dvec3> start = world.bodies.lattices[index].position;

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  uint32_t touching = 0u;

  for (const talus::dynamics::Contact& contact : world.contacts) {
    if (contact.a.side == talus::dynamics::Side::Lattice && contact.a.body == index) {
      ++touching;
      CHECK(contact.memory.normal == 0.0);
    }
  }

  CHECK(touching > 0u);
  CHECK(world.bodies.lattices[index].position == start);
}

TEST_CASE("stacks a box on a box resting on a jelly", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf lowerField = boxField(glm::dvec3(0.2), 0.04);
  const talus::field::SampledSdf upperField = boxField(glm::dvec3(0.15), 0.03);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));

  const uint32_t lower = talus::dynamics::addRigidBox(
    world.bodies, lowerField, glm::dvec3(0.2), 20.0, glm::dvec3(0.0, 0.81, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  const uint32_t upper = talus::dynamics::addRigidBox(
    world.bodies, upperField, glm::dvec3(0.15), 10.0, glm::dvec3(0.02, 1.17, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  glm::dvec3 settled(0.0);
  double fastest = 0.0;
  double drift = 0.0;
  talus::dynamics::StepReport report;

  for (uint32_t frame = 1u; frame <= 360u; ++frame) {
    report = talus::dynamics::step(world);
    REQUIRE(!report.nonFinite);

    const talus::dynamics::RigidBody& top = world.bodies.rigids[upper];

    if (frame == 240u) {
      settled = top.position;
    }

    if (frame > 240u) {
      fastest = std::max(fastest, glm::length(top.velocity) + 0.3 * glm::length(top.angularVelocity));
      drift = std::max(drift, glm::length(top.position - settled));
    }
  }

  const double stacked = world.bodies.rigids[upper].position.y - world.bodies.rigids[lower].position.y;

  CHECK(report.strainstepIslands == 1u);
  CHECK(report.softstepIslands == 0u);
  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
  CHECK(std::abs(stacked - 0.35) < 0.02);
  CHECK(std::abs(cachedLoad(world, lower, upper) - 10.0 * 9.81) <= 0.1 * 10.0 * 9.81);
}

TEST_CASE("brings a plank leaning from the ground onto a jelly to rest", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf plankField = boxField(glm::dvec3(1.0, 0.05, 0.3), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, shape, glm::dvec3(0.8, 0.3, 0.0));

  const uint32_t plank = talus::dynamics::addRigidBox(
    world.bodies, plankField, glm::dvec3(1.0, 0.05, 0.3), 10.0, glm::dvec3(0.4, 0.66, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  glm::dvec3 settled(0.0);
  double fastest = 0.0;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= 480u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& body = world.bodies.rigids[plank];

    if (frame == 360u) {
      settled = body.position;
    }

    if (frame > 360u) {
      fastest = std::max(fastest, glm::length(body.velocity) + glm::length(body.angularVelocity));
      drift = std::max(drift, glm::length(body.position - settled));
    }
  }

  CHECK(restingOn(world, plank));
  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
}

TEST_CASE("bounces a ball off a box resting on a jelly with the pair's restitution", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf boxShape = boxField(glm::dvec3(0.3), 0.05);
  const talus::field::SampledSdf ball = ballField(0.1, 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));
  (void)talus::dynamics::addRigidBox(
    world.bodies, boxShape, glm::dvec3(0.3), 200.0, glm::dvec3(0.0, 0.91, 0.0), glm::dvec3(0.0), 0.5, 5u
  );

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const uint32_t thrown = talus::dynamics::addRigidSphere(
    world.bodies, ball, 0.1, 0.1, glm::dvec3(0.0, 2.3, 0.0), glm::dvec3(0.0), 0.5, 0.9
  );
  const double top = 2.3 - 1.0;
  double previous = world.bodies.rigids[thrown].velocity.y;
  double apex = -std::numeric_limits<double>::infinity();
  bool bounced = false;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& body = world.bodies.rigids[thrown];

    bounced = bounced || (previous < 0.0 && body.velocity.y > 0.0);

    if (bounced) {
      apex = std::max(apex, body.position.y);
    }

    previous = body.velocity.y;
  }

  CHECK(bounced);
  CHECK(apex - top > 0.5);
}

TEST_CASE("pushes a box started inside another box out", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf lowerField = boxField(glm::dvec3(0.2), 0.04);
  const talus::field::SampledSdf upperField = boxField(glm::dvec3(0.15), 0.03);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));

  const uint32_t lower = talus::dynamics::addRigidBox(
    world.bodies, lowerField, glm::dvec3(0.2), 20.0, glm::dvec3(0.0, 0.81, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  const uint32_t upper = talus::dynamics::addRigidBox(
    world.bodies, upperField, glm::dvec3(0.15), 10.0, glm::dvec3(0.0, 1.13, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  glm::dvec3 settled(0.0);
  double fastest = 0.0;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& top = world.bodies.rigids[upper];

    if (frame == 60u) {
      settled = top.position;
    }

    if (frame > 60u) {
      fastest = std::max(fastest, glm::length(top.velocity) + 0.3 * glm::length(top.angularVelocity));
      drift = std::max(drift, glm::length(top.position - settled));
    }
  }

  const double stacked = world.bodies.rigids[upper].position.y - world.bodies.rigids[lower].position.y;

  CHECK(std::abs(stacked - 0.35) < 0.02);
  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
  CHECK(std::abs(cachedLoad(world, lower, upper) - 10.0 * 9.81) <= 0.1 * 10.0 * 9.81);
}

TEST_CASE("keeps the momentum of a ball that hits a box off-centre in a jelly's island", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.1), 0.02);
  const talus::field::SampledSdf ballShape = ballField(0.1, 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t soft = addSoft(world, shape, glm::dvec3(0.0));
  const double crateY = 0.3 + 1.0e-4 + 0.015 + 0.1;
  const uint32_t crate = talus::dynamics::addRigidBox(
    world.bodies, crateField, glm::dvec3(0.1), 2.0, glm::dvec3(0.0, crateY, 0.0), glm::dvec3(0.0), 0.0, 5u
  );
  const uint32_t ball = talus::dynamics::addRigidSphere(
    world.bodies, ballShape, 0.1, 1.0, glm::dvec3(-0.5, crateY + 0.02, 0.0), glm::dvec3(2.0, 0.0, 0.0), 0.0, 0.0
  );
  const double mass = latticeMass(world.bodies.lattices[soft]) + world.bodies.rigids[crate].mass
                      + world.bodies.rigids[ball].mass;
  const auto linearMomentum = [&world, soft, crate, ball]() {
    glm::dvec3 total = latticeMomentum(world.bodies.lattices[soft]);

    for (const uint32_t index : {crate, ball}) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[index];

      total += body.mass * body.velocity;
    }

    return total;
  };
  const auto angularMomentum = [&world, soft, crate, ball]() {
    glm::dvec3 total = latticeAngularMomentum(world.bodies.lattices[soft]);

    for (const uint32_t index : {crate, ball}) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[index];

      total += body.mass * glm::cross(body.position, body.velocity)
               + talus::dynamics::worldInertia(body) * body.angularVelocity;
    }

    return total;
  };
  const glm::dvec3 linearStart = linearMomentum();
  const glm::dvec3 angularStart = angularMomentum();
  double linearDrift = 0.0;
  double angularDrift = 0.0;
  uint32_t touching = 0u;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    linearDrift = std::max(linearDrift, glm::length(linearMomentum() - linearStart) / mass);
    angularDrift = std::max(angularDrift, glm::length(angularMomentum() - angularStart) / mass);
    touching += cachedLoad(world, crate, ball) > 0.0 ? 1u : 0u;
  }

  CHECK(touching >= 1u);
  CHECK(glm::length(world.bodies.rigids[crate].angularVelocity) > 0.1);
  CHECK(linearDrift < 1.0e-9);
  CHECK(angularDrift < 1.0e-4);
}

TEST_CASE("stands ten boxes on a jelly", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.1), 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);
  (void)addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));

  std::vector<uint32_t> crates;

  for (uint32_t level = 0u; level != 10u; ++level) {
    crates.push_back(talus::dynamics::addRigidBox(
      world.bodies, crateField, glm::dvec3(0.1), 5.0, glm::dvec3(0.0, 0.705 + 0.2025 * level, 0.0), glm::dvec3(0.0), 0.5, 5u
    ));
  }

  std::vector<glm::dvec3> settled(crates.size(), glm::dvec3(0.0));
  double fastest = 0.0;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (size_t at = 0u; at != crates.size(); ++at) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[crates[at]];

      if (frame == 60u) {
        settled[at] = body.position;
      }

      if (frame > 60u) {
        fastest = std::max(fastest, glm::length(body.velocity));
        drift = std::max(drift, glm::length(body.position - settled[at]));
      }
    }
  }

  CHECK(drift < 0.01);
  CHECK(fastest < 0.05);
}

TEST_CASE("hands a column of boxes resting beside a jelly back to softstep without a jolt", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.15, jelly(5.0e4, 500.0));
  const talus::field::SampledSdf crateField = boxField(glm::dvec3(0.1), 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t soft = addSoft(world, shape, glm::dvec3(0.0, 0.3, 0.0));
  std::vector<uint32_t> crates;

  for (uint32_t level = 0u; level != 10u; ++level) {
    crates.push_back(talus::dynamics::addRigidBox(
      world.bodies, crateField, glm::dvec3(0.1), 5.0, glm::dvec3(0.4051, 0.1 + 0.2025 * level, 0.0), glm::dvec3(0.0), 0.5, 5u
    ));
  }

  talus::dynamics::StepReport report;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    report = talus::dynamics::step(world);
    REQUIRE(!report.nonFinite);
  }

  REQUIRE(report.strainstepIslands == 1u);
  REQUIRE(report.softstepIslands == 0u);

  std::vector<glm::dvec3> settled;

  for (const uint32_t crate : crates) {
    settled.push_back(world.bodies.rigids[crate].position);
  }

  const glm::dvec3 away(-50.0, 0.0, 0.0);
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[soft];

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    lattice.position[node] += away;
    lattice.initial[node] += away;
    lattice.inertial[node] += away;
  }

  double fastest = 0.0;
  double drift = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    report = talus::dynamics::step(world);
    REQUIRE(!report.nonFinite);
    REQUIRE(report.softstepIslands == 1u);

    for (size_t at = 0u; at != crates.size(); ++at) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[crates[at]];

      fastest = std::max(fastest, glm::length(body.velocity));
      drift = std::max(drift, glm::length(body.position - settled[at]));
    }
  }

  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
}

TEST_CASE("gives a stiff cantilever the substeps its cells need and bends it as beam theory says", "[strainstep][slow]") {
  const Cantilever bent = cantilever(0.4, 0.2, beam(5.0e5, 0.3), 0u, 600u);

  CHECK(bent.substeps == 16u);
  CHECK(std::abs(bent.sag - bent.theory) / bent.theory < 0.1);
}

TEST_CASE("drops a body too stiff for the world substeps on the ground without it gaining energy (the always-rules limit)", "[strainstep][slow]") {
  const Drop drop = dropStiff(5.0e6);

  CHECK(drop.substeps == 7u);
  CHECK(drop.peak <= 1.0 + 2.0e-2);
}

TEST_CASE("caps the substeps at 64 and keeps a body far too stiff for them from gaining energy on the ground", "[strainstep][slow]") {
  const Drop drop = dropStiff(1.0e9);

  CHECK(drop.substeps == 64u);
  CHECK(drop.peak <= 1.0 + 2.0e-2);
}

TEST_CASE("keeps the authored cells of a body soft enough for the world substeps", "[strainstep]") {
  uint32_t coarsening = 0u;
  const SoftShape shape = resolvedBox(glm::dvec3(0.3), 0.15, jelly(8.0e3), coarsening);

  CHECK(coarsening == 1u);
  CHECK(shape.mesh->hexes.size() == 64u);
}

TEST_CASE("coarsens a stiff beam as far as its shape allows and bends it as beam theory says", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 1> clamp{
    holdAt(glm::dvec3(-2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  uint32_t coarsening = 0u;
  const SoftShape shape = resolvedBox(glm::dvec3(2.0, 0.2, 0.2), 0.05, stiffBeam(2.0e8), coarsening, clamp);

  REQUIRE(coarsening == 8u);
  REQUIRE(shape.mesh->hexes.size() == 10u);

  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  const uint32_t tip = nearestNode(*shape.mesh, glm::dvec3(2.0, 0.2, 0.2));
  double sag = 0.0;

  for (uint32_t frame = 1u; frame <= 600u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (frame > 300u) {
      sag += (0.2 - world.bodies.lattices[index].position[tip].y) / 300.0;
    }
  }

  const double theory = 1.5 * 1000.0 * 9.81 * std::pow(4.0, 4.0) / (2.0e8 * 0.4 * 0.4);

  CHECK(std::abs(sag - theory) / theory < 0.1);
}

TEST_CASE("keeps a thin bar at the authored cells when coarser cells would lose its shape", "[strainstep]") {
  uint32_t coarsening = 0u;
  const SoftShape shape = resolvedBox(glm::dvec3(2.0, 0.05, 0.05), 0.1, stiffBeam(2.0e8), coarsening);

  CHECK(coarsening == 1u);
  CHECK(shape.mesh->hexes.size() == 40u);
}

TEST_CASE("keeps the cavity of a hollow body open when it coarsens", "[strainstep]") {
  SoftShape shape;
  talus::dynamics::IntegrationParameters parameters;
  const glm::vec3 reach(0.8f);

  shape.field = talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(0.6f + 1.0e-4f)),
      fixture::box(glm::vec3(0.0f), glm::vec3(0.3f - 1.0e-4f), talus::field::FIELD_OP_SUBTRACTION)
    },
    -reach,
    reach,
    glm::uvec3(17u)
  );
  shape.lanes.assign(2u, stiffBeam(1.0e8));

  const talus::dynamics::ResolvedMesh resolved =
    talus::dynamics::resolveMesh(shape.field, shape.lanes, glm::mat4(1.0f), {}, parameters);

  REQUIRE(resolved.coarsening == 3u);

  for (const talus::PreparedHex& hex : resolved.mesh->hexes) {
    glm::dvec3 centre(0.0);

    for (const uint32_t node : hex.nodes) {
      centre += glm::dvec3(resolved.mesh->restPositions[node]) / 8.0;
    }

    CHECK(std::max({std::abs(centre.x), std::abs(centre.y), std::abs(centre.z)}) > 0.3);
  }
}

TEST_CASE("keeps the authored cells of a body that tears or flows", "[strainstep]") {
  talus::PhysicalMaterial tearing = stiffBeam(2.0e8);
  talus::PhysicalMaterial flowing = stiffBeam(2.0e8);

  tearing.tearStrain = 0.4f;
  flowing.yieldStress = 1.0e6f;

  for (const talus::PhysicalMaterial& material : {tearing, flowing}) {
    uint32_t coarsening = 0u;
    const SoftShape shape = resolvedBox(glm::dvec3(2.0, 0.2, 0.2), 0.05, material, coarsening);

    CHECK(coarsening == 1u);
  }
}


TEST_CASE("keeps the volume of a box whose faces lie on grid planes when it coarsens", "[strainstep]") {
  const glm::dvec3 half(2.0, 0.2, 0.2);
  const std::vector<talus::PhysicalMaterial> lanes(2u, stiffBeam(2.0e8));
  const talus::dynamics::IntegrationParameters parameters;
  const talus::dynamics::ResolvedMesh resolved =
    talus::dynamics::resolveMesh(boxField(half, 0.05), lanes, glm::mat4(1.0f), {}, parameters);

  REQUIRE(resolved.mesh != nullptr);
  REQUIRE(resolved.coarsening == 8u);

  double volume = 0.0;

  for (const talus::PreparedHex& hex : resolved.mesh->hexes) {
    volume += hex.volume;
  }

  const double box = 8.0 * half.x * half.y * half.z;

  CHECK(std::abs(volume - box) / box <= talus::dynamics::SHAPE_TOLERANCE);
}

TEST_CASE("keeps the cells of a bar one cell thick whose faces lie on grid planes", "[strainstep]") {
  const std::vector<talus::PhysicalMaterial> lanes(2u, stiffBeam(2.0e8));
  const talus::dynamics::IntegrationParameters parameters;
  const talus::dynamics::ResolvedMesh resolved = talus::dynamics::resolveMesh(
    boxField(glm::dvec3(2.0, 0.05, 0.05), 0.1),
    lanes,
    glm::mat4(1.0f),
    {},
    parameters
  );

  REQUIRE(resolved.mesh != nullptr);
  CHECK(resolved.coarsening == 1u);
  CHECK(resolved.mesh->hexes.size() == 40u);
}

TEST_CASE("keeps a clamped body clamped when it coarsens", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 2> clamps{
    holdAt(glm::dvec3(-2.0, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(-1.8, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const glm::dvec3 half(2.0, 0.2, 0.2);
  const SoftShape authored = softBox(half, 0.05, stiffBeam(2.0e8), clamps);
  uint32_t coarsening = 0u;
  const SoftShape resolved = resolvedBox(half, 0.05, stiffBeam(2.0e8), coarsening, clamps);

  for (const talus::deformable::SupportVolume& clamp : clamps) {
    REQUIRE(nodesIn(*authored.mesh, clamp) > 0u);
    CHECK(nodesIn(*resolved.mesh, clamp) > 0u);
  }

  CHECK(coarsening == 4u);
}

TEST_CASE("finds the crack face of a torn mesh as a surface", "[strainstep]") {
  talus::PreparedMesh mesh;
  const auto node = [](uint32_t x, uint32_t y, uint32_t z) { return x + 3u * (y + 2u * z); };

  mesh.voxelSize = glm::vec3(0.1f);

  for (uint32_t z = 0u; z != 2u; ++z) {
    for (uint32_t y = 0u; y != 2u; ++y) {
      for (uint32_t x = 0u; x != 3u; ++x) {
        mesh.restPositions.push_back(glm::vec3(-0.1f + 0.1f * x, -0.05f + 0.1f * y, -0.05f + 0.1f * z));
      }
    }
  }

  std::array<uint32_t, 4> copies{};

  for (uint32_t at = 0u; at != 4u; ++at) {
    copies[at] = static_cast<uint32_t>(mesh.restPositions.size());
    mesh.restPositions.push_back(mesh.restPositions[node(1u, at & 1u, at >> 1u)]);
  }

  talus::PreparedHex left;
  talus::PreparedHex right;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const uint32_t x = corner & 1u;
    const uint32_t y = (corner >> 1u) & 1u;
    const uint32_t z = (corner >> 2u) & 1u;

    left.nodes[corner] = node(x, y, z);
    right.nodes[corner] = x == 0u ? copies[y + 2u * z] : node(2u, y, z);
  }

  left.volume = right.volume = 0.001f;
  left.halfExtents = right.halfExtents = glm::vec3(0.05f);
  mesh.hexes = {left, right};
  mesh.crackFaces = {static_cast<uint8_t>(1u << 1u), static_cast<uint8_t>(1u << 0u)};

  const talus::field::SampledSdf field = boxField(glm::dvec3(0.1, 0.05, 0.05), 0.05);
  talus::deformable::DeformedProbe probe = talus::deformable::deformedProbe(mesh, field);
  std::vector<glm::dvec3> placed(mesh.restPositions.begin(), mesh.restPositions.end());

  for (const uint32_t moved : right.nodes) {
    placed[moved].x += 0.1;
  }

  talus::deformable::placeDeformedProbe(probe, placed);

  const talus::deformable::DeformedDistance opening =
    talus::deformable::probeDeformed(probe, glm::dvec3(0.05, 0.0, 0.0));
  const talus::deformable::DeformedDistance inside =
    talus::deformable::probeDeformed(probe, glm::dvec3(-0.01, 0.0, 0.0));

  REQUIRE(opening.found);
  CHECK(opening.distance > 0.04);
  REQUIRE(inside.found);
  CHECK(std::abs(inside.distance + 0.01) < 1.0e-3);
  CHECK(inside.normal.x > 0.99);
}

TEST_CASE("keeps a point inside its yield surface elastic", "[strainstep]") {
  const glm::dmat3 elastic(glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.001, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0));

  CHECK(!talus::dynamics::returnMap(elastic, 1.0e5, 1.0e3).has_value());
}

TEST_CASE("brings a point past its yield back onto its yield surface and keeps its volume", "[strainstep]") {
  const glm::dmat3 elastic(glm::dvec3(1.05, 0.02, 0.0), glm::dvec3(0.2, 0.98, 0.01), glm::dvec3(0.0, 0.03, 1.01));
  const std::optional<talus::dynamics::ReturnMap> mapped = talus::dynamics::returnMap(elastic, 1.0e5, 2.0e3);

  REQUIRE(mapped.has_value());
  CHECK(std::abs(talus::dynamics::vonMises(mapped->elastic, 1.0e5) - 2.0e3) <= 1.0e-9 * 2.0e3);
  CHECK(std::abs(glm::determinant(mapped->elastic) - glm::determinant(elastic)) <= 1.0e-12);
  CHECK(std::abs(mapped->strain - (mapped->trial - 2.0e3) / 3.0e5) <= 1.0e-12);
}

TEST_CASE("hardens a point as its material's hardening modulus says", "[strainstep]") {
  talus::PhysicalMaterial material = clay(1.0e5, 2.0e3);

  material.hardeningModulus = 5.0e4f;

  const SoftShape shape = softBox(glm::dvec3(0.1), 0.2, material);
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const glm::dvec3 rest(shape.mesh->restPositions[node]);

    lattice.position[node] = rest + glm::dvec3(0.2 * rest.y, 0.0, 0.0);
  }

  (void)talus::dynamics::flowLattice(lattice);

  REQUIRE(lattice.hardening.size() == 8u);

  const talus::dynamics::ElementMaterial moduli = talus::dynamics::elementMaterial(material);
  const talus::dynamics::GaussGradients& gradients = lattice.gaussGradients[lattice.gaussGradientsOf[0]];
  glm::dmat3 deformation(0.0);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    deformation += glm::outerProduct(lattice.position[shape.mesh->hexes[0].nodes[corner]], gradients[0][corner]);
  }

  const double yieldNow = 2.0e3 + 5.0e4 * lattice.hardening[0];

  CHECK(lattice.hardening[0] > 0.0);
  CHECK(std::abs(talus::dynamics::vonMises(deformation * lattice.plastic[0], moduli.shear) - yieldNow) <= 1.0e-6 * yieldNow);
}

TEST_CASE("keeps every point's plastic stretch within the material's cap, as Rapier's plastic_max", "[strainstep]") {
  talus::PhysicalMaterial material = clay(5.0e3, 50.0);
  const SoftShape shape = softBox(glm::dvec3(0.25), 0.5, material);
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  for (uint32_t pull = 1u; pull <= 6u; ++pull) {
    const double stretch = 1.0 + static_cast<double>(pull);

    for (size_t node = 0u; node != lattice.position.size(); ++node) {
      const glm::dvec3 rest(shape.mesh->restPositions[node]);

      lattice.position[node] = glm::dvec3(rest.x * stretch, rest.y / stretch, rest.z);
    }

    (void)talus::dynamics::flowLattice(lattice);
  }

  double largest = 0.0;

  for (const glm::dmat3& inverse : lattice.plastic) {
    largest = std::max(largest, frobenius(plasticStretchOf(inverse) - glm::dmat3(1.0)));
  }

  CHECK(largest > 0.5);
  CHECK(largest <= 1.0 + 1.0e-9);
}

TEST_CASE("crushes a squeezed point no further than its crush limit", "[strainstep]") {
  talus::PhysicalMaterial material = clay(1.0e5, 1.0e9);

  material.crushStress = 1.0e3f;
  material.crushLimit = 0.7f;

  const SoftShape shape = softBox(glm::dvec3(0.1), 0.2, material);
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  for (uint32_t squeeze = 0u; squeeze != 10u; ++squeeze) {
    for (size_t node = 0u; node != lattice.position.size(); ++node) {
      lattice.position[node] = 0.6 * glm::dvec3(shape.mesh->restPositions[node]);
    }

    (void)talus::dynamics::flowLattice(lattice);
  }

  double smallest = 1.0;

  for (const glm::dmat3& inverse : lattice.plastic) {
    smallest = std::min(smallest, glm::determinant(plasticStretchOf(inverse)));
  }

  CHECK(smallest < 0.99);
  CHECK(smallest >= static_cast<double>(material.crushLimit) - 1.0e-9);
}

TEST_CASE("fits a cell's rest shape to a uniform plastic stretch and writes the same stretch back", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.1), 0.2, clay(1.0e5, 2.0e3));
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];
  const glm::dmat3 inverse = glm::inverse(glm::dmat3(
    glm::dvec3(1.1, 0.05, 0.0),
    glm::dvec3(0.0, 0.9, 0.02),
    glm::dvec3(0.01, 0.0, 1.0)
  ));

  lattice.plastic.assign(8u, inverse);
  lattice.hardening.assign(8u, 0.0);

  const talus::dynamics::ElementRest rest = talus::dynamics::plasticRest(lattice, 0u);

  talus::dynamics::writeFitted(lattice, 0u, rest);

  for (const glm::dmat3& written : lattice.plastic) {
    CHECK(frobenius(written - inverse) <= 1.0e-12);
  }

  for (const glm::dvec3& hourglass : rest.hourglass) {
    CHECK(glm::length(hourglass) <= 1.0e-12);
  }
}

TEST_CASE("keeps a cell's plastic volume when its rest shape is fitted and written back", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.1), 0.2, clay(1.0e5, 2.0e3));
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  lattice.plastic.assign(8u, glm::dmat3(1.0));
  lattice.hardening.assign(8u, 0.0);

  for (uint32_t point = 0u; point != 8u; ++point) {
    const double side = talus::dynamics::cornerSigns(point).z;

    lattice.plastic[point] = glm::inverse(glm::dmat3(
      glm::dvec3(std::exp(0.2 * side), 0.0, 0.0),
      glm::dvec3(0.0, std::exp(-0.2 * side), 0.0),
      glm::dvec3(0.0, 0.0, 1.0)
    ));
  }

  talus::dynamics::writeFitted(lattice, 0u, talus::dynamics::plasticRest(lattice, 0u));

  double volume = 0.0;

  for (const glm::dmat3& inverse : lattice.plastic) {
    volume += glm::determinant(plasticStretchOf(inverse)) / 8.0;
  }

  CHECK(std::abs(volume - 1.0) <= 1.0e-12);

  const std::vector<glm::dmat3> written = lattice.plastic;

  for (uint32_t pass = 0u; pass != 5u; ++pass) {
    talus::dynamics::writeFitted(lattice, 0u, talus::dynamics::plasticRest(lattice, 0u));
  }

  for (size_t point = 0u; point != written.size(); ++point) {
    CHECK(frobenius(lattice.plastic[point] - written[point]) <= 1.0e-12);
  }
}

TEST_CASE("leaves a resting plastic state untouched when nothing moves", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.1), 0.2, clay(1.0e5, 2.0e3));
  World world;
  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const glm::dvec3 rest(shape.mesh->restPositions[node]);

    lattice.position[node] = rest + glm::dvec3(0.2 * rest.y, 0.0, 0.0);
  }

  REQUIRE(talus::dynamics::flowLattice(lattice) > 0.0);

  const std::vector<glm::dmat3> resting = lattice.plastic;
  const std::vector<double> hardened = lattice.hardening;

  CHECK(talus::dynamics::flowLattice(lattice) <= 1.0e-12);
  CHECK(lattice.plastic == resting);
  CHECK(lattice.hardening == hardened);
}

TEST_CASE("springs an elastic block sheared by its pinned top back once the top is released (Rapier plastic_cells_keep_their_deformation, elastic half)", "[strainstep]") {
  CHECK(std::abs(lean(0.0)) < 0.15);
}

TEST_CASE("keeps the lean of a block sheared past its yield once the top is released (Rapier plastic_cells_keep_their_deformation, plastic half)", "[strainstep]") {
  CHECK(lean(0.05) > 0.4);
}

TEST_CASE("keeps the plastic volume of a clay column collapsing under its own weight", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial column = jelly(2.0e3, 5.0);

  column.poissonsRatio = 0.35f;
  column.friction = 1.0f;
  column.yieldStress = 2.0e3f * 0.04f;
  column.plasticMax = 1.0f;

  const SoftShape shape = softBox(glm::dvec3(0.3, 1.2, 0.3), 0.3, column);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.6);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 1.2, 0.0));

  for (uint32_t frame = 0u; frame != 600u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];
  double volume = 0.0;

  REQUIRE(!body.plastic.empty());

  for (const glm::dmat3& inverse : body.plastic) {
    volume += 1.0 / glm::determinant(inverse) / static_cast<double>(body.plastic.size());
  }

  CHECK(plasticStrain(body) > 0.0);
  CHECK(std::abs(volume - 1.0) < 1.0e-6);
}

TEST_CASE("stops flowing in a strip one cell thick held bent past its yield", "[strainstep]") {
  const SoftShape shape = stripShape();
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.0, 0.0));
  const std::vector<uint32_t> tip = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.x > 1.19; });
  std::vector<double> rest;
  double held = 0.0;

  for (const uint32_t node : tip) {
    rest.push_back(world.bodies.lattices[index].position[node].y);
  }

  for (uint32_t frame = 1u; frame <= 120u; ++frame) {
    for (size_t at = 0u; at != tip.size(); ++at) {
      world.bodies.lattices[index].position[tip[at]].y =
        rest[at] - 0.5 * std::min(static_cast<double>(frame) / 60.0, 1.0);
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (frame == 90u) {
      held = plasticStrain(world.bodies.lattices[index]);
    }
  }

  CHECK(held > 0.0);
  CHECK(plasticStrain(world.bodies.lattices[index]) - held < 0.01 * held);
}

TEST_CASE("keeps most of the bend of a strip one cell thick bent past its yield once released", "[strainstep]") {
  const SoftShape shape = stripShape();
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.0, 0.0));
  const std::vector<uint32_t> tip = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.x > 1.19; });
  std::vector<double> rest;

  for (const uint32_t node : tip) {
    rest.push_back(world.bodies.lattices[index].position[node].y);
  }

  for (uint32_t frame = 1u; frame <= 120u; ++frame) {
    for (size_t at = 0u; at != tip.size(); ++at) {
      world.bodies.lattices[index].position[tip[at]].y =
        rest[at] - 0.5 * std::min(static_cast<double>(frame) / 60.0, 1.0);
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  setAnchored(world.bodies.lattices[index], tip, 0u);

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  double kept = 0.0;

  for (size_t at = 0u; at != tip.size(); ++at) {
    kept += (rest[at] - world.bodies.lattices[index].position[tip[at]].y) / static_cast<double>(tip.size());
  }

  CHECK(kept > 0.25);
}

TEST_CASE("keeps a clay slab stamped four times sound (Rapier plastic_flow_is_bounded)", "[strainstep][slow]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial slab = jelly(3.0e4, 20.0);

  slab.poissonsRatio = 0.35f;
  slab.friction = 0.8f;
  slab.dampingRatio = 1.0f;
  slab.yieldStress = 0.02f * 3.0e4f;
  slab.plasticMax = 1.0f;

  const SoftShape shape = softBox(glm::dvec3(2.4, 0.4, 0.8), 0.8 / 3.0, slab);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.35), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.6);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.4, 0.0));
  const double before = latticeVolume(world.bodies.lattices[index]);
  const uint32_t press = talus::dynamics::addRigidBox(
    world.bodies, pressField, glm::dvec3(0.35), 0.0, glm::dvec3(-1.5, 2.2, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  const glm::dquat corner = glm::angleAxis(0.25 * 3.14159265358979323846, glm::dvec3(0.0, 0.0, 1.0));
  const double dt = world.parameters.dt;
  double time = 0.0;

  world.bodies.rigids[press].rotation = corner;
  world.bodies.rigids[press].initialRotation = corner;
  world.bodies.rigids[press].inertialRotation = corner;

  for (uint32_t frame = 0u; frame != 720u; ++frame) {
    time += dt;

    const double cycle = std::floor(time / 3.0);
    const double phase = time - 3.0 * cycle;
    const double depth = phase < 1.0 ? phase : phase < 2.0 ? 2.0 - phase : 0.0;
    talus::dynamics::RigidBody& body = world.bodies.rigids[press];

    body.velocity = glm::dvec3(
      (-1.5 + cycle - body.position.x) / dt,
      (2.2 - depth - body.position.y) / dt,
      -body.position.z / dt
    );
    body.angularVelocity = glm::dvec3(0.0);

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];
  uint32_t inverted = 0u;

  for (const talus::PreparedHex& hex : body.mesh->hexes) {
    glm::dmat3 shapeNow(0.0);

    for (uint32_t c = 0u; c != 8u; ++c) {
      shapeNow += glm::outerProduct(body.position[hex.nodes[c]], talus::dynamics::cornerSigns(c));
    }

    inverted += glm::determinant(shapeNow) > 0.0 ? 0u : 1u;
  }

  CHECK(plasticStrain(body) > 0.0);
  CHECK(inverted <= 2u);
  CHECK(std::abs(latticeVolume(body) - before) < 0.05 * before);
  CHECK(fastestNode(body) < 1.0);
}

TEST_CASE("keeps a clay slab stamped deeper sound when its plastic stretch is not capped", "[strainstep][slow][!mayfail]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial slab = jelly(3.0e4, 20.0);

  slab.poissonsRatio = 0.35f;
  slab.friction = 0.8f;
  slab.dampingRatio = 1.0f;
  slab.yieldStress = 0.02f * 3.0e4f;

  const SoftShape shape = softBox(glm::dvec3(2.4, 0.4, 0.8), 0.8 / 3.0, slab);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.35), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.6);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.4, 0.0));
  const double before = latticeVolume(world.bodies.lattices[index]);
  const uint32_t press = talus::dynamics::addRigidBox(
    world.bodies, pressField, glm::dvec3(0.35), 0.0, glm::dvec3(-1.5, 2.2, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  const glm::dquat corner = glm::angleAxis(0.25 * 3.14159265358979323846, glm::dvec3(0.0, 0.0, 1.0));
  const double dt = world.parameters.dt;
  double time = 0.0;

  world.bodies.rigids[press].rotation = corner;
  world.bodies.rigids[press].initialRotation = corner;
  world.bodies.rigids[press].inertialRotation = corner;

  for (uint32_t frame = 0u; frame != 720u; ++frame) {
    time += dt;

    const double cycle = std::floor(time / 3.0);
    const double phase = time - 3.0 * cycle;
    const double depth = 1.5 * (phase < 1.0 ? phase : phase < 2.0 ? 2.0 - phase : 0.0);
    talus::dynamics::RigidBody& body = world.bodies.rigids[press];

    body.velocity = glm::dvec3(
      (-1.5 + cycle - body.position.x) / dt,
      (2.2 - depth - body.position.y) / dt,
      -body.position.z / dt
    );
    body.angularVelocity = glm::dvec3(0.0);

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];
  uint32_t inverted = 0u;

  for (const talus::PreparedHex& hex : body.mesh->hexes) {
    glm::dmat3 shapeNow(0.0);

    for (uint32_t c = 0u; c != 8u; ++c) {
      shapeNow += glm::outerProduct(body.position[hex.nodes[c]], talus::dynamics::cornerSigns(c));
    }

    inverted += glm::determinant(shapeNow) > 0.0 ? 0u : 1u;
  }

  CHECK(plasticStrain(body) > 0.0);
  CHECK(inverted <= 2u);
  CHECK(std::abs(latticeVolume(body) - before) < 0.05 * before);
  CHECK(fastestNode(body) < 1.0);
}

TEST_CASE("keeps a deep-stamped uncapped clay slab sound when its substeps change", "[strainstep][slow][!mayfail]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial slab = jelly(3.0e4, 20.0);

  slab.poissonsRatio = 0.35f;
  slab.friction = 0.8f;
  slab.dampingRatio = 1.0f;
  slab.yieldStress = 0.02f * 3.0e4f;

  const SoftShape shape = softBox(glm::dvec3(2.4, 0.4, 0.8), 0.8 / 3.0, slab);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.35), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.6);

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.4, 0.0));
  const double before = latticeVolume(world.bodies.lattices[index]);
  const uint32_t press = talus::dynamics::addRigidBox(
    world.bodies, pressField, glm::dvec3(0.35), 0.0, glm::dvec3(-1.5, 2.2, 0.0), glm::dvec3(0.0), 0.5, 5u
  );
  const glm::dquat corner = glm::angleAxis(0.25 * 3.14159265358979323846, glm::dvec3(0.0, 0.0, 1.0));
  const double dt = world.parameters.dt;
  double time = 0.0;

  world.bodies.rigids[press].rotation = corner;
  world.bodies.rigids[press].initialRotation = corner;
  world.bodies.rigids[press].inertialRotation = corner;

  for (uint32_t frame = 0u; frame != 720u; ++frame) {
    time += dt;

    const double cycle = std::floor(time / 3.0);
    const double phase = time - 3.0 * cycle;
    const double depth = 1.5 * (phase < 1.0 ? phase : phase < 2.0 ? 2.0 - phase : 0.0);
    talus::dynamics::RigidBody& body = world.bodies.rigids[press];

    body.velocity = glm::dvec3(
      (-1.5 + cycle - body.position.x) / dt,
      (2.2 - depth - body.position.y) / dt,
      -body.position.z / dt
    );
    body.angularVelocity = glm::dvec3(0.0);

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  world.bodies.rigids[press].velocity = glm::dvec3(0.0);
  world.bodies.lattices[index].substeps = 8u;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);

    if (frame == 0u) {
      CHECK(report.substeps == 8u);
    }
  }

  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];
  uint32_t inverted = 0u;

  for (const talus::PreparedHex& hex : body.mesh->hexes) {
    glm::dmat3 shapeNow(0.0);

    for (uint32_t c = 0u; c != 8u; ++c) {
      shapeNow += glm::outerProduct(body.position[hex.nodes[c]], talus::dynamics::cornerSigns(c));
    }

    inverted += glm::determinant(shapeNow) > 0.0 ? 0u : 1u;
  }

  CHECK(plasticStrain(body) > 0.0);
  CHECK(inverted <= 2u);
  CHECK(std::abs(latticeVolume(body) - before) < 0.05 * before);
  CHECK(fastestNode(body) < 1.0);
}

TEST_CASE("rebuilds a deep-stamped uncapped clay slab without a jolt", "[strainstep][slow][!mayfail]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial slab = jelly(3.0e4, 20.0);

  slab.poissonsRatio = 0.35f;
  slab.friction = 0.8f;
  slab.dampingRatio = 1.0f;
  slab.yieldStress = 0.02f * 3.0e4f;

  const SoftShape shape = softBox(glm::dvec3(2.4, 0.4, 0.8), 0.8 / 3.0, slab);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.35), 0.05);

  struct Aftermath {
    double peak = 0.0;
    double before = 0.0;
    double volume = 0.0;
    double plastic = 0.0;
    double fastest = 0.0;
    uint32_t inverted = 0u;
    uint32_t substeps = 0u;
  };

  const auto stamp = [&](uint32_t substeps) {
    World world;

    useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
    addFloor(world, floor, 0.6);

    const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 0.4, 0.0));
    const uint32_t press = talus::dynamics::addRigidBox(
      world.bodies, pressField, glm::dvec3(0.35), 0.0, glm::dvec3(-1.5, 2.2, 0.0), glm::dvec3(0.0), 0.5, 5u
    );
    const glm::dquat corner = glm::angleAxis(0.25 * 3.14159265358979323846, glm::dvec3(0.0, 0.0, 1.0));
    const double dt = world.parameters.dt;
    double time = 0.0;
    Aftermath after;

    after.before = latticeVolume(world.bodies.lattices[index]);
    world.bodies.rigids[press].rotation = corner;
    world.bodies.rigids[press].initialRotation = corner;
    world.bodies.rigids[press].inertialRotation = corner;

    for (uint32_t frame = 0u; frame != 720u; ++frame) {
      time += dt;

      const double cycle = std::floor(time / 3.0);
      const double phase = time - 3.0 * cycle;
      const double depth = 1.5 * (phase < 1.0 ? phase : phase < 2.0 ? 2.0 - phase : 0.0);
      talus::dynamics::RigidBody& body = world.bodies.rigids[press];

      body.velocity = glm::dvec3(
        (-1.5 + cycle - body.position.x) / dt,
        (2.2 - depth - body.position.y) / dt,
        -body.position.z / dt
      );
      body.angularVelocity = glm::dvec3(0.0);

      REQUIRE(!talus::dynamics::step(world).nonFinite);
    }

    world.bodies.rigids[press].velocity = glm::dvec3(0.0);
    world.bodies.lattices[index].substeps = substeps;

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      const talus::dynamics::StepReport report = talus::dynamics::step(world);

      REQUIRE(!report.nonFinite);

      if (frame == 0u) {
        after.substeps = report.substeps;
      }

      if (frame < 10u) {
        after.peak = std::max(after.peak, fastestNode(world.bodies.lattices[index]));
      }
    }

    const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];

    for (const talus::PreparedHex& hex : body.mesh->hexes) {
      glm::dmat3 shapeNow(0.0);

      for (uint32_t c = 0u; c != 8u; ++c) {
        shapeNow += glm::outerProduct(body.position[hex.nodes[c]], talus::dynamics::cornerSigns(c));
      }

      after.inverted += glm::determinant(shapeNow) > 0.0 ? 0u : 1u;
    }

    after.volume = latticeVolume(body);
    after.plastic = plasticStrain(body);
    after.fastest = fastestNode(body);

    return after;
  };
  const Aftermath rebuilt = stamp(8u);
  const Aftermath kept = stamp(4u);

  REQUIRE(rebuilt.substeps == 8u);
  REQUIRE(kept.substeps == 4u);
  REQUIRE(rebuilt.before == kept.before);

  CHECK(rebuilt.peak < kept.peak + 0.05);
  CHECK(rebuilt.plastic > 0.0);
  CHECK(rebuilt.inverted <= 2u);
  CHECK(std::abs(rebuilt.volume - rebuilt.before) < 0.05 * rebuilt.before);
  CHECK(rebuilt.fastest < 1.0);
}

TEST_CASE("tears a strip pulled apart into pieces without removing a cell or any mass (Rapier strip_tears_when_pulled_apart)", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(1.2, 0.2, 0.2), 0.2, tearable(), ends);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.0, 0.0));
  const size_t cells = shape.mesh->hexes.size();
  const size_t nodes = shape.mesh->restPositions.size();
  const double mass = latticeMass(world.bodies.lattices[index]);
  const std::vector<uint32_t> right = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.x > 1.19; });
  std::vector<double> start;

  for (const uint32_t node : right) {
    start.push_back(world.bodies.lattices[index].position[node].x);
  }

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    for (size_t at = 0u; at != right.size(); ++at) {
      world.bodies.lattices[index].position[right[at]].x =
        start[at] + 2.4 * std::min(static_cast<double>(frame) / 180.0, 1.0);
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];

  CHECK(talus::dynamics::latticePieces(body) >= 2u);
  CHECK(body.mesh->hexes.size() == cells);
  CHECK(std::abs(latticeMass(body) - mass) <= 1.0e-6 * mass);
  CHECK(body.position.size() > nodes);
  CHECK(orphans(body) == 0u);
  CHECK(longestEdge(body) < 1.8);
}

TEST_CASE("splits the nodes two halves share alone after a crack through a strip one cell thick", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, tearable());
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 1.0, 0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];
  const size_t nodes = lattice.position.size();
  const double mass = latticeMass(lattice);

  REQUIRE(shape.mesh->hexes.size() == 4u);

  for (glm::dvec3& position : lattice.position) {
    position.x *= 1.01;
  }

  CHECK(talus::dynamics::tearCells(lattice, std::array<uint32_t, 1>{1u}) == 4u);
  CHECK(lattice.position.size() == nodes + 4u);
  CHECK(talus::dynamics::latticePieces(lattice) == 2u);
  CHECK(lattice.mesh->hexes.size() == 4u);
  CHECK(std::abs(latticeMass(lattice) - mass) <= 1.0e-6 * mass);
  CHECK(lattice.mesh->crackFaces.size() == 4u);
  CHECK(lattice.mesh->crackFaces[1] != 0u);
  CHECK(lattice.mesh->crackFaces[2] != 0u);

  for (uint32_t copy = static_cast<uint32_t>(nodes); copy != nodes + 4u; ++copy) {
    CHECK(std::find(lattice.mesh->surfaceNodes.begin(), lattice.mesh->surfaceNodes.end(), copy)
          != lattice.mesh->surfaceNodes.end());
  }

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(orphans(world.bodies.lattices[index]) == 0u);
}

TEST_CASE("tears a strip a ball rests on and keeps every contact on a node that exists", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(1.2, 0.2, 0.2), 0.2, tearable(), ends);
  const talus::field::SampledSdf ball = ballField(0.1, 0.02);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.0, 0.0));

  (void)talus::dynamics::addRigidSphere(
    world.bodies, ball, 0.1, 0.5, glm::dvec3(0.0, 2.31, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  const std::vector<uint32_t> right = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.x > 1.19; });
  std::vector<double> start;

  for (const uint32_t node : right) {
    start.push_back(world.bodies.lattices[index].position[node].x);
  }

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    for (size_t at = 0u; at != right.size(); ++at) {
      world.bodies.lattices[index].position[right[at]].x =
        start[at] + 2.4 * std::min(static_cast<double>(frame) / 180.0, 1.0);
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (const talus::dynamics::Contact& contact : world.contacts) {
      for (const talus::dynamics::ContactSide* side : {&contact.a, &contact.b}) {
        if (side->side != talus::dynamics::Side::Lattice) {
          continue;
        }

        for (uint32_t corner = 0u; corner != 8u; ++corner) {
          if (side->weights[corner] != 0.0) {
            REQUIRE(side->nodes[corner] < world.bodies.lattices[side->body].position.size());
          }
        }
      }
    }
  }

  CHECK(talus::dynamics::latticePieces(world.bodies.lattices[index]) >= 2u);
}

TEST_CASE("leaves the mesh another lattice shares untouched when one lattice tears", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, tearable());
  World world;
  const uint32_t torn = addSoft(world, shape, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t kept = addSoft(world, shape, glm::dvec3(0.0, 3.0, 0.0));
  const size_t nodes = shape.mesh->restPositions.size();

  for (glm::dvec3& position : world.bodies.lattices[torn].position) {
    position.x *= 1.01;
  }

  REQUIRE(talus::dynamics::tearCells(world.bodies.lattices[torn], std::array<uint32_t, 1>{1u}) > 0u);
  CHECK(world.bodies.lattices[torn].mesh != shape.mesh);
  CHECK(world.bodies.lattices[kept].mesh == shape.mesh);
  CHECK(shape.mesh->restPositions.size() == nodes);
  CHECK(shape.mesh->crackFaces.empty());
}

TEST_CASE("keeps a torn lattice's plastic state one entry per Gauss point per cell, in cell order", "[strainstep]") {
  talus::PhysicalMaterial material = tearable();

  material.yieldStress = 250.0f;
  material.plasticMax = 1.0f;

  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(1.2, 0.2, 0.2), 0.2, material, ends);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.0, 0.0));
  const size_t cells = shape.mesh->hexes.size();
  const std::vector<uint32_t> right = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.x > 1.19; });
  std::vector<double> start;

  for (const uint32_t node : right) {
    start.push_back(world.bodies.lattices[index].position[node].x);
  }

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    for (size_t at = 0u; at != right.size(); ++at) {
      world.bodies.lattices[index].position[right[at]].x =
        start[at] + 2.4 * std::min(static_cast<double>(frame) / 180.0, 1.0);
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    REQUIRE(world.bodies.lattices[index].plastic.size() == 8u * cells);
    REQUIRE(world.bodies.lattices[index].hardening.size() == 8u * cells);
  }

  const talus::dynamics::LatticeBody& body = world.bodies.lattices[index];

  CHECK(body.mesh != shape.mesh);
  CHECK(talus::dynamics::latticePieces(body) >= 2u);
  CHECK(talus::dynamics::flowed(body));
  REQUIRE(body.mesh->hexes.size() == cells);

  for (uint32_t cell = 0u; cell != cells; ++cell) {
    glm::dvec3 before(0.0);
    glm::dvec3 after(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      before += glm::dvec3(shape.mesh->restPositions[shape.mesh->hexes[cell].nodes[corner]]) / 8.0;
      after += glm::dvec3(body.mesh->restPositions[body.mesh->hexes[cell].nodes[corner]]) / 8.0;
    }

    CHECK(glm::length(after - before) <= 1.0e-9);
  }
}

TEST_CASE("demotes a promoted body once it rests and promotes it again turned over without a jolt", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.1, jelly(2.0e5, 500.0));
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.31, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  talus::dynamics::promote(world, id);

  uint32_t demotions = 0u;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    demotions += report.demotions;
  }

  REQUIRE(demotions >= 1u);
  REQUIRE(world.bodies.switchables.front().tier == talus::dynamics::Tier::Rigid);

  const std::optional<talus::dynamics::BodyRef> rigid = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(rigid.has_value());

  talus::dynamics::RigidBody& body = world.bodies.rigids[rigid->index];
  const glm::dquat over = glm::angleAxis(3.14159265358979323846, glm::dvec3(1.0, 0.0, 0.0)) * body.rotation;

  body.rotation = over;
  body.initialRotation = over;
  body.inertialRotation = over;
  talus::dynamics::promote(world, id);

  double fastest = 0.0;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (const talus::dynamics::LatticeBody& lattice : world.bodies.lattices) {
      fastest = std::max(fastest, fastestNode(lattice));
    }
  }

  CHECK(fastest < 0.5);
}

TEST_CASE("keeps a dent in a clay body that returns to rigid", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial material = clay(1.0e6, 2.0e4);

  material.friction = 0.8f;

  const SoftShape shape = softBox(glm::dvec3(0.3), 0.1, material);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.15), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.8);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.3, 0.0), glm::dvec3(0.0), 0.8, 0.0
  );
  const uint32_t pressId = world.bodies.rigids[talus::dynamics::addRigidBox(
    world.bodies, pressField, glm::dvec3(0.15), 0.0, glm::dvec3(0.0, 0.76, 0.0), glm::dvec3(0.0), 0.5, 5u
  )].id;
  const double mass = world.bodies.switchables.front().rest.totalMass;

  talus::dynamics::promote(world, id);

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    const std::optional<talus::dynamics::BodyRef> found = talus::dynamics::findBody(world.bodies, pressId);

    REQUIRE(found.has_value());

    talus::dynamics::RigidBody& pusher = world.bodies.rigids[found->index];
    const double time = static_cast<double>(frame) / 60.0;

    pusher.velocity = time < 1.0 ? glm::dvec3(0.0, -0.08, 0.0)
                      : time < 2.0 ? glm::dvec3(0.0, 0.5, 0.0)
                                   : glm::dvec3(2.0, 0.0, 0.0);
    pusher.angularVelocity = glm::dvec3(0.0);

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::SwitchableBody& body = world.bodies.switchables.front();

  REQUIRE(body.tier == talus::dynamics::Tier::Rigid);
  REQUIRE(body.baked != nullptr);

  const double before = talus::field::probe<double>(shape.field, glm::dvec3(0.0, 0.27, 0.0)).distance;
  const double after = talus::field::probe<double>(*body.baked, glm::dvec3(0.0, 0.27, 0.0)).distance;

  CHECK(before < 0.0);
  CHECK(after > 0.0);
  CHECK(std::abs(body.rest.totalMass - mass) <= 1.0e-6 * mass);
}

TEST_CASE("keeps a torn switchable body a lattice", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, tearable());
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.26, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  talus::dynamics::promote(world, id);

  const std::optional<talus::dynamics::BodyRef> lattice = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(lattice.has_value());
  REQUIRE(lattice->side == talus::dynamics::Side::Lattice);

  for (glm::dvec3& position : world.bodies.lattices[lattice->index].position) {
    position.x *= 1.01;
  }

  REQUIRE(talus::dynamics::tearCells(world.bodies.lattices[lattice->index], std::array<uint32_t, 1>{1u}) > 0u);

  uint32_t demotions = 0u;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    demotions += report.demotions;
  }

  CHECK(demotions == 0u);
  CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Lattice);
}


TEST_CASE("keeps a torn switchable body a lattice while its untorn twin demotes", "[strainstep]") {
  talus::PhysicalMaterial material = tearable();

  material.youngsModulus = 5.0e5f;

  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, material);

  for (const bool torn : {false, true}) {
    World world;

    useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
    addFloor(world, floor, 0.5);

    const uint32_t id = talus::dynamics::addSwitchable(
      world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.26, 0.0), glm::dvec3(0.0), 0.5, 0.0
    );

    talus::dynamics::promote(world, id);

    const std::optional<talus::dynamics::BodyRef> lattice = talus::dynamics::findBody(world.bodies, id);

    REQUIRE(lattice.has_value());
    REQUIRE(lattice->side == talus::dynamics::Side::Lattice);

    for (glm::dvec3& position : world.bodies.lattices[lattice->index].position) {
      position.x *= 1.01;
    }

    if (torn) {
      REQUIRE(talus::dynamics::tearCells(world.bodies.lattices[lattice->index], std::array<uint32_t, 1>{1u}) > 0u);
    }

    uint32_t demotions = 0u;

    for (uint32_t frame = 0u; frame != 300u; ++frame) {
      const talus::dynamics::StepReport report = talus::dynamics::step(world);

      REQUIRE(!report.nonFinite);
      demotions += report.demotions;
    }

    if (torn) {
      CHECK(demotions == 0u);
      CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Lattice);
    } else {
      CHECK(demotions >= 1u);
    }
  }
}

TEST_CASE("refuses to demote a torn switchable body directly", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, tearable());
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.26, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  talus::dynamics::promote(world, id);

  const std::optional<talus::dynamics::BodyRef> lattice = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(lattice.has_value());
  REQUIRE(lattice->side == talus::dynamics::Side::Lattice);

  for (glm::dvec3& position : world.bodies.lattices[lattice->index].position) {
    position.x *= 1.01;
  }

  REQUIRE(talus::dynamics::tearCells(world.bodies.lattices[lattice->index], std::array<uint32_t, 1>{1u}) > 0u);

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  talus::dynamics::demote(world, id);

  const std::optional<talus::dynamics::BodyRef> after = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(after.has_value());
  CHECK(after->side == talus::dynamics::Side::Lattice);
  CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Lattice);
  CHECK(world.bodies.rigids.empty());
  CHECK(world.bodies.lattices.size() == 1u);

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Lattice);
}

TEST_CASE("keeps every copy of a surface node on the surface after a pinch splits", "[strainstep]") {
  const SoftShape shape = pinchedBoxes();

  REQUIRE(shape.mesh != nullptr);
  REQUIRE(shape.mesh->hexes.size() == 16u);
  REQUIRE(nodesWhere(*shape.mesh, [](const glm::dvec3& rest) {
            return std::abs(rest.x) < 1.0e-6 && std::abs(rest.y) < 1.0e-6;
          }).size() == 3u);

  const talus::dynamics::LatticeBody whole =
    talus::dynamics::makeLattice(shape.mesh, shape.lanes, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, &shape.field);
  const std::vector<uint32_t>& surface = shape.mesh->surfaceNodes;
  uint32_t checked = 0u;
  uint32_t missing = 0u;

  for (glm::length_t axis = 0; axis != 3; ++axis) {
    for (uint32_t cell = 0u; cell != shape.mesh->hexes.size(); ++cell) {
      talus::dynamics::LatticeBody lattice = whole;

      for (glm::dvec3& position : lattice.position) {
        position[axis] *= 1.01;
      }

      if (talus::dynamics::tearCells(lattice, std::array<uint32_t, 1>{cell}) == 0u) {
        continue;
      }

      const talus::PreparedMesh& torn = *lattice.mesh;

      for (size_t copy = whole.position.size(); copy != lattice.position.size(); ++copy) {
        const uint32_t original = nearestNode(*shape.mesh, glm::dvec3(torn.restPositions[copy]));

        if (std::find(surface.begin(), surface.end(), original) == surface.end()) {
          continue;
        }

        ++checked;
        missing += std::find(torn.surfaceNodes.begin(), torn.surfaceNodes.end(), copy) == torn.surfaceNodes.end() ? 1u : 0u;
      }
    }
  }

  CHECK(checked > 0u);
  CHECK(missing == 0u);
}

TEST_CASE("rebuilds a torn strip's surface so no surface edge spans the crack", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, tearable());
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 1.0, 0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];
  const size_t triangles = shape.mesh->surfaceTriangles.size();

  REQUIRE(shape.mesh->hexes.size() == 4u);
  REQUIRE(surfaceFlaws(*shape.mesh).defects() == 0u);

  for (glm::dvec3& position : lattice.position) {
    position.x *= 1.01;
  }

  REQUIRE(talus::dynamics::tearCells(lattice, std::array<uint32_t, 1>{1u}) == 4u);
  REQUIRE(talus::dynamics::latticePieces(lattice) == 2u);

  const talus::PreparedMesh& torn = *lattice.mesh;
  const SurfaceFlaws flaws = surfaceFlaws(torn);

  CHECK(flaws.edgesAcross == 0u);
  CHECK(flaws.trianglesAcross == 0u);
  CHECK(flaws.cracks == 2u);
  CHECK(flaws.cracksBare == 0u);
  CHECK(flaws.nodesOff == 0u);
  CHECK(flaws.arraysOff == 0u);
  CHECK(torn.surfaceTriangles.size() == triangles + 4u);
  CHECK(shape.mesh->surfaceTriangles.size() == triangles);
}

TEST_CASE("keeps a strip's surface on its cells through every tear as it is pulled apart", "[strainstep]") {
  const std::array<talus::deformable::SupportVolume, 2> ends{
    holdAt(glm::dvec3(-1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0)),
    holdAt(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.01, 1.0, 1.0))
  };
  const SoftShape shape = softBox(glm::dvec3(1.2, 0.2, 0.2), 0.2, tearable(), ends);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0, 2.0, 0.0));
  const std::vector<uint32_t> right = nodesWhere(*shape.mesh, [](const glm::dvec3& rest) { return rest.x > 1.19; });
  std::vector<double> start;

  for (const uint32_t node : right) {
    start.push_back(world.bodies.lattices[index].position[node].x);
  }

  std::shared_ptr<const talus::PreparedMesh> seen = shape.mesh;
  uint32_t tears = 0u;
  uint32_t defects = 0u;

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    for (size_t at = 0u; at != right.size(); ++at) {
      world.bodies.lattices[index].position[right[at]].x =
        start[at] + 2.4 * std::min(static_cast<double>(frame) / 180.0, 1.0);
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (world.bodies.lattices[index].mesh != seen) {
      seen = world.bodies.lattices[index].mesh;
      ++tears;
      defects += surfaceFlaws(*seen).defects();
    }
  }

  CHECK(tears > 0u);
  CHECK(talus::dynamics::latticePieces(world.bodies.lattices[index]) >= 2u);
  CHECK(surfaceFlaws(*seen).cracks > 0u);
  CHECK(defects == 0u);
}

TEST_CASE("keeps a pinched body's surface on its cells after any single tear", "[strainstep]") {
  const SoftShape shape = pinchedBoxes();

  REQUIRE(shape.mesh != nullptr);
  REQUIRE(surfaceFlaws(*shape.mesh).defects() == 0u);

  const talus::dynamics::LatticeBody whole =
    talus::dynamics::makeLattice(shape.mesh, shape.lanes, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, &shape.field);
  uint32_t tears = 0u;
  uint32_t defects = 0u;

  for (glm::length_t axis = 0; axis != 3; ++axis) {
    for (uint32_t cell = 0u; cell != shape.mesh->hexes.size(); ++cell) {
      talus::dynamics::LatticeBody lattice = whole;

      for (glm::dvec3& position : lattice.position) {
        position[axis] *= 1.01;
      }

      if (talus::dynamics::tearCells(lattice, std::array<uint32_t, 1>{cell}) == 0u) {
        continue;
      }

      ++tears;
      defects += surfaceFlaws(*lattice.mesh).defects();
    }
  }

  CHECK(tears > 0u);
  CHECK(defects == 0u);
}

TEST_CASE("tears a lattice that has no rest field", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(1.0, 0.25, 0.25), 0.5, tearable());
  World world;
  const uint32_t index = talus::dynamics::addLattice(
    world.bodies, shape.mesh, shape.lanes, glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0), 0.5
  );
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  REQUIRE(lattice.field == nullptr);

  for (glm::dvec3& position : lattice.position) {
    position.x *= 1.01;
  }

  CHECK(talus::dynamics::tearCells(lattice, std::array<uint32_t, 1>{1u}) == 4u);
  CHECK(talus::dynamics::latticePieces(lattice) == 2u);
}





TEST_CASE("rests a body baked from a dent on the floor in its rigid tier", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial material = clay(1.0e6, 2.0e4);

  material.friction = 0.8f;

  const SoftShape shape = softBox(glm::dvec3(0.3), 0.1, material);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.15), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.8);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.3, 0.0), glm::dvec3(0.0), 0.8, 0.0
  );
  const uint32_t pressId = world.bodies.rigids[talus::dynamics::addRigidBox(
    world.bodies, pressField, glm::dvec3(0.15), 0.0, glm::dvec3(0.0, 0.76, 0.0), glm::dvec3(0.0), 0.5, 5u
  )].id;
  const double mass = world.bodies.switchables.front().rigid.mass;
  bool baked = false;
  uint32_t promotions = 0u;

  talus::dynamics::promote(world, id);

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    const std::optional<talus::dynamics::BodyRef> found = talus::dynamics::findBody(world.bodies, pressId);

    REQUIRE(found.has_value());

    talus::dynamics::RigidBody& pusher = world.bodies.rigids[found->index];

    pusher.velocity = frame < 60u ? glm::dvec3(0.0, -0.08, 0.0) : glm::dvec3(0.0, 0.5, 0.0);
    pusher.angularVelocity = glm::dvec3(0.0);

    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    promotions += baked ? report.promotions : 0u;
    baked = baked || (report.demotions > 0u && world.bodies.switchables.front().baked != nullptr);
  }

  const talus::dynamics::SwitchableBody& body = world.bodies.switchables.front();
  const std::optional<talus::dynamics::BodyRef> ref = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(baked);
  REQUIRE(ref.has_value());
  REQUIRE(ref->side == talus::dynamics::Side::Rigid);
  CHECK(promotions == 0u);

  const talus::dynamics::RigidBody& rigid = world.bodies.rigids[ref->index];
  const glm::dmat3 turn = glm::mat3_cast(rigid.rotation);
  const glm::dmat3 axes = glm::mat3_cast(rigid.fieldRotation);
  const double voxel = static_cast<double>(shape.field.voxelSize.x);
  double lowest = std::numeric_limits<double>::infinity();
  double off = 0.0;

  for (const glm::dvec3& sample : rigid.samples) {
    lowest = std::min(lowest, rigid.position.y + (turn * sample).y);
    off = std::max(off, std::abs(talus::field::probe<double>(*body.baked, rigid.fieldOrigin + axes * sample).distance));
  }

  CHECK(std::abs(lowest) <= voxel);
  CHECK(off <= 0.01 * voxel);
  CHECK(std::abs(rigid.mass - mass) <= 1.0e-12 * mass);
}

TEST_CASE("demotes a pressed clay body once it rests, with sleep on", "[strainstep][slow]") {
  const talus::field::SampledSdf floor = floorField();
  talus::PhysicalMaterial material = clay(1.0e6, 2.0e4);

  material.friction = 0.8f;

  const SoftShape shape = softBox(glm::dvec3(0.3), 0.1, material);
  const talus::field::SampledSdf pressField = boxField(glm::dvec3(0.15), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  world.parameters.sleep = true;
  addFloor(world, floor, 0.8);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 0.3, 0.0), glm::dvec3(0.0), 0.8, 0.0
  );
  const uint32_t pressId = world.bodies.rigids[talus::dynamics::addRigidBox(
    world.bodies, pressField, glm::dvec3(0.15), 0.0, glm::dvec3(0.0, 0.76, 0.0), glm::dvec3(0.0), 0.5, 5u
  )].id;

  talus::dynamics::promote(world, id);

  constexpr uint32_t PUSH_FRAMES = 90u;
  constexpr uint32_t REST_FRAMES = 180u;
  uint32_t demotions = 0u;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    const std::optional<talus::dynamics::BodyRef> found = talus::dynamics::findBody(world.bodies, pressId);

    REQUIRE(found.has_value());

    talus::dynamics::RigidBody& pusher = world.bodies.rigids[found->index];

    pusher.velocity = frame < PUSH_FRAMES ? glm::dvec3(0.0, -0.08, 0.0)
                      : frame < PUSH_FRAMES + 60u ? glm::dvec3(0.0, 0.5, 0.0)
                                                  : glm::dvec3(2.0, 0.0, 0.0);
    pusher.angularVelocity = glm::dvec3(0.0);

    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    demotions += frame >= PUSH_FRAMES ? report.demotions : 0u;

    if (frame + 1u == PUSH_FRAMES + REST_FRAMES) {
      CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Rigid);
    }
  }

  CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Rigid);
  CHECK(demotions == 1u);
}

TEST_CASE("bakes a dent deeper than its cells into a body's field", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.1, jelly(1.0e6));
  const talus::PreparedMesh& mesh = *shape.mesh;
  float bottom = 1.0e9f;
  float top = -1.0e9f;

  for (const glm::vec3& node : mesh.restPositions) {
    bottom = std::min(bottom, node.y);
    top = std::max(top, node.y);
  }

  const auto bake = [&](const std::vector<glm::vec3>& position, bool carve) {
    talus::field::SampledSdf current = shape.field;

    current.materials.assign(current.distances.size(), 0u);
    (void)talus::deformable::bakeDeformedField(shape.field, mesh, position, talus::Pose{}, current, carve);

    return current;
  };
  const auto rebuild = [&](const talus::field::SampledSdf& field) {
    auto rebuilt = talus::deformable::buildMesh(
      field,
      shape.lanes,
      glm::mat4(1.0f),
      std::span<const talus::deformable::SupportVolume>{},
      talus::deformable::VolumeElementType::Hex
    );

    REQUIRE(rebuilt != nullptr);

    return rebuilt;
  };

  std::vector<glm::vec3> dented(mesh.restPositions.begin(), mesh.restPositions.end());

  for (glm::vec3& node : dented) {
    if (std::abs(node.x) <= 0.15f && std::abs(node.z) <= 0.15f) {
      node.y -= 0.25f * (node.y - bottom) / (top - bottom);
    }
  }

  const talus::field::SampledSdf carved = bake(dented, true);
  const talus::field::SampledSdf kept = bake(dented, false);

  for (const float height : {0.10f, 0.15f, 0.20f, 0.25f, 0.29f}) {
    CHECK(talus::field::distance(carved, glm::vec3(0.0f, height, 0.0f)) > 0.0f);
  }

  CHECK(talus::field::distance(kept, glm::vec3(0.0f, 0.25f, 0.0f)) < 0.0f);
  CHECK(talus::field::distance(carved, glm::vec3(0.0f, 0.0f, 0.0f)) < 0.0f);
  CHECK(talus::field::distance(carved, glm::vec3(0.25f, 0.25f, 0.0f)) < 0.0f);

  const auto dent = rebuild(carved);

  CHECK(talus::dynamics::meshPieces(*dent) == 1u);
  CHECK(dent->hexes.size() < mesh.hexes.size());

  std::vector<glm::vec3> squashed(mesh.restPositions.begin(), mesh.restPositions.end());

  for (glm::vec3& node : squashed) {
    node.y = bottom + 0.5f * (node.y - bottom);
  }

  const talus::field::SampledSdf flattened = bake(squashed, true);

  for (const float height : {0.10f, 0.15f, 0.20f, 0.25f}) {
    CHECK(talus::field::distance(flattened, glm::vec3(0.0f, height, 0.0f)) > 0.0f);
  }

  CHECK(talus::dynamics::meshPieces(*rebuild(flattened)) == 1u);
}

TEST_CASE("demotes a flowed one-voxel clay body without crashing, or keeps it soft", "[strainstep]") {
  const SoftShape shape = gridPlaneBox(glm::dvec3(0.05), 0.1, clay(1.0e6, 2.0e4));

  REQUIRE(shape.mesh != nullptr);
  REQUIRE(shape.mesh->hexes.size() == 1u);

  const glm::dmat3 squash(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.0, 1.0 / 1.2, 0.0), glm::dvec3(0.0, 0.0, 1.0));
  const glm::dmat3 shear(glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.3, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0));

  for (const glm::dmat3& flow : {squash, shear}) {
    World world;
    const uint32_t id = flowedLattice(world, shape, flow);

    talus::dynamics::demote(world, id);

    const std::optional<talus::dynamics::BodyRef> refused = talus::dynamics::findBody(world.bodies, id);
    const talus::dynamics::SwitchableBody& body = world.bodies.switchables.front();

    REQUIRE(refused.has_value());
    CHECK(refused->side == talus::dynamics::Side::Lattice);
    CHECK(body.tier == talus::dynamics::Tier::Lattice);
    CHECK(body.baked == nullptr);
    CHECK(world.bodies.rigids.empty());
    CHECK(world.bodies.lattices.size() == 1u);
    CHECK(world.bodies.lattices.front().mesh == shape.mesh);
    CHECK(talus::dynamics::flowed(world.bodies.lattices.front()));

    uint32_t demotions = 0u;

    for (uint32_t frame = 0u; frame != 30u; ++frame) {
      const talus::dynamics::StepReport report = talus::dynamics::step(world);

      REQUIRE(!report.nonFinite);
      demotions += report.demotions;
    }

    CHECK(demotions == 0u);
    CHECK(body.tier == talus::dynamics::Tier::Lattice);
    CHECK(world.bodies.lattices.size() == 1u);
  }
}

TEST_CASE("carries no contact force across a demotion its dent bake refuses", "[strainstep]") {
  const SoftShape shape = gridPlaneBox(glm::dvec3(0.05), 0.1, clay(1.0e6, 2.0e4));

  REQUIRE(shape.mesh != nullptr);

  const glm::dmat3 squash(glm::dvec3(1.2, 0.0, 0.0), glm::dvec3(0.0, 1.0 / 1.2, 0.0), glm::dvec3(0.0, 0.0, 1.0));
  const talus::field::SampledSdf floor = floorField();
  World world;
  const uint32_t id = flowedLattice(world, shape, squash, glm::dvec3(0.0, 0.0427, 0.0));

  world.parameters.gravity = glm::dvec3(0.0, -9.81, 0.0);
  addFloor(world, floor, 0.5);

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(!world.contacts.empty());

  talus::dynamics::StepReport report;

  world.bodies.switchables.front().frames = world.parameters.switchFrames - 1u;
  talus::dynamics::switchTiers(world, report);

  CHECK(report.demotions == 0u);
  CHECK(world.carried.empty());

  talus::dynamics::demote(world, id);

  CHECK(world.carried.empty());
  CHECK(world.bodies.switchables.front().tier == talus::dynamics::Tier::Lattice);
}

TEST_CASE("keeps the volume of a dented thin plate when its dent is baked", "[strainstep]") {
  const glm::dvec3 half(0.6, 0.05, 0.6);
  const SoftShape shape = gridPlaneBox(half, 0.1, clay(1.0e6, 2.0e4));
  const double plate = 8.0 * half.x * half.y * half.z;

  REQUIRE(shape.mesh != nullptr);
  REQUIRE(shape.mesh->hexes.size() == 144u);
  REQUIRE(std::abs(cellVolume(*shape.mesh) - plate) <= 1.0e-6 * plate);

  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  talus::dynamics::promote(world, id);

  const std::optional<talus::dynamics::BodyRef> soft = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(soft.has_value());
  REQUIRE(soft->side == talus::dynamics::Side::Lattice);

  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[soft->index];
  const glm::dvec3 dentCentre(0.05, 0.0, 0.05);
  const glm::dmat3 squeeze(
    glm::dvec3(std::exp(0.1), 0.0, 0.0),
    glm::dvec3(0.0, std::exp(-0.2), 0.0),
    glm::dvec3(0.0, 0.0, std::exp(0.1))
  );

  lattice.plastic.assign(8u * shape.mesh->hexes.size(), glm::dmat3(1.0));
  lattice.hardening.assign(lattice.plastic.size(), 0.0);

  uint32_t dented = std::numeric_limits<uint32_t>::max();

  for (uint32_t cell = 0u; cell != shape.mesh->hexes.size(); ++cell) {
    glm::dvec3 centre(0.0);

    for (const uint32_t node : shape.mesh->hexes[cell].nodes) {
      centre += glm::dvec3(shape.mesh->restPositions[node]) / 8.0;
    }

    if (glm::length(centre - dentCentre) <= 1.0e-6) {
      dented = cell;
    }
  }

  REQUIRE(dented != std::numeric_limits<uint32_t>::max());

  for (uint32_t point = 0u; point != 8u; ++point) {
    lattice.plastic[8u * dented + point] = glm::inverse(squeeze);
  }

  REQUIRE(talus::dynamics::flowed(lattice));

  talus::dynamics::demote(world, id);

  const std::optional<talus::dynamics::BodyRef> after = talus::dynamics::findBody(world.bodies, id);
  const talus::dynamics::SwitchableBody& body = world.bodies.switchables.front();

  REQUIRE(after.has_value());
  REQUIRE(after->side == talus::dynamics::Side::Rigid);
  REQUIRE(body.baked != nullptr);

  const double kept = cellVolume(*body.lattice.mesh);

  CHECK(kept < plate);
  CHECK(std::abs(kept - plate) / plate <= talus::dynamics::SHAPE_TOLERANCE);
}






TEST_CASE("shares one inverse between elements with the same rest shape, material and masses", "[strainstep]") {
  const TestCell cell = testCell(0.1, 1.0);
  const talus::dynamics::StrainElement first = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const talus::dynamics::StrainElement twin = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const talus::dynamics::StrainElement softer = testElement(cell, 5.0e4, 0.3, 1.0 / 240.0);
  const talus::dynamics::StrainElement larger = testElement(testCell(0.11, 1.0), 1.0e5, 0.3, 1.0 / 240.0);
  const talus::dynamics::StrainElement slower = testElement(cell, 1.0e5, 0.3, 1.0 / 120.0);
  talus::dynamics::InversePool pool;

  REQUIRE(first.inverse != twin.inverse);
  REQUIRE(*first.inverse == *twin.inverse);

  const std::shared_ptr<const talus::dynamics::ElementInverse> shared = pool.intern(first.inverse);

  CHECK(pool.intern(twin.inverse) == shared);
  CHECK(pool.intern(softer.inverse) != shared);
  CHECK(pool.intern(larger.inverse) != shared);
  CHECK(pool.intern(slower.inverse) != shared);
  CHECK(pool.size() == 4u);
  CHECK(*shared == *twin.inverse);
}

TEST_CASE("finds a hex's rotation from the last substep's rotation to within a billionth of a radian", "[strainstep]") {
  std::mt19937 source(20261017u);
  const auto unit = [&] { return static_cast<double>(source()) / 4294967296.0; };
  const auto signedUnit = [&] { return 2.0 * unit() - 1.0; };
  const auto direction = [&] {
    return glm::normalize(glm::dvec3(signedUnit(), signedUnit(), signedUnit() + 1.0e-3));
  };
  const auto between = [](const glm::dquat& a, const glm::dquat& b) {
    const glm::dquat relative = b * glm::conjugate(a);

    return 2.0 * glm::length(glm::dvec3(relative.x, relative.y, relative.z));
  };
  double worst = 0.0;
  double reference = 0.0;

  for (uint32_t sample = 0u; sample != 4000u; ++sample) {
    const glm::dquat turn = glm::angleAxis(3.0 * unit(), direction());
    glm::dmat3 random(0.0);

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        random[column][row] = signedUnit();
      }
    }

    const glm::dmat3 stretch = glm::dmat3(1.0) + 0.05 * (random + glm::transpose(random));
    const glm::dmat3 deformation = glm::mat3_cast(turn) * stretch;
    const glm::dquat guess = glm::angleAxis(2.0e-3 * unit(), direction()) * turn;
    const glm::dquat found = talus::dynamics::rotationOf(deformation, guess);
    glm::dquat exact = found;

    for (uint32_t refine = 0u; refine != 64u; ++refine) {
      const glm::dquat next = talus::dynamics::rotationOf(deformation, exact);
      const bool settled = between(next, exact) < 1.0e-14;

      exact = next;

      if (settled) {
        break;
      }
    }

    worst = std::max(worst, between(found, exact));
    reference = std::max(reference, between(exact, turn));
  }

  CHECK(reference <= 1.0e-10);
  CHECK(worst <= 2.0e-9);
}

TEST_CASE("solves an element the same way whether its inverse is shared or its own", "[strainstep]") {
  const TestCell cell = testCell(0.1, 1.0);
  talus::dynamics::StrainElement own = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  talus::dynamics::StrainElement shared = own;
  talus::dynamics::InversePool pool;
  const talus::dynamics::StrainElement other = testElement(cell, 1.0e5, 0.3, 1.0 / 240.0);
  const glm::dmat3 shape(
    glm::dvec3(1.08, 0.03, -0.02),
    glm::dvec3(-0.05, 0.93, 0.04),
    glm::dvec3(0.02, -0.01, 1.11)
  );
  std::vector<glm::dvec3> positions(8u);
  std::vector<glm::dvec3> ownVelocities(8u);
  std::vector<glm::dvec3> sharedVelocities(8u);

  shared.inverse = pool.intern(other.inverse);
  REQUIRE(shared.inverse != own.inverse);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const double turn = static_cast<double>(corner);

    positions[corner] = shape * cell.rest[corner];
    ownVelocities[corner] = glm::dvec3(std::cos(turn), 0.5 * std::sin(2.0 * turn), -0.3 * turn);
    sharedVelocities[corner] = ownVelocities[corner];
  }

  talus::dynamics::updateElement(own, positions);
  talus::dynamics::updateElement(shared, positions);

  for (uint32_t pass = 0u; pass != 3u; ++pass) {
    talus::dynamics::solveElement(own, ownVelocities);
    talus::dynamics::solveElement(shared, sharedVelocities);
  }

  CHECK(own.impulse == shared.impulse);
  CHECK(own.volumeImpulse == shared.volumeImpulse);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    CHECK(ownVelocities[corner] == sharedVelocities[corner]);
  }
}

TEST_CASE("holds as many inverses for four identical bodies as for one", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  size_t alone = 0u;

  for (const uint32_t bodies : {1u, 4u}) {
    World world;

    useStrainstep(world, glm::dvec3(0.0));

    for (uint32_t body = 0u; body != bodies; ++body) {
      (void)addSoft(world, shape, glm::dvec3(3.0 * static_cast<double>(body), 0.0, 0.0));
    }

    REQUIRE(talus::dynamics::step(world).strainstepIslands == bodies);
    REQUIRE(world.strainstepArena != nullptr);

    const size_t held = talus::dynamics::strainstepStats(*world.strainstepArena).inverses;

    CAPTURE(bodies);
    CHECK(held > 0u);
    CHECK(held <= shape.mesh->hexes.size());

    if (bodies == 1u) {
      alone = held;
    } else {
      CHECK(held == alone);
    }
  }
}

TEST_CASE("keeps built elements by key and drops the least recently used when its cells run out", "[strainstep]") {
  talus::dynamics::ElementCache cache(100u);
  const auto built = [](size_t cells, double lowest) {
    talus::dynamics::BuiltElements entry;

    entry.lowest = lowest;
    entry.elements.resize(cells);

    return entry;
  };

  cache.store("a", built(40u, 1.0));
  cache.store("b", built(40u, 2.0));

  REQUIRE(cache.find("a") != nullptr);

  cache.store("c", built(40u, 3.0));

  CHECK(cache.find("b") == nullptr);
  REQUIRE(cache.find("a") != nullptr);
  CHECK(cache.find("a")->lowest == 1.0);
  REQUIRE(cache.find("c") != nullptr);
  CHECK(cache.find("c")->lowest == 3.0);
  CHECK(cache.cells() == 80u);
  CHECK(cache.size() == 2u);
}

TEST_CASE("keeps no entry larger than its cells and replaces an entry stored again", "[strainstep]") {
  talus::dynamics::ElementCache cache(50u);
  talus::dynamics::BuiltElements big;
  talus::dynamics::BuiltElements small;
  talus::dynamics::BuiltElements larger;

  big.elements.resize(51u);
  small.elements.resize(20u);
  larger.elements.resize(30u);
  larger.lowest = 2.0;

  cache.store("big", big);
  CHECK(cache.find("big") == nullptr);
  CHECK(cache.cells() == 0u);

  cache.store("x", small);
  cache.store("x", larger);
  REQUIRE(cache.find("x") != nullptr);
  CHECK(cache.find("x")->lowest == 2.0);
  CHECK(cache.cells() == 30u);
  CHECK(cache.size() == 1u);
}

TEST_CASE("builds the elements of bodies with the same cells, materials and substeps once", "[strainstep]") {
  const std::array<SoftShape, 3> shapes{
    softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3)),
    softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3)),
    softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3))
  };
  World world;

  useStrainstep(world, glm::dvec3(0.0));
  REQUIRE(shapes[0].mesh != shapes[1].mesh);
  REQUIRE(shapes[1].mesh != shapes[2].mesh);

  for (uint32_t body = 0u; body != shapes.size(); ++body) {
    (void)addSoft(world, shapes[body], glm::dvec3(3.0 * static_cast<double>(body), 0.0, 0.0));
  }

  REQUIRE(talus::dynamics::step(world).strainstepIslands == 3u);

  const talus::dynamics::StrainstepStats stats = talus::dynamics::strainstepStats(*world.strainstepArena);

  CHECK(stats.builds == 1u);
  CHECK(stats.reuses == 2u);
  CHECK(stats.cachedCells == shapes[0].mesh->hexes.size());
}

TEST_CASE("builds the elements again for a body of another material or substep length", "[strainstep]") {
  const SoftShape soft = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  const SoftShape twin = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  const SoftShape stiffer = softBox(glm::dvec3(0.3), 0.2, jelly(1.6e4));
  const SoftShape finer = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  World world;

  useStrainstep(world, glm::dvec3(0.0));
  (void)addSoft(world, soft, glm::dvec3(0.0, 0.0, 0.0));
  (void)addSoft(world, twin, glm::dvec3(3.0, 0.0, 0.0));
  (void)addSoft(world, stiffer, glm::dvec3(6.0, 0.0, 0.0));
  world.bodies.lattices[addSoft(world, finer, glm::dvec3(9.0, 0.0, 0.0))].substeps = 8u;

  REQUIRE(talus::dynamics::step(world).strainstepIslands == 4u);

  const talus::dynamics::StrainstepStats stats = talus::dynamics::strainstepStats(*world.strainstepArena);

  CHECK(stats.builds == 3u);
  CHECK(stats.reuses == 1u);
}

TEST_CASE("builds a body that turns soft again from the elements its first time left", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.1, jelly(2.0e5, 500.0));
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, shape.mesh, shape.lanes, shape.field, glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  talus::dynamics::promote(world, id);
  REQUIRE(!talus::dynamics::step(world).nonFinite);
  talus::dynamics::demote(world, id);
  REQUIRE(!talus::dynamics::step(world).nonFinite);
  REQUIRE(world.bodies.lattices.empty());
  talus::dynamics::promote(world, id);
  REQUIRE(!talus::dynamics::step(world).nonFinite);
  REQUIRE(world.bodies.lattices.size() == 1u);

  const talus::dynamics::StrainstepStats stats = talus::dynamics::strainstepStats(*world.strainstepArena);

  CHECK(stats.builds == 1u);
  CHECK(stats.reuses == 1u);
}

TEST_CASE("solves a body built from the cache as it solves the same body built alone", "[strainstep]") {
  const glm::dmat3 shear(glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.12, 1.0, 0.0), glm::dvec3(-0.05, 0.08, 1.0));
  const glm::dmat3 turn = glm::mat3_cast(glm::angleAxis(0.4, glm::dvec3(0.0, 1.0, 0.0)));
  const SoftShape first = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  const SoftShape second = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  const SoftShape firstAlone = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  const SoftShape secondAlone = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));
  const auto strain = [](talus::dynamics::LatticeBody& body, const glm::dmat3& by) {
    glm::dvec3 centre(0.0);

    for (const glm::dvec3& position : body.position) {
      centre += position;
    }

    centre /= static_cast<double>(body.position.size());

    for (glm::dvec3& position : body.position) {
      position = centre + by * (position - centre);
    }
  };
  World together;
  World apartFirst;
  World apartSecond;

  for (World* world : {&together, &apartFirst, &apartSecond}) {
    useStrainstep(*world, glm::dvec3(0.0));
  }

  const uint32_t one = addSoft(together, first, glm::dvec3(0.0, 0.0, 0.0));
  const uint32_t two = addSoft(together, second, glm::dvec3(4.0, 0.0, 0.0));
  const uint32_t oneAlone = addSoft(apartFirst, firstAlone, glm::dvec3(0.0, 0.0, 0.0));
  const uint32_t twoAlone = addSoft(apartSecond, secondAlone, glm::dvec3(4.0, 0.0, 0.0));

  strain(together.bodies.lattices[one], turn * shear);
  strain(together.bodies.lattices[two], shear);
  strain(apartFirst.bodies.lattices[oneAlone], turn * shear);
  strain(apartSecond.bodies.lattices[twoAlone], shear);

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(together).nonFinite);
    REQUIRE(!talus::dynamics::step(apartFirst).nonFinite);
    REQUIRE(!talus::dynamics::step(apartSecond).nonFinite);
  }

  REQUIRE(talus::dynamics::strainstepStats(*together.strainstepArena).reuses == 1u);
  REQUIRE(talus::dynamics::strainstepStats(*apartFirst.strainstepArena).reuses == 0u);
  REQUIRE(talus::dynamics::strainstepStats(*apartSecond.strainstepArena).reuses == 0u);
  CHECK(together.bodies.lattices[one].position == apartFirst.bodies.lattices[oneAlone].position);
  CHECK(together.bodies.lattices[one].velocity == apartFirst.bodies.lattices[oneAlone].velocity);
  CHECK(together.bodies.lattices[two].position == apartSecond.bodies.lattices[twoAlone].position);
  CHECK(together.bodies.lattices[two].velocity == apartSecond.bodies.lattices[twoAlone].velocity);
}

TEST_CASE("solves the elements of the bodies in one island on workers and lands where the serial solve lands", "[strainstep]") {
  constexpr uint32_t BODIES = 12u;
  const talus::field::SampledSdf floor = floorField();
  const SoftShape shape = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));

  struct Outcome {
    std::vector<std::vector<glm::dvec3>> positions;
    std::vector<std::vector<glm::dvec3>> velocities;
    uint32_t batches = 0u;
    uint32_t islands = 0u;
  };

  const auto drop = [&](uint32_t workers) {
    World world;
    Outcome outcome;

    useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
    world.workerCount = workers;
    addFloor(world, floor, 0.5);

    for (uint32_t body = 0u; body != BODIES; ++body) {
      (void)addSoft(world, shape, glm::dvec3(0.605 * static_cast<double>(body), 0.31, 0.0));
    }

    for (uint32_t frame = 0u; frame != 40u; ++frame) {
      const talus::dynamics::StepReport report = talus::dynamics::step(world);

      REQUIRE(!report.nonFinite);
      outcome.islands = std::max(outcome.islands, report.strainstepIslands);
    }

    for (const talus::dynamics::LatticeBody& lattice : world.bodies.lattices) {
      outcome.positions.push_back(lattice.position);
      outcome.velocities.push_back(lattice.velocity);
    }

    outcome.batches = talus::dynamics::strainstepStats(*world.strainstepArena).parallelBatches;

    return outcome;
  };

  REQUIRE(BODIES * shape.mesh->hexes.size() >= talus::dynamics::PARALLEL_CELLS);

  const Outcome serial = drop(1u);
  const Outcome parallel = drop(4u);

  REQUIRE(serial.islands == 1u);
  REQUIRE(parallel.islands == 1u);
  CHECK(serial.batches == 0u);
  CHECK(parallel.batches > 0u);
  CHECK(parallel.positions == serial.positions);
  CHECK(parallel.velocities == serial.velocities);
}

TEST_CASE("keeps the elements of a lone body or a small island on one thread", "[strainstep]") {
  const SoftShape big = softBox(glm::dvec3(0.9), 0.2, jelly(8.0e3));
  const SoftShape small = softBox(glm::dvec3(0.3), 0.2, jelly(8.0e3));

  REQUIRE(big.mesh->hexes.size() >= talus::dynamics::PARALLEL_CELLS);
  REQUIRE(2u * small.mesh->hexes.size() < talus::dynamics::PARALLEL_CELLS);

  for (const bool lone : {true, false}) {
    World world;

    useStrainstep(world, glm::dvec3(0.0));
    world.workerCount = 4u;

    if (lone) {
      (void)addSoft(world, big, glm::dvec3(0.0));
    } else {
      (void)addSoft(world, small, glm::dvec3(0.0));
      (void)addSoft(world, small, glm::dvec3(0.605, 0.0, 0.0));
    }

    for (uint32_t frame = 0u; frame != 3u; ++frame) {
      REQUIRE(!talus::dynamics::step(world).nonFinite);
    }

    CAPTURE(lone);
    CHECK(talus::dynamics::strainstepStats(*world.strainstepArena).parallelBatches == 0u);
  }
}

namespace {

using talus::deformable::DeformedDistance;

struct ProbeSweep {
  size_t contacts = 0u;
  size_t missing = 0u;
  size_t changed = 0u;
  size_t added = 0u;
  double fullPad = 0.0;
  double sizedPad = 0.0;
};

struct SweptFace {
  glm::dvec3 centre{0.0};
  glm::dvec3 normal{1.0, 0.0, 0.0};
  glm::dvec3 across{0.0, 1.0, 0.0};
  glm::dvec3 up{0.0, 0.0, 1.0};
  double halfAcross = 0.0;
  double halfUp = 0.0;
  double depth = 0.0;
};

SoftShape softBall(double radius, double voxel, const talus::PhysicalMaterial& material) {
  SoftShape shape;

  shape.field = ballField(radius, voxel);
  shape.lanes.assign(2u, material);
  shape.mesh = talus::deformable::buildMesh(
    shape.field,
    shape.lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );

  return shape;
}

SoftShape softPlate(const glm::dvec3& half, const glm::dvec3& voxel, const talus::PhysicalMaterial& material) {
  SoftShape shape;
  const glm::vec3 reach = glm::vec3(half) + glm::vec3(2.0 * voxel);

  shape.field = talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(glm::vec3(0.0f), glm::vec3(half) + glm::vec3(1.0e-4f))},
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / glm::vec3(voxel))) + glm::uvec3(1u)
  );
  shape.lanes.assign(2u, material);
  shape.mesh = talus::deformable::buildMesh(
    shape.field,
    shape.lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );

  return shape;
}

bool isContact(const DeformedDistance& hit, double reach) {
  return hit.found && std::isfinite(hit.distance) && hit.distance < reach && glm::length(hit.normal) > 0.5;
}

bool sameHit(const DeformedDistance& a, const DeformedDistance& b) {
  return a.band == b.band && a.element == b.element && a.distance == b.distance && a.normal == b.normal
         && a.nodes == b.nodes && a.weights == b.weights;
}

ProbeSweep sweepProbes(
  const talus::PreparedMesh& mesh,
  const talus::field::SampledSdf& field,
  std::span<const glm::dvec3> placed,
  std::span<const glm::dvec3> queries,
  double reach
) {
  talus::deformable::DeformedProbe full = talus::deformable::deformedProbe(mesh, field);
  talus::deformable::DeformedProbe sized = talus::deformable::deformedProbe(mesh, field, reach);

  talus::deformable::placeDeformedProbe(full, placed);
  talus::deformable::placeDeformedProbe(sized, placed);

  ProbeSweep sweep;

  sweep.fullPad = full.pad;
  sweep.sizedPad = sized.pad;

  for (const glm::dvec3& query : queries) {
    const DeformedDistance wide = talus::deformable::probeDeformed(full, query);
    const DeformedDistance narrow = talus::deformable::probeDeformed(sized, query);

    if (isContact(wide, reach)) {
      ++sweep.contacts;
      sweep.missing += isContact(narrow, reach) ? 0u : 1u;
      sweep.changed += isContact(narrow, reach) && !sameHit(wide, narrow) ? 1u : 0u;
    } else if (isContact(narrow, reach)) {
      ++sweep.added;
    }
  }

  return sweep;
}

void sweepFace(const SweptFace& face, double reach, const glm::dmat3& turn, const glm::dvec3& shift, std::vector<glm::dvec3>& queries) {
  constexpr double RIM = 0.1;
  constexpr double BEHIND = 0.1;
  constexpr uint32_t SAMPLES = 13u;
  const uint32_t steps = static_cast<uint32_t>(std::ceil((face.depth + BEHIND) / (reach / 3.0)));

  for (uint32_t i = 0u; i != SAMPLES; ++i) {
    for (uint32_t j = 0u; j != SAMPLES; ++j) {
      for (uint32_t k = 0u; k <= steps; ++k) {
        const double u = (face.halfAcross + RIM) * (2.0 * i / (SAMPLES - 1u) - 1.0);
        const double v = (face.halfUp + RIM) * (2.0 * j / (SAMPLES - 1u) - 1.0);
        const double t = -BEHIND + (face.depth + BEHIND) * k / steps;

        queries.push_back(turn * (face.centre + u * face.across + v * face.up + t * face.normal) + shift);
      }
    }
  }
}

void sweepEnds(
  const glm::dvec3& low,
  const glm::dvec3& high,
  double halfAcross,
  double halfUp,
  double depth,
  double reach,
  const glm::dmat3& turn,
  const glm::dvec3& shift,
  std::vector<glm::dvec3>& queries
) {
  sweepFace({low, glm::dvec3(-1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), halfAcross, halfUp, depth}, reach, turn, shift, queries);
  sweepFace({high, glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), halfAcross, halfUp, depth}, reach, turn, shift, queries);
}

void sweepSides(const glm::dvec3& half, double stretch, double reach, const glm::dmat3& turn, const glm::dvec3& shift, std::vector<glm::dvec3>& queries) {
  const double depth = 1.4 * reach;

  for (const double side : {-1.0, 1.0}) {
    sweepFace({glm::dvec3(0.0, side * half.y, 0.0), glm::dvec3(0.0, side, 0.0), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), half.x * stretch, half.z, depth}, reach, turn, shift, queries);
    sweepFace({glm::dvec3(0.0, 0.0, side * half.z), glm::dvec3(0.0, 0.0, side), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), half.x * stretch, half.y, depth}, reach, turn, shift, queries);
  }
}

glm::dmat3 turnedBy(double angle) {
  return glm::mat3_cast(glm::angleAxis(angle, glm::normalize(glm::dvec3(1.0, 2.0, 3.0))));
}

ProbeSweep stripSweep(double stretch, const glm::dmat3& turn, const glm::dvec3& shift, double reach) {
  const glm::dvec3 half(0.6, 0.2, 0.2);
  const SoftShape shape = softBox(half, 0.2, jelly(1.0e4));
  std::vector<glm::dvec3> placed;
  std::vector<glm::dvec3> queries;

  for (const glm::vec3& rest : shape.mesh->restPositions) {
    placed.push_back(turn * glm::dvec3(stretch * rest.x, rest.y, rest.z) + shift);
  }

  sweepEnds(glm::dvec3(-half.x * stretch, 0.0, 0.0), glm::dvec3(half.x * stretch, 0.0, 0.0), half.y, half.z, 1.4 * stretch * reach, reach, turn, shift, queries);
  sweepSides(half, stretch, reach, turn, shift, queries);

  return sweepProbes(*shape.mesh, shape.field, placed, queries, reach);
}

ProbeSweep tornStripSweep(double stretch, double gap, const glm::dmat3& turn, const glm::dvec3& shift, double reach) {
  const glm::dvec3 half(1.0, 0.25, 0.25);
  const SoftShape shape = softBox(half, 0.5, tearable());
  World world;

  useStrainstep(world, glm::dvec3(0.0));

  const uint32_t index = addSoft(world, shape, glm::dvec3(0.0));
  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[index];

  for (glm::dvec3& position : lattice.position) {
    position.x *= 1.01;
  }

  REQUIRE(talus::dynamics::tearCells(lattice, std::array<uint32_t, 1>{1u}) == 4u);

  const talus::PreparedMesh& mesh = *lattice.mesh;
  std::vector<uint8_t> right(lattice.position.size(), 0u);

  for (const talus::PreparedHex& hex : mesh.hexes) {
    double centre = 0.0;

    for (const uint32_t node : hex.nodes) {
      centre += mesh.restPositions[node].x / 8.0;
    }

    for (const uint32_t node : hex.nodes) {
      right[node] = centre > 0.0 ? 1u : right[node];
    }
  }

  std::vector<glm::dvec3> placed;
  std::vector<glm::dvec3> queries;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::vec3 rest = mesh.restPositions[node];

    placed.push_back(turn * glm::dvec3(stretch * rest.x + (right[node] != 0u ? gap : 0.0), rest.y, rest.z) + shift);
  }

  const double depth = 1.4 * stretch * reach;

  sweepEnds(glm::dvec3(-half.x * stretch, 0.0, 0.0), glm::dvec3(half.x * stretch + gap, 0.0, 0.0), half.y, half.z, depth, reach, turn, shift, queries);
  sweepEnds(glm::dvec3(gap, 0.0, 0.0), glm::dvec3(0.0, 0.0, 0.0), half.y, half.z, depth, reach, turn, shift, queries);

  return sweepProbes(mesh, shape.field, placed, queries, reach);
}

ProbeSweep plateSweep(const glm::dmat3& turn, const glm::dvec3& shift, double reach) {
  const glm::dvec3 half(0.6, 0.04, 0.4);
  const SoftShape shape = softPlate(half, glm::dvec3(0.2, 0.02, 0.2), jelly(1.0e4));
  std::vector<glm::dvec3> placed;
  std::vector<glm::dvec3> queries;

  for (const glm::vec3& rest : shape.mesh->restPositions) {
    placed.push_back(turn * glm::dvec3(rest) + shift);
  }

  sweepFace({glm::dvec3(0.0, half.y, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), half.x, half.z, 1.4 * reach}, reach, turn, shift, queries);
  sweepFace({glm::dvec3(0.0, -half.y, 0.0), glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), half.x, half.z, 1.4 * reach}, reach, turn, shift, queries);
  sweepFace({glm::dvec3(half.x, 0.0, 0.0), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0), half.y, half.z, 1.4 * reach}, reach, turn, shift, queries);

  return sweepProbes(*shape.mesh, shape.field, placed, queries, reach);
}

ProbeSweep ballSweep(const glm::dmat3& map, const glm::dvec3& shift, double reach) {
  constexpr double RADIUS = 0.3;
  constexpr uint32_t DIRECTIONS = 900u;
  const SoftShape shape = softBall(RADIUS, 0.1, jelly(1.0e4));
  std::vector<glm::dvec3> placed;
  std::vector<glm::dvec3> queries;

  for (const glm::vec3& rest : shape.mesh->restPositions) {
    placed.push_back(map * glm::dvec3(rest) + shift);
  }

  for (uint32_t direction = 0u; direction != DIRECTIONS; ++direction) {
    const double height = 1.0 - 2.0 * (direction + 0.5) / DIRECTIONS;
    const double ring = std::sqrt(1.0 - height * height);
    const double angle = direction * std::numbers::pi * (3.0 - std::sqrt(5.0));
    const glm::dvec3 unit(ring * std::cos(angle), height, ring * std::sin(angle));

    for (uint32_t step = 0u; step <= 40u; ++step) {
      queries.push_back(map * (unit * (RADIUS - 0.15 + 0.01 * step)) + shift);
    }
  }

  return sweepProbes(*shape.mesh, shape.field, placed, queries, reach);
}

}

TEST_CASE("sizes a deformed probe's pad from the contact reach, between a fifth of a cell and the full pad", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.6, 0.2, 0.2), 0.2, jelly(1.0e4));
  std::vector<glm::dvec3> rest;

  for (const glm::vec3& at : shape.mesh->restPositions) {
    rest.push_back(glm::dvec3(at));
  }

  const auto probeFor = [&](double reach) {
    talus::deformable::DeformedProbe probe = talus::deformable::deformedProbe(*shape.mesh, shape.field, reach);

    talus::deformable::placeDeformedProbe(probe, rest);

    return probe;
  };
  const double cell = probeFor(0.0).cell;

  CHECK(probeFor(0.0).padCells == talus::deformable::DEFORMED_PAD_FLOOR);
  CHECK(probeFor(0.02).padCells == 3.0 * 0.02 / cell);
  CHECK(probeFor(0.06).padCells == 3.0 * 0.06 / cell);
  CHECK(probeFor(0.15).padCells == talus::deformable::DEFORMED_PAD);
  CHECK(probeFor(10.0).padCells == talus::deformable::DEFORMED_PAD);
  CHECK(probeFor(std::numeric_limits<double>::infinity()).padCells == talus::deformable::DEFORMED_PAD);
  CHECK(probeFor(std::numeric_limits<double>::quiet_NaN()).padCells == talus::deformable::DEFORMED_PAD);
  CHECK(talus::deformable::deformedProbe(*shape.mesh, shape.field).padCells == talus::deformable::DEFORMED_PAD);
  CHECK(std::abs(probeFor(0.02).pad - 3.0 * 0.02) <= 1.0e-6 * 3.0 * 0.02);
  CHECK(probeFor(10.0).pad == talus::deformable::DEFORMED_PAD * cell);

  const auto padWhenStretched = [&](double stretch) {
    std::vector<glm::dvec3> pulled = rest;

    for (glm::dvec3& at : pulled) {
      at.x *= stretch;
    }

    talus::deformable::DeformedProbe probe = talus::deformable::deformedProbe(*shape.mesh, shape.field, 0.02);

    talus::deformable::placeDeformedProbe(probe, pulled);

    return probe.pad;
  };
  const double resting = padWhenStretched(1.0);

  CHECK(std::abs(resting - 3.0 * 0.02) <= 1.0e-6 * resting);
  CHECK(std::abs(padWhenStretched(2.0) - 2.0 * resting) <= 1.0e-6 * resting);
  CHECK(std::abs(padWhenStretched(0.5) - resting) <= 1.0e-6 * resting);
  CHECK(padWhenStretched(50.0) == talus::deformable::DEFORMED_PAD * cell);
}

TEST_CASE("keeps every contact the full pad finds in a strip stretched along its length", "[strainstep]") {
  const glm::dvec3 shift(0.3, -1.0, 0.7);

  for (const double stretch : {1.0, 3.0, 8.0}) {
    for (const bool turned : {false, true}) {
      const ProbeSweep sweep = stripSweep(stretch, turned ? turnedBy(0.6) : glm::dmat3(1.0), turned ? shift : glm::dvec3(0.0), 0.03);

      CAPTURE(stretch, turned);
      REQUIRE(sweep.contacts > 200u);
      CHECK(sweep.missing == 0u);
      CHECK(sweep.changed == 0u);
      CHECK(sweep.added == 0u);
      CHECK(sweep.sizedPad <= sweep.fullPad);

      if (stretch == 1.0) {
        CHECK(sweep.sizedPad < 0.5 * sweep.fullPad);
      }
    }
  }
}

TEST_CASE("keeps every contact the full pad finds in a torn strip pulled open", "[strainstep]") {
  for (const double stretch : {1.0, 4.0}) {
    for (const double gap : {0.1, 0.6}) {
      const ProbeSweep sweep = tornStripSweep(stretch, gap, turnedBy(0.4), glm::dvec3(-0.2, 0.5, 0.1), 0.03);

      CAPTURE(stretch, gap);
      REQUIRE(sweep.contacts > 200u);
      CHECK(sweep.missing == 0u);
      CHECK(sweep.changed == 0u);
      CHECK(sweep.added == 0u);
    }
  }
}

TEST_CASE("keeps every contact the full pad finds on a curved lattice sheared and stretched at an angle", "[strainstep]") {
  const glm::dmat3 shear(glm::dvec3(1.8, 0.0, 0.0), glm::dvec3(0.4, 1.0, 0.0), glm::dvec3(0.0, 0.0, 0.5));

  for (const double reach : {0.012, 0.03}) {
    const ProbeSweep sweep = ballSweep(turnedBy(0.7) * shear, glm::dvec3(0.1, 2.0, -0.3), reach);

    CAPTURE(reach);
    REQUIRE(sweep.contacts > 200u);
    CHECK(sweep.missing == 0u);
    CHECK(sweep.changed == 0u);
    CHECK(sweep.added == 0u);
  }
}

TEST_CASE("keeps every contact the full pad finds on a plate whose cells are ten times thinner than wide", "[strainstep]") {
  for (const double reach : {0.012, 0.03}) {
    for (const bool turned : {false, true}) {
      const ProbeSweep sweep = plateSweep(turned ? turnedBy(0.5) : glm::dmat3(1.0), turned ? glm::dvec3(0.2, 0.1, -0.4) : glm::dvec3(0.0), reach);

      CAPTURE(reach, turned);
      REQUIRE(sweep.contacts > 200u);
      CHECK(sweep.missing == 0u);
      CHECK(sweep.changed == 0u);
      CHECK(sweep.added == 0u);
    }
  }
}

TEST_CASE("finds the lattice contacts the full probe pad finds, at the reach of each pair", "[strainstep]") {
  struct Meeting {
    double stretch;
    glm::dvec3 offset;
    double travel;
  };

  const SoftShape strip = softBox(glm::dvec3(0.6, 0.2, 0.2), 0.2, jelly(1.0e4));
  const SoftShape block = softBox(glm::dvec3(0.2), 0.2, jelly(1.0e4));
  const std::array<Meeting, 4> meetings{
    Meeting{1.0, glm::dvec3(0.81, 0.0, 0.0), 0.0},
    Meeting{4.0, glm::dvec3(2.65, 0.0, 0.0), 0.0},
    Meeting{1.0, glm::dvec3(0.0, 0.55, 0.0), 0.25},
    Meeting{4.0, glm::dvec3(2.8, 0.0, 0.0), 0.25}
  };

  for (const Meeting& meeting : meetings) {
    World world;

    useStrainstep(world, glm::dvec3(0.0));

    const uint32_t first = addSoft(world, strip, glm::dvec3(0.0));
    const uint32_t second = addSoft(world, block, meeting.offset);

    for (glm::dvec3& position : world.bodies.lattices[first].position) {
      position.x *= meeting.stretch;
    }

    std::vector<double> travel(2u, 0.0);
    std::vector<talus::deformable::DeformedProbe> full;

    travel[second] = meeting.travel;

    for (const talus::dynamics::LatticeBody& lattice : world.bodies.lattices) {
      full.push_back(talus::deformable::deformedProbe(*lattice.mesh, *lattice.field));
      talus::deformable::placeDeformedProbe(full.back(), lattice.position);
    }

    const std::array<talus::dynamics::BodyPair, 1> pairs{talus::dynamics::BodyPair{
      talus::dynamics::BodyRef{talus::dynamics::Side::Lattice, first},
      talus::dynamics::BodyRef{talus::dynamics::Side::Lattice, second}
    }};
    const double band = world.parameters.speculativeDistance + world.parameters.margin;
    const double reach = band + travel[first] + travel[second];
    const std::vector<talus::dynamics::Contact> found =
      talus::dynamics::narrowphase(world.bodies, pairs, band, travel);
    size_t expected = 0u;

    for (const uint32_t source : {first, second}) {
      const uint32_t target = source == first ? second : first;
      const talus::dynamics::LatticeBody& body = world.bodies.lattices[source];

      REQUIRE(body.mesh->surfaceSeeds.empty());

      for (const uint32_t node : body.mesh->surfaceNodes) {
        const DeformedDistance hit = talus::deformable::probeDeformed(full[target], body.position[node]);

        if (!isContact(hit, reach)) {
          continue;
        }

        const auto contact = std::find_if(found.begin(), found.end(), [&](const talus::dynamics::Contact& candidate) {
          return candidate.a.body == source && candidate.a.nodes[0] == node && candidate.b.body == target;
        });

        ++expected;
        CAPTURE(meeting.stretch, meeting.travel, source, node);
        REQUIRE(contact != found.end());
        CHECK(contact->b.nodes == hit.nodes);
        CHECK(contact->b.weights == hit.weights);
        CHECK(contact->gap == hit.distance);
        CHECK(contact->basis[0] == glm::normalize(hit.normal));
      }
    }

    CAPTURE(meeting.stretch, meeting.travel);
    CHECK(expected > 8u);
    CHECK(found.size() == expected);
  }
}

TEST_CASE("splits each island once per build, however many lattices it holds", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.1), 0.1, jelly(1.0e4));
  talus::dynamics::BodySet bodies;
  const talus::dynamics::ManifoldCache cache;
  talus::dynamics::IslandStore store;
  std::vector<talus::dynamics::Island> islands;

  for (uint32_t index = 0u; index != 12u; ++index) {
    (void)talus::dynamics::addLattice(
      bodies, shape.mesh, shape.lanes, glm::dvec3(0.5 * index, 0.0, 0.0), glm::dvec3(0.0), 0.5, &shape.field
    );
  }

  const auto touching = [](std::initializer_list<uint32_t> firsts) {
    std::vector<talus::dynamics::Contact> contacts;

    for (const uint32_t first : firsts) {
      talus::dynamics::Contact contact;

      contact.a.side = talus::dynamics::Side::Lattice;
      contact.a.body = first;
      contact.b.side = talus::dynamics::Side::Lattice;
      contact.b.body = first + 1u;
      contacts.push_back(contact);
    }

    return contacts;
  };
  const auto island = [&](uint32_t lattice) { return store.islandOf(bodies.lattices[lattice].id); };

  store.sync(bodies);

  const std::vector<talus::dynamics::Contact> chains = touching({0u, 1u, 2u, 4u, 5u, 6u});

  store.build(
    bodies,
    cache,
    {},
    false,
    chains,
    islands,
    talus::dynamics::JointSet{}
  );

  CHECK(store.splits() == 2u);
  CHECK(islands.size() == 6u);
  CHECK(island(0u) == island(3u));
  CHECK(island(4u) == island(7u));
  CHECK(island(0u) != island(4u));
  CHECK(island(8u) != island(9u));

  const std::vector<talus::dynamics::Contact> broken = touching({0u, 2u, 4u, 5u, 6u});

  store.build(
    bodies,
    cache,
    {},
    false,
    broken,
    islands,
    talus::dynamics::JointSet{}
  );

  CHECK(store.splits() == 2u);
  CHECK(islands.size() == 7u);
  CHECK(island(0u) == island(1u));
  CHECK(island(2u) == island(3u));
  CHECK(island(1u) != island(2u));
  CHECK(island(4u) == island(7u));
}

TEST_CASE("lists a body's contacts alike with a pair set built once and with one built per call", "[strainstep]") {
  const talus::field::SampledSdf floor = floorField();
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.1), 0.05);
  World world;

  useStrainstep(world, glm::dvec3(0.0, -9.81, 0.0));
  addFloor(world, floor, 0.5);

  const uint32_t lower = talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.1), 1.0, glm::dvec3(0.0, 0.1, 0.0), glm::dvec3(0.0), 0.5, 3u);
  const uint32_t upper = talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.1), 1.0, glm::dvec3(0.0, 0.31, 0.0), glm::dvec3(0.0), 0.5, 3u);

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::BodyRef lowerRef{talus::dynamics::Side::Rigid, lower};
  const talus::dynamics::BodyRef upperRef{talus::dynamics::Side::Rigid, upper};
  const uint64_t stack = talus::dynamics::pairKey(world.bodies, lowerRef, upperRef);
  const auto contactsOnStack = [&](const std::vector<talus::dynamics::Contact>& contacts) {
    return std::count_if(contacts.begin(), contacts.end(), [&](const talus::dynamics::Contact& contact) { return contact.pair == stack; });
  };

  REQUIRE(contactsOnStack(talus::dynamics::contactsOf(world, lowerRef)) > 0);

  talus::dynamics::Contact recorded;

  recorded.a.side = talus::dynamics::Side::Rigid;
  recorded.a.body = lower;
  recorded.b.side = talus::dynamics::Side::Rigid;
  recorded.b.body = upper;
  recorded.pair = stack;
  world.contacts.push_back(recorded);

  const std::unordered_set<uint64_t> pairs = talus::dynamics::recordedPairs(world.contacts);

  for (const talus::dynamics::BodyRef& ref : {lowerRef, upperRef}) {
    const std::vector<talus::dynamics::Contact> each = talus::dynamics::contactsOf(world, ref);
    const std::vector<talus::dynamics::Contact> once = talus::dynamics::contactsOf(world, ref, pairs);

    REQUIRE(once.size() == each.size());
    CHECK(contactsOnStack(once) == 1);

    for (size_t index = 0u; index != each.size(); ++index) {
      CHECK(once[index].key == each[index].key);
      CHECK(once[index].pair == each[index].pair);
      CHECK(once[index].gap == each[index].gap);
    }
  }
}

namespace {

std::array<uint32_t, 4> hexFace(const talus::PreparedHex& hex, uint32_t axis, uint32_t side) {
  std::array<uint32_t, 4> face{};
  uint32_t count = 0u;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    if (((corner >> axis) & 1u) == side) {
      face[count++] = hex.nodes[corner];
    }
  }

  std::sort(face.begin(), face.end());

  return face;
}

}

TEST_CASE("flags only a box's outer faces as boundary in the deformed probe when its cells sit half a cell off the origin", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.4), 0.8 / 3.0, jelly(1.0e4));
  const talus::PreparedMesh& mesh = *shape.mesh;
  const talus::deformable::DeformedProbe probe = talus::deformable::deformedProbe(mesh, shape.field);
  std::map<std::array<uint32_t, 4>, uint32_t> users;

  for (const talus::PreparedHex& hex : mesh.hexes) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      for (uint32_t side = 0u; side != 2u; ++side) {
        ++users[hexFace(hex, axis, side)];
      }
    }
  }

  uint32_t outer = 0u;
  uint32_t wrong = 0u;

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      for (uint32_t side = 0u; side != 2u; ++side) {
        const bool alone = users[hexFace(mesh.hexes[element], axis, side)] == 1u;
        const bool flagged = (probe.boundary[element] & (1u << (2u * axis + side))) != 0u;

        outer += alone ? 1u : 0u;
        wrong += alone != flagged ? 1u : 0u;
      }
    }
  }

  REQUIRE(mesh.hexes.size() == 27u);
  CHECK(outer == 54u);
  CHECK(wrong == 0u);
}

TEST_CASE("reads a point just above a box whose top cells are squashed as outside the box", "[strainstep]") {
  const SoftShape shape = softBox(glm::dvec3(0.4), 0.8 / 3.0, jelly(1.0e4));
  const talus::PreparedMesh& mesh = *shape.mesh;
  const double voxel = static_cast<double>(mesh.voxelSize.y);
  double top = -std::numeric_limits<double>::infinity();

  for (const glm::vec3& rest : mesh.restPositions) {
    top = std::max(top, static_cast<double>(rest.y));
  }

  const double floorOfTop = top - voxel;
  const double squashed = floorOfTop + 0.05 * voxel;
  std::vector<glm::dvec3> placed;

  for (const glm::vec3& rest : mesh.restPositions) {
    glm::dvec3 at(rest);

    if (at.y > floorOfTop + 0.5 * voxel) {
      at.y = squashed;
    }

    placed.push_back(at);
  }

  talus::deformable::DeformedProbe probe = talus::deformable::deformedProbe(mesh, shape.field);

  talus::deformable::placeDeformedProbe(probe, placed);

  const talus::deformable::DeformedDistance hit =
    talus::deformable::probeDeformed(probe, glm::dvec3(0.0, squashed + 0.02, 0.0));

  CHECK(!(hit.found && hit.distance < 0.0));
}
