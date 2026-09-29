#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/factor/factor.h>
#include <talus/deformable/solver/solver.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/serialize/materialfile/materialfile.h>

#include <glm/glm.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

using talus::serialize::MATERIAL_VERSION;
using talus::serialize::physicalMaterial;
using talus::serialize::readMaterial;
using talus::serialize::setPhysicalMaterial;
using talus::serialize::StoredMaterial;
using talus::serialize::writeMaterial;

TEST_CASE("a material survives a round trip through text", "[materialfile]") {
  StoredMaterial authored{};

  authored.baseColor = glm::vec4(0.5f, 0.25f, 0.125f, 1.0f);
  authored.emissive = glm::vec4(0.1f, 0.2f, 0.3f, 1.0f);
  authored.metallic = 0.75f;
  authored.roughness = 0.4f;
  authored.normalScale = 2.0f;
  authored.occlusionStrength = 0.6f;
  authored.alphaCutoff = 0.25f;
  authored.flags = 3u;
  authored.baseColorTexture.texture = 0xA1B2C3D4E5F60718ULL;
  authored.baseColorTexture.sampler = talus::core::SAMPLER_NEAREST_CLAMP;
  authored.normalTexture.texture = 0x1807F6E5D4C3B2A1ULL;

  const std::string text = writeMaterial(authored);
  REQUIRE(!text.empty());

  StoredMaterial read{};
  REQUIRE(readMaterial(text, read));

  REQUIRE(read.baseColor == authored.baseColor);
  REQUIRE(read.emissive == authored.emissive);
  REQUIRE(read.metallic == authored.metallic);
  REQUIRE(read.roughness == authored.roughness);
  REQUIRE(read.normalScale == authored.normalScale);
  REQUIRE(read.occlusionStrength == authored.occlusionStrength);
  REQUIRE(read.alphaCutoff == authored.alphaCutoff);
  REQUIRE(read.flags == authored.flags);
  REQUIRE(read.baseColorTexture.texture == authored.baseColorTexture.texture);
  REQUIRE(read.baseColorTexture.sampler == authored.baseColorTexture.sampler);
  REQUIRE(read.normalTexture.texture == authored.normalTexture.texture);
}

TEST_CASE("a material file states only what differs", "[materialfile]") {
  StoredMaterial read{};

  read.roughness = 0.0f;
  read.alphaCutoff = 0.0f;

  REQUIRE(readMaterial(R"({"baseColor": [0.5, 0.0, 0.3, 1.0]})", read));

  REQUIRE(read.baseColor == glm::vec4(0.5f, 0.0f, 0.3f, 1.0f));
  REQUIRE(read.roughness == 1.0f);
  REQUIRE(read.alphaCutoff == 0.5f);
  REQUIRE(read.metallic == 0.0f);
  REQUIRE(read.baseColorTexture.texture == talus::core::INVALID_ASSET);
  REQUIRE(read.baseColorTexture.sampler == talus::core::SAMPLER_LINEAR_REPEAT);
  REQUIRE(read.version == MATERIAL_VERSION);
}

TEST_CASE("a material from another version refuses to load", "[materialfile]") {
  StoredMaterial read{};

  REQUIRE(!readMaterial(R"({"version": 9999})", read));
  REQUIRE(!readMaterial("not json at all", read));
}

TEST_CASE(
  "a material file without the physical fields loads with the defaults",
  "[materialfile]"
) {
  StoredMaterial read{};

  REQUIRE(readMaterial(R"({"baseColor":[1,1,1,1]})", read));

  REQUIRE(read.friction == 0.5f);
  REQUIRE(read.restitution == 0.0f);
  REQUIRE(read.density == 1000.0f);
  REQUIRE_FALSE(read.strength.has_value());

  REQUIRE(std::isinf(physicalMaterial(read).strength));
}

TEST_CASE("the physical fields of a material round-trip", "[materialfile]") {
  StoredMaterial authored{};

  authored.friction = 0.9f;
  authored.restitution = 0.6f;
  authored.density = 2500.0f;
  authored.strength = 250.0f;

  const std::string text = writeMaterial(authored);
  REQUIRE(!text.empty());
  REQUIRE(text.find("strength") != std::string::npos);

  StoredMaterial read{};
  REQUIRE(readMaterial(text, read));

  REQUIRE(read.friction == authored.friction);
  REQUIRE(read.restitution == authored.restitution);
  REQUIRE(read.density == authored.density);
  REQUIRE(read.strength == authored.strength);

  const std::string defaultText = writeMaterial(StoredMaterial{});
  REQUIRE(defaultText.find("strength") == std::string::npos);
}

TEST_CASE(
  "a physical material converts to and from a stored material",
  "[materialfile]"
) {
  talus::objectmodel::PhysicalMaterial physical{};
  physical.friction = 0.8f;
  physical.restitution = 0.3f;
  physical.density = 800.0f;
  physical.strength = 42.0f;

  StoredMaterial stored{};
  setPhysicalMaterial(stored, physical);

  REQUIRE(stored.friction == 0.8f);
  REQUIRE(stored.restitution == 0.3f);
  REQUIRE(stored.density == 800.0f);
  REQUIRE(stored.strength == 42.0f);

  const talus::objectmodel::PhysicalMaterial roundTripped =
    physicalMaterial(stored);

  REQUIRE(roundTripped.friction == physical.friction);
  REQUIRE(roundTripped.restitution == physical.restitution);
  REQUIRE(roundTripped.density == physical.density);
  REQUIRE(roundTripped.strength == physical.strength);

  StoredMaterial infinite{};
  setPhysicalMaterial(infinite, talus::objectmodel::PhysicalMaterial{});

  REQUIRE_FALSE(infinite.strength.has_value());
}

TEST_CASE("the plastic fields of a material round-trip", "[materialfile]") {
  talus::objectmodel::PhysicalMaterial physical{};
  physical.yieldStress = 2.5e5f;
  physical.hardeningModulus = 3.0e7f;

  StoredMaterial stored{};
  setPhysicalMaterial(stored, physical);

  const std::string text = writeMaterial(stored);
  REQUIRE(text.find("yieldStress") != std::string::npos);
  REQUIRE(text.find("hardeningModulus") != std::string::npos);

  StoredMaterial read{};
  REQUIRE(readMaterial(text, read));

  const talus::objectmodel::PhysicalMaterial roundTripped =
    physicalMaterial(read);

  REQUIRE(roundTripped.yieldStress == physical.yieldStress);
  REQUIRE(roundTripped.hardeningModulus == physical.hardeningModulus);

  StoredMaterial elastic{};
  setPhysicalMaterial(elastic, talus::objectmodel::PhysicalMaterial{});

  REQUIRE_FALSE(elastic.yieldStress.has_value());
  REQUIRE_FALSE(elastic.hardeningModulus.has_value());
  REQUIRE(std::isinf(physicalMaterial(StoredMaterial{}).yieldStress));
  REQUIRE(physicalMaterial(StoredMaterial{}).hardeningModulus == 0.0f);
}

namespace {

constexpr uint32_t CUBE_LANE = 1u;
constexpr float CUBE_VOXEL = 0.05f;

std::vector<talus::objectmodel::PhysicalMaterial> cubeMaterials(float tau) {
  std::vector<talus::objectmodel::PhysicalMaterial> made(2);

  made[CUBE_LANE].density = 500.0f;
  made[CUBE_LANE].youngsModulus = 1.0e6f;
  made[CUBE_LANE].poissonsRatio = 0.3f;
  made[CUBE_LANE].stiffnessDamping = tau;

  return made;
}

std::shared_ptr<const talus::objectmodel::PreparedMesh> dampingCube(
  std::span<const talus::objectmodel::PhysicalMaterial> materials
) {
  const glm::vec3 half(CUBE_VOXEL);

  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * CUBE_VOXEL));

  primitive.shapeConfig.y = CUBE_LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(3u)
  );

  return talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    {},
    talus::deformable::VolumeElementType::Hex
  );
}

talus::objectmodel::DeformableState steppedCube(float tau) {
  const std::vector<talus::objectmodel::PhysicalMaterial> materials =
    cubeMaterials(tau);
  const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
    dampingCube(materials);

  REQUIRE(mesh != nullptr);
  REQUIRE(mesh->hexes.size() == 8u);

  const glm::mat3 pattern(
    glm::vec3(1.0f, 0.5f, -0.3f),
    glm::vec3(0.5f, -0.6f, 0.2f),
    glm::vec3(-0.3f, 0.2f, 0.4f)
  );
  const glm::vec3 spin(0.0f, 3.0f, 1.0f);

  talus::objectmodel::DeformableState state =
    talus::deformable::restState(*mesh, talus::objectmodel::Pose{});

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 rest = mesh->restPositions[node];

    state.position[node] = rest + 0.01f * (pattern * rest);
    state.velocity[node] = glm::cross(spin, rest) + 0.5f * (pattern * rest);
  }

  talus::deformable::Settings settings;

  settings.element = talus::deformable::VolumeElementType::Hex;

  talus::deformable::DeformableSolverState solver;
  talus::deformable::StepReport report;

  talus::deformable::step(
    *mesh,
    state,
    materials,
    settings,
    glm::vec3(0.0f, -9.81f, 0.0f),
    1.0f / 60.0f,
    {},
    {},
    &solver,
    report
  );

  return state;
}

bool sameState(
  const talus::objectmodel::DeformableState& left,
  const talus::objectmodel::DeformableState& right
) {
  return left.position == right.position && left.velocity == right.velocity
         && left.acceleration == right.acceleration;
}

}

TEST_CASE("the stiffness damping of a material round-trips", "[materialfile]") {
  talus::objectmodel::PhysicalMaterial physical{};
  physical.stiffnessDamping = 2.5e-3f;

  StoredMaterial stored{};
  setPhysicalMaterial(stored, physical);

  REQUIRE(stored.stiffnessDamping == 2.5e-3f);

  const std::string text = writeMaterial(stored);
  REQUIRE(text.find("stiffnessDamping") != std::string::npos);

  StoredMaterial read{};
  REQUIRE(readMaterial(text, read));

  REQUIRE(physicalMaterial(read).stiffnessDamping == physical.stiffnessDamping);
  REQUIRE(read.version == MATERIAL_VERSION);
}

TEST_CASE(
  "a material file without stiffness damping loads it as zero",
  "[materialfile]"
) {
  const std::string defaultText = writeMaterial(StoredMaterial{});
  REQUIRE(defaultText.find("stiffnessDamping") == std::string::npos);

  StoredMaterial undamped{};
  setPhysicalMaterial(undamped, talus::objectmodel::PhysicalMaterial{});

  REQUIRE_FALSE(undamped.stiffnessDamping.has_value());

  StoredMaterial read{};
  REQUIRE(readMaterial(R"({"baseColor":[1,1,1,1],"youngsModulus":1e7})", read));

  REQUIRE_FALSE(read.stiffnessDamping.has_value());
  REQUIRE(physicalMaterial(read).stiffnessDamping == 0.0f);
  REQUIRE(talus::objectmodel::PhysicalMaterial{}.stiffnessDamping == 0.0f);
}

TEST_CASE(
  "an invalid stiffness damping counts as zero at use",
  "[materialfile]"
) {
  StoredMaterial read{};
  REQUIRE(readMaterial(R"({"stiffnessDamping":-0.5})", read));

  const talus::objectmodel::PhysicalMaterial negative = physicalMaterial(read);

  REQUIRE(negative.stiffnessDamping == -0.5f);
  REQUIRE(talus::deformable::stiffnessDamping(negative) == 0.0f);

  StoredMaterial rewritten{};
  setPhysicalMaterial(rewritten, negative);

  REQUIRE_FALSE(rewritten.stiffnessDamping.has_value());

  const talus::objectmodel::DeformableState undamped = steppedCube(0.0f);

  REQUIRE_FALSE(sameState(steppedCube(1.0e-3f), undamped));

  for (const float invalid :
       {-0.5f,
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()}) {
    talus::objectmodel::PhysicalMaterial held{};
    held.stiffnessDamping = invalid;

    REQUIRE(talus::deformable::stiffnessDamping(held) == 0.0f);
    REQUIRE(sameState(steppedCube(invalid), undamped));
  }
}
