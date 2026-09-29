#include <catch2/catch_test_macros.hpp>

#include <talus/core/config/config.h>
#include <talus/field/field.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/instantiate/instantiate.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/world/world.h>
#include <talus/objectmodel/worldcontents/worldcontents.h>

#include <cstdint>

namespace {

using talus::objectmodel::BakedVolume;
using talus::objectmodel::Entity;
using talus::objectmodel::Material;
using talus::objectmodel::Mesh;
using talus::objectmodel::Pose;
using talus::objectmodel::SdfGrid;
using talus::objectmodel::World;

SdfGrid blank() {
  SdfGrid grid{};
  grid.resolution = glm::uvec3(talus::core::FIELD_GRID_RESOLUTION);

  return grid;
}

Pose marked(float marker) {
  Pose pose{};
  pose.position.x = marker;

  return pose;
}

talus::field::Primitive marker(uint32_t material) {
  return talus::field::makeSphere(glm::vec3(0.0f), 1.0f, material);
}

float markerAt(const World& world, Entity entity) {
  return world.registry().get<Pose>(entity).position.x;
}

uint32_t materialAt(const World& world, Entity entity, uint32_t index) {
  return world.primitives(entity)[index].shapeConfig.y;
}

}

TEST_CASE("every object arrives at the id it had", "[instantiate]") {
  World authored;

  const Entity first = authored.createSdfGrid(marked(10.0f), blank());
  const Entity second = authored.createSdfGrid(marked(11.0f), blank());

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.sdfGrids().size() == 2);
  REQUIRE(runtime.registry().all_of<SdfGrid>(first));
  REQUIRE(runtime.registry().all_of<SdfGrid>(second));
  REQUIRE(markerAt(runtime, first) == 10.0f);
  REQUIRE(markerAt(runtime, second) == 11.0f);
}

TEST_CASE("ids with holes in them are preserved", "[instantiate]") {
  World authored;

  const Entity zero{0};
  const Entity five{5};

  REQUIRE(authored.createSdfGridAt(zero, marked(10.0f), blank()));
  REQUIRE(authored.createSdfGridAt(five, marked(15.0f), blank()));

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.registry().all_of<SdfGrid>(zero));
  REQUIRE(runtime.registry().all_of<SdfGrid>(five));
  REQUIRE_FALSE(runtime.registry().all_of<SdfGrid>(Entity{1}));
  REQUIRE(markerAt(runtime, five) == 15.0f);
}

TEST_CASE("primitives arrive in csg order", "[instantiate]") {
  World authored;

  const Entity id = authored.createSdfGrid(marked(10.0f), blank());

  REQUIRE(authored.addPrimitive(id, marker(1)));
  REQUIRE(authored.addPrimitive(id, marker(2)));
  REQUIRE(authored.addPrimitive(id, marker(3)));

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.primitiveCount(id) == 3);
  REQUIRE(materialAt(runtime, id, 0) == 1);
  REQUIRE(materialAt(runtime, id, 1) == 2);
  REQUIRE(materialAt(runtime, id, 2) == 3);
}

TEST_CASE("a baked volume does not cross the boundary", "[instantiate]") {
  World authored;

  const Entity id = authored.createSdfGrid(marked(10.0f), blank());
  authored.setBakedVolume(id, 3);

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE_FALSE(runtime.registry().all_of<BakedVolume>(id));
  REQUIRE(authored.registry().get<BakedVolume>(id).index == 3);
}

TEST_CASE("mesh entities arrive in order", "[instantiate]") {
  World authored;

  authored.createMesh(Pose{}, Mesh{0}, Material{2});
  authored.createMesh(Pose{}, Mesh{1}, Material{4});

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  const std::span<const Entity> meshes = runtime.meshes();

  REQUIRE(meshes.size() == 2);
  REQUIRE(runtime.registry().get<Mesh>(meshes[0]).index == 0);
  REQUIRE(runtime.registry().get<Material>(meshes[0]).index == 2);
  REQUIRE(runtime.registry().get<Mesh>(meshes[1]).index == 1);
  REQUIRE(runtime.registry().get<Material>(meshes[1]).index == 4);
}

TEST_CASE("instantiating leaves the authored world alone", "[instantiate]") {
  World authored;

  const Entity id = authored.createSdfGrid(marked(10.0f), blank());
  REQUIRE(authored.addPrimitive(id, marker(1)));
  authored.createMesh(Pose{}, Mesh{0}, Material{0});

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(authored.sdfGrids().size() == 1);
  REQUIRE(authored.primitiveCount(id) == 1);
  REQUIRE(authored.meshes().size() == 1);
  REQUIRE(markerAt(authored, id) == 10.0f);
}

TEST_CASE(
  "editing the runtime world does not reach the authored one",
  "[instantiate]"
) {
  World authored;

  const Entity id = authored.createSdfGrid(marked(10.0f), blank());
  REQUIRE(authored.addPrimitive(id, marker(1)));

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  runtime.replace<Pose>(
    id,
    Pose{glm::vec3(99.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}
  );
  REQUIRE(runtime.addPrimitive(id, marker(2)));
  runtime.createMesh(Pose{}, Mesh{7}, Material{7});

  REQUIRE(markerAt(authored, id) == 10.0f);
  REQUIRE(authored.primitiveCount(id) == 1);
  REQUIRE(materialAt(authored, id, 0) == 1);
  REQUIRE(authored.meshes().size() == 0);
}

TEST_CASE("mesh entities arrive at the ids they had", "[instantiate]") {
  World authored;

  const Entity two{2};
  const Entity seven{7};

  REQUIRE(authored.createMeshAt(two, Pose{}, Mesh{0}, Material{1}));
  REQUIRE(authored.createMeshAt(seven, Pose{}, Mesh{1}, Material{2}));

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.registry().all_of<Mesh>(two));
  REQUIRE(runtime.registry().all_of<Mesh>(seven));
  REQUIRE_FALSE(runtime.registry().all_of<Mesh>(Entity{3}));
  REQUIRE(runtime.registry().get<Material>(seven).index == 2);
}

TEST_CASE("a static body stays static across instantiation", "[instantiate]") {
  World authored;

  const Entity ground = authored.createSdfGrid(marked(10.0f), blank());
  const Entity faller = authored.createSdfGrid(marked(11.0f), blank());

  authored.addStaticBody(ground);

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.registry().all_of<talus::objectmodel::StaticBody>(ground));
  REQUIRE_FALSE(
    runtime.registry().all_of<talus::objectmodel::StaticBody>(faller)
  );
}

TEST_CASE(
  "a deformable stays deformable across instantiation",
  "[instantiate]"
) {
  static_assert(
    talus::objectmodel::selfContained<talus::objectmodel::DeformableBody>,
    "DeformableBody has to be SelfContained or emplaceOrReplace refuses it"
  );

  World authored;

  const Entity dented = authored.createSdfGrid(marked(12.0f), blank());
  const Entity rigid = authored.createSdfGrid(marked(13.0f), blank());

  authored.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    dented,
    talus::objectmodel::DeformableBody{}
  );

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.registry().all_of<talus::objectmodel::DeformableBody>(dented));
  REQUIRE_FALSE(
    runtime.registry().all_of<talus::objectmodel::DeformableBody>(rigid)
  );
}

TEST_CASE(
  "a dented lattice stays dented across instantiation",
  "[instantiate]"
) {
  static_assert(
    talus::objectmodel::selfContained<talus::objectmodel::Deformed>,
    "Deformed has to be SelfContained or emplaceOrReplace refuses it"
  );

  World authored;

  const Entity dented = authored.createSdfGrid(marked(14.0f), blank());
  const Entity pristine = authored.createSdfGrid(marked(15.0f), blank());

  for (const Entity entity : {dented, pristine}) {
    authored.addStaticBody(entity);

    talus::field::SampledSdf field;
    field.resolution = glm::uvec3(2u);
    field.distances.assign(8u, 1.0f);
    field.materials.assign(8u, 0u);

    authored.setSampledSdf(entity, std::move(field));
  }

  authored.patchSampledSdf(dented, [](talus::field::SampledSdf& lattice) {
    lattice.distances[0] = -1.0f;
  });

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.registry().all_of<talus::objectmodel::Deformed>(dented));
  REQUIRE_FALSE(
    runtime.registry().all_of<talus::objectmodel::Deformed>(pristine)
  );
}

TEST_CASE(
  "an object whose primitives were all removed arrives built from none",
  "[instantiate]"
) {
  using talus::objectmodel::SdfPrimitiveRange;

  World authored;

  const Entity emptied = authored.createSdfGrid(marked(17.0f), blank());
  const Entity built = authored.createSdfGrid(marked(18.0f), blank());
  const Entity carried = authored.createSdfGrid(marked(19.0f), blank());

  REQUIRE(authored.addPrimitive(emptied, marker(1)));
  REQUIRE(authored.removePrimitive(emptied, 0u));
  REQUIRE(authored.addPrimitive(built, marker(2)));

  for (const Entity entity : {emptied, built, carried}) {
    talus::field::SampledSdf field;
    field.resolution = glm::uvec3(2u);
    field.distances.assign(8u, -1.0f);
    field.materials.assign(8u, 0u);

    authored.setSampledSdf(entity, std::move(field));
  }

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.registry().all_of<SdfPrimitiveRange>(emptied));
  REQUIRE(runtime.primitives(emptied).empty());
  REQUIRE(runtime.needsResample(emptied));
  REQUIRE(runtime.currentSampledSdf(emptied) == nullptr);

  REQUIRE(runtime.primitiveCount(built) == 1u);
  REQUIRE(runtime.needsResample(built));

  REQUIRE_FALSE(runtime.registry().all_of<SdfPrimitiveRange>(carried));
  REQUIRE_FALSE(runtime.needsResample(carried));
  REQUIRE(runtime.currentSampledSdf(carried) == authored.sampledSdf(carried));
}

TEST_CASE("the runtime shares the authored lattice", "[instantiate]") {
  World authored;

  const Entity entity = authored.createSdfGrid(marked(16.0f), blank());

  talus::field::SampledSdf field;
  field.resolution = glm::uvec3(2u);
  field.distances.assign(8u, 1.0f);
  field.materials.assign(8u, 0u);

  authored.setSampledSdf(entity, std::move(field));

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.sampledSdf(entity) == authored.sampledSdf(entity));
  REQUIRE(authored.sampledSdfUsers(entity) == 2);
}

TEST_CASE(
  "an entity that is neither a grid nor a mesh survives",
  "[instantiate]"
) {
  using talus::objectmodel::Lens;

  World authored;

  const Entity eye = authored.createAuthored();

  Pose seat{};
  seat.position = glm::vec3(13.0f, 10.0f, 25.0f);

  Lens lens{};
  lens.verticalFov = 55.0f;

  authored.emplaceAuthored(eye, seat);
  authored.emplaceAuthored(eye, lens);

  World runtime;

  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(
    talus::objectmodel::liveEntities(runtime).size()
    == talus::objectmodel::liveEntities(authored).size()
  );

  REQUIRE(runtime.registry().all_of<Lens>(eye));
  REQUIRE(runtime.registry().get<Lens>(eye).verticalFov == 55.0f);
  REQUIRE(runtime.registry().get<Pose>(eye).position == seat.position);
}

TEST_CASE("a cleared world takes back the ids it held", "[instantiate]") {
  World live;

  const Entity first = live.createSdfGrid(marked(10.0f), blank());
  const Entity second = live.createSdfGrid(marked(11.0f), blank());

  World kept;
  talus::objectmodel::instantiateWorld(live, kept);

  live.clear();

  REQUIRE(talus::objectmodel::liveEntities(live).empty());

  talus::objectmodel::instantiateWorld(kept, live);

  REQUIRE(live.registry().all_of<SdfGrid>(first));
  REQUIRE(live.registry().all_of<SdfGrid>(second));
  REQUIRE(markerAt(live, first) == 10.0f);
  REQUIRE(markerAt(live, second) == 11.0f);
}

TEST_CASE("a recycled slot gives its id back", "[instantiate]") {
  World live;

  const Entity held = live.createSdfGrid(marked(10.0f), blank());

  World kept;
  talus::objectmodel::instantiateWorld(live, kept);

  REQUIRE(live.destroy(held));

  const Entity recycled = live.createSdfGrid(marked(99.0f), blank());

  REQUIRE(recycled != held);

  live.clear();

  talus::objectmodel::instantiateWorld(kept, live);

  REQUIRE(live.registry().all_of<SdfGrid>(held));
  REQUIRE(markerAt(live, held) == 10.0f);
  REQUIRE(talus::objectmodel::liveEntities(live).size() == 1);
}
