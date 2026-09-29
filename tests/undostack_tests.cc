#include <catch2/catch_test_macros.hpp>

#include <talus/core/config/config.h>
#include <talus/field/field.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/deformed/deformed.h>
#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/world/world.h>
#include <talus/objectmodel/worldcontents/worldcontents.h>
#include <talus/undo/undostack/undostack.h>

#include <cstdint>
#include <cstring>
#include <utility>

namespace {

using talus::objectmodel::DeformableBody;
using talus::objectmodel::Deformed;
using talus::objectmodel::Entity;
using talus::objectmodel::MassScale;
using talus::objectmodel::Material;
using talus::objectmodel::Pose;
using talus::objectmodel::SdfGrid;
using talus::objectmodel::StaticBody;
using talus::objectmodel::World;
using talus::undo::UndoStack;

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

float markerAt(const World& world, Entity entity) {
  return world.registry().get<Pose>(entity).position.x;
}

void dent(World& world, Entity entity) {
  world.addStaticBody(entity);

  talus::field::SampledSdf field;
  field.resolution = glm::uvec3(2u);
  field.distances.assign(8u, 1.0f);
  field.materials.assign(8u, 0u);

  world.setSampledSdf(entity, std::move(field));

  world.patchSampledSdf(entity, [](talus::field::SampledSdf& lattice) {
    lattice.distances[0] = -1.0f;
  });
}

}

TEST_CASE("an empty stack has nothing to undo", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  REQUIRE_FALSE(stack.undo(world));
  REQUIRE_FALSE(stack.redo(world));
  REQUIRE_FALSE(stack.undoLabel().has_value());
  REQUIRE_FALSE(stack.redoLabel().has_value());
  REQUIRE(markerAt(world, entity) == 10.0f);
}

TEST_CASE("undo restores the world as it was recorded", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");

  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE(markerAt(world, entity) == 20.0f);
  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
}

TEST_CASE("redo puts the change back", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");

  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);

  REQUIRE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);
}

TEST_CASE("a stack walks back through several steps", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "First");
  world.replace<Pose>(entity, marked(20.0f));

  stack.record(world, "Second");
  world.replace<Pose>(entity, marked(30.0f));

  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);

  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);

  REQUIRE_FALSE(stack.undo(world));

  REQUIRE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);

  REQUIRE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 30.0f);

  REQUIRE_FALSE(stack.redo(world));
}

TEST_CASE("recording drops the redo side", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");
  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE(stack.undo(world));
  REQUIRE(stack.redoLabel().has_value());

  stack.record(world, "Elsewhere");
  world.replace<Pose>(entity, marked(30.0f));

  REQUIRE_FALSE(stack.redoLabel().has_value());
  REQUIRE_FALSE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 30.0f);
}

TEST_CASE("the oldest step falls off a full stack", "[undo]") {
  World world;
  UndoStack stack{2};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "First");
  world.replace<Pose>(entity, marked(20.0f));

  stack.record(world, "Second");
  world.replace<Pose>(entity, marked(30.0f));

  stack.record(world, "Third");
  world.replace<Pose>(entity, marked(40.0f));

  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 30.0f);

  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);

  REQUIRE_FALSE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);
}

TEST_CASE("each side names the step it holds", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");
  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE(stack.undoLabel() == "Move");
  REQUIRE_FALSE(stack.redoLabel().has_value());

  REQUIRE(stack.undo(world));

  REQUIRE_FALSE(stack.undoLabel().has_value());
  REQUIRE(stack.redoLabel() == "Move");
}

TEST_CASE("clearing forgets both sides", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");
  world.replace<Pose>(entity, marked(20.0f));

  stack.clear();

  REQUIRE_FALSE(stack.undo(world));
  REQUIRE_FALSE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);
}

TEST_CASE("a depth of zero records nothing", "[undo]") {
  World world;
  UndoStack stack{0};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");
  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE_FALSE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);
}

TEST_CASE("a destroyed entity comes back whole", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  REQUIRE(world.addPrimitive(
    entity,
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f, 7u)
  ));

  world.emplaceOrReplace<Material>(entity, Material{4u});
  world.emplaceOrReplace<MassScale>(entity, MassScale{2.5f});
  world.addStaticBody(entity);

  dent(world, entity);

  stack.record(world, "Delete");

  REQUIRE(world.destroy(entity));
  REQUIRE(talus::objectmodel::liveEntities(world).empty());

  REQUIRE(stack.undo(world));

  REQUIRE(world.registry().all_of<SdfGrid>(entity));
  REQUIRE(markerAt(world, entity) == 10.0f);

  REQUIRE(world.primitiveCount(entity) == 1);
  REQUIRE(world.primitives(entity)[0].shapeConfig.y == 7u);

  REQUIRE(world.registry().get<Material>(entity).index == 4u);
  REQUIRE(world.registry().get<MassScale>(entity).factor == 2.5f);
  REQUIRE(world.registry().all_of<StaticBody>(entity));
  REQUIRE(world.registry().all_of<Deformed>(entity));

  REQUIRE(world.hasSampledSdf(entity));
  REQUIRE(world.sampledSdf(entity)->distances[0] == -1.0f);
}

TEST_CASE("a recorded step outlives later carving", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  dent(world, entity);

  stack.record(world, "Carve");

  world.patchSampledSdf(entity, [](talus::field::SampledSdf& lattice) {
    lattice.distances[1] = -2.0f;
  });

  REQUIRE(world.sampledSdf(entity)->distances[1] == -2.0f);

  REQUIRE(stack.undo(world));

  REQUIRE(world.sampledSdf(entity)->distances[0] == -1.0f);
  REQUIRE(world.sampledSdf(entity)->distances[1] == 1.0f);
}

TEST_CASE(
  "a discarded edit leaves no step and rolls the world back",
  "[undo]"
) {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  REQUIRE(stack.begin(world, "Move"));

  REQUIRE(stack.open());

  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE(stack.discard(world));

  REQUIRE_FALSE(stack.open());
  REQUIRE_FALSE(stack.undoLabel().has_value());
  REQUIRE_FALSE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
}

TEST_CASE("a discarded edit keeps the redo side", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  stack.record(world, "Move");
  world.replace<Pose>(entity, marked(20.0f));

  REQUIRE(stack.undo(world));
  REQUIRE(stack.redoLabel() == "Move");

  REQUIRE(stack.begin(world, "Refused"));

  REQUIRE(stack.discard(world));

  REQUIRE(stack.redoLabel() == "Move");
  REQUIRE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);
}

TEST_CASE("committing an edit pushes exactly one step", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(10.0f), blank());

  REQUIRE(stack.begin(world, "Move"));

  world.replace<Pose>(entity, marked(20.0f));
  world.replace<Pose>(entity, marked(30.0f));

  REQUIRE(stack.commit());
  REQUIRE_FALSE(stack.open());

  REQUIRE(stack.undoLabel() == "Move");
  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);

  REQUIRE_FALSE(stack.undo(world));
}

TEST_CASE("committing without an open edit does nothing", "[undo]") {
  World world;
  UndoStack stack{8};

  REQUIRE_FALSE(stack.commit());
  REQUIRE_FALSE(stack.discard(world));
  REQUIRE_FALSE(stack.undoLabel().has_value());
}

TEST_CASE("a second begin is refused while an edit is open", "[undo]") {
  World world;
  UndoStack stack{8};
  const Entity entity = world.createSdfGrid(marked(10.0f), blank());
  REQUIRE(stack.begin(world, "Move"));
  world.replace<Pose>(entity, marked(20.0f));
  REQUIRE_FALSE(stack.begin(world, "Rotate"));
  REQUIRE(stack.open());
  REQUIRE(stack.commit());
  REQUIRE(stack.undoLabel() == "Move");
  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
}

namespace {

talus::undo::FieldDiff nudge(Entity entity, float from, float to) {
  talus::undo::FieldDiff diff;
  diff.entity = entity;
  diff.component = "Pose";
  diff.field = 0u;
  diff.kind = talus::objectmodel::FieldKind::Vec3;
  const float before[3]{from, 0.0f, 0.0f};
  const float after[3]{to, 0.0f, 0.0f};
  diff.before.resize(12u);
  diff.after.resize(12u);
  std::memcpy(diff.before.data(), before, 12u);
  std::memcpy(diff.after.data(), after, 12u);
  return diff;
}

void moveTo(World& world, Entity entity, float x) {
  world.replace<Pose>(entity, marked(x));
}

}

TEST_CASE(
  "a field transaction undoes and redoes its diffs without a snapshot",
  "[undo][fields]"
) {
  World world;
  UndoStack stack{8};
  const Entity entity = world.createSdfGrid(marked(10.0f), blank());
  REQUIRE(stack.beginFields("Nudge"));
  REQUIRE(stack.fieldsOpen());
  moveTo(world, entity, 20.0f);
  stack.note(nudge(entity, 10.0f, 20.0f));
  REQUIRE(stack.commit());
  REQUIRE(stack.undoLabel() == "Nudge");
  REQUIRE(stack.undoIsFields());
  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
  REQUIRE(stack.redoIsFields());
  REQUIRE(stack.redo(world));
  REQUIRE(markerAt(world, entity) == 20.0f);
}

TEST_CASE(
  "notes on one field merge to first before and last after",
  "[undo][fields]"
) {
  World world;
  UndoStack stack{8};
  const Entity entity = world.createSdfGrid(marked(10.0f), blank());
  REQUIRE(stack.beginFields("Nudge"));
  moveTo(world, entity, 20.0f);
  stack.note(nudge(entity, 10.0f, 20.0f));
  moveTo(world, entity, 30.0f);
  stack.note(nudge(entity, 20.0f, 30.0f));
  REQUIRE(stack.commit());
  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
  REQUIRE_FALSE(stack.undo(world));
}

TEST_CASE(
  "discarding a field transaction applies the befores",
  "[undo][fields]"
) {
  World world;
  UndoStack stack{8};
  const Entity entity = world.createSdfGrid(marked(10.0f), blank());
  REQUIRE(stack.beginFields("Nudge"));
  moveTo(world, entity, 20.0f);
  stack.note(nudge(entity, 10.0f, 20.0f));
  REQUIRE(stack.discard(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
  REQUIRE_FALSE(stack.open());
  REQUIRE_FALSE(stack.undoLabel().has_value());
}

TEST_CASE("an empty field transaction commits nothing", "[undo][fields]") {
  World world;
  UndoStack stack{8};
  REQUIRE(stack.beginFields("Nothing"));
  REQUIRE_FALSE(stack.commit());
  REQUIRE_FALSE(stack.open());
  REQUIRE(stack.depth() == 0u);
}

TEST_CASE(
  "forgetting since a mark drops the play steps and keeps the edit steps",
  "[undo][fields]"
) {
  World world;
  UndoStack stack{8};
  const Entity entity = world.createSdfGrid(marked(10.0f), blank());
  stack.record(world, "Move");
  moveTo(world, entity, 20.0f);
  const size_t mark = stack.depth();
  REQUIRE(mark == 1u);
  REQUIRE(stack.beginFields("Nudge"));
  moveTo(world, entity, 30.0f);
  stack.note(nudge(entity, 20.0f, 30.0f));
  REQUIRE(stack.commit());
  REQUIRE(stack.depth() == 2u);
  REQUIRE(stack.undo(world));
  REQUIRE(stack.redoLabel() == "Nudge");
  stack.forgetSince(mark);
  REQUIRE(stack.depth() == 1u);
  REQUIRE(stack.undoLabel() == "Move");
  REQUIRE_FALSE(stack.redoLabel().has_value());
  REQUIRE_FALSE(stack.undoIsFields());
  REQUIRE(stack.undo(world));
  REQUIRE(markerAt(world, entity) == 10.0f);
}

TEST_CASE(
  "neither kind of begin opens while another is open",
  "[undo][fields]"
) {
  World world;
  UndoStack stack{8};
  REQUIRE(stack.beginFields("Nudge"));
  REQUIRE_FALSE(stack.begin(world, "Move"));
  REQUIRE_FALSE(stack.beginFields("Again"));
  REQUIRE(stack.discard(world));
  REQUIRE(stack.begin(world, "Move"));
  REQUIRE_FALSE(stack.beginFields("Nudge"));
  REQUIRE(stack.discard(world));
}

TEST_CASE("an edit on a deformable body records without a material", "[undo]") {
  World world;
  UndoStack stack{8};

  const Entity entity = world.createSdfGrid(marked(30.0f), blank());

  world.emplaceOrReplace<DeformableBody>(entity, DeformableBody{});

  talus::field::SampledSdf field;
  field.resolution = glm::uvec3(2u);
  field.distances.assign(8u, -1.0f);
  field.materials.assign(8u, 0u);

  world.setSampledSdf(entity, std::move(field));

  stack.record(world, "Strip");

  REQUIRE(world.removeAuthored<DeformableBody>(entity));
  REQUIRE_FALSE(world.registry().all_of<DeformableBody>(entity));

  REQUIRE(stack.undo(world));
  REQUIRE(world.registry().all_of<DeformableBody>(entity));
}
