#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>

#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include "../engine/app/capture/capture.h"
#include "../engine/capi/win32windowsystem/win32windowsystem.h"

#include <talus/core/telemetry/telemetry.h>
#include <talus/engine/engine/engine.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/core/config/config.h>
#include <talus/gpu/buffer/buffer.h>
#include <talus/gpu/device/device.h>
#include <talus/gpu/oneshot/oneshotcommand.h>
#include <talus/imagecompare/imagecompare.h>
#include <talus/objectmodel/component/bakedvolume/bakedvolume.h>
#include <talus/objectmodel/component/directionallight/directionallight.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/sharedfield/sharedfield.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/objectmodel/world/world.h>
#include <talus/renderer/sdfrecordtable/sdfrecordtable.h>
#include <talus/renderer/sdfresidency/sdfresidency.h>
#include <talus/renderer/volumepool/volumepool.h>
#include <talus/undo/undostack/undostack.h>
#include <talus/view/viewport/viewport.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace {

using talus::engine::Engine;
using talus::objectmodel::BakedVolume;
using talus::objectmodel::Entity;
using talus::objectmodel::Pose;
using talus::objectmodel::SdfGrid;
using talus::objectmodel::World;

constexpr uint32_t WIDTH = 320u;
constexpr uint32_t HEIGHT = 240u;

constexpr float BOX_HALF = 0.1f;
constexpr float BOX_SPACING = 0.6f;
constexpr uint32_t BOX_SAMPLES = 9u;

constexpr float LATTICE_HALF = 0.6f;
constexpr float LATTICE_MARGIN = 0.1f;
constexpr uint32_t LATTICE_SAMPLES = 25u;

constexpr uint32_t TINY_SAMPLES = 3u;

constexpr uint32_t EDITED_SAMPLES = 17u;

constexpr float DENT_DEPTH = 0.01f;
constexpr float DENT_SLOPE = 1.5f;

constexpr int SURFACE_CONTRAST = 24;

struct WindowDeleter {
  void operator()(HWND window) const {
    DestroyWindow(window);
  }
};

struct EngineDeleter {
  void operator()(Engine* engine) const {
    engine->context().device().waitIdle();
    delete engine;
  }
};

using OwnedWindow = std::unique_ptr<HWND__, WindowDeleter>;

using OwnedEngine = std::unique_ptr<Engine, EngineDeleter>;

OwnedWindow openWindow() {
  RECT frame{0, 0, static_cast<LONG>(WIDTH), static_cast<LONG>(HEIGHT)};
  AdjustWindowRect(&frame, WS_OVERLAPPEDWINDOW, FALSE);

  OwnedWindow window(CreateWindowExW(
    0,
    L"STATIC",
    L"Talus renderer tests",
    WS_OVERLAPPEDWINDOW,
    0,
    0,
    frame.right - frame.left,
    frame.bottom - frame.top,
    nullptr,
    nullptr,
    GetModuleHandleW(nullptr),
    nullptr
  ));

  REQUIRE(window != nullptr);

  return window;
}

OwnedEngine openEngine(HWND window) {
  return OwnedEngine(new Engine(
    std::make_unique<talus::capi::Win32WindowSystem>(window),
    TALUS_DEMO_PROJECT
  ));
}

struct Scene {
  OwnedWindow window;
  OwnedEngine engine;
  talus::view::ViewportId viewport{};
  talus::core::Telemetry telemetry;
};

void lookFrom(Scene& scene, const glm::vec3& position) {
  talus::view::Viewport port{};
  port.pose.position = position;
  port.pose.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

  REQUIRE(scene.engine->configureViewport(scene.viewport, port));
}

std::unique_ptr<Scene> openScene() {
  auto scene = std::make_unique<Scene>();

  scene->window = openWindow();
  scene->engine = openEngine(scene->window.get());
  scene->viewport = scene->engine->createViewport();

  lookFrom(*scene, glm::vec3(0.0f, 0.0f, 4.0f));

  World& world = scene->engine->world();

  talus::objectmodel::DirectionalLight sun{};
  sun.direction = glm::vec3(0.2f, 0.4f, 1.0f);

  const Entity light = world.createAuthored();
  world.emplaceAuthored(light, Pose{});
  world.emplaceAuthored(light, sun);

  return scene;
}

double reading(const Scene& scene, std::string_view name) {
  const talus::core::Telemetry::Key key = scene.telemetry.find(name);

  return key == talus::core::Telemetry::INVALID_KEY ? -1.0
                                                    : scene.telemetry.get(key);
}

void render(Scene& scene, const std::function<void(VkImage)>& capture = {}) {
  talus::engine::FrameRequest request;
  request.viewport = scene.viewport;
  request.deltaSeconds = 1.0f / 60.0f;
  request.capture = capture;

  scene.telemetry.clear();

  REQUIRE_FALSE(scene.engine->renderFrame(request, scene.telemetry));
}

talus::image::Bitmap shoot(Scene& scene) {
  talus::image::Bitmap shot;

  render(scene, [&](VkImage image) {
    shot = talus::capture::read(
      scene.engine->context().device(),
      image,
      VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
      scene.engine->swapChain().extent(),
      scene.engine->swapChain().imageFormat()
    );
  });

  REQUIRE(shot.width > 0u);
  REQUIRE(shot.pixels.size() == size_t(shot.width) * shot.height * 4u);

  return shot;
}

uint32_t surfacePixels(
  const talus::image::Bitmap& shot,
  uint32_t firstColumn,
  uint32_t endColumn
) {
  uint32_t found = 0u;

  for (uint32_t y = 0u; y != shot.height; ++y) {
    for (uint32_t x = firstColumn; x != endColumn; ++x) {
      const size_t at = 4u * (size_t(y) * shot.width + x);

      const int contrast = std::abs(int(shot.pixels[at]))
                           + std::abs(int(shot.pixels[at + 1u]))
                           + std::abs(int(shot.pixels[at + 2u]));

      if (contrast > SURFACE_CONTRAST) {
        ++found;
      }
    }
  }

  return found;
}

talus::field::SampledSdf latticeOf(
  const talus::field::Primitive& primitive,
  float half,
  uint32_t samples
) {
  const std::array<talus::field::Primitive, 1> solid{primitive};

  return talus::field::bake(
    solid,
    glm::vec3(-half),
    glm::vec3(half),
    glm::uvec3(samples)
  );
}

Entity latticeEntity(
  World& world,
  const glm::vec3& at,
  talus::field::SampledSdf lattice
) {
  SdfGrid grid{};
  grid.resolution = lattice.resolution;
  grid.voxelSize = lattice.voxelSize;
  grid.origin = glm::vec4(lattice.origin, 1.0f);
  grid.margin = LATTICE_MARGIN;

  const Entity entity =
    world.createSdfGrid(Pose{at, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}, grid);

  world.setSampledSdf(entity, std::move(lattice));

  return entity;
}

uint32_t volumeOf(const World& world, Entity entity) {
  const auto* volume = world.registry().try_get<BakedVolume>(entity);

  return volume == nullptr ? UINT32_MAX : volume->index;
}

uint32_t drawnEntities(const World& world) {
  uint32_t drawn = 0u;

  for (const Entity entity : world.sdfGrids()) {
    if (world.registry().all_of<BakedVolume>(entity)) {
      ++drawn;
    }
  }

  return drawn;
}

std::vector<float> brickBoundsOf(
  Engine& engine,
  uint32_t volume,
  size_t count
) {
  const talus::gpu::Device& device = engine.context().device();

  device.waitIdle();

  const VkDeviceSize sizeBytes =
    static_cast<VkDeviceSize>(count * sizeof(float));

  talus::gpu::Buffer readback(
    device,
    sizeBytes,
    VK_BUFFER_USAGE_TRANSFER_DST_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  talus::gpu::OneShotCommand cmd;
  cmd.start(device);

  VkMemoryBarrier2 written{};
  written.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  written.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
  written.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
  written.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
  written.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;

  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.memoryBarrierCount = 1;
  dependency.pMemoryBarriers = &written;

  vkCmdPipelineBarrier2(cmd.cmdBuffer(), &dependency);

  VkBufferCopy region{};
  region.srcOffset = static_cast<VkDeviceSize>(volume)
                     * talus::core::BRICK_TABLE_STRIDE * sizeof(float);
  region.size = sizeBytes;

  vkCmdCopyBuffer(
    cmd.cmdBuffer(),
    engine.storage().recordTable().brickBounds().buffer(),
    readback.buffer(),
    1,
    &region
  );

  cmd.submit(device);

  std::vector<float> values(count);

  void* mapped = nullptr;

  REQUIRE(
    vkMapMemory(device.vkDevice(), readback.memory(), 0, sizeBytes, 0, &mapped)
    == VK_SUCCESS
  );

  std::memcpy(values.data(), mapped, static_cast<size_t>(sizeBytes));
  vkUnmapMemory(device.vkDevice(), readback.memory());

  return values;
}

bool boundsMatch(
  Engine& engine,
  uint32_t volume,
  const talus::field::SampledSdf& lattice
) {
  const std::vector<float> uploaded =
    brickBoundsOf(engine, volume, 1u + lattice.brickLipschitz.size());

  return uploaded.front() == lattice.globalLipschitz
         && std::equal(
           lattice.brickLipschitz.begin(),
           lattice.brickLipschitz.end(),
           uploaded.begin() + 1
         );
}

Entity uniqueLattice(World& world, uint32_t index) {
  return latticeEntity(
    world,
    glm::vec3(float(index % 64u), 0.0f, 10.0f + float(index / 64u)),
    latticeOf(
      fixture::sphere(glm::vec3(0.0f), 0.1f + 1.0e-5f * float(index)),
      0.2f,
      TINY_SAMPLES
    )
  );
}

void dent(World& world, Entity entity) {
  world.patchSampledSdf(entity, [](talus::field::SampledSdf& field) {
    for (float& distance : field.distances) {
      distance += DENT_DEPTH;
    }

    field.globalLipschitz *= DENT_SLOPE;

    for (float& bound : field.brickLipschitz) {
      bound *= DENT_SLOPE;
    }
  });

  world.markSdfDirty(
    entity,
    talus::field::SampleBox{
      glm::uvec3(0u),
      world.sampledSdf(entity)->resolution
    }
  );
}

std::vector<Entity> undrawn(const World& world) {
  std::vector<Entity> found;

  for (const Entity entity : world.sdfGrids()) {
    if (!world.registry().all_of<BakedVolume>(entity)) {
      found.push_back(entity);
    }
  }

  return found;
}

Entity drawnExcept(const World& world, Entity kept) {
  for (const Entity entity : world.sdfGrids()) {
    if (entity != kept && world.registry().all_of<BakedVolume>(entity)) {
      return entity;
    }
  }

  return talus::objectmodel::INVALID_ENTITY;
}

struct SurfaceSides {
  uint32_t left = 0u;
  uint32_t right = 0u;
};

SurfaceSides surfaceSides(Scene& scene) {
  const talus::image::Bitmap shot = shoot(scene);

  return {
    surfacePixels(shot, 0u, shot.width / 2u),
    surfacePixels(shot, shot.width / 2u, shot.width)
  };
}

Entity boxEntity(World& world, const glm::vec3& at) {
  SdfGrid grid{};
  grid.resolution = glm::uvec3(BOX_SAMPLES);

  const Entity entity =
    world.createSdfGrid(Pose{at, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}, grid);

  REQUIRE(world.addPrimitive(
    entity,
    fixture::box(glm::vec3(0.0f), glm::vec3(BOX_HALF))
  ));

  return entity;
}

Entity sphereEntity(World& world, const glm::vec3& at) {
  SdfGrid grid{};
  grid.resolution = glm::uvec3(BOX_SAMPLES);

  const Entity entity =
    world.createSdfGrid(Pose{at, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}, grid);

  REQUIRE(
    world.addPrimitive(entity, fixture::sphere(glm::vec3(0.0f), BOX_HALF))
  );

  return entity;
}

struct Emptied {
  Entity emptied = talus::objectmodel::INVALID_ENTITY;
  SurfaceSides drawn;
};

Emptied emptyBesideASphere(Scene& scene) {
  World& world = scene.engine->world();

  const Entity emptied = boxEntity(world, glm::vec3(-0.8f, 0.0f, 0.0f));
  const Entity kept = sphereEntity(world, glm::vec3(0.8f, 0.0f, 0.0f));

  render(scene);

  const SurfaceSides drawn = surfaceSides(scene);

  REQUIRE(drawn.left > 0u);
  REQUIRE(drawn.right > 0u);
  REQUIRE(volumeOf(world, emptied) != volumeOf(world, kept));

  REQUIRE(world.removePrimitive(emptied, 0u));

  render(scene);

  const SurfaceSides removed = surfaceSides(scene);

  REQUIRE(removed.left == 0u);
  REQUIRE(removed.right == drawn.right);

  return {emptied, drawn};
}

bool holdsNoInterior(const World& world, Entity entity) {
  const talus::field::SampledSdf* lattice = world.currentSampledSdf(entity);

  return lattice != nullptr
         && std::ranges::none_of(lattice->distances, [](float distance) {
              return distance < 0.0f;
            });
}

Entity boxDonor(World& world, const glm::vec3& at) {
  const Entity entity = boxEntity(world, at);

  const SdfGrid& fitted = world.registry().get<SdfGrid>(entity);
  const auto primitives = world.primitives(entity);
  const talus::field::Aabb box = talus::objectmodel::gridBox(
    primitives,
    talus::objectmodel::gridMargin(fitted, primitives)
  );

  world.setSampledSdf(
    entity,
    talus::field::bake(primitives, box.minimum, box.maximum, fitted.resolution)
  );

  return entity;
}

void buildBoxes(World& world, uint32_t count) {
  const uint32_t side =
    static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(count))));

  const auto at = [side](uint32_t index) {
    return glm::vec3(
      (float(index % side) - 0.5f * float(side)) * BOX_SPACING,
      -1.0f,
      -(float(index / side) + 1.0f) * BOX_SPACING
    );
  };

  const Entity donor = boxDonor(world, at(0u));

  SdfGrid shared = world.registry().get<SdfGrid>(donor);
  shared.margin =
    talus::objectmodel::gridMargin(shared, world.primitives(donor));

  for (uint32_t index = 1u; index != count; ++index) {
    const Entity entity = world.createSdfGrid(
      Pose{at(index), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)},
      shared
    );

    world.shareSampledSdf(donor, entity);
  }
}

void everyEntityHasARecord(uint32_t count) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  buildBoxes(world, count);

  REQUIRE(world.sdfGrids().size() == count);

  for (uint32_t frame = 0u; frame != 2u; ++frame) {
    render(*scene);

    CHECK(
      scene->engine->storage().framePacker().sdfRecordEntities().size() == count
    );
    CHECK(reading(*scene, "sdfRecords") == double(count));
    CHECK(reading(*scene, "droppedVolumes") == 0.0);
    CHECK(reading(*scene, "shadowBinned") == 1.0);
    CHECK(drawnEntities(world) == count);
  }
}

}

TEST_CASE("every field entity has a record at 5000 boxes", "[renderer]") {
  everyEntityHasARecord(5000u);
}

TEST_CASE("every field entity has a record at 100000 boxes", "[renderer]") {
  everyEntityHasARecord(100000u);
}

TEST_CASE(
  "an entity with no primitives is drawn from its own lattice",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const Entity large = latticeEntity(
    world,
    glm::vec3(-0.8f, 0.0f, 0.0f),
    latticeOf(
      fixture::sphere(glm::vec3(0.0f), 0.5f),
      LATTICE_HALF,
      LATTICE_SAMPLES
    )
  );

  const Entity small = latticeEntity(
    world,
    glm::vec3(0.8f, 0.0f, 0.0f),
    latticeOf(
      fixture::sphere(glm::vec3(0.0f), 0.2f),
      LATTICE_HALF,
      LATTICE_SAMPLES
    )
  );

  REQUIRE(world.primitives(large).empty());
  REQUIRE(world.primitives(small).empty());

  render(*scene);

  const talus::image::Bitmap shot = shoot(*scene);

  const uint32_t left = surfacePixels(shot, 0u, shot.width / 2u);
  const uint32_t right = surfacePixels(shot, shot.width / 2u, shot.width);

  std::cout << "lattice surface pixels: left " << left << " right " << right
            << '\n';

  CHECK(volumeOf(world, large) != volumeOf(world, small));
  CHECK(right > 0u);
  CHECK(left > 2u * right);
}

TEST_CASE("equal lattices share one volume", "[renderer]") {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const talus::field::SampledSdf round = latticeOf(
    fixture::sphere(glm::vec3(0.0f), 0.4f),
    LATTICE_HALF,
    LATTICE_SAMPLES
  );

  const Entity donor =
    latticeEntity(world, glm::vec3(-1.5f, 0.0f, 0.0f), round);

  const Entity taker =
    latticeEntity(world, glm::vec3(-0.5f, 0.0f, 0.0f), round);
  world.shareSampledSdf(donor, taker);

  const Entity copy = latticeEntity(world, glm::vec3(0.5f, 0.0f, 0.0f), round);

  const Entity other = latticeEntity(
    world,
    glm::vec3(1.5f, 0.0f, 0.0f),
    latticeOf(
      fixture::box(glm::vec3(0.0f), glm::vec3(0.3f)),
      LATTICE_HALF,
      LATTICE_SAMPLES
    )
  );

  REQUIRE(world.sampledSdf(copy) != world.sampledSdf(donor));

  render(*scene);

  const uint32_t shared = volumeOf(world, donor);

  REQUIRE(shared != UINT32_MAX);
  CHECK(volumeOf(world, taker) == shared);
  CHECK(volumeOf(world, copy) == shared);
  CHECK(volumeOf(world, other) != shared);
  CHECK(volumeOf(world, other) != UINT32_MAX);
  CHECK(scene->engine->storage().volumePool().users(shared) == 3u);
  CHECK(scene->engine->storage().volumePool().allocated() == 2u);
  CHECK(reading(*scene, "volumes") == 2.0);
}

TEST_CASE(
  "a lattice changed in place is not shared by its old content",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const talus::field::SampledSdf round = latticeOf(
    fixture::sphere(glm::vec3(0.0f), 0.4f),
    LATTICE_HALF,
    LATTICE_SAMPLES
  );

  const Entity dented =
    latticeEntity(world, glm::vec3(-1.0f, 0.0f, 0.0f), round);
  world.addStaticBody(dented);

  render(*scene);

  const uint32_t held = volumeOf(world, dented);

  REQUIRE(held != UINT32_MAX);

  world.patchSampledSdf(dented, [](talus::field::SampledSdf& field) {
    for (float& distance : field.distances) {
      distance += 0.1f;
    }
  });
  world.markSdfDirty(
    dented,
    talus::field::SampleBox{glm::uvec3(0u), round.resolution}
  );

  const Entity fresh = latticeEntity(world, glm::vec3(1.0f, 0.0f, 0.0f), round);

  render(*scene);

  CHECK(volumeOf(world, dented) == held);
  CHECK(volumeOf(world, fresh) != held);
  CHECK(volumeOf(world, fresh) != UINT32_MAX);
  CHECK(scene->engine->storage().volumePool().users(held) == 1u);
}

TEST_CASE(
  "the volume pool grows to its ceiling and counts what it drops",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const Entity front = latticeEntity(
    world,
    glm::vec3(0.0f),
    latticeOf(
      fixture::sphere(glm::vec3(0.0f), 0.5f),
      LATTICE_HALF,
      LATTICE_SAMPLES
    )
  );

  uint32_t made = 1u;

  const auto addUnique = [&](uint32_t upTo) {
    for (; made != upTo; ++made) {
      uniqueLattice(world, made);
    }
  };

  addUnique(2000u);

  render(*scene);

  CHECK(reading(*scene, "volumes") == 2000.0);
  CHECK(reading(*scene, "sdfRecords") == 2000.0);
  CHECK(reading(*scene, "droppedVolumes") == 0.0);
  CHECK(drawnEntities(world) == 2000u);

  const uint32_t ceiling = 4095u;
  const uint32_t extra = 5u;

  addUnique(ceiling + extra);

  for (uint32_t frame = 0u; frame != 3u; ++frame) {
    render(*scene);

    CHECK(reading(*scene, "volumes") == double(ceiling));
    CHECK(reading(*scene, "sdfRecords") == double(ceiling));
    CHECK(reading(*scene, "droppedVolumes") == double(extra));
  }

  CHECK(scene->engine->storage().volumePool().capacity() == ceiling);
  CHECK(drawnEntities(world) == ceiling);

  const talus::image::Bitmap shot = shoot(*scene);

  CHECK(surfacePixels(shot, shot.width / 3u, 2u * shot.width / 3u) > 0u);
  REQUIRE(volumeOf(world, front) != UINT32_MAX);

  const talus::field::SampledSdf& frontLattice = *world.sampledSdf(front);

  const std::vector<float> uploaded = brickBoundsOf(
    *scene->engine,
    volumeOf(world, front),
    1u + frontLattice.brickLipschitz.size()
  );

  CHECK(uploaded.front() == frontLattice.globalLipschitz);
  CHECK(
    std::equal(
      frontLattice.brickLipschitz.begin(),
      frontLattice.brickLipschitz.end(),
      uploaded.begin() + 1
    )
  );

  std::vector<Entity> dropped;
  Entity drawn = talus::objectmodel::INVALID_ENTITY;

  for (const Entity entity : world.sdfGrids()) {
    if (!world.registry().all_of<BakedVolume>(entity)) {
      dropped.push_back(entity);
    } else if (entity != front) {
      drawn = entity;
    }
  }

  REQUIRE(dropped.size() == extra);
  REQUIRE(drawn != talus::objectmodel::INVALID_ENTITY);

  REQUIRE(world.destroy(drawn));

  for (uint32_t frame = 0u; frame != 2u; ++frame) {
    render(*scene);

    CHECK(reading(*scene, "volumes") == double(ceiling - 1u));
    CHECK(reading(*scene, "droppedVolumes") == double(extra));
  }

  world.setSampledSdf(
    dropped.front(),
    latticeOf(fixture::sphere(glm::vec3(0.0f), 0.05f), 0.2f, TINY_SAMPLES)
  );

  render(*scene);

  CHECK(reading(*scene, "volumes") == double(ceiling));
  CHECK(reading(*scene, "droppedVolumes") == double(extra - 1u));
  CHECK(volumeOf(world, dropped.front()) != UINT32_MAX);

  const Entity dented = dropped.back();

  world.addStaticBody(dented);

  REQUIRE(world.destroy(drawnExcept(world, front)));

  render(*scene);

  CHECK(reading(*scene, "volumes") == double(ceiling - 1u));
  CHECK(reading(*scene, "droppedVolumes") == double(extra - 1u));

  dent(world, dented);

  render(*scene);

  CHECK(volumeOf(world, dented) != UINT32_MAX);
  CHECK(reading(*scene, "volumes") == double(ceiling));
  CHECK(reading(*scene, "droppedVolumes") == double(extra - 2u));

  struct Kept {
    Entity entity;
    Pose pose;
    SdfGrid grid;
    talus::objectmodel::SharedSdf lattice;
  };

  std::vector<Kept> kept;

  for (const Entity entity : undrawn(world)) {
    kept.push_back(
      {entity,
       world.registry().get<Pose>(entity),
       world.registry().get<SdfGrid>(entity),
       world.registry().get<talus::objectmodel::SharedSdf>(entity)}
    );
  }

  REQUIRE_FALSE(kept.empty());

  scene->engine->storage().residency().releaseAll(world);
  world.clear();

  for (const Kept& held : kept) {
    REQUIRE(world.createSdfGridAt(held.entity, held.pose, held.grid));
    world.adoptSampledSdf(held.entity, held.lattice);
  }

  render(*scene);

  CHECK(reading(*scene, "volumes") == double(kept.size()));
  CHECK(reading(*scene, "droppedVolumes") == 0.0);
  CHECK(drawnEntities(world) == kept.size());
}

TEST_CASE(
  "a grid edit on a lattice-only entity keeps it and its sharer drawn",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const talus::field::SampledSdf round = latticeOf(
    fixture::sphere(glm::vec3(0.0f), 0.5f),
    LATTICE_HALF,
    LATTICE_SAMPLES
  );

  const Entity edited =
    latticeEntity(world, glm::vec3(-0.8f, 0.0f, 0.0f), round);
  const Entity sharer =
    latticeEntity(world, glm::vec3(0.8f, 0.0f, 0.0f), round);

  render(*scene);

  REQUIRE(volumeOf(world, edited) != UINT32_MAX);
  REQUIRE(volumeOf(world, edited) == volumeOf(world, sharer));

  const SurfaceSides before = surfaceSides(*scene);

  REQUIRE(before.left > 0u);
  REQUIRE(before.right > 0u);

  SdfGrid grid = world.registry().get<SdfGrid>(edited);
  grid.resolution = glm::uvec3(EDITED_SAMPLES);
  world.emplaceAuthored(edited, grid);

  render(*scene);

  CHECK(reading(*scene, "bakes") == 0.0);

  const SurfaceSides after = surfaceSides(*scene);

  std::cout << "grid edit surface pixels: edited " << before.left << " -> "
            << after.left << " sharer " << before.right << " -> " << after.right
            << '\n';

  CHECK(after.right == before.right);
  CHECK(after.left == before.left);
  CHECK(volumeOf(world, edited) == volumeOf(world, sharer));
}

TEST_CASE(
  "a primitive added to a lattice-only entity leaves its sharer drawn",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const talus::field::SampledSdf round = latticeOf(
    fixture::sphere(glm::vec3(0.0f), 0.5f),
    LATTICE_HALF,
    LATTICE_SAMPLES
  );

  const Entity grown =
    latticeEntity(world, glm::vec3(-0.8f, 0.0f, 0.0f), round);
  const Entity sharer =
    latticeEntity(world, glm::vec3(0.8f, 0.0f, 0.0f), round);

  render(*scene);

  const uint32_t shared = volumeOf(world, sharer);

  REQUIRE(shared != UINT32_MAX);
  REQUIRE(volumeOf(world, grown) == shared);

  const SurfaceSides before = surfaceSides(*scene);

  REQUIRE(before.right > 0u);

  REQUIRE(
    world.addPrimitive(grown, fixture::box(glm::vec3(0.0f), glm::vec3(0.15f)))
  );

  render(*scene);

  const SurfaceSides after = surfaceSides(*scene);

  std::cout << "added primitive surface pixels: grown " << before.left << " -> "
            << after.left << " sharer " << before.right << " -> " << after.right
            << '\n';

  CHECK(after.right == before.right);
  CHECK(after.left > 0u);
  CHECK(after.left < before.left);
  CHECK(volumeOf(world, sharer) == shared);
  CHECK(volumeOf(world, grown) != shared);
  CHECK(volumeOf(world, grown) != UINT32_MAX);
  CHECK(scene->engine->storage().volumePool().users(shared) == 1u);
}

TEST_CASE(
  "equal primitive entities keep one volume through an uncommitted bake",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const Entity left = boxEntity(world, glm::vec3(-0.8f, 0.0f, 0.0f));
  const Entity right = boxEntity(world, glm::vec3(0.8f, 0.0f, 0.0f));

  talus::engine::FrameRequest stale;
  stale.viewport = scene->viewport;
  stale.deltaSeconds = 1.0f / 60.0f;
  stale.surfaceStale = true;

  scene->telemetry.clear();

  REQUIRE(scene->engine->renderFrame(stale, scene->telemetry));

  const uint32_t shared = volumeOf(world, left);

  REQUIRE(shared != UINT32_MAX);
  REQUIRE(volumeOf(world, right) == shared);
  REQUIRE(world.needsBake(left));
  REQUIRE(world.needsBake(right));

  render(*scene);

  CHECK(volumeOf(world, left) == shared);
  CHECK(volumeOf(world, right) == shared);
  CHECK(scene->engine->storage().volumePool().users(shared) == 2u);
  CHECK(reading(*scene, "volumes") == 1.0);
  CHECK(reading(*scene, "bakes") == 1.0);
  CHECK_FALSE(world.needsBake(left));

  const SurfaceSides drawn = surfaceSides(*scene);

  CHECK(drawn.left > 0u);
  CHECK(drawn.right > 0u);
}

TEST_CASE(
  "equal lattices share again after an in-place write to their old volume",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const talus::field::SampledSdf round = latticeOf(
    fixture::sphere(glm::vec3(0.0f), 0.4f),
    LATTICE_HALF,
    LATTICE_SAMPLES
  );

  const Entity dented =
    latticeEntity(world, glm::vec3(-1.5f, 0.0f, 0.0f), round);
  world.addStaticBody(dented);

  render(*scene);

  const uint32_t held = volumeOf(world, dented);

  REQUIRE(held != UINT32_MAX);
  REQUIRE(scene->engine->storage().volumePool().users(held) == 1u);

  dent(world, dented);

  const Entity first = latticeEntity(world, glm::vec3(0.0f), round);
  const Entity second =
    latticeEntity(world, glm::vec3(1.5f, 0.0f, 0.0f), round);

  render(*scene);

  CHECK(volumeOf(world, dented) == held);
  CHECK(volumeOf(world, first) != held);
  CHECK(volumeOf(world, first) != UINT32_MAX);
  CHECK(volumeOf(world, second) == volumeOf(world, first));
  CHECK(
    scene->engine->storage().volumePool().users(volumeOf(world, first)) == 2u
  );
  CHECK(reading(*scene, "volumes") == 2.0);
}

TEST_CASE(
  "a dent that grows the volume pool keeps every brick bound",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const uint32_t initial = talus::renderer::INITIAL_SDF_VOLUMES;

  const Entity alone = uniqueLattice(world, 0u);
  world.addStaticBody(alone);

  std::vector<std::array<Entity, 2>> pairs;

  for (uint32_t index = 1u; index != initial; ++index) {
    pairs.push_back({uniqueLattice(world, index), uniqueLattice(world, index)});
  }

  const Entity split = pairs.back()[0];
  const Entity kept = pairs.back()[1];
  const Entity untouched = pairs.front()[0];

  world.addStaticBody(split);

  render(*scene);

  const talus::renderer::VolumePool& pool =
    scene->engine->storage().volumePool();

  REQUIRE(pool.allocated() == initial);
  REQUIRE(pool.capacity() == initial);
  REQUIRE(scene->engine->storage().recordTable().volumeCapacity() == initial);
  REQUIRE(pool.users(volumeOf(world, alone)) == 1u);
  REQUIRE(pool.users(volumeOf(world, split)) == 2u);
  REQUIRE(volumeOf(world, kept) == volumeOf(world, split));

  const uint32_t shared = volumeOf(world, split);

  dent(world, alone);
  dent(world, split);

  render(*scene);

  CHECK(pool.capacity() == 2u * initial);
  CHECK(
    scene->engine->storage().recordTable().volumeCapacity() == 2u * initial
  );
  CHECK(pool.allocated() == initial + 1u);
  CHECK(volumeOf(world, kept) == shared);
  CHECK(volumeOf(world, split) >= initial);

  for (const Entity entity : {alone, split, kept, untouched}) {
    CHECK(boundsMatch(
      *scene->engine,
      volumeOf(world, entity),
      *world.sampledSdf(entity)
    ));
  }

  CHECK(
    world.sampledSdf(split)->globalLipschitz
    != world.sampledSdf(kept)->globalLipschitz
  );
}

TEST_CASE(
  "an entity whose last primitive is removed draws nothing and leaves its "
  "sharer drawn",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const Entity emptied = boxEntity(world, glm::vec3(-0.8f, 0.0f, 0.0f));
  const Entity sharer = boxEntity(world, glm::vec3(0.8f, 0.0f, 0.0f));

  render(*scene);

  const uint32_t shared = volumeOf(world, sharer);

  REQUIRE(shared != UINT32_MAX);
  REQUIRE(volumeOf(world, emptied) == shared);

  const SurfaceSides before = surfaceSides(*scene);

  REQUIRE(before.left > 0u);
  REQUIRE(before.right > 0u);

  const talus::field::SampledSdf* sharerLattice = world.sampledSdf(sharer);

  REQUIRE(world.removePrimitive(emptied, 0u));
  REQUIRE(world.primitives(emptied).empty());

  render(*scene);

  const SurfaceSides after = surfaceSides(*scene);

  std::cout << "emptied surface pixels: emptied " << before.left << " -> "
            << after.left << " sharer " << before.right << " -> " << after.right
            << '\n';

  CHECK(after.left == 0u);
  CHECK(after.right == before.right);
  CHECK(volumeOf(world, sharer) == shared);
  CHECK(volumeOf(world, emptied) != shared);
  CHECK(volumeOf(world, emptied) != UINT32_MAX);
  CHECK(scene->engine->storage().volumePool().users(shared) == 1u);
  CHECK(world.sampledSdf(sharer) == sharerLattice);
  CHECK(world.sampledSdf(emptied) != sharerLattice);
}

TEST_CASE(
  "equal primitives on a grid left stale by a resolution edit never share",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  SdfGrid fine{};
  fine.resolution = glm::uvec3(EDITED_SAMPLES);

  const Entity fitted = world.createSdfGrid(
    Pose{glm::vec3(-0.8f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)},
    fine
  );

  REQUIRE(world.addPrimitive(
    fitted,
    fixture::box(glm::vec3(0.0f), glm::vec3(BOX_HALF))
  ));

  const Entity stale = boxEntity(world, glm::vec3(0.8f, 0.0f, 0.0f));

  SdfGrid edited = world.registry().get<SdfGrid>(stale);
  edited.resolution = glm::uvec3(EDITED_SAMPLES);
  world.emplaceAuthored(stale, edited);

  const SdfGrid& fittedGrid = world.registry().get<SdfGrid>(fitted);
  const SdfGrid& staleGrid = world.registry().get<SdfGrid>(stale);

  REQUIRE(fittedGrid.resolution == staleGrid.resolution);
  REQUIRE(fittedGrid.margin == staleGrid.margin);
  REQUIRE(fittedGrid.voxelSize != staleGrid.voxelSize);
  REQUIRE(fittedGrid.origin != staleGrid.origin);
  REQUIRE(world.primitives(fitted).size() == world.primitives(stale).size());
  REQUIRE(
    std::memcmp(
      world.primitives(fitted).data(),
      world.primitives(stale).data(),
      world.primitives(fitted).size_bytes()
    )
    == 0
  );

  render(*scene);

  CHECK(volumeOf(world, fitted) != UINT32_MAX);
  CHECK(volumeOf(world, stale) != UINT32_MAX);
  CHECK(volumeOf(world, fitted) != volumeOf(world, stale));
  CHECK(reading(*scene, "volumes") == 2.0);
}

TEST_CASE(
  "an entity whose primitives were all removed stays empty through undo",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const Emptied held = emptyBesideASphere(*scene);

  talus::undo::UndoStack history(4u);
  history.record(world, "emptied");

  scene->engine->context().device().waitIdle();

  REQUIRE(history.undo(world));

  render(*scene);

  const SurfaceSides restored = surfaceSides(*scene);

  std::cout << "restored surface pixels: emptied " << held.drawn.left << " -> "
            << restored.left << " sphere " << held.drawn.right << " -> "
            << restored.right << '\n';

  CHECK(restored.left == 0u);
  CHECK(restored.right == held.drawn.right);
  CHECK(world.primitives(held.emptied).empty());
  CHECK(holdsNoInterior(world, held.emptied));
}

TEST_CASE(
  "an entity whose primitives were all removed stays empty through Play",
  "[renderer]"
) {
  const auto scene = openScene();

  const Emptied held = emptyBesideASphere(*scene);

  scene->engine->play();

  REQUIRE(scene->engine->playing());

  const World& played = scene->engine->activeWorld();

  REQUIRE(&played != &scene->engine->world());

  render(*scene);

  const SurfaceSides running = surfaceSides(*scene);

  std::cout << "played surface pixels: emptied " << held.drawn.left << " -> "
            << running.left << " sphere " << held.drawn.right << " -> "
            << running.right << '\n';

  CHECK(running.left == 0u);
  CHECK(running.right > 0u);
  CHECK(played.primitives(held.emptied).empty());
  CHECK(holdsNoInterior(played, held.emptied));

  scene->engine->stop();
}

TEST_CASE(
  "a drawn lattice-only entity given another lattice draws the new one",
  "[renderer]"
) {
  const auto scene = openScene();
  World& world = scene->engine->world();

  const talus::field::SampledSdf round = latticeOf(
    fixture::sphere(glm::vec3(0.0f), 0.5f),
    LATTICE_HALF,
    LATTICE_SAMPLES
  );

  const Entity switched =
    latticeEntity(world, glm::vec3(-0.8f, 0.0f, 0.0f), round);

  const Entity donor = latticeEntity(
    world,
    glm::vec3(0.8f, 0.0f, 0.0f),
    latticeOf(
      fixture::sphere(glm::vec3(0.0f), 0.2f),
      LATTICE_HALF,
      LATTICE_SAMPLES
    )
  );

  const Entity sharer =
    latticeEntity(world, glm::vec3(0.0f, 0.0f, 10.0f), round);

  const Entity lone = latticeEntity(
    world,
    glm::vec3(1.5f, 0.0f, 10.0f),
    latticeOf(
      fixture::box(glm::vec3(0.0f), glm::vec3(0.3f)),
      LATTICE_HALF,
      LATTICE_SAMPLES
    )
  );

  const SurfaceSides before = surfaceSides(*scene);

  const uint32_t roundVolume = volumeOf(world, switched);
  const uint32_t smallVolume = volumeOf(world, donor);

  REQUIRE(roundVolume != UINT32_MAX);
  REQUIRE(smallVolume != UINT32_MAX);
  REQUIRE(volumeOf(world, sharer) == roundVolume);
  REQUIRE(volumeOf(world, lone) != UINT32_MAX);
  REQUIRE(reading(*scene, "volumes") == 3.0);
  REQUIRE(before.right > 0u);
  REQUIRE(before.left > 2u * before.right);

  world.shareSampledSdf(donor, switched);
  world.shareSampledSdf(donor, lone);

  const SurfaceSides after = surfaceSides(*scene);

  std::cout << "replaced lattice surface pixels: switched " << before.left
            << " -> " << after.left << " donor " << before.right << " -> "
            << after.right << '\n';

  const talus::renderer::VolumePool& pool =
    scene->engine->storage().volumePool();

  CHECK(after.right == before.right);
  CHECK(after.left > 0u);
  CHECK(2u * after.left < before.left);
  CHECK(volumeOf(world, switched) == smallVolume);
  CHECK(volumeOf(world, lone) == smallVolume);
  CHECK(volumeOf(world, sharer) == roundVolume);
  CHECK(pool.users(smallVolume) == 3u);
  CHECK(pool.users(roundVolume) == 1u);
  CHECK(reading(*scene, "volumes") == 2.0);
  CHECK(boundsMatch(
    *scene->engine,
    volumeOf(world, switched),
    *world.sampledSdf(switched)
  ));
}
