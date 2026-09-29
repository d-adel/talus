#include "engine.ih"

namespace talus::engine {

namespace {

constexpr uint32_t MAX_PHYSICS_SUBSTEPS = 5;

constexpr double MAX_PHYSICS_DELTA = MAX_PHYSICS_SUBSTEPS / 60.0;

talus::gpu::WindowSystem& require(
  const std::unique_ptr<talus::gpu::WindowSystem>& windowSystem
) {
  if (windowSystem == nullptr) {
    throw std::runtime_error("An engine needs a window system");
  }

  return *windowSystem;
}

const std::vector<VkVertexInputBindingDescription> MESH_BINDINGS{
  talus::renderer::Vertex::getBindingDescription()
};

const auto MESH_ATTRIBUTES =
  talus::renderer::Vertex::getAttributeDescriptions();

}

const talus::script::PhysicsVerbs Engine::PHYSICS_VERBS{
  &Engine::onSetRigidBody,
  &Engine::onSetVelocity,
  &Engine::onDestroy,
  &Engine::onRestart
};

namespace {

constexpr const char* CONTACTS = "physics.contacts";

constexpr const char* DEFORMABLE = "physics.deformable";

constexpr float CONTACT_REACH = 0.5f;

const glm::vec4 CONTACT_COLOUR(1.0f, 0.6f, 0.1f, 1.0f);

const glm::vec4 DEFORMABLE_COLOUR(0.3f, 0.85f, 1.0f, 1.0f);

const glm::vec4 ANCHOR_COLOUR(1.0f, 0.2f, 0.5f, 1.0f);

constexpr size_t POINT_LINES = 3u;

talus::gpu::PipelineDescription depthOnly(talus::gpu::PipelineType type) {
  talus::gpu::PipelineDescription described =
    talus::gpu::describeDepthOnly(type);

  if (type == talus::gpu::PipelineType::Mesh) {
    described.bindings = MESH_BINDINGS;
    described.attributes.assign(MESH_ATTRIBUTES.begin(), MESH_ATTRIBUTES.end());
  }

  return described;
}

}

Engine::Engine(
  std::unique_ptr<talus::gpu::WindowSystem> windowSystem,
  const std::filesystem::path& projectRoot
)
    : m_windowSystem(std::move(windowSystem)),
      m_context(require(m_windowSystem)),
      m_swapChain(m_context),
      m_projectRoot(projectRoot),
      m_assetLibrary(m_context, projectRoot),
      m_storage(
        m_context.device(),
        m_assetLibrary.textures(),
        m_assetLibrary.samplers(),
        m_assetLibrary.materials()
      ),
      m_meshPipeline(
        talus::gpu::PipelineType::Mesh,
        m_context.device().vkDevice(),
        setLayouts(talus::gpu::PipelineType::Mesh),
        m_swapChain,
        MESH_BINDINGS,
        MESH_ATTRIBUTES
      ),
      m_sdfPipeline(
        talus::gpu::PipelineType::Sdf,
        m_context.device().vkDevice(),
        setLayouts(talus::gpu::PipelineType::Sdf),
        m_swapChain
      ),
      m_skyPipeline(
        talus::gpu::PipelineType::Sky,
        m_context.device().vkDevice(),
        setLayouts(talus::gpu::PipelineType::Sky),
        m_swapChain
      ),
      m_meshDepthPipeline(
        depthOnly(talus::gpu::PipelineType::Mesh),
        m_context.device().vkDevice(),
        setLayouts(talus::gpu::PipelineType::Mesh),
        m_swapChain
      ),
      m_sdfDepthPipeline(
        depthOnly(talus::gpu::PipelineType::Sdf),
        m_context.device().vkDevice(),
        setLayouts(talus::gpu::PipelineType::Sdf),
        m_swapChain
      ),
      m_lines(m_context.device(), m_swapChain, globalsLayouts()),
      m_renderer(
        m_context.device(),
        m_storage.recordTable(),
        m_storage.sdfBaker(),
        m_storage.volumePool(),
        m_storage.frameGlobals(),
        m_meshPipeline,
        m_sdfPipeline,
        m_meshDepthPipeline,
        m_sdfDepthPipeline,
        m_lines,
        m_storage.resourceTable(),
        m_context.surface().handle(),
        m_swapChain.imageCount()
      ) {
  m_storage.residency().attach(m_world);

  syncMaterials();

  m_debug.show(CONTACTS, false);
  m_debug.show(DEFORMABLE, false);
}

void Engine::syncMaterials() {
  m_world.setMaterials(
    std::vector<talus::objectmodel::PhysicalMaterial>(
      m_assetLibrary.physicalMaterials().begin(),
      m_assetLibrary.physicalMaterials().end()
    )
  );
}

Engine::~Engine() {
  try {
    stop();
  } catch (const std::exception& failure) {
    std::cerr << "Engine teardown: " << failure.what() << std::endl;
  }
}

std::vector<VkDescriptorSetLayout> Engine::setLayouts(
  talus::gpu::PipelineType type
) {
  return talus::renderer::pipelineSetLayouts(
    type,
    m_storage.frameGlobals(),
    m_storage.resourceTable(),
    m_storage.recordTable()
  );
}

std::vector<VkDescriptorSetLayout> Engine::globalsLayouts() {
  return talus::renderer::bindGroupLayouts(
    talus::renderer::globalsBindGroups(),
    m_storage.frameGlobals(),
    m_storage.resourceTable(),
    m_storage.recordTable()
  );
}

const std::filesystem::path& Engine::projectRoot() const noexcept {
  return m_projectRoot;
}

bool Engine::renderFrame(
  const FrameRequest& request,
  talus::core::Telemetry& telemetry
) {
  const VkExtent2D extent = m_swapChain.extent();

  if (extent.width == 0 || extent.height == 0) {
    return false;
  }

  const auto stamp = []() {
    return std::chrono::steady_clock::now();
  };
  const auto since = [](auto from) {
    return 1000.0
           * std::chrono::duration<double>(
               std::chrono::steady_clock::now() - from
           )
               .count();
  };

  auto mark = stamp();

  m_storage.uploader().retire();

  telemetry.set(telemetry.key("retire"), since(mark));

  m_input.setViewport(
    static_cast<float>(extent.width),
    static_cast<float>(extent.height)
  );

  mark = stamp();

  tick(request.deltaSeconds, telemetry);

  telemetry.set(telemetry.key("tick"), since(mark));

  mark = stamp();

  endFrame();

  telemetry.set(telemetry.key("endFrame"), since(mark));

  lookThroughViewport(
    request.viewport,
    static_cast<float>(extent.width) / static_cast<float>(extent.height)
  );

  mark = stamp();

  flushVolumes(telemetry);

  telemetry.set(telemetry.key("flushVolumes"), since(mark));

  mark = stamp();

  packFrame();

  telemetry.set(telemetry.key("packFrame"), since(mark));

  if (request.surfaceStale) {
    return true;
  }

  std::vector<talus::renderer::ScenePass> passes;

  if (m_frame.environment.has_value()) {
    passes.push_back(
      {talus::renderer::PassOrder::BeforeScene,
       [this](VkCommandBuffer commands) { drawSky(commands); }}
    );
  }

  passes.insert(passes.end(), request.passes.begin(), request.passes.end());

  m_frame.selected = request.selected;
  m_frame.debugTested = m_debug.tested();
  m_frame.debugUntested = m_debug.untested();
  m_frame.overlayLines = request.overlay;

  m_renderer.submittedAt(uint64_t(m_frameIndex));

  mark = stamp();

  const bool stale =
    m_renderer.drawFrame(m_swapChain, m_frame, passes, request.capture);

  telemetry.set(telemetry.key("drawFrame"), since(mark));

  const talus::renderer::Renderer::Timing& inner = m_renderer.timing();

  telemetry.set(telemetry.key("fenceWait"), inner.fenceWaitMs);
  telemetry.set(telemetry.key("acquire"), inner.acquireMs);
  telemetry.set(telemetry.key("record"), inner.recordMs);
  telemetry.set(telemetry.key("bake"), inner.bakeMs);
  telemetry.set(telemetry.key("submit"), inner.submitMs);
  telemetry.set(telemetry.key("present"), inner.presentMs);
  telemetry.set(telemetry.key("bakes"), double(inner.bakes));
  telemetry.set(telemetry.key("recordTable"), inner.recordTableMs);
  telemetry.set(telemetry.key("shadowBuild"), inner.shadowBuildMs);
  telemetry.set(telemetry.key("shadowIndices"), double(inner.shadowIndices));
  telemetry.set(telemetry.key("shadowBinned"), inner.shadowBinned ? 1.0 : 0.0);
  telemetry.set(telemetry.key("sdfRecords"), double(m_frame.sdfRecordCount));
  telemetry.set(
    telemetry.key("droppedVolumes"),
    double(m_frame.droppedVolumes)
  );
  telemetry.set(
    telemetry.key("volumes"),
    double(m_storage.volumePool().allocated())
  );

  if (inner.gpuValid) {
    const std::array<const char*, 11> segments{
      "gpuEntry",
      "gpuSky",
      "gpuMesh",
      "gpuTable",
      "gpuSdf",
      "gpuSceneEnd",
      "gpuSelection",
      "gpuOverlay",
      "gpuBlit",
      "gpuPresentBarrier",
      "gpuTail"
    };

    double named = 0.0;

    for (size_t at = 0u; at != segments.size(); ++at) {
      telemetry.set(telemetry.key(segments[at]), inner.gpuSegmentMs[at]);

      named += inner.gpuSegmentMs[at];
    }

    telemetry.set(telemetry.key("gpuTotal"), inner.gpuTotalMs);
    telemetry.set(telemetry.key("gpuUnnamed"), inner.gpuTotalMs - named);
    telemetry.set(telemetry.key("gpuFrame"), double(inner.gpuFrame));
  }

  mark = stamp();

  if (!stale) {
    m_storage.framePacker().commitBakes();
  }

  telemetry.set(telemetry.key("commitBakes"), since(mark));
  telemetry.set(telemetry.key("frameIndex"), double(m_frameIndex));

  return stale;
}

void Engine::lookThroughViewport(talus::view::ViewportId id, float aspect) {
  const talus::view::Viewport* port = m_viewports.find(id);

  if (port == nullptr) {
    m_frame.drawMeshes = false;
    m_frame.drawSdf = false;

    return;
  }

  const talus::objectmodel::CameraView seen =
    talus::view::lookThrough(*port, activeWorld(), aspect);

  m_frame.view = seen.view;
  m_frame.proj = seen.projection;
  m_frame.cameraPos = glm::vec4(seen.position, seen.nearPlane);

  m_frame.drawMeshes = drawsMeshes(port->mode);
  m_frame.drawSdf = drawsSdf(port->mode);

  m_frame.fieldRepresentation = port->fieldRepresentation;

  m_frame.march.epsilon = port->march.epsilon;
  m_frame.march.maxDistance = port->march.maxDistance;
  m_frame.march.omega = port->march.omega;
  m_frame.march.gradientEpsilon = port->march.gradientEpsilon;
  m_frame.march.shadowMaxDistance = port->march.shadowMaxDistance;
  m_frame.march.shadowSharpness = port->march.shadowSharpness;
  m_frame.march.maxIterations = port->march.maxIterations;
}

talus::view::ViewportId Engine::createViewport() {
  return m_viewports.create();
}

bool Engine::destroyViewport(talus::view::ViewportId id) {
  return m_viewports.destroy(id);
}

bool Engine::configureViewport(
  talus::view::ViewportId id,
  const talus::view::Viewport& config
) {
  return m_viewports.configure(id, config);
}

const talus::view::Viewport* Engine::viewport(
  talus::view::ViewportId id
) const {
  return m_viewports.find(id);
}

void Engine::loadWorld(const std::string& world) {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_projectRoot);

  if (!project.has_value()) {
    throw std::runtime_error("No project at " + m_projectRoot.string());
  }

  talus::serialize::StoredWorld stored;

  if (!project->loadWorld(world, stored)) {
    throw std::runtime_error("No world named " + world);
  }

  if (!talus::serialize::restoreWorld(
        stored,
        m_world,
        m_assetLibrary.assets()
      )) {
    throw std::runtime_error("The world " + world + " did not load");
  }
}

void Engine::play() {
  if (m_runtime) {
    return;
  }

  m_storage.residency().releaseAll(m_world);

  m_runtime.emplace(m_world, m_jolt);

  ++m_playGeneration;

  m_storage.residency().attach(m_runtime->world());

  m_physicsScope = talus::script::PhysicsScope(&Engine::PHYSICS_VERBS, this);
}

void Engine::stop() {
  m_restartPending = false;

  if (!m_runtime) {
    return;
  }

  m_storage.residency().releaseAll(m_runtime->world());

  m_physicsScope = {};

  m_pendingBodies.clear();
  m_accumulator = 0.0;
  m_stepOnce = false;

  m_runtime.reset();
}

bool Engine::playing() const noexcept {
  return m_runtime.has_value();
}

void Engine::setFixedStep(std::optional<float> seconds) {
  if (seconds.has_value() && !(*seconds > 0.0f)) {
    throw std::runtime_error("a fixed step must be greater than zero");
  }

  m_fixedStep = seconds;

  m_swapChain.setUncapped(m_fixedStep.has_value());
}

std::optional<float> Engine::fixedStep() const noexcept {
  return m_fixedStep;
}

void Engine::setPaused(bool paused) noexcept {
  m_paused = paused;
}

bool Engine::paused() const noexcept {
  return m_paused;
}

void Engine::stepOnce() noexcept {
  m_stepOnce = true;
}

talus::objectmodel::World& Engine::world() noexcept {
  return m_world;
}

const talus::objectmodel::World& Engine::world() const noexcept {
  return m_world;
}

talus::objectmodel::World& Engine::activeWorld() noexcept {
  return m_runtime ? m_runtime->world() : m_world;
}

const talus::objectmodel::World& Engine::activeWorld() const noexcept {
  return m_runtime ? m_runtime->world() : m_world;
}

talus::assets::AssetLibrary& Engine::assets() noexcept {
  return m_assetLibrary;
}

const talus::assets::AssetLibrary& Engine::assets() const noexcept {
  return m_assetLibrary;
}

talus::gpu::Context& Engine::context() noexcept {
  return m_context;
}

const talus::gpu::Context& Engine::context() const noexcept {
  return m_context;
}

talus::gpu::SwapChain& Engine::swapChain() noexcept {
  return m_swapChain;
}

const talus::gpu::SwapChain& Engine::swapChain() const noexcept {
  return m_swapChain;
}

talus::gpu::WindowSystem& Engine::windowSystem() noexcept {
  return require(m_windowSystem);
}

talus::renderer::RendererStorage& Engine::storage() noexcept {
  return m_storage;
}

const talus::renderer::RendererStorage& Engine::storage() const noexcept {
  return m_storage;
}

talus::renderer::Renderer& Engine::renderer() noexcept {
  return m_renderer;
}

const talus::renderer::Renderer& Engine::renderer() const noexcept {
  return m_renderer;
}

talus::systems::Schedule& Engine::schedule() noexcept {
  return m_schedule;
}

talus::debugdraw::DebugDraw& Engine::debug() noexcept {
  return m_debug;
}

talus::systems::InputState& Engine::input() noexcept {
  return m_input;
}

const talus::systems::InputState& Engine::input() const noexcept {
  return m_input;
}

talus::runtime::Runtime* Engine::runtime() noexcept {
  return m_runtime ? &*m_runtime : nullptr;
}

uint32_t Engine::frameIndex() const noexcept {
  return m_frameIndex;
}

void Engine::stepPhysics(
  float deltaSeconds,
  talus::core::Telemetry& telemetry
) {
  if (!m_runtime) {
    return;
  }

  m_accumulator += std::min(double(deltaSeconds), MAX_PHYSICS_DELTA);

  uint32_t substeps = 0;

  const auto started = std::chrono::steady_clock::now();

  while (m_accumulator >= talus::physics::PhysicsWorld::TIME_STEP
         && substeps != MAX_PHYSICS_SUBSTEPS) {
    ++substeps;
    m_runtime->step();

    m_accumulator -= talus::physics::PhysicsWorld::TIME_STEP;
  }

  telemetry.set(
    telemetry.key("physics"),
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started
    )
      .count()
  );

  telemetry.set(telemetry.key("substeps"), double(substeps));

  telemetry.set(
    telemetry.key("awake"),
    double(m_runtime->physics().system().GetNumActiveBodies(
      JPH::EBodyType::RigidBody
    ))
  );

  m_runtime->collectImpacts();
}

void Engine::drawDeformables(talus::core::Telemetry& telemetry) {
  if (!m_debug.shown(DEFORMABLE)) {
    telemetry.set(telemetry.key("wireframe"), 0.0);
    telemetry.set(telemetry.key("wireframeSkipped"), 0.0);

    return;
  }

  size_t drawn = 0u;
  uint32_t skipped = 0u;

  const auto meshes = activeWorld()
                        .registry()
                        .view<
                          talus::objectmodel::DeformablePrepared,
                          talus::objectmodel::DeformableState>();

  for (const talus::objectmodel::Entity entity : meshes) {
    const talus::objectmodel::PreparedMesh* mesh =
      meshes.get<talus::objectmodel::DeformablePrepared>(entity).mesh.get();

    if (mesh == nullptr) {
      continue;
    }

    const talus::objectmodel::DeformableState& state =
      meshes.get<talus::objectmodel::DeformableState>(entity);

    size_t anchored = 0u;

    for (const uint8_t held : mesh->anchored) {
      if (held != 0u) {
        ++anchored;
      }
    }

    const size_t need = mesh->surfaceEdges.size() + POINT_LINES * anchored;

    if (m_debug.count() + need > talus::debugdraw::MAX_LINES) {
      ++skipped;
      continue;
    }

    for (const glm::uvec2& edge : mesh->surfaceEdges) {
      if (
        m_debug.line(
          state.position[edge.x],
          state.position[edge.y],
          DEFORMABLE_COLOUR,
          0.0f,
          DEFORMABLE
        )
      ) {
        ++drawn;
      }
    }

    const float size = glm::compMax(mesh->voxelSize);

    for (size_t node = 0u; node != mesh->anchored.size(); ++node) {
      if (mesh->anchored[node] == 0u) {
        continue;
      }

      if (
        m_debug.point(
          state.position[node],
          size,
          ANCHOR_COLOUR,
          0.0f,
          DEFORMABLE
        )
      ) {
        drawn += POINT_LINES;
      }
    }
  }

  telemetry.set(telemetry.key("wireframe"), double(drawn));
  telemetry.set(telemetry.key("wireframeSkipped"), double(skipped));
}

void Engine::tick(float deltaSeconds, talus::core::Telemetry& telemetry) {
  ++m_frameIndex;

  if (m_fixedStep.has_value()) {
    deltaSeconds = *m_fixedStep;
  }

  m_debug.advance(deltaSeconds);

  if (!m_runtime) {
    return;
  }

  if (m_paused && !m_stepOnce) {
    m_runtime->applyPoseEdits();
    m_runtime->syncPoses(
      float(m_accumulator / talus::physics::PhysicsWorld::TIME_STEP)
    );
    drawDeformables(telemetry);
    processRestart();
    return;
  }

  if (m_stepOnce) {
    m_stepOnce = false;
    deltaSeconds = float(talus::physics::PhysicsWorld::TIME_STEP);
  }

  m_runtime->beginFrame();

  runSystems(talus::systems::Phase::PrePhysics, deltaSeconds);

  m_runtime->applyPoseEdits();

  stepPhysics(deltaSeconds, telemetry);

  if (m_debug.shown(CONTACTS)) {
    for (const talus::objectmodel::Impact& hit : m_runtime->impacts()) {
      m_debug.ray(
        hit.point,
        hit.outward * CONTACT_REACH,
        CONTACT_COLOUR,
        0.0f,
        CONTACTS
      );
    }
  }

  drawDeformables(telemetry);

  runSystems(talus::systems::Phase::PostPhysics, deltaSeconds);

  applyPendingBodies();

  m_runtime->refreshDeformedBodies();

  m_runtime->syncPoses(
    float(m_accumulator / talus::physics::PhysicsWorld::TIME_STEP)
  );

  runSystems(talus::systems::Phase::PostUpdateWork, deltaSeconds);

  processRestart();
}

void Engine::processRestart() {
  if (!std::exchange(m_restartPending, false)) {
    return;
  }

  stop();
  play();
}

void Engine::runSystems(talus::systems::Phase phase, float deltaSeconds) {
  const talus::query::Context queries{
    activeWorld(),
    m_runtime ? &m_runtime->physics() : nullptr
  };

  talus::systems::Context context{
    activeWorld(),
    m_input,
    deltaSeconds,
    m_frameIndex,
    m_runtime ? m_runtime->impacts()
              : std::span<const talus::objectmodel::Impact>{},
    &m_debug,
    &queries
  };

  m_schedule.run(phase, context);
}

void Engine::flushVolumes(talus::core::Telemetry& telemetry) {
  m_storage.residency().flush(activeWorld(), telemetry);
}

void Engine::packFrame(bool holdBakes) {
  m_frame.environment.reset();
  m_frame.light.reset();

  m_storage.framePacker().pack(
    m_frame,
    activeWorld(),
    holdBakes ? std::span<const talus::objectmodel::Entity>()
              : activeWorld().sdfGrids(),
    m_assetLibrary.meshBuffers(),
    [this](talus::objectmodel::Entity entity) {
      if (m_runtime) {
        m_runtime->refreshBody(entity);
      }
    }
  );
}

void Engine::endFrame() noexcept {
  m_input.beginFrame();
}

void Engine::drawSky(VkCommandBuffer commands) const {
  vkCmdBindPipeline(
    commands,
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_skyPipeline.pipeline()
  );

  const VkDescriptorSet set = globals();

  vkCmdBindDescriptorSets(
    commands,
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_skyPipeline.pipelineLayout(),
    0,
    1,
    &set,
    0,
    nullptr
  );

  vkCmdDraw(commands, 3, 1, 0, 0);
}

VkDescriptorSet Engine::globals() const noexcept {
  return m_storage.frameGlobals().descriptorSet(m_renderer.currentFrame());
}

void Engine::resize() {
  m_swapChain.recreate();
}

void Engine::retarget(std::unique_ptr<talus::gpu::WindowSystem> windowSystem) {
  if (windowSystem == nullptr) {
    throw std::runtime_error("An engine needs a window system");
  }

  m_swapChain.release();

  m_windowSystem = std::move(windowSystem);

  m_context.retarget(*m_windowSystem);

  m_swapChain.recreate();
}

int32_t Engine::onSetRigidBody(void* host, void*, uint32_t entity, float mass) {
  auto* live = static_cast<Engine*>(host);

  if (live == nullptr || !live->m_runtime) {
    return 0;
  }

  const auto subject =
    static_cast<talus::objectmodel::Entity>(entt::entity{entity});

  if (!live->m_runtime->world().registry().valid(subject)) {
    return 0;
  }

  live->m_runtime->world().addRigidBody(subject);

  live->remember(subject).mass = mass;

  return 1;
}

int32_t Engine::onSetVelocity(
  void* host,
  void*,
  uint32_t entity,
  const float* velocity
) {
  auto* live = static_cast<Engine*>(host);

  if (live == nullptr || velocity == nullptr || !live->m_runtime) {
    return 0;
  }

  const auto subject =
    static_cast<talus::objectmodel::Entity>(entt::entity{entity});

  if (!live->m_runtime->world().registry().valid(subject)) {
    return 0;
  }

  live->remember(subject).velocity =
    glm::vec3(velocity[0], velocity[1], velocity[2]);

  return 1;
}

int32_t Engine::onDestroy(void* host, void*, uint32_t entity) {
  auto* live = static_cast<Engine*>(host);

  if (live == nullptr || !live->m_runtime) {
    return 0;
  }

  const auto subject =
    static_cast<talus::objectmodel::Entity>(entt::entity{entity});

  std::erase_if(live->m_pendingBodies, [subject](const PendingBody& waiting) {
    return waiting.entity == subject;
  });

  return live->m_runtime->destroy(subject) ? 1 : 0;
}

int32_t Engine::onRestart(void* host, void* world) {
  auto* live = static_cast<Engine*>(host);

  if (
    live == nullptr || !live->m_runtime || world != &live->m_runtime->world()
  ) {
    return 0;
  }

  live->m_restartPending = true;

  return 1;
}

Engine::PendingBody& Engine::remember(talus::objectmodel::Entity entity) {
  for (PendingBody& waiting : m_pendingBodies) {
    if (waiting.entity == entity) {
      return waiting;
    }
  }

  m_pendingBodies.push_back({entity, std::nullopt, std::nullopt});

  return m_pendingBodies.back();
}

void Engine::applyPendingBodies() {
  if (!m_runtime || m_pendingBodies.empty()) {
    return;
  }

  std::vector<PendingBody> waiting;

  for (const PendingBody& body : m_pendingBodies) {
    if (!m_runtime->world().registry().valid(body.entity)) {
      continue;
    }

    if (!m_runtime->world().hasSampledSdf(body.entity)) {
      waiting.push_back(body);

      continue;
    }

    m_runtime->refreshBody(body.entity);

    if (body.mass.has_value()) {
      m_runtime->setMass(body.entity, *body.mass);
    }

    if (body.velocity.has_value()) {
      m_runtime->setLinearVelocity(body.entity, *body.velocity);
    }
  }

  m_pendingBodies = std::move(waiting);
}

}
