#pragma once

#include <talus/assets/assetlibrary/assetlibrary.h>
#include <talus/core/telemetry/telemetry.h>
#include <talus/debugdraw/debugdraw/debugdraw.h>
#include <talus/gpu/context/context.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/gpu/swapchain/swapchain.h>
#include <talus/gpu/windowsystem/windowsystem.h>
#include <talus/objectmodel/world/world.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/renderer/frame/frame.h>
#include <talus/renderer/lines/lines.h>
#include <talus/renderer/renderer.h>
#include <talus/renderer/rendererstorage/rendererstorage.h>
#include <talus/runtime/runtime/runtime.h>
#include <talus/script/api/api.h>
#include <talus/systems/input/input.h>
#include <talus/systems/schedule/schedule.h>
#include <talus/view/lookthrough/lookthrough.h>
#include <talus/view/viewportstore/viewportstore.h>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <span>
#include <optional>
#include <string>
#include <vector>

namespace talus::engine {

struct FrameRequest {
  talus::view::ViewportId viewport = talus::view::INVALID_VIEWPORT;

  float deltaSeconds = 0.0f;

  bool surfaceStale = false;

  talus::objectmodel::Entity selected = talus::objectmodel::INVALID_ENTITY;

  std::span<const talus::renderer::ScenePass> passes = {};

  std::span<const talus::debugdraw::LineVertex> overlay = {};

  std::function<void(VkImage)> capture{};
};

class Engine {
public:
  Engine(
    std::unique_ptr<talus::gpu::WindowSystem> windowSystem,
    const std::filesystem::path& projectRoot
  );
  ~Engine();

  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;
  Engine(Engine&&) = delete;
  Engine& operator=(Engine&&) = delete;

  void loadWorld(const std::string& world);

  void syncMaterials();

  [[nodiscard]] talus::view::ViewportId createViewport();

  [[nodiscard]] bool destroyViewport(talus::view::ViewportId id);

  [[nodiscard]] bool configureViewport(
    talus::view::ViewportId id,
    const talus::view::Viewport& config
  );

  [[nodiscard]] const talus::view::Viewport* viewport(
    talus::view::ViewportId id
  ) const;

  [[nodiscard]] const std::filesystem::path& projectRoot() const noexcept;

  void play();

  void stop();

  [[nodiscard]] bool playing() const noexcept;

  void setFixedStep(std::optional<float> seconds);

  [[nodiscard]] std::optional<float> fixedStep() const noexcept;

  void setPaused(bool paused) noexcept;
  [[nodiscard]] bool paused() const noexcept;
  void stepOnce() noexcept;

  [[nodiscard]] talus::objectmodel::World& world() noexcept;
  [[nodiscard]] const talus::objectmodel::World& world() const noexcept;

  [[nodiscard]] talus::objectmodel::World& activeWorld() noexcept;
  [[nodiscard]] const talus::objectmodel::World& activeWorld() const noexcept;

  [[nodiscard]] talus::assets::AssetLibrary& assets() noexcept;
  [[nodiscard]] const talus::assets::AssetLibrary& assets() const noexcept;

  [[nodiscard]] talus::gpu::Context& context() noexcept;
  [[nodiscard]] const talus::gpu::Context& context() const noexcept;

  [[nodiscard]] talus::gpu::SwapChain& swapChain() noexcept;
  [[nodiscard]] const talus::gpu::SwapChain& swapChain() const noexcept;

  [[nodiscard]] talus::gpu::WindowSystem& windowSystem() noexcept;

  [[nodiscard]] talus::renderer::RendererStorage& storage() noexcept;
  [[nodiscard]] const talus::renderer::RendererStorage&
  storage() const noexcept;

  [[nodiscard]] talus::renderer::Renderer& renderer() noexcept;
  [[nodiscard]] const talus::renderer::Renderer& renderer() const noexcept;

  [[nodiscard]] talus::systems::Schedule& schedule() noexcept;

  [[nodiscard]] talus::debugdraw::DebugDraw& debug() noexcept;

  [[nodiscard]] talus::systems::InputState& input() noexcept;
  [[nodiscard]] const talus::systems::InputState& input() const noexcept;

  [[nodiscard]] talus::runtime::Runtime* runtime() noexcept;

  [[nodiscard]] uint32_t frameIndex() const noexcept;

  [[nodiscard]] bool renderFrame(
    const FrameRequest& request,
    talus::core::Telemetry& telemetry
  );

  [[nodiscard]] VkDescriptorSet globals() const noexcept;

  void resize();

  void retarget(std::unique_ptr<talus::gpu::WindowSystem> windowSystem);

  [[nodiscard]] std::vector<VkDescriptorSetLayout> setLayouts(
    talus::gpu::PipelineType type
  );

  [[nodiscard]] std::vector<VkDescriptorSetLayout> globalsLayouts();

private:
  friend struct RestartTestAccess;

  static int32_t onSetRigidBody(
    void* host,
    void* world,
    uint32_t entity,
    float mass
  );

  static int32_t onSetVelocity(
    void* host,
    void* world,
    uint32_t entity,
    const float* velocity
  );

  static int32_t onDestroy(void* host, void* world, uint32_t entity);

  static int32_t onRestart(void* host, void* world);

  static const talus::script::PhysicsVerbs PHYSICS_VERBS;

  struct PendingBody {
    talus::objectmodel::Entity entity;
    std::optional<float> mass;
    std::optional<glm::vec3> velocity;
  };

  PendingBody& remember(talus::objectmodel::Entity entity);

  void applyPendingBodies();

  void lookThroughViewport(talus::view::ViewportId id, float aspect);

  void tick(float deltaSeconds, talus::core::Telemetry& telemetry);

  void processRestart();

  void flushVolumes(talus::core::Telemetry& telemetry);

  void packFrame(bool holdBakes = false);

  void endFrame() noexcept;

  void drawSky(VkCommandBuffer commands) const;

  void stepPhysics(float deltaSeconds, talus::core::Telemetry& telemetry);

  void drawDeformables(talus::core::Telemetry& telemetry);

  void runSystems(talus::systems::Phase phase, float deltaSeconds);

  std::unique_ptr<talus::gpu::WindowSystem> m_windowSystem;
  talus::gpu::Context m_context;
  talus::gpu::SwapChain m_swapChain;

  std::filesystem::path m_projectRoot;

  talus::assets::AssetLibrary m_assetLibrary;

  talus::objectmodel::World m_world;

  talus::physics::JoltLibrary m_jolt;
  std::optional<talus::runtime::Runtime> m_runtime;

  uint64_t m_playGeneration = 0;
  bool m_restartPending = false;

  talus::script::PhysicsScope m_physicsScope;

  std::vector<PendingBody> m_pendingBodies;

  talus::systems::Schedule m_schedule;
  talus::systems::InputState m_input;

  talus::view::ViewportStore m_viewports;

  uint32_t m_frameIndex = 0;

  double m_accumulator = 0.0;
  std::optional<float> m_fixedStep;
  bool m_paused = false;
  bool m_stepOnce = false;

  talus::renderer::RendererStorage m_storage;

  talus::gpu::Pipeline m_meshPipeline;
  talus::gpu::Pipeline m_sdfPipeline;
  talus::gpu::Pipeline m_skyPipeline;
  talus::gpu::Pipeline m_meshDepthPipeline;
  talus::gpu::Pipeline m_sdfDepthPipeline;

  talus::renderer::Lines m_lines;

  talus::renderer::Renderer m_renderer;

  talus::renderer::Frame m_frame{};

  talus::debugdraw::DebugDraw m_debug;
};

}
