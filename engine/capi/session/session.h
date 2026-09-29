#pragma once

#include <capi/sceneview/sceneview.h>

#include <talus/engine/engine/engine.h>

#include <talus/gpu/windowsystem/windowsystem.h>
#include <talus/field/field.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/serialize/componentschema/componentschema.h>
#include <talus/undo/undostack/undostack.h>

#include <chrono>
#include <cstddef>
#include <span>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace talus::capi {

class Session {
public:
  Session(
    std::unique_ptr<talus::gpu::WindowSystem> windowSystem,
    const std::filesystem::path& projectRoot,
    const std::string& world
  );
  ~Session();

  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;
  Session(Session&&) = delete;
  Session& operator=(Session&&) = delete;

  void resize();

  void retarget(std::unique_ptr<talus::gpu::WindowSystem> windowSystem);

  void render();

  [[nodiscard]] VkExtent2D extent() const noexcept;

  [[nodiscard]] const talus::objectmodel::World& world() const noexcept;

  [[nodiscard]] talus::objectmodel::World& world() noexcept;

  [[nodiscard]] talus::systems::Schedule& schedule() noexcept;

  void play();

  void stop();

  [[nodiscard]] bool playing() const noexcept;

  void setFixedStep(std::optional<float> seconds);

  [[nodiscard]] std::optional<float> fixedStep() const noexcept;

  void setPaused(bool paused) noexcept;
  [[nodiscard]] bool paused() const noexcept;
  void stepOnce() noexcept;

  [[nodiscard]] talus::objectmodel::World& activeWorld() noexcept;

  [[nodiscard]] const talus::objectmodel::World& activeWorld() const noexcept;

  [[nodiscard]] talus::view::ViewportId createView();

  [[nodiscard]] bool destroyView(talus::view::ViewportId id);

  [[nodiscard]] SceneView* view(talus::view::ViewportId id) noexcept;

  [[nodiscard]] const SceneView* view(
    talus::view::ViewportId id
  ) const noexcept;

  [[nodiscard]] SceneView& mainView() noexcept;

  [[nodiscard]] const SceneView& mainView() const noexcept;

  void setKey(uint32_t key, bool down) noexcept;

  void setMouseButton(uint32_t button, bool down) noexcept;

  void setCursor(float x, float y) noexcept;

  void showDebug(const std::string& category, bool shown);

  [[nodiscard]] bool package(
    const std::string& playerExecutable,
    const std::string& output,
    const std::vector<std::string>& worlds,
    std::string& result
  ) const;

  [[nodiscard]] talus::objectmodel::Entity createLight();

  [[nodiscard]] talus::objectmodel::Entity createEnvironment();

  [[nodiscard]] talus::objectmodel::Entity createCamera(
    const glm::vec3& position,
    const glm::vec3& target,
    float verticalFov
  );

  [[nodiscard]] talus::objectmodel::Entity createSdf(
    const talus::objectmodel::Pose& pose,
    const glm::uvec3& resolution,
    float margin
  );

  [[nodiscard]] bool addPrimitive(
    talus::objectmodel::Entity entity,
    const talus::field::Primitive& primitive
  );

  void setStatic(talus::objectmodel::Entity entity);
  void setDeformable(talus::objectmodel::Entity entity);

  [[nodiscard]] bool destroyEntity(talus::objectmodel::Entity entity);
  [[nodiscard]] std::vector<std::string> componentNames() const;
  [[nodiscard]] bool addComponent(
    talus::objectmodel::Entity entity,
    std::string_view component
  );
  [[nodiscard]] bool removeComponent(
    talus::objectmodel::Entity entity,
    std::string_view component
  );
  [[nodiscard]] std::string removalBlocker(
    talus::objectmodel::Entity entity,
    std::string_view component
  ) const;
  [[nodiscard]] bool canAddComponent(
    talus::objectmodel::Entity entity,
    std::string_view component
  ) const;
  [[nodiscard]] std::optional<talus::serialize::ComponentSchema>
  componentSchema(std::string_view component) const;
  [[nodiscard]] std::optional<std::vector<std::byte>> componentBytes(
    talus::objectmodel::Entity entity,
    std::string_view component
  ) const;
  [[nodiscard]] talus::serialize::WriteStatus setField(
    talus::objectmodel::Entity entity,
    std::string_view component,
    uint32_t field,
    talus::objectmodel::FieldKind kind,
    std::span<const std::byte> bytes
  );

  [[nodiscard]] bool record(std::string label);

  [[nodiscard]] uint32_t drainChanges(
    std::span<uint32_t> touched,
    bool& wholesale
  );

  [[nodiscard]] bool commit();

  [[nodiscard]] bool discard();

  [[nodiscard]] bool undo();

  [[nodiscard]] bool redo();

  [[nodiscard]] std::optional<std::string_view> undoLabel() const noexcept;

  [[nodiscard]] std::optional<std::string_view> redoLabel() const noexcept;

  [[nodiscard]] bool save() const;

  [[nodiscard]] bool openWorld(const std::string& name);

  [[nodiscard]] bool newWorld(const std::string& name);

  [[nodiscard]] bool saveAs(const std::string& name);

  [[nodiscard]] const std::string& worldName() const noexcept;

  [[nodiscard]] size_t materialCount() const noexcept;

  [[nodiscard]] talus::core::AssetId materialAt(uint32_t index) const noexcept;

  [[nodiscard]] uint32_t materialIndex(talus::core::AssetId id) const noexcept;

  [[nodiscard]] talus::core::AssetId addMaterial(
    const glm::vec4& baseColor,
    float metallic,
    float roughness
  );

  [[nodiscard]] talus::objectmodel::PhysicalMaterial materialPhysics(
    talus::core::AssetId id
  ) const;

  void setMaterialPhysics(
    talus::core::AssetId id,
    const talus::objectmodel::PhysicalMaterial& physical
  );

private:
  static constexpr size_t UNDO_DEPTH = 32;

  void resizeTargets();

  talus::engine::Engine m_engine;

  talus::undo::UndoStack m_history{UNDO_DEPTH};
  size_t m_playMark = 0u;

  std::string m_worldName;

  std::vector<std::unique_ptr<SceneView>> m_views;

  std::chrono::steady_clock::time_point m_lastFrame =
    std::chrono::steady_clock::now();
};

}
