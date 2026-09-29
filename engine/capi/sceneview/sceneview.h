#pragma once

#include <talus/engine/engine/engine.h>
#include <talus/field/field.h>
#include <talus/gizmos/grid/grid.h>
#include <talus/gizmos/handles/handles.h>
#include <talus/gizmos/outline/outline.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/entity/entity.h>
#include <talus/renderer/scenetarget/scenetarget.h>
#include <talus/view/camera/camera.h>
#include <talus/view/viewport/viewport.h>

#include <vulkan/vulkan.h>

#include <functional>
#include <span>

namespace talus::capi {

class SceneView {
public:
  explicit SceneView(talus::engine::Engine& engine);

  ~SceneView();

  SceneView(const SceneView&) = delete;
  SceneView& operator=(const SceneView&) = delete;
  SceneView(SceneView&&) = delete;
  SceneView& operator=(SceneView&&) = delete;

  [[nodiscard]] talus::view::ViewportId id() const noexcept;

  [[nodiscard]] const talus::view::Viewport& config() const noexcept;

  void configure(const talus::view::Viewport& settings);

  void orbit(float deltaYaw, float deltaPitch);

  void pan(float deltaX, float deltaY);

  void zoom(float delta);

  void frameOn(talus::objectmodel::Entity entity);

  void frameOnWorld();

  void alignToSceneCamera();

  void resetCamera() noexcept;

  [[nodiscard]] bool throughScene() const noexcept;

  void setThroughScene(bool through) noexcept;

  void bind();

  [[nodiscard]] talus::objectmodel::Entity pick(float x, float y) const;

  void placeHandles(talus::objectmodel::Entity entity);

  [[nodiscard]] bool handlesShown() const noexcept;

  [[nodiscard]] uint32_t handleAxis(float x, float y) const;

  void highlightHandle(uint32_t axis) noexcept;

  [[nodiscard]] bool startHandleDrag(uint32_t axis, float x, float y);

  [[nodiscard]] bool moveHandles(float x, float y);

  void endHandleDrag() noexcept;

  using PoseSink = std::function<void(
    talus::objectmodel::Entity,
    const talus::objectmodel::Pose&,
    const talus::objectmodel::Pose&
  )>;

  void onPoseEdited(PoseSink sink);

  [[nodiscard]] std::span<const talus::debugdraw::LineVertex>
  handleLines() const noexcept;

  [[nodiscard]] bool outlineShown() const noexcept;

  [[nodiscard]] talus::objectmodel::Entity selected() const noexcept;

  void recordOutline(VkCommandBuffer commands) const;

  void resizeTarget(VkExtent2D extent);

  [[nodiscard]] bool gridDrawn() const noexcept;

  void updateGrid();

  void recordGrid(VkCommandBuffer commands) const;

  [[nodiscard]] talus::renderer::SceneTarget& target() noexcept;

private:
  [[nodiscard]] talus::field::Ray rayAt(float x, float y) const;

  [[nodiscard]] bool alongAxis(const talus::field::Ray& ray, float& at) const;

  talus::engine::Engine& m_engine;

  talus::view::ViewportId m_id;

  talus::view::Viewport m_config{};

  talus::view::Camera m_camera;

  bool m_throughScene = false;

  talus::renderer::SceneTarget m_target;

  talus::gizmos::Grid m_grid;

  talus::gizmos::Handles m_handles;

  talus::gizmos::Outline m_outline;

  talus::objectmodel::Entity m_selected = talus::objectmodel::INVALID_ENTITY;

  talus::objectmodel::Entity m_handled = talus::objectmodel::INVALID_ENTITY;

  uint32_t m_axis = talus::gizmos::NO_AXIS;

  glm::vec3 m_anchor{0.0f};

  glm::vec3 m_origin{0.0f};

  float m_grabbed = 0.0f;
  talus::objectmodel::Pose m_before{};
  PoseSink m_onPoseEdited;
};

}
