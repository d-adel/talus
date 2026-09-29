#include "sceneview.ih"

namespace talus::capi {

SceneView::SceneView(talus::engine::Engine& engine)
    : m_engine(engine),
      m_id(engine.createViewport()),
      m_target(
        engine.context().device(),
        engine.swapChain().imageFormat(),
        engine.swapChain().extent(),
        1.0f
      ),
      m_grid(
        engine.context().device(),
        engine.swapChain(),
        engine.globalsLayouts()
      ),
      m_outline(
        engine.context().device(),
        engine.swapChain(),
        m_target.selectionDepthView(),
        m_target.depthView()
      ) {
  m_config.gridVisible = true;
}

bool SceneView::outlineShown() const noexcept {
  return m_outline.shown();
}

talus::objectmodel::Entity SceneView::selected() const noexcept {
  return m_selected;
}

void SceneView::recordOutline(VkCommandBuffer commands) const {
  m_outline.record(commands);
}

void SceneView::resizeTarget(VkExtent2D extent) {
  m_target.resize(extent);

  m_outline.retarget(m_target.selectionDepthView(), m_target.depthView());
}

SceneView::~SceneView() {
  static_cast<void>(m_engine.destroyViewport(m_id));
}

talus::view::ViewportId SceneView::id() const noexcept {
  return m_id;
}

const talus::view::Viewport& SceneView::config() const noexcept {
  return m_config;
}

void SceneView::configure(const talus::view::Viewport& settings) {
  const bool scaleChanged = settings.supersample != m_config.supersample;

  m_config.gridVisible = settings.gridVisible;
  m_config.supersample = settings.supersample;
  m_config.mode = settings.mode;
  m_config.fieldRepresentation = settings.fieldRepresentation;
  m_config.march = settings.march;

  if (scaleChanged) {
    m_engine.context().device().waitIdle();

    m_target.setScale(m_config.supersample);

    m_outline.retarget(m_target.selectionDepthView(), m_target.depthView());
  }
}

void SceneView::orbit(float deltaYaw, float deltaPitch) {
  m_camera.orbit(deltaYaw, deltaPitch);
}

void SceneView::pan(float deltaX, float deltaY) {
  m_camera.pan(deltaX, deltaY);
}

void SceneView::zoom(float delta) {
  m_camera.zoom(delta);
}

void SceneView::frameOnWorld() {
  const talus::objectmodel::WorldExtent whole =
    talus::objectmodel::sceneExtent(m_engine.world());

  if (whole.empty) {
    m_camera.frame(glm::vec3(0.0f), 4.0f);

    return;
  }

  m_camera.frame(whole.centre(), 0.5f * glm::length(whole.span()));
}

void SceneView::frameOn(talus::objectmodel::Entity entity) {
  if (entity == talus::objectmodel::INVALID_ENTITY) {
    frameOnWorld();

    return;
  }

  const talus::objectmodel::WorldExtent around =
    talus::objectmodel::entityExtent(m_engine.activeWorld(), entity);

  if (around.empty) {
    return;
  }

  m_camera.frame(around.centre(), 0.5f * glm::length(around.span()));
}

void SceneView::alignToSceneCamera() {
  const talus::objectmodel::Entity eye =
    talus::objectmodel::firstLens(m_engine.world());

  if (eye == talus::objectmodel::INVALID_ENTITY) {
    return;
  }

  const talus::objectmodel::Pose& seat =
    m_engine.world().registry().get<talus::objectmodel::Pose>(eye);

  const talus::objectmodel::WorldExtent whole =
    talus::objectmodel::sceneExtent(m_engine.world());

  const float reach =
    whole.empty ? 10.0f : glm::length(whole.centre() - seat.position);

  m_camera.placeFrom(seat, glm::max(reach, 1.0f));
}

void SceneView::resetCamera() noexcept {
  m_camera.reset();
}

bool SceneView::throughScene() const noexcept {
  return m_throughScene;
}

void SceneView::setThroughScene(bool through) noexcept {
  m_throughScene = through;
}

void SceneView::bind() {
  m_config.mode = talus::view::DrawMode::Both;

  if (m_throughScene) {
    m_config.camera = talus::objectmodel::mainCamera(m_engine.activeWorld());

    if (m_config.camera == talus::objectmodel::INVALID_ENTITY) {
      m_config.mode = talus::view::DrawMode::Nothing;
    }
  } else {
    if (!m_camera.placed()) {
      frameOnWorld();
    }

    m_config.camera = talus::objectmodel::INVALID_ENTITY;
    m_config.pose = m_camera.pose();
    m_config.lens = m_camera.lens();
  }

  m_handles.update(m_camera.eye());

  if (!m_engine.configureViewport(m_id, m_config)) {
    throw std::runtime_error("the scene view lost its viewport");
  }
}

talus::field::Ray SceneView::rayAt(float x, float y) const {
  const VkExtent2D size = m_engine.swapChain().extent();

  if (size.width == 0u || size.height == 0u) {
    return talus::field::Ray{};
  }

  const glm::vec2 ndc(
    2.0f * x / static_cast<float>(size.width) - 1.0f,
    2.0f * y / static_cast<float>(size.height) - 1.0f
  );

  const talus::objectmodel::CameraView seen = talus::view::lookThrough(
    m_config,
    m_engine.activeWorld(),
    static_cast<float>(size.width) / static_cast<float>(size.height)
  );

  return talus::field::screenPointToRay(
    glm::inverse(seen.projection * seen.view),
    seen.position,
    ndc
  );
}

talus::objectmodel::Entity SceneView::pick(float x, float y) const {
  const VkExtent2D size = m_engine.swapChain().extent();

  if (size.width == 0u || size.height == 0u) {
    return talus::objectmodel::INVALID_ENTITY;
  }

  auto* runtime = m_engine.runtime();
  const auto hit = talus::query::raycast(
    {m_engine.activeWorld(), runtime ? &runtime->physics() : nullptr},
    rayAt(x, y)
  );
  return hit ? hit->entity : talus::objectmodel::INVALID_ENTITY;
}

bool SceneView::gridDrawn() const noexcept {
  return m_config.gridVisible && !m_throughScene;
}

void SceneView::updateGrid() {
  m_grid.update(m_camera.eye());
}

void SceneView::placeHandles(talus::objectmodel::Entity entity) {
  m_handled = entity;
  m_axis = talus::gizmos::NO_AXIS;

  if (
    entity == talus::objectmodel::INVALID_ENTITY
    || !m_engine.activeWorld().registry().all_of<talus::objectmodel::Pose>(
      entity
    )
  ) {
    m_handled = talus::objectmodel::INVALID_ENTITY;

    m_handles.hide();
    m_outline.hide();
    m_selected = talus::objectmodel::INVALID_ENTITY;

    return;
  }

  m_selected = entity;
  m_outline.show();

  m_handles.place(m_engine.activeWorld()
                    .registry()
                    .get<talus::objectmodel::Pose>(entity)
                    .position);
}

bool SceneView::handlesShown() const noexcept {
  return m_handles.shown();
}

uint32_t SceneView::handleAxis(float x, float y) const {
  const talus::field::Ray ray = rayAt(x, y);

  return m_handles.under(ray.origin, glm::normalize(ray.direction));
}

void SceneView::highlightHandle(uint32_t axis) noexcept {
  m_handles.highlight(axis);
}

bool SceneView::alongAxis(const talus::field::Ray& ray, float& at) const {
  const std::optional<float> found =
    talus::field::alongAxis(m_anchor, m_handles.direction(m_axis), ray);

  if (!found.has_value()) {
    return false;
  }

  at = *found;

  return true;
}

bool SceneView::startHandleDrag(uint32_t axis, float x, float y) {
  if (
    axis >= 3u || !m_handles.shown()
    || m_handled == talus::objectmodel::INVALID_ENTITY
  ) {
    return false;
  }

  m_axis = axis;
  m_anchor = m_handles.centre();
  m_before =
    m_engine.activeWorld().registry().get<talus::objectmodel::Pose>(m_handled);
  m_origin = m_before.position;

  m_handles.highlight(axis);

  return alongAxis(rayAt(x, y), m_grabbed);
}

bool SceneView::moveHandles(float x, float y) {
  if (m_axis >= 3u || m_handled == talus::objectmodel::INVALID_ENTITY) {
    return false;
  }

  float at = 0.0f;

  if (!alongAxis(rayAt(x, y), at)) {
    return false;
  }

  talus::objectmodel::Pose moved =
    m_engine.activeWorld().registry().get<talus::objectmodel::Pose>(m_handled);

  moved.position = m_origin + m_handles.direction(m_axis) * (at - m_grabbed);

  m_engine.activeWorld().emplaceOrReplace<talus::objectmodel::Pose>(
    m_handled,
    moved
  );

  m_handles.place(moved.position);

  return true;
}

void SceneView::endHandleDrag() noexcept {
  if (
    m_axis != talus::gizmos::NO_AXIS
    && m_handled != talus::objectmodel::INVALID_ENTITY && m_onPoseEdited
    && m_engine.activeWorld().registry().valid(m_handled)
  ) {
    const talus::objectmodel::Pose& now =
      m_engine.activeWorld().registry().get<talus::objectmodel::Pose>(
        m_handled
      );

    if (now.position != m_before.position) {
      m_onPoseEdited(m_handled, m_before, now);
    }
  }

  m_axis = talus::gizmos::NO_AXIS;

  m_handles.highlight(talus::gizmos::NO_AXIS);
}

void SceneView::onPoseEdited(PoseSink sink) {
  m_onPoseEdited = std::move(sink);
}

std::span<const talus::debugdraw::LineVertex> SceneView::
  handleLines() const noexcept {
  return m_handles.shown() ? m_handles.lines()
                           : std::span<const talus::debugdraw::LineVertex>{};
}

void SceneView::recordGrid(VkCommandBuffer commands) const {
  m_grid.record(commands, m_engine.globals());
}

talus::renderer::SceneTarget& SceneView::target() noexcept {
  return m_target;
}

}
