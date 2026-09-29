#include "session.ih"

namespace talus::capi {

namespace {

talus::gpu::WindowSystem& require(
  const std::unique_ptr<talus::gpu::WindowSystem>& windowSystem
) {
  if (windowSystem == nullptr) {
    throw std::runtime_error("A session needs a window system");
  }

  return *windowSystem;
}
}

Session::Session(
  std::unique_ptr<talus::gpu::WindowSystem> windowSystem,
  const std::filesystem::path& projectRoot,
  const std::string& world
)
    : m_engine(std::move(windowSystem), projectRoot), m_worldName(world) {
  m_views.push_back(std::make_unique<SceneView>(m_engine));

  m_engine.loadWorld(world);
}

Session::~Session() {
  m_engine.context().device().waitIdle();

  try {
    stop();
  } catch (const std::exception& failure) {
    std::cerr << "Session teardown: " << failure.what() << std::endl;
  }
}

talus::view::ViewportId Session::createView() {
  m_views.push_back(std::make_unique<SceneView>(m_engine));
  m_views.back()->onPoseEdited([this](
                                 talus::objectmodel::Entity entity,
                                 const talus::objectmodel::Pose& before,
                                 const talus::objectmodel::Pose& after
                               ) {
    if (!playing() || !m_history.fieldsOpen()) {
      return;
    }

    talus::undo::FieldDiff diff;
    diff.entity = entity;
    diff.component = "Pose";
    diff.field = 0u;
    diff.kind = talus::objectmodel::FieldKind::Vec3;
    diff.before.resize(12u);
    diff.after.resize(12u);

    const float was[3]{before.position.x, before.position.y, before.position.z};
    const float is[3]{after.position.x, after.position.y, after.position.z};

    std::memcpy(diff.before.data(), was, 12u);
    std::memcpy(diff.after.data(), is, 12u);

    m_history.note(std::move(diff));
  });

  return m_views.back()->id();
}

bool Session::destroyView(talus::view::ViewportId id) {
  const auto found =
    std::find_if(m_views.begin(), m_views.end(), [id](const auto& held) {
      return held->id() == id;
    });

  if (found == m_views.end() || m_views.size() == 1u) {
    return false;
  }

  m_engine.context().device().waitIdle();

  m_views.erase(found);

  return true;
}

SceneView* Session::view(talus::view::ViewportId id) noexcept {
  const auto found =
    std::find_if(m_views.begin(), m_views.end(), [id](const auto& held) {
      return held->id() == id;
    });

  return found == m_views.end() ? nullptr : found->get();
}

const SceneView* Session::view(talus::view::ViewportId id) const noexcept {
  const auto found =
    std::find_if(m_views.begin(), m_views.end(), [id](const auto& held) {
      return held->id() == id;
    });

  return found == m_views.end() ? nullptr : found->get();
}

SceneView& Session::mainView() noexcept {
  return *m_views.front();
}

const SceneView& Session::mainView() const noexcept {
  return *m_views.front();
}

void Session::setKey(uint32_t key, bool down) noexcept {
  m_engine.input().setKey(static_cast<talus::systems::Key>(key), down);
}

void Session::setMouseButton(uint32_t button, bool down) noexcept {
  m_engine.input().setMouseButton(
    static_cast<talus::systems::MouseButton>(button),
    down
  );
}

void Session::setCursor(float x, float y) noexcept {
  m_engine.input().setCursor(x, y);
}

void Session::showDebug(const std::string& category, bool shown) {
  m_engine.debug().show(category, shown);
}

bool Session::package(
  const std::string& playerExecutable,
  const std::string& output,
  const std::vector<std::string>& worlds,
  std::string& result
) const {
  talus::package::PackageSpec spec{};
  spec.playerExecutable = playerExecutable;
  spec.projectRoot = m_engine.projectRoot();
  spec.output = output;
  spec.worlds = worlds.empty() ? std::vector<std::string>{m_worldName} : worlds;

  if (!talus::package::packageProject(spec, result)) {
    return false;
  }

  result = talus::package::packagedExecutable(spec).string();

  return true;
}

void Session::resize() {
  const VkExtent2D current = m_engine.windowSystem().framebufferExtent();

  if (current.width == 0 || current.height == 0) {
    return;
  }

  m_engine.swapChain().recreate();

  resizeTargets();
}

void Session::render() {
  const VkExtent2D current = m_engine.windowSystem().framebufferExtent();

  if (current.width == 0 || current.height == 0) {
    return;
  }

  const auto frameAt = std::chrono::steady_clock::now();

  const float elapsed =
    std::chrono::duration<float>(frameAt - m_lastFrame).count();

  m_lastFrame = frameAt;

  SceneView& drawn = mainView();

  drawn.bind();

  m_engine.renderer().useTarget(&drawn.target());

  std::vector<talus::renderer::ScenePass> passes;

  if (drawn.outlineShown()) {
    passes.push_back(
      {talus::renderer::PassOrder::AfterScene,
       [&drawn](VkCommandBuffer commands) { drawn.recordOutline(commands); }}
    );
  }

  if (drawn.gridDrawn()) {
    drawn.updateGrid();

    passes.push_back(
      {talus::renderer::PassOrder::AfterScene,
       [&drawn](VkCommandBuffer commands) { drawn.recordGrid(commands); }}
    );
  }

  talus::core::Telemetry ignored;

  talus::engine::FrameRequest request;
  request.viewport = drawn.id();
  request.selected = drawn.selected();
  request.deltaSeconds = elapsed;
  request.passes = passes;
  request.overlay = drawn.handleLines();

  if (m_engine.renderFrame(request, ignored)) {
    m_engine.swapChain().recreate();

    resizeTargets();
  }
}

void Session::retarget(std::unique_ptr<talus::gpu::WindowSystem> windowSystem) {
  m_engine.retarget(std::move(windowSystem));

  resizeTargets();
}

void Session::resizeTargets() {
  for (const auto& held : m_views) {
    held->resizeTarget(m_engine.swapChain().extent());
  }
}

VkExtent2D Session::extent() const noexcept {
  return m_engine.swapChain().extent();
}

const talus::objectmodel::World& Session::world() const noexcept {
  return m_engine.world();
}

talus::objectmodel::World& Session::world() noexcept {
  return m_engine.world();
}

talus::systems::Schedule& Session::schedule() noexcept {
  return m_engine.schedule();
}

void Session::play() {
  if (m_engine.playing()) {
    return;
  }

  m_playMark = m_history.depth();
  m_engine.play();

  for (const auto& held : m_views) {
    held->setThroughScene(true);
  }
}

void Session::stop() {
  if (!m_engine.playing()) {
    return;
  }

  m_history.forgetSince(m_playMark);
  m_engine.stop();

  for (const auto& held : m_views) {
    held->setThroughScene(false);
  }
}

bool Session::playing() const noexcept {
  return m_engine.playing();
}

void Session::setFixedStep(std::optional<float> seconds) {
  m_engine.setFixedStep(seconds);
}

std::optional<float> Session::fixedStep() const noexcept {
  return m_engine.fixedStep();
}

void Session::setPaused(bool paused) noexcept {
  m_engine.setPaused(paused);
}

bool Session::paused() const noexcept {
  return m_engine.paused();
}

void Session::stepOnce() noexcept {
  m_engine.stepOnce();
}

talus::objectmodel::World& Session::activeWorld() noexcept {
  return m_engine.activeWorld();
}

const talus::objectmodel::World& Session::activeWorld() const noexcept {
  return m_engine.activeWorld();
}

talus::objectmodel::Entity Session::createLight() {
  const talus::objectmodel::Entity entity = m_engine.world().createAuthored();

  m_engine.world().emplaceOrReplace<talus::objectmodel::Pose>(
    entity,
    talus::objectmodel::Pose{}
  );
  m_engine.world().emplaceOrReplace<talus::objectmodel::DirectionalLight>(
    entity,
    talus::objectmodel::DirectionalLight{}
  );

  return entity;
}

talus::objectmodel::Entity Session::createEnvironment() {
  const talus::objectmodel::Entity entity = m_engine.world().createAuthored();

  m_engine.world().emplaceOrReplace<talus::objectmodel::Pose>(
    entity,
    talus::objectmodel::Pose{}
  );
  m_engine.world().emplaceOrReplace<talus::objectmodel::Environment>(
    entity,
    talus::objectmodel::Environment{}
  );

  return entity;
}

talus::objectmodel::Entity Session::createCamera(
  const glm::vec3& position,
  const glm::vec3& target,
  float verticalFov
) {
  const glm::vec3 ahead = glm::normalize(target - position);

  const float yaw = std::atan2(ahead.x, -ahead.z);
  const float pitch = std::asin(glm::clamp(ahead.y, -1.0f, 1.0f));

  talus::objectmodel::Pose seat{};
  seat.position = position;
  seat.rotation = glm::angleAxis(yaw, glm::vec3(0.0f, -1.0f, 0.0f))
                  * glm::angleAxis(pitch, glm::vec3(1.0f, 0.0f, 0.0f));

  talus::objectmodel::Lens lens{};
  lens.verticalFov = verticalFov;

  const bool first = talus::objectmodel::mainCamera(m_engine.world())
                     == talus::objectmodel::INVALID_ENTITY;

  const talus::objectmodel::Entity entity = m_engine.world().createAuthored();

  m_engine.world().emplaceOrReplace<talus::objectmodel::Pose>(entity, seat);
  m_engine.world().emplaceOrReplace<talus::objectmodel::Lens>(entity, lens);

  if (first && !m_engine.world().setMainCamera(entity)) {
    throw std::runtime_error("A new camera could not become the main camera");
  }

  return entity;
}

talus::objectmodel::Entity Session::createSdf(
  const talus::objectmodel::Pose& pose,
  const glm::uvec3& resolution,
  float margin
) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = resolution;
  grid.margin = margin;

  return m_engine.world().createSdfGrid(pose, grid);
}

bool Session::addPrimitive(
  talus::objectmodel::Entity entity,
  const talus::field::Primitive& primitive
) {
  return m_engine.world().addPrimitive(entity, primitive);
}

void Session::setStatic(talus::objectmodel::Entity entity) {
  m_engine.world().addStaticBody(entity);
}

void Session::setDeformable(talus::objectmodel::Entity entity) {
  m_engine.world().emplaceOrReplace<talus::objectmodel::DeformableBody>(
    entity,
    talus::objectmodel::DeformableBody{}
  );
}

bool Session::destroyEntity(talus::objectmodel::Entity entity) {
  return m_engine.world().destroy(entity);
}

std::vector<std::string> Session::componentNames() const {
  return talus::serialize::componentNames(activeWorld());
}

bool Session::addComponent(
  talus::objectmodel::Entity entity,
  std::string_view component
) {
  return talus::serialize::addComponent(activeWorld(), entity, component);
}

bool Session::removeComponent(
  talus::objectmodel::Entity entity,
  std::string_view component
) {
  return talus::serialize::removeComponent(activeWorld(), entity, component);
}

std::string Session::removalBlocker(
  talus::objectmodel::Entity entity,
  std::string_view component
) const {
  return talus::serialize::removalBlocker(activeWorld(), entity, component);
}

bool Session::canAddComponent(
  talus::objectmodel::Entity entity,
  std::string_view component
) const {
  return talus::serialize::canAddComponent(activeWorld(), entity, component);
}

std::optional<talus::serialize::ComponentSchema> Session::componentSchema(
  std::string_view component
) const {
  return talus::serialize::componentSchema(activeWorld(), component);
}

std::optional<std::vector<std::byte>> Session::componentBytes(
  talus::objectmodel::Entity entity,
  std::string_view component
) const {
  return talus::serialize::componentBytes(activeWorld(), entity, component);
}

talus::serialize::WriteStatus Session::setField(
  talus::objectmodel::Entity entity,
  std::string_view component,
  uint32_t field,
  talus::objectmodel::FieldKind kind,
  std::span<const std::byte> bytes
) {
  std::vector<std::byte> before;

  if (playing() && m_history.fieldsOpen()) {
    const auto schema = componentSchema(component);
    const auto held = componentBytes(entity, component);

    if (schema && held && field < schema->spec.fields.size()) {
      const auto& spec = schema->spec.fields[field];
      const uint32_t size =
        talus::serialize::fieldByteSize(spec.kind, spec.flags);

      if (spec.offset + size <= held->size()) {
        before.assign(
          held->begin() + spec.offset,
          held->begin() + spec.offset + size
        );
      }
    }
  }

  const auto status = talus::serialize::writeField(
    activeWorld(),
    entity,
    component,
    field,
    kind,
    bytes
  );

  if (status == talus::serialize::WriteStatus::Written && !before.empty()) {
    m_history.note(
      talus::undo::FieldDiff{
        entity,
        std::string(component),
        field,
        kind,
        std::move(before),
        std::vector<std::byte>(bytes.begin(), bytes.end())
      }
    );
  }

  return status;
}

bool Session::record(std::string label) {
  if (playing()) {
    return m_history.beginFields(std::move(label));
  }

  return m_history.begin(m_engine.world(), std::move(label));
}

uint32_t Session::drainChanges(std::span<uint32_t> touched, bool& wholesale) {
  talus::objectmodel::World& world = activeWorld();

  const std::span<const talus::objectmodel::Entity> moved = world.touched();

  wholesale = world.touchedOverflowed() || moved.size() > touched.size();

  const uint32_t written = wholesale ? 0u : static_cast<uint32_t>(moved.size());

  for (uint32_t index = 0; index != written; ++index) {
    touched[index] = static_cast<uint32_t>(moved[index]);
  }

  world.clearTouched();

  return written;
}

bool Session::commit() {
  return m_history.commit();
}

bool Session::discard() {
  return m_history.discard(activeWorld());
}

bool Session::undo() {
  if (playing()) {
    if (m_history.depth() <= m_playMark || !m_history.undoIsFields()) {
      return false;
    }

    return m_history.undo(activeWorld());
  }

  m_engine.context().device().waitIdle();

  return m_history.undo(m_engine.world());
}

bool Session::redo() {
  if (playing()) {
    if (!m_history.redoIsFields()) {
      return false;
    }

    return m_history.redo(activeWorld());
  }

  m_engine.context().device().waitIdle();

  return m_history.redo(m_engine.world());
}

std::optional<std::string_view> Session::undoLabel() const noexcept {
  return m_history.undoLabel();
}

std::optional<std::string_view> Session::redoLabel() const noexcept {
  return m_history.redoLabel();
}

bool Session::save() const {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_engine.projectRoot());

  if (!project.has_value()) {
    return false;
  }

  return project->saveWorld(
    m_worldName,
    talus::serialize::captureWorld(m_engine.world(), m_engine.assets().assets())
  );
}

size_t Session::materialCount() const noexcept {
  return m_engine.assets().assets().of<talus::objectmodel::Material>().size();
}

talus::core::AssetId Session::materialAt(uint32_t index) const noexcept {
  return m_engine.assets().assets().id<talus::objectmodel::Material>(index);
}

bool Session::openWorld(const std::string& name) {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_engine.projectRoot());

  if (!project.has_value()) {
    return false;
  }

  talus::serialize::StoredWorld stored;

  if (!project->loadWorld(name, stored)) {
    return false;
  }

  stop();

  m_engine.context().device().waitIdle();

  m_engine.storage().residency().releaseAll(m_engine.world());

  m_engine.world().clear();

  m_history.clear();

  if (!talus::serialize::restoreWorld(
        stored,
        m_engine.world(),
        m_engine.assets().assets()
      )) {
    return false;
  }

  m_worldName = name;
  for (const auto& held : m_views) {
    held->resetCamera();
  }

  return true;
}

bool Session::newWorld(const std::string& name) {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_engine.projectRoot());

  if (!project.has_value()) {
    return false;
  }

  talus::objectmodel::World fresh;

  talus::objectmodel::addDefaultEntities(fresh);

  if (!project->saveWorld(
        name,
        talus::serialize::captureWorld(fresh, m_engine.assets().assets())
      )) {
    return false;
  }

  return openWorld(name);
}

bool Session::saveAs(const std::string& name) {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_engine.projectRoot());

  if (!project.has_value()) {
    return false;
  }

  if (!project->saveWorld(
        name,
        talus::serialize::captureWorld(
          m_engine.world(),
          m_engine.assets().assets()
        )
      )) {
    return false;
  }

  m_worldName = name;

  return true;
}

const std::string& Session::worldName() const noexcept {
  return m_worldName;
}

uint32_t Session::materialIndex(talus::core::AssetId id) const noexcept {
  return m_engine.assets().assets().index<talus::objectmodel::Material>(id);
}

talus::core::AssetId Session::addMaterial(
  const glm::vec4& baseColor,
  float metallic,
  float roughness
) {
  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_engine.projectRoot());

  if (!project.has_value()) {
    throw std::runtime_error("The session has no project to add a material to");
  }

  talus::serialize::StoredMaterial stored{};
  stored.baseColor = baseColor;
  stored.metallic = metallic;
  stored.roughness = roughness;

  const std::filesystem::path staged =
    std::filesystem::temp_directory_path()
    / ("material-" + std::to_string(materialCount()) + ".mat.json");

  if (!talus::serialize::writeText(
        staged,
        talus::serialize::writeMaterial(stored)
      )) {
    throw std::runtime_error("The material could not be written");
  }

  const talus::core::AssetId minted = project->importAsset(staged, "material");

  std::error_code ignored;
  std::filesystem::remove(staged, ignored);

  if (minted == talus::core::INVALID_ASSET) {
    throw std::runtime_error("The material could not be registered");
  }

  if (!project->save()) {
    throw std::runtime_error("The project manifest could not be saved");
  }

  static_cast<void>(m_engine.assets().addMaterial(minted, stored));

  m_engine.syncMaterials();

  m_engine.context().device().waitIdle();
  m_engine.storage().resourceTable().refresh(m_engine.assets().materials());

  return minted;
}

talus::objectmodel::PhysicalMaterial Session::materialPhysics(
  talus::core::AssetId id
) const {
  const uint32_t index = materialIndex(id);

  const std::vector<talus::objectmodel::PhysicalMaterial>& held =
    m_engine.assets().physicalMaterials();

  if (index == talus::core::UNBOUND_ASSET || index >= held.size()) {
    throw std::runtime_error("The material is not in this project");
  }

  return held[index];
}

void Session::setMaterialPhysics(
  talus::core::AssetId id,
  const talus::objectmodel::PhysicalMaterial& physical
) {
  if (!std::isfinite(physical.friction) || physical.friction < 0.0f) {
    throw std::runtime_error("friction must be a finite value of zero or more");
  }

  if (
    !std::isfinite(physical.restitution) || physical.restitution < 0.0f
    || physical.restitution > 1.0f
  ) {
    throw std::runtime_error(
      "restitution must be a finite value from zero to one"
    );
  }

  if (!std::isfinite(physical.density) || physical.density <= 0.0f) {
    throw std::runtime_error("density must be a finite value above zero");
  }

  if (!(physical.strength > 0.0f)) {
    throw std::runtime_error("strength must be above zero");
  }

  const uint32_t index = materialIndex(id);

  if (index == talus::core::UNBOUND_ASSET) {
    throw std::runtime_error("The material is not in this project");
  }

  std::optional<talus::serialize::Project> project =
    talus::serialize::Project::open(m_engine.projectRoot());

  if (!project.has_value()) {
    throw std::runtime_error(
      "The session has no project to write the material to"
    );
  }

  const talus::serialize::AssetEntry* entry = project->find(id);

  if (entry == nullptr) {
    throw std::runtime_error("The project lists no material under that id");
  }

  const std::filesystem::path file = project->root() / entry->path;

  const std::optional<std::string> text = talus::serialize::readText(file);

  if (!text.has_value()) {
    throw std::runtime_error("The material file could not be read");
  }

  talus::serialize::StoredMaterial stored{};

  if (!talus::serialize::readMaterial(*text, stored)) {
    throw std::runtime_error("The material file could not be parsed");
  }

  talus::serialize::setPhysicalMaterial(stored, physical);

  if (!talus::serialize::writeText(
        file,
        talus::serialize::writeMaterial(stored)
      )) {
    throw std::runtime_error("The material file could not be written");
  }

  m_engine.assets().setPhysicalMaterial(index, physical);

  m_engine.syncMaterials();
}

}
