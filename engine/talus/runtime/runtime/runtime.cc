#include "runtime.ih"

namespace talus::runtime {

namespace {

constexpr uint32_t REOPTIMIZE_MINIMUM_ADDED = 64u;
constexpr uint32_t REOPTIMIZE_ADDED_DIVISOR = 100u;
constexpr double DEMOTE_RESCALE = 1.0 - 0x1.0p-21;

template<typename T>
void appendAll(std::vector<T>& into, std::vector<T>& from) {
  if (into.empty()) {
    into.swap(from);

    return;
  }

  into.insert(into.end(), from.begin(), from.end());
}

template<typename T>
void replaceWhenFilled(std::vector<T>& into, std::vector<T>& from) {
  if (from.empty()) {
    return;
  }

  into.swap(from);
}

void mergeStepReport(
  deformable::StepReport& from,
  deformable::StepReport& into
) {
  into.nodes += from.nodes;
  into.elements += from.elements;
  into.cgIterations += from.cgIterations;
  into.cgCappedSolves += from.cgCappedSolves;
  into.plasticFallbacks += from.plasticFallbacks;
  into.plasticFallbackRejections += from.plasticFallbackRejections;
  into.primaryCgOperatorApplications += from.primaryCgOperatorApplications;
  into.contacts += from.contacts;
  into.cgWorstRelativeResidual =
    glm::max(into.cgWorstRelativeResidual, from.cgWorstRelativeResidual);
  into.setupSeconds += from.setupSeconds;
  into.setupScratchSeconds += from.setupScratchSeconds;
  into.setupStiffnessSeconds += from.setupStiffnessSeconds;
  into.setupIncidenceSeconds += from.setupIncidenceSeconds;
  into.setupAllocateSeconds += from.setupAllocateSeconds;
  into.elasticSeconds += from.elasticSeconds;
  into.primaryCgSeconds += from.primaryCgSeconds;
  into.primaryCgMassSeconds += from.primaryCgMassSeconds;
  into.primaryCgTetSeconds += from.primaryCgTetSeconds;
  into.primaryCgAnchorSeconds += from.primaryCgAnchorSeconds;
  into.primaryCgConditionSeconds += from.primaryCgConditionSeconds;
  into.factorPackSeconds += from.factorPackSeconds;
  into.factorSolveSeconds += from.factorSolveSeconds;
  into.factorUnpackSeconds += from.factorUnpackSeconds;
  into.factorApplications += from.factorApplications;
  into.factorRows = from.factorRows;
  into.factorNonZeros = from.factorNonZeros;
  into.systemNonZeros = from.systemNonZeros;
  into.basisSeconds += from.basisSeconds;
  into.responseCacheSeconds += from.responseCacheSeconds;
  into.delassusSeconds += from.delassusSeconds;
  into.contactSeconds += from.contactSeconds;
  into.contactSweepSeconds += from.contactSweepSeconds;
  into.velocityContactSweepSeconds += from.velocityContactSweepSeconds;
  into.contactSweeps += from.contactSweeps;
  into.contactSweepNodes += from.contactSweepNodes;
  into.responseApplySeconds += from.responseApplySeconds;
  into.contactRows += from.contactRows;
  into.maxPenetration = glm::max(into.maxPenetration, from.maxPenetration);
  into.contactWorstRelativeChange =
    glm::max(into.contactWorstRelativeChange, from.contactWorstRelativeChange);
  into.contactPasses = glm::max(into.contactPasses, from.contactPasses);
  into.contactTotalPasses += from.contactTotalPasses;
  into.contactSettlePass =
    glm::max(into.contactSettlePass, from.contactSettlePass);
  into.contactTightPass =
    glm::max(into.contactTightPass, from.contactTightPass);
  into.contactResidual = glm::max(into.contactResidual, from.contactResidual);
  into.velocityContactPasses =
    glm::max(into.velocityContactPasses, from.velocityContactPasses);
  into.velocityContactTotalPasses += from.velocityContactTotalPasses;
  into.velocityContactResidual =
    glm::max(into.velocityContactResidual, from.velocityContactResidual);
  into.velocityContactCapped += from.velocityContactCapped;
  into.velocityResponseApplySeconds += from.velocityResponseApplySeconds;
  into.posedSubsteps += from.posedSubsteps;
  into.posedSolves += from.posedSolves;
  into.posedCapped += from.posedCapped;
  into.posedKineticChange += from.posedKineticChange;
  into.basisSolves += from.basisSolves;
  into.elasticEnergy += from.elasticEnergy;
  into.kineticEnergy += from.kineticEnergy;
  into.cpuSeconds += from.cpuSeconds;
  into.plasticGrowth = into.plasticGrowth || from.plasticGrowth;
  into.refactors += from.refactors;
  into.stiffnessRebuilds += from.stiffnessRebuilds;
  into.retries += from.retries;
  into.refreshRequests += from.refreshRequests;
  into.retriesFailed += from.retriesFailed;
  into.wastedIterations += from.wastedIterations;
  into.worstSubstep = glm::max(into.worstSubstep, from.worstSubstep);
  into.refactorSeconds += from.refactorSeconds;
  into.contactResponseResidual =
    glm::max(into.contactResponseResidual, from.contactResponseResidual);

  if (from.refreshedAtAge != 0u) {
    into.refreshedAtAge = from.refreshedAtAge;
  }

  if (from.spentBeforeRefresh != 0u) {
    into.spentBeforeRefresh = from.spentBeforeRefresh;
  }

  if (from.spentAfterRefresh != 0u) {
    into.spentAfterRefresh = from.spentAfterRefresh;
  }

  appendAll(into.cgPerSubstep, from.cgPerSubstep);
  appendAll(into.cgStartResidual, from.cgStartResidual);
  appendAll(into.cgEndResidual, from.cgEndResidual);
  appendAll(into.contactSolves, from.contactSolves);
  appendAll(into.exchange, from.exchange);

  replaceWhenFilled(into.contactImpulses, from.contactImpulses);
  replaceWhenFilled(into.contactLoad, from.contactLoad);
  replaceWhenFilled(into.contactWarmStart, from.contactWarmStart);
  replaceWhenFilled(into.contactOperator, from.contactOperator);
  replaceWhenFilled(into.contactRight, from.contactRight);
  replaceWhenFilled(into.contactFriction, from.contactFriction);
  replaceWhenFilled(into.contactFrames, from.contactFrames);
  replaceWhenFilled(into.contactChecks, from.contactChecks);
}

JPH::AABox surfaceBox(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& state
) {
  JPH::AABox bounds = JPH::AABox::sBiggest();
  bounds.SetEmpty();

  for (const uint32_t node : mesh.surfaceNodes) {
    const glm::vec3& at = state.position[node];

    bounds.Encapsulate(JPH::Vec3(at.x, at.y, at.z));
  }

  bounds.ExpandBy(JPH::Vec3::sReplicate(glm::compMax(mesh.voxelSize)));

  return bounds;
}

bool overlaps(
  const glm::vec3& minimumA,
  const glm::vec3& maximumA,
  const glm::vec3& minimumB,
  const glm::vec3& maximumB
) {
  return glm::all(glm::lessThanEqual(minimumA, maximumB))
         && glm::all(glm::lessThanEqual(minimumB, maximumA));
}

bool finiteValue(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y)
         && std::isfinite(value.z);
}

bool finiteState(const objectmodel::DeformableState& state) {
  for (const glm::vec3& position : state.position) {
    if (!finiteValue(position)) {
      return false;
    }
  }

  for (const glm::vec3& velocity : state.velocity) {
    if (!finiteValue(velocity)) {
      return false;
    }
  }

  for (const glm::vec3& acceleration : state.acceleration) {
    if (!finiteValue(acceleration)) {
      return false;
    }
  }

  return true;
}

deformable::ContactGaps contactGaps(const objectmodel::PreparedMesh& mesh) {
  const float voxel = glm::compMax(mesh.voxelSize);

  return deformable::ContactGaps{0.1f * voxel, 0.2f * voxel};
}

std::optional<float> yieldTrigger(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials
) {
  if (mesh.hexes.empty() || !mesh.tets.empty()) {
    return std::nullopt;
  }

  const auto anchoredNode = [](uint8_t anchored) {
    return anchored != 0u;
  };

  if (std::any_of(mesh.anchored.begin(), mesh.anchored.end(), anchoredNode)) {
    return std::nullopt;
  }

  float trigger = std::numeric_limits<float>::infinity();

  const auto yieldingLane = [&](uint32_t lane) {
    const objectmodel::PhysicalMaterial& held =
      deformable::laneMaterial(materials, lane);

    if (
      !deformable::yields(
        deformable::PlasticParams{held.yieldStress, held.hardeningModulus}
      )
      || deformable::stiffnessDamping(held) > 0.0f
    ) {
      return false;
    }

    trigger = std::min(
      trigger,
      held.yieldStress / std::sqrt(held.density * held.youngsModulus)
    );

    return true;
  };

  for (const objectmodel::PreparedHex& hex : mesh.hexes) {
    if (!yieldingLane(hex.material)) {
      return std::nullopt;
    }
  }

  return trigger;
}

deformable::RigidMass meshMass(
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::vec3> position
) {
  return deformable::rigidMass(std::span<const float>(mesh.nodeMass), position);
}

struct Momentum {
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
  double kinetic = 0.0;
};

Momentum nodeMomentum(
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::dvec3> position,
  std::span<const glm::dvec3> velocity
) {
  double mass = 0.0;
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);

    mass += weight;
    centre += weight * position[node];
  }

  if (mass > 0.0) {
    centre /= mass;
  }

  Momentum held;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3& moving = velocity[node];

    held.linear += weight * moving;
    held.angular += weight * glm::cross(position[node] - centre, moving);
    held.kinetic += 0.5 * weight * glm::dot(moving, moving);
  }

  return held;
}

bool holdsSolid(const talus::field::SampledSdf& field) {
  return std::any_of(
    field.distances.begin(),
    field.distances.end(),
    [](float distance) { return distance < 0.0f; }
  );
}

glm::dvec3 fromJolt(JPH::Vec3Arg value) {
  return glm::dvec3(value.GetX(), value.GetY(), value.GetZ());
}

glm::dmat3 fromJolt(const JPH::Mat44& value) {
  glm::dmat3 held(0.0);

  for (int column = 0; column != 3; ++column) {
    held[column] = fromJolt(value.GetColumn3(column));
  }

  return held;
}

JPH::Vec3 toJolt(const glm::dvec3& value) {
  return JPH::Vec3(
    static_cast<float>(value.x),
    static_cast<float>(value.y),
    static_cast<float>(value.z)
  );
}

JPH::ShapeRefC yieldShape(
  const JPH::ShapeRefC& field,
  const objectmodel::PreparedMesh& mesh
) {
  return JPH::ShapeRefC(new JPH::OffsetCenterOfMassShape(
    field,
    toJolt(meshMass(mesh, mesh.restPositions).centre) - field->GetCenterOfMass()
  ));
}

double partnerEnergy(
  double mass,
  const glm::dmat3& inertia,
  const glm::dvec3& linear,
  const glm::dvec3& angular,
  const glm::dvec3& centre,
  const glm::dvec3& gravity
) {
  return 0.5 * mass * glm::dot(linear, linear)
         + 0.5 * glm::dot(angular, inertia * angular)
         - mass * glm::dot(gravity, centre);
}

JPH::Vec3 clampedTo(JPH::Vec3Arg value, float limit) {
  const float lengthSq = value.LengthSq();

  return lengthSq > JPH::Square(limit) ? value * (limit / JPH::Sqrt(lengthSq))
                                       : value;
}

std::pair<JPH::Vec3, JPH::Vec3> contactFree(
  const JPH::Body& body,
  JPH::Vec3Arg gravity,
  float step
) {
  const JPH::MotionProperties* motion = body.GetMotionProperties();

  JPH::Vec3 linear = motion->LockTranslation(
    body.GetLinearVelocity()
    + step
        * (motion->GetGravityFactor() * gravity + motion->GetInverseMass() * JPH::Vec3::sZero())
  );
  JPH::Vec3 angular = body.GetAngularVelocity();

  linear *= std::max(0.0f, 1.0f - motion->GetLinearDamping() * step);
  angular *= std::max(0.0f, 1.0f - motion->GetAngularDamping() * step);

  return {
    clampedTo(linear, motion->GetMaxLinearVelocity()),
    clampedTo(angular, motion->GetMaxAngularVelocity())
  };
}

std::pair<glm::dvec3, glm::dvec3> landingVelocity(
  const JPH::Body& body,
  const glm::dvec3& centre,
  const glm::dquat& rotation,
  const glm::dvec3& gravity,
  double step
) {
  const JPH::MotionProperties* motion = body.GetMotionProperties();
  const JPH::Quat held = body.GetRotation();

  const double linearKept =
    std::max(0.0, 1.0 - static_cast<double>(motion->GetLinearDamping()) * step);
  const double angularKept = std::max(
    0.0,
    1.0 - static_cast<double>(motion->GetAngularDamping()) * step
  );

  glm::dquat turn =
    rotation
    * glm::conjugate(
      glm::dquat(held.GetW(), held.GetX(), held.GetY(), held.GetZ())
    );

  if (turn.w < 0.0) {
    turn = -turn;
  }

  const glm::dvec3 axis(turn.x, turn.y, turn.z);
  const double half = glm::length(axis);
  const glm::dvec3 spin =
    half > 0.0 ? (2.0 * std::atan2(half, turn.w) / (half * step)) * axis
               : glm::dvec3(0.0);
  const glm::dvec3 travel =
    (centre - fromJolt(body.GetCenterOfMassPosition())) / step;

  return {
    linearKept > 0.0
      ? travel / linearKept
          - step * static_cast<double>(motion->GetGravityFactor()) * gravity
      : travel,
    angularKept > 0.0 ? spin / angularKept : spin
  };
}

Momentum bodyMomentum(const JPH::Body& body) {
  const JPH::MotionProperties* motion = body.GetMotionProperties();

  const glm::dvec3 linear = fromJolt(body.GetLinearVelocity());
  const glm::dvec3 angular = fromJolt(body.GetAngularVelocity());

  Momentum held;

  held.linear = linear / static_cast<double>(motion->GetInverseMass());
  held.angular = glm::inverse(fromJolt(body.GetInverseInertia())) * angular;
  held.kinetic =
    0.5 * (glm::dot(linear, held.linear) + glm::dot(angular, held.angular));

  return held;
}

void poseRigidWindow(
  const JPH::Body& body,
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::vec3> shape,
  deformable::ImpactWindow& window
) {
  const std::span<const glm::vec3> local =
    shape.empty() ? std::span<const glm::vec3>(mesh.restPositions) : shape;

  if (local.size() != window.position.size()) {
    return;
  }

  const JPH::Quat turned = body.GetRotation();
  const glm::dmat3 rotation = glm::mat3_cast(
    glm::normalize(
      glm::dquat(turned.GetW(), turned.GetX(), turned.GetY(), turned.GetZ())
    )
  );
  const glm::dvec3 placed = fromJolt(body.GetPosition());
  const glm::dvec3 linear = fromJolt(body.GetLinearVelocity());
  const glm::dvec3 angular = fromJolt(body.GetAngularVelocity());

  std::vector<glm::dvec3> world(local.size());

  for (size_t node = 0u; node != local.size(); ++node) {
    world[node] = placed + rotation * glm::dvec3(local[node]);
  }

  const deformable::RigidMass rigid = deformable::rigidMass(
    std::span<const float>(mesh.nodeMass).first(local.size()),
    std::span<const glm::dvec3>(world)
  );

  if (!(rigid.mass > 0.0)) {
    return;
  }

  const glm::dvec3 centre = rigid.centre;
  const glm::dmat3 inertia = rigid.inertia;

  window.origin = centre;

  for (size_t node = 0u; node != local.size(); ++node) {
    window.position[node] = world[node] - centre;
  }

  const Momentum jolt = bodyMomentum(body);
  const glm::dvec3 spin = glm::determinant(inertia) > 0.0
                            ? glm::inverse(inertia) * jolt.angular
                            : angular;

  double kinetic = 0.0;

  for (size_t node = 0u; node != local.size(); ++node) {
    window.velocity[node] = linear + glm::cross(spin, window.position[node]);
    kinetic += 0.5 * static_cast<double>(mesh.nodeMass[node])
               * glm::dot(window.velocity[node], window.velocity[node]);
  }

  if (kinetic > jolt.kinetic) {
    const double scale = std::sqrt(jolt.kinetic / kinetic);

    for (glm::dvec3& moving : window.velocity) {
      moving *= scale;
    }
  }
}

struct FieldBody {
  const physics::FieldShape* shape = nullptr;
  glm::vec3 fieldOffset{0.0f};
};

FieldBody fieldBodyOf(const JPH::Shape* outer) {
  const JPH::OffsetCenterOfMassShape* offset = nullptr;

  if (outer->GetSubType() == JPH::EShapeSubType::OffsetCenterOfMass) {
    offset = static_cast<const JPH::OffsetCenterOfMassShape*>(outer);
    outer = offset->GetInnerShape();
  }

  const auto* shape = dynamic_cast<const physics::FieldShape*>(outer);

  if (shape == nullptr) {
    return {};
  }

  FieldBody held{shape, shape->centerOfMass()};

  if (offset != nullptr) {
    held.fieldOffset += glm::vec3(fromJolt(offset->GetOffset()));
  }

  return held;
}

enum class PartnerKind : uint8_t {
  Field,
  Unsupported,
  Degenerate
};

PartnerKind windowPartnerOf(
  const JPH::Body& body,
  deformable::WindowPartner& partner
) {
  const FieldBody held = fieldBodyOf(body.GetShape());
  const physics::FieldShape* shape = held.shape;

  if (shape == nullptr) {
    return PartnerKind::Unsupported;
  }

  if (!holdsSolid(shape->field())) {
    return PartnerKind::Degenerate;
  }

  partner.probe.field = &shape->field();
  partner.probe.fieldOffset = held.fieldOffset;

  const JPH::Quat turned = body.GetRotation();

  partner.body = body.GetID().GetIndexAndSequenceNumber();
  partner.centerOfMass = fromJolt(body.GetCenterOfMassPosition());
  partner.rotation =
    glm::dquat(turned.GetW(), turned.GetX(), turned.GetY(), turned.GetZ());
  partner.linearVelocity = fromJolt(body.GetLinearVelocity());
  partner.angularVelocity = fromJolt(body.GetAngularVelocity());
  partner.dynamic = body.GetMotionType() == JPH::EMotionType::Dynamic;

  if (
    const JPH::MotionProperties* motion = body.GetMotionPropertiesUnchecked();
    motion != nullptr && partner.dynamic
  ) {
    partner.inverseMass = static_cast<double>(motion->GetInverseMass());
    partner.inverseInertia = fromJolt(body.GetInverseInertia());
  }

  if (partner.dynamic) {
    partner.seeds.reserve(shape->seeds().size());

    for (const physics::FieldSeed& seed : shape->seeds()) {
      partner.seeds.push_back(seed.point - partner.probe.fieldOffset);
    }
  }

  return PartnerKind::Field;
}

class TouchingBodies final : public JPH::CollideShapeCollector {
public:
  void AddHit(const JPH::CollideShapeResult& result) override {
    if (!bodies.empty() && bodies.back() == result.mBodyID2) {
      return;
    }

    bodies.push_back(result.mBodyID2);
  }

  std::vector<JPH::BodyID> bodies;
};

}

Runtime::Runtime(const objectmodel::World& source, physics::JoltLibrary&) {
  objectmodel::validateBodies(source);
  objectmodel::instantiateWorld(source, m_world);

  const std::span<const objectmodel::PhysicalMaterial> materials =
    source.materials();

  m_world.setMaterials(
    std::vector<objectmodel::PhysicalMaterial>(
      materials.begin(),
      materials.end()
    )
  );

  m_physicsWorld.impacts().setMaterials(m_world.materials());

  m_loadGravity = gravity();
}

objectmodel::World& Runtime::world() noexcept {
  return m_world;
}

const objectmodel::World& Runtime::world() const noexcept {
  return m_world;
}

physics::PhysicsWorld& Runtime::physics() noexcept {
  return m_physicsWorld;
}

JPH::ShapeRefC Runtime::shapeFor(const objectmodel::SharedSdf& held) {
  const talus::field::SampledSdf* key = held.field.get();

  const auto found = m_shapes.find(key);

  if (found != m_shapes.end()) {
    if (!found->second.lattice.expired()) {
      return found->second.shape;
    }

    m_shapes.erase(found);
  }

  JPH::ShapeRefC shape(new physics::FieldShape(*held.field));

  rememberShape(held, shape);

  return shape;
}

void Runtime::rememberShape(
  const objectmodel::SharedSdf& held,
  const JPH::ShapeRefC& shape
) {
  std::erase_if(m_shapes, [](const auto& entry) {
    return entry.second.lattice.expired();
  });

  m_shapes.insert_or_assign(held.field.get(), SharedShape{held.field, shape});
}

void Runtime::refreshBody(objectmodel::Entity entity) {
  refreshBodies(std::span<const objectmodel::Entity>(&entity, 1u));
}

void Runtime::refreshBodies(std::span<const objectmodel::Entity> entities) {
  std::vector<JPH::BodyID> created;

  try {
    for (const objectmodel::Entity entity : entities) {
      refreshInto(entity, created);
    }
  } catch (...) {
    addBodies(created);

    throw;
  }

  addBodies(created);
}

void Runtime::optimizeBroadPhase() {
  m_physicsWorld.optimizeBroadPhase();

  m_broadPhaseStale = false;
  m_addedSinceOptimize = 0u;
  m_optimizeAfter = std::max(
    REOPTIMIZE_MINIMUM_ADDED,
    m_physicsWorld.system().GetNumBodies() / REOPTIMIZE_ADDED_DIVISOR
  );
}

void Runtime::refreshInto(
  objectmodel::Entity entity,
  std::vector<JPH::BodyID>& created
) {
  const entt::registry& registry = m_world.registry();

  const auto* held = registry.try_get<objectmodel::SharedSdf>(entity);

  if (held == nullptr) {
    return;
  }

  if (registry.all_of<objectmodel::DeformableBody>(entity)) {
    static_cast<void>(prepareDeformable(entity));

    return;
  }

  if (!registry.any_of<objectmodel::RigidBody, objectmodel::StaticBody>(
        entity
      )) {
    return;
  }

  placeBody(entity, shapeFor(*held), created);
}

void Runtime::addBodies(std::vector<JPH::BodyID>& created) {
  if (created.empty()) {
    return;
  }

  JPH::BodyInterface& bodies = m_physicsWorld.bodies();

  const int count = static_cast<int>(created.size());

  const JPH::BodyInterface::AddState state =
    bodies.AddBodiesPrepare(created.data(), count);

  bodies.AddBodiesFinalize(
    created.data(),
    count,
    state,
    JPH::EActivation::Activate
  );

  created.clear();

  m_addedSinceOptimize += static_cast<uint32_t>(count);

  if (m_addedSinceOptimize >= m_optimizeAfter) {
    m_broadPhaseStale = true;
  }
}

void Runtime::setDeformableSettings(
  const deformable::Settings& settings
) noexcept {
  m_deformableSettings = settings;
}

const deformable::Settings& Runtime::deformableSettings() const noexcept {
  return m_deformableSettings;
}

const deformable::StepReport& Runtime::deformableReport() const noexcept {
  return m_deformableReport;
}

const DeformableActivity& Runtime::deformableActivity() const noexcept {
  return m_deformableActivity;
}

bool Runtime::deformableQuiescent(objectmodel::Entity entity) const {
  const auto found = m_deformableSolvers.find(entity);

  return found != m_deformableSolvers.end()
         && found->second.mode == DeformableMode::Quiescent;
}

const deformable::DeformableSolverState* Runtime::deformableSolver(
  objectmodel::Entity entity
) const {
  const auto found = m_deformableSolvers.find(entity);

  return found == m_deformableSolvers.end() ? nullptr : &found->second.solver;
}

bool Runtime::rigidUntilYield(objectmodel::Entity entity) const {
  return m_yielding.contains(entity);
}

bool Runtime::promotedDeformable(objectmodel::Entity entity) const {
  const auto found = m_yielding.find(entity);

  return found != m_yielding.end() && found->second.promoted;
}

bool Runtime::forcePromotion(objectmodel::Entity entity) {
  const auto found = m_yielding.find(entity);

  if (found == m_yielding.end() || found->second.promoted) {
    return false;
  }

  found->second.forced = true;

  return true;
}

const deformable::ImpactWindow* Runtime::impactWindow(
  objectmodel::Entity entity
) const {
  const auto found = m_yielding.find(entity);

  if (
    found == m_yielding.end() || !found->second.promoted
    || !found->second.window.has_value()
  ) {
    return nullptr;
  }

  return &*found->second.window;
}

std::span<const glm::vec3> Runtime::storedShape(
  objectmodel::Entity entity
) const {
  const auto found = m_yielding.find(entity);

  if (found == m_yielding.end()) {
    return {};
  }

  return found->second.shape;
}

bool Runtime::armedDeformable(objectmodel::Entity entity) const {
  const auto found = m_deformableSolvers.find(entity);

  return found != m_deformableSolvers.end()
         && found->second.mode == DeformableMode::Arming;
}

Runtime::SleepGate Runtime::sleepGate() const {
  const JPH::PhysicsSettings& sleeping =
    m_physicsWorld.system().GetPhysicsSettings();

  return SleepGate{
    sleeping.mPointVelocitySleepThreshold * sleeping.mTimeBeforeSleep,
    glm::max(
      1u,
      static_cast<uint32_t>(std::lround(
        sleeping.mTimeBeforeSleep / physics::PhysicsWorld::TIME_STEP
      ))
    )
  };
}

glm::vec3 Runtime::gravity() const {
  const JPH::Vec3 pull = m_physicsWorld.system().GetGravity();

  return glm::vec3(pull.GetX(), pull.GetY(), pull.GetZ());
}

void Runtime::markBody(
  uint32_t body,
  WakeReason reason,
  const std::optional<Bounds>& before
) {
  m_marks.push_back(BodyMark{body, reason, before});
}

std::optional<Runtime::Bounds> Runtime::bodyBounds(uint32_t body) const {
  const JPH::BodyLockRead lock(
    m_physicsWorld.system().GetBodyLockInterface(),
    JPH::BodyID(body)
  );

  if (!lock.Succeeded()) {
    return std::nullopt;
  }

  const JPH::AABox& box = lock.GetBody().GetWorldSpaceBounds();

  return Bounds{
    glm::vec3(box.mMin.GetX(), box.mMin.GetY(), box.mMin.GetZ()),
    glm::vec3(box.mMax.GetX(), box.mMax.GetY(), box.mMax.GetZ())
  };
}

void Runtime::countWake(WakeReason reason) noexcept {
  ++m_deformableActivity.wakes;

  switch (reason) {
    case WakeReason::Edit:
      ++m_deformableActivity.editWakes;
      break;
    case WakeReason::Mover:
      ++m_deformableActivity.moverWakes;
      break;
    case WakeReason::Destroy:
      ++m_deformableActivity.destroyWakes;
      break;
    case WakeReason::Activation:
      ++m_deformableActivity.activationWakes;
      break;
    case WakeReason::Global:
      ++m_deformableActivity.globalWakes;
      break;
    case WakeReason::Other:
      ++m_deformableActivity.otherWakes;
      break;
  }
}

void Runtime::refreshStepping() {
  if (!m_steppingStale) {
    return;
  }

  m_steppingStale = false;
  m_stepping.clear();
  m_quiescent.clear();

  for (const objectmodel::Entity entity : m_world.registry()
                                            .view<
                                              objectmodel::DeformablePrepared,
                                              objectmodel::DeformableState>()) {
    const auto found = m_deformableSolvers.find(entity);

    if (m_yielding.contains(entity)) {
      continue;
    }

    if (
      found != m_deformableSolvers.end()
      && found->second.mode == DeformableMode::Quiescent
    ) {
      m_quiescent.push_back(entity);
    } else {
      m_stepping.push_back(entity);
    }
  }
}

bool Runtime::prepareDeformable(objectmodel::Entity entity) {
  const entt::registry& registry = m_world.registry();

  const auto* held = registry.try_get<objectmodel::SharedSdf>(entity);

  if (held == nullptr || held->field == nullptr) {
    return false;
  }

  std::vector<deformable::SupportVolume> supports;

  for (const auto [holder, box, pose] :
       registry.view<const objectmodel::Support, const objectmodel::Pose>()
         .each()) {
    supports.push_back(
      deformable::SupportVolume{
        glm::inverse(objectmodel::model(pose)),
        box.halfExtents
      }
    );
  }

  const objectmodel::Pose& pose = registry.get<objectmodel::Pose>(entity);

  const auto* authored = registry.try_get<objectmodel::DeformableBody>(entity);

  const objectmodel::VolumeElementType element =
    authored != nullptr && authored->element.has_value()
      ? *authored->element
      : m_deformableSettings.element;

  const std::shared_ptr<const objectmodel::PreparedMesh> mesh =
    deformable::buildMesh(
      *held->field,
      m_world.materials(),
      objectmodel::model(pose),
      supports,
      element
    );

  if (mesh == nullptr) {
    return false;
  }

  if (
    const auto found = m_deformableSolvers.find(entity);
    found != m_deformableSolvers.end()
  ) {
    if (found->second.mode != DeformableMode::Active) {
      ++m_pendingOtherWakes;
    }

    static_cast<void>(wakeIsland(entity));

    releaseIsland(entity, found->second);
  }

  const std::optional<float> trigger =
    m_deformableSettings.rigidUntilYield
      ? yieldTrigger(*mesh, m_world.materials())
      : std::nullopt;

  if (trigger.has_value()) {
    const bool wasPromoted = promotedDeformable(entity);

    YieldTrack& track = m_yielding[entity];

    track = YieldTrack{};
    track.trigger = *trigger;

    const JPH::ShapeRefC shape = yieldShape(shapeFor(*held), *mesh);

    if (
      const auto* body = registry.try_get<objectmodel::BodyId>(entity);
      body != nullptr && wasPromoted
    ) {
      m_physicsWorld.bodies().SetPositionAndRotation(
        JPH::BodyID(body->id),
        JPH::RVec3(pose.position.x, pose.position.y, pose.position.z),
        JPH::Quat(
          pose.rotation.x,
          pose.rotation.y,
          pose.rotation.z,
          pose.rotation.w
        ),
        JPH::EActivation::DontActivate
      );
      m_physicsWorld.bodies().AddBody(
        JPH::BodyID(body->id),
        JPH::EActivation::Activate
      );
    }

    std::vector<JPH::BodyID> created;

    placeBody(entity, shape, created);

    addBodies(created);

    if (const auto* body = registry.try_get<objectmodel::BodyId>(entity)) {
      applyYieldMotion(body->id, *mesh, track);
    }
  } else if (const auto* body = registry.try_get<objectmodel::BodyId>(entity)) {
    const JPH::BodyID id(body->id);

    markBody(body->id, WakeReason::Edit, bodyBounds(body->id));

    if (m_physicsWorld.bodies().IsAdded(id)) {
      m_physicsWorld.bodies().RemoveBody(id);
    }

    m_physicsWorld.bodies().DestroyBody(id);

    m_world.clearBodyId(entity);
  }

  if (!trigger.has_value()) {
    m_yielding.erase(entity);
  }

  m_world.setDeformablePrepared(entity, mesh);
  m_world.setDeformableState(entity, deformable::restState(*mesh, pose));
  m_deformableSolvers.erase(entity);
  m_steppingStale = true;

  return true;
}

void Runtime::applyYieldMotion(
  uint32_t body,
  const objectmodel::PreparedMesh& mesh,
  const YieldTrack& track
) {
  const deformable::RigidMass held = meshMass(
    mesh,
    track.shape.empty() ? std::span<const glm::vec3>(mesh.restPositions)
                        : std::span<const glm::vec3>(track.shape)
  );

  JPH::BodyLockWrite lock(
    m_physicsWorld.system().GetBodyLockInterface(),
    JPH::BodyID(body)
  );

  if (!lock.Succeeded()) {
    return;
  }

  JPH::MotionProperties* motion = lock.GetBody().GetMotionPropertiesUnchecked();

  if (motion == nullptr) {
    return;
  }

  motion->SetLinearDamping(0.0f);
  motion->SetAngularDamping(0.0f);
  motion->SetMaxAngularVelocity(
    2.0f * JPH::JPH_PI / physics::PhysicsWorld::TIME_STEP
  );

  if (!(held.mass > 0.0)) {
    return;
  }

  JPH::MassProperties properties;

  properties.mMass = static_cast<float>(held.mass);
  properties.mInertia = JPH::Mat44::sIdentity();

  for (int column = 0; column != 3; ++column) {
    properties.mInertia.SetColumn3(column, toJolt(held.inertia[column]));
  }

  motion->SetMassProperties(JPH::EAllowedDOFs::All, properties);
}

void Runtime::reshapeAfterDeform(
  objectmodel::Entity entity,
  const glm::uvec3& brickBegin,
  const glm::uvec3& brickEnd
) {
  const entt::registry& registry = m_world.registry();

  const auto* held = registry.try_get<objectmodel::SharedSdf>(entity);
  const auto* body = registry.try_get<objectmodel::BodyId>(entity);

  if (held == nullptr || body == nullptr) {
    return;
  }

  const talus::field::SampledSdf* field = held->field.get();

  const JPH::Shape* current =
    m_physicsWorld.bodies().GetShape(JPH::BodyID(body->id));

  const auto* shape = dynamic_cast<const physics::FieldShape*>(current);

  if (shape == nullptr || &shape->field() != field) {
    refreshBody(entity);
    return;
  }

  const JPH::ShapeRefC reshaped(
    new physics::FieldShape(*field, *shape, brickBegin, brickEnd)
  );

  rememberShape(*held, reshaped);

  setBodyShape(entity, reshaped);
}

size_t Runtime::shapeCount() const noexcept {
  return m_shapes.size();
}

void Runtime::refreshDeformedBodies() {
  for (const auto& [entity, box] : m_world.sdfDirty()) {
    const auto* held =
      m_world.registry().try_get<objectmodel::SharedSdf>(entity);

    if (held == nullptr || held->field == nullptr) {
      continue;
    }

    const field::SampledSdf& lattice = *held->field;

    const field::BrickRange bricks = field::brickRange(lattice, box);

    reshapeAfterDeform(entity, bricks.begin, bricks.end);

    if (!m_world.registry().all_of<objectmodel::Pose>(entity)) {
      continue;
    }

    const glm::vec3 low =
      lattice.origin + glm::vec3(box.minimum) * lattice.voxelSize;
    const glm::vec3 high =
      lattice.origin + glm::vec3(box.minimum + box.extent) * lattice.voxelSize;

    const glm::mat4 model =
      objectmodel::model(m_world.registry().get<objectmodel::Pose>(entity));

    glm::vec3 minimum(std::numeric_limits<float>::max());
    glm::vec3 maximum(std::numeric_limits<float>::lowest());

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::vec3 pick(
        (corner & 1u) != 0u ? high.x : low.x,
        (corner & 2u) != 0u ? high.y : low.y,
        (corner & 4u) != 0u ? high.z : low.z
      );

      const glm::vec3 at = glm::vec3(model * glm::vec4(pick, 1.0f));

      minimum = glm::min(minimum, at);
      maximum = glm::max(maximum, at);
    }

    const glm::vec3 slack(glm::compMax(lattice.voxelSize) * 4.0f);

    wake(minimum - slack, maximum + slack);
  }
}

void Runtime::wake(const glm::vec3& minimum, const glm::vec3& maximum) {
  m_physicsWorld.bodies().ActivateBodiesInAABox(
    JPH::AABox(
      JPH::Vec3(minimum.x, minimum.y, minimum.z),
      JPH::Vec3(maximum.x, maximum.y, maximum.z)
    ),
    {},
    {}
  );
}

void Runtime::setBodyShape(
  objectmodel::Entity entity,
  const JPH::ShapeRefC& shape
) {
  std::vector<JPH::BodyID> created;

  placeBody(entity, shape, created);

  addBodies(created);
}

void Runtime::placeBody(
  objectmodel::Entity entity,
  const JPH::ShapeRefC& shape,
  std::vector<JPH::BodyID>& created
) {
  const entt::registry& registry = m_world.registry();

  if (
    registry.all_of<objectmodel::DeformablePrepared>(entity)
    && !m_yielding.contains(entity)
  ) {
    return;
  }

  const bool isStatic = registry.all_of<objectmodel::StaticBody>(entity);

  const bool hasMass = shape->GetMassProperties().mMass > 0.0f;

  JPH::BodyInterface& bodies = m_physicsWorld.bodies();

  if (const auto* body = registry.try_get<objectmodel::BodyId>(entity)) {
    markBody(body->id, WakeReason::Edit, bodyBounds(body->id));

    bodies.SetShape(
      JPH::BodyID(body->id),
      shape,
      !isStatic && hasMass,
      isStatic ? JPH::EActivation::DontActivate : JPH::EActivation::Activate
    );

    applyMassScale(entity);

    return;
  }

  if (!isStatic && !hasMass) {
    if (!m_masslessReported) {
      m_masslessReported = true;
      std::cout << "A field with nothing solid in it gets no body" << std::endl;
    }

    return;
  }

  const objectmodel::Pose& pose = registry.get<objectmodel::Pose>(entity);

  JPH::BodyCreationSettings settings(
    shape,
    JPH::RVec3(pose.position.x, pose.position.y, pose.position.z),
    JPH::Quat(
      pose.rotation.x,
      pose.rotation.y,
      pose.rotation.z,
      pose.rotation.w
    ),
    isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic,
    isStatic ? physics::ObjectLayers::NON_MOVING : physics::ObjectLayers::MOVING
  );

  settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

  settings.mUserData = static_cast<JPH::uint64>(static_cast<uint32_t>(entity));

  const JPH::Body* made = bodies.CreateBody(settings);

  if (made == nullptr) {
    throw std::runtime_error("Runtime: a field entity got no body");
  }

  const JPH::BodyID id = made->GetID();

  created.push_back(id);

  markBody(id.GetIndexAndSequenceNumber(), WakeReason::Edit, std::nullopt);

  m_world.setBodyId(entity, id.GetIndexAndSequenceNumber());

  applyMassScale(entity);
}

void Runtime::setLinearVelocity(
  objectmodel::Entity entity,
  const glm::vec3& velocity
) {
  const auto* body = m_world.registry().try_get<objectmodel::BodyId>(entity);

  if (body == nullptr) {
    return;
  }

  const JPH::BodyID id(body->id);

  markBody(body->id, WakeReason::Edit, std::nullopt);

  if (promotedDeformable(entity)) {
    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
    YieldTrack& track = m_yielding.at(entity);

    if (mesh == nullptr || !track.window.has_value()) {
      return;
    }

    deformable::ImpactWindow& window = *track.window;

    double mass = 0.0;
    glm::dvec3 linear(0.0);

    for (size_t node = 0u; node != window.velocity.size(); ++node) {
      const double weight = static_cast<double>(mesh->nodeMass[node]);

      mass += weight;
      linear += weight * window.velocity[node];
    }

    if (!(mass > 0.0)) {
      return;
    }

    deformable::editWindowVelocity(
      window,
      *mesh,
      glm::dvec3(velocity) - linear / mass
    );

    m_world.patchDeformableState(
      entity,
      [&](objectmodel::DeformableState& state) {
        deformable::mirrorWindow(window, state);
      }
    );

    return;
  }

  m_physicsWorld.bodies().SetLinearVelocity(
    id,
    JPH::Vec3(velocity.x, velocity.y, velocity.z)
  );
  m_physicsWorld.bodies().ActivateBody(id);
}

void Runtime::setMass(objectmodel::Entity entity, float mass) {
  const auto* body = m_world.registry().try_get<objectmodel::BodyId>(entity);

  if (body == nullptr || mass <= 0.0f) {
    return;
  }

  if (m_yielding.contains(entity)) {
    std::cout << "A yielding deformable's mass is its material's; setMass is "
                 "refused"
              << std::endl;

    return;
  }

  markBody(body->id, WakeReason::Edit, bodyBounds(body->id));

  JPH::BodyLockWrite lock(
    m_physicsWorld.system().GetBodyLockInterface(),
    JPH::BodyID(body->id)
  );

  if (!lock.Succeeded()) {
    return;
  }

  JPH::MotionProperties* motion = lock.GetBody().GetMotionPropertiesUnchecked();

  if (motion == nullptr) {
    return;
  }

  motion->ScaleToMass(mass);

  const float derived = lock.GetBody().GetShape()->GetMassProperties().mMass;

  if (derived > 0.0f) {
    m_world.emplaceOrReplace<objectmodel::MassScale>(entity, {mass / derived});
  }
}

void Runtime::applyMassScale(objectmodel::Entity entity) {
  const entt::registry& registry = m_world.registry();

  const auto* scale = registry.try_get<objectmodel::MassScale>(entity);
  const auto* body = registry.try_get<objectmodel::BodyId>(entity);

  if (scale == nullptr || body == nullptr) {
    return;
  }

  JPH::BodyLockWrite lock(
    m_physicsWorld.system().GetBodyLockInterface(),
    JPH::BodyID(body->id)
  );

  if (!lock.Succeeded()) {
    return;
  }

  JPH::MotionProperties* motion = lock.GetBody().GetMotionPropertiesUnchecked();
  const float derived = lock.GetBody().GetShape()->GetMassProperties().mMass;

  if (motion != nullptr && derived > 0.0f) {
    motion->ScaleToMass(derived * scale->factor);
  }
}

bool Runtime::destroy(objectmodel::Entity entity) {
  if (
    const auto* body = m_world.registry().try_get<objectmodel::BodyId>(entity)
  ) {
    const JPH::BodyID id(body->id);

    markBody(body->id, WakeReason::Destroy, bodyBounds(body->id));

    if (m_physicsWorld.bodies().IsAdded(id)) {
      m_physicsWorld.bodies().RemoveBody(id);
    }

    m_physicsWorld.bodies().DestroyBody(id);
  }

  if (m_world.deformablePrepared(entity) != nullptr) {
    m_steppingStale = true;
  }

  m_yielding.erase(entity);

  if (wakeIsland(entity)) {
    ++m_pendingDestroyWakes;
  }

  if (
    const auto found = m_deformableSolvers.find(entity);
    found != m_deformableSolvers.end()
  ) {
    releaseIsland(entity, found->second);

    m_deformableSolvers.erase(found);
  }

  return m_world.destroy(entity);
}

void Runtime::applyPoseEdits() {
  for (const objectmodel::Entity entity : m_world.takePoseEdits()) {
    const auto* body = m_world.registry().try_get<objectmodel::BodyId>(entity);

    if (body == nullptr) {
      continue;
    }

    if (promotedDeformable(entity)) {
      std::cout << "A pose edit on a promoted deformable is refused until it "
                   "returns to Jolt"
                << std::endl;

      continue;
    }

    const objectmodel::Pose& pose =
      m_world.registry().get<objectmodel::Pose>(entity);

    markBody(body->id, WakeReason::Edit, bodyBounds(body->id));

    m_physicsWorld.bodies().SetPositionAndRotation(
      JPH::BodyID(body->id),
      JPH::RVec3(pose.position.x, pose.position.y, pose.position.z),
      JPH::Quat(
        pose.rotation.x,
        pose.rotation.y,
        pose.rotation.z,
        pose.rotation.w
      ),
      JPH::EActivation::Activate
    );

    m_previousPoses[static_cast<uint32_t>(entity)] = pose;
  }
}

void Runtime::reportUnsupported(
  objectmodel::Entity entity,
  const JPH::BodyID& id
) {
  const uint64_t pair =
    (static_cast<uint64_t>(static_cast<uint32_t>(entity)) << 32u)
    | id.GetIndexAndSequenceNumber();

  if (!m_unsupportedSeen.insert(pair).second) {
    return;
  }

  std::cout << "A deformable meets a body its contact path cannot read"
            << std::endl;
}

void Runtime::touchBodies(
  objectmodel::Entity entity,
  float pad,
  DeformableSlot& slot
) {
  slot.touching.clear();

  const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
  const objectmodel::DeformableState* state = m_world.deformableState(entity);

  if (mesh == nullptr || state == nullptr || mesh->surfaceTriangles.empty()) {
    return;
  }

  JPH::AABox bounds = surfaceBox(*mesh, *state);

  if (pad > 0.0f) {
    bounds.ExpandBy(JPH::Vec3::sReplicate(pad));
  }

  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> found;

  m_physicsWorld.system().GetBroadPhaseQuery().CollideAABox(bounds, found);

  slot.touching.assign(found.mHits.begin(), found.mHits.end());
}

void Runtime::gatherContacts(
  objectmodel::Entity entity,
  const glm::vec3& gravity,
  std::span<const uint32_t> woken,
  DeformableSlot& slot
) {
  slot.contacts.clear();
  slot.partners.clear();
  slot.probes.clear();
  slot.unsupported.clear();

  const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
  const objectmodel::DeformableState* state = m_world.deformableState(entity);

  if (mesh == nullptr || state == nullptr || mesh->surfaceTriangles.empty()) {
    return;
  }

  const bool armed = armedDeformable(entity);

  const auto* own = m_world.registry().try_get<objectmodel::BodyId>(entity);

  for (const JPH::BodyID& id : slot.touching) {
    if (own != nullptr && id.GetIndexAndSequenceNumber() == own->id) {
      continue;
    }

    const JPH::BodyLockRead lock(
      m_physicsWorld.system().GetBodyLockInterface(),
      id
    );

    if (!lock.Succeeded()) {
      continue;
    }

    const JPH::Body& body = lock.GetBody();

    if (body.IsSensor()) {
      continue;
    }

    const FieldBody fieldBody = fieldBodyOf(body.GetShape());

    if (fieldBody.shape == nullptr) {
      slot.unsupported.push_back(id.GetIndexAndSequenceNumber());

      continue;
    }

    deformable::RigidPartner partner;

    partner.body = id.GetIndexAndSequenceNumber();

    const JPH::Vec3 centre = body.GetCenterOfMassPosition();
    const JPH::Quat turned = body.GetRotation();
    const JPH::Vec3 linear = body.GetLinearVelocity();
    const JPH::Vec3 angular = body.GetAngularVelocity();

    partner.centerOfMass =
      glm::vec3(centre.GetX(), centre.GetY(), centre.GetZ());
    partner.rotation =
      glm::quat(turned.GetW(), turned.GetX(), turned.GetY(), turned.GetZ());
    partner.linearVelocity =
      glm::vec3(linear.GetX(), linear.GetY(), linear.GetZ());
    partner.angularVelocity =
      glm::vec3(angular.GetX(), angular.GetY(), angular.GetZ());

    const bool awake =
      body.IsActive() || std::ranges::find(woken, partner.body) != woken.end();

    partner.dynamic =
      body.GetMotionType() == JPH::EMotionType::Dynamic && (!armed || awake);
    partner.friction = body.GetFriction();
    partner.restitution = body.GetRestitution();

    if (
      const JPH::MotionProperties* motion = body.GetMotionPropertiesUnchecked();
      motion != nullptr && partner.dynamic
    ) {
      partner.inverseMass = motion->GetInverseMass();
      partner.linearVelocity +=
        gravity * motion->GetGravityFactor() * physics::PhysicsWorld::TIME_STEP;

      const JPH::Mat44 inertia = body.GetInverseInertia();

      for (int column = 0; column != 3; ++column) {
        const JPH::Vec3 held = inertia.GetColumn3(column);

        partner.inverseInertia[column] =
          glm::vec3(held.GetX(), held.GetY(), held.GetZ());
      }
    }

    partner.originalLinearVelocity = partner.linearVelocity;
    partner.originalAngularVelocity = partner.angularVelocity;

    deformable::ShapeProbe probe;

    probe.field = &fieldBody.shape->field();
    probe.centerOfMass = partner.centerOfMass;
    probe.rotation = partner.rotation;
    probe.fieldOffset = fieldBody.fieldOffset;
    probe.linearVelocity = partner.linearVelocity;
    probe.angularVelocity = partner.angularVelocity;

    slot.partners.push_back(partner);
    slot.probes.push_back(probe);
  }

  slot.contacts = deformable::findContacts(
    *mesh,
    *state,
    m_world.materials(),
    gravity,
    slot.probes,
    physics::PhysicsWorld::TIME_STEP,
    contactGaps(*mesh)
  );
}

void Runtime::stepDeformables() {
  refreshStepping();

  m_unsupportedPartners = 0u;
  m_deformableReport = deformable::StepReport{};

  if (m_stepping.empty()) {
    return;
  }

  const glm::vec3 pull = gravity();
  const SleepGate gate = sleepGate();

  m_deformableRun.clear();

  bool startsNonFinite = false;

  for (const objectmodel::Entity entity : m_stepping) {
    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
    const objectmodel::DeformableState* state = m_world.deformableState(entity);

    if (mesh == nullptr || state == nullptr) {
      continue;
    }

    if (!finiteState(*state)) {
      startsNonFinite = true;

      break;
    }

    m_deformableSolvers.try_emplace(entity);
    m_deformableRun.push_back(entity);
  }

  if (m_deformableSlots.size() < m_deformableRun.size()) {
    m_deformableSlots.resize(m_deformableRun.size());
  }

  for (size_t index = 0u; index != m_deformableRun.size(); ++index) {
    DeformableSlot& slot = m_deformableSlots[index];

    slot.failure = nullptr;
    slot.wakes.clear();

    touchBodies(m_deformableRun[index], 0.0f, slot);
  }

  const size_t threads =
    m_deformableSettings.stepThreads != 0u
      ? m_deformableSettings.stepThreads
      : std::max(std::thread::hardware_concurrency(), 2u) - 1u;

  groupDeformables(threads);

  const size_t groups = m_groupStarts.size() - 1u;
  const size_t workers = std::max<size_t>(std::min(threads, groups), 1u);
  const unsigned int control = _mm_getcsr();

  std::atomic<size_t> next{0u};
  std::atomic<uint32_t> limit{static_cast<uint32_t>(m_deformableRun.size())};

  const auto work = [&] {
    _mm_setcsr(control);

    std::vector<uint32_t> woken;

    for (size_t group = next.fetch_add(1u);
         group < groups && m_groupMembers[m_groupStarts[group]] <= limit;
         group = next.fetch_add(1u)) {
      stepGroup(group, pull, gate, woken, limit);
    }
  };

  {
    std::vector<std::jthread> helpers;
    helpers.reserve(workers - 1u);

    for (size_t worker = 1u; worker != workers; ++worker) {
      helpers.emplace_back(work);
    }

    work();
  }

  size_t stop = 0u;

  while (stop != m_deformableRun.size()
         && applyDeformable(m_deformableRun[stop], m_deformableSlots[stop])) {
    ++stop;
  }

  if (stop == m_deformableRun.size()) {
    if (startsNonFinite) {
      m_deformableReport.nonFinite = true;
    }

    return;
  }

  for (size_t later = stop; later != m_deformableRun.size(); ++later) {
    activatePartners(m_deformableSlots[later]);
  }

  if (m_deformableSlots[stop].failure) {
    std::rethrow_exception(m_deformableSlots[stop].failure);
  }
}

void Runtime::groupDeformables(size_t threads) {
  const uint32_t count = static_cast<uint32_t>(m_deformableRun.size());

  m_groupParents.resize(count);

  for (uint32_t index = 0u; index != count; ++index) {
    m_groupParents[index] = threads > 1u ? index : 0u;
  }

  const auto root = [this](uint32_t index) {
    while (m_groupParents[index] != index) {
      m_groupParents[index] = m_groupParents[m_groupParents[index]];
      index = m_groupParents[index];
    }

    return index;
  };

  if (threads > 1u) {
    m_groupOwners.clear();

    const JPH::BodyLockInterface& locks =
      m_physicsWorld.system().GetBodyLockInterface();

    for (uint32_t index = 0u; index != count; ++index) {
      for (const JPH::BodyID& id : m_deformableSlots[index].touching) {
        const JPH::BodyLockRead lock(locks, id);

        if (!lock.Succeeded()) {
          continue;
        }

        const JPH::Body& body = lock.GetBody();

        if (
          body.IsSensor() || body.GetMotionType() != JPH::EMotionType::Dynamic
          || fieldBodyOf(body.GetShape()).shape == nullptr
        ) {
          continue;
        }

        const auto [owner, fresh] =
          m_groupOwners.try_emplace(id.GetIndexAndSequenceNumber(), index);

        if (fresh) {
          continue;
        }

        const uint32_t mine = root(index);
        const uint32_t theirs = root(owner->second);

        m_groupParents[std::max(mine, theirs)] = std::min(mine, theirs);
      }
    }
  }

  for (uint32_t index = 0u; index != count; ++index) {
    m_groupParents[index] = root(index);
  }

  m_groupMembers.resize(count);

  std::iota(m_groupMembers.begin(), m_groupMembers.end(), 0u);

  std::ranges::stable_sort(
    m_groupMembers,
    std::less<>{},
    [this](uint32_t index) { return m_groupParents[index]; }
  );

  m_groupStarts.clear();

  for (uint32_t at = 0u; at != count; ++at) {
    if (
      at == 0u
      || m_groupParents[m_groupMembers[at]]
           != m_groupParents[m_groupMembers[at - 1u]]
    ) {
      m_groupStarts.push_back(at);
    }
  }

  m_groupStarts.push_back(count);
}

void Runtime::stepGroup(
  size_t group,
  const glm::vec3& pull,
  const SleepGate& gate,
  std::vector<uint32_t>& woken,
  std::atomic<uint32_t>& limit
) {
  woken.clear();

  for (uint32_t at = m_groupStarts[group]; at != m_groupStarts[group + 1u];
       ++at) {
    const uint32_t index = m_groupMembers[at];

    if (index > limit) {
      return;
    }

    const objectmodel::Entity entity = m_deformableRun[index];
    DeformableSlot& slot = m_deformableSlots[index];

    try {
      stepDeformable(entity, pull, gate, woken, slot);

      if (!exchangeDeformable(entity, slot, woken)) {
        return;
      }
    } catch (...) {
      slot.failure = std::current_exception();

      uint32_t seen = limit;

      while (index < seen && !limit.compare_exchange_weak(seen, index)) {
      }

      return;
    }
  }
}

void Runtime::stepDeformable(
  objectmodel::Entity entity,
  const glm::vec3& pull,
  const SleepGate& gate,
  std::span<const uint32_t> woken,
  DeformableSlot& slot
) {
  const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);

  gatherContacts(entity, pull, woken, slot);

  slot.report = deformable::StepReport{};
  slot.armAttempts = 0u;
  slot.armAborts = 0u;

  deformable::StepReport& entityReport = slot.report;

  entityReport.contacts = static_cast<uint32_t>(slot.contacts.size());

  bool stateIsFinite = true;

  m_world.patchDeformableState(
    entity,
    [&](objectmodel::DeformableState& state) {
      DeformableTrack& held = m_deformableSolvers.at(entity);
      DeformableArming& arming = held.arming;

      if (arming.anchor.size() != state.position.size()) {
        arming.anchor = state.position;
      }

      deformable::step(
        *mesh,
        state,
        m_world.materials(),
        m_deformableSettings,
        pull,
        physics::PhysicsWorld::TIME_STEP,
        slot.contacts,
        slot.partners,
        &held.solver,
        entityReport
      );

      ++held.solver.stats.factorAge;
      held.solver.stats.lastIterations = entityReport.cgIterations;
      held.solver.stats.lastResidual = entityReport.cgWorstRelativeResidual;

      stateIsFinite = finiteState(state);

      float drift = 0.0f;

      for (size_t node = 0u; node != mesh->anchored.size(); ++node) {
        if (mesh->anchored[node] != 0u) {
          continue;
        }

        drift = glm::max(
          drift,
          glm::distance(state.position[node], arming.anchor[node])
        );
      }

      const bool healthy =
        stateIsFinite && entityReport.cgCappedSolves == 0u
        && (entityReport.contacts == 0u
            || entityReport.contactResidual
                 <= m_deformableSettings.contactTolerance);

      const bool quiet = healthy && drift <= gate.allowance;

      if (arming.holdoff != 0u) {
        --arming.holdoff;

        if (arming.holdoff == 0u) {
          arming.anchor = state.position;
          arming.quiet = 0u;
        }
      } else if (held.mode == DeformableMode::Arming) {
        if (!quiet) {
          disarm(held, state.position, gate.window, slot.armAborts);
        }
      } else if (!quiet) {
        arming.anchor = state.position;
        arming.quiet = 0u;
      } else {
        ++arming.quiet;

        if (arming.quiet >= gate.window) {
          held.mode = DeformableMode::Arming;
          arming.consecutiveAborts = 0u;

          ++slot.armAttempts;
        }
      }

      arming.partners.clear();

      if (held.mode != DeformableMode::Arming) {
        arming.load.clear();
        arming.loadSteps = 0u;

        return;
      }

      if (arming.load.size() != state.position.size()) {
        arming.load.assign(state.position.size(), glm::vec3(0.0f));
      }

      for (size_t node = 0u; node != entityReport.contactLoad.size()
                             && node != arming.load.size();
           ++node) {
        arming.load[node] += entityReport.contactLoad[node];
      }

      ++arming.loadSteps;

      for (const deformable::SurfaceContact& contact : slot.contacts) {
        if (
          contact.partner < 0
          || static_cast<size_t>(contact.partner) >= slot.partners.size()
        ) {
          continue;
        }

        arming.partners.push_back(
          slot.partners[static_cast<size_t>(contact.partner)].body
        );
      }

      std::sort(arming.partners.begin(), arming.partners.end());

      arming.partners.erase(
        std::unique(arming.partners.begin(), arming.partners.end()),
        arming.partners.end()
      );
    }
  );

  for (deformable::ContactSolve& solve : entityReport.contactSolves) {
    solve.deformable = static_cast<uint32_t>(entity);
  }

  slot.finite = stateIsFinite;
}

bool Runtime::exchangeDeformable(
  objectmodel::Entity entity,
  DeformableSlot& slot,
  std::vector<uint32_t>& woken
) {
  slot.wakes.clear();

  for (const deformable::RigidPartner& partner : slot.partners) {
    slot.finite = slot.finite && finiteValue(partner.linearVelocity)
                  && finiteValue(partner.angularVelocity);
  }

  if (!slot.finite) {
    return false;
  }

  for (const deformable::RigidPartner& partner : slot.partners) {
    const glm::vec3 linear =
      partner.linearVelocity - partner.originalLinearVelocity;
    const glm::vec3 angular =
      partner.angularVelocity - partner.originalAngularVelocity;

    if (finiteValue(linear) && finiteValue(angular)) {
      continue;
    }

    slot.finite = false;

    return false;
  }

  const bool armed = armedDeformable(entity);

  for (const deformable::RigidPartner& partner : slot.partners) {
    const glm::vec3 linear =
      partner.linearVelocity - partner.originalLinearVelocity;
    const glm::vec3 angular =
      partner.angularVelocity - partner.originalAngularVelocity;

    if (linear == glm::vec3(0.0f) && angular == glm::vec3(0.0f)) {
      continue;
    }

    if (!armed) {
      slot.wakes.emplace_back(partner.body, true);
      woken.push_back(partner.body);
    }

    const JPH::BodyLockWrite lock(
      m_physicsWorld.system().GetBodyLockInterface(),
      JPH::BodyID(partner.body)
    );

    if (!lock.Succeeded()) {
      continue;
    }

    JPH::Body& body = lock.GetBody();

    if (body.IsStatic()) {
      continue;
    }

    body.SetLinearVelocityClamped(
      body.GetLinearVelocity() + JPH::Vec3(linear.x, linear.y, linear.z)
    );
    body.SetAngularVelocityClamped(
      body.GetAngularVelocity() + JPH::Vec3(angular.x, angular.y, angular.z)
    );

    const bool moving = !body.GetLinearVelocity().IsNearZero()
                        || !body.GetAngularVelocity().IsNearZero();

    if (body.IsInBroadPhase() && moving) {
      slot.wakes.emplace_back(partner.body, false);
      woken.push_back(partner.body);
    }
  }

  return true;
}

bool Runtime::applyDeformable(
  objectmodel::Entity entity,
  DeformableSlot& slot
) {
  for (const uint32_t unsupported : slot.unsupported) {
    ++m_unsupportedPartners;
    reportUnsupported(entity, JPH::BodyID(unsupported));
  }

  if (slot.failure) {
    return false;
  }

  m_deformableActivity.armAttempts += slot.armAttempts;
  m_deformableActivity.armAborts += slot.armAborts;

  mergeStepReport(slot.report, m_deformableReport);

  m_deformableReport.unsupportedPartners = m_unsupportedPartners;

  if (!slot.finite) {
    m_deformableReport.nonFinite = true;

    return false;
  }

  activatePartners(slot);

  return true;
}

void Runtime::activatePartners(const DeformableSlot& slot) {
  JPH::BodyInterface& bodies = m_physicsWorld.bodies();

  for (const auto& [body, always] : slot.wakes) {
    const JPH::BodyID id(body);

    if (always || !bodies.IsActive(id)) {
      bodies.ActivateBody(id);
    }
  }
}

void Runtime::disarm(
  DeformableTrack& track,
  std::span<const glm::vec3> position,
  uint32_t holdoff,
  uint32_t& aborts
) {
  track.mode = DeformableMode::Active;
  track.arming.quiet = 0u;
  track.arming.holdoff = holdoff;
  track.arming.anchor.assign(position.begin(), position.end());
  track.arming.load.clear();
  track.arming.loadSteps = 0u;
  track.arming.partners.clear();

  ++track.arming.consecutiveAborts;
  ++aborts;
}

void Runtime::gatherMovers(const glm::vec3& pull) {
  m_movers.clear();
  m_moverReach = 0.0f;

  const JPH::PhysicsSystem& system = m_physicsWorld.system();

  JPH::BodyIDVector active;
  system.GetActiveBodies(JPH::EBodyType::RigidBody, active);

  const float step = physics::PhysicsWorld::TIME_STEP;
  const float reach = system.GetPhysicsSettings().mSpeculativeContactDistance;

  for (const JPH::BodyID& id : active) {
    const JPH::BodyLockRead lock(system.GetBodyLockInterface(), id);

    if (!lock.Succeeded()) {
      continue;
    }

    const JPH::Body& body = lock.GetBody();
    const JPH::AABox& box = body.GetWorldSpaceBounds();

    const float travel =
      (body.GetLinearVelocity().Length() + glm::length(pull) * step
       + body.GetAngularVelocity().Length() * 0.5f * box.GetSize().Length())
      * step;

    const glm::vec3 pad(travel + reach);

    m_moverReach = glm::max(m_moverReach, travel + reach);

    m_movers.push_back(
      Bounds{
        glm::vec3(box.mMin.GetX(), box.mMin.GetY(), box.mMin.GetZ()) - pad,
        glm::vec3(box.mMax.GetX(), box.mMax.GetY(), box.mMax.GetZ()) + pad
      }
    );
  }
}

bool Runtime::moverReaches(const Bounds& region) const {
  for (const Bounds& mover : m_movers) {
    if (
      overlaps(mover.minimum, mover.maximum, region.minimum, region.maximum)
    ) {
      return true;
    }
  }

  return false;
}

bool Runtime::partnersAsleep(std::span<const uint32_t> partners) const {
  const JPH::BodyLockInterface& locks =
    m_physicsWorld.system().GetBodyLockInterface();

  for (const uint32_t partner : partners) {
    const JPH::BodyLockRead lock(locks, JPH::BodyID(partner));

    if (!lock.Succeeded()) {
      return false;
    }

    if (!lock.GetBody().IsStatic() && lock.GetBody().IsActive()) {
      return false;
    }
  }

  return true;
}

Runtime::Bounds Runtime::wakeRegion(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& state,
  std::span<const uint32_t> partners
) const {
  JPH::AABox region = surfaceBox(mesh, state);

  const JPH::BodyLockInterface& locks =
    m_physicsWorld.system().GetBodyLockInterface();

  for (const uint32_t partner : partners) {
    const JPH::BodyLockRead lock(locks, JPH::BodyID(partner));

    if (!lock.Succeeded() || lock.GetBody().IsStatic()) {
      continue;
    }

    region.Encapsulate(lock.GetBody().GetWorldSpaceBounds());
  }

  return Bounds{
    glm::vec3(region.mMin.GetX(), region.mMin.GetY(), region.mMin.GetZ()),
    glm::vec3(region.mMax.GetX(), region.mMax.GetY(), region.mMax.GetZ())
  };
}

Runtime::IslandScan Runtime::scanIsland(
  const objectmodel::PreparedMesh& mesh,
  std::span<const uint32_t> partners,
  bool stopAtAwake,
  std::span<const BodyMark> marks
) {
  const JPH::PhysicsSystem& system = m_physicsWorld.system();
  const JPH::BodyLockInterface& locks = system.GetBodyLockInterface();
  const JPH::PhysicsSettings& settings = system.GetPhysicsSettings();

  const float allowance = settings.mSpeculativeContactDistance
                          + settings.mPenetrationSlop
                          + contactGaps(mesh).threshold;
  const glm::vec3 grown(allowance);

  JPH::CollideShapeSettings touching;

  touching.mMaxSeparationDistance = allowance;

  m_islandMembers.clear();
  m_islandPending.clear();

  for (const uint32_t partner : partners) {
    if (m_islandMembers.insert(partner).second) {
      m_islandPending.push_back(partner);
    }
  }

  while (!m_islandPending.empty()) {
    const uint32_t member = m_islandPending.back();
    const JPH::BodyID id(member);

    m_islandPending.pop_back();

    const auto markedAs = [member](const BodyMark& mark) {
      return mark.body == member;
    };

    if (std::any_of(marks.begin(), marks.end(), markedAs)) {
      return IslandScan::Edited;
    }

    JPH::TransformedShape shape;

    {
      const JPH::BodyLockRead lock(locks, id);

      if (!lock.Succeeded()) {
        continue;
      }

      const JPH::Body& body = lock.GetBody();

      if (stopAtAwake && !body.IsStatic() && body.IsActive()) {
        return IslandScan::Awake;
      }

      if (body.GetMotionType() != JPH::EMotionType::Dynamic) {
        continue;
      }

      const JPH::AABox& box = body.GetWorldSpaceBounds();
      const glm::vec3 low(box.mMin.GetX(), box.mMin.GetY(), box.mMin.GetZ());
      const glm::vec3 high(box.mMax.GetX(), box.mMax.GetY(), box.mMax.GetZ());

      const auto touchedBefore = [&](const BodyMark& mark) {
        return mark.before.has_value()
               && overlaps(
                 mark.before->minimum - grown,
                 mark.before->maximum + grown,
                 low,
                 high
               );
      };

      if (std::any_of(marks.begin(), marks.end(), touchedBefore)) {
        return IslandScan::Edited;
      }

      shape = body.GetTransformedShape();
    }

    TouchingBodies found;

    system.GetNarrowPhaseQuery().CollideShape(
      shape.mShape,
      shape.GetShapeScale(),
      shape.GetCenterOfMassTransform(),
      touching,
      JPH::RVec3::sZero(),
      found,
      {},
      {},
      JPH::IgnoreSingleBodyFilter(id)
    );

    for (const JPH::BodyID& hit : found.bodies) {
      if (m_islandMembers.insert(hit.GetIndexAndSequenceNumber()).second) {
        m_islandPending.push_back(hit.GetIndexAndSequenceNumber());
      }
    }
  }

  return IslandScan::Complete;
}

void Runtime::holdIsland(
  objectmodel::Entity entity,
  std::span<const uint32_t> island
) {
  for (const uint32_t member : island) {
    m_dependents[member].push_back(entity);
  }
}

void Runtime::releaseIsland(
  objectmodel::Entity entity,
  DeformableTrack& track
) {
  for (const uint32_t member : track.island) {
    const auto found = m_dependents.find(member);

    if (found == m_dependents.end()) {
      continue;
    }

    std::erase(found->second, entity);

    if (found->second.empty()) {
      m_dependents.erase(found);
    }
  }

  track.island.clear();
}

bool Runtime::wakeIsland(objectmodel::Entity entity) {
  const auto found = m_deformableSolvers.find(entity);

  if (found == m_deformableSolvers.end()) {
    return false;
  }

  const DeformableTrack& track = found->second;

  if (!track.island.empty()) {
    for (const uint32_t member : track.island) {
      if (m_physicsWorld.bodies().IsAdded(JPH::BodyID(member))) {
        m_physicsWorld.bodies().ActivateBody(JPH::BodyID(member));
      }
    }

    return true;
  }

  const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
  const objectmodel::DeformableState* state = m_world.deformableState(entity);

  if (mesh == nullptr || state == nullptr) {
    return false;
  }

  const Bounds region = wakeRegion(*mesh, *state, track.arming.partners);

  wake(region.minimum, region.maximum);

  return true;
}

void Runtime::disarmEdited(bool global) {
  if (!global && m_marks.empty()) {
    return;
  }

  const uint32_t holdoff = sleepGate().window;

  for (const objectmodel::Entity entity : m_stepping) {
    const auto found = m_deformableSolvers.find(entity);
    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
    const objectmodel::DeformableState* state = m_world.deformableState(entity);

    if (
      found == m_deformableSolvers.end()
      || found->second.mode != DeformableMode::Arming || state == nullptr
    ) {
      continue;
    }

    if (!global) {
      if (mesh == nullptr) {
        continue;
      }

      const IslandScan scan =
        scanIsland(*mesh, found->second.arming.partners, false, m_marks);

      if (scan != IslandScan::Edited) {
        continue;
      }
    }

    disarm(
      found->second,
      state->position,
      holdoff,
      m_deformableActivity.armAborts
    );

    ++m_deformableActivity.editDisarms;
  }
}

void Runtime::wakeDeformables() {
  m_physicsWorld.activations().drain(m_activated);

  const glm::vec3 pull = gravity();
  const bool global = pull != m_loadGravity;

  m_loadGravity = pull;

  disarmEdited(global);

  m_waking.clear();

  const auto enlist = [this](objectmodel::Entity entity, WakeReason reason) {
    const auto found = m_deformableSolvers.find(entity);

    if (
      found == m_deformableSolvers.end()
      || found->second.mode != DeformableMode::Quiescent
    ) {
      return;
    }

    found->second.mode = DeformableMode::Active;

    m_waking.emplace_back(entity, reason);
  };

  const auto dependents = [this, &enlist](uint32_t body, WakeReason reason) {
    const auto found = m_dependents.find(body);

    if (found == m_dependents.end()) {
      return;
    }

    for (const objectmodel::Entity entity : found->second) {
      enlist(entity, reason);
    }
  };

  if (global) {
    for (const objectmodel::Entity entity : m_quiescent) {
      enlist(entity, WakeReason::Global);
    }
  }

  for (const BodyMark& mark : m_marks) {
    dependents(mark.body, mark.reason);
  }

  m_marks.clear();

  for (const uint32_t body : m_activated) {
    dependents(body, WakeReason::Activation);
  }

  if (m_waking.size() < m_quiescent.size()) {
    gatherMovers(pull);

    for (const objectmodel::Entity entity : m_quiescent) {
      const auto found = m_deformableSolvers.find(entity);

      if (
        found != m_deformableSolvers.end()
        && found->second.mode == DeformableMode::Quiescent
        && moverReaches(found->second.region)
      ) {
        enlist(entity, WakeReason::Mover);
      }
    }
  }

  for (size_t woken = 0u; woken != m_waking.size(); ++woken) {
    const auto [entity, reason] = m_waking[woken];

    for (const uint32_t member : m_deformableSolvers.at(entity).island) {
      dependents(member, reason);
    }
  }

  for (const auto& [entity, reason] : m_waking) {
    DeformableTrack& track = m_deformableSolvers.at(entity);

    static_cast<void>(wakeIsland(entity));

    releaseIsland(entity, track);

    track.arming = DeformableArming{};

    countWake(reason);

    m_steppingStale = true;
  }
}

void Runtime::sleepDeformables() {
  const glm::vec3 pull = gravity();
  const SleepGate gate = sleepGate();

  bool gathered = false;

  for (const objectmodel::Entity entity : m_stepping) {
    const auto found = m_deformableSolvers.find(entity);

    if (
      found == m_deformableSolvers.end()
      || found->second.mode != DeformableMode::Arming
    ) {
      continue;
    }

    DeformableTrack& track = found->second;

    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
    const objectmodel::DeformableState* state = m_world.deformableState(entity);

    if (
      mesh == nullptr || state == nullptr
      || !partnersAsleep(track.arming.partners)
    ) {
      continue;
    }

    const Bounds region = wakeRegion(*mesh, *state, track.arming.partners);

    if (!gathered) {
      gatherMovers(pull);
      gathered = true;
    }

    if (moverReaches(region)) {
      continue;
    }

    const IslandScan scan = scanIsland(*mesh, track.arming.partners, true, {});

    if (scan != IslandScan::Complete) {
      continue;
    }

    std::vector<uint32_t> island(
      m_islandMembers.begin(),
      m_islandMembers.end()
    );

    std::sort(island.begin(), island.end());

    std::vector<glm::vec3> load(track.arming.load.size(), glm::vec3(0.0f));
    glm::vec3 total(0.0f);

    if (track.arming.loadSteps != 0u) {
      const float steps = static_cast<float>(track.arming.loadSteps);

      for (size_t node = 0u; node != load.size(); ++node) {
        load[node] = track.arming.load[node] / steps;
        total += load[node];
      }
    }

    const deformable::EquilibriumCheck confirmed = deformable::checkEquilibrium(
      *mesh,
      *state,
      m_world.materials(),
      m_deformableSettings,
      pull,
      load
    );

    ++m_deformableActivity.confirms;

    m_deformableActivity.confirmSteps += track.arming.loadSteps;
    m_deformableActivity.confirmLoad += total;
    m_deformableActivity.worstCorrection =
      glm::max(m_deformableActivity.worstCorrection, confirmed.maxCorrection);
    m_deformableActivity.lastConfirm = confirmed;

    if (
      confirmed.cgCapped || confirmed.unbalanced
      || !(confirmed.maxCorrection <= QUIESCENT_CORRECTION)
    ) {
      ++m_deformableActivity.refusals;

      disarm(
        track,
        state->position,
        gate.window,
        m_deformableActivity.armAborts
      );

      continue;
    }

    track.mode = DeformableMode::Quiescent;
    track.region = region;
    track.solver = deformable::DeformableSolverState{};
    track.arming = DeformableArming{};
    track.island = std::move(island);

    holdIsland(entity, track.island);

    m_steppingStale = true;

    ++m_deformableActivity.freezes;
  }
}

bool Runtime::yieldReachable(
  uint32_t body,
  const YieldTrack& track,
  const glm::vec3& pull
) const {
  const JPH::PhysicsSystem& system = m_physicsWorld.system();
  const JPH::BodyLockInterface& locks = system.GetBodyLockInterface();

  const float step = physics::PhysicsWorld::TIME_STEP;
  const float reach = system.GetPhysicsSettings().mSpeculativeContactDistance;

  const auto pointSpeed = [&](const JPH::Body& held) {
    if (!held.IsActive()) {
      return 0.0f;
    }

    return held.GetLinearVelocity().Length() + glm::length(pull) * step
           + held.GetAngularVelocity().Length() * 0.5f
               * held.GetWorldSpaceBounds().GetSize().Length();
  };

  JPH::AABox region;
  float ownSpeed = 0.0f;

  {
    const JPH::BodyLockRead lock(locks, JPH::BodyID(body));

    if (!lock.Succeeded()) {
      return false;
    }

    region = lock.GetBody().GetWorldSpaceBounds();
    ownSpeed = pointSpeed(lock.GetBody());
  }

  region.ExpandBy(
    JPH::Vec3::sReplicate(ownSpeed * step + reach + m_moverReach)
  );

  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> found;

  system.GetBroadPhaseQuery().CollideAABox(region, found);

  for (const JPH::BodyID& id : found.mHits) {
    if (id.GetIndexAndSequenceNumber() == body) {
      continue;
    }

    const JPH::BodyLockRead lock(locks, id);

    if (!lock.Succeeded() || lock.GetBody().IsSensor()) {
      continue;
    }

    if (ownSpeed + pointSpeed(lock.GetBody()) > track.trigger) {
      return true;
    }
  }

  return false;
}

objectmodel::DeformableState Runtime::rigidState(
  objectmodel::Entity entity,
  uint32_t body,
  const objectmodel::PreparedMesh& mesh,
  const YieldTrack& track
) const {
  JPH::RVec3 position = JPH::RVec3::sZero();
  JPH::Quat rotation = JPH::Quat::sIdentity();
  glm::vec3 centre(0.0f);
  glm::vec3 linear(0.0f);
  glm::vec3 angular(0.0f);

  {
    const JPH::BodyLockRead lock(
      m_physicsWorld.system().GetBodyLockInterface(),
      JPH::BodyID(body)
    );

    if (lock.Succeeded()) {
      const JPH::Body& held = lock.GetBody();

      position = held.GetPosition();
      rotation = held.GetRotation();
      centre = glm::vec3(fromJolt(held.GetCenterOfMassPosition()));
      linear = glm::vec3(fromJolt(held.GetLinearVelocity()));
      angular = glm::vec3(fromJolt(held.GetAngularVelocity()));
    }
  }

  const objectmodel::Pose placed{
    glm::vec3(position.GetX(), position.GetY(), position.GetZ()),
    glm::quat(
      rotation.GetW(),
      rotation.GetX(),
      rotation.GetY(),
      rotation.GetZ()
    )
  };

  objectmodel::DeformableState state = deformable::restState(mesh, placed);

  if (!track.shape.empty()) {
    const glm::mat4 model = objectmodel::model(placed);

    for (size_t node = 0u; node != state.position.size(); ++node) {
      state.position[node] =
        glm::vec3(model * glm::vec4(track.shape[node], 1.0f));
    }
  }

  for (size_t node = 0u; node != state.position.size(); ++node) {
    state.velocity[node] =
      linear + glm::cross(angular, state.position[node] - centre);
  }

  if (
    const objectmodel::DeformableState* kept = m_world.deformableState(entity)
  ) {
    state.plastic = kept->plastic;
  }

  return state;
}

float Runtime::yieldApproach(
  objectmodel::Entity entity,
  const glm::vec3& pull
) {
  const objectmodel::DeformableState* state = m_world.deformableState(entity);

  if (state == nullptr) {
    return 0.0f;
  }

  const float step = physics::PhysicsWorld::TIME_STEP;

  float fastest = 0.0f;

  for (const glm::vec3& moving : state->velocity) {
    fastest = glm::max(fastest, glm::length(moving));
  }

  const float reach =
    m_physicsWorld.system().GetPhysicsSettings().mSpeculativeContactDistance;

  DeformableSlot slot;

  touchBodies(
    entity,
    (fastest + glm::length(pull) * step) * step + reach + m_moverReach,
    slot
  );

  gatherContacts(entity, pull, {}, slot);

  for (const uint32_t unsupported : slot.unsupported) {
    ++m_unsupportedPartners;
    reportUnsupported(entity, JPH::BodyID(unsupported));
  }

  const JPH::BodyLockInterface& locks =
    m_physicsWorld.system().GetBodyLockInterface();

  float approach = 0.0f;

  for (const deformable::SurfaceContact& contact : slot.contacts) {
    if (
      contact.partner < 0
      || static_cast<size_t>(contact.partner) >= slot.partners.size()
    ) {
      continue;
    }

    const deformable::RigidPartner& partner =
      slot.partners[static_cast<size_t>(contact.partner)];

    const glm::vec3 at = contact.weights.x * state->position[contact.nodes.x]
                         + contact.weights.y * state->position[contact.nodes.y]
                         + contact.weights.z * state->position[contact.nodes.z];
    const glm::vec3 moving =
      contact.weights.x * state->velocity[contact.nodes.x]
      + contact.weights.y * state->velocity[contact.nodes.y]
      + contact.weights.z * state->velocity[contact.nodes.z];

    glm::vec3 other(0.0f);

    {
      const JPH::BodyLockRead lock(locks, JPH::BodyID(partner.body));

      if (lock.Succeeded()) {
        const JPH::Body& held = lock.GetBody();

        other = glm::vec3(
          fromJolt(held.GetLinearVelocity())
          + glm::cross(
            fromJolt(held.GetAngularVelocity()),
            glm::dvec3(at) - fromJolt(held.GetCenterOfMassPosition())
          )
        );
      }
    }

    approach = glm::max(approach, glm::dot(contact.normal, other - moving));
  }

  return approach;
}

void Runtime::promote(
  objectmodel::Entity entity,
  uint32_t body,
  float approach
) {
  const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
  const objectmodel::DeformableState* state = m_world.deformableState(entity);

  YieldTransition transition;

  transition.entity = entity;
  transition.promotion = true;
  transition.approach = approach;

  YieldTrack& track = m_yielding.at(entity);

  track.window = deformable::openWindow(
    *mesh,
    *state,
    meshMass(*mesh, state->position).centre
  );

  {
    const JPH::BodyLockRead lock(
      m_physicsWorld.system().GetBodyLockInterface(),
      JPH::BodyID(body)
    );

    if (lock.Succeeded()) {
      const Momentum before = bodyMomentum(lock.GetBody());

      transition.linearBefore = before.linear;
      transition.angularBefore = before.angular;
      transition.kineticBefore = before.kinetic;

      poseRigidWindow(lock.GetBody(), *mesh, track.shape, *track.window);
    }
  }

  const Momentum after =
    nodeMomentum(*mesh, track.window->position, track.window->velocity);

  transition.linearAfter = after.linear;
  transition.angularAfter = after.angular;
  transition.kineticAfter = after.kinetic;

  markBody(body, WakeReason::Edit, bodyBounds(body));

  m_physicsWorld.bodies().RemoveBody(JPH::BodyID(body));

  track.promoted = true;
  track.forced = false;

  m_world.patchDeformableState(
    entity,
    [&opened = *track.window](objectmodel::DeformableState& mirrored) {
      deformable::mirrorWindow(opened, mirrored);
    }
  );

  m_steppingStale = true;

  ++m_deformableActivity.promotions;
  m_deformableActivity.transitions.push_back(transition);
}

void Runtime::demote(
  objectmodel::Entity entity,
  uint32_t body,
  YieldTrack& track
) {
  const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);

  const deformable::WindowExit leaving = deformable::closeWindow(
    *mesh,
    m_world.materials(),
    *track.window,
    m_deformableSettings
  );

  const Momentum before =
    nodeMomentum(*mesh, leaving.position, leaving.velocity);

  std::vector<glm::vec3> placed(leaving.position.size());

  for (size_t node = 0u; node != placed.size(); ++node) {
    placed[node] = glm::vec3(leaving.position[node]);
  }

  const deformable::MeshFrame fit = deformable::meshFrame(*mesh, placed);
  const glm::dvec3 origin = fit.centre - fit.rotation * fit.restCentre;
  const glm::dmat3 back = glm::transpose(fit.rotation);

  track.shape.resize(leaving.position.size());

  float permanent = 0.0f;

  for (size_t node = 0u; node != track.shape.size(); ++node) {
    track.shape[node] = glm::vec3(back * (leaving.position[node] - origin));

    permanent = glm::max(
      permanent,
      glm::distance(track.shape[node], mesh->restPositions[node])
    );
  }

  if (
    const auto* held =
      m_world.registry().try_get<objectmodel::SharedSdf>(entity);
    held != nullptr && held->field != nullptr
  ) {
    const JPH::ShapeRefC authored = shapeFor(*held);
    const auto* authoredField =
      dynamic_cast<const physics::FieldShape*>(authored.GetPtr());

    if (authoredField != nullptr && authoredField->hull() != nullptr) {
      const bool displaced = std::any_of(
        leaving.plastic.begin(),
        leaving.plastic.end(),
        [](const objectmodel::PlasticPoint& point) {
          return point.equivalent > 0.0f;
        }
      );
      const JPH::RefConst<JPH::ConvexHullShape> moved =
        displaced ? physics::displacedHull(*held->field, *mesh, track.shape)
                  : nullptr;

      if (displaced && moved == nullptr) {
        ++m_deformableActivity.hullFallbacks;
      }

      setBodyShape(
        entity,
        yieldShape(
          moved != nullptr
            ? JPH::ShapeRefC(new physics::FieldShape(*authoredField, moved))
            : authored,
          *mesh
        )
      );
    }
  }

  const glm::dquat turned = glm::quat_cast(fit.rotation);

  JPH::BodyInterface& bodies = m_physicsWorld.bodies();
  const JPH::BodyID id(body);

  bodies.SetPositionAndRotation(
    id,
    JPH::RVec3(toJolt(origin)),
    JPH::Quat(
      static_cast<float>(turned.x),
      static_cast<float>(turned.y),
      static_cast<float>(turned.z),
      static_cast<float>(turned.w)
    )
      .Normalized(),
    JPH::EActivation::DontActivate
  );

  applyYieldMotion(body, *mesh, track);

  bodies.AddBody(id, JPH::EActivation::Activate);

  YieldTransition transition;

  transition.entity = entity;
  transition.linearBefore = before.linear;
  transition.angularBefore = before.angular;
  transition.kineticBefore = before.kinetic;
  transition.permanent = permanent;

  {
    JPH::BodyLockWrite lock(m_physicsWorld.system().GetBodyLockInterface(), id);

    if (lock.Succeeded()) {
      JPH::Body& held = lock.GetBody();

      const JPH::MotionProperties* motion = held.GetMotionProperties();

      const glm::dvec3 linear =
        before.linear * static_cast<double>(motion->GetInverseMass());
      const glm::dvec3 angular =
        fromJolt(held.GetInverseInertia()) * before.angular;

      held.SetLinearVelocityClamped(toJolt(linear));
      held.SetAngularVelocityClamped(toJolt(angular));

      Momentum after = bodyMomentum(held);

      if (after.kinetic > before.kinetic) {
        const double rescale =
          std::sqrt(before.kinetic / after.kinetic) * DEMOTE_RESCALE;

        held.SetLinearVelocityClamped(
          toJolt(rescale * fromJolt(held.GetLinearVelocity()))
        );
        held.SetAngularVelocityClamped(
          toJolt(rescale * fromJolt(held.GetAngularVelocity()))
        );

        after = bodyMomentum(held);
      }

      transition.linearAfter = after.linear;
      transition.angularAfter = after.angular;
      transition.kineticAfter = after.kinetic;
    }
  }

  markBody(body, WakeReason::Edit, std::nullopt);

  m_world.patchDeformableState(
    entity,
    [&](objectmodel::DeformableState& state) {
      const size_t nodes = leaving.position.size();

      state.position.resize(nodes);
      state.velocity.resize(nodes);
      state.acceleration.assign(nodes, glm::vec3(0.0f));

      for (size_t node = 0u; node != nodes; ++node) {
        state.position[node] = placed[node];
        state.velocity[node] = glm::vec3(leaving.velocity[node]);
      }

      state.plastic = leaving.plastic;
    }
  );

  track.promoted = false;
  track.window.reset();

  m_steppingStale = true;

  ++m_deformableActivity.demotions;
  m_deformableActivity.transitions.push_back(transition);
}

void Runtime::stepYielding() {
  if (m_yielding.empty()) {
    return;
  }

  const glm::vec3 pull = gravity();

  gatherMovers(pull);

  for (const objectmodel::Entity entity : m_world.registry()
                                            .view<
                                              objectmodel::DeformablePrepared,
                                              objectmodel::DeformableState>()) {
    const auto found = m_yielding.find(entity);

    if (found == m_yielding.end()) {
      continue;
    }

    YieldTrack& track = found->second;

    const auto* body = m_world.registry().try_get<objectmodel::BodyId>(entity);
    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);

    if (body == nullptr || mesh == nullptr) {
      continue;
    }

    if (track.promoted) {
      if (
        !m_deformableSettings.demoteAfterYield || !track.window.has_value()
        || !track.window->closing
      ) {
        continue;
      }

      if (yieldApproach(entity, pull) > track.trigger) {
        track.window->closing = false;
      } else {
        demote(entity, body->id, track);
      }

      continue;
    }

    if (!track.forced && !yieldReachable(body->id, track, pull)) {
      continue;
    }

    m_world.setDeformableState(
      entity,
      rigidState(entity, body->id, *mesh, track)
    );

    const float approach = yieldApproach(entity, pull);

    if (track.forced || approach > track.trigger) {
      promote(entity, body->id, approach);
    }
  }
}

void Runtime::gatherWindowPartners(WindowSlot& slot) {
  slot.partners.clear();

  const deformable::ImpactWindow& window = *slot.track->window;

  glm::dvec3 low(std::numeric_limits<double>::infinity());
  glm::dvec3 high(-std::numeric_limits<double>::infinity());
  double fastest = 0.0;

  for (size_t node = 0u; node != window.position.size(); ++node) {
    const glm::dvec3 world = window.origin + window.position[node];

    low = glm::min(low, world);
    high = glm::max(high, world);
    fastest = std::max(fastest, glm::length(window.velocity[node]));
  }

  const float pad =
    static_cast<float>(fastest) * physics::PhysicsWorld::TIME_STEP
    + glm::compMax(slot.mesh->voxelSize);

  slot.minimum = glm::vec3(low) - glm::vec3(pad);
  slot.maximum = glm::vec3(high) + glm::vec3(pad);

  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> found;

  m_physicsWorld.system().GetBroadPhaseQuery().CollideAABox(
    JPH::AABox(
      JPH::Vec3(slot.minimum.x, slot.minimum.y, slot.minimum.z),
      JPH::Vec3(slot.maximum.x, slot.maximum.y, slot.maximum.z)
    ),
    found
  );

  std::vector<JPH::BodyID> hits(found.mHits.begin(), found.mHits.end());

  std::sort(
    hits.begin(),
    hits.end(),
    [](const JPH::BodyID& left, const JPH::BodyID& right) {
      return left.GetIndexAndSequenceNumber()
             < right.GetIndexAndSequenceNumber();
    }
  );

  for (const JPH::BodyID& id : hits) {
    const JPH::BodyLockRead lock(
      m_physicsWorld.system().GetBodyLockInterface(),
      id
    );

    if (!lock.Succeeded()) {
      continue;
    }

    const JPH::Body& body = lock.GetBody();

    if (body.IsSensor()) {
      continue;
    }

    deformable::WindowPartner partner;

    const PartnerKind kind = windowPartnerOf(body, partner);

    if (kind == PartnerKind::Unsupported) {
      ++m_unsupportedPartners;
      reportUnsupported(slot.entity, id);

      continue;
    }

    if (kind == PartnerKind::Degenerate) {
      ++m_deformableActivity.degeneratePartners;

      continue;
    }

    slot.partners.push_back(partner);
  }
}

void Runtime::gatherPartnerSupports(deformable::WindowGroup& group) {
  const JPH::BodyLockInterface& locks =
    m_physicsWorld.system().GetBodyLockInterface();
  const float pull = glm::length(gravity());
  const size_t gathered = group.partners.size();

  for (size_t index = 0u; index != gathered; ++index) {
    if (!group.partners[index].dynamic) {
      continue;
    }

    const JPH::BodyID held(group.partners[index].body);

    JPH::AABox around;

    {
      const JPH::BodyLockRead lock(locks, held);

      if (!lock.Succeeded()) {
        continue;
      }

      const JPH::Body& body = lock.GetBody();
      const float speed = body.GetLinearVelocity().Length()
                          + pull * physics::PhysicsWorld::TIME_STEP;

      around = body.GetWorldSpaceBounds();
      around.ExpandBy(
        JPH::Vec3::sReplicate(
          speed * physics::PhysicsWorld::TIME_STEP
          + glm::compMax(group.partners[index].probe.field->voxelSize)
        )
      );
    }

    JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> found;

    m_physicsWorld.system().GetBroadPhaseQuery().CollideAABox(around, found);

    std::vector<JPH::BodyID> hits(found.mHits.begin(), found.mHits.end());

    std::sort(
      hits.begin(),
      hits.end(),
      [](const JPH::BodyID& left, const JPH::BodyID& right) {
        return left.GetIndexAndSequenceNumber()
               < right.GetIndexAndSequenceNumber();
      }
    );

    for (const JPH::BodyID& id : hits) {
      const uint32_t body = id.GetIndexAndSequenceNumber();

      if (
        std::any_of(
          group.partners.begin(),
          group.partners.end(),
          [body](const deformable::WindowPartner& partner) {
            return partner.body == body;
          }
        )
      ) {
        continue;
      }

      const JPH::BodyLockRead lock(locks, id);

      if (
        !lock.Succeeded() || lock.GetBody().IsSensor()
        || lock.GetBody().GetMotionType() != JPH::EMotionType::Static
      ) {
        continue;
      }

      deformable::WindowPartner partner;

      if (windowPartnerOf(lock.GetBody(), partner) == PartnerKind::Field) {
        group.partners.push_back(partner);
      }
    }
  }
}

void Runtime::countWindowTouches(std::span<const WindowSlot> slots) {
  const entt::registry& registry = m_world.registry();

  for (size_t first = 0u; first != slots.size(); ++first) {
    for (size_t second = first + 1u; second != slots.size(); ++second) {
      if (!overlaps(
            slots[first].minimum,
            slots[first].maximum,
            slots[second].minimum,
            slots[second].maximum
          )) {
        continue;
      }

      bool touching = false;
      double deepest = 0.0;

      for (const auto& [probing, posed] :
           {std::pair<size_t, size_t>{first, second},
            std::pair<size_t, size_t>{second, first}}) {
        const WindowSlot& seeking = slots[probing];
        const WindowSlot& target = slots[posed];

        const auto* held =
          registry.try_get<objectmodel::SharedSdf>(target.entity);
        const objectmodel::DeformableState* state =
          m_world.deformableState(target.entity);

        if (held == nullptr || held->field == nullptr || state == nullptr) {
          continue;
        }

        const deformable::MeshFrame fit =
          deformable::meshFrame(*target.mesh, state->position);
        const glm::dmat3 back = glm::transpose(fit.rotation);
        const deformable::ImpactWindow& window = *seeking.track->window;

        for (const uint32_t node : seeking.mesh->surfaceNodes) {
          const glm::dvec3 world = window.origin + window.position[node];
          const glm::dvec3 local = back * (world - fit.centre) + fit.restCentre;
          const float distance =
            talus::field::probeDistance(*held->field, glm::vec3(local));

          if (distance < 0.0f) {
            touching = true;
            deepest = std::max(deepest, -static_cast<double>(distance));
          }
        }
      }

      if (touching) {
        ++m_deformableActivity.windowTouches;
        m_deformableActivity.windowTouchDepth =
          std::max(m_deformableActivity.windowTouchDepth, deepest);
      }
    }
  }
}

void Runtime::stepWindows() {
  m_handbackPredictions.clear();

  std::vector<WindowSlot> slots;

  for (const objectmodel::Entity entity : m_world.registry()
                                            .view<
                                              objectmodel::DeformablePrepared,
                                              objectmodel::DeformableState>()) {
    const auto found = m_yielding.find(entity);

    if (
      found == m_yielding.end() || !found->second.promoted
      || !found->second.window.has_value()
    ) {
      continue;
    }

    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);

    if (mesh == nullptr) {
      continue;
    }

    WindowSlot slot;

    slot.entity = entity;
    slot.mesh = mesh;
    slot.track = &found->second;

    slots.push_back(std::move(slot));
  }

  if (slots.empty()) {
    m_partnerLoads.clear();

    return;
  }

  for (WindowSlot& slot : slots) {
    gatherWindowPartners(slot);
  }

  const uint32_t count = static_cast<uint32_t>(slots.size());

  std::vector<uint32_t> parents(count);

  std::iota(parents.begin(), parents.end(), 0u);

  const auto root = [&parents](uint32_t index) {
    while (parents[index] != index) {
      parents[index] = parents[parents[index]];
      index = parents[index];
    }

    return index;
  };

  std::unordered_map<uint32_t, uint32_t> owners;

  for (uint32_t index = 0u; index != count; ++index) {
    for (const deformable::WindowPartner& partner : slots[index].partners) {
      if (!partner.dynamic) {
        continue;
      }

      const auto [owner, fresh] = owners.try_emplace(partner.body, index);

      if (fresh) {
        continue;
      }

      const uint32_t mine = root(index);
      const uint32_t theirs = root(owner->second);

      parents[std::max(mine, theirs)] = std::min(mine, theirs);
    }
  }

  std::vector<deformable::WindowGroup> groups;
  std::vector<uint32_t> groupOf(count, 0u);
  std::vector<uint32_t> memberOf(count, 0u);
  std::unordered_map<uint32_t, uint32_t> rootGroups;

  for (uint32_t index = 0u; index != count; ++index) {
    const uint32_t top = root(index);
    const auto [placed, fresh] =
      rootGroups.try_emplace(top, static_cast<uint32_t>(groups.size()));

    if (fresh) {
      groups.emplace_back();
    }

    deformable::WindowGroup& group = groups[placed->second];
    const WindowSlot& slot = slots[index];

    deformable::WindowMember member;

    member.mesh = slot.mesh;
    member.window = &*slot.track->window;
    member.trigger = slot.track->trigger;
    member.permanent = !m_deformableSettings.demoteAfterYield;
    member.key = static_cast<uint32_t>(slot.entity);

    for (const deformable::WindowPartner& partner : slot.partners) {
      const auto kept = std::find_if(
        group.partners.begin(),
        group.partners.end(),
        [&partner](const deformable::WindowPartner& held) {
          return held.body == partner.body;
        }
      );

      if (kept != group.partners.end()) {
        member.partners.push_back(
          static_cast<uint32_t>(kept - group.partners.begin())
        );

        continue;
      }

      member.partners.push_back(static_cast<uint32_t>(group.partners.size()));
      group.partners.push_back(partner);

      const auto carried = m_partnerLoads.find(partner.body);

      if (partner.dynamic && carried != m_partnerLoads.end()) {
        group.partners.back().previousLoad = carried->second.load;
        group.partners.back().previousDrag = carried->second.drag;
        group.partners.back().supportTracks = carried->second.supports;
      }
    }

    groupOf[index] = placed->second;
    memberOf[index] = static_cast<uint32_t>(group.members.size());
    group.members.push_back(std::move(member));
  }

  for (deformable::WindowGroup& group : groups) {
    gatherPartnerSupports(group);
  }

  const size_t threads =
    m_deformableSettings.stepThreads != 0u
      ? m_deformableSettings.stepThreads
      : std::max(std::thread::hardware_concurrency(), 2u) - 1u;
  const size_t workers = std::max<size_t>(std::min(threads, groups.size()), 1u);
  const unsigned int control = _mm_getcsr();
  const glm::dvec3 pull(gravity());
  const double frameTime =
    static_cast<double>(physics::PhysicsWorld::TIME_STEP);
  const std::span<const objectmodel::PhysicalMaterial> materials =
    m_world.materials();

  std::vector<std::vector<deformable::WindowStep>> results(groups.size());
  std::vector<std::exception_ptr> failures(groups.size());
  std::atomic<size_t> next{0u};

  const auto work = [&] {
    _mm_setcsr(control);

    for (size_t group = next.fetch_add(1u); group < groups.size();
         group = next.fetch_add(1u)) {
      try {
        results[group] = deformable::stepWindows(
          groups[group],
          materials,
          pull,
          frameTime,
          m_deformableSettings
        );
      } catch (...) {
        failures[group] = std::current_exception();
      }
    }
  };

  {
    std::vector<std::jthread> helpers;
    helpers.reserve(workers - 1u);

    for (size_t worker = 1u; worker != workers; ++worker) {
      helpers.emplace_back(work);
    }

    work();
  }

  for (const std::exception_ptr& failure : failures) {
    if (failure) {
      std::rethrow_exception(failure);
    }
  }

  m_partnerLoads.clear();

  for (const deformable::WindowGroup& group : groups) {
    for (const deformable::WindowPartner& partner : group.partners) {
      if (partner.dynamic && partner.inverseMass > 0.0) {
        m_partnerLoads[partner.body] = PartnerCarry{
          partner.previousLoad,
          partner.previousDrag,
          partner.supportTracks
        };
      }
    }
  }

  JPH::BodyInterface& bodies = m_physicsWorld.bodies();
  const JPH::BodyLockInterface& locks =
    m_physicsWorld.system().GetBodyLockInterface();
  const JPH::Vec3 joltGravity = m_physicsWorld.system().GetGravity();

  std::vector<std::vector<PartnerHandback>> handedBack(groups.size());
  std::vector<std::vector<HandbackPrediction>> predictedBack(groups.size());

  for (size_t group = 0u; group != groups.size(); ++group) {
    double groupInput = 0.0;

    for (const deformable::WindowStep& result : results[group]) {
      groupInput += result.input;
    }

    for (const deformable::WindowPartner& partner : groups[group].partners) {
      if (
        !partner.dynamic || !(partner.inverseMass > 0.0) || !partner.touched
      ) {
        continue;
      }

      const JPH::BodyID id(partner.body);

      bodies.ActivateBody(id);

      glm::dvec3 landingLinear = partner.linearVelocity;
      glm::dvec3 landingAngular = partner.angularVelocity;

      {
        const JPH::BodyLockRead lock(locks, id);

        if (lock.Succeeded()) {
          const auto [linear, angular] = landingVelocity(
            lock.GetBody(),
            partner.centerOfMass,
            partner.rotation,
            pull,
            frameTime
          );

          landingLinear = linear;
          landingAngular = angular;
        }
      }

      bodies.SetLinearAndAngularVelocity(
        id,
        toJolt(landingLinear),
        toJolt(landingAngular)
      );

      HandbackPrediction predicted;

      predicted.body = partner.body;

      {
        const JPH::BodyLockRead lock(locks, id);

        if (lock.Succeeded()) {
          const auto [linear, angular] = contactFree(
            lock.GetBody(),
            joltGravity,
            physics::PhysicsWorld::TIME_STEP
          );

          predicted.linear =
            glm::vec3(linear.GetX(), linear.GetY(), linear.GetZ());
          predicted.angular =
            glm::vec3(angular.GetX(), angular.GetY(), angular.GetZ());
        }
      }

      PartnerHandback handback;

      handback.body = partner.body;
      handback.centre = partner.centerOfMass;
      handback.rotation = partner.rotation;
      handback.linear = partner.linearVelocity;
      handback.angular = partner.angularVelocity;
      handback.windowEnergy = partnerEnergy(
        1.0 / partner.inverseMass,
        glm::determinant(partner.inverseInertia) != 0.0
          ? glm::inverse(partner.inverseInertia)
          : glm::dmat3(0.0),
        partner.linearVelocity,
        partner.angularVelocity,
        partner.centerOfMass,
        pull
      );
      handback.groupInput = groupInput;

      handedBack[group].push_back(handback);
      predictedBack[group].push_back(predicted);
    }
  }

  for (uint32_t index = 0u; index != count; ++index) {
    const WindowSlot& slot = slots[index];
    const deformable::ImpactWindow& window = *slot.track->window;
    const deformable::WindowStep& result =
      results[groupOf[index]][memberOf[index]];

    m_world.patchDeformableState(
      slot.entity,
      [&window](objectmodel::DeformableState& state) {
        deformable::mirrorWindow(window, state);
      }
    );

    WindowRecord record;

    record.entity = slot.entity;
    record.frame = window.frames;
    record.group = groupOf[index];
    record.step = result;
    record.firstContact = window.firstContact;
    record.lead = window.lead;
    record.openingInside = window.openingInside;
    record.closing = window.closing;
    record.diverged = window.diverged;
    record.restored = result.restored;
    m_deformableActivity.windowRestores += result.restored ? 1u : 0u;

    if (memberOf[index] == 0u) {
      record.handbacks = handedBack[groupOf[index]];

      for (size_t handback = 0u; handback != record.handbacks.size();
           ++handback) {
        HandbackPrediction predicted = predictedBack[groupOf[index]][handback];

        predicted.record = m_deformableActivity.windowFrames.size();
        predicted.handback = handback;

        m_handbackPredictions.push_back(predicted);
      }
    }

    m_deformableActivity.windowFrames.push_back(record);
    m_deformableActivity.windowSubsteps += result.substeps;

    if (window.frames != 1u) {
      continue;
    }

    for (YieldTransition& transition : m_deformableActivity.transitions) {
      if (transition.promotion && transition.entity == slot.entity) {
        transition.penaltyAfter = window.openingPenalty;
      }
    }
  }

  m_deformableActivity.windows = count;
  m_deformableActivity.windowGroups = static_cast<uint32_t>(groups.size());

  countWindowTouches(slots);

  m_deformableReport.unsupportedPartners = m_unsupportedPartners;
}

void Runtime::finishHandbacks() {
  const glm::dvec3 pull(gravity());

  JPH::PhysicsSystem& system = m_physicsWorld.system();
  JPH::BodyInterface& bodies = m_physicsWorld.bodies();
  const physics::ImpactListener& impacts = m_physicsWorld.impacts();
  JPH::BroadPhase& broadPhase = const_cast<JPH::BroadPhase&>(
    static_cast<const JPH::BroadPhase&>(system.GetBroadPhaseQuery())
  );

  for (const HandbackPrediction& predicted : m_handbackPredictions) {
    PartnerHandback& handback =
      m_deformableActivity.windowFrames[predicted.record].handbacks
        [predicted.handback];
    JPH::BodyID id(predicted.body);

    JPH::Vec3 linearChange = JPH::Vec3::sZero();
    JPH::Vec3 angularChange = JPH::Vec3::sZero();

    {
      const JPH::BodyLockRead lock(system.GetBodyLockInterface(), id);

      if (!lock.Succeeded()) {
        continue;
      }

      linearChange =
        lock.GetBody().GetLinearVelocity()
        - JPH::Vec3(predicted.linear.x, predicted.linear.y, predicted.linear.z);
      angularChange = lock.GetBody().GetAngularVelocity()
                      - JPH::Vec3(
                        predicted.angular.x,
                        predicted.angular.y,
                        predicted.angular.z
                      );
    }

    handback.owned = !impacts.contactedMoving(id);

    if (handback.owned) {
      linearChange = JPH::Vec3::sZero();
      angularChange = JPH::Vec3::sZero();
    }

    handback.contactLinear = fromJolt(linearChange);
    handback.contactAngular = fromJolt(angularChange);

    bodies.SetLinearAndAngularVelocity(
      id,
      toJolt(handback.linear + handback.contactLinear),
      toJolt(handback.angular + handback.contactAngular)
    );

    const bool unchanged =
      linearChange == JPH::Vec3::sZero() && angularChange == JPH::Vec3::sZero();

    const JPH::BodyLockWrite lock(system.GetBodyLockInterface(), id);

    if (!lock.Succeeded()) {
      continue;
    }

    JPH::Body& held = lock.GetBody();

    if (unchanged) {
      const JPH::Quat rotation = JPH::Quat(
                                   static_cast<float>(handback.rotation.x),
                                   static_cast<float>(handback.rotation.y),
                                   static_cast<float>(handback.rotation.z),
                                   static_cast<float>(handback.rotation.w)
      )
                                   .Normalized();

      held.SetPositionAndRotationInternal(
        toJolt(handback.centre) - rotation * held.GetShape()->GetCenterOfMass(),
        rotation,
        false
      );

      if (held.IsInBroadPhase()) {
        broadPhase.NotifyBodiesAABBChanged(&id, 1);
      }
    }

    const glm::dmat3 inverse = fromJolt(held.GetInverseInertia());

    handback.forwardEnergy = partnerEnergy(
      1.0 / static_cast<double>(held.GetMotionProperties()->GetInverseMass()),
      glm::determinant(inverse) != 0.0 ? glm::inverse(inverse)
                                       : glm::dmat3(0.0),
      fromJolt(held.GetLinearVelocity()),
      fromJolt(held.GetAngularVelocity()),
      fromJolt(held.GetCenterOfMassPosition()),
      pull
    );
  }
}

void Runtime::recordYieldFlights() {
  m_flights.clear();

  const JPH::BodyLockInterface& locks =
    m_physicsWorld.system().GetBodyLockInterface();

  for (const auto& [entity, track] : m_yielding) {
    const auto* body = m_world.registry().try_get<objectmodel::BodyId>(entity);

    if (
      track.promoted || body == nullptr
      || std::any_of(
        m_handbackPredictions.begin(),
        m_handbackPredictions.end(),
        [&body](const HandbackPrediction& predicted) {
          return predicted.body == body->id;
        }
      )
    ) {
      continue;
    }

    const JPH::BodyLockRead lock(locks, JPH::BodyID(body->id));

    if (!lock.Succeeded()) {
      continue;
    }

    const JPH::Body& held = lock.GetBody();

    if (!held.IsActive() || !held.IsDynamic()) {
      continue;
    }

    const JPH::Quat rotation = held.GetRotation();
    const JPH::Vec3 spin = held.GetAngularVelocity();

    YieldFlight flight;

    flight.body = body->id;
    flight.rotation = glm::dquat(
      rotation.GetW(),
      rotation.GetX(),
      rotation.GetY(),
      rotation.GetZ()
    );
    flight.spin = glm::vec3(spin.GetX(), spin.GetY(), spin.GetZ());

    const glm::dmat3 turn = glm::mat3_cast(flight.rotation);

    flight.inverseInertia =
      turn * fromJolt(held.GetMotionProperties()->GetLocalSpaceInverseInertia())
      * glm::transpose(turn);

    m_flights.push_back(flight);
  }
}

void Runtime::advanceYieldFlights() {
  JPH::PhysicsSystem& system = m_physicsWorld.system();
  const physics::ImpactListener& impacts = m_physicsWorld.impacts();
  JPH::BroadPhase& broadPhase = const_cast<JPH::BroadPhase&>(
    static_cast<const JPH::BroadPhase&>(system.GetBroadPhaseQuery())
  );

  const auto sameBits = [](float left, float right) {
    return std::bit_cast<uint32_t>(left) == std::bit_cast<uint32_t>(right);
  };

  for (const YieldFlight& flight : m_flights) {
    JPH::BodyID id(flight.body);

    if (impacts.contacted(id)) {
      continue;
    }

    const JPH::BodyLockWrite lock(system.GetBodyLockInterface(), id);

    if (!lock.Succeeded()) {
      continue;
    }

    JPH::Body& moved = lock.GetBody();

    if (!moved.IsActive()) {
      continue;
    }

    const JPH::Vec3 spun = moved.GetAngularVelocity();

    if (
      !sameBits(spun.GetX(), flight.spin.x)
      || !sameBits(spun.GetY(), flight.spin.y)
      || !sameBits(spun.GetZ(), flight.spin.z)
    ) {
      continue;
    }

    const glm::dvec3 momentum =
      glm::inverse(flight.inverseInertia) * glm::dvec3(flight.spin);

    if (glm::dot(momentum, momentum) == 0.0) {
      continue;
    }

    const deformable::RigidMotion end = deformable::rigidAdvance(
      flight.inverseInertia,
      deformable::RigidMotion{momentum, glm::dmat3(1.0)},
      static_cast<double>(physics::PhysicsWorld::TIME_STEP),
      deformable::RIGID_PARTS
    );

    const glm::dquat turned = glm::normalize(
      glm::quat_cast(end.turn * glm::mat3_cast(flight.rotation))
    );
    const JPH::Quat rotation = JPH::Quat(
                                 static_cast<float>(turned.x),
                                 static_cast<float>(turned.y),
                                 static_cast<float>(turned.z),
                                 static_cast<float>(turned.w)
    )
                                 .Normalized();

    const JPH::RVec3 centre = moved.GetCenterOfMassPosition();
    const JPH::Vec3 offset = moved.GetShape()->GetCenterOfMass();

    moved.SetPositionAndRotationInternal(
      centre - rotation * offset,
      rotation,
      false
    );
    moved.SetAngularVelocityClamped(
      toJolt(end.turn * (flight.inverseInertia * end.momentum))
    );

    if (moved.IsInBroadPhase()) {
      broadPhase.NotifyBodiesAABBChanged(&id, 1);
    }
  }
}

void Runtime::step() {
  m_deformableActivity = DeformableActivity{};
  m_deformableActivity.destroyWakes = std::exchange(m_pendingDestroyWakes, 0u);
  m_deformableActivity.otherWakes = std::exchange(m_pendingOtherWakes, 0u);
  m_deformableActivity.wakes =
    m_deformableActivity.destroyWakes + m_deformableActivity.otherWakes;

  if (m_broadPhaseStale) {
    optimizeBroadPhase();
  }

  refreshStepping();

  wakeDeformables();

  stepYielding();

  stepDeformables();

  stepWindows();

  recordYieldFlights();

  m_physicsWorld.step();

  finishHandbacks();

  advanceYieldFlights();

  sleepDeformables();
}

void Runtime::syncPoses(float alpha) {
  JPH::BodyIDVector active;
  m_physicsWorld.system().GetActiveBodies(JPH::EBodyType::RigidBody, active);

  const auto promotedTrack = [](const auto& entry) {
    return entry.second.promoted;
  };

  if (
    active.empty()
    && std::none_of(m_yielding.begin(), m_yielding.end(), promotedTrack)
  ) {
    return;
  }

  const JPH::BodyInterface& bodies = m_physicsWorld.bodies();
  const entt::registry& registry = m_world.registry();

  m_poseScratch.clear();
  m_poseScratch.reserve(active.size());

  m_renderScratch.clear();
  m_renderScratch.reserve(active.size());

  for (const JPH::BodyID& id : active) {
    const objectmodel::Entity entity{
      static_cast<uint32_t>(bodies.GetUserData(id))
    };

    if (!registry.valid(entity)) {
      continue;
    }

    JPH::RVec3 position;
    JPH::Quat rotation;
    bodies.GetPositionAndRotation(id, position, rotation);

    const objectmodel::Pose settled{
      glm::vec3(position.GetX(), position.GetY(), position.GetZ()),
      glm::quat(
        rotation.GetW(),
        rotation.GetX(),
        rotation.GetY(),
        rotation.GetZ()
      )
    };

    const uint32_t slot = static_cast<uint32_t>(entity);

    const auto seen = m_previousPoses.find(slot);

    const objectmodel::Pose before =
      seen == m_previousPoses.end() ? settled : seen->second;

    m_poseScratch.push_back({entity, settled});

    m_renderScratch.push_back(
      {entity,
       objectmodel::RenderPose{objectmodel::Pose{
         glm::mix(before.position, settled.position, alpha),
         glm::slerp(before.rotation, settled.rotation, alpha)
       }}}
    );

    m_previousPoses[slot] = settled;
  }

  for (const auto& [entity, track] : m_yielding) {
    const objectmodel::PreparedMesh* mesh = m_world.deformablePrepared(entity);
    const objectmodel::DeformableState* state = m_world.deformableState(entity);

    if (!track.promoted || mesh == nullptr || state == nullptr) {
      continue;
    }

    const deformable::MeshFrame fit =
      deformable::meshFrame(*mesh, state->position);

    const objectmodel::Pose settled{
      glm::vec3(fit.centre - fit.rotation * fit.restCentre),
      glm::normalize(glm::quat(glm::quat_cast(fit.rotation)))
    };

    const uint32_t slot = static_cast<uint32_t>(entity);

    const auto seen = m_previousPoses.find(slot);

    const objectmodel::Pose before =
      seen == m_previousPoses.end() ? settled : seen->second;

    m_poseScratch.push_back({entity, settled});

    m_renderScratch.push_back(
      {entity,
       objectmodel::RenderPose{objectmodel::Pose{
         glm::mix(before.position, settled.position, alpha),
         glm::slerp(before.rotation, settled.rotation, alpha)
       }}}
    );

    m_previousPoses[slot] = settled;
  }

  m_world.syncMany<objectmodel::Pose>(m_poseScratch);
  m_world.syncMany<objectmodel::RenderPose>(m_renderScratch);
}

void Runtime::beginFrame() noexcept {}

void Runtime::collectImpacts() {
  m_physicsWorld.impacts().drain(m_impacts);

  objectmodel::sortStrongestFirst(m_impacts);
}

std::span<const objectmodel::Impact> Runtime::impacts() const noexcept {
  return m_impacts;
}

}
