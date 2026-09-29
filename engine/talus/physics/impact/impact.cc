#include "impact.ih"

namespace {

constexpr JPH::EShapeSubType FIELD_SHAPE_SUB_TYPE = JPH::EShapeSubType::User1;

uint32_t strongestPoint(const JPH::CollisionEstimationResult& estimate) {
  float strongest = 0.0f;
  uint32_t at = 0u;

  for (uint32_t i = 0; i != estimate.mContactImpulse.size(); ++i) {
    if (estimate.mContactImpulse[i] > strongest) {
      strongest = estimate.mContactImpulse[i];
      at = i;
    }
  }

  return at;
}

glm::vec3 fieldPoint(
  const talus::physics::FieldShape& sampled,
  JPH::RMat44Arg toLocal,
  JPH::RVec3Arg worldPoint
) {
  const JPH::RVec3 local = toLocal * worldPoint;

  return glm::vec3(
           float(local.GetX()),
           float(local.GetY()),
           float(local.GetZ())
         )
         + sampled.centerOfMass();
}

bool readsOneMaterial(
  const JPH::Body& body,
  JPH::RVec3Arg baseOffset,
  const JPH::ContactPoints& points
) {
  const JPH::Shape* shape = body.GetShape();

  if (shape->GetSubType() != FIELD_SHAPE_SUB_TYPE) {
    return true;
  }

  const auto& sampled = static_cast<const talus::physics::FieldShape&>(*shape);

  if (!sampled.uniformMaterial()) {
    return false;
  }

  const talus::field::SampledSdf& field = sampled.field();

  const glm::vec3 low = field.origin;
  const glm::vec3 high =
    field.origin
    + field.voxelSize * glm::vec3(field.resolution - glm::uvec3(1u));

  const JPH::RMat44 toLocal = body.GetInverseCenterOfMassTransform();

  for (const JPH::Vec3& relative : points) {
    const glm::vec3 at = fieldPoint(sampled, toLocal, baseOffset + relative);

    if (
      !glm::all(glm::greaterThanEqual(at, low))
      || !glm::all(glm::lessThanEqual(at, high))
    ) {
      return false;
    }
  }

  return true;
}

}

namespace talus::physics {

ImpactListener::ImpactListener(float threshold, uint32_t bodies)
    : m_threshold(threshold), m_touched(bodies), m_touchedMoving(bodies) {}

void ImpactListener::beginStep() noexcept {
  ++m_step;
}

bool ImpactListener::contacted(JPH::BodyID body) const noexcept {
  const uint32_t index = body.GetIndex();

  return index < m_touched.size()
         && m_touched[index].load(std::memory_order_relaxed) == m_step;
}

bool ImpactListener::contactedMoving(JPH::BodyID body) const noexcept {
  const uint32_t index = body.GetIndex();

  return index < m_touchedMoving.size()
         && m_touchedMoving[index].load(std::memory_order_relaxed) == m_step;
}

void ImpactListener::touch(
  const JPH::Body& body1,
  const JPH::Body& body2
) noexcept {
  for (const auto [body, other] :
       {std::pair{&body1, &body2}, std::pair{&body2, &body1}}) {
    const uint32_t index = body->GetID().GetIndex();

    if (index >= m_touched.size()) {
      continue;
    }

    m_touched[index].store(m_step, std::memory_order_relaxed);

    if (!other->IsStatic()) {
      m_touchedMoving[index].store(m_step, std::memory_order_relaxed);
    }
  }
}

uint32_t ImpactListener::materialAt(
  const JPH::Body& body,
  JPH::RVec3Arg worldPoint
) {
  const JPH::Shape* shape = body.GetShape();

  if (shape->GetSubType() != FIELD_SHAPE_SUB_TYPE) {
    return static_cast<uint32_t>(shape->GetUserData());
  }

  const auto& sampled = static_cast<const FieldShape&>(*shape);

  return talus::field::material(
    sampled.field(),
    fieldPoint(sampled, body.GetInverseCenterOfMassTransform(), worldPoint)
  );
}

const talus::objectmodel::PhysicalMaterial& ImpactListener::material(
  uint32_t index
) const noexcept {
  static const talus::objectmodel::PhysicalMaterial DEFAULT{};

  return index < m_materials.size() ? m_materials[index] : DEFAULT;
}

ImpactListener::Contact ImpactListener::combine(
  const JPH::Body& body1,
  const JPH::Body& body2,
  const JPH::ContactManifold& manifold,
  JPH::ContactSettings& settings,
  bool strongest
) const {
  Contact contact;

  if (strongest) {
    JPH::CollisionEstimationResult estimate;

    JPH::EstimateCollisionResponse(
      body1,
      body2,
      manifold,
      estimate,
      settings.mCombinedFriction,
      settings.mCombinedRestitution
    );

    contact.point1 = std::min(
      strongestPoint(estimate),
      uint32_t(manifold.mRelativeContactPointsOn1.size() - 1u)
    );
    contact.point2 = std::min(
      contact.point1,
      uint32_t(manifold.mRelativeContactPointsOn2.size() - 1u)
    );
  }

  contact.material1 =
    materialAt(body1, manifold.GetWorldSpaceContactPointOn1(contact.point1));
  contact.material2 =
    materialAt(body2, manifold.GetWorldSpaceContactPointOn2(contact.point2));

  const talus::objectmodel::PhysicalMaterial& first =
    material(contact.material1);
  const talus::objectmodel::PhysicalMaterial& second =
    material(contact.material2);

  settings.mCombinedFriction = std::sqrt(first.friction * second.friction);
  settings.mCombinedRestitution =
    std::max(first.restitution, second.restitution);

  return contact;
}

float ImpactListener::estimateImpulse(
  const JPH::Body& body1,
  const JPH::Body& body2,
  const JPH::ContactManifold& manifold,
  const JPH::ContactSettings& settings
) {
  JPH::CollisionEstimationResult estimate;

  JPH::EstimateCollisionResponse(
    body1,
    body2,
    manifold,
    estimate,
    settings.mCombinedFriction,
    settings.mCombinedRestitution
  );

  float impulse = 0.0f;

  for (uint32_t i = 0; i != estimate.mContactImpulse.size(); ++i) {
    impulse += estimate.mContactImpulse[i];
  }

  return impulse;
}

void ImpactListener::OnContactAdded(
  const JPH::Body& body1,
  const JPH::Body& body2,
  const JPH::ContactManifold& manifold,
  JPH::ContactSettings& settings
) {
  touch(body1, body2);

  if (manifold.mRelativeContactPointsOn1.empty()) {
    return;
  }

  const Contact contact = combine(body1, body2, manifold, settings, true);

  const float impulse = estimateImpulse(body1, body2, manifold, settings);

  const JPH::RVec3 on1 = manifold.GetWorldSpaceContactPointOn1(contact.point1);
  const JPH::RVec3 on2 = manifold.GetWorldSpaceContactPointOn2(contact.point2);

  const JPH::Vec3 closing =
    body1.GetPointVelocity(on1) - body2.GetPointVelocity(on2);

  const float speed = closing.Dot(manifold.mWorldSpaceNormal);

  if (speed < m_threshold.load(std::memory_order_relaxed)) {
    return;
  }

  const glm::vec3 normal(
    manifold.mWorldSpaceNormal.GetX(),
    manifold.mWorldSpaceNormal.GetY(),
    manifold.mWorldSpaceNormal.GetZ()
  );

  const talus::objectmodel::Entity one{
    static_cast<uint32_t>(body1.GetUserData())
  };
  const talus::objectmodel::Entity two{
    static_cast<uint32_t>(body2.GetUserData())
  };

  const talus::objectmodel::Impact first{
    one,
    two,
    glm::vec3(float(on1.GetX()), float(on1.GetY()), float(on1.GetZ())),
    normal,
    impulse,
    speed,
    contact.material1,
    contact.material2
  };

  const talus::objectmodel::Impact second{
    two,
    one,
    glm::vec3(float(on2.GetX()), float(on2.GetY()), float(on2.GetZ())),
    -normal,
    impulse,
    speed,
    contact.material2,
    contact.material1
  };

  const std::lock_guard<std::mutex> guard(m_mutex);

  m_impacts.push_back(first);
  m_impacts.push_back(second);
}

void ImpactListener::OnContactPersisted(
  const JPH::Body& body1,
  const JPH::Body& body2,
  const JPH::ContactManifold& manifold,
  JPH::ContactSettings& settings
) {
  touch(body1, body2);

  if (manifold.mRelativeContactPointsOn1.empty()) {
    return;
  }

  combine(
    body1,
    body2,
    manifold,
    settings,
    !(readsOneMaterial(
        body1,
        manifold.mBaseOffset,
        manifold.mRelativeContactPointsOn1
      )
      && readsOneMaterial(
        body2,
        manifold.mBaseOffset,
        manifold.mRelativeContactPointsOn2
      ))
  );
}

void ImpactListener::drain(std::vector<talus::objectmodel::Impact>& out) {
  const std::lock_guard<std::mutex> guard(m_mutex);

  out.clear();
  out.swap(m_impacts);
}

void ImpactListener::setThreshold(float speed) noexcept {
  m_threshold.store(speed, std::memory_order_relaxed);
}

float ImpactListener::threshold() const noexcept {
  return m_threshold.load(std::memory_order_relaxed);
}

void ImpactListener::setMaterials(
  std::span<const talus::objectmodel::PhysicalMaterial> materials
) noexcept {
  m_materials = materials;
}

}
