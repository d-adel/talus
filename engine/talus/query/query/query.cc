#include "query.ih"

namespace talus::query {

namespace {

bool finite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y)
         && std::isfinite(value.z);
}

bool precedes(float distance, objectmodel::Entity entity, const Hit& other) {
  if (distance != other.distance) {
    return distance < other.distance;
  }
  return static_cast<uint32_t>(entity) < static_cast<uint32_t>(other.entity);
}

objectmodel::Pose poseOf(const Context& context, objectmodel::Entity entity) {
  const auto& registry = context.world.registry();
  const auto* body = registry.try_get<objectmodel::BodyId>(entity);
  if (context.physics != nullptr && body != nullptr) {
    JPH::BodyLockRead lock(
      context.physics->system().GetBodyLockInterface(),
      JPH::BodyID(body->id)
    );
    if (
      !lock.Succeeded()
      || lock.GetBody().GetUserData() != static_cast<uint32_t>(entity)
    ) {
      throw std::logic_error(
        "query: a rigid body record does not match the physics world"
      );
    }
    const auto position = lock.GetBody().GetPosition();
    const auto rotation = lock.GetBody().GetRotation();
    return {
      glm::vec3(position.GetX(), position.GetY(), position.GetZ()),
      glm::quat(
        rotation.GetW(),
        rotation.GetX(),
        rotation.GetY(),
        rotation.GetZ()
      )
    };
  }
  return registry.get<objectmodel::Pose>(entity);
}

template<typename Field>
std::optional<Hit> march(
  const Field& source,
  const field::Aabb& bounds,
  const field::Ray& ray,
  float maxDistance
) {
  const auto interval = field::intersect(bounds, ray);
  if (!interval) {
    return std::nullopt;
  }
  float travelled = interval->first;
  const float end = std::min(interval->second, maxDistance);
  for (int iteration = 0;
       iteration < field::MARCH_MAX_ITERATIONS && travelled <= end;
       ++iteration) {
    const glm::vec3 point = glm::clamp(
      ray.origin + ray.direction * travelled,
      bounds.minimum,
      bounds.maximum
    );
    const float radius = std::abs(field::distance(source, point));
    if (radius <= field::MARCH_EPSILON) {
      const glm::vec3 derivative = field::gradient(source, point);
      const float length = glm::length(derivative);
      return Hit{
        objectmodel::INVALID_ENTITY,
        point,
        length > field::RAY_EPSILON ? derivative / length : -ray.direction,
        travelled,
        field::material(source, point)
      };
    }
    const float step = field::stepBound(source, point, ray.direction);
    if (!std::isfinite(step) || step <= 0.0f || travelled + step == travelled) {
      break;
    }
    if (travelled == end) {
      break;
    }
    travelled = std::min(travelled + step, end);
  }
  return std::nullopt;
}

}

std::optional<Hit> raycast(
  const Context& context,
  const field::Ray& ray,
  float maxDistance
) {
  const float length = glm::length(ray.direction);
  if (
    !finite(ray.origin) || !finite(ray.direction) || !std::isfinite(length)
    || length <= field::RAY_EPSILON || std::isnan(maxDistance)
    || maxDistance < 0.0f
  ) {
    throw std::invalid_argument(
      "raycast: expected a finite ray, nonzero direction and nonnegative "
      "distance"
    );
  }
  const field::Ray normalized{ray.origin, ray.direction / length};
  std::optional<Hit> nearest;
  for (const auto entity : context.world.fields()) {
    const auto pose = poseOf(context, entity);
    const glm::mat4 toWorld = objectmodel::model(pose);
    const glm::mat4 toLocal = glm::inverse(toWorld);
    const field::Ray local{
      glm::vec3(toLocal * glm::vec4(normalized.origin, 1.0f)),
      glm::vec3(toLocal * glm::vec4(normalized.direction, 0.0f))
    };
    std::optional<Hit> hit;
    if (const auto* sampled = context.world.currentSampledSdf(entity)) {
      const field::Aabb bounds{
        sampled->origin,
        sampled->origin
          + sampled->voxelSize * glm::vec3(sampled->resolution - glm::uvec3(1u))
      };
      hit = march(*sampled, bounds, local, maxDistance);
    } else {
      const auto primitives = context.world.primitives(entity);
      if (primitives.empty()) {
        continue;
      }
      const auto& grid =
        context.world.registry().get<objectmodel::SdfGrid>(entity);
      const auto bounds = objectmodel::gridBox(
        primitives,
        objectmodel::gridMargin(grid, primitives)
      );
      hit =
        march(field::AnalyticFieldView{primitives}, bounds, local, maxDistance);
    }
    if (!hit || (nearest && !precedes(hit->distance, entity, *nearest))) {
      continue;
    }
    hit->entity = entity;
    hit->position = normalized.origin + normalized.direction * hit->distance;
    hit->normal = glm::normalize(glm::mat3(toWorld) * hit->normal);
    nearest = hit;
  }
  return nearest;
}

namespace {

struct BodyShape {
  JPH::BodyID id;
  JPH::ShapeRefC shape;
  JPH::RMat44 centre;
};

BodyShape bodyShape(const Context& context, objectmodel::Entity entity) {
  if (context.physics == nullptr || !context.world.registry().valid(entity)) {
    throw std::invalid_argument(
      "query: a body query needs a physics world and a live entity"
    );
  }
  const auto* body =
    context.world.registry().try_get<objectmodel::BodyId>(entity);
  if (body == nullptr) {
    throw std::invalid_argument("query: the entity has no physics body");
  }
  const JPH::BodyID id(body->id);
  JPH::BodyLockRead lock(context.physics->system().GetBodyLockInterface(), id);
  if (
    !lock.Succeeded()
    || lock.GetBody().GetUserData() != static_cast<uint32_t>(entity)
  ) {
    throw std::invalid_argument(
      "query: body does not belong to the supplied world"
    );
  }
  const auto& held = lock.GetBody();
  return {id, held.GetShape(), held.GetCenterOfMassTransform()};
}

class OtherBodies final : public JPH::BodyFilter {
public:
  OtherBodies(const objectmodel::World& world, JPH::BodyID ignored)
      : m_world(world), m_ignored(ignored) {}

  bool ShouldCollide(const JPH::BodyID& id) const override {
    return id != m_ignored;
  }

  bool ShouldCollideLocked(const JPH::Body& body) const override {
    const auto entity = static_cast<objectmodel::Entity>(
      static_cast<uint32_t>(body.GetUserData())
    );
    if (!m_world.registry().valid(entity)) {
      return false;
    }
    const auto* record =
      m_world.registry().try_get<objectmodel::BodyId>(entity);
    return record != nullptr
           && record->id == body.GetID().GetIndexAndSequenceNumber();
  }

private:
  const objectmodel::World& m_world;
  JPH::BodyID m_ignored;
};

objectmodel::Entity entityOf(const Context& context, JPH::BodyID id) {
  JPH::BodyLockRead lock(context.physics->system().GetBodyLockInterface(), id);
  if (!lock.Succeeded()) {
    throw std::runtime_error("query: body disappeared during a query");
  }
  return static_cast<objectmodel::Entity>(
    static_cast<uint32_t>(lock.GetBody().GetUserData())
  );
}

glm::vec3 fromJolt(JPH::Vec3Arg value) {
  return {value.GetX(), value.GetY(), value.GetZ()};
}

}

std::optional<Hit> sweep(
  const Context& context,
  objectmodel::Entity entity,
  const glm::vec3& displacement
) {
  const auto body = bodyShape(context, entity);
  const float length = glm::length(displacement);
  if (
    !finite(displacement) || !std::isfinite(length)
    || length <= field::RAY_EPSILON
  ) {
    throw std::invalid_argument(
      "sweep: expected a finite nonzero displacement"
    );
  }
  const JPH::RShapeCast cast(
    body.shape,
    JPH::Vec3::sReplicate(1.0f),
    body.centre,
    JPH::Vec3(displacement.x, displacement.y, displacement.z)
  );
  JPH::AllHitCollisionCollector<JPH::CastShapeCollector> collector;
  const OtherBodies filter(context.world, body.id);
  context.physics->system().GetNarrowPhaseQuery().CastShape(
    cast,
    {},
    JPH::RVec3::sZero(),
    collector,
    {},
    {},
    filter
  );
  std::optional<Hit> nearest;
  for (const auto& contact : collector.mHits) {
    const auto other = entityOf(context, contact.mBodyID2);
    const float distance = std::max(0.0f, contact.mFraction) * length;
    if (nearest && !precedes(distance, other, *nearest)) {
      continue;
    }
    const glm::vec3 point = fromJolt(contact.mContactPointOn2);
    const glm::vec3 axis = fromJolt(contact.mPenetrationAxis);
    const float axisLength = glm::length(axis);
    const auto* sampled = context.world.sampledSdf(other);
    const glm::vec3 local = glm::vec3(
      glm::inverse(objectmodel::model(poseOf(context, other)))
      * glm::vec4(point, 1.0f)
    );
    nearest = Hit{
      other,
      point,
      axisLength > field::RAY_EPSILON ? -axis / axisLength
                                      : -displacement / length,
      distance,
      sampled != nullptr ? field::material(*sampled, local) : 0u
    };
  }
  return nearest;
}

std::vector<objectmodel::Entity> overlap(
  const Context& context,
  objectmodel::Entity entity
) {
  const auto body = bodyShape(context, entity);
  JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
  const OtherBodies filter(context.world, body.id);
  context.physics->system().GetNarrowPhaseQuery().CollideShape(
    body.shape,
    JPH::Vec3::sReplicate(1.0f),
    body.centre,
    {},
    JPH::RVec3::sZero(),
    collector,
    {},
    {},
    filter
  );
  std::vector<objectmodel::Entity> hits;
  for (const auto& contact : collector.mHits) {
    hits.push_back(entityOf(context, contact.mBodyID2));
  }
  std::sort(hits.begin(), hits.end());
  hits.erase(std::unique(hits.begin(), hits.end()), hits.end());
  return hits;
}

}
