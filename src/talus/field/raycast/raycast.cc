#include "raycast.ih"

namespace talus::field {

Ray screenPointToRay(
  const glm::mat4& inverseViewProj,
  const glm::vec3& cameraPosition,
  const glm::vec2& ndc
) {
  glm::vec4 world = inverseViewProj * glm::vec4(ndc.x, ndc.y, 1.0f, 1.0f);
  world /= world.w;

  return {cameraPosition, glm::normalize(glm::vec3(world) - cameraPosition)};
}

std::optional<float> alongAxis(
  const glm::vec3& anchor,
  const glm::vec3& along,
  const Ray& ray
) {
  const glm::vec3 towards = glm::normalize(ray.direction);

  const glm::vec3 between = anchor - ray.origin;

  const float alongDotRay = glm::dot(along, towards);
  const float determinant = 1.0f - alongDotRay * alongDotRay;

  if (determinant < 1e-6f) {
    return std::nullopt;
  }

  return (alongDotRay * glm::dot(between, towards) - glm::dot(between, along))
         / determinant;
}

std::optional<RayHit> raymarch(
  std::span<const Primitive> primitives,
  const Ray& ray,
  const MarchSettings& settings
) {
  float omega = settings.omega;
  float travelled = 0.0f;
  float previousRadius = 0.0f;
  float stepLength = 0.0f;
  float functionSign = 1.0f;

  for (int i = 0; i < settings.maxIterations; ++i) {
    const glm::vec3 point = ray.origin + ray.direction * travelled;
    const AnalyticSample current = sample(primitives, point);

    if (i == 0) {
      functionSign = current.distance < 0.0f ? -1.0f : 1.0f;
    }

    const float signedRadius = functionSign * current.distance;
    const float radius = std::abs(signedRadius);

    const bool relaxationFailed =
      omega > 1.0f && (radius + previousRadius) < stepLength;

    if (relaxationFailed) {
      stepLength -= omega * stepLength;
      omega = 1.0f;
    } else {
      stepLength = signedRadius * omega;
    }

    previousRadius = radius;

    if (!relaxationFailed && radius <= settings.epsilon) {
      return RayHit{point, travelled, current.material};
    }

    travelled += stepLength;

    if (travelled > settings.maxDistance) {
      break;
    }
  }

  return std::nullopt;
}

std::optional<std::pair<float, float>> intersect(
  const Aabb& box,
  const Ray& ray
) {
  float tEnter = -std::numeric_limits<float>::infinity();
  float tExit = std::numeric_limits<float>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    const float origin = ray.origin[axis];
    const float direction = ray.direction[axis];
    const float minimum = box.minimum[axis];
    const float maximum = box.maximum[axis];

    if (std::abs(direction) < RAY_EPSILON) {
      if (origin < minimum || origin > maximum) {
        return std::nullopt;
      }

      continue;
    }

    float t0 = (minimum - origin) / direction;
    float t1 = (maximum - origin) / direction;

    if (t0 > t1) {
      std::swap(t0, t1);
    }

    tEnter = std::max(tEnter, t0);
    tExit = std::min(tExit, t1);

    if (tEnter > tExit) {
      return std::nullopt;
    }
  }

  if (tExit < 0.0f) {
    return std::nullopt;
  }

  return std::pair{std::max(tEnter, 0.0f), tExit};
}
}
