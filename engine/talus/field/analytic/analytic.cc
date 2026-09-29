#include "analytic.ih"

namespace talus::field {

namespace {

constexpr float FAR_DISTANCE = 1e9f;

float smoothMin(float a, float b, float k) {
  const float h = std::clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);

  return (b * (1.0f - h) + a * h) - k * h * (1.0f - h);
}

float boxDistance(const glm::vec3& point, const glm::vec3& halfExtents) {
  const glm::vec3 q = glm::abs(point) - halfExtents;

  const float outside = glm::length(glm::max(q, glm::vec3(0.0f)));
  const float inside = std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);

  return outside + inside;
}

float cylinderDistance(const glm::vec3& point, float radius, float halfHeight) {
  const glm::vec2 d(
    glm::length(glm::vec2(point.x, point.z)) - radius,
    std::abs(point.y) - halfHeight
  );

  return std::min(std::max(d.x, d.y), 0.0f)
         + glm::length(glm::max(d, glm::vec2(0.0f)));
}

float primitiveDistance(const Primitive& primitive, const glm::vec3& point) {
  const glm::vec3 local =
    glm::vec3(primitive.inverseModel * glm::vec4(point, 1.0f));

  switch (shapeKindOf(primitive.shapeConfig.x)) {
    case ShapeKind::Sphere:
      return glm::length(local) - primitive.shape.x;
    case ShapeKind::Box:
      return boxDistance(local, glm::vec3(primitive.shape));
    case ShapeKind::Plane:
      return local.y;
    case ShapeKind::Cylinder:
      return cylinderDistance(local, primitive.shape.x, primitive.shape.y);
  }

  return FAR_DISTANCE;
}

bool skippable(
  const Primitive& primitive,
  const glm::vec3& point,
  float accumulated
) {
  if (primitive.bounds.w <= 0.0f) {
    return false;
  }

  const float bound =
    glm::length(point - glm::vec3(primitive.bounds)) - primitive.bounds.w;

  switch (operationOf(primitive.shapeConfig.z)) {
    case CsgOperation::Union:
    case CsgOperation::SmoothUnion:
      return bound > accumulated;

    case CsgOperation::Subtraction:
    case CsgOperation::SmoothSubtraction:
      return bound >= -accumulated;

    case CsgOperation::Intersection:
      return false;
  }

  return false;
}

bool addsMaterial(uint32_t operation) {
  return talus::field::addsMaterial(operationOf(operation));
}

glm::vec3 halfExtent(const Primitive& primitive, bool& bounded) {
  glm::vec3 extent(0.0f);
  bounded = true;

  switch (primitive.shapeConfig.x) {
    case 0u:
      extent = glm::vec3(primitive.shape.x);
      break;

    case 1u: {
      const glm::mat3 model = glm::mat3(glm::inverse(primitive.inverseModel));

      glm::mat3 absolute;
      absolute[0] = glm::abs(model[0]);
      absolute[1] = glm::abs(model[1]);
      absolute[2] = glm::abs(model[2]);

      extent = absolute * glm::vec3(primitive.shape);
      break;
    }

    case 3u: {
      const glm::mat3 model = glm::mat3(glm::inverse(primitive.inverseModel));

      glm::mat3 absolute;
      absolute[0] = glm::abs(model[0]);
      absolute[1] = glm::abs(model[1]);
      absolute[2] = glm::abs(model[2]);

      extent =
        absolute
        * glm::vec3(primitive.shape.x, primitive.shape.y, primitive.shape.x);
      break;
    }

    default:
      bounded = false;
      break;
  }

  if (
    bounded && (primitive.shapeConfig.z == 1u || primitive.shapeConfig.z == 4u)
  ) {
    extent += glm::vec3(primitive.shape.w);
  }

  return extent;
}

}

std::optional<Aabb> primitiveBox(const Primitive& primitive) {
  bool bounded = true;

  const glm::vec3 extent = halfExtent(primitive, bounded);

  if (!bounded) {
    return std::nullopt;
  }

  const glm::vec3 centre = glm::vec3(glm::inverse(primitive.inverseModel)[3]);

  return Aabb{centre - extent, centre + extent};
}

void updateBounds(Primitive& primitive) {
  const glm::mat4 model = glm::inverse(primitive.inverseModel);
  const glm::vec3 centre = glm::vec3(model[3]);

  float radius = 0.0f;

  switch (shapeKindOf(primitive.shapeConfig.x)) {
    case ShapeKind::Sphere:
      radius = primitive.shape.x;
      break;

    case ShapeKind::Box:
      radius = glm::length(glm::vec3(primitive.shape));
      break;

    case ShapeKind::Cylinder:
      radius = glm::length(glm::vec2(primitive.shape.x, primitive.shape.y));
      break;

    case ShapeKind::Plane:
      radius = 0.0f;
      break;
  }

  if (
    radius > 0.0f
    && (primitive.shapeConfig.z == 1u || primitive.shapeConfig.z == 4u)
  ) {
    radius += primitive.shape.w;
  }

  primitive.bounds = glm::vec4(centre, radius);
}

std::optional<Aabb> boundedExtent(std::span<const Primitive> primitives) {
  bool any = false;
  Aabb extent{glm::vec3(0.0f), glm::vec3(0.0f)};

  for (const Primitive& primitive : primitives) {
    if (primitive.bounds.w <= 0.0f || !addsMaterial(primitive.shapeConfig.z)) {
      continue;
    }

    const glm::vec3 centre = glm::vec3(primitive.bounds);
    bool bounded = true;
    const glm::vec3 reach = halfExtent(primitive, bounded);

    if (!any) {
      extent = {centre - reach, centre + reach};
      any = true;
      continue;
    }

    extent.minimum = glm::min(extent.minimum, centre - reach);
    extent.maximum = glm::max(extent.maximum, centre + reach);
  }

  if (!any) {
    return std::nullopt;
  }

  return extent;
}

AnalyticSample combine(
  const AnalyticSample& accumulated_,
  const Primitive& primitive,
  const glm::vec3& point
) {
  AnalyticSample accumulated = accumulated_;

  {
    const AnalyticSample current{
      primitiveDistance(primitive, point),
      primitive.shapeConfig.y
    };

    switch (primitive.shapeConfig.z) {
      case 1u:
        accumulated = {
          smoothMin(accumulated.distance, current.distance, primitive.shape.w),
          accumulated.distance < current.distance ? accumulated.material
                                                  : current.material
        };
        break;

      case 2u:
        accumulated =
          accumulated.distance > current.distance ? accumulated : current;
        break;

      case 3u:
        accumulated = {
          std::max(accumulated.distance, -current.distance),
          accumulated.material
        };
        break;

      case 4u:
        accumulated = {
          -smoothMin(
            -accumulated.distance,
            current.distance,
            primitive.shape.w
          ),
          accumulated.material
        };
        break;

      default:
        accumulated =
          accumulated.distance < current.distance ? accumulated : current;
        break;
    }
  }

  return accumulated;
}

AnalyticSample sample(
  std::span<const Primitive> primitives,
  const glm::vec3& point
) {
  AnalyticSample accumulated{FAR_DISTANCE, 0};

  for (const Primitive& primitive : primitives) {
    if (skippable(primitive, point, accumulated.distance)) {
      continue;
    }

    accumulated = combine(accumulated, primitive, point);
  }

  return accumulated;
}

glm::vec3 gradient(
  std::span<const Primitive> primitives,
  const glm::vec3& point,
  float epsilon
) {
  const glm::vec3 offsetX(epsilon, 0.0f, 0.0f);
  const glm::vec3 offsetY(0.0f, epsilon, 0.0f);
  const glm::vec3 offsetZ(0.0f, 0.0f, epsilon);

  const float scale = 1.0f / (2.0f * epsilon);

  return glm::vec3(
    (sample(primitives, point + offsetX).distance
     - sample(primitives, point - offsetX).distance)
      * scale,
    (sample(primitives, point + offsetY).distance
     - sample(primitives, point - offsetY).distance)
      * scale,
    (sample(primitives, point + offsetZ).distance
     - sample(primitives, point - offsetZ).distance)
      * scale
  );
}

glm::vec3 normal(
  std::span<const Primitive> primitives,
  const glm::vec3& point,
  float epsilon
) {
  return glm::normalize(gradient(primitives, point, epsilon));
}

float distance(AnalyticFieldView field, const glm::vec3& point) {
  return sample(field.primitives, point).distance;
}

uint32_t material(AnalyticFieldView field, const glm::vec3& point) {
  return sample(field.primitives, point).material;
}

glm::vec3 gradient(
  AnalyticFieldView field,
  const glm::vec3& point,
  float epsilon
) {
  return gradient(field.primitives, point, epsilon);
}

float stepBound(
  AnalyticFieldView field,
  const glm::vec3& point,
  const glm::vec3&
) {
  return std::abs(distance(field, point));
}

}
