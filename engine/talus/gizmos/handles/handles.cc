#include "handles.ih"

namespace talus::gizmos {

namespace {

constexpr float SCREEN_SIZE = 0.12f;

constexpr std::array<glm::vec4, 3> AXIS_COLOURS{
  glm::vec4(0.90f, 0.25f, 0.25f, 1.0f),
  glm::vec4(0.35f, 0.85f, 0.35f, 1.0f),
  glm::vec4(0.30f, 0.50f, 0.95f, 1.0f)
};

constexpr glm::vec4 HIGHLIGHT_COLOUR(1.0f, 0.85f, 0.30f, 1.0f);

}

void Handles::rebuild() noexcept {
  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    const glm::vec4 colour =
      axis == m_highlight ? HIGHLIGHT_COLOUR : AXIS_COLOURS[axis];

    m_lines[axis * 2u] = {m_centre, colour};
    m_lines[axis * 2u + 1u] = {m_centre + direction(axis) * m_scale, colour};
  }
}

void Handles::place(const glm::vec3& centre) {
  m_centre = centre;
  m_shown = true;

  rebuild();
}

void Handles::hide() noexcept {
  m_shown = false;
  m_highlight = NO_AXIS;

  rebuild();
}

bool Handles::shown() const noexcept {
  return m_shown;
}

const glm::vec3& Handles::centre() const noexcept {
  return m_centre;
}

void Handles::highlight(uint32_t axis) noexcept {
  m_highlight = axis;

  rebuild();
}

void Handles::update(const glm::vec3& cameraPosition) {
  m_scale = glm::length(cameraPosition - m_centre) * SCREEN_SIZE;

  if (m_scale < 1e-4f) {
    m_scale = 1e-4f;
  }

  rebuild();
}

float Handles::reach() const noexcept {
  return m_scale;
}

glm::vec3 Handles::direction(uint32_t axis) const noexcept {
  glm::vec3 along(0.0f);

  if (axis < 3u) {
    along[static_cast<int>(axis)] = 1.0f;
  }

  return along;
}

uint32_t Handles::under(
  const glm::vec3& origin,
  const glm::vec3& towards
) const {
  if (!m_shown) {
    return NO_AXIS;
  }

  const float tolerance = m_scale * 0.30f;

  uint32_t closest = NO_AXIS;
  float nearest = tolerance;

  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    const glm::vec3 along = direction(axis);

    const std::optional<float> found =
      talus::field::alongAxis(m_centre, along, {origin, towards});

    if (!found.has_value()) {
      continue;
    }

    const float onAxis = *found;

    if (onAxis < 0.0f || onAxis > m_scale) {
      continue;
    }

    const glm::vec3 between = m_centre - origin;

    const float onRay =
      glm::dot(between, towards) + onAxis * glm::dot(along, towards);

    if (onRay < 0.0f) {
      continue;
    }

    const glm::vec3 gap =
      (m_centre + along * onAxis) - (origin + towards * onRay);

    const float apart = glm::length(gap);

    if (apart < nearest) {
      nearest = apart;
      closest = axis;
    }
  }

  return closest;
}

std::span<const talus::debugdraw::LineVertex> Handles::lines() const noexcept {
  return m_lines;
}

}
