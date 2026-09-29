#include "debugdraw.ih"

namespace talus::debugdraw {

uint32_t DebugDraw::categoryOf(std::string_view name) {
  for (uint32_t index = 0u; index < m_categories.size(); ++index) {
    if (m_categories[index].name == name) {
      return index;
    }
  }

  m_categories.push_back({std::string(name), true});

  return static_cast<uint32_t>(m_categories.size() - 1u);
}

bool DebugDraw::room(size_t lines) noexcept {
  if (m_lines.size() + lines > MAX_LINES) {
    ++m_dropped;

    return false;
  }

  return true;
}

void DebugDraw::push(
  const glm::vec3& from,
  const glm::vec3& to,
  const glm::vec4& colour,
  float seconds,
  uint32_t category,
  bool depthTested
) {
  m_lines.push_back({from, to, colour, seconds, category, depthTested});
  m_dirty = true;
}

bool DebugDraw::line(
  const glm::vec3& from,
  const glm::vec3& to,
  const glm::vec4& colour,
  float seconds,
  std::string_view category,
  bool depthTested
) {
  if (!room(1u)) {
    return false;
  }

  push(from, to, colour, seconds, categoryOf(category), depthTested);

  return true;
}

bool DebugDraw::ray(
  const glm::vec3& origin,
  const glm::vec3& along,
  const glm::vec4& colour,
  float seconds,
  std::string_view category,
  bool depthTested
) {
  return line(origin, origin + along, colour, seconds, category, depthTested);
}

bool DebugDraw::point(
  const glm::vec3& at,
  float size,
  const glm::vec4& colour,
  float seconds,
  std::string_view category,
  bool depthTested
) {
  if (!room(3u)) {
    return false;
  }

  const uint32_t slot = categoryOf(category);
  const float half = size * 0.5f;

  for (int axis = 0; axis != 3; ++axis) {
    glm::vec3 offset(0.0f);
    offset[axis] = half;

    push(at - offset, at + offset, colour, seconds, slot, depthTested);
  }

  return true;
}

bool DebugDraw::sphere(
  const glm::vec3& centre,
  float radius,
  const glm::vec4& colour,
  float seconds,
  std::string_view category,
  bool depthTested
) {
  if (!room(3u * SPHERE_SEGMENTS)) {
    return false;
  }

  const uint32_t slot = categoryOf(category);

  for (int plane = 0; plane != 3; ++plane) {
    const int u = (plane + 1) % 3;
    const int v = (plane + 2) % 3;

    glm::vec3 previous = centre;
    previous[u] += radius;

    for (uint32_t step = 1u; step <= SPHERE_SEGMENTS; ++step) {
      const float angle = glm::two_pi<float>() * static_cast<float>(step)
                          / static_cast<float>(SPHERE_SEGMENTS);

      glm::vec3 next = centre;
      next[u] += radius * std::cos(angle);
      next[v] += radius * std::sin(angle);

      push(previous, next, colour, seconds, slot, depthTested);

      previous = next;
    }
  }

  return true;
}

bool DebugDraw::box(
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::vec4& colour,
  float seconds,
  std::string_view category,
  bool depthTested
) {
  if (!room(12u)) {
    return false;
  }

  const uint32_t slot = categoryOf(category);

  const auto corner = [&](uint32_t bits) {
    return glm::vec3(
      (bits & 1u) != 0u ? maximum.x : minimum.x,
      (bits & 2u) != 0u ? maximum.y : minimum.y,
      (bits & 4u) != 0u ? maximum.z : minimum.z
    );
  };

  for (uint32_t bits = 0u; bits != 8u; ++bits) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      const uint32_t flag = 1u << axis;

      if ((bits & flag) == 0u) {
        push(
          corner(bits),
          corner(bits | flag),
          colour,
          seconds,
          slot,
          depthTested
        );
      }
    }
  }

  return true;
}

void DebugDraw::show(std::string_view category, bool shown) {
  m_categories[categoryOf(category)].shown = shown;
  m_dirty = true;
}

bool DebugDraw::shown(std::string_view category) const noexcept {
  for (const Category& held : m_categories) {
    if (held.name == category) {
      return held.shown;
    }
  }

  return true;
}

void DebugDraw::advance(float seconds) {
  for (Line& held : m_lines) {
    held.remaining -= seconds;
  }

  std::erase_if(m_lines, [](const Line& held) {
    return held.remaining < 0.0f;
  });

  m_dropped = 0u;
  m_dirty = true;
}

void DebugDraw::rebuild() {
  if (!m_dirty) {
    return;
  }

  m_tested.clear();
  m_untested.clear();

  for (const Line& held : m_lines) {
    if (!m_categories[held.category].shown) {
      continue;
    }

    std::vector<LineVertex>& out = held.depthTested ? m_tested : m_untested;

    out.push_back({held.from, held.colour});
    out.push_back({held.to, held.colour});
  }

  m_dirty = false;
}

std::span<const LineVertex> DebugDraw::tested() {
  rebuild();

  return m_tested;
}

std::span<const LineVertex> DebugDraw::untested() {
  rebuild();

  return m_untested;
}

size_t DebugDraw::count() const noexcept {
  return m_lines.size();
}

size_t DebugDraw::dropped() const noexcept {
  return m_dropped;
}

}
