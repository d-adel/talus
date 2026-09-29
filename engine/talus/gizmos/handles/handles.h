#pragma once

#include <talus/debugdraw/debugdraw/debugdraw.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <span>

namespace talus::gizmos {

inline constexpr uint32_t NO_AXIS = 3u;

class Handles {
public:
  void place(const glm::vec3& centre);

  void hide() noexcept;

  [[nodiscard]] bool shown() const noexcept;

  [[nodiscard]] const glm::vec3& centre() const noexcept;

  void highlight(uint32_t axis) noexcept;

  void update(const glm::vec3& cameraPosition);

  [[nodiscard]] float reach() const noexcept;

  [[nodiscard]] glm::vec3 direction(uint32_t axis) const noexcept;

  [[nodiscard]] uint32_t under(
    const glm::vec3& origin,
    const glm::vec3& towards
  ) const;

  [[nodiscard]] std::span<const talus::debugdraw::LineVertex>
  lines() const noexcept;

private:
  void rebuild() noexcept;

  std::array<talus::debugdraw::LineVertex, 6> m_lines{};

  glm::vec3 m_centre{0.0f};

  float m_scale = 1.0f;

  uint32_t m_highlight = NO_AXIS;

  bool m_shown = false;
};

}
