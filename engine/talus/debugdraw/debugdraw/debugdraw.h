#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talus::debugdraw {

struct LineVertex {
  glm::vec3 position{0.0f};
  glm::vec4 colour{1.0f};
};

static_assert(
  offsetof(LineVertex, colour) == 16,
  "LineVertex must keep the stride the line pipeline binds"
);
static_assert(
  sizeof(LineVertex) == 32,
  "LineVertex must keep the stride the line pipeline binds"
);

inline constexpr size_t MAX_LINES = 65536u;

inline constexpr uint32_t SPHERE_SEGMENTS = 24u;

class DebugDraw {
public:
  bool line(
    const glm::vec3& from,
    const glm::vec3& to,
    const glm::vec4& colour,
    float seconds,
    std::string_view category,
    bool depthTested = true
  );

  bool ray(
    const glm::vec3& origin,
    const glm::vec3& along,
    const glm::vec4& colour,
    float seconds,
    std::string_view category,
    bool depthTested = true
  );

  bool point(
    const glm::vec3& at,
    float size,
    const glm::vec4& colour,
    float seconds,
    std::string_view category,
    bool depthTested = true
  );

  bool sphere(
    const glm::vec3& centre,
    float radius,
    const glm::vec4& colour,
    float seconds,
    std::string_view category,
    bool depthTested = true
  );

  bool box(
    const glm::vec3& minimum,
    const glm::vec3& maximum,
    const glm::vec4& colour,
    float seconds,
    std::string_view category,
    bool depthTested = true
  );

  void show(std::string_view category, bool shown);

  [[nodiscard]] bool shown(std::string_view category) const noexcept;

  void advance(float seconds);

  [[nodiscard]] std::span<const LineVertex> tested();

  [[nodiscard]] std::span<const LineVertex> untested();

  [[nodiscard]] size_t count() const noexcept;

  [[nodiscard]] size_t dropped() const noexcept;

private:
  struct Category {
    std::string name;
    bool shown = true;
  };

  struct Line {
    glm::vec3 from;
    glm::vec3 to;
    glm::vec4 colour;
    float remaining;
    uint32_t category;
    bool depthTested;
  };

  [[nodiscard]] uint32_t categoryOf(std::string_view name);

  [[nodiscard]] bool room(size_t lines) noexcept;

  void push(
    const glm::vec3& from,
    const glm::vec3& to,
    const glm::vec4& colour,
    float seconds,
    uint32_t category,
    bool depthTested
  );

  void rebuild();

  std::vector<Category> m_categories;
  std::vector<Line> m_lines;
  std::vector<LineVertex> m_tested;
  std::vector<LineVertex> m_untested;
  size_t m_dropped = 0u;
  bool m_dirty = true;
};

}
