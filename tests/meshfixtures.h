#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <numbers>
#include <utility>
#include <vector>

namespace meshfixture {

struct Mesh {
  std::vector<glm::vec3> vertices;
  std::vector<uint32_t> indices;
};

inline Mesh icosphere(float radius, uint32_t subdivisions) {
  const double golden = (1.0 + std::sqrt(5.0)) / 2.0;

  std::vector<glm::dvec3> unit{
    {-1.0, golden, 0.0},
    {1.0, golden, 0.0},
    {-1.0, -golden, 0.0},
    {1.0, -golden, 0.0},
    {0.0, -1.0, golden},
    {0.0, 1.0, golden},
    {0.0, -1.0, -golden},
    {0.0, 1.0, -golden},
    {golden, 0.0, -1.0},
    {golden, 0.0, 1.0},
    {-golden, 0.0, -1.0},
    {-golden, 0.0, 1.0}
  };

  for (glm::dvec3& corner : unit) {
    corner = glm::normalize(corner);
  }

  std::vector<uint32_t> faces{0,  11, 5,  0, 5, 1,  0, 1,  7, 0,  7,  10,
                              0,  10, 11, 1, 5, 9,  5, 11, 4, 11, 10, 2,
                              10, 7,  6,  7, 1, 8,  3, 9,  4, 3,  4,  2,
                              3,  2,  6,  3, 6, 8,  3, 8,  9, 4,  9,  5,
                              2,  4,  11, 6, 2, 10, 8, 6,  7, 9,  8,  1};

  for (uint32_t level = 0u; level != subdivisions; ++level) {
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> middles;
    std::vector<uint32_t> finer;

    const auto middle = [&](uint32_t a, uint32_t b) {
      const std::pair<uint32_t, uint32_t> key{std::min(a, b), std::max(a, b)};
      const auto found = middles.find(key);

      if (found != middles.end()) {
        return found->second;
      }

      unit.push_back(glm::normalize(unit[a] + unit[b]));

      const uint32_t made = static_cast<uint32_t>(unit.size() - 1u);
      middles.emplace(key, made);

      return made;
    };

    for (size_t face = 0u; face < faces.size(); face += 3u) {
      const uint32_t a = faces[face];
      const uint32_t b = faces[face + 1u];
      const uint32_t c = faces[face + 2u];
      const uint32_t ab = middle(a, b);
      const uint32_t bc = middle(b, c);
      const uint32_t ca = middle(c, a);

      finer.insert(finer.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
    }

    faces = std::move(finer);
  }

  Mesh mesh;

  for (const glm::dvec3& corner : unit) {
    mesh.vertices.push_back(glm::vec3(corner * static_cast<double>(radius)));
  }

  mesh.indices = std::move(faces);

  return mesh;
}

inline Mesh prism(
  const std::vector<glm::vec2>& outline,
  uint32_t fan,
  float low,
  float high
) {
  const uint32_t count = static_cast<uint32_t>(outline.size());

  Mesh mesh;

  for (const glm::vec2& corner : outline) {
    mesh.vertices.emplace_back(corner, low);
  }

  for (const glm::vec2& corner : outline) {
    mesh.vertices.emplace_back(corner, high);
  }

  for (uint32_t step = 1u; step + 1u < count; ++step) {
    const uint32_t b = (fan + step) % count;
    const uint32_t c = (fan + step + 1u) % count;

    mesh.indices.insert(
      mesh.indices.end(),
      {count + fan, count + b, count + c}
    );
    mesh.indices.insert(mesh.indices.end(), {fan, c, b});
  }

  for (uint32_t i = 0u; i != count; ++i) {
    const uint32_t next = (i + 1u) % count;

    mesh.indices.insert(
      mesh.indices.end(),
      {i, next, count + next, i, count + next, count + i}
    );
  }

  return mesh;
}

inline Mesh torus(double major, double minor, uint32_t around, uint32_t tube) {
  const double turn = 2.0 * std::numbers::pi;

  Mesh mesh;

  for (uint32_t i = 0u; i != around; ++i) {
    for (uint32_t j = 0u; j != tube; ++j) {
      const double u = turn * i / around;
      const double v = turn * j / tube;
      const double reach = major + minor * std::cos(v);

      mesh.vertices.push_back(
        glm::vec3(
          glm::dvec3(
            reach * std::cos(u),
            reach * std::sin(u),
            minor * std::sin(v)
          )
        )
      );
    }
  }

  for (uint32_t i = 0u; i != around; ++i) {
    for (uint32_t j = 0u; j != tube; ++j) {
      const uint32_t next = (i + 1u) % around;
      const uint32_t up = (j + 1u) % tube;
      const uint32_t a = i * tube + j;
      const uint32_t b = next * tube + j;
      const uint32_t c = next * tube + up;
      const uint32_t d = i * tube + up;

      mesh.indices.insert(mesh.indices.end(), {a, b, c, a, c, d});
    }
  }

  return mesh;
}

inline Mesh box(const glm::vec3& low, const glm::vec3& high) {
  return prism(
    {{low.x, low.y}, {high.x, low.y}, {high.x, high.y}, {low.x, high.y}},
    0u,
    low.z,
    high.z
  );
}

}
