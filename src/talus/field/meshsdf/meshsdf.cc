#include "meshsdf.h"

#include <talus/dynamics/scheduler/scheduler.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace talus::field {

namespace {

constexpr uint32_t LEAF_TRIANGLES = 4u;

constexpr uint32_t MINIMUM_ROWS_PER_BLOCK = 4u;

constexpr double RELATIVE_BOX_PADDING = 1e-9;

constexpr double SIGN_CARRY_MARGIN = 1.0 + 1e-6;

struct Triangle {
  std::array<glm::dvec3, 3> corner{};
  glm::dvec3 minimum{0.0};
  glm::dvec3 maximum{0.0};
  std::array<uint32_t, 3> vertex{};
};

struct Node {
  glm::dvec3 minimum{0.0};
  glm::dvec3 maximum{0.0};
  uint32_t first = 0u;
  uint32_t count = 0u;
};

struct Tree {
  std::vector<Node> nodes;
  std::vector<Triangle> triangles;
};

struct TreeBuild {
  std::vector<Triangle> source;
  std::vector<glm::dvec3> centroids;
  std::vector<uint32_t> order;
  double padding = 0.0;
  Tree tree;
};

struct Pending {
  double distanceSquared = 0.0;
  uint32_t node = 0u;
};

struct Nearest {
  double distance = 0.0;
  uint32_t triangle = 0u;
};

struct Scratch {
  std::vector<Pending> heap;
  std::vector<uint32_t> stack;
};

struct RayFrame {
  glm::dvec3 inverse{0.0};
  glm::dvec2 shear{0.0};
  double depthScale = 0.0;
  int across = 0;
  int up = 0;
  int along = 0;
};

struct EdgeSide {
  double value = 0.0;
  int side = 0;
};

uint64_t directedEdge(uint32_t from, uint32_t to) {
  return (static_cast<uint64_t>(from) << 32) | to;
}

void requireClosed(std::span<const uint32_t> indices) {
  std::vector<uint64_t> edges;
  edges.reserve(indices.size());

  for (size_t face = 0u; face < indices.size(); face += 3u) {
    for (size_t corner = 0u; corner != 3u; ++corner) {
      const uint32_t from = indices[face + corner];
      const uint32_t to = indices[face + (corner + 1u) % 3u];

      if (from == to) {
        throw std::invalid_argument(
          "A mesh triangle must use three different vertices"
        );
      }

      edges.push_back(directedEdge(from, to));
    }
  }

  std::sort(edges.begin(), edges.end());

  const bool repeated =
    std::adjacent_find(edges.begin(), edges.end()) != edges.end();

  const bool unpaired =
    std::any_of(edges.begin(), edges.end(), [&edges](uint64_t edge) {
      const uint64_t reverse = (edge << 32) | (edge >> 32);

      return !std::binary_search(edges.begin(), edges.end(), reverse);
    });

  if (repeated || unpaired) {
    throw std::invalid_argument(
      "A mesh must be closed, with each edge shared by two faces in opposite "
      "directions"
    );
  }
}

void validate(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  const glm::uvec3& resolution
) {
  if (vertices.empty() || indices.empty()) {
    throw std::invalid_argument("A mesh needs vertices and triangles");
  }

  if (indices.size() % 3u != 0u) {
    throw std::invalid_argument("Mesh indices must come in threes");
  }

  if (std::any_of(indices.begin(), indices.end(), [&](uint32_t index) {
        return index >= vertices.size();
      })) {
    throw std::invalid_argument("A mesh index must name one of its vertices");
  }

  if (std::any_of(vertices.begin(), vertices.end(), [](const glm::vec3& at) {
        return !std::isfinite(at.x) || !std::isfinite(at.y)
               || !std::isfinite(at.z);
      })) {
    throw std::invalid_argument("Mesh vertices must be finite");
  }

  if (resolution.x < 2u || resolution.y < 2u || resolution.z < 2u) {
    throw std::invalid_argument(
      "A sampled field needs at least two lattice points on every axis"
    );
  }

  requireClosed(indices);
}

uint32_t buildNode(TreeBuild& build, uint32_t begin, uint32_t end) {
  const uint32_t index = static_cast<uint32_t>(build.tree.nodes.size());
  build.tree.nodes.emplace_back();

  glm::dvec3 minimum(std::numeric_limits<double>::infinity());
  glm::dvec3 maximum(-std::numeric_limits<double>::infinity());
  glm::dvec3 lowest = minimum;
  glm::dvec3 highest = maximum;

  for (uint32_t i = begin; i != end; ++i) {
    const uint32_t triangle = build.order[i];

    for (const glm::dvec3& corner : build.source[triangle].corner) {
      minimum = glm::min(minimum, corner);
      maximum = glm::max(maximum, corner);
    }

    lowest = glm::min(lowest, build.centroids[triangle]);
    highest = glm::max(highest, build.centroids[triangle]);
  }

  Node node{
    minimum - glm::dvec3(build.padding),
    maximum + glm::dvec3(build.padding),
    0u,
    0u
  };

  if (end - begin <= LEAF_TRIANGLES) {
    node.first = static_cast<uint32_t>(build.tree.triangles.size());
    node.count = end - begin;

    for (uint32_t i = begin; i != end; ++i) {
      build.tree.triangles.push_back(build.source[build.order[i]]);
    }

    build.tree.nodes[index] = node;

    return index;
  }

  const glm::dvec3 extent = highest - lowest;
  const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0
                   : extent.y >= extent.z                       ? 1
                                                                : 2;
  const uint32_t middle = begin + (end - begin) / 2u;

  std::nth_element(
    build.order.begin() + begin,
    build.order.begin() + middle,
    build.order.begin() + end,
    [&build, axis](uint32_t a, uint32_t b) {
      const double first = build.centroids[a][axis];
      const double second = build.centroids[b][axis];

      return first < second || (first == second && a < b);
    }
  );

  buildNode(build, begin, middle);
  node.first = buildNode(build, middle, end);

  build.tree.nodes[index] = node;

  return index;
}

Tree buildTree(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  double padding
) {
  TreeBuild build;
  build.padding = padding;

  const uint32_t count = static_cast<uint32_t>(indices.size() / 3u);

  build.source.resize(count);
  build.centroids.resize(count);
  build.order.resize(count);

  for (uint32_t triangle = 0u; triangle != count; ++triangle) {
    Triangle& made = build.source[triangle];

    for (uint32_t corner = 0u; corner != 3u; ++corner) {
      made.vertex[corner] = indices[3u * triangle + corner];
      made.corner[corner] = glm::dvec3(vertices[made.vertex[corner]]);
    }

    made.minimum =
      glm::min(glm::min(made.corner[0], made.corner[1]), made.corner[2])
      - glm::dvec3(padding);
    made.maximum =
      glm::max(glm::max(made.corner[0], made.corner[1]), made.corner[2])
      + glm::dvec3(padding);

    build.centroids[triangle] =
      (made.corner[0] + made.corner[1] + made.corner[2]) / 3.0;
    build.order[triangle] = triangle;
  }

  build.tree.nodes.reserve(2u * static_cast<size_t>(count));
  build.tree.triangles.reserve(count);

  buildNode(build, 0u, count);

  return std::move(build.tree);
}

double boxDistanceSquared(
  const glm::dvec3& minimum,
  const glm::dvec3& maximum,
  const glm::dvec3& point
) {
  const glm::dvec3 gap =
    glm::max(glm::max(minimum - point, point - maximum), glm::dvec3(0.0));

  return glm::dot(gap, gap);
}

double boxDistanceSquared(const Node& node, const glm::dvec3& point) {
  return boxDistanceSquared(node.minimum, node.maximum, point);
}

double triangleDistanceSquared(
  const Triangle& triangle,
  const glm::dvec3& point
) {
  const glm::dvec3& a = triangle.corner[0];
  const glm::dvec3& b = triangle.corner[1];
  const glm::dvec3& c = triangle.corner[2];

  const glm::dvec3 ab = b - a;
  const glm::dvec3 ac = c - a;

  const auto away = [&point](const glm::dvec3& closest) {
    const glm::dvec3 gap = point - closest;

    return glm::dot(gap, gap);
  };

  const glm::dvec3 ap = point - a;
  const double d1 = glm::dot(ab, ap);
  const double d2 = glm::dot(ac, ap);

  if (d1 <= 0.0 && d2 <= 0.0) {
    return away(a);
  }

  const glm::dvec3 bp = point - b;
  const double d3 = glm::dot(ab, bp);
  const double d4 = glm::dot(ac, bp);

  if (d3 >= 0.0 && d4 <= d3) {
    return away(b);
  }

  const double vc = d1 * d4 - d3 * d2;

  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return away(a + ab * (d1 / (d1 - d3)));
  }

  const glm::dvec3 cp = point - c;
  const double d5 = glm::dot(ab, cp);
  const double d6 = glm::dot(ac, cp);

  if (d6 >= 0.0 && d5 <= d6) {
    return away(c);
  }

  const double vb = d5 * d2 - d1 * d6;

  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return away(a + ac * (d2 / (d2 - d6)));
  }

  const double va = d3 * d6 - d5 * d4;

  if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
    return away(b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))));
  }

  const double total = va + vb + vc;

  return away(a + ab * (vb / total) + ac * (vc / total));
}

Nearest nearestTriangle(
  const Tree& tree,
  const glm::dvec3& point,
  uint32_t seed,
  std::vector<Pending>& heap
) {
  const auto farther = [](const Pending& a, const Pending& b) {
    return a.distanceSquared > b.distanceSquared;
  };

  double best = std::numeric_limits<double>::infinity();
  uint32_t nearest = seed;

  const double seeded = triangleDistanceSquared(tree.triangles[seed], point);

  if (seeded < best) {
    best = seeded;
  }

  heap.clear();

  Pending next{boxDistanceSquared(tree.nodes[0], point), 0u};

  while (next.distanceSquared < best) {
    const Node& node = tree.nodes[next.node];

    if (node.count != 0u) {
      for (uint32_t i = node.first; i != node.first + node.count; ++i) {
        const Triangle& triangle = tree.triangles[i];

        if (!(boxDistanceSquared(triangle.minimum, triangle.maximum, point)
              < best)) {
          continue;
        }

        const double candidate = triangleDistanceSquared(triangle, point);

        if (candidate < best) {
          best = candidate;
          nearest = i;
        }
      }
    } else {
      Pending nearer{
        boxDistanceSquared(tree.nodes[next.node + 1u], point),
        next.node + 1u
      };
      Pending further{
        boxDistanceSquared(tree.nodes[node.first], point),
        node.first
      };

      if (further.distanceSquared < nearer.distanceSquared) {
        std::swap(nearer, further);
      }

      if (further.distanceSquared < best) {
        heap.push_back(further);
        std::push_heap(heap.begin(), heap.end(), farther);
      }

      if (nearer.distanceSquared < best) {
        if (heap.empty()
            || nearer.distanceSquared <= heap.front().distanceSquared) {
          next = nearer;
          continue;
        }

        heap.push_back(nearer);
        std::push_heap(heap.begin(), heap.end(), farther);
      }
    }

    if (heap.empty()) {
      break;
    }

    std::pop_heap(heap.begin(), heap.end(), farther);
    next = heap.back();
    heap.pop_back();
  }

  return {std::sqrt(best), nearest};
}

RayFrame rayFrame(const glm::dvec3& direction) {
  const glm::dvec3 size = glm::abs(direction);

  RayFrame frame;
  frame.along = size.x >= size.y && size.x >= size.z ? 0
                : size.y >= size.z                   ? 1
                                                     : 2;
  frame.across = (frame.along + 1) % 3;
  frame.up = (frame.across + 1) % 3;
  frame.inverse = 1.0 / direction;
  frame.shear = glm::dvec2(direction[frame.across], direction[frame.up])
                / direction[frame.along];
  frame.depthScale = 1.0 / direction[frame.along];

  return frame;
}

bool rayMeetsBox(
  const Node& node,
  const glm::dvec3& origin,
  const RayFrame& frame
) {
  const glm::dvec3 low = (node.minimum - origin) * frame.inverse;
  const glm::dvec3 high = (node.maximum - origin) * frame.inverse;
  const glm::dvec3 nearer = glm::min(low, high);
  const glm::dvec3 farther = glm::max(low, high);

  const double enter = std::max({nearer.x, nearer.y, nearer.z});
  const double leave = std::min({farther.x, farther.y, farther.z});

  return enter <= leave && leave >= 0.0;
}

EdgeSide edgeSide(const glm::dvec2& from, const glm::dvec2& to) {
  const double value = from.x * to.y - from.y * to.x;

  if (value != 0.0) {
    return {value, value > 0.0 ? 1 : -1};
  }

  const glm::dvec2 run = to - from;

  if (run.y != 0.0) {
    return {0.0, run.y < 0.0 ? 1 : -1};
  }

  return {0.0, run.x > 0.0 ? 1 : (run.x < 0.0 ? -1 : 0)};
}

EdgeSide orientedEdgeSide(
  const std::array<glm::dvec2, 3>& flat,
  const Triangle& triangle,
  uint32_t from,
  uint32_t to
) {
  if (triangle.vertex[from] < triangle.vertex[to]) {
    return edgeSide(flat[from], flat[to]);
  }

  const EdgeSide reverse = edgeSide(flat[to], flat[from]);

  return {-reverse.value, -reverse.side};
}

bool rayCrosses(
  const Triangle& triangle,
  const glm::dvec3& origin,
  const RayFrame& frame
) {
  std::array<glm::dvec2, 3> flat{};
  std::array<double, 3> depth{};

  for (uint32_t corner = 0u; corner != 3u; ++corner) {
    const glm::dvec3 relative = triangle.corner[corner] - origin;
    const double along = relative[frame.along];

    flat[corner] = glm::dvec2(relative[frame.across], relative[frame.up])
                   - frame.shear * along;
    depth[corner] = frame.depthScale * along;
  }

  const EdgeSide u = orientedEdgeSide(flat, triangle, 1u, 2u);
  const EdgeSide v = orientedEdgeSide(flat, triangle, 2u, 0u);
  const EdgeSide w = orientedEdgeSide(flat, triangle, 0u, 1u);

  if (u.side == 0 || u.side != v.side || v.side != w.side) {
    return false;
  }

  const double determinant = u.value + v.value + w.value;

  if (determinant == 0.0) {
    return false;
  }

  const double scaledDepth =
    u.value * depth[0] + v.value * depth[1] + w.value * depth[2];

  return determinant > 0.0 ? scaledDepth > 0.0 : scaledDepth < 0.0;
}

bool oddCrossings(
  const Tree& tree,
  const glm::dvec3& origin,
  const RayFrame& frame,
  std::vector<uint32_t>& stack
) {
  bool odd = false;

  stack.clear();
  stack.push_back(0u);

  while (!stack.empty()) {
    const uint32_t index = stack.back();
    stack.pop_back();

    const Node& node = tree.nodes[index];

    if (!rayMeetsBox(node, origin, frame)) {
      continue;
    }

    if (node.count != 0u) {
      for (uint32_t i = node.first; i != node.first + node.count; ++i) {
        if (rayCrosses(tree.triangles[i], origin, frame)) {
          odd = !odd;
        }
      }

      continue;
    }

    stack.push_back(index + 1u);
    stack.push_back(node.first);
  }

  return odd;
}

bool enclosed(
  const Tree& tree,
  const glm::dvec3& point,
  const std::array<RayFrame, 3>& frames,
  std::vector<uint32_t>& stack
) {
  const Node& root = tree.nodes[0];

  if (glm::any(glm::lessThan(point, root.minimum))
      || glm::any(glm::greaterThan(point, root.maximum))) {
    return false;
  }

  const bool first = oddCrossings(tree, point, frames[0], stack);
  const bool second = oddCrossings(tree, point, frames[1], stack);

  if (first == second) {
    return first;
  }

  return oddCrossings(tree, point, frames[2], stack);
}

double coordinateScale(
  std::span<const glm::vec3> vertices,
  const glm::vec3& minimum,
  const glm::vec3& maximum
) {
  glm::vec3 largest = glm::max(glm::abs(minimum), glm::abs(maximum));

  for (const glm::vec3& vertex : vertices) {
    largest = glm::max(largest, glm::abs(vertex));
  }

  return static_cast<double>(std::max({largest.x, largest.y, largest.z}));
}

void bakeRows(
  const Tree& tree,
  const std::array<RayFrame, 3>& frames,
  SampledSdf& field,
  uint32_t begin,
  uint32_t end,
  Scratch& scratch
) {
  uint32_t seed = 0u;

  for (uint32_t row = begin; row != end; ++row) {
    const uint32_t y = row % field.resolution.y;
    const uint32_t z = row / field.resolution.y;

    size_t index = static_cast<size_t>(row) * field.resolution.x;
    glm::dvec3 previous(0.0);
    double previousDistance = 0.0;
    bool previousInside = false;

    for (uint32_t x = 0u; x != field.resolution.x; ++x) {
      const glm::dvec3 point(
        field.origin + field.voxelSize * glm::vec3(x, y, z)
      );

      const Nearest nearest = nearestTriangle(tree, point, seed, scratch.heap);
      seed = nearest.triangle;

      const double step = SIGN_CARRY_MARGIN * glm::length(point - previous);
      const bool carried =
        x != 0u && (nearest.distance > step || previousDistance > step);
      const bool inside =
        carried ? previousInside : enclosed(tree, point, frames, scratch.stack);

      field.distances[index] =
        static_cast<float>(inside ? -nearest.distance : nearest.distance);

      previous = point;
      previousDistance = nearest.distance;
      previousInside = inside;
      ++index;
    }
  }
}
}

SampledSdf bakeMesh(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution,
  uint8_t material
) {
  return bakeMesh(
    vertices,
    indices,
    minimum,
    maximum,
    resolution,
    material,
    dynamics::Workers{}
  );
}

SampledSdf bakeMesh(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution,
  uint8_t material,
  const dynamics::Workers& workers
) {
  validate(vertices, indices, resolution);

  const Tree tree = buildTree(
    vertices,
    indices,
    RELATIVE_BOX_PADDING * coordinateScale(vertices, minimum, maximum)
  );

  const std::array<RayFrame, 3> frames{
    rayFrame(glm::normalize(glm::dvec3(1.0, 0.3713, 0.2179))),
    rayFrame(glm::normalize(glm::dvec3(-0.2531, 1.0, 0.4127))),
    rayFrame(glm::normalize(glm::dvec3(0.3319, -0.1877, 1.0)))
  };

  SampledSdf field;
  field.origin = minimum;
  field.resolution = resolution;
  field.voxelSize = voxelSize(minimum, maximum, resolution);

  const size_t count =
    static_cast<size_t>(resolution.x) * resolution.y * resolution.z;

  field.distances.resize(count);
  field.materials.assign(count, material);

  std::vector<Scratch> scratch(std::max(workers.count, 1u));

  dynamics::parallelFor(
    workers,
    resolution.y * resolution.z,
    MINIMUM_ROWS_PER_BLOCK,
    [&](uint32_t begin, uint32_t end, uint32_t worker) {
      bakeRows(tree, frames, field, begin, end, scratch[worker]);
    }
  );

  finishBricks(field);

  return field;
}
}
