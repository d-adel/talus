#include "compound.h"

#include <talus/dynamics/jacobi/jacobi.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace talus::dynamics {

namespace {

constexpr uint32_t leafSize = 4u;

glm::dvec3 middleOf(const Bounds& box) {
  return 0.5 * (box.low + box.high);
}

double largestHalf(const CompoundChild& child) {
  return std::max({child.half.x, child.half.y, child.half.z});
}

double cornerReach(const CompoundChild& child, const glm::dmat3& turn) {
  double farthest = 0.0;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 sign(
      (corner & 1u) != 0u ? 1.0 : -1.0,
      (corner & 2u) != 0u ? 1.0 : -1.0,
      (corner & 4u) != 0u ? 1.0 : -1.0
    );

    farthest = std::max(farthest, glm::length(child.centre + turn * (sign * child.half)));
  }

  return farthest;
}

void collectOverlapping(
  const CompoundShape& shape,
  const Bounds& probe,
  std::vector<uint32_t>& found
) {
  found.clear();

  if (shape.nodes.empty()) {
    return;
  }

  std::array<uint32_t, 64> stack{};
  uint32_t top = 0u;

  stack[top++] = 0u;

  while (top != 0u) {
    const ChildNode& node = shape.nodes[stack[--top]];

    if (!overlaps(node.bounds, probe)) {
      continue;
    }

    if (node.left == 0u) {
      for (uint32_t at = node.first; at != node.first + node.count; ++at) {
        const uint32_t child = shape.leaves[at];

        if (overlaps(shape.childBounds[child], probe)) {
          found.push_back(child);
        }
      }
    } else {
      stack[top++] = node.right;
      stack[top++] = node.left;
    }
  }
}

uint8_t coveredBy(const CompoundChild& child, const CompoundChild& neighbour) {
  const glm::dquat unturned(1.0, 0.0, 0.0, 0.0);
  const bool plain = child.rotation == unturned && neighbour.rotation == unturned;
  const glm::dvec3 offset = neighbour.centre - child.centre;
  uint8_t covered = 0u;
  Bounds box{offset - neighbour.half, offset + neighbour.half};

  if (!plain) {
    const glm::dmat3 back = glm::transpose(glm::mat3_cast(child.rotation));
    const glm::dmat3 relative = back * glm::mat3_cast(neighbour.rotation);
    bool aligned = true;

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        const double entry = std::abs(relative[column][row]);

        aligned = aligned && (entry <= 1.0e-9 || entry >= 1.0 - 1.0e-9);
      }
    }

    if (!aligned) {
      return covered;
    }

    box = boxBoundsIn(back * offset, relative, neighbour.half);
  }

  const double tolerance = 1.0e-9 * std::max(largestHalf(child), largestHalf(neighbour));

  for (int axis = 0; axis != 3; ++axis) {
    const int first = (axis + 1) % 3;
    const int second = (axis + 2) % 3;
    const bool spans = box.low[first] <= -child.half[first] + tolerance
                       && box.high[first] >= child.half[first] - tolerance
                       && box.low[second] <= -child.half[second] + tolerance
                       && box.high[second] >= child.half[second] - tolerance;

    if (!spans) {
      continue;
    }

    if (std::abs(box.low[axis] - child.half[axis]) <= tolerance) {
      covered = static_cast<uint8_t>(covered | (1u << (2 * axis + 1)));
    }

    if (std::abs(box.high[axis] + child.half[axis]) <= tolerance) {
      covered = static_cast<uint8_t>(covered | (1u << (2 * axis)));
    }
  }

  return covered;
}

void countCovers(CompoundShape& shape, uint32_t index, std::vector<uint32_t>& nearby) {
  const CompoundChild& child = shape.children[index];
  uint8_t exposed = allFaces;

  collectOverlapping(shape, grown(shape.childBounds[index], 1.0e-9 * largestHalf(child)), nearby);

  for (const uint32_t other : nearby) {
    if (other == index) {
      continue;
    }

    const uint8_t covered = coveredBy(child, shape.children[other]);

    for (uint32_t face = 0u; face != 6u; ++face) {
      if ((covered & (1u << face)) == 0u) {
        continue;
      }

      uint16_t& count = shape.faceCovers[6u * index + face];

      if (count == std::numeric_limits<uint16_t>::max()) {
        throw std::runtime_error("a compound child face can have at most 65535 covering children");
      }

      count = static_cast<uint16_t>(count + 1u);
    }

    exposed = static_cast<uint8_t>(exposed & ~covered);
  }

  shape.exposed[index] = exposed;
}

uint32_t buildNode(
  CompoundShape& shape,
  std::vector<uint32_t>& order,
  uint32_t begin,
  uint32_t end
) {
  const uint32_t index = static_cast<uint32_t>(shape.nodes.size());
  Bounds box = shape.childBounds[order[begin]];
  Bounds middles{middleOf(box), middleOf(box)};

  for (uint32_t at = begin + 1u; at != end; ++at) {
    const Bounds& child = shape.childBounds[order[at]];
    const glm::dvec3 middle = middleOf(child);

    box = unite(box, child);
    middles = unite(middles, Bounds{middle, middle});
  }

  shape.nodes.push_back(ChildNode{box, 0u, 0u, begin, 0u});

  if (end - begin <= leafSize) {
    shape.nodes[index].count = end - begin;

    return index;
  }

  const glm::dvec3 spread = middles.high - middles.low;
  const int axis = spread.x >= spread.y && spread.x >= spread.z ? 0 : spread.y >= spread.z ? 1 : 2;

  std::sort(order.begin() + begin, order.begin() + end, [&](uint32_t first, uint32_t second) {
    const double a = middleOf(shape.childBounds[first])[axis];
    const double b = middleOf(shape.childBounds[second])[axis];

    return a < b || (a == b && first < second);
  });

  const uint32_t split = begin + (end - begin) / 2u;
  const uint32_t left = buildNode(shape, order, begin, split);
  const uint32_t right = buildNode(shape, order, split, end);

  shape.nodes[index].left = left;
  shape.nodes[index].right = right;

  return index;
}

}

Bounds boxBoundsIn(const glm::dvec3& centre, const glm::dmat3& turn, const glm::dvec3& half) {
  const glm::dvec3 reach(
    std::abs(turn[0][0]) * half.x + std::abs(turn[1][0]) * half.y
      + std::abs(turn[2][0]) * half.z,
    std::abs(turn[0][1]) * half.x + std::abs(turn[1][1]) * half.y
      + std::abs(turn[2][1]) * half.z,
    std::abs(turn[0][2]) * half.x + std::abs(turn[1][2]) * half.y
      + std::abs(turn[2][2]) * half.z
  );

  return {centre - reach, centre + reach};
}

std::shared_ptr<const CompoundShape> makeCompoundShape(std::vector<CompoundChild> children, uint64_t revision) {
  if (children.empty()) {
    throw std::runtime_error("a compound body needs at least one child");
  }

  std::sort(children.begin(), children.end(), [](const CompoundChild& a, const CompoundChild& b) {
    return a.key < b.key;
  });

  for (size_t index = 0u; index != children.size(); ++index) {
    const CompoundChild& child = children[index];

    if (index != 0u && children[index - 1u].key == child.key) {
      throw std::runtime_error("compound child keys must be unique");
    }

    if (!(child.half.x > 0.0 && child.half.y > 0.0 && child.half.z > 0.0)) {
      throw std::runtime_error("compound child half extents must be positive");
    }

    if (!(child.density >= 0.0)) {
      throw std::runtime_error("compound child density must not be negative");
    }
  }

  auto shape = std::make_shared<CompoundShape>();

  shape->revision = revision;
  shape->children = std::move(children);
  shape->childBounds.reserve(shape->children.size());
  shape->childReach.reserve(shape->children.size());
  shape->bounds = Bounds{
    glm::dvec3(std::numeric_limits<double>::infinity()),
    glm::dvec3(-std::numeric_limits<double>::infinity())
  };
  shape->thinnest = std::numeric_limits<double>::infinity();

  for (const CompoundChild& child : shape->children) {
    const glm::dmat3 turn = glm::mat3_cast(child.rotation);
    const Bounds box = boxBoundsIn(child.centre, turn, child.half);

    const double farthest = cornerReach(child, turn);

    shape->childBounds.push_back(box);
    shape->bounds = unite(shape->bounds, box);
    shape->thinnest = std::min({shape->thinnest, child.half.x, child.half.y, child.half.z});
    shape->childReach.push_back(farthest);
    shape->reach = std::max(shape->reach, farthest);
  }

  std::vector<uint32_t> order(shape->children.size());

  for (uint32_t index = 0u; index != order.size(); ++index) {
    order[index] = index;
  }

  shape->nodes.reserve(2u * order.size() / leafSize + 1u);
  (void)buildNode(*shape, order, 0u, static_cast<uint32_t>(order.size()));
  shape->leaves = std::move(order);

  std::vector<uint32_t> nearby;

  shape->exposed.assign(shape->children.size(), allFaces);
  shape->faceCovers.assign(6u * shape->children.size(), 0u);

  for (uint32_t index = 0u; index != shape->children.size(); ++index) {
    countCovers(*shape, index, nearby);
  }

  return shape;
}

void removeChildren(
  std::shared_ptr<const CompoundShape>& handle,
  std::span<const uint32_t> keys
) {
  constexpr uint32_t gone = 0xFFFFFFFFu;
  const uint32_t count = static_cast<uint32_t>(handle->children.size());
  std::vector<uint32_t> removed;

  removed.reserve(keys.size());

  for (const uint32_t key : keys) {
    const std::optional<uint32_t> index = findChild(*handle, key);

    if (!index) {
      throw std::runtime_error("a removed compound child must belong to the shape");
    }

    removed.push_back(*index);
  }

  std::sort(removed.begin(), removed.end());

  if (std::adjacent_find(removed.begin(), removed.end()) != removed.end()) {
    throw std::runtime_error("a compound child can be removed only once");
  }

  if (removed.size() >= count) {
    throw std::runtime_error("a compound body keeps at least one child");
  }

  if (removed.empty()) {
    return;
  }

  if (handle.use_count() != 1) {
    handle = std::make_shared<CompoundShape>(*handle);
  }

  CompoundShape& shape = *std::const_pointer_cast<CompoundShape>(handle);
  const uint32_t kept = count - static_cast<uint32_t>(removed.size());
  std::vector<uint32_t> renamed(count, gone);
  uint32_t begin = 0u;

  for (size_t at = 0u; at <= removed.size(); ++at) {
    const uint32_t end = at == removed.size() ? count : removed[at];

    for (uint32_t index = begin; index != end; ++index) {
      renamed[index] = index - static_cast<uint32_t>(at);
    }

    begin = end + 1u;
  }

  const glm::dvec3 extent = shape.bounds.high - shape.bounds.low;
  const double reachOut = 1.0e-6 * std::max({extent.x, extent.y, extent.z});
  std::vector<uint32_t> found;

  for (const uint32_t index : removed) {
    collectOverlapping(shape, grown(shape.childBounds[index], reachOut), found);

    for (const uint32_t other : found) {
      if (renamed[other] == gone) {
        continue;
      }

      const CompoundChild& child = shape.children[other];

      if (!overlaps(shape.childBounds[index], grown(shape.childBounds[other], 1.0e-9 * largestHalf(child)))) {
        continue;
      }

      const uint8_t covered = coveredBy(child, shape.children[index]);

      for (uint32_t face = 0u; face != 6u; ++face) {
        if ((covered & (1u << face)) == 0u) {
          continue;
        }

        uint16_t& covers = shape.faceCovers[6u * other + face];

        covers = static_cast<uint16_t>(covers - 1u);

        if (covers == 0u) {
          shape.exposed[other] = static_cast<uint8_t>(shape.exposed[other] | (1u << face));
        }
      }
    }
  }

  bool reachMoves = false;
  bool thinnestMoves = false;

  for (const uint32_t index : removed) {
    const glm::dvec3& half = shape.children[index].half;

    reachMoves = reachMoves || shape.childReach[index] == shape.reach;
    thinnestMoves = thinnestMoves || std::min(std::min(half.x, half.y), half.z) == shape.thinnest;
  }

  uint32_t write = removed.front();

  for (size_t at = 0u; at != removed.size(); ++at) {
    const uint32_t from = removed[at] + 1u;
    const uint32_t to = at + 1u == removed.size() ? count : removed[at + 1u];

    std::copy(shape.children.begin() + from, shape.children.begin() + to, shape.children.begin() + write);
    std::copy(shape.childBounds.begin() + from, shape.childBounds.begin() + to, shape.childBounds.begin() + write);
    std::copy(shape.exposed.begin() + from, shape.exposed.begin() + to, shape.exposed.begin() + write);
    std::copy(shape.childReach.begin() + from, shape.childReach.begin() + to, shape.childReach.begin() + write);
    std::copy(
      shape.faceCovers.begin() + 6u * static_cast<size_t>(from),
      shape.faceCovers.begin() + 6u * static_cast<size_t>(to),
      shape.faceCovers.begin() + 6u * static_cast<size_t>(write)
    );
    write += to - from;
  }

  shape.children.resize(kept);
  shape.childBounds.resize(kept);
  shape.exposed.resize(kept);
  shape.childReach.resize(kept);
  shape.faceCovers.resize(6u * static_cast<size_t>(kept));

  const Bounds empty{
    glm::dvec3(std::numeric_limits<double>::infinity()),
    glm::dvec3(-std::numeric_limits<double>::infinity())
  };
  uint32_t written = 0u;

  for (ChildNode& node : shape.nodes) {
    if (node.left != 0u) {
      continue;
    }

    const uint32_t first = written;
    Bounds box = empty;

    for (uint32_t at = node.first; at != node.first + node.count; ++at) {
      const uint32_t child = renamed[shape.leaves[at]];

      if (child != gone) {
        shape.leaves[written++] = child;
        box = unite(box, shape.childBounds[child]);
      }
    }

    node.first = first;
    node.count = written - first;
    node.bounds = box;
  }

  shape.leaves.resize(written);

  for (size_t index = shape.nodes.size(); index-- != 0u;) {
    ChildNode& node = shape.nodes[index];

    if (node.left != 0u) {
      node.bounds = unite(shape.nodes[node.left].bounds, shape.nodes[node.right].bounds);
    }
  }

  shape.bounds = shape.nodes.front().bounds;

  if (reachMoves) {
    shape.reach = 0.0;

    for (const double farthest : shape.childReach) {
      shape.reach = std::max(shape.reach, farthest);
    }
  }

  if (thinnestMoves) {
    shape.thinnest = std::numeric_limits<double>::infinity();

    for (const CompoundChild& child : shape.children) {
      shape.thinnest = std::min(shape.thinnest, std::min(std::min(child.half.x, child.half.y), child.half.z));
    }
  }

  shape.revision += 1u;
}

void replaceChildren(
  std::shared_ptr<const CompoundShape>& handle,
  std::span<const ChildReplacement> replacements
) {
  constexpr uint32_t gone = 0xFFFFFFFFu;

  if (replacements.empty()) {
    return;
  }

  const CompoundShape& before = *handle;
  const auto count = static_cast<uint32_t>(before.children.size());
  std::vector<uint32_t> replacing(count, gone);
  std::vector<CompoundChild> parts;
  std::vector<uint32_t> owner;

  for (uint32_t at = 0u; at != replacements.size(); ++at) {
    const std::optional<uint32_t> index = findChild(before, replacements[at].key);

    if (!index.has_value() || replacements[at].parts.empty() || replacing[*index] != gone) {
      throw std::runtime_error("a replaced compound child must belong to the shape once and have parts");
    }

    replacing[*index] = at;

    for (const CompoundChild& piece : replacements[at].parts) {
      parts.push_back(piece);
      owner.push_back(at);
    }
  }

  std::vector<uint32_t> order(parts.size());

  std::iota(order.begin(), order.end(), 0u);
  std::sort(order.begin(), order.end(), [&parts](uint32_t a, uint32_t b) {
    return parts[a].key < parts[b].key;
  });

  for (size_t at = 0u; at != order.size(); ++at) {
    const CompoundChild& piece = parts[order[at]];

    if ((at != 0u && parts[order[at - 1u]].key == piece.key) || findChild(before, piece.key).has_value()) {
      throw std::runtime_error("the parts of a compound child need keys the shape does not hold");
    }

    if (!(piece.half.x > 0.0 && piece.half.y > 0.0 && piece.half.z > 0.0) || !(piece.density >= 0.0)) {
      throw std::runtime_error("the parts of a compound child need positive half extents and a density of at least zero");
    }
  }

  if (handle.use_count() != 1) {
    handle = std::make_shared<CompoundShape>(*handle);
  }

  CompoundShape& shape = *std::const_pointer_cast<CompoundShape>(handle);
  const glm::dvec3 extent = shape.bounds.high - shape.bounds.low;
  const double reachOut = 1.0e-6 * std::max({extent.x, extent.y, extent.z});
  std::vector<uint32_t> found;

  for (uint32_t index = 0u; index != count; ++index) {
    if (replacing[index] == gone) {
      continue;
    }

    collectOverlapping(shape, grown(shape.childBounds[index], reachOut), found);

    for (const uint32_t other : found) {
      if (replacing[other] != gone) {
        continue;
      }

      const CompoundChild& child = shape.children[other];

      if (!overlaps(shape.childBounds[index], grown(shape.childBounds[other], 1.0e-9 * largestHalf(child)))) {
        continue;
      }

      const uint8_t covered = coveredBy(child, shape.children[index]);

      for (uint32_t face = 0u; face != 6u; ++face) {
        if ((covered & (1u << face)) == 0u) {
          continue;
        }

        uint16_t& covers = shape.faceCovers[6u * other + face];

        covers = static_cast<uint16_t>(covers - 1u);

        if (covers == 0u) {
          shape.exposed[other] = static_cast<uint8_t>(shape.exposed[other] | (1u << face));
        }
      }
    }
  }

  const size_t total = count - replacements.size() + parts.size();
  std::vector<CompoundChild> children;
  std::vector<Bounds> childBounds;
  std::vector<uint8_t> exposed;
  std::vector<double> childReach;
  std::vector<uint16_t> faceCovers;
  std::vector<uint32_t> renamed(count, gone);
  std::vector<uint32_t> placed;
  std::vector<std::vector<uint32_t>> partsOf(replacements.size());

  children.reserve(total);
  childBounds.reserve(total);
  exposed.reserve(total);
  childReach.reserve(total);
  faceCovers.reserve(6u * total);
  placed.reserve(parts.size());

  uint32_t old = 0u;
  size_t next = 0u;

  while (old != count || next != order.size()) {
    if (next != order.size() && (old == count || parts[order[next]].key < shape.children[old].key)) {
      const CompoundChild& piece = parts[order[next]];
      const glm::dmat3 turn = glm::mat3_cast(piece.rotation);
      const auto at = static_cast<uint32_t>(children.size());

      partsOf[owner[order[next]]].push_back(at);
      placed.push_back(at);
      children.push_back(piece);
      childBounds.push_back(boxBoundsIn(piece.centre, turn, piece.half));
      exposed.push_back(allFaces);
      childReach.push_back(cornerReach(piece, turn));
      faceCovers.insert(faceCovers.end(), 6u, uint16_t{0u});
      ++next;

      continue;
    }

    if (replacing[old] == gone) {
      renamed[old] = static_cast<uint32_t>(children.size());
      children.push_back(shape.children[old]);
      childBounds.push_back(shape.childBounds[old]);
      exposed.push_back(shape.exposed[old]);
      childReach.push_back(shape.childReach[old]);
      faceCovers.insert(
        faceCovers.end(),
        shape.faceCovers.begin() + 6u * static_cast<size_t>(old),
        shape.faceCovers.begin() + 6u * static_cast<size_t>(old) + 6u
      );
    }

    ++old;
  }

  const Bounds empty{
    glm::dvec3(std::numeric_limits<double>::infinity()),
    glm::dvec3(-std::numeric_limits<double>::infinity())
  };
  std::vector<uint32_t> leaves;

  leaves.reserve(total);

  for (ChildNode& node : shape.nodes) {
    if (node.left != 0u) {
      continue;
    }

    const auto first = static_cast<uint32_t>(leaves.size());
    Bounds box = empty;

    for (uint32_t at = node.first; at != node.first + node.count; ++at) {
      const uint32_t child = shape.leaves[at];

      if (replacing[child] == gone) {
        leaves.push_back(renamed[child]);
        box = unite(box, childBounds[renamed[child]]);
        continue;
      }

      for (const uint32_t piece : partsOf[replacing[child]]) {
        leaves.push_back(piece);
        box = unite(box, childBounds[piece]);
      }
    }

    node.first = first;
    node.count = static_cast<uint32_t>(leaves.size()) - first;
    node.bounds = box;
  }

  shape.children = std::move(children);
  shape.childBounds = std::move(childBounds);
  shape.exposed = std::move(exposed);
  shape.childReach = std::move(childReach);
  shape.faceCovers = std::move(faceCovers);
  shape.leaves = std::move(leaves);

  for (size_t index = shape.nodes.size(); index-- != 0u;) {
    ChildNode& node = shape.nodes[index];

    if (node.left != 0u) {
      node.bounds = unite(shape.nodes[node.left].bounds, shape.nodes[node.right].bounds);
    }
  }

  shape.bounds = shape.nodes.front().bounds;

  std::vector<uint8_t> fresh(shape.children.size(), 0u);
  std::vector<uint32_t> nearby;

  for (const uint32_t index : placed) {
    fresh[index] = 1u;
  }

  for (const uint32_t index : placed) {
    countCovers(shape, index, nearby);
    collectOverlapping(shape, grown(shape.childBounds[index], reachOut), found);

    for (const uint32_t other : found) {
      if (fresh[other] != 0u) {
        continue;
      }

      const CompoundChild& child = shape.children[other];

      if (!overlaps(shape.childBounds[index], grown(shape.childBounds[other], 1.0e-9 * largestHalf(child)))) {
        continue;
      }

      const uint8_t covered = coveredBy(child, shape.children[index]);

      for (uint32_t face = 0u; face != 6u; ++face) {
        if ((covered & (1u << face)) == 0u) {
          continue;
        }

        uint16_t& covers = shape.faceCovers[6u * other + face];

        if (covers == std::numeric_limits<uint16_t>::max()) {
          throw std::runtime_error("a compound child face can have at most 65535 covering children");
        }

        covers = static_cast<uint16_t>(covers + 1u);
        shape.exposed[other] = static_cast<uint8_t>(shape.exposed[other] & ~(1u << face));
      }
    }
  }

  shape.reach = 0.0;
  shape.thinnest = std::numeric_limits<double>::infinity();

  for (size_t index = 0u; index != shape.children.size(); ++index) {
    const glm::dvec3& half = shape.children[index].half;

    shape.reach = std::max(shape.reach, shape.childReach[index]);
    shape.thinnest = std::min({shape.thinnest, half.x, half.y, half.z});
  }

  shape.revision += 1u;
}

std::optional<uint32_t> findChild(const CompoundShape& shape, uint32_t key) {
  const auto found = std::lower_bound(
    shape.children.begin(),
    shape.children.end(),
    key,
    [](const CompoundChild& child, uint32_t wanted) { return child.key < wanted; }
  );

  if (found == shape.children.end() || found->key != key) {
    return std::nullopt;
  }

  return static_cast<uint32_t>(found - shape.children.begin());
}

void overlappingChildren(
  const CompoundShape& shape,
  const Bounds& probe,
  std::vector<uint32_t>& found
) {
  collectOverlapping(shape, probe, found);
  std::sort(found.begin(), found.end());
}

double childMass(const CompoundChild& child) {
  const glm::dvec3 size = 2.0 * child.half;

  return child.density * (size.x * size.y * size.z);
}

CompoundMass compoundMass(std::span<const CompoundChild> children) {
  CompoundMass result;
  glm::dvec3 moment(0.0);

  for (const CompoundChild& child : children) {
    const double mass = childMass(child);

    result.mass += mass;
    moment += mass * child.centre;
  }

  if (!(result.mass > 0.0)) {
    return result;
  }

  result.centre = moment / result.mass;

  for (const CompoundChild& child : children) {
    const glm::dvec3 size = 2.0 * child.half;
    const double mass = childMass(child);
    const glm::dmat3 turn = glm::mat3_cast(child.rotation);
    const glm::dvec3 arm = child.centre - result.centre;
    glm::dmat3 own(0.0);

    own[0][0] = mass * (size.y * size.y + size.z * size.z) / 12.0;
    own[1][1] = mass * (size.x * size.x + size.z * size.z) / 12.0;
    own[2][2] = mass * (size.x * size.x + size.y * size.y) / 12.0;
    result.inertia += turn * own * glm::transpose(turn)
                      + mass * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  return result;
}

PrincipalFrame principalFrame(std::span<const CompoundChild> children) {
  const CompoundMass mass = compoundMass(children);
  const SymmetricEigen eigen = jacobiEigen(mass.inertia);
  PrincipalFrame frame;

  frame.mass = mass.mass;
  frame.centre = mass.centre;
  frame.rotation = glm::normalize(glm::quat_cast(eigen.vectors));
  frame.moment = eigen.values;

  return frame;
}

std::vector<CompoundChild> childrenInFrame(
  std::span<const CompoundChild> children,
  const glm::dvec3& centre,
  const glm::dquat& rotation
) {
  const glm::dmat3 back = glm::transpose(glm::mat3_cast(rotation));
  const glm::dquat inverse = glm::conjugate(rotation);
  std::vector<CompoundChild> placed(children.begin(), children.end());

  for (CompoundChild& child : placed) {
    child.centre = back * (child.centre - centre);
    child.rotation = glm::normalize(inverse * child.rotation);
  }

  return placed;
}

std::array<glm::dvec3, childSampleCount> childSamples(const CompoundChild& child) {
  std::array<glm::dvec3, childSampleCount> samples{};
  const glm::dmat3 turn = glm::mat3_cast(child.rotation);
  uint32_t next = 0u;

  for (uint32_t i = 0u; i != 3u; ++i) {
    for (uint32_t j = 0u; j != 3u; ++j) {
      for (uint32_t k = 0u; k != 3u; ++k) {
        const bool surface = i == 0u || j == 0u || k == 0u || i == 2u || j == 2u || k == 2u;

        if (!surface) {
          continue;
        }

        const glm::dvec3 unit(i / 2.0, j / 2.0, k / 2.0);

        samples[next++] = child.centre + turn * ((2.0 * unit - glm::dvec3(1.0)) * child.half);
      }
    }
  }

  return samples;
}

}
