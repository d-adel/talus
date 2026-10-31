#pragma once

#include <talus/dynamics/bounds/bounds.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace talus::dynamics {

inline constexpr uint32_t noNode = 0xFFFFFFFFu;

class DynamicTree {
public:
  uint32_t create(const Bounds& box, uint32_t user);
  void destroy(uint32_t proxy);
  void move(uint32_t proxy, const Bounds& box);
  void resize(uint32_t proxy, const Bounds& box);
  void refitFlagged();
  void rebuild();

  [[nodiscard]] const Bounds& box(uint32_t proxy) const;
  [[nodiscard]] uint32_t user(uint32_t proxy) const;
  [[nodiscard]] uint32_t height() const;
  [[nodiscard]] uint32_t count() const;

  template<typename Visit>
  void query(const Bounds& probe, Visit&& visit) const {
    if (m_root == noNode) {
      return;
    }

    std::array<uint32_t, 1024> stack;
    uint32_t top = 0u;

    stack[top++] = m_root;

    while (top != 0u) {
      const uint32_t index = stack[--top];
      const Node& node = m_nodes[index];

      if (!overlaps(node.box, probe)) {
        continue;
      }

      if (node.child1 == noNode) {
        visit(index, node.user);
      } else {
        stack[top++] = node.child1;
        stack[top++] = node.child2;
      }
    }
  }

  template<typename Visit>
  double rayQuery(
    const glm::dvec3& from,
    const glm::dvec3& to,
    double clip,
    Visit&& visit
  ) const {
    if (m_root == noNode) {
      return clip;
    }

    const glm::dvec3 span = to - from;
    std::array<uint32_t, 1024> stack;
    uint32_t top = 0u;

    stack[top++] = m_root;

    while (top != 0u) {
      const Node& node = m_nodes[stack[--top]];

      if (!segmentMeets(node.box, from, span, clip)) {
        continue;
      }

      if (node.child1 == noNode) {
        clip = visit(node.user, clip);
      } else {
        stack[top++] = node.child1;
        stack[top++] = node.child2;
      }
    }

    return clip;
  }

private:
  struct Node {
    Bounds box;
    uint32_t parent = noNode;
    uint32_t child1 = noNode;
    uint32_t child2 = noNode;
    uint32_t height = 0u;
    uint32_t user = 0u;
    bool flagged = false;
  };

  uint32_t allocate();
  void release(uint32_t index);
  void insertLeaf(uint32_t leaf);
  void removeLeaf(uint32_t leaf);
  [[nodiscard]] uint32_t findBestSibling(const Bounds& box) const;
  void rotate(uint32_t index);
  void refit(uint32_t index);
  uint32_t build(uint32_t begin, uint32_t end);

  std::vector<Node> m_nodes;
  std::vector<uint32_t> m_free;
  std::vector<uint32_t> m_leaves;
  std::vector<uint64_t> m_keys;
  std::vector<uint32_t> m_stack;
  std::vector<uint32_t> m_order;
  uint32_t m_root = noNode;
  uint32_t m_count = 0u;
};

}
