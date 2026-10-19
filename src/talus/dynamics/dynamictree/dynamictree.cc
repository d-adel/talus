#include "dynamictree.h"

#include <algorithm>
#include <limits>

namespace talus::dynamics {

namespace {

uint32_t spreadBits(uint32_t value) {
  value = (value * 0x00010001u) & 0xFF0000FFu;
  value = (value * 0x00000101u) & 0x0F00F00Fu;
  value = (value * 0x00000011u) & 0xC30C30C3u;
  value = (value * 0x00000005u) & 0x49249249u;

  return value;
}

uint32_t mortonCode(uint32_t x, uint32_t y, uint32_t z) {
  return (spreadBits(x) << 2u) | (spreadBits(y) << 1u) | spreadBits(z);
}

}

uint32_t DynamicTree::create(const Bounds& box, uint32_t user) {
  rebuild();

  const uint32_t leaf = allocate();

  m_nodes[leaf].box = box;
  m_nodes[leaf].user = user;
  m_nodes[leaf].height = 0u;
  insertLeaf(leaf);
  m_count += 1u;

  return leaf;
}

void DynamicTree::destroy(uint32_t proxy) {
  removeLeaf(proxy);
  release(proxy);
  m_count -= 1u;
}

void DynamicTree::move(uint32_t proxy, const Bounds& box) {
  rebuild();
  removeLeaf(proxy);
  m_nodes[proxy].box = box;
  insertLeaf(proxy);
}

void DynamicTree::resize(uint32_t proxy, const Bounds& box) {
  m_nodes[proxy].box = box;

  for (uint32_t index = m_nodes[proxy].parent;
       index != noNode && !m_nodes[index].flagged;
       index = m_nodes[index].parent) {
    m_nodes[index].flagged = true;
  }
}

void DynamicTree::refitFlagged() {
  if (m_root == noNode || !m_nodes[m_root].flagged) {
    return;
  }

  m_order.clear();
  m_stack.clear();
  m_stack.push_back(m_root);

  while (!m_stack.empty()) {
    const uint32_t index = m_stack.back();

    m_stack.pop_back();

    if (m_nodes[index].child1 == noNode || !m_nodes[index].flagged) {
      continue;
    }

    m_order.push_back(index);
    m_stack.push_back(m_nodes[index].child1);
    m_stack.push_back(m_nodes[index].child2);
  }

  for (size_t entry = m_order.size(); entry != 0u; --entry) {
    Node& node = m_nodes[m_order[entry - 1u]];

    node.box = unite(m_nodes[node.child1].box, m_nodes[node.child2].box);
  }
}

void DynamicTree::rebuild() {
  if (m_root == noNode || !m_nodes[m_root].flagged) {
    return;
  }

  m_leaves.clear();
  m_stack.clear();
  m_stack.push_back(m_root);

  while (!m_stack.empty()) {
    const uint32_t index = m_stack.back();

    m_stack.pop_back();

    if (m_nodes[index].child1 == noNode || !m_nodes[index].flagged) {
      m_leaves.push_back(index);
      continue;
    }

    m_stack.push_back(m_nodes[index].child1);
    m_stack.push_back(m_nodes[index].child2);
    release(index);
  }

  glm::dvec3 low(std::numeric_limits<double>::infinity());
  glm::dvec3 high(-std::numeric_limits<double>::infinity());

  for (const uint32_t index : m_leaves) {
    const glm::dvec3 centre = m_nodes[index].box.low + m_nodes[index].box.high;

    low = glm::min(low, centre);
    high = glm::max(high, centre);
  }

  const glm::dvec3 scale = 1023.0 / glm::max(high - low, glm::dvec3(1e-12));

  m_keys.clear();

  for (const uint32_t index : m_leaves) {
    const glm::dvec3 centre = m_nodes[index].box.low + m_nodes[index].box.high;
    const glm::dvec3 cell = glm::clamp((centre - low) * scale, glm::dvec3(0.0), glm::dvec3(1023.0));

    m_keys.push_back(
      static_cast<uint64_t>(mortonCode(
        static_cast<uint32_t>(cell.x),
        static_cast<uint32_t>(cell.y),
        static_cast<uint32_t>(cell.z)
      )) << 32u
      | index
    );
  }

  std::sort(m_keys.begin(), m_keys.end());

  for (size_t entry = 0u; entry != m_keys.size(); ++entry) {
    m_leaves[entry] = static_cast<uint32_t>(m_keys[entry] & 0xFFFFFFFFu);
  }

  m_root = build(0u, static_cast<uint32_t>(m_leaves.size()));
  m_nodes[m_root].parent = noNode;
}

uint32_t DynamicTree::build(uint32_t begin, uint32_t end) {
  if (end - begin == 1u) {
    return m_leaves[begin];
  }

  const uint32_t middle = begin + (end - begin) / 2u;
  const uint32_t left = build(begin, middle);
  const uint32_t right = build(middle, end);
  const uint32_t parent = allocate();

  m_nodes[parent].child1 = left;
  m_nodes[parent].child2 = right;
  m_nodes[left].parent = parent;
  m_nodes[right].parent = parent;
  refit(parent);

  return parent;
}

const Bounds& DynamicTree::box(uint32_t proxy) const {
  return m_nodes[proxy].box;
}

uint32_t DynamicTree::user(uint32_t proxy) const {
  return m_nodes[proxy].user;
}

uint32_t DynamicTree::height() const {
  return m_root == noNode ? 0u : m_nodes[m_root].height;
}

uint32_t DynamicTree::count() const {
  return m_count;
}

uint32_t DynamicTree::allocate() {
  if (!m_free.empty()) {
    const uint32_t index = m_free.back();

    m_free.pop_back();
    m_nodes[index] = Node{};

    return index;
  }

  m_nodes.emplace_back();

  return static_cast<uint32_t>(m_nodes.size() - 1u);
}

void DynamicTree::release(uint32_t index) {
  m_free.push_back(index);
}

void DynamicTree::refit(uint32_t index) {
  Node& node = m_nodes[index];

  node.box = unite(m_nodes[node.child1].box, m_nodes[node.child2].box);
  node.height = 1u + std::max(m_nodes[node.child1].height, m_nodes[node.child2].height);
}

uint32_t DynamicTree::findBestSibling(const Bounds& box) const {
  const glm::dvec3 centre = 0.5 * (box.low + box.high);
  const double areaD = area(box);
  uint32_t index = m_root;
  double areaBase = area(m_nodes[index].box);
  double directCost = area(unite(m_nodes[index].box, box));
  double inheritedCost = 0.0;
  uint32_t best = index;
  double bestCost = directCost;

  while (m_nodes[index].height > 0u) {
    const uint32_t child1 = m_nodes[index].child1;
    const uint32_t child2 = m_nodes[index].child2;
    const double cost = directCost + inheritedCost;

    if (cost < bestCost) {
      best = index;
      bestCost = cost;
    }

    inheritedCost += directCost - areaBase;

    const bool leaf1 = m_nodes[child1].height == 0u;
    const bool leaf2 = m_nodes[child2].height == 0u;
    const Bounds& box1 = m_nodes[child1].box;
    const Bounds& box2 = m_nodes[child2].box;
    const double directCost1 = area(unite(box1, box));
    const double directCost2 = area(unite(box2, box));
    double lowerCost1 = std::numeric_limits<double>::max();
    double lowerCost2 = std::numeric_limits<double>::max();
    double area1 = 0.0;
    double area2 = 0.0;

    if (leaf1) {
      if (directCost1 + inheritedCost < bestCost) {
        best = child1;
        bestCost = directCost1 + inheritedCost;
      }
    } else {
      area1 = area(box1);
      lowerCost1 = inheritedCost + directCost1 + std::min(areaD - area1, 0.0);
    }

    if (leaf2) {
      if (directCost2 + inheritedCost < bestCost) {
        best = child2;
        bestCost = directCost2 + inheritedCost;
      }
    } else {
      area2 = area(box2);
      lowerCost2 = inheritedCost + directCost2 + std::min(areaD - area2, 0.0);
    }

    if (leaf1 && leaf2) {
      break;
    }

    if (bestCost <= lowerCost1 && bestCost <= lowerCost2) {
      break;
    }

    if (lowerCost1 == lowerCost2 && !leaf1) {
      const glm::dvec3 d1 = 0.5 * (box1.low + box1.high) - centre;
      const glm::dvec3 d2 = 0.5 * (box2.low + box2.high) - centre;

      lowerCost1 = glm::dot(d1, d1);
      lowerCost2 = glm::dot(d2, d2);
    }

    if (lowerCost1 < lowerCost2 && !leaf1) {
      index = child1;
      areaBase = area1;
      directCost = directCost1;
    } else {
      index = child2;
      areaBase = area2;
      directCost = directCost2;
    }
  }

  return best;
}

void DynamicTree::rotate(uint32_t a) {
  Node& nodeA = m_nodes[a];

  if (nodeA.height < 2u) {
    return;
  }

  const uint32_t b = nodeA.child1;
  const uint32_t c = nodeA.child2;
  Node& nodeB = m_nodes[b];
  Node& nodeC = m_nodes[c];

  if (nodeB.height == 0u) {
    const uint32_t f = nodeC.child1;
    const uint32_t g = nodeC.child2;
    const Bounds boxBG = unite(nodeB.box, m_nodes[g].box);
    const Bounds boxBF = unite(nodeB.box, m_nodes[f].box);
    const double costBase = area(nodeC.box);
    const double costBF = area(boxBG);
    const double costBG = area(boxBF);

    if (costBase < costBF && costBase < costBG) {
      return;
    }

    if (costBF < costBG) {
      nodeA.child1 = f;
      nodeC.child1 = b;
      nodeB.parent = c;
      m_nodes[f].parent = a;
      nodeC.box = boxBG;
      nodeC.height = 1u + std::max(nodeB.height, m_nodes[g].height);
      nodeA.height = 1u + std::max(nodeC.height, m_nodes[f].height);
    } else {
      nodeA.child1 = g;
      nodeC.child2 = b;
      nodeB.parent = c;
      m_nodes[g].parent = a;
      nodeC.box = boxBF;
      nodeC.height = 1u + std::max(nodeB.height, m_nodes[f].height);
      nodeA.height = 1u + std::max(nodeC.height, m_nodes[g].height);
    }

    return;
  }

  if (nodeC.height == 0u) {
    const uint32_t d = nodeB.child1;
    const uint32_t e = nodeB.child2;
    const Bounds boxCE = unite(nodeC.box, m_nodes[e].box);
    const Bounds boxCD = unite(nodeC.box, m_nodes[d].box);
    const double costBase = area(nodeB.box);
    const double costCD = area(boxCE);
    const double costCE = area(boxCD);

    if (costBase < costCD && costBase < costCE) {
      return;
    }

    if (costCD < costCE) {
      nodeA.child2 = d;
      nodeB.child1 = c;
      nodeC.parent = b;
      m_nodes[d].parent = a;
      nodeB.box = boxCE;
      nodeB.height = 1u + std::max(nodeC.height, m_nodes[e].height);
      nodeA.height = 1u + std::max(nodeB.height, m_nodes[d].height);
    } else {
      nodeA.child2 = e;
      nodeB.child2 = c;
      nodeC.parent = b;
      m_nodes[e].parent = a;
      nodeB.box = boxCD;
      nodeB.height = 1u + std::max(nodeC.height, m_nodes[d].height);
      nodeA.height = 1u + std::max(nodeB.height, m_nodes[e].height);
    }

    return;
  }

  const uint32_t d = nodeB.child1;
  const uint32_t e = nodeB.child2;
  const uint32_t f = nodeC.child1;
  const uint32_t g = nodeC.child2;
  const double areaB = area(nodeB.box);
  const double areaC = area(nodeC.box);
  const Bounds boxBG = unite(nodeB.box, m_nodes[g].box);
  const Bounds boxBF = unite(nodeB.box, m_nodes[f].box);
  const Bounds boxCE = unite(nodeC.box, m_nodes[e].box);
  const Bounds boxCD = unite(nodeC.box, m_nodes[d].box);
  const std::array<double, 5> costs{
    areaB + areaC,
    areaB + area(boxBG),
    areaB + area(boxBF),
    areaC + area(boxCE),
    areaC + area(boxCD)
  };
  const auto best = static_cast<uint32_t>(
    std::min_element(costs.begin(), costs.end()) - costs.begin()
  );

  if (best == 1u) {
    nodeA.child1 = f;
    nodeC.child1 = b;
    nodeB.parent = c;
    m_nodes[f].parent = a;
    nodeC.box = boxBG;
    nodeC.height = 1u + std::max(nodeB.height, m_nodes[g].height);
    nodeA.height = 1u + std::max(nodeC.height, m_nodes[f].height);
  } else if (best == 2u) {
    nodeA.child1 = g;
    nodeC.child2 = b;
    nodeB.parent = c;
    m_nodes[g].parent = a;
    nodeC.box = boxBF;
    nodeC.height = 1u + std::max(nodeB.height, m_nodes[f].height);
    nodeA.height = 1u + std::max(nodeC.height, m_nodes[g].height);
  } else if (best == 3u) {
    nodeA.child2 = d;
    nodeB.child1 = c;
    nodeC.parent = b;
    m_nodes[d].parent = a;
    nodeB.box = boxCE;
    nodeB.height = 1u + std::max(nodeC.height, m_nodes[e].height);
    nodeA.height = 1u + std::max(nodeB.height, m_nodes[d].height);
  } else if (best == 4u) {
    nodeA.child2 = e;
    nodeB.child2 = c;
    nodeC.parent = b;
    m_nodes[e].parent = a;
    nodeB.box = boxCD;
    nodeB.height = 1u + std::max(nodeC.height, m_nodes[d].height);
    nodeA.height = 1u + std::max(nodeB.height, m_nodes[e].height);
  }
}

void DynamicTree::insertLeaf(uint32_t leaf) {
  if (m_root == noNode) {
    m_root = leaf;
    m_nodes[leaf].parent = noNode;
    return;
  }

  const uint32_t sibling = findBestSibling(m_nodes[leaf].box);
  const uint32_t oldParent = m_nodes[sibling].parent;
  const uint32_t newParent = allocate();

  m_nodes[newParent].parent = oldParent;
  m_nodes[newParent].child1 = sibling;
  m_nodes[newParent].child2 = leaf;
  m_nodes[newParent].box = unite(m_nodes[leaf].box, m_nodes[sibling].box);
  m_nodes[newParent].height = m_nodes[sibling].height + 1u;

  if (oldParent == noNode) {
    m_root = newParent;
  } else if (m_nodes[oldParent].child1 == sibling) {
    m_nodes[oldParent].child1 = newParent;
  } else {
    m_nodes[oldParent].child2 = newParent;
  }

  m_nodes[sibling].parent = newParent;
  m_nodes[leaf].parent = newParent;

  for (uint32_t index = m_nodes[leaf].parent; index != noNode; index = m_nodes[index].parent) {
    refit(index);
    rotate(index);
  }
}

void DynamicTree::removeLeaf(uint32_t leaf) {
  if (leaf == m_root) {
    m_root = noNode;
    return;
  }

  const uint32_t parent = m_nodes[leaf].parent;
  const uint32_t grandParent = m_nodes[parent].parent;
  const uint32_t sibling =
    m_nodes[parent].child1 == leaf ? m_nodes[parent].child2 : m_nodes[parent].child1;

  release(parent);

  if (grandParent == noNode) {
    m_root = sibling;
    m_nodes[sibling].parent = noNode;
    return;
  }

  if (m_nodes[grandParent].child1 == parent) {
    m_nodes[grandParent].child1 = sibling;
  } else {
    m_nodes[grandParent].child2 = sibling;
  }

  m_nodes[sibling].parent = grandParent;

  for (uint32_t index = grandParent; index != noNode; index = m_nodes[index].parent) {
    refit(index);
  }
}

}
