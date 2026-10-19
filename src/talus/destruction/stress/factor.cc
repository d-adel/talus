#include "factor.h"

#include "sums.h"

#include <talus/dynamics/softstep/simdwidth.h>

#include <Eigen/OrderingMethods>
#include <Eigen/SparseCore>

#include <emmintrin.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace talus::destruction {

namespace {

constexpr double parallelWork = 2.0e6;
constexpr double stripeWork = 6.0e4;

size_t at(int32_t column, int32_t stride, int32_t row) {
  return static_cast<size_t>(column) * static_cast<size_t>(stride) + static_cast<size_t>(row);
}

bool factorDiagonal(double* block, int32_t stride) {
  for (int32_t j = 0; j != 6; ++j) {
    double pivot = block[at(j, stride, j)];

    for (int32_t k = 0; k != j; ++k) {
      const double value = block[at(k, stride, j)];

      pivot -= value * value;
    }

    if (!(pivot > 0.0)) {
      return false;
    }

    const double root = std::sqrt(pivot);

    block[at(j, stride, j)] = root;

    for (int32_t i = j + 1; i != 6; ++i) {
      double value = block[at(j, stride, i)];

      for (int32_t k = 0; k != j; ++k) {
        value -= block[at(k, stride, i)] * block[at(k, stride, j)];
      }

      block[at(j, stride, i)] = value / root;
    }
  }

  return true;
}

void solveRows(double* column, int32_t stride, int32_t begin, int32_t end) {
  double* below = column + 6;

  for (int32_t q = 0; q != 6; ++q) {
    double* out = below + at(q, stride, 0);

    for (int32_t s = 0; s != q; ++s) {
      const double weight = column[at(s, stride, q)];
      const double* in = below + at(s, stride, 0);

      for (int32_t r = begin; r != end; ++r) {
        out[r] -= in[r] * weight;
      }
    }

    const double diagonal = column[at(q, stride, q)];

    for (int32_t r = begin; r != end; ++r) {
      out[r] /= diagonal;
    }
  }
}

int32_t countOf(const BlockPattern& pattern, int32_t j) {
  return pattern.start[static_cast<size_t>(j) + 1u] - pattern.start[static_cast<size_t>(j)];
}

int32_t strideOf(const BlockPattern& pattern, int32_t j) {
  return 6 * countOf(pattern, j);
}

struct Node {
  int32_t first = 0;
  int32_t last = 0;
  const int32_t* below = nullptr;
  int32_t belowCount = 0;
};

Node nodeAt(const BlockPattern& pattern, int32_t node) {
  const int32_t first = pattern.nodeStart[static_cast<size_t>(node)];
  const int32_t last = pattern.nodeStart[static_cast<size_t>(node) + 1u] - 1;

  return Node{
    first,
    last,
    pattern.rows.data() + pattern.start[static_cast<size_t>(last)] + 1,
    countOf(pattern, last) - 1,
  };
}

struct Scratch {
  std::vector<int32_t> local;
  std::vector<double> product;
  bool wide = false;
};

Scratch scratchFor(const BlockPattern& pattern, Lanes lanes) {
  const int32_t columns = static_cast<int32_t>(pattern.start.size()) - 1;
  int32_t widest = 0;

  for (int32_t j = 0; j != columns; ++j) {
    widest = std::max(widest, countOf(pattern, j));
  }

  return Scratch{
    std::vector<int32_t>(static_cast<size_t>(columns), -1),
    std::vector<double>(36u * static_cast<size_t>(widest), 0.0),
    lanes == Lanes::Widest && dynamics::hasAvx2(),
  };
}

void sumNarrow(double* product, int32_t span, const SumSources& sources, int32_t count) {
  int32_t r = 0;

  for (; r + 4 <= span; r += 4) {
    __m128d sum[12];

    for (__m128d& lane : sum) {
      lane = _mm_setzero_pd();
    }

    for (int32_t which = 0; which != count; ++which) {
      const SumSource& source = sources[static_cast<size_t>(which)];

      for (int32_t k = 0; k != 6; ++k) {
        const double* in = source.rows + static_cast<ptrdiff_t>(k) * source.stride + r;
        const double* weight = source.pivot + static_cast<ptrdiff_t>(k) * source.stride;
        const __m128d low = _mm_loadu_pd(in);
        const __m128d high = _mm_loadu_pd(in + 2);

        for (int32_t q = 0; q != 6; ++q) {
          const __m128d w = _mm_set1_pd(weight[q]);

          sum[2 * q] = _mm_add_pd(sum[2 * q], _mm_mul_pd(low, w));
          sum[2 * q + 1] = _mm_add_pd(sum[2 * q + 1], _mm_mul_pd(high, w));
        }
      }
    }

    for (int32_t q = 0; q != 6; ++q) {
      _mm_storeu_pd(product + at(q, span, r), sum[2 * q]);
      _mm_storeu_pd(product + at(q, span, r) + 2, sum[2 * q + 1]);
    }
  }

  for (; r != span; r += 2) {
    __m128d sum[6];

    for (__m128d& lane : sum) {
      lane = _mm_setzero_pd();
    }

    for (int32_t which = 0; which != count; ++which) {
      const SumSource& source = sources[static_cast<size_t>(which)];

      for (int32_t k = 0; k != 6; ++k) {
        const double* weight = source.pivot + static_cast<ptrdiff_t>(k) * source.stride;
        const __m128d low = _mm_loadu_pd(source.rows + static_cast<ptrdiff_t>(k) * source.stride + r);

        for (int32_t q = 0; q != 6; ++q) {
          sum[q] = _mm_add_pd(sum[q], _mm_mul_pd(low, _mm_set1_pd(weight[q])));
        }
      }
    }

    for (int32_t q = 0; q != 6; ++q) {
      _mm_storeu_pd(product + at(q, span, r), sum[q]);
    }
  }
}

void sumSources(const Scratch& scratch, double* product, int32_t span, const SumSources& sources, int32_t count) {
  if (scratch.wide) {
    sumWide(product, span, sources, count);
  } else {
    sumNarrow(product, span, sources, count);
  }
}

void subtractRows(double* target, int32_t stride, const double* product, int32_t rows, int32_t begin) {
  for (int32_t q = 0; q != 6; ++q) {
    double* out = target + at(q, stride, 0);
    const double* in = product + at(q, rows, begin);

    for (int32_t r = 0; r != 6; ++r) {
      out[r] -= in[r];
    }
  }
}

void mapNode(const BlockPattern& pattern, const Node& node, std::vector<int32_t>& local) {
  const int32_t first = pattern.start[static_cast<size_t>(node.first)];
  const int32_t count = countOf(pattern, node.first);

  for (int32_t t = 0; t != count; ++t) {
    local[static_cast<size_t>(pattern.rows[static_cast<size_t>(first + t)])] = t;
  }
}

int32_t firstAtLeast(const int32_t* rows, int32_t begin, int32_t end, int32_t wanted, const std::vector<int32_t>& local) {
  while (begin != end) {
    const int32_t middle = begin + (end - begin) / 2;

    if (local[static_cast<size_t>(rows[middle])] < wanted) {
      begin = middle + 1;
    } else {
      end = middle;
    }
  }

  return begin;
}

void applySource(
  const BlockPattern& pattern,
  std::vector<double>& values,
  const Node& target,
  int32_t source,
  int32_t from,
  int32_t low,
  int32_t high,
  Scratch& scratch
) {
  const std::vector<int32_t>& local = scratch.local;
  const Node node = nodeAt(pattern, source);
  const int32_t* rows = node.below;
  const int32_t count = node.belowCount;
  const int32_t first = static_cast<int32_t>(std::lower_bound(rows, rows + count, from) - rows);
  const int32_t last = static_cast<int32_t>(std::lower_bound(rows + first, rows + count, target.last + 1) - rows);

  if (first == last) {
    return;
  }

  const int32_t begin = firstAtLeast(rows, first, count, low, local);
  const int32_t end = firstAtLeast(rows, begin, count, high, local);

  for (int32_t q = first; q != last; ++q) {
    const int32_t column = rows[q];
    const int32_t shift = column - target.first;
    const int32_t stride = strideOf(pattern, column);
    double* panel = values.data() + pattern.panel[static_cast<size_t>(column)];

    const int32_t i0 = std::max(q, begin);

    if (i0 >= end) {
      continue;
    }

    const int32_t span = 6 * (end - i0);
    double* product = scratch.product.data();
    SumSources sources{};

    for (int32_t c = node.first; c <= node.last; ++c) {
      const double* sourcePanel = values.data() + pattern.panel[static_cast<size_t>(c)];
      const int32_t offset = node.last - c + 1;

      sources[static_cast<size_t>(c - node.first)] =
        SumSource{sourcePanel + 6 * (offset + i0), sourcePanel + 6 * (offset + q), strideOf(pattern, c)};
    }

    sumSources(scratch, product, span, sources, node.last - node.first + 1);

    for (int32_t i = i0; i != end; ++i) {
      subtractRows(panel + 6 * (local[static_cast<size_t>(rows[i])] - shift), stride, product, span, 6 * (i - i0));
    }
  }
}

void applyIntra(
  const BlockPattern& pattern,
  std::vector<double>& values,
  const Node& node,
  int32_t column,
  int32_t begin,
  int32_t end,
  Scratch& scratch
) {
  if (column == node.first || begin == end) {
    return;
  }

  const int32_t stride = strideOf(pattern, column);
  const int32_t span = 6 * (end - begin);
  double* panel = values.data() + pattern.panel[static_cast<size_t>(column)];
  double* product = scratch.product.data();
  SumSources sources{};

  for (int32_t previous = node.first; previous != column; ++previous) {
    const double* sourcePanel = values.data() + pattern.panel[static_cast<size_t>(previous)];
    const int32_t shift = column - previous;

    sources[static_cast<size_t>(previous - node.first)] =
      SumSource{sourcePanel + 6 * (begin + shift), sourcePanel + 6 * shift, strideOf(pattern, previous)};
  }

  sumSources(scratch, product, span, sources, column - node.first);

  for (int32_t q = 0; q != 6; ++q) {
    double* out = panel + at(q, stride, 6 * begin);
    const double* in = product + at(q, span, 0);

    for (int32_t r = 0; r != span; ++r) {
      out[r] -= in[r];
    }
  }
}

int32_t factorTriangle(const BlockPattern& pattern, std::vector<double>& values, const Node& node, int32_t from, Scratch& scratch) {
  for (int32_t column = from; column <= node.last; ++column) {
    const int32_t stride = strideOf(pattern, column);
    double* panel = values.data() + pattern.panel[static_cast<size_t>(column)];

    applyIntra(pattern, values, node, column, 0, node.last - column + 1, scratch);

    if (!factorDiagonal(panel, stride)) {
      return column;
    }

    solveRows(panel, stride, 0, 6 * (node.last - column));
  }

  return -1;
}

void solveBelow(
  const BlockPattern& pattern,
  std::vector<double>& values,
  const Node& node,
  int32_t from,
  int32_t begin,
  int32_t end,
  Scratch& scratch
) {
  for (int32_t column = from; column <= node.last; ++column) {
    const int32_t offset = node.last - column + 1;
    double* panel = values.data() + pattern.panel[static_cast<size_t>(column)];

    applyIntra(pattern, values, node, column, offset + begin, offset + end, scratch);
    solveRows(panel, strideOf(pattern, column), 6 * (offset + begin - 1), 6 * (offset + end - 1));
  }
}

void applySources(
  const BlockPattern& pattern,
  std::vector<double>& values,
  int32_t node,
  const Node& target,
  int32_t from,
  int32_t low,
  int32_t high,
  Scratch& scratch
) {
  for (int32_t at = pattern.sourceStart[static_cast<size_t>(node)]; at != pattern.sourceStart[static_cast<size_t>(node) + 1u]; ++at) {
    applySource(pattern, values, target, pattern.sources[static_cast<size_t>(at)], from, low, high, scratch);
  }
}

int32_t factorNode(const BlockPattern& pattern, std::vector<double>& values, int32_t node, int32_t from, Scratch& scratch) {
  const Node target = nodeAt(pattern, node);

  mapNode(pattern, target, scratch.local);
  applySources(pattern, values, node, target, from, from - target.first, countOf(pattern, target.first), scratch);

  const int32_t failed = factorTriangle(pattern, values, target, from, scratch);

  if (failed >= 0) {
    return failed;
  }

  solveBelow(pattern, values, target, from, 0, target.belowCount, scratch);

  return -1;
}

std::vector<int32_t> startsOf(const BlockPattern& pattern, std::span<const int32_t> columns) {
  std::vector<int32_t> starts(pattern.nodeStart.size() - 1u, -1);

  for (auto at = columns.rbegin(); at != columns.rend(); ++at) {
    starts[static_cast<size_t>(pattern.nodeOf[static_cast<size_t>(*at)])] = *at;
  }

  return starts;
}

void orderNodes(BlockPattern& pattern) {
  const int32_t columns = static_cast<int32_t>(pattern.start.size()) - 1;

  pattern.nodeStart.clear();
  pattern.nodeOf.assign(static_cast<size_t>(columns), 0);

  for (int32_t column = 0; column != columns; ++column) {
    const bool continues = column > 0 && pattern.parent[static_cast<size_t>(column) - 1u] == column
                           && countOf(pattern, column - 1) == countOf(pattern, column) + 1
                           && column - pattern.nodeStart.back() < widestNode;

    if (!continues) {
      pattern.nodeStart.push_back(column);
    }

    pattern.nodeOf[static_cast<size_t>(column)] = static_cast<int32_t>(pattern.nodeStart.size()) - 1;
  }

  pattern.nodeStart.push_back(columns);

  const int32_t nodes = static_cast<int32_t>(pattern.nodeStart.size()) - 1;
  std::vector<int32_t> seen(static_cast<size_t>(nodes), -1);
  std::vector<int32_t> counts(static_cast<size_t>(nodes) + 1u, 0);
  std::vector<std::vector<int32_t>> targets(static_cast<size_t>(nodes));

  pattern.nodeParent.assign(static_cast<size_t>(nodes), -1);

  for (int32_t node = 0; node != nodes; ++node) {
    const Node source = nodeAt(pattern, node);

    if (source.belowCount != 0) {
      pattern.nodeParent[static_cast<size_t>(node)] = pattern.nodeOf[static_cast<size_t>(source.below[0])];
    }

    for (int32_t i = 0; i != source.belowCount; ++i) {
      const int32_t target = pattern.nodeOf[static_cast<size_t>(source.below[i])];

      if (seen[static_cast<size_t>(target)] != node) {
        seen[static_cast<size_t>(target)] = node;
        targets[static_cast<size_t>(node)].push_back(target);
        counts[static_cast<size_t>(target) + 1u] += 1;
      }
    }
  }

  for (int32_t node = 0; node != nodes; ++node) {
    counts[static_cast<size_t>(node) + 1u] += counts[static_cast<size_t>(node)];
  }

  pattern.sourceStart = counts;
  pattern.sources.assign(static_cast<size_t>(counts.back()), 0);

  for (int32_t node = 0; node != nodes; ++node) {
    for (const int32_t target : targets[static_cast<size_t>(node)]) {
      pattern.sources[static_cast<size_t>(counts[static_cast<size_t>(target)]++)] = node;
    }
  }

  pattern.work.assign(static_cast<size_t>(columns), 0.0);

  for (int32_t column = 0; column != columns; ++column) {
    const int32_t first = pattern.start[static_cast<size_t>(column)];
    const int32_t below = countOf(pattern, column) - 1;

    pattern.work[static_cast<size_t>(column)] += 126.0 * below + 36.0;

    for (int32_t t = 1; t <= below; ++t) {
      pattern.work[static_cast<size_t>(pattern.rows[static_cast<size_t>(first + t)])] += 216.0 * (below - t + 1);
    }
  }
}

struct Plan {
  std::vector<int32_t> starts;
  std::vector<int32_t> roots;
  std::vector<int32_t> rootStart;
  std::vector<int32_t> members;
  std::vector<int32_t> top;
  std::vector<double> nodeWork;
};

Plan planOf(const BlockPattern& pattern, std::span<const int32_t> columns, uint32_t threads, double& total) {
  const size_t nodes = pattern.nodeStart.size() - 1u;
  Plan plan;

  plan.starts = startsOf(pattern, columns);
  plan.nodeWork.assign(nodes, 0.0);
  total = 0.0;

  for (const int32_t j : columns) {
    plan.nodeWork[static_cast<size_t>(pattern.nodeOf[static_cast<size_t>(j)])] += pattern.work[static_cast<size_t>(j)];
    total += pattern.work[static_cast<size_t>(j)];
  }

  if (threads <= 1u) {
    return plan;
  }

  const double limit = total / (4.0 * threads);
  std::vector<double> subtree(plan.nodeWork);

  for (size_t node = 0u; node != nodes; ++node) {
    const int32_t up = pattern.nodeParent[node];

    if (plan.starts[node] >= 0 && up >= 0) {
      subtree[static_cast<size_t>(up)] += subtree[node];
    }
  }

  std::vector<int32_t> owner(nodes, -1);

  for (size_t node = nodes; node-- != 0u;) {
    if (plan.starts[node] < 0) {
      continue;
    }

    const int32_t up = pattern.nodeParent[node];

    if (subtree[node] > limit) {
      plan.top.push_back(static_cast<int32_t>(node));
    } else if (up >= 0 && owner[static_cast<size_t>(up)] >= 0) {
      owner[node] = owner[static_cast<size_t>(up)];
    } else {
      owner[node] = static_cast<int32_t>(plan.roots.size());
      plan.roots.push_back(static_cast<int32_t>(node));
    }
  }

  std::reverse(plan.top.begin(), plan.top.end());
  plan.rootStart.assign(plan.roots.size() + 1u, 0);

  for (size_t node = 0u; node != nodes; ++node) {
    if (owner[node] >= 0) {
      plan.rootStart[static_cast<size_t>(owner[node]) + 1u] += 1;
    }
  }

  for (size_t root = 0u; root != plan.roots.size(); ++root) {
    plan.rootStart[root + 1u] += plan.rootStart[root];
  }

  plan.members.resize(static_cast<size_t>(plan.rootStart.back()));

  std::vector<int32_t> cursor(plan.rootStart.begin(), plan.rootStart.end() - 1);

  for (size_t node = 0u; node != nodes; ++node) {
    if (owner[node] >= 0) {
      plan.members[static_cast<size_t>(cursor[static_cast<size_t>(owner[node])]++)] = static_cast<int32_t>(node);
    }
  }

  std::vector<int32_t> byWork(plan.roots.size());
  std::vector<int32_t> roots = plan.roots;
  std::vector<int32_t> starts = plan.rootStart;
  std::vector<int32_t> members = plan.members;

  for (size_t root = 0u; root != byWork.size(); ++root) {
    byWork[root] = static_cast<int32_t>(root);
  }

  std::stable_sort(byWork.begin(), byWork.end(), [&](int32_t a, int32_t b) {
    return subtree[static_cast<size_t>(roots[static_cast<size_t>(a)])] > subtree[static_cast<size_t>(roots[static_cast<size_t>(b)])];
  });

  plan.members.clear();
  plan.rootStart.assign(1u, 0);

  for (size_t at = 0u; at != byWork.size(); ++at) {
    const int32_t root = byWork[at];

    plan.roots[at] = roots[static_cast<size_t>(root)];
    plan.members.insert(
      plan.members.end(),
      members.begin() + starts[static_cast<size_t>(root)],
      members.begin() + starts[static_cast<size_t>(root) + 1u]
    );
    plan.rootStart.push_back(static_cast<int32_t>(plan.members.size()));
  }

  return plan;
}

}

BlockPattern blockPattern(int32_t columns, std::span<const std::array<int32_t, 2>> links) {
  BlockPattern pattern;

  pattern.place.assign(static_cast<size_t>(columns), -1);
  pattern.start.assign(static_cast<size_t>(columns) + 1u, 0);
  pattern.panel.assign(static_cast<size_t>(columns) + 1u, 0);
  pattern.nodeStart.assign(1u, 0);
  pattern.sourceStart.assign(1u, 0);

  if (columns == 0) {
    return pattern;
  }

  std::vector<Eigen::Triplet<double, int>> entries;

  for (int32_t index = 0; index != columns; ++index) {
    entries.emplace_back(index, index, 1.0);
  }

  for (const std::array<int32_t, 2>& link : links) {
    entries.emplace_back(link[0], link[1], 1.0);
    entries.emplace_back(link[1], link[0], 1.0);
  }

  Eigen::SparseMatrix<double, Eigen::ColMajor, int> graph(columns, columns);
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> order;
  Eigen::AMDOrdering<int> ordering;

  graph.setFromTriplets(entries.begin(), entries.end());
  ordering(graph, order);

  for (int32_t place = 0; place != columns; ++place) {
    pattern.place[static_cast<size_t>(order.indices()(place))] = place;
  }

  std::vector<std::vector<int32_t>> lower(static_cast<size_t>(columns));

  for (const std::array<int32_t, 2>& link : links) {
    const int32_t a = pattern.place[static_cast<size_t>(link[0])];
    const int32_t b = pattern.place[static_cast<size_t>(link[1])];

    lower[static_cast<size_t>(std::min(a, b))].push_back(std::max(a, b));
  }

  std::vector<std::vector<int32_t>> structure(static_cast<size_t>(columns));
  std::vector<std::vector<int32_t>> children(static_cast<size_t>(columns));
  std::vector<int32_t> mark(static_cast<size_t>(columns), -1);

  for (int32_t column = 0; column != columns; ++column) {
    std::vector<int32_t>& rows = structure[static_cast<size_t>(column)];

    mark[static_cast<size_t>(column)] = column;

    for (const int32_t row : lower[static_cast<size_t>(column)]) {
      if (mark[static_cast<size_t>(row)] != column) {
        mark[static_cast<size_t>(row)] = column;
        rows.push_back(row);
      }
    }

    for (const int32_t child : children[static_cast<size_t>(column)]) {
      for (const int32_t row : structure[static_cast<size_t>(child)]) {
        if (mark[static_cast<size_t>(row)] != column) {
          mark[static_cast<size_t>(row)] = column;
          rows.push_back(row);
        }
      }
    }

    std::sort(rows.begin(), rows.end());

    if (!rows.empty()) {
      children[static_cast<size_t>(rows.front())].push_back(column);
    }
  }

  for (int32_t column = 0; column != columns; ++column) {
    const int32_t blocks = 1 + static_cast<int32_t>(structure[static_cast<size_t>(column)].size());
    const size_t next = static_cast<size_t>(column) + 1u;

    pattern.start[next] = pattern.start[next - 1u] + blocks;
    pattern.panel[next] = pattern.panel[next - 1u] + 36 * static_cast<int64_t>(blocks);
  }

  pattern.rows.resize(static_cast<size_t>(pattern.start.back()));
  pattern.parent.assign(static_cast<size_t>(columns), -1);

  for (int32_t column = 0; column != columns; ++column) {
    const size_t first = static_cast<size_t>(pattern.start[static_cast<size_t>(column)]);
    const std::vector<int32_t>& below = structure[static_cast<size_t>(column)];

    pattern.rows[first] = column;
    std::copy(below.begin(), below.end(), pattern.rows.begin() + static_cast<std::ptrdiff_t>(first) + 1);

    if (!below.empty()) {
      pattern.parent[static_cast<size_t>(column)] = below.front();
    }
  }

  orderNodes(pattern);

  return pattern;
}

void addBlock(const BlockPattern& pattern, std::vector<double>& values, int32_t row, int32_t column, const MatrixBlock& block) {
  if (row < column) {
    MatrixBlock flipped{};

    for (int32_t c = 0; c != 6; ++c) {
      for (int32_t r = 0; r != 6; ++r) {
        flipped[at(c, 6, r)] = block[at(r, 6, c)];
      }
    }

    addBlock(pattern, values, column, row, flipped);

    return;
  }

  const auto first = pattern.rows.begin() + pattern.start[static_cast<size_t>(column)];
  const auto last = pattern.rows.begin() + pattern.start[static_cast<size_t>(column) + 1u];
  const auto found = std::lower_bound(first, last, row);

  if (found == last || *found != row) {
    throw std::logic_error("A block pattern lacks a block its matrix needs");
  }

  const int32_t stride = 6 * static_cast<int32_t>(last - first);
  const int32_t local = 6 * static_cast<int32_t>(found - first);
  double* target = values.data() + pattern.panel[static_cast<size_t>(column)];

  for (int32_t c = 0; c != 6; ++c) {
    for (int32_t r = 0; r != 6; ++r) {
      target[at(c, stride, local + r)] += block[at(c, 6, r)];
    }
  }
}

std::optional<int32_t> factorBlocks(const BlockPattern& pattern, std::vector<double>& values, Lanes lanes) {
  std::vector<int32_t> columns(pattern.start.size() - 1u);

  for (size_t j = 0u; j != columns.size(); ++j) {
    columns[j] = static_cast<int32_t>(j);
  }

  return factorBlocks(pattern, values, columns, lanes);
}

std::optional<int32_t> factorBlocks(
  const BlockPattern& pattern,
  std::vector<double>& values,
  std::span<const int32_t> columns,
  Lanes lanes
) {
  const std::vector<int32_t> starts = startsOf(pattern, columns);
  Scratch scratch = scratchFor(pattern, lanes);

  for (size_t node = 0u; node != starts.size(); ++node) {
    if (starts[node] < 0) {
      continue;
    }

    const int32_t failed = factorNode(pattern, values, static_cast<int32_t>(node), starts[node], scratch);

    if (failed >= 0) {
      return failed;
    }
  }

  return std::nullopt;
}

bool factorBlocks(
  const BlockPattern& pattern,
  std::vector<double>& values,
  std::span<const int32_t> columns,
  const dynamics::Workers& workers,
  Lanes lanes
) {
  double total = 0.0;
  const Plan plan = planOf(pattern, columns, workers.count, total);

  if (workers.count <= 1u || total < parallelWork) {
    return !factorBlocks(pattern, values, columns, lanes).has_value();
  }

  const uint32_t threads = workers.count;
  std::vector<uint32_t> blockCounts{static_cast<uint32_t>(plan.roots.size())};
  std::vector<int32_t> sourceParts;
  std::vector<int32_t> belowParts;

  for (const int32_t node : plan.top) {
    const Node target = nodeAt(pattern, node);
    const double work = plan.nodeWork[static_cast<size_t>(node)];
    const int32_t positions = countOf(pattern, target.first) - (plan.starts[static_cast<size_t>(node)] - target.first);
    const int32_t sources = static_cast<int32_t>(std::clamp(work / stripeWork, 1.0, 2.0 * threads));
    const int32_t belows = std::clamp(target.belowCount / 8, 1, static_cast<int32_t>(threads));

    sourceParts.push_back(std::min(sources, positions));
    belowParts.push_back(std::max(1, std::min(belows, target.belowCount)));
    blockCounts.push_back(static_cast<uint32_t>(sourceParts.back()));
    blockCounts.push_back(static_cast<uint32_t>(belowParts.back()));
  }

  std::vector<Scratch> scratches(threads, scratchFor(pattern, lanes));
  std::vector<int32_t> bounds(2u * threads + 1u, 0);
  std::atomic<bool> failed{false};
  dynamics::StageRunner runner;

  runner.reset(workers, blockCounts, [&](uint32_t stage, uint32_t block, uint32_t worker) {
    Scratch& scratch = scratches[worker];

    if (stage == 0u) {
      for (int32_t member = plan.rootStart[block]; member != plan.rootStart[block + 1u]; ++member) {
        const int32_t node = plan.members[static_cast<size_t>(member)];

        if (failed.load(std::memory_order_relaxed)) {
          return;
        }

        if (factorNode(pattern, values, node, plan.starts[static_cast<size_t>(node)], scratch) >= 0) {
          failed.store(true, std::memory_order_relaxed);
          return;
        }
      }

      return;
    }

    const size_t index = (stage - 1u) / 2u;
    const int32_t node = plan.top[index];
    const Node target = nodeAt(pattern, node);
    const int32_t from = plan.starts[static_cast<size_t>(node)];

    if ((stage - 1u) % 2u == 0u) {
      mapNode(pattern, target, scratch.local);
      applySources(pattern, values, node, target, from, bounds[block], bounds[block + 1u], scratch);
    } else {
      const int32_t parts = belowParts[index];
      const int32_t begin = static_cast<int32_t>(static_cast<int64_t>(target.belowCount) * block / parts);
      const int32_t end = static_cast<int32_t>(static_cast<int64_t>(target.belowCount) * (block + 1u) / parts);

      solveBelow(pattern, values, target, from, begin, end, scratch);
    }
  });

  runner.run([&] {
    runner.execute(0u);

    for (size_t index = 0u; index != plan.top.size() && !failed.load(std::memory_order_relaxed); ++index) {
      const int32_t node = plan.top[index];
      const Node target = nodeAt(pattern, node);
      const int32_t from = plan.starts[static_cast<size_t>(node)];
      const int32_t parts = sourceParts[index];
      const int32_t low = from - target.first;
      const int32_t high = countOf(pattern, target.first);
      const int32_t width = target.last - from + 1;
      const double triangle = static_cast<double>(width) * (width + 1) / 2.0;
      const double rest = static_cast<double>(width) * (high - low - width);
      const double share = (triangle + rest) / parts;
      double sum = 0.0;
      int32_t position = low;

      bounds[0] = low;

      for (int32_t part = 1; part != parts; ++part) {
        while (position != high && sum < share * part) {
          const int32_t offset = position - low;

          sum += offset < width ? offset + 1 : width;
          ++position;
        }

        bounds[static_cast<size_t>(part)] = position;
      }

      bounds[static_cast<size_t>(parts)] = high;
      runner.execute(static_cast<uint32_t>(1u + 2u * index));

      if (factorTriangle(pattern, values, target, from, scratches[0]) >= 0) {
        failed.store(true, std::memory_order_relaxed);
        break;
      }

      runner.execute(static_cast<uint32_t>(2u + 2u * index));
    }
  });

  return !failed.load();
}

void solveBlocks(const BlockPattern& pattern, const std::vector<double>& values, std::vector<double>& x) {
  const int32_t columns = static_cast<int32_t>(pattern.start.size()) - 1;

  for (int32_t j = 0; j != columns; ++j) {
    const int32_t first = pattern.start[static_cast<size_t>(j)];
    const int32_t count = pattern.start[static_cast<size_t>(j) + 1u] - first;
    const int32_t stride = 6 * count;
    const double* column = values.data() + pattern.panel[static_cast<size_t>(j)];
    double* own = x.data() + at(j, 6, 0);

    for (int32_t q = 0; q != 6; ++q) {
      double value = own[q];

      for (int32_t s = 0; s != q; ++s) {
        value -= column[at(s, stride, q)] * own[s];
      }

      own[q] = value / column[at(q, stride, q)];
    }

    for (int32_t t = 1; t != count; ++t) {
      double* other = x.data() + at(pattern.rows[static_cast<size_t>(first + t)], 6, 0);

      for (int32_t s = 0; s != 6; ++s) {
        const double weight = own[s];
        const double* in = column + at(s, stride, 6 * t);

        for (int32_t r = 0; r != 6; ++r) {
          other[r] -= in[r] * weight;
        }
      }
    }
  }

  for (int32_t j = columns - 1; j >= 0; --j) {
    const int32_t first = pattern.start[static_cast<size_t>(j)];
    const int32_t count = pattern.start[static_cast<size_t>(j) + 1u] - first;
    const int32_t stride = 6 * count;
    const double* column = values.data() + pattern.panel[static_cast<size_t>(j)];
    double* own = x.data() + at(j, 6, 0);

    for (int32_t t = 1; t != count; ++t) {
      const double* other = x.data() + at(pattern.rows[static_cast<size_t>(first + t)], 6, 0);

      for (int32_t s = 0; s != 6; ++s) {
        const double* in = column + at(s, stride, 6 * t);
        double value = own[s];

        for (int32_t r = 0; r != 6; ++r) {
          value -= in[r] * other[r];
        }

        own[s] = value;
      }
    }

    for (int32_t q = 5; q >= 0; --q) {
      double value = own[q];

      for (int32_t i = q + 1; i != 6; ++i) {
        value -= column[at(q, stride, i)] * own[i];
      }

      own[q] = value / column[at(q, stride, q)];
    }
  }
}

bool solveBlock(MatrixBlock system, VectorBlock& x) {
  if (!factorDiagonal(system.data(), 6)) {
    return false;
  }

  for (int32_t q = 0; q != 6; ++q) {
    double value = x[static_cast<size_t>(q)];

    for (int32_t s = 0; s != q; ++s) {
      value -= system[at(s, 6, q)] * x[static_cast<size_t>(s)];
    }

    x[static_cast<size_t>(q)] = value / system[at(q, 6, q)];
  }

  for (int32_t q = 5; q >= 0; --q) {
    double value = x[static_cast<size_t>(q)];

    for (int32_t i = q + 1; i != 6; ++i) {
      value -= system[at(q, 6, i)] * x[static_cast<size_t>(i)];
    }

    x[static_cast<size_t>(q)] = value / system[at(q, 6, q)];
  }

  return true;
}

}
