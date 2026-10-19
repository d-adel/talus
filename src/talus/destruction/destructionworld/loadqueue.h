#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace talus::destruction {

enum class Enqueued : uint8_t { Dropped, Replaced, Joined, Merged };

template <typename Case, typename Difference, typename Merge>
Enqueued enqueueCase(std::vector<Case>& cases, Case next, bool nearNewest, size_t capacity, Difference difference, Merge merge, size_t protectedCases = 0u) {
  if (nearNewest) {
    if (cases.size() == protectedCases) {
      return Enqueued::Dropped;
    }

    merge(cases.back(), next);
    cases.back() = std::move(next);

    return Enqueued::Replaced;
  }

  cases.push_back(std::move(next));

  if (cases.size() <= capacity + protectedCases) {
    return Enqueued::Joined;
  }

  size_t least = protectedCases;
  double smallest = difference(cases[least], cases[least + 1u]);

  for (size_t at = least + 1u; at + 1u < cases.size(); ++at) {
    const double gap = difference(cases[at], cases[at + 1u]);

    if (gap < smallest) {
      smallest = gap;
      least = at;
    }
  }

  merge(cases[least], cases[least + 1u]);
  cases.erase(cases.begin() + static_cast<std::ptrdiff_t>(least));

  return Enqueued::Merged;
}

template <typename Case, typename Difference>
Enqueued enqueueCase(std::vector<Case>& cases, Case next, bool nearNewest, size_t capacity, Difference difference) {
  return enqueueCase(cases, std::move(next), nearNewest, capacity, difference, [](const Case&, Case&) {});
}

}
