#include "impact.ih"

namespace talus::objectmodel {

void sortStrongestFirst(std::span<Impact> impacts) {
  std::ranges::sort(impacts, [](const Impact& a, const Impact& b) {
    if (a.impulse != b.impulse) {
      return a.impulse > b.impulse;
    }

    if (a.entity != b.entity) {
      return static_cast<uint32_t>(a.entity) < static_cast<uint32_t>(b.entity);
    }

    return static_cast<uint32_t>(a.other) < static_cast<uint32_t>(b.other);
  });
}

}
