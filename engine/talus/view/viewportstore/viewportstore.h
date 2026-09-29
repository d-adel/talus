#pragma once

#include <talus/view/viewport/viewport.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace talus::view {

class ViewportStore {
public:
  ViewportStore() = default;

  ViewportStore(const ViewportStore&) = delete;
  ViewportStore& operator=(const ViewportStore&) = delete;

  [[nodiscard]] ViewportId create();

  [[nodiscard]] bool destroy(ViewportId id);

  [[nodiscard]] bool configure(ViewportId id, const Viewport& config);

  [[nodiscard]] const Viewport* find(ViewportId id) const;

  [[nodiscard]] std::size_t count() const noexcept;

  void clear() noexcept;

private:
  using Entry = std::pair<ViewportId, Viewport>;

  [[nodiscard]] std::vector<Entry>::const_iterator locate(ViewportId id) const;

  [[nodiscard]] std::vector<Entry>::iterator locate(ViewportId id);

  std::vector<Entry> m_entries;

  ViewportId m_next = 0;
};

}
