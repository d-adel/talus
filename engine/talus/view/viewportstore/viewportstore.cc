#include "viewportstore.ih"

namespace talus::view {

ViewportId ViewportStore::create() {
  const ViewportId id = m_next++;

  m_entries.emplace_back(id, Viewport{});

  return id;
}

bool ViewportStore::destroy(ViewportId id) {
  const auto entry = locate(id);

  if (entry == m_entries.end()) {
    return false;
  }

  m_entries.erase(entry);

  return true;
}

bool ViewportStore::configure(ViewportId id, const Viewport& config) {
  const auto entry = locate(id);

  if (entry == m_entries.end()) {
    return false;
  }

  entry->second = config;

  return true;
}

const Viewport* ViewportStore::find(ViewportId id) const {
  const auto entry = locate(id);

  return entry == m_entries.end() ? nullptr : &entry->second;
}

std::size_t ViewportStore::count() const noexcept {
  return m_entries.size();
}

void ViewportStore::clear() noexcept {
  m_entries.clear();
}

std::vector<ViewportStore::Entry>::const_iterator ViewportStore::locate(
  ViewportId id
) const {
  return std::find_if(
    m_entries.begin(),
    m_entries.end(),
    [id](const Entry& entry) { return entry.first == id; }
  );
}

std::vector<ViewportStore::Entry>::iterator ViewportStore::locate(
  ViewportId id
) {
  return std::find_if(
    m_entries.begin(),
    m_entries.end(),
    [id](const Entry& entry) { return entry.first == id; }
  );
}

}
