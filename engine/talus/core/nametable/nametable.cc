#include "nametable.ih"

namespace talus::core {

NameTable::Index NameTable::find(std::string_view name) const {
  for (size_t i = 0; i != m_names.size(); ++i) {
    if (m_names[i] == name) {
      return static_cast<Index>(i);
    }
  }

  return INVALID;
}

NameTable::Index NameTable::intern(std::string_view name) {
  const Index existing = find(name);

  if (existing != INVALID) {
    return existing;
  }

  m_names.emplace_back(name);

  return static_cast<Index>(m_names.size() - 1);
}

std::string_view NameTable::name(Index index) const {
  return index < m_names.size() ? std::string_view(m_names[index])
                                : std::string_view();
}

size_t NameTable::size() const noexcept {
  return m_names.size();
}

std::span<const std::string> NameTable::names() const noexcept {
  return m_names;
}

}
