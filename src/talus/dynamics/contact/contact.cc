#include "contact.h"

namespace talus::dynamics {

ContactMatch::ContactMatch(std::span<const Contact> previous) {
  m_index.reserve(previous.size());

  for (size_t index = 0u; index != previous.size(); ++index) {
    const auto [slot, fresh] =
      m_index.try_emplace(previous[index].key, static_cast<int32_t>(index));

    if (!fresh) {
      slot->second = -1;
    }
  }
}

int32_t ContactMatch::find(uint64_t key) const {
  const auto found = m_index.find(key);

  return found == m_index.end() ? -1 : found->second;
}

void carryMemory(std::span<Contact> contacts, std::span<const Contact> previous) {
  const ContactMatch match(previous);

  for (Contact& contact : contacts) {
    const int32_t index = match.find(contact.key);

    contact.memory =
      index < 0 ? ContactMemory{} : previous[static_cast<size_t>(index)].memory;
  }
}

}
