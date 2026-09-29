#include "schedule.ih"

namespace talus::systems {

std::optional<Phase> phaseFrom(uint32_t value) noexcept {
  if (value >= PHASES.size()) {
    return std::nullopt;
  }

  return static_cast<Phase>(value);
}

Entry* Schedule::find(std::string_view name) noexcept {
  const auto found = std::ranges::find(m_systems, name, &Entry::name);

  return found == m_systems.end() ? nullptr : &*found;
}

const Entry* Schedule::find(std::string_view name) const noexcept {
  const auto found = std::ranges::find(m_systems, name, &Entry::name);

  return found == m_systems.end() ? nullptr : &*found;
}

bool Schedule::add(
  Phase phase,
  int32_t order,
  std::string name,
  Function function
) {
  if (name.empty() || function == nullptr || find(name) != nullptr) {
    return false;
  }

  const auto key = [](const Entry& entry) {
    return std::pair{entry.phase, entry.order};
  };

  const auto at =
    std::ranges::upper_bound(m_systems, std::pair{phase, order}, {}, key);

  m_systems.insert(
    at,
    Entry{phase, order, std::move(name), std::move(function)}
  );

  return true;
}

bool Schedule::remove(std::string_view name) {
  const auto found = std::ranges::find(m_systems, name, &Entry::name);

  if (found == m_systems.end()) {
    return false;
  }

  m_systems.erase(found);

  return true;
}

bool Schedule::enable(std::string_view name, bool wanted) {
  Entry* entry = find(name);

  if (entry == nullptr) {
    return false;
  }

  entry->enabled = wanted;

  return true;
}

bool Schedule::enabled(std::string_view name) const noexcept {
  const Entry* entry = find(name);

  return entry != nullptr && entry->enabled;
}

void Schedule::clear() noexcept {
  m_systems.clear();
  m_lastMilliseconds = 0.0;
}

void Schedule::run(Phase phase, Context& context) {
  using Clock = std::chrono::steady_clock;
  using Milliseconds = std::chrono::duration<double, std::milli>;

  for (Entry& entry : m_systems) {
    if (entry.phase != phase) {
      continue;
    }

    if (!entry.enabled) {
      entry.lastMilliseconds = 0.0;
      continue;
    }

    const Clock::time_point began = Clock::now();

    entry.function(context);

    entry.lastMilliseconds = Milliseconds(Clock::now() - began).count();
  }

  m_lastMilliseconds = 0.0;

  for (const Entry& entry : m_systems) {
    m_lastMilliseconds += entry.lastMilliseconds;
  }
}

void Schedule::run(Context& context) {
  for (const Phase phase : PHASES) {
    run(phase, context);
  }
}

std::span<const Entry> Schedule::systems() const noexcept {
  return m_systems;
}

size_t Schedule::size() const noexcept {
  return m_systems.size();
}

double Schedule::lastMilliseconds() const noexcept {
  return m_lastMilliseconds;
}

double Schedule::lastMilliseconds(std::string_view name) const noexcept {
  const Entry* entry = find(name);

  return entry == nullptr ? 0.0 : entry->lastMilliseconds;
}

}
