#include "telemetry.ih"

namespace talus::core {

Telemetry::Key Telemetry::find(std::string_view name) const {
  return m_names.find(name);
}

Telemetry::Key Telemetry::key(std::string_view name) {
  const Key interned = m_names.intern(name);

  if (interned >= m_values.size()) {
    m_values.resize(m_names.size(), 0.0);
  }

  return interned;
}

void Telemetry::add(Key key, double amount) {
  if (key < m_values.size()) {
    m_values[key] += amount;
  }
}

void Telemetry::set(Key key, double amount) {
  if (key < m_values.size()) {
    m_values[key] = amount;
  }
}

void Telemetry::max(Key key, double amount) {
  if (key < m_values.size()) {
    m_values[key] = std::max(m_values[key], amount);
  }
}

double Telemetry::get(Key key) const {
  return key < m_values.size() ? m_values[key] : 0.0;
}

void Telemetry::clear() {
  std::fill(m_values.begin(), m_values.end(), 0.0);
}

std::span<const double> Telemetry::values() const noexcept {
  return m_values;
}

std::span<const std::string> Telemetry::names() const noexcept {
  return m_names.names();
}

size_t Telemetry::size() const noexcept {
  return m_values.size();
}

}
