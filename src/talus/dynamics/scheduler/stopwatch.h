#pragma once

#include <chrono>

namespace talus::dynamics {

class Stopwatch {
public:
  double lap() {
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double, std::milli>(now - m_last).count();

    m_last = now;

    return elapsed;
  }

  [[nodiscard]] double total() const {
    return std::chrono::duration<double, std::milli>(m_last - m_start).count();
  }

private:
  std::chrono::steady_clock::time_point m_start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point m_last = m_start;
};

}
