#pragma once

#include <Jolt/Jolt.h>

#include <cstdint>

namespace talus::physics {

struct JoltMemory {
  int64_t liveBytes = 0;
  int64_t peakBytes = 0;
};

[[nodiscard]] JoltMemory joltMemory() noexcept;

class JoltLibrary {
public:
  JoltLibrary();
  ~JoltLibrary();

  JoltLibrary(const JoltLibrary&) = delete;
  JoltLibrary& operator=(const JoltLibrary&) = delete;
  JoltLibrary(JoltLibrary&&) = delete;
  JoltLibrary& operator=(JoltLibrary&&) = delete;
};

}
