#pragma once

#include <talus/objectmodel/world/world.h>
#include <talus/script/host/host.h>
#include <talus/systems/schedule/schedule.h>

#include <filesystem>
#include <string>

namespace talus::script {

class Runner {
public:
  Runner() = default;
  ~Runner() = default;

  Runner(const Runner&) = delete;
  Runner& operator=(const Runner&) = delete;
  Runner(Runner&&) = delete;
  Runner& operator=(Runner&&) = delete;

  [[nodiscard]] bool load(const std::filesystem::path& managed);

  [[nodiscard]] bool initialize(
    talus::systems::Schedule& schedule,
    talus::objectmodel::World& world,
    const std::filesystem::path& scripts
  );

  [[nodiscard]] bool running() const noexcept;

  [[nodiscard]] const std::string& lastError() const noexcept;

private:
  Host m_host;

  std::filesystem::path m_managed;

  bool m_initialized = false;
};

}
