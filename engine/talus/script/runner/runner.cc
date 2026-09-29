#include "runner.ih"

namespace talus::script {

namespace {

inline constexpr const char* ASSEMBLY = "Talus.Engine";

}

bool Runner::load(const std::filesystem::path& managed) {
  m_managed = managed;

  return m_host.start(
    m_managed / (std::string(ASSEMBLY) + ".runtimeconfig.json")
  );
}

bool Runner::initialize(
  talus::systems::Schedule& schedule,
  talus::objectmodel::World& world,
  const std::filesystem::path& scripts
) {
  if (!m_host.running()) {
    return false;
  }

  void* found = m_host.entryPoint(
    m_managed / (std::string(ASSEMBLY) + ".dll"),
    std::string(ASSEMBLY) + ".Boot, " + ASSEMBLY,
    "Initialize"
  );

  if (found == nullptr) {
    return false;
  }

  using Entry = int (*)(const Api*, void*, void*, const char*);

  const std::string directory = scripts.string();

  m_initialized =
    reinterpret_cast<Entry>(found)(&api(), &schedule, &world, directory.c_str())
    == 1;

  return m_initialized;
}

bool Runner::running() const noexcept {
  return m_initialized;
}

const std::string& Runner::lastError() const noexcept {
  return m_host.lastError();
}

}
