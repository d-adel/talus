#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace talus::script {

class Host {
public:
  Host() = default;
  ~Host();

  Host(const Host&) = delete;
  Host& operator=(const Host&) = delete;
  Host(Host&&) = delete;
  Host& operator=(Host&&) = delete;

  [[nodiscard]] bool start(const std::filesystem::path& runtimeConfig);

  [[nodiscard]] void* entryPoint(
    const std::filesystem::path& assembly,
    std::string_view type,
    std::string_view method
  );

  [[nodiscard]] bool running() const noexcept;

  [[nodiscard]] const std::string& lastError() const noexcept;

private:
  void* m_context = nullptr;
  void* m_loadAssembly = nullptr;

  std::string m_lastError;
};

}
