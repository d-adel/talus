#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace talus::serialize {

[[nodiscard]] bool writeText(
  const std::filesystem::path& file,
  std::string_view text
);

[[nodiscard]] std::optional<std::string> readText(
  const std::filesystem::path& file
);

}
