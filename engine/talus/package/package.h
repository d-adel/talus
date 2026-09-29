#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace talus::package {

struct PackageSpec {
  std::filesystem::path playerExecutable;
  std::filesystem::path projectRoot;
  std::filesystem::path output;

  std::vector<std::string> worlds;
};

[[nodiscard]] std::filesystem::path packagedExecutable(const PackageSpec& spec);

[[nodiscard]] bool packageProject(const PackageSpec& spec, std::string& failure);

}
