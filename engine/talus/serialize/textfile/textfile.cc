#include "textfile.ih"

namespace talus::serialize {

bool writeText(const std::filesystem::path& file, std::string_view text) {
  std::ofstream out(file, std::ios::binary | std::ios::trunc);

  if (!out) {
    return false;
  }

  out.write(text.data(), static_cast<std::streamsize>(text.size()));

  return out.good();
}

std::optional<std::string> readText(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);

  if (!in) {
    return std::nullopt;
  }

  std::ostringstream buffer;
  buffer << in.rdbuf();

  return buffer.str();
}

}
