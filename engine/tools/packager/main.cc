#include <talus/package/package.h>

#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  try {
    const std::span<char*> arguments(argv, argc);

    if (arguments.size() < 5) {
      std::cerr << "Usage: TalusPackager PLAYER PROJECT OUTPUT WORLD...\n";

      return 1;
    }

    talus::package::PackageSpec spec{};
    spec.playerExecutable = arguments[1];
    spec.projectRoot = arguments[2];
    spec.output = arguments[3];

    for (size_t at = 4; at < arguments.size(); ++at) {
      spec.worlds.emplace_back(arguments[at]);
    }

    std::string failure;

    if (!talus::package::packageProject(spec, failure)) {
      std::cerr << failure << '\n';

      return 1;
    }

    std::cout << talus::package::packagedExecutable(spec).string() << '\n';

    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';

    return 1;
  }
}
