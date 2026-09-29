#include "startupoptions.h"

#include <talus/serialize/gamemanifest/gamemanifest.h>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {

std::string valueFor(
  std::span<char*> arguments,
  size_t& at,
  const std::string& flag
) {
  if (at + 1 >= arguments.size()) {
    throw std::runtime_error(flag + " needs a value");
  }

  return arguments[++at];
}

}

StartupOptions::StartupOptions(std::span<char*> arguments) {
  if (!arguments.empty()) {
    const std::optional<talus::serialize::GameManifest> game =
      talus::serialize::readGameManifest(
        std::filesystem::path(arguments[0]).parent_path()
      );

    if (game.has_value()) {
      project = game->project;
      world = game->worlds.front();
      packaged = true;
    }
  }

  for (size_t i = 1; i < arguments.size(); ++i) {
    const std::string argument = arguments[i];

    if (argument == "--") {
      break;
    }

    if (argument == "--project") {
      project = valueFor(arguments, i, argument);
    } else if (argument == "--world") {
      world = valueFor(arguments, i, argument);
    } else if (argument == "--frames") {
      frames =
        static_cast<uint32_t>(std::stoul(valueFor(arguments, i, argument)));
    } else if (argument == "--fixed-step") {
      fixedStep = std::stof(valueFor(arguments, i, argument));

      if (!(fixedStep > 0.0f)) {
        throw std::runtime_error("--fixed-step needs a positive number");
      }
    }
#if TALUS_TOOLS
    else if (argument == "--analytic") {
      analytic = true;
    } else if (argument == "--verify-bake") {
      verifyBake = true;
    } else if (argument == "--screenshot") {
      screenshot = valueFor(arguments, i, argument);
    } else if (argument == "--golden") {
      golden = valueFor(arguments, i, argument);
    } else if (argument == "--capture") {
      capture = valueFor(arguments, i, argument);
    } else if (argument == "--benchmark") {
      benchmark = valueFor(arguments, i, argument);
    } else if (argument == "--warmup") {
      warmup =
        static_cast<uint32_t>(std::stoul(valueFor(arguments, i, argument)));
    }
#endif
    else {
      throw std::runtime_error(
        "Unknown argument: " + argument + "\nUsage: talus"
#if TALUS_TOOLS
        " [--analytic] [--verify-bake] [--screenshot PATH]"
        " [--golden PATH] [--capture DIR] [--benchmark PATH] [--warmup N]"
#endif
        " [--project DIR] [--world NAME] [--frames N] [--fixed-step SECONDS]"
        " [-- ARGS FOR THE SCRIPTS]"
      );
    }
  }
}
