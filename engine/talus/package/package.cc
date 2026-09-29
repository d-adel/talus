#include "package.ih"

namespace talus::package {

namespace {

const std::filesystem::copy_options REPLACE =
  std::filesystem::copy_options::overwrite_existing
  | std::filesystem::copy_options::recursive;

std::string projectName(const std::filesystem::path& root) {
  const std::filesystem::path trimmed =
    root.filename().empty() ? root.parent_path() : root;

  return trimmed.filename().string();
}

bool copyTree(
  const std::filesystem::path& from,
  const std::filesystem::path& to,
  std::string& failure
) {
  std::error_code ec;

  std::filesystem::create_directories(to, ec);
  std::filesystem::copy(from, to, REPLACE, ec);

  if (ec) {
    failure = "could not copy " + from.string() + ": " + ec.message();

    return false;
  }

  return true;
}

bool copyOne(
  const std::filesystem::path& from,
  const std::filesystem::path& to,
  std::string& failure
) {
  std::error_code ec;

  std::filesystem::create_directories(to.parent_path(), ec);

  std::filesystem::copy_file(
    from,
    to,
    std::filesystem::copy_options::overwrite_existing,
    ec
  );

  if (ec) {
    failure = "could not copy " + from.string() + ": " + ec.message();

    return false;
  }

  return true;
}

bool copyProjectWithoutWorlds(
  const std::filesystem::path& from,
  const std::filesystem::path& to,
  std::string& failure
) {
  std::error_code ec;

  std::filesystem::create_directories(to, ec);

  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(from, ec)) {
    if (entry.path().filename() == "worlds") {
      continue;
    }

    const std::filesystem::path target = to / entry.path().filename();

    const bool copied = entry.is_directory()
                          ? copyTree(entry.path(), target, failure)
                          : copyOne(entry.path(), target, failure);

    if (!copied) {
      return false;
    }
  }

  return true;
}

}

std::filesystem::path packagedExecutable(const PackageSpec& spec) {
  return spec.output
         / (projectName(spec.projectRoot)
            + spec.playerExecutable.extension().string());
}

bool packageProject(const PackageSpec& spec, std::string& failure) {
  std::error_code ec;

  if (!std::filesystem::exists(spec.playerExecutable)) {
    failure = "no player executable at " + spec.playerExecutable.string();

    return false;
  }

  if (!std::filesystem::exists(spec.projectRoot)) {
    failure = "no project at " + spec.projectRoot.string();

    return false;
  }

  if (spec.worlds.empty()) {
    failure = "a package needs at least one world, and the first one starts it";

    return false;
  }

  const std::filesystem::path inside =
    std::filesystem::weakly_canonical(spec.projectRoot);
  const std::filesystem::path target =
    std::filesystem::weakly_canonical(spec.output);

  if (target.string().starts_with(inside.string())) {
    failure = "the package output cannot sit inside the project it packages";

    return false;
  }

  const std::filesystem::path staging = spec.playerExecutable.parent_path();
  const std::string name = projectName(spec.projectRoot);
  const std::filesystem::path packaged = spec.output / "projects" / name;

  std::filesystem::create_directories(spec.output, ec);

  if (ec) {
    failure = "could not create " + spec.output.string() + ": " + ec.message();

    return false;
  }

  if (!copyOne(spec.playerExecutable, packagedExecutable(spec), failure)) {
    return false;
  }

  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(staging, ec)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".dll") {
      continue;
    }

    if (!copyOne(entry.path(), spec.output / entry.path().filename(), failure)) {
      return false;
    }
  }

  if (!copyTree(staging / "shaders", spec.output / "shaders", failure)) {
    return false;
  }

  if (std::filesystem::exists(staging / "managed")) {
    if (!copyTree(staging / "managed", spec.output / "managed", failure)) {
      return false;
    }
  }

  if (!copyProjectWithoutWorlds(spec.projectRoot, packaged, failure)) {
    return false;
  }

  for (const std::string& world : spec.worlds) {
    const std::string file = world + ".world.json";

    const std::filesystem::path source = spec.projectRoot / "worlds" / file;

    if (!std::filesystem::exists(source)) {
      failure = "the package lists a world that is not in the project: " + world;

      return false;
    }

    if (!copyOne(source, packaged / "worlds" / file, failure)) {
      return false;
    }
  }

  std::ofstream manifest(spec.output / "game.json");

  if (!manifest) {
    failure = "could not write the game manifest";

    return false;
  }

  manifest << "{\n  \"project\": \"projects/" << name << "\",\n  \"worlds\": [";

  for (size_t at = 0; at != spec.worlds.size(); ++at) {
    manifest << (at == 0 ? "\"" : ", \"") << spec.worlds[at] << "\"";
  }

  manifest << "]\n}\n";

  return manifest.good();
}

}
