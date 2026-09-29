#include "host.ih"

namespace talus::script {

namespace {

hostfxr_initialize_for_runtime_config_fn g_initialize = nullptr;
hostfxr_get_runtime_delegate_fn g_getDelegate = nullptr;
hostfxr_close_fn g_close = nullptr;

std::wstring widen(std::string_view text) {
  if (text.empty()) {
    return {};
  }

  const int needed = MultiByteToWideChar(
    CP_UTF8,
    0,
    text.data(),
    static_cast<int>(text.size()),
    nullptr,
    0
  );

  std::wstring wide(static_cast<size_t>(needed), L'\0');

  MultiByteToWideChar(
    CP_UTF8,
    0,
    text.data(),
    static_cast<int>(text.size()),
    wide.data(),
    needed
  );

  return wide;
}

bool loadHostfxr(std::string& error) {
  if (g_initialize != nullptr) {
    return true;
  }

  char_t path[1024];
  size_t length = sizeof(path) / sizeof(char_t);

  if (get_hostfxr_path(path, &length, nullptr) != 0) {
    error = "The .NET hosting library could not be located";
    return false;
  }

  const HMODULE library = LoadLibraryW(path);

  if (library == nullptr) {
    error = "The .NET hosting library could not be loaded";
    return false;
  }

  g_initialize = reinterpret_cast<hostfxr_initialize_for_runtime_config_fn>(
    reinterpret_cast<void*>(
      GetProcAddress(library, "hostfxr_initialize_for_runtime_config")
    )
  );

  g_getDelegate =
    reinterpret_cast<hostfxr_get_runtime_delegate_fn>(reinterpret_cast<void*>(
      GetProcAddress(library, "hostfxr_get_runtime_delegate")
    ));

  g_close = reinterpret_cast<hostfxr_close_fn>(
    reinterpret_cast<void*>(GetProcAddress(library, "hostfxr_close"))
  );

  if (
    g_initialize == nullptr || g_getDelegate == nullptr || g_close == nullptr
  ) {
    error = "The .NET hosting library is missing an entry point";
    return false;
  }

  return true;
}

}

Host::~Host() {
  if (m_context != nullptr && g_close != nullptr) {
    g_close(static_cast<hostfxr_handle>(m_context));
  }
}

bool Host::start(const std::filesystem::path& runtimeConfig) {
  if (m_loadAssembly != nullptr) {
    return true;
  }

  if (!std::filesystem::exists(runtimeConfig)) {
    m_lastError = "No runtime configuration at " + runtimeConfig.string();
    return false;
  }

  if (!loadHostfxr(m_lastError)) {
    return false;
  }

  hostfxr_handle context = nullptr;

  const int32_t started =
    g_initialize(runtimeConfig.c_str(), nullptr, &context);

  if ((started != 0 && started != 1 && started != 2) || context == nullptr) {
    m_lastError = "The .NET runtime refused " + runtimeConfig.string();

    if (context != nullptr) {
      g_close(context);
    }

    return false;
  }

  void* loader = nullptr;

  if (
    g_getDelegate(context, hdt_load_assembly_and_get_function_pointer, &loader)
      != 0
    || loader == nullptr
  ) {
    m_lastError = "The .NET runtime gave no assembly loader";
    g_close(context);

    return false;
  }

  m_context = context;
  m_loadAssembly = loader;

  return true;
}

void* Host::entryPoint(
  const std::filesystem::path& assembly,
  std::string_view type,
  std::string_view method
) {
  if (m_loadAssembly == nullptr) {
    m_lastError = "The script host is not running";
    return nullptr;
  }

  if (!std::filesystem::exists(assembly)) {
    m_lastError = "No script assembly at " + assembly.string();
    return nullptr;
  }

  const std::wstring wideType = widen(type);
  const std::wstring wideMethod = widen(method);

  void* found = nullptr;

  const auto load =
    reinterpret_cast<load_assembly_and_get_function_pointer_fn>(m_loadAssembly);

  if (
    load(
      assembly.c_str(),
      wideType.c_str(),
      wideMethod.c_str(),
      UNMANAGEDCALLERSONLY_METHOD,
      nullptr,
      &found
    )
    != 0
  ) {
    m_lastError = "The script assembly has no entry point " + std::string(type)
                  + "." + std::string(method);

    return nullptr;
  }

  return found;
}

bool Host::running() const noexcept {
  return m_loadAssembly != nullptr;
}

const std::string& Host::lastError() const noexcept {
  return m_lastError;
}

}
