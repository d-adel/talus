#include "joltlibrary.ih"

namespace {

void trace(const char* format, ...) {
  va_list args;
  va_start(args, format);
  std::vprintf(format, args);
  va_end(args);

  std::putchar('\n');
  std::fflush(stdout);
}

struct Counted {
  std::atomic<int64_t> live{0};
  std::atomic<int64_t> peak{0};
};

Counted& counted() {
  static Counted memory;

  return memory;
}

void account(int64_t bytes) {
  const int64_t now =
    counted().live.fetch_add(bytes, std::memory_order_relaxed) + bytes;

  int64_t seen = counted().peak.load(std::memory_order_relaxed);

  while (now > seen
         && !counted().peak.compare_exchange_weak(
           seen,
           now,
           std::memory_order_relaxed
         )) {
  }
}

constexpr size_t HEADER = 16u;

void* joltAllocate(size_t size) {
  auto* base = static_cast<uint8_t*>(_malloc_base(size + HEADER));

  if (base == nullptr) {
    return nullptr;
  }

  std::memcpy(base, &size, sizeof(size));
  account(static_cast<int64_t>(size));

  return base + HEADER;
}

void joltFree(void* block) {
  if (block == nullptr) {
    return;
  }

  uint8_t* base = static_cast<uint8_t*>(block) - HEADER;

  size_t size = 0u;
  std::memcpy(&size, base, sizeof(size));
  account(-static_cast<int64_t>(size));

  _free_base(base);
}

void* joltReallocate(void* block, size_t oldSize, size_t newSize) {
  void* moved = joltAllocate(newSize);

  if (moved == nullptr) {
    return nullptr;
  }

  if (block != nullptr) {
    std::memcpy(moved, block, std::min(oldSize, newSize));
  }

  joltFree(block);

  return moved;
}

void* joltAlignedAllocate(size_t size, size_t alignment) {
  const size_t align = std::max(alignment, HEADER);
  auto* raw = static_cast<uint8_t*>(_malloc_base(size + align + HEADER));

  if (raw == nullptr) {
    return nullptr;
  }

  const uintptr_t first = reinterpret_cast<uintptr_t>(raw) + HEADER;
  auto* block =
    reinterpret_cast<uint8_t*>((first + align - 1u) & ~(uintptr_t{align} - 1u));

  std::memcpy(block - HEADER, &size, sizeof(size));
  std::memcpy(block - sizeof(raw), &raw, sizeof(raw));
  account(static_cast<int64_t>(size));

  return block;
}

void joltAlignedFree(void* block) {
  if (block == nullptr) {
    return;
  }

  auto* aligned = static_cast<uint8_t*>(block);

  size_t size = 0u;
  uint8_t* raw = nullptr;
  std::memcpy(&size, aligned - HEADER, sizeof(size));
  std::memcpy(&raw, aligned - sizeof(raw), sizeof(raw));
  account(-static_cast<int64_t>(size));

  _free_base(raw);
}

#ifdef JPH_ENABLE_ASSERTS

bool assertFailed(
  const char* expression,
  const char* message,
  const char* file,
  JPH::uint line
) {
  std::fprintf(
    stderr,
    "%s:%u: (%s) %s\n",
    file,
    line,
    expression,
    message != nullptr ? message : ""
  );

  return true;
}

#endif

}

namespace talus::physics {

JoltMemory joltMemory() noexcept {
  return JoltMemory{
    counted().live.load(std::memory_order_relaxed),
    counted().peak.load(std::memory_order_relaxed)
  };
}

JoltLibrary::JoltLibrary() {
  JPH::Allocate = joltAllocate;
  JPH::Reallocate = joltReallocate;
  JPH::Free = joltFree;
  JPH::AlignedAllocate = joltAlignedAllocate;
  JPH::AlignedFree = joltAlignedFree;

  JPH::Trace = trace;

#ifdef JPH_ENABLE_ASSERTS
  JPH::AssertFailed = assertFailed;
#endif

  JPH::Factory::sInstance = new JPH::Factory();

  JPH::RegisterTypes();

  registerFieldShape();
}

JoltLibrary::~JoltLibrary() {
  JPH::UnregisterTypes();

  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;

  JPH::Trace = nullptr;

#ifdef JPH_ENABLE_ASSERTS
  JPH::AssertFailed = nullptr;
#endif
}

}
