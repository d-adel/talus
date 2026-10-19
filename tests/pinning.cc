#include "pinning.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace testthreads {

bool pinToCore(std::jthread& thread, uint32_t core) {
#if defined(_WIN32)
  return SetThreadAffinityMask(thread.native_handle(), static_cast<DWORD_PTR>(1) << (core % 64u)) != 0u;
#else
  cpu_set_t set;

  CPU_ZERO(&set);
  CPU_SET(core, &set);

  return pthread_setaffinity_np(thread.native_handle(), sizeof(set), &set) == 0;
#endif
}

}
