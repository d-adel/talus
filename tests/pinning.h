#pragma once

#include <cstdint>
#include <thread>

namespace testthreads {

[[nodiscard]] bool pinToCore(std::jthread& thread, uint32_t core);

}
