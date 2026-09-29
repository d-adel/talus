#pragma once

#include <cstdint>
#include <span>
#include <string>

struct StartupOptions {
  StartupOptions() = default;
  explicit StartupOptions(std::span<char*> arguments);

  std::string capture;

  bool analytic = false;

  bool verifyBake = false;

  std::string screenshot;

  std::string golden;

  std::string benchmark;

  uint32_t warmup = 30u;

  std::string project = "projects/demo";

  std::string world = "main";

  bool packaged = false;
  uint32_t frames = 0u;
  float fixedStep = 0.0f;
};
