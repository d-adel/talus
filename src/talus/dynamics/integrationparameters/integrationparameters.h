#pragma once

#include <glm/glm.hpp>

#include <cstdint>

namespace talus::dynamics {

struct IntegrationParameters {
  double dt = 1.0 / 60.0;
  glm::dvec3 gravity{0.0, -9.81, 0.0};
  double margin = 0.0;
  double restitutionThreshold = 1.0;
  uint32_t restitutionIterations = 2u;
  uint32_t softstepSubsteps = 4u;
  double contactHertz = 30.0;
  double contactDampingRatio = 10.0;
  double contactSpeed = 3.0;
  double maxLinearSpeed = 400.0;
  double speculativeDistance = 0.02;
  double promoteDent = 0.03;
  double demoteDent = 0.015;
  uint32_t switchFrames = 3u;
  double screenLoadChange = 0.10;
  bool sleep = true;
  double sleepThreshold = 0.05;
  double timeToSleep = 0.5;
  double hitEventThreshold = 1.0;
  double thawSpeed = 0.3;
  bool continuous = true;
  bool timePairs = false;
};

}
