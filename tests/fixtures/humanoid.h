#pragma once

#include <talus/dynamics/pipeline/pipeline.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace fixture {

inline constexpr std::array<const char*, 19> humanoidJointNames{
  "spineLower",
  "spineMiddle",
  "spineUpper",
  "neckLower",
  "neckUpper",
  "leftClavicle",
  "leftShoulder",
  "leftElbow",
  "leftWrist",
  "rightClavicle",
  "rightShoulder",
  "rightElbow",
  "rightWrist",
  "leftHip",
  "leftKnee",
  "leftAnkle",
  "rightHip",
  "rightKnee",
  "rightAnkle"
};

struct Humanoid {
  std::vector<uint32_t> bodies;
  std::vector<uint32_t> joints;
};

Humanoid addHumanoid(
  talus::dynamics::World& world,
  const glm::dvec3& pelvis,
  bool pinPelvis
);

void excludeSelfContact(
  talus::dynamics::World& world,
  const Humanoid& humanoid
);

}
