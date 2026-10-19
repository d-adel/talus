#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>

namespace talus::dynamics {

[[nodiscard]] double speedOf(const BodySet& bodies, const BodyRef& ref);

}
