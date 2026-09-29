#pragma once

#include <talus/objectmodel/world/world.h>

namespace talus::objectmodel {

void validateBodies(const World& source);

void instantiateWorld(const World& source, World& destination);

}
