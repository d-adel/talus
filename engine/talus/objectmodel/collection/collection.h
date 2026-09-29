#pragma once

#include <talus/objectmodel/entity/entity.h>

#include <string>
#include <vector>

namespace talus::objectmodel {

struct Collection {
  std::string name;
  std::vector<Entity> members;
};

}
