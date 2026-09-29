#pragma once

#include <talus/objectmodel/trait/transient/transient.h>

#include <talus/field/sampledsdf/sampledsdf.h>

#include <memory>

namespace talus::objectmodel {

struct SharedSdf {
  std::shared_ptr<talus::field::SampledSdf> field;
};

template<>
inline constexpr bool transient<SharedSdf> = true;

}
