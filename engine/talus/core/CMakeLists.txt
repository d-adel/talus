add_library(talus_core STATIC
    event/event.cc
    asset/asset.cc
    nametable/nametable.cc
    telemetry/telemetry.cc
)
add_library(talus::core ALIAS talus_core)
target_include_directories(talus_core PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_core PUBLIC cxx_std_23)
target_compile_definitions(talus_core PUBLIC
    TALUS_MAX_SDF_PRIMITIVES=${TALUS_MAX_SDF_PRIMITIVES}
    TALUS_MAX_TEXTURES=${TALUS_MAX_TEXTURES}
    TALUS_MAX_SAMPLERS=${TALUS_MAX_SAMPLERS}
    TALUS_MAX_MATERIALS=${TALUS_MAX_MATERIALS}
    TALUS_MAX_SDF_VOLUMES=${TALUS_MAX_SDF_VOLUMES}
    TALUS_MAX_PRIMITIVE_POOL=${TALUS_MAX_PRIMITIVE_POOL}
    TALUS_BRICK_CELLS=${TALUS_BRICK_CELLS}
    TALUS_SAMPLER_LINEAR_REPEAT=${TALUS_SAMPLER_LINEAR_REPEAT}
    TALUS_SAMPLER_LINEAR_CLAMP=${TALUS_SAMPLER_LINEAR_CLAMP}
    TALUS_SAMPLER_NEAREST_CLAMP=${TALUS_SAMPLER_NEAREST_CLAMP}
)
if (MSVC)
    target_compile_options(talus_core PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_core Runtime)
