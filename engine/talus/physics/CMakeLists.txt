add_library(talus_physics STATIC
   fieldshape/fieldshape.cc
   impact/impact.cc
   joltlibrary/joltlibrary.cc
   layers/layers.cc
   physicsworld/physicsworld.cc
)
add_library(talus::physics ALIAS talus_physics)
target_include_directories(talus_physics PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_physics PUBLIC cxx_std_23)
target_link_libraries(talus_physics PUBLIC
    talus::core
    talus::field
    talus::objectmodel
    Jolt
)
if (MSVC)
    target_compile_options(talus_physics PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_physics PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_physics Runtime)
