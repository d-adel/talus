add_library(talus_systems STATIC
    input/input.cc
    schedule/schedule.cc
)

add_library(talus::systems ALIAS talus_systems)
target_include_directories(talus_systems PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_systems PUBLIC cxx_std_23)

target_link_libraries(talus_systems PUBLIC
    talus::debugdraw
    talus::objectmodel
)

if (MSVC)
    target_compile_options(talus_systems PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_systems PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_systems Runtime)
