add_library(talus_gizmos STATIC
    grid/grid.cc
    handles/handles.cc
    outline/outline.cc
    pipelines/pipelines.cc
)

add_library(talus::gizmos ALIAS talus_gizmos)
target_include_directories(talus_gizmos PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_gizmos PUBLIC cxx_std_23)

target_link_libraries(talus_gizmos PUBLIC
    talus::core
    talus::debugdraw
    talus::field
    talus::gpu
    talus::view
)

if (MSVC)
    target_compile_options(talus_gizmos PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_gizmos PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_gizmos Editor)
