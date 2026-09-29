add_library(talus_engine STATIC
    engine/engine.cc
)
add_library(talus::engine ALIAS talus_engine)
target_include_directories(talus_engine PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_compile_features(talus_engine PUBLIC cxx_std_23)

target_link_libraries(talus_engine PUBLIC
    talus::assets
    talus::debugdraw
    talus::core
    talus::field
    talus::gpu
    talus::objectmodel
    talus::physics
    talus::renderer
    talus::runtime
    talus::script
    talus::serialize
    talus::systems
    talus::view
)

target_link_libraries(talus_engine PRIVATE talus::query)

if (MSVC)
    target_compile_options(talus_engine PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_engine PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_engine Runtime)
