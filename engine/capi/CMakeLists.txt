add_library(talus_c SHARED
    win32keys/win32keys.cc
    talus_c.cc
    sceneview/sceneview.cc
    session/session.cc
    win32windowsystem/win32windowsystem.cc
)
add_library(talus::capi ALIAS talus_c)
target_include_directories(talus_c PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_compile_features(talus_c PUBLIC cxx_std_23)

target_link_libraries(talus_c PRIVATE
    talus::query
    talus::gizmos
    talus::package
    talus::assets
    talus::core
    talus::undo
    talus::engine
    talus::field
    talus::gpu
    talus::objectmodel
    talus::renderer
    talus::physics
    talus::runtime
    talus::serialize
    talus::script
    talus::systems
    talus::view
)

if (MSVC)
    target_compile_options(talus_c PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_c PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_c Editor)
