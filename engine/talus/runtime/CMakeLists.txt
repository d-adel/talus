add_library(talus_runtime STATIC
    runtime/runtime.cc
)
add_library(talus::runtime ALIAS talus_runtime)
target_include_directories(talus_runtime PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_compile_features(talus_runtime PUBLIC cxx_std_23)

target_link_libraries(talus_runtime PUBLIC
    talus::core
    talus::deformable
    talus::field
    talus::objectmodel
    talus::physics
)
if (MSVC)
    target_compile_options(talus_runtime PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_runtime PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_runtime Runtime)
