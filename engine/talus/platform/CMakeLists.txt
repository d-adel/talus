add_library(talus_platform STATIC
    glfwlibrary/glfwlibrary.cc
    input/input.cc
    window/window.cc
)
add_library(talus::platform ALIAS talus_platform)
target_include_directories(talus_platform PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_platform PUBLIC cxx_std_23)
target_link_libraries(talus_platform PUBLIC
    talus::core
    glfw
)
if (MSVC)
    target_compile_options(talus_platform PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_platform PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_platform Runtime)
