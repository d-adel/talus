add_library(talus_debugdraw STATIC
    debugdraw/debugdraw.cc
)

add_library(talus::debugdraw ALIAS talus_debugdraw)
target_include_directories(talus_debugdraw PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_debugdraw PUBLIC cxx_std_23)

target_link_libraries(talus_debugdraw PUBLIC
    talus::core
    glm::glm
)

if (MSVC)
    target_compile_options(talus_debugdraw PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_debugdraw PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_debugdraw Runtime)
