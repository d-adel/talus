add_library(talus_package STATIC
    package.cc
)
add_library(talus::package ALIAS talus_package)
target_include_directories(talus_package PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_package PUBLIC cxx_std_23)
if (MSVC)
    target_compile_options(talus_package PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_package PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_package Editor)
