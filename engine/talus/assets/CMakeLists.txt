add_library(talus_assets STATIC
    assetlibrary/assetlibrary.cc
)
add_library(talus::assets ALIAS talus_assets)
target_include_directories(talus_assets PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_compile_features(talus_assets PUBLIC cxx_std_23)

target_link_libraries(talus_assets PUBLIC
    talus::core
    talus::gpu
    talus::objectmodel
    talus::renderer
    talus::serialize
)

if (MSVC)
    target_compile_options(talus_assets PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_assets PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_assets Runtime)
