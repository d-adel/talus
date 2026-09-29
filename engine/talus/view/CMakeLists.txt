add_library(talus_view STATIC
    camera/camera.cc
    viewportstore/viewportstore.cc
    lookthrough/lookthrough.cc
    outlinerule/outlinerule.cc
)

add_library(talus::view ALIAS talus_view)
target_include_directories(talus_view PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_view PUBLIC cxx_std_23)

target_link_libraries(talus_view PUBLIC
    talus::core
    talus::objectmodel
)

if (MSVC)
    target_compile_options(talus_view PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_view PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_view Runtime)
