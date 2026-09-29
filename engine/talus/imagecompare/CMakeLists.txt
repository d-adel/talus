add_library(talus_imagecompare STATIC
    imagecompare.cc
)
add_library(talus::imagecompare ALIAS talus_imagecompare)
target_include_directories(talus_imagecompare PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_link_libraries(talus_imagecompare PRIVATE talus::stb)
target_compile_features(talus_imagecompare PUBLIC cxx_std_23)
if (MSVC)
    target_compile_options(talus_imagecompare PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_imagecompare PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_imagecompare Developer)
