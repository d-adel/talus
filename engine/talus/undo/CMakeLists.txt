add_library(talus_undo STATIC
    undostack/undostack.cc
)

add_library(talus::undo ALIAS talus_undo)
target_include_directories(talus_undo PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_undo PUBLIC cxx_std_23)

target_link_libraries(talus_undo PUBLIC
    talus::objectmodel
)

target_link_libraries(talus_undo PRIVATE talus::serialize)

if (MSVC)
    target_compile_options(talus_undo PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_undo PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_undo Editor)
