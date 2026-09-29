add_library(talus_serialize STATIC
    gamemanifest/gamemanifest.cc
    editorstate/editorstate.cc
    project/project.cc
    textfile/textfile.cc
    materialfile/materialfile.cc
    componentschema/componentschema.cc
    worldfile/worldfile.cc
)
add_library(talus::serialize ALIAS talus_serialize)
target_include_directories(talus_serialize PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_serialize PUBLIC cxx_std_23)
target_link_libraries(talus_serialize PUBLIC
    talus::core
    talus::field
    talus::objectmodel
    glaze::glaze
)
if (MSVC)
    target_compile_options(talus_serialize PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_serialize PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_serialize Runtime)
