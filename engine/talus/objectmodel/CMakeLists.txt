add_library(talus_objectmodel STATIC
    instantiate/instantiate.cc
    defaultworld/defaultworld.cc
    component/renderpose/renderpose.cc
    component/sdfgrid/sdfgrid.cc
    sdfprimitivestore/sdfprimitivestore.cc
    dynamiccomponents/dynamiccomponents.cc
    impact/impact.cc
    sdfdeform/sdfdeform.cc
    world/world.cc
    worldcontents/worldcontents.cc
    worldquery/worldquery.cc
)
add_library(talus::objectmodel ALIAS talus_objectmodel)
target_include_directories(talus_objectmodel PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_compile_features(talus_objectmodel PUBLIC cxx_std_23)
target_link_libraries(talus_objectmodel PUBLIC
    talus::core
    talus::field
    glm::glm
    EnTT::EnTT
)
if (MSVC)
    target_compile_options(talus_objectmodel PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_objectmodel PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_objectmodel Runtime)
