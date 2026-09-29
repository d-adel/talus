add_library(talus_renderer STATIC
    renderer.cc
    bindgroups/bindgroups.cc
    rendererstorage/rendererstorage.cc
    sdfbaker/sdfbaker.cc
    sdfrecordpacker/sdfrecordpacker.cc
    framepacker/framepacker.cc
    meshrecordpacker/meshrecordpacker.cc
    sdfrecordtable/sdfrecordtable.cc
    sdfrecord/sdfrecord.cc
    frame/frame.cc
    frameglobals/frameglobals.cc
    lines/lines.cc
    meshbuffers/meshbuffers.cc
    meshbuffers/tiny_obj_loader_impl.cc
    resourcetable/resourcetable.cc
    vertex/vertex.cc
    shadowgrid/shadowgrid.cc
    sdfresidency/sdfresidency.cc
    scenetarget/scenetarget.cc
    volumepool/volumepool.cc
)
add_library(talus::renderer ALIAS talus_renderer)

target_include_directories(talus_renderer PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_include_directories(talus_renderer PRIVATE "${tinyobjloader_SOURCE_DIR}")
target_compile_features(talus_renderer PUBLIC cxx_std_23)

target_link_libraries(talus_renderer PUBLIC
    talus::core
    talus::debugdraw
    talus::field
    talus::objectmodel
    talus::gpu
)
if (MSVC)
    target_compile_options(talus_renderer PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_renderer PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_renderer Runtime)
