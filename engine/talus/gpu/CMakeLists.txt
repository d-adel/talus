add_library(talus_gpu STATIC
    buffer/buffer.cc
    computepipeline/computepipeline.cc
    context/context.cc
    depthimage/depthimage.cc
    descriptorgroup/descriptorgroup.cc
    device/device.cc
    image/image.cc
    instance/instance.cc
    oneshot/oneshotcommand.cc
    pipeline/pipeline.cc
    querypool/querypool.cc
    sampler/sampler.cc
    shadermodule/shadermodule.cc
    surface/surface.cc
    swapchain/swapchain.cc
    texture/texture.cc
    uploader/uploader.cc
)
add_library(talus::gpu ALIAS talus_gpu)
target_include_directories(talus_gpu PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_include_directories(talus_gpu PRIVATE "${stb_SOURCE_DIR}")

target_compile_features(talus_gpu PUBLIC cxx_std_23)

target_link_libraries(talus_gpu PUBLIC
    talus::stb
    talus::core
    Vulkan::Vulkan
    glm::glm
)
target_compile_definitions(talus_gpu PRIVATE
    SHADER_SOURCE_DIR="${SHADER_SOURCE_DIR}"
    GLSLC_PATH="${GLSLC_EXECUTABLE}"
    GLSLC_DEFINES="${SHADER_DEFINES_ARGS}"
    GLSLC_INCLUDE_DIR="${SHADER_SOURCE_DIR}"
)

if (MSVC)
    target_compile_options(talus_gpu PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_gpu PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_gpu Runtime)
