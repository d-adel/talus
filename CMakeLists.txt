cmake_minimum_required(VERSION 3.28)

project(Talus VERSION 0.0 LANGUAGES CXX C)

set_property(GLOBAL APPEND PROPERTY JOB_POOLS dotnet=1)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(TalusModule)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "/O2 /Ob2")
    add_compile_options(/Zc:preprocessor)
endif()
include(FetchContent)
FetchContent_Declare(
  glfw
  URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
  URL_HASH SHA256=c038d34200234d071fae9345bc455e4a8f2f544ab60150765d7704e08f3dac01
)
set(GLFW_BUILD_DOCS OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_EXAMPLES OFF)
set(SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
set(TALUS_MAX_SDF_PRIMITIVES 128)
set(TALUS_MAX_TEXTURES 16)
set(TALUS_MAX_SAMPLERS 8)
set(TALUS_MAX_MATERIALS 64)
set(TALUS_MAX_SDF_VOLUMES 4095)

set(TALUS_MAX_PRIMITIVE_POOL 8192)
set(TALUS_BRICK_CELLS 8)

set(TALUS_MARCH_EPSILON 0.001f)
set(TALUS_MARCH_MAX_ITERATIONS 800)
set(TALUS_SHADOW_MAX_DISTANCE 20.0f)
set(TALUS_MARCH_MAX_DISTANCE 100.0f)
set(TALUS_GRADIENT_EPSILON 0.01f)
set(TALUS_MARCH_OMEGA 1.2f)
set(TALUS_SHADOW_SHARPNESS 12.0f)
set(TALUS_SAMPLER_LINEAR_REPEAT 0)
set(TALUS_SAMPLER_LINEAR_CLAMP 1)
set(TALUS_SAMPLER_NEAREST_CLAMP 2)

set(TALUS_SHAPE_SPHERE 0)
set(TALUS_SHAPE_BOX 1)
set(TALUS_SHAPE_PLANE 2)
set(TALUS_SHAPE_CYLINDER 3)
set(TALUS_CSG_UNION 0)
set(TALUS_CSG_SMOOTH_UNION 1)
set(TALUS_CSG_INTERSECTION 2)
set(TALUS_CSG_SUBTRACTION 3)
set(TALUS_CSG_SMOOTH_SUBTRACTION 4)
set(SHADER_DEFINES
    "-DTALUS_MAX_SDF_PRIMITIVES=${TALUS_MAX_SDF_PRIMITIVES}"
    "-DTALUS_MAX_TEXTURES=${TALUS_MAX_TEXTURES}"
    "-DTALUS_MAX_SAMPLERS=${TALUS_MAX_SAMPLERS}"
    "-DTALUS_MAX_MATERIALS=${TALUS_MAX_MATERIALS}"
    "-DTALUS_MAX_SDF_VOLUMES=${TALUS_MAX_SDF_VOLUMES}"
    "-DTALUS_BRICK_CELLS=${TALUS_BRICK_CELLS}"
    "-DTALUS_SAMPLER_LINEAR_REPEAT=${TALUS_SAMPLER_LINEAR_REPEAT}"
    "-DTALUS_SAMPLER_LINEAR_CLAMP=${TALUS_SAMPLER_LINEAR_CLAMP}"
    "-DTALUS_SAMPLER_NEAREST_CLAMP=${TALUS_SAMPLER_NEAREST_CLAMP}"
    "-DTALUS_SHAPE_SPHERE=${TALUS_SHAPE_SPHERE}"
    "-DTALUS_SHAPE_BOX=${TALUS_SHAPE_BOX}"
    "-DTALUS_SHAPE_PLANE=${TALUS_SHAPE_PLANE}"
    "-DTALUS_SHAPE_CYLINDER=${TALUS_SHAPE_CYLINDER}"
    "-DTALUS_CSG_UNION=${TALUS_CSG_UNION}"
    "-DTALUS_CSG_SMOOTH_UNION=${TALUS_CSG_SMOOTH_UNION}"
    "-DTALUS_CSG_INTERSECTION=${TALUS_CSG_INTERSECTION}"
    "-DTALUS_CSG_SUBTRACTION=${TALUS_CSG_SUBTRACTION}"
    "-DTALUS_CSG_SMOOTH_SUBTRACTION=${TALUS_CSG_SMOOTH_SUBTRACTION}"
)
set(SHADER_INCLUDE_FLAG "-I${SHADER_SOURCE_DIR}")

file(GLOB SHADER_INCLUDES CONFIGURE_DEPENDS "${SHADER_SOURCE_DIR}/*.glsl")
string(JOIN " " SHADER_DEFINES_ARGS ${SHADER_DEFINES})
FetchContent_Declare(
  glm
  URL https://github.com/g-truc/glm/archive/0af55ccecd98d4e5a8d1fad7de25ba429d60e863.tar.gz
  URL_HASH SHA256=e7f187d83523f505eb38dd25d297ea6c0d4ed856d733e808f18253f5a8fa88a0
)
FetchContent_Declare(
    stb
    URL https://github.com/nothings/stb/archive/31c1ad37456438565541f4919958214b6e762fb4.tar.gz
    URL_HASH SHA256=e4e3bba9c572a4a4148373a914d88ea0f0d11de8cc2c66739926e7eca0223319
)
FetchContent_Declare(
    tinyobjloader
    URL https://github.com/tinyobjloader/tinyobjloader/archive/e60d33385e2e4f7fa891513150f2532b5bbcb093.tar.gz
    URL_HASH SHA256=e6729d1f8825728574c013884c2e5d7512d89b8a702286e134221b6a57084f91
    SOURCE_SUBDIR  cmake_disabled
)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF)
set(USE_ASSERTS ON)
set(OVERRIDE_CXX_FLAGS OFF)

FetchContent_Declare(
    JoltPhysics
    URL https://github.com/jrouwe/JoltPhysics/archive/refs/tags/v5.6.0.tar.gz
    URL_HASH SHA256=6e069ee0172478cc78182047aac87e5310ba14a67a53348ae14cc37801fd3f8e
    SOURCE_SUBDIR   Build
)
FetchContent_Declare(
    EnTT
    URL https://github.com/skypjack/entt/archive/refs/tags/v4.0.0.tar.gz
    URL_HASH SHA256=32a2ff2c72cb047dfd57306006ef238820b70da7c6ce4e7e8a507ac63365212e
)
FetchContent_Declare(
    glaze
    URL https://github.com/stephenberry/glaze/archive/refs/tags/v8.3.0.tar.gz
    URL_HASH SHA256=6a8f97f5cdb8df5c87f1b32710679585f3f1a577b67bb66150f2516072e00317
)
FetchContent_Declare(
    Eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR   cmake_disabled
)
FetchContent_MakeAvailable(glfw glm stb tinyobjloader JoltPhysics EnTT glaze Eigen)

add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")

target_compile_definitions(glm INTERFACE
    GLM_FORCE_RADIANS
    GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
    GLM_FORCE_DEPTH_ZERO_TO_ONE
    GLM_ENABLE_EXPERIMENTAL
)

find_package(Vulkan REQUIRED)

add_subdirectory(engine/talus/core)
add_subdirectory(engine/talus/objectmodel)
add_subdirectory(engine/talus/undo)
add_subdirectory(engine/talus/package)
add_subdirectory(engine/talus/view)
add_subdirectory(engine/talus/debugdraw)
add_subdirectory(engine/talus/gizmos)
add_subdirectory(engine/talus/platform)
add_subdirectory(engine/talus/field)
add_subdirectory(engine/talus/physics)
add_subdirectory(engine/talus/query)
add_subdirectory(engine/talus/serialize)
add_subdirectory(engine/talus/deformable)
add_subdirectory(engine/talus/runtime)
add_subdirectory(engine/talus/systems)
add_subdirectory(engine/talus/script)
add_subdirectory(engine/talus/stb)
add_subdirectory(engine/talus/imagecompare)
enable_testing()
find_program(
    GLSLC_EXECUTABLE
    NAMES glslc
    HINTS
        "$ENV{VULKAN_SDK}/Bin"
        "$ENV{VULKAN_SDK}/bin"
    REQUIRED
)
add_subdirectory(engine/talus/gpu)
add_subdirectory(engine/talus/renderer)
add_subdirectory(engine/talus/assets)
add_subdirectory(engine/talus/engine)
add_subdirectory(engine/capi)
if (EXISTS "${PROJECT_SOURCE_DIR}/editor/CMakeLists.txt")
    add_subdirectory(editor)
endif()
file(
    GLOB SHADER_SOURCES
    CONFIGURE_DEPENDS
    "${SHADER_SOURCE_DIR}/*.vert"
    "${SHADER_SOURCE_DIR}/*.frag"
    "${SHADER_SOURCE_DIR}/*.comp"
    "${SHADER_SOURCE_DIR}/*.geom"
    "${SHADER_SOURCE_DIR}/*.tesc"
    "${SHADER_SOURCE_DIR}/*.tese"
)
set(SHADER_OUTPUTS)
foreach(SHADER_SOURCE IN LISTS SHADER_SOURCES)
    get_filename_component(SHADER_FILENAME "${SHADER_SOURCE}" NAME)
    set(SHADER_OUTPUT
        "${SHADER_OUTPUT_DIR}/${SHADER_FILENAME}.spv"
    )
    add_custom_command(
        OUTPUT "${SHADER_OUTPUT}"
        COMMAND
            "${CMAKE_COMMAND}" -E make_directory
            "${SHADER_OUTPUT_DIR}"
        COMMAND
            "${GLSLC_EXECUTABLE}"
            ${SHADER_DEFINES}
            "${SHADER_INCLUDE_FLAG}"
            "${SHADER_SOURCE}"
            -o "${SHADER_OUTPUT}"

        DEPENDS "${SHADER_SOURCE}" ${SHADER_INCLUDES}
        COMMENT "Compiling shader ${SHADER_FILENAME}"
        VERBATIM
    )
    list(APPEND SHADER_OUTPUTS "${SHADER_OUTPUT}")
endforeach()

add_custom_target(
    SHADER_TARGET
    DEPENDS ${SHADER_OUTPUTS}
)

option(TALUS_TOOLS "Build the player's developer tooling: goldens, capture, bake verification" ON)

add_executable(TalusPlayer
    engine/player/main.cc
    engine/app/startupoptions/startupoptions.cc
    engine/app/inputbridge/inputbridge.cc
    engine/app/glfwwindowsystem/glfwwindowsystem.cc
    engine/app/engineloop/engineloop.cc
)

if (TALUS_TOOLS)
    target_sources(TalusPlayer PRIVATE
        engine/app/capture/capture.cc
        engine/app/framecheck/framecheck.cc
    )
endif()

target_compile_definitions(TalusPlayer PRIVATE TALUS_TOOLS=$<BOOL:${TALUS_TOOLS}>)
add_dependencies(TalusPlayer SHADER_TARGET)
if (MSVC)
    target_compile_options(TalusPlayer PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(TalusPlayer PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
target_link_libraries(TalusPlayer PRIVATE
    talus::assets
    talus::engine
    talus::serialize
    glfw
    Vulkan::Vulkan
    glm::glm
    talus::core
    talus::objectmodel
    talus::platform
    talus::gpu
    talus::renderer
    talus::field
    talus::physics
    talus::runtime
    talus::script
    talus::systems
)

if (TALUS_TOOLS)
    target_link_libraries(TalusPlayer PRIVATE talus::imagecompare)
endif()

target_include_directories(TalusPlayer PRIVATE engine ${stb_SOURCE_DIR} ${tinyobjloader_SOURCE_DIR})
add_custom_target(TALUS_CONTENT ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${SHADER_OUTPUT_DIR} $<TARGET_FILE_DIR:TalusPlayer>/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${PROJECT_SOURCE_DIR}/projects $<TARGET_FILE_DIR:TalusPlayer>/projects
    COMMENT "Copying shaders and projects"
    VERBATIM
)
add_dependencies(TALUS_CONTENT TalusPlayer SHADER_TARGET)
add_executable(TalusPackager
    engine/tools/packager/main.cc
)
target_link_libraries(TalusPackager PRIVATE talus::package)
if (MSVC)
    target_compile_options(TalusPackager PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(TalusPackager PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(TalusPackager Editor)
if (TALUS_TOOLS)
    talus_module(TalusPlayer Developer)
else()
    talus_module(TalusPlayer Runtime)
endif()

add_subdirectory(tests)

talus_require_declared()
talus_check_modules()
