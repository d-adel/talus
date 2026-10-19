cmake_minimum_required(VERSION 3.28)
project(Talus VERSION 0.1.0 LANGUAGES CXX)

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

option(TALUS_BUILD_TESTS "Build physics regression tests" ${PROJECT_IS_TOP_LEVEL})
option(TALUS_BUILD_EXAMPLES "Build headless physics examples" ${PROJECT_IS_TOP_LEVEL})
option(TALUS_FETCH_DEPENDENCIES "Fetch missing header dependencies" ON)

find_package(glm 1 CONFIG QUIET)
if (NOT TARGET glm::glm)
    if (NOT TALUS_FETCH_DEPENDENCIES)
        message(FATAL_ERROR "GLM 1.x is required. Install it or enable TALUS_FETCH_DEPENDENCIES.")
    endif()
    FetchContent_Declare(glm
        URL https://github.com/g-truc/glm/archive/0af55ccecd98d4e5a8d1fad7de25ba429d60e863.tar.gz
        URL_HASH SHA256=e7f187d83523f505eb38dd25d297ea6c0d4ed856d733e808f18253f5a8fa88a0
        SOURCE_SUBDIR talus_headers_only)
    FetchContent_MakeAvailable(glm)
    add_library(glm::glm INTERFACE IMPORTED GLOBAL)
    set_target_properties(glm::glm PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${glm_SOURCE_DIR}")
endif()

find_package(Eigen3 3.4 CONFIG QUIET)
if (NOT TARGET Eigen3::Eigen)
    if (NOT TALUS_FETCH_DEPENDENCIES)
        message(FATAL_ERROR "Eigen 3.4 is required. Install it or enable TALUS_FETCH_DEPENDENCIES.")
    endif()
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
        SOURCE_SUBDIR talus_headers_only)
    FetchContent_MakeAvailable(eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
endif()

include(cmake/sources.cmake)
add_library(talus_physics STATIC ${TALUS_SOURCES})
add_library(Talus::Physics ALIAS talus_physics)
set_target_properties(talus_physics PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME Physics)
target_compile_features(talus_physics PUBLIC cxx_std_23)
target_include_directories(talus_physics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(talus_physics PUBLIC glm::glm Eigen3::Eigen Threads::Threads)
target_compile_definitions(talus_physics PUBLIC
    GLM_FORCE_RADIANS GLM_FORCE_DEFAULT_ALIGNED_GENTYPES GLM_ENABLE_EXPERIMENTAL
    $<$<CXX_COMPILER_ID:MSVC>:GLM_FORCE_INLINE>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:_MSC_EXTENSIONS>)
target_compile_definitions(talus_physics PUBLIC
    TALUS_MARCH_EPSILON=0.001f TALUS_MARCH_MAX_ITERATIONS=800
    TALUS_MARCH_MAX_DISTANCE=100.0f TALUS_GRADIENT_EPSILON=0.01f
    TALUS_MARCH_OMEGA=1.2f
    TALUS_BRICK_CELLS=8 TALUS_SHAPE_SPHERE=0 TALUS_SHAPE_BOX=1 TALUS_SHAPE_PLANE=2
    TALUS_SHAPE_CYLINDER=3 TALUS_CSG_UNION=0 TALUS_CSG_SMOOTH_UNION=1
    TALUS_CSG_INTERSECTION=2 TALUS_CSG_SUBTRACTION=3 TALUS_CSG_SMOOTH_SUBTRACTION=4)

if (MSVC)
    target_compile_options(talus_physics PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus /Zc:preprocessor)
else()
    target_compile_options(talus_physics PRIVATE -Wall -Wextra -Wpedantic -Werror -ffp-contract=off)
endif()
if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|amd64|x86_64)$")
    target_compile_definitions(talus_physics PRIVATE TALUS_WIDE_CONTACTS)
    if (MSVC)
        set_source_files_properties(src/talus/dynamics/softstep/wide8.cc
            src/talus/destruction/stress/widesums.cc PROPERTIES COMPILE_OPTIONS /arch:AVX2)
    else()
        set_source_files_properties(src/talus/dynamics/softstep/wide8.cc
            src/talus/destruction/stress/widesums.cc PROPERTIES COMPILE_OPTIONS -mavx2)
    endif()
else()
    message(FATAL_ERROR "This initial extraction supports x86-64; other architectures are not yet qualified.")
endif()

install(TARGETS talus_physics EXPORT TalusTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY src/talus DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.ih" PATTERN "*.inl")
install(EXPORT TalusTargets NAMESPACE Talus:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Talus)
configure_package_config_file(cmake/TalusConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/TalusConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Talus)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/TalusConfigVersion.cmake"
    COMPATIBILITY SameMajorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/TalusConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/TalusConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Talus)

if (TALUS_BUILD_EXAMPLES)
    add_executable(talus_headless examples/headless.cc)
    target_link_libraries(talus_headless PRIVATE Talus::Physics)
endif()
if (TALUS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
