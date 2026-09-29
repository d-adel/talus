file(GLOB TALUS_NETHOST_CANDIDATES
    "$ENV{ProgramFiles}/dotnet/packs/Microsoft.NETCore.App.Host.win-x64/*/runtimes/win-x64/native"
)

if (NOT TALUS_NETHOST_CANDIDATES)
    message(FATAL_ERROR
        "The .NET hosting pack was not found. Install the .NET 10 SDK, or point "
        "TALUS_NETHOST_DIR at a directory holding nethost.h and libnethost.lib."
    )
endif()

list(SORT TALUS_NETHOST_CANDIDATES)
list(GET TALUS_NETHOST_CANDIDATES -1 TALUS_NETHOST_DIR)

message(STATUS "talus::script hosting pack: ${TALUS_NETHOST_DIR}")

add_library(talus_script STATIC
    api/api.cc
    host/host.cc
    runner/runner.cc
)

add_library(talus::script ALIAS talus_script)
target_include_directories(talus_script PUBLIC "${PROJECT_SOURCE_DIR}/engine")
target_include_directories(talus_script PRIVATE "${TALUS_NETHOST_DIR}")
target_compile_features(talus_script PUBLIC cxx_std_23)


target_link_libraries(talus_script PUBLIC
    talus::core
    talus::objectmodel
    talus::systems
)

target_link_libraries(talus_script PRIVATE
    talus::query
    "${TALUS_NETHOST_DIR}/nethost.lib"
)

if (MSVC)
    target_compile_options(talus_script PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_script PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

find_program(TALUS_DOTNET dotnet REQUIRED)

set(TALUS_MANAGED_DIR "${CMAKE_BINARY_DIR}/managed")

add_custom_target(TALUS_MANAGED ALL
    COMMAND "${TALUS_DOTNET}" build
            "${PROJECT_SOURCE_DIR}/engine/managed/Talus.Engine/Talus.Engine.csproj"
            -c $<IF:$<CONFIG:Release>,Release,Debug>
            -o "${TALUS_MANAGED_DIR}"
            --nologo -v quiet
    JOB_POOL dotnet
    BYPRODUCTS "${TALUS_MANAGED_DIR}/Talus.Engine.dll"
    COMMENT "Building the managed engine assembly"
)

file(GLOB TALUS_SCRIPT_PROJECTS "${PROJECT_SOURCE_DIR}/projects/*/scripts/Scripts.csproj")

foreach(TALUS_SCRIPT_PROJECT ${TALUS_SCRIPT_PROJECTS})
    get_filename_component(TALUS_SCRIPT_DIR "${TALUS_SCRIPT_PROJECT}" DIRECTORY)
    get_filename_component(TALUS_PROJECT_DIR "${TALUS_SCRIPT_DIR}" DIRECTORY)
    get_filename_component(TALUS_PROJECT_NAME "${TALUS_PROJECT_DIR}" NAME)

    add_custom_target(TALUS_SCRIPTS_${TALUS_PROJECT_NAME} ALL
        COMMAND "${TALUS_DOTNET}" build "${TALUS_SCRIPT_PROJECT}"
                -c $<IF:$<CONFIG:Release>,Release,Debug>
                -o "${CMAKE_BINARY_DIR}/projects/${TALUS_PROJECT_NAME}/scripts"
                "-p:TalusEngineDir=${TALUS_MANAGED_DIR}"
                --nologo -v quiet
    JOB_POOL dotnet
        DEPENDS TALUS_MANAGED
        COMMENT "Building scripts for project ${TALUS_PROJECT_NAME}"
    )
endforeach()

file(COPY "${TALUS_NETHOST_DIR}/nethost.dll" DESTINATION "${CMAKE_BINARY_DIR}")
file(COPY "${TALUS_NETHOST_DIR}/nethost.dll" DESTINATION "${CMAKE_BINARY_DIR}/tests")

talus_module(talus_script Runtime)
