FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2
  GIT_TAG        v3.7.1
  SYSTEM
)

FetchContent_MakeAvailable(Catch2)

add_executable(unit_tests
    quietfailures.cc
    asset_tests.cc
    debugdraw_tests.cc
    assetdatabase_tests.cc
    axisdrag_tests.cc
    body_tests.cc
    boundedextent_tests.cc
    recordbounds_tests.cc
    maincamera_tests.cc
    capi_tests.cc
    crack_tests.cc
    deform_tests.cc
    deformable_tests.cc
    dynamiccomponents_tests.cc
    entt_tests.cc
    input_tests.cc
    ../engine/capi/win32keys/win32keys.cc
    field_tests.cc
    glmmeta_tests.cc
    worldmeta_tests.cc
    materialfile_tests.cc
    outlinerule_tests.cc
    plasticity_tests.cc
    worldfile_tests.cc
    viewport_tests.cc
    viewportstore_tests.cc
    lookthrough_tests.cc
    camerapose_tests.cc
    fieldshape_tests.cc
    impact_tests.cc
    impactwindow_tests.cc
    windowmonitor.cc
    lens_tests.cc
    latticekey_tests.cc
    componentcoverage_tests.cc
    instantiateworld_tests.cc
    pose_tests.cc
    physics_tests.cc
    project_tests.cc
    rangestore_tests.cc
    raycast_tests.cc
    query_tests.cc
    redistance_tests.cc
    runtime_tests.cc
    schedule_tests.cc
    screenray_tests.cc
    scriptverbs_tests.cc
    scriptbridge_tests.cc
    scripthost_tests.cc
    sdfprimitivestore_tests.cc
    shadowgrid_tests.cc
    telemetry_tests.cc
    sampled_tests.cc
    worldcontents_tests.cc
    componentschema_tests.cc
    undostack_tests.cc
    world_tests.cc
)

target_include_directories(unit_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(unit_tests PRIVATE talus::query talus::debugdraw talus::view talus::script talus::core talus::undo talus::field talus::objectmodel talus::physics talus::runtime talus::serialize talus::systems talus::renderer talus::capi Catch2::Catch2WithMain)

if (MSVC)
    target_compile_options(unit_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/investigations")
    add_executable(investigation_probes EXCLUDE_FROM_ALL
        quietfailures.cc
        investigations/frozencontact.cc
        investigations/contactconvergence.cc
        investigations/contactcost.cc
        investigations/contactgeometry.cc
        investigations/factorcache.cc
        investigations/floatingframe.cc
        investigations/r11contact.cc
        investigations/integrator.cc
        investigations/meshresolution.cc
        investigations/crackrelease.cc
        investigations/hexmodes.cc
        investigations/impactsubsteps.cc
        investigations/plasticdynamics.cc
        investigations/tetplasticity.cc
        investigations/restingprojectile.cc
    )

    target_include_directories(investigation_probes PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

    target_compile_definitions(investigation_probes PRIVATE TALUS_DEMO_PROJECT="${PROJECT_SOURCE_DIR}/projects/demo")

    target_link_libraries(investigation_probes PRIVATE talus::query talus::debugdraw talus::view talus::script talus::core talus::undo talus::field talus::objectmodel talus::physics talus::runtime talus::serialize talus::systems talus::capi Catch2::Catch2WithMain)

    if (MSVC)
        target_compile_options(investigation_probes PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
    else()
        target_compile_options(investigation_probes PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()

    talus_module(investigation_probes Editor)

    add_test(
        NAME "impact substeps: a struck anchored beam stays within its injected energy"
        COMMAND investigation_probes "impact substeps: a struck anchored beam stays within its injected energy"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "impact substeps: a struck anchored beam stays within its injected energy"
        PROPERTIES
            LABELS tools
            TIMEOUT 600
            RUN_SERIAL TRUE
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
    )

    add_test(
        NAME "resting projectile: a dented field ball dropped on the demo floor keeps its support and sleeps"
        COMMAND investigation_probes "resting projectile: a dented field ball dropped on the demo floor keeps its support and sleeps"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "resting projectile: a dented field ball dropped on the demo floor keeps its support and sleeps"
        PROPERTIES
            LABELS tools
            TIMEOUT 600
            RUN_SERIAL TRUE
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
    )
endif()


if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/campaign")
    add_executable(campaign_benchmarks
        quietfailures.cc
        campaign/worldscale.cc
        windowmonitor.cc
    )

    target_include_directories(campaign_benchmarks PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

    target_link_libraries(campaign_benchmarks PRIVATE talus::query talus::debugdraw talus::view talus::script talus::core talus::undo talus::field talus::objectmodel talus::physics talus::runtime talus::serialize talus::systems talus::capi Catch2::Catch2WithMain)

    if (MSVC)
        target_compile_options(campaign_benchmarks PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
    else()
        target_compile_options(campaign_benchmarks PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()

    talus_module(campaign_benchmarks Editor)
endif()


if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/investigations")
    add_executable(render_scale EXCLUDE_FROM_ALL
        quietfailures.cc
        investigations/renderscale.cc
        ../engine/app/capture/capture.cc
        ../engine/capi/win32windowsystem/win32windowsystem.cc
    )
    target_include_directories(render_scale PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(render_scale PRIVATE talus::engine talus::imagecompare user32 Catch2::Catch2WithMain)
    target_compile_definitions(render_scale PRIVATE TALUS_DEMO_PROJECT="${PROJECT_SOURCE_DIR}/projects/demo")
    if (MSVC)
        target_compile_options(render_scale PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
    else()
        target_compile_options(render_scale PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    talus_module(render_scale Editor)
    add_dependencies(render_scale SHADER_TARGET)
endif()


add_executable(renderer_tests
    quietfailures.cc
    renderer_tests.cc
    ../engine/app/capture/capture.cc
    ../engine/capi/win32windowsystem/win32windowsystem.cc
)
target_include_directories(renderer_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(renderer_tests PRIVATE talus::engine talus::imagecompare talus::undo user32 Catch2::Catch2WithMain)
target_compile_definitions(renderer_tests PRIVATE TALUS_DEMO_PROJECT="${PROJECT_SOURCE_DIR}/projects/demo")
if (MSVC)
    target_compile_options(renderer_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(renderer_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
talus_module(renderer_tests Editor)
add_dependencies(renderer_tests SHADER_TARGET)

foreach(renderer_case
    "every field entity has a record at 5000 boxes"
    "every field entity has a record at 100000 boxes"
    "an entity with no primitives is drawn from its own lattice"
    "equal lattices share one volume"
    "a lattice changed in place is not shared by its old content"
    "the volume pool grows to its ceiling and counts what it drops"
    "a grid edit on a lattice-only entity keeps it and its sharer drawn"
    "a primitive added to a lattice-only entity leaves its sharer drawn"
    "equal primitive entities keep one volume through an uncommitted bake"
    "equal lattices share again after an in-place write to their old volume"
    "a dent that grows the volume pool keeps every brick bound"
    "an entity whose last primitive is removed draws nothing and leaves its sharer drawn"
    "equal primitives on a grid left stale by a resolution edit never share"
    "an entity whose primitives were all removed stays empty through undo"
    "an entity whose primitives were all removed stays empty through Play"
    "a drawn lattice-only entity given another lattice draws the new one"
)
    add_test(
        NAME "${renderer_case}"
        COMMAND renderer_tests "${renderer_case}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("${renderer_case}" PROPERTIES TIMEOUT 600 RUN_SERIAL TRUE)
endforeach()

add_executable(restart_tests
    quietfailures.cc
    restart_tests.cc
    ../engine/capi/win32windowsystem/win32windowsystem.cc
)
target_link_libraries(restart_tests PRIVATE talus::engine user32 Catch2::Catch2WithMain)
target_compile_definitions(restart_tests PRIVATE TALUS_DEMO_PROJECT="${PROJECT_SOURCE_DIR}/projects/demo")
if (MSVC)
    target_compile_options(restart_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(restart_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
talus_module(restart_tests Editor)
add_dependencies(restart_tests SHADER_TARGET)
add_test(
    NAME "restart engine lifecycle"
    COMMAND restart_tests
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties("restart engine lifecycle" PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE)

add_executable(fixedstep_tests
    quietfailures.cc
    fixedstep_tests.cc
    ../engine/capi/win32windowsystem/win32windowsystem.cc
)
target_link_libraries(fixedstep_tests PRIVATE talus::engine user32 Catch2::Catch2WithMain)
target_compile_definitions(fixedstep_tests PRIVATE TALUS_DEMO_PROJECT="${PROJECT_SOURCE_DIR}/projects/demo")
if (MSVC)
    target_compile_options(fixedstep_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(fixedstep_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
talus_module(fixedstep_tests Editor)
add_dependencies(fixedstep_tests SHADER_TARGET)
add_test(
    NAME "fixed step drives the tick"
    COMMAND fixedstep_tests
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties("fixed step drives the tick" PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE)

if (EXISTS "${PROJECT_SOURCE_DIR}/projects/demo/worlds/bending.world.json")
    add_executable(wireframe_tests
        quietfailures.cc
        wireframe_tests.cc
        ../engine/capi/win32windowsystem/win32windowsystem.cc
    )
    target_link_libraries(wireframe_tests PRIVATE talus::engine user32 Catch2::Catch2WithMain)
    target_compile_definitions(wireframe_tests PRIVATE TALUS_DEMO_PROJECT="${PROJECT_SOURCE_DIR}/projects/demo")
    if (MSVC)
        target_compile_options(wireframe_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
    else()
        target_compile_options(wireframe_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    talus_module(wireframe_tests Editor)
    add_dependencies(wireframe_tests SHADER_TARGET)
    add_test(
        NAME "the deformable wireframe draws"
        COMMAND wireframe_tests
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("the deformable wireframe draws" PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE)

    add_test(
        NAME "a deformable world reports its rendered scaling envelope"
        COMMAND wireframe_tests "a deformable world reports its rendered scaling envelope"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "a deformable world reports its rendered scaling envelope"
        PROPERTIES
            LABELS tools
            TIMEOUT 600
            RUN_SERIAL TRUE
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
    )

    add_test(
        NAME "a deformable contact reports its rendered inventory"
        COMMAND wireframe_tests "a deformable contact reports its rendered inventory"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "a deformable contact reports its rendered inventory"
        PROPERTIES
            LABELS tools
            TIMEOUT 600
            RUN_SERIAL TRUE
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
    )

    add_test(
        NAME "a deformable contact measures unbuilt sustained response"
        COMMAND wireframe_tests "a deformable contact measures unbuilt sustained response"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "a deformable contact measures unbuilt sustained response"
        PROPERTIES
            LABELS tools
            TIMEOUT 600
            RUN_SERIAL TRUE
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
    )
endif()


add_test(
    NAME "a deformable world reports its CPU scaling envelope"
    COMMAND unit_tests "a deformable world reports its CPU scaling envelope"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable world reports its CPU scaling envelope"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable contact reports its controlled scale"
    COMMAND unit_tests "a deformable contact reports its controlled scale"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable contact reports its controlled scale"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable contact compares dense and unbuilt response"
    COMMAND unit_tests "a deformable contact compares dense and unbuilt response"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable contact compares dense and unbuilt response"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable co-sleep probe"
    COMMAND unit_tests "a deformable co-sleep probe"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable co-sleep probe"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable arming holds its crate awake"
    COMMAND unit_tests "a deformable arming holds its crate awake"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable arming holds its crate awake"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a soft oscillating deformable never freezes"
    COMMAND unit_tests "a soft oscillating deformable never freezes"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a soft oscillating deformable never freezes"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a heavy crate needs its load to freeze"
    COMMAND unit_tests "a heavy crate needs its load to freeze"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a heavy crate needs its load to freeze"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable beam carries a resting box"
    COMMAND unit_tests "a deformable beam carries a resting box"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable beam carries a resting box"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable arming holds a sleeping partner"
    COMMAND unit_tests "a deformable arming holds a sleeping partner"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable arming holds a sleeping partner"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a destroyed frozen deformable drops its load"
    COMMAND unit_tests "a destroyed frozen deformable drops its load"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a destroyed frozen deformable drops its load"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a destroyed deformable wakes the box its refusal left asleep"
    COMMAND unit_tests "a destroyed deformable wakes the box its refusal left asleep"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a destroyed deformable wakes the box its refusal left asleep"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a frozen deformable wakes on an edit and on a mover"
    COMMAND unit_tests "a frozen deformable wakes on an edit and on a mover"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a frozen deformable wakes on an edit and on a mover"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a frozen deformable wakes when a crate it holds is destroyed"
    COMMAND unit_tests "a frozen deformable wakes when a crate it holds is destroyed"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a frozen deformable wakes when a crate it holds is destroyed"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a frozen deformable wakes when the top of its stack is removed"
    COMMAND unit_tests "a frozen deformable wakes when the top of its stack is removed"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a frozen deformable wakes when the top of its stack is removed"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a frozen deformable wakes at the step after a paused teleport of its load"
    COMMAND unit_tests "a frozen deformable wakes at the step after a paused teleport of its load"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a frozen deformable wakes at the step after a paused teleport of its load"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a frozen deformable wakes when a static touching its load is destroyed"
    COMMAND unit_tests "a frozen deformable wakes when a static touching its load is destroyed"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a frozen deformable wakes when a static touching its load is destroyed"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a load activated inside a physics update wakes its deformable"
    COMMAND unit_tests "a load activated inside a physics update wakes its deformable"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a load activated inside a physics update wakes its deformable"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "an edit outside every loading island wakes and disarms nothing"
    COMMAND unit_tests "an edit outside every loading island wakes and disarms nothing"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "an edit outside every loading island wakes and disarms nothing"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a gravity change wakes a frozen deformable"
    COMMAND unit_tests "a gravity change wakes a frozen deformable"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a gravity change wakes a frozen deformable"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a crate shared by two frozen deformables wakes both"
    COMMAND unit_tests "a crate shared by two frozen deformables wakes both"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a crate shared by two frozen deformables wakes both"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a re-prepared then destroyed deformable leaves no box asleep"
    COMMAND unit_tests "a re-prepared then destroyed deformable leaves no box asleep"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a re-prepared then destroyed deformable leaves no box asleep"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "an arming deformable disarms on an edit above its sleeping partner"
    COMMAND unit_tests "an arming deformable disarms on an edit above its sleeping partner"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "an arming deformable disarms on an edit above its sleeping partner"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a free spinning deformable keeps its energy bounded"
    COMMAND unit_tests "a free spinning deformable keeps its energy bounded"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a free spinning deformable keeps its energy bounded"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable contact dissipates through friction at a converged tolerance"
    COMMAND unit_tests "a deformable contact dissipates through friction at a converged tolerance"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable contact dissipates through friction at a converged tolerance"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a plastic bar keeps its permanent strain under two refinements"
    COMMAND unit_tests "a plastic bar keeps its permanent strain under two refinements"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a plastic bar keeps its permanent strain under two refinements"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a plastic bar keeps its static strain at one production substep"
    COMMAND unit_tests "a plastic bar keeps its static strain at one production substep"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a plastic bar keeps its static strain at one production substep"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a capped plastic substep ends no worse than its linear iterate"
    COMMAND unit_tests "a capped plastic substep ends no worse than its linear iterate"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a capped plastic substep ends no worse than its linear iterate"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a hex bar yielded and released at rest follows its eigenstrain path"
    COMMAND unit_tests "a hex bar yielded and released at rest follows its eigenstrain path"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a hex bar yielded and released at rest follows its eigenstrain path"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a tet bar yielded and released at rest follows its eigenstrain path"
    COMMAND unit_tests "a tet bar yielded and released at rest follows its eigenstrain path"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a tet bar yielded and released at rest follows its eigenstrain path"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a clamped plastic bar under a step load hands its first step to Newton"
    COMMAND unit_tests "a clamped plastic bar under a step load hands its first step to Newton"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a clamped plastic bar under a step load hands its first step to Newton"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a plastic bar at rest under a step load below yield follows the linear path"
    COMMAND unit_tests "a plastic bar at rest under a step load below yield follows the linear path"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a plastic bar at rest under a step load below yield follows the linear path"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a free plastic bar spun rigidly at 5 and 10 rad/s does not flow over 300 frames"
    COMMAND unit_tests "a free plastic bar spun rigidly at 5 and 10 rad/s does not flow over 300 frames"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a free plastic bar spun rigidly at 5 and 10 rad/s does not flow over 300 frames"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "every damped operator site assembles one damping"
    COMMAND unit_tests "every damped operator site assembles one damping"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "every damped operator site assembles one damping"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a damped cantilever recovers its authored damping ratio"
    COMMAND unit_tests "a damped cantilever recovers its authored damping ratio"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a damped cantilever recovers its authored damping ratio"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "damping leaves the rigid motion of a spinning body to the step"
    COMMAND unit_tests "damping leaves the rigid motion of a spinning body to the step"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "damping leaves the rigid motion of a spinning body to the step"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a damped plastic body keeps its damping on the Newton path"
    COMMAND unit_tests "a damped plastic body keeps its damping on the Newton path"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a damped plastic body keeps its damping on the Newton path"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a yielding bar struck by a box and a spun bar forced through promotion keep momentum and add no energy"
    COMMAND unit_tests "a yielding bar struck by a box and a spun bar forced through promotion keep momentum and add no energy"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a yielding bar struck by a box and a spun bar forced through promotion keep momentum and add no energy"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a free plastic bar spun at 20 and 30 rad/s from zero acceleration stays elastic"
    COMMAND unit_tests "a free plastic bar spun at 20 and 30 rad/s from zero acceleration stays elastic"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a free plastic bar spun at 20 and 30 rad/s from zero acceleration stays elastic"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a plastic bar dropped obliquely on a field slab comes to rest with finite plastic strain"
    COMMAND unit_tests "a plastic bar dropped obliquely on a field slab comes to rest with finite plastic strain"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a plastic bar dropped obliquely on a field slab comes to rest with finite plastic strain"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a plastic bar spun while resting on a field slab yields without a fallback and within its tight summed strain"
    COMMAND unit_tests "a plastic bar spun while resting on a field slab yields without a fallback and within its tight summed strain"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a plastic bar spun while resting on a field slab yields without a fallback and within its tight summed strain"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a landing deformable's acceleration holds only its smooth solve"
    COMMAND unit_tests "a landing deformable's acceleration holds only its smooth solve"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a landing deformable's acceleration holds only its smooth solve"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a free deformable bar resting on a field slab freezes with its confirm resolved"
    COMMAND unit_tests "a free deformable bar resting on a field slab freezes with its confirm resolved"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a free deformable bar resting on a field slab freezes with its confirm resolved"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a frozen free deformable bar wakes on an impulse and refreezes within two arms"
    COMMAND unit_tests "a frozen free deformable bar wakes on an impulse and refreezes within two arms"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a frozen free deformable bar wakes on an impulse and refreezes within two arms"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a deformable bar dropped face down comes to rest on a field slab"
    COMMAND unit_tests "a deformable bar dropped face down comes to rest on a field slab"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a deformable bar dropped face down comes to rest on a field slab"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a free deformable bar spun about its long axis lands on a wide slab without gaining energy"
    COMMAND unit_tests "a free deformable bar spun about its long axis lands on a wide slab without gaining energy"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a free deformable bar spun about its long axis lands on a wide slab without gaining energy"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a free deformable bar tumbling about a skew axis lands on a wide slab without gaining energy"
    COMMAND unit_tests "a free deformable bar tumbling about a skew axis lands on a wide slab without gaining energy"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a free deformable bar tumbling about a skew axis lands on a wide slab without gaining energy"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

add_test(
    NAME "a one-cell hex plate in tension reaches its equilibrium"
    COMMAND unit_tests "a one-cell hex plate in tension reaches its equilibrium"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
)
set_tests_properties(
    "a one-cell hex plate in tension reaches its equilibrium"
    PROPERTIES
        LABELS tools
        TIMEOUT 600
        RUN_SERIAL TRUE
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
)

foreach(WINDOW_CASE
    "an impact window in free flight creates no energy and keeps its momentum"
    "an elastic landing in an impact window stays within its energy bound"
    "impact window landings match the explicit reference"
    "impact window groups step alike on one and four threads"
    "a yielding deformable promotes on impact and returns to Jolt with its momentum"
    "a yielding deformable demoting faster than Jolt's default angular limit keeps its angular momentum"
    "a velocity edit on a promoted deformable moves its state's momentum by exactly the edit and leaves Jolt untouched"
    "a field box resting on a permanent impact window bar stays on it"
    "a demoted bent bar resting on the slab promotes without stored penalty energy"
    "a demoted bent bar's hull holds its stored surface and a bar demoted without plastic strain keeps its authored hull"
    "a field box resting on a permanent impact window bar is handed back at the window's end velocity with no violation line"
    "a struck field box handed back by a permanent impact window lands on the window's end pose"
    "a permanent impact window bar struck by a free field box keeps the system's momentum"
)
    add_test(
        NAME "${WINDOW_CASE}"
        COMMAND unit_tests "${WINDOW_CASE}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "${WINDOW_CASE}"
        PROPERTIES
            LABELS tools
            TIMEOUT 1200
            RUN_SERIAL TRUE
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
    )
endforeach()

add_library(spirv_reflect STATIC
    "$ENV{VULKAN_SDK}/Source/SPIRV-Reflect/spirv_reflect.c"
)
target_include_directories(spirv_reflect SYSTEM PUBLIC "$ENV{VULKAN_SDK}/Source/SPIRV-Reflect")
talus_module(spirv_reflect Developer)

add_executable(spirv_tests
    quietfailures.cc
    spirv_tests.cc
)
target_include_directories(spirv_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(spirv_tests PRIVATE talus::gizmos spirv_reflect talus::renderer talus::view talus::field talus::gpu Catch2::Catch2WithMain)
target_compile_definitions(spirv_tests PRIVATE TALUS_SPIRV_DIR="${CMAKE_BINARY_DIR}/shaders")
add_dependencies(spirv_tests SHADER_TARGET)
if (MSVC)
    target_compile_options(spirv_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(spirv_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
talus_module(spirv_tests Editor)

add_executable(image_tests
    quietfailures.cc
    image_tests.cc
)
target_link_libraries(image_tests PRIVATE talus::imagecompare Catch2::Catch2WithMain)
if (MSVC)
    target_compile_options(image_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(image_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
include(Catch)

catch_discover_tests(unit_tests DL_PATHS "$<TARGET_FILE_DIR:talus_c>")
catch_discover_tests(image_tests)
catch_discover_tests(spirv_tests)

set(GOLDEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/golden")
set(GOLDEN_GAME_DIR "${CMAKE_BINARY_DIR}/builds/demo")

file(GLOB DEMO_WORLD_FILES "${PROJECT_SOURCE_DIR}/projects/demo/worlds/*.world.json")
set(DEMO_WORLDS)
foreach(DEMO_WORLD_FILE IN LISTS DEMO_WORLD_FILES)
    get_filename_component(DEMO_WORLD "${DEMO_WORLD_FILE}" NAME)
    string(REGEX REPLACE "\\.world\\.json$" "" DEMO_WORLD "${DEMO_WORLD}")
    list(APPEND DEMO_WORLDS "${DEMO_WORLD}")
endforeach()
list(REMOVE_ITEM DEMO_WORLDS main)
list(PREPEND DEMO_WORLDS main)

add_test(
    NAME "package demo"
    COMMAND TalusPackager
            $<TARGET_FILE:TalusPlayer>
            "${CMAKE_BINARY_DIR}/projects/demo"
            "${GOLDEN_GAME_DIR}"
            ${DEMO_WORLDS}
)
set_tests_properties("package demo" PROPERTIES FIXTURES_SETUP packaged_demo)

function(add_golden_test name)
    add_test(
        NAME "golden ${name}"
        COMMAND "${GOLDEN_GAME_DIR}/demo.exe"
                --golden "${GOLDEN_DIR}/${name}.png" ${ARGN}
        WORKING_DIRECTORY "${GOLDEN_GAME_DIR}"
    )
    set_tests_properties(
        "golden ${name}" PROPERTIES FIXTURES_REQUIRED packaged_demo
    )
endfunction()
add_golden_test(field-sampled --world golden)
add_golden_test(field-analytic --world golden --analytic)

if (TALUS_TOOLS AND EXISTS "${PROJECT_SOURCE_DIR}/projects/demo/worlds/rain.world.json")
    add_executable(playerframes_tests
        quietfailures.cc
        playerframes_tests.cc
    )
    target_link_libraries(playerframes_tests PRIVATE talus::imagecompare Catch2::Catch2WithMain)
    target_compile_definitions(playerframes_tests PRIVATE
        TALUS_PLAYER_EXE="$<TARGET_FILE:TalusPlayer>"
        TALUS_DEMO_PROJECT="${CMAKE_BINARY_DIR}/projects/demo"
    )
    if (MSVC)
        target_compile_options(playerframes_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
    else()
        target_compile_options(playerframes_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    talus_module(playerframes_tests Developer)
    add_dependencies(playerframes_tests TalusPlayer TALUS_CONTENT)
    add_test(
        NAME "a screenshot run simulates its frames"
        COMMAND playerframes_tests
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties(
        "a screenshot run simulates its frames"
        PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE
    )
endif()

add_dependencies(unit_tests TALUS_MANAGED TALUS_SCRIPTS_demo)
target_compile_definitions(unit_tests PRIVATE TALUS_CAPI_DIR="${PROJECT_SOURCE_DIR}/engine/capi" TALUS_MANAGED_DIR="${CMAKE_BINARY_DIR}/managed" TALUS_SCRIPTS_DIR="${CMAKE_BINARY_DIR}/projects/demo/scripts")

talus_module(unit_tests Editor)
talus_module(image_tests Developer)

if (EXISTS "${PROJECT_SOURCE_DIR}/editor")
    add_custom_target(TALUS_QUERY_INTEGRATION ALL
        COMMAND "${TALUS_DOTNET}" build
                "${CMAKE_CURRENT_SOURCE_DIR}/queryintegration/QueryIntegration.csproj"
                -c $<IF:$<CONFIG:Release>,Release,Debug>
                -o "${CMAKE_BINARY_DIR}/tests/queryintegration"
                --nologo -v quiet
        JOB_POOL dotnet
        DEPENDS TALUS_MANAGED talus_c
        VERBATIM
    )
    add_test(
        NAME "query editor script integration"
        COMMAND "${TALUS_DOTNET}" "${CMAKE_BINARY_DIR}/tests/queryintegration/Scripts.dll" "${CMAKE_BINARY_DIR}" "$<TARGET_FILE:talus_c>"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("query editor script integration" PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)

    add_custom_target(TALUS_INSPECTOR_INTEGRATION ALL
        COMMAND "${TALUS_DOTNET}" build
                "${CMAKE_CURRENT_SOURCE_DIR}/inspectorintegration/InspectorIntegration.csproj"
                -c $<IF:$<CONFIG:Release>,Release,Debug>
                -o "${CMAKE_BINARY_DIR}/tests/inspectorintegration"
                --nologo -v quiet
        JOB_POOL dotnet
        DEPENDS talus_c
        VERBATIM
    )
    add_test(
        NAME "inspector editor integration"
        COMMAND "${TALUS_DOTNET}" "${CMAKE_BINARY_DIR}/tests/inspectorintegration/InspectorIntegration.dll" "${CMAKE_BINARY_DIR}" "$<TARGET_FILE:talus_c>"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("inspector editor integration" PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)

    add_custom_target(TALUS_SCRIPTS_INTEGRATION ALL
        COMMAND "${TALUS_DOTNET}" build
                "${CMAKE_CURRENT_SOURCE_DIR}/scriptsintegration/ScriptsIntegration.csproj"
                -c $<IF:$<CONFIG:Release>,Release,Debug>
                -o "${CMAKE_BINARY_DIR}/tests/scriptsintegration"
                --nologo -v quiet
        JOB_POOL dotnet
        DEPENDS talus_c
        VERBATIM
    )
    add_test(
        NAME "scripts editor integration"
        COMMAND "${TALUS_DOTNET}" "${CMAKE_BINARY_DIR}/tests/scriptsintegration/ScriptsIntegration.dll"
                "$<TARGET_FILE:talus_c>" "${PROJECT_SOURCE_DIR}/projects/demo"
                "${CMAKE_BINARY_DIR}/tests/scriptsintegration/demo"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("scripts editor integration" PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE)

    add_custom_target(TALUS_SHELL_INTEGRATION ALL
        COMMAND "${TALUS_DOTNET}" build
                "${CMAKE_CURRENT_SOURCE_DIR}/shellintegration/ShellIntegration.csproj"
                -c $<IF:$<CONFIG:Release>,Release,Debug>
                -o "${CMAKE_BINARY_DIR}/tests/shellintegration"
                "-p:TalusNativeDir=$<TARGET_FILE_DIR:talus_c>"
                --nologo -v quiet
        JOB_POOL dotnet
        DEPENDS talus_c
        VERBATIM
    )
    add_test(
        NAME "shell editor integration"
        COMMAND "${TALUS_DOTNET}" "${CMAKE_BINARY_DIR}/tests/shellintegration/ShellIntegration.dll"
                "$<TARGET_FILE:talus_c>"
                --project "${PROJECT_SOURCE_DIR}/projects/demo" --world main
                --copy "${CMAKE_BINARY_DIR}/tests/shellintegration/demo"
                --layout "${CMAKE_BINARY_DIR}/tests/shellintegration/layout.json"
                --shot "${CMAKE_BINARY_DIR}/tests/shellintegration/shell.png"
                --player "$<TARGET_FILE:TalusPlayer>"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("shell editor integration" PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE)

    add_test(
        NAME "shell deformable wireframe"
        COMMAND "${TALUS_DOTNET}" "${CMAKE_BINARY_DIR}/tests/shellintegration/ShellIntegration.dll"
                "$<TARGET_FILE:talus_c>"
                --wireframe
                --project "${PROJECT_SOURCE_DIR}/projects/demo" --world bending
                --copy "${CMAKE_BINARY_DIR}/tests/shellintegration/wireframe"
                --layout "${CMAKE_BINARY_DIR}/tests/shellintegration/wireframe-layout.json"
                --shot "${CMAKE_BINARY_DIR}/tests/shellintegration/wireframe.png"
                --player "$<TARGET_FILE:TalusPlayer>"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    )
    set_tests_properties("shell deformable wireframe" PROPERTIES TIMEOUT 240 RUN_SERIAL TRUE)
endif()


if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/campaign")
    foreach(CAMPAIGN_CASE
        "campaign: sleeping worlds hold every body within memory"
        "campaign: inactive objects cost little at constant activity"
        "campaign: 10k dropped boxes stay above the ground"
        "campaign: dropped hull and field piles settle and sleep"
        "campaign: a dropped field pile of 10k sleeps within 120 frames of landing"
        "campaign: an impact window at f 0.9 and 16 k0 fires the energy monitor"
        "campaign: the pile's invariant monitor prints each violation line"
        "campaign: the plastic pile prints its impact windows every frame"
    )
        add_test(
            NAME "${CAMPAIGN_CASE}"
            COMMAND campaign_benchmarks "${CAMPAIGN_CASE}"
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        )
        set_tests_properties(
            "${CAMPAIGN_CASE}"
            PROPERTIES
                LABELS campaign
                TIMEOUT 3600
                RUN_SERIAL TRUE
                ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:talus_c>"
        )
    endforeach()
endif()

