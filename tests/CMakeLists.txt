FetchContent_Declare(Catch2
    URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.7.1.tar.gz
    URL_HASH SHA256=c991b247a1a0d7bb9c39aa35faf0fe9e19764213f28ffba3109388e62ee0269c
    SYSTEM)
FetchContent_MakeAvailable(Catch2)

add_executable(talus_tests
    dynamics_tests.cc dynamics_rows_tests.cc dynamics_softstep_rows_tests.cc
    dynamics_strainstep_tests.cc dynamics_world_tests.cc dynamics_compound_tests.cc dynamics_joint_tests.cc dynamics_ray_tests.cc dynamics_mover_tests.cc dynamics_sleepcost_tests.cc dynamics_awake_tests.cc dynamics_fieldcompound_tests.cc
    fieldbody_tests.cc deformedfield_tests.cc destruction_tests.cc
    destruction_factor_tests.cc destructionworld_tests.cc budget_tests.cc
    neohookean_tests.cc destructionfixtures.cc pinning.cc quietfailures.cc
    fixtures/district.cc fixtures/humanoid.cc fixtures/mover.cc integration_tests.cc)
target_link_libraries(talus_tests PRIVATE Talus::Physics Catch2::Catch2WithMain)
target_include_directories(talus_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
if (MSVC)
    target_compile_options(talus_tests PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus /Zc:preprocessor)
else()
    target_compile_options(talus_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -ffp-contract=off)
endif()
add_test(NAME talus.physics COMMAND talus_tests)
set_tests_properties(talus.physics PROPERTIES TIMEOUT 180)
if (TALUS_BUILD_EXAMPLES)
    add_test(NAME talus.headless COMMAND talus_headless)
endif()
