add_library(talus_deformable STATIC
    contact/contact.cc
    factor/factor.cc
    impactwindow/impactwindow.cc
    material/material.cc
    solver/solver.cc
)
add_library(talus::deformable ALIAS talus_deformable)

target_include_directories(talus_deformable PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_compile_features(talus_deformable PUBLIC cxx_std_23)

target_link_libraries(talus_deformable PUBLIC
    talus::core
    talus::field
    talus::objectmodel
    glm::glm
    eigen
)
if (MSVC)
    target_compile_options(talus_deformable PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_deformable PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_deformable Runtime)
