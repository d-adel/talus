add_library(talus_field STATIC
    analytic/analytic.cc
    crossings/crossings.cc
    deform/deform.cc
    raycast/raycast.cc
    redistance/redistance.cc
    sampledsdf/sampledsdf.cc
    primitive.cc
)
add_library(talus::field ALIAS talus_field)

target_include_directories(talus_field PUBLIC "${PROJECT_SOURCE_DIR}/engine")

target_link_libraries(talus_field PUBLIC glm::glm)

target_compile_features(talus_field PUBLIC cxx_std_23)
target_compile_definitions(talus_field PUBLIC
    TALUS_MARCH_EPSILON=${TALUS_MARCH_EPSILON}
    TALUS_MARCH_MAX_ITERATIONS=${TALUS_MARCH_MAX_ITERATIONS}
    TALUS_MARCH_MAX_DISTANCE=${TALUS_MARCH_MAX_DISTANCE}
    TALUS_GRADIENT_EPSILON=${TALUS_GRADIENT_EPSILON}
    TALUS_MARCH_OMEGA=${TALUS_MARCH_OMEGA}
    TALUS_SHADOW_MAX_DISTANCE=${TALUS_SHADOW_MAX_DISTANCE}
    TALUS_SHADOW_SHARPNESS=${TALUS_SHADOW_SHARPNESS}
    TALUS_BRICK_CELLS=${TALUS_BRICK_CELLS}
    TALUS_SHAPE_SPHERE=${TALUS_SHAPE_SPHERE}
    TALUS_SHAPE_BOX=${TALUS_SHAPE_BOX}
    TALUS_SHAPE_PLANE=${TALUS_SHAPE_PLANE}
    TALUS_SHAPE_CYLINDER=${TALUS_SHAPE_CYLINDER}
    TALUS_CSG_UNION=${TALUS_CSG_UNION}
    TALUS_CSG_SMOOTH_UNION=${TALUS_CSG_SMOOTH_UNION}
    TALUS_CSG_INTERSECTION=${TALUS_CSG_INTERSECTION}
    TALUS_CSG_SUBTRACTION=${TALUS_CSG_SUBTRACTION}
    TALUS_CSG_SMOOTH_SUBTRACTION=${TALUS_CSG_SMOOTH_SUBTRACTION}
)
if (MSVC)
    target_compile_options(talus_field PRIVATE /W4 /WX /w15038 /w14062 /Zc:__cplusplus)
else()
    target_compile_options(talus_field PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

talus_module(talus_field Runtime)
