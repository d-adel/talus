add_library(talus_stb STATIC
    stb_impl.cc
)
add_library(talus::stb ALIAS talus_stb)
target_include_directories(talus_stb PUBLIC "${stb_SOURCE_DIR}")
target_compile_features(talus_stb PUBLIC cxx_std_23)

talus_module(talus_stb Runtime)
