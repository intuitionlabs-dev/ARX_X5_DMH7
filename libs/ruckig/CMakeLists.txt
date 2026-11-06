cmake_minimum_required(VERSION 3.22)
project(ruckig_c VERSION 0.15.3 LANGUAGES C CXX)

add_library(ruckig_c STATIC
    ruckig_c.cpp
    vendor/src/brake.cpp
    vendor/src/position_first_step1.cpp
    vendor/src/position_first_step2.cpp
    vendor/src/position_second_step1.cpp
    vendor/src/position_second_step2.cpp
    vendor/src/position_third_step1.cpp
    vendor/src/position_third_step2.cpp
    vendor/src/velocity_second_step1.cpp
    vendor/src/velocity_second_step2.cpp
    vendor/src/velocity_third_step1.cpp
    vendor/src/velocity_third_step2.cpp
)
add_library(ruckig::c ALIAS ruckig_c)
target_compile_features(ruckig_c PRIVATE cxx_std_17)
target_include_directories(ruckig_c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor/include)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ruckig_c PRIVATE -fno-exceptions -fno-rtti
        -Werror=double-promotion -Werror=float-conversion)
endif()

option(RUCKIG_C_BUILD_TESTS "Build standalone C API tests and example" OFF)
if(RUCKIG_C_BUILD_TESTS)
    enable_testing()
    add_executable(ruckig_c_test tests/test_ruckig_c.c)
    target_link_libraries(ruckig_c_test PRIVATE ruckig_c)
    # Keep runtime checks enabled in Release builds too.
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(ruckig_c_test PRIVATE -UNDEBUG)
    endif()
    add_test(NAME ruckig_c_test COMMAND ruckig_c_test)
    add_executable(ruckig_c_example examples/position.c)
    target_link_libraries(ruckig_c_example PRIVATE ruckig_c)
    add_test(NAME ruckig_c_example COMMAND ruckig_c_example)
endif()
