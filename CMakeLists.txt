cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CtrBoard-H7_ALL)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Application
    App/cmd.c
    App/gimbal_arm.c
    App/test/can_loopback_test.c

    # Board support
    Bsp/BMI088driver.c
    Bsp/BMI088Middleware.c
    Bsp/bsp_buzzer.c
    Bsp/bsp_user_key.c
    # Bsp/can_bsp.c is intentionally NOT built: libs/can/bsp_can.c replaces it,
    # and both define HAL_FDCAN_RxFifo0Callback (duplicate symbol if both link).
    Bsp/lcd.c
    Bsp/w25q64.c
    Bsp/ws2812.c

    # Algorithms
    Algorithm/MahonyAHRS.c
    Algorithm/user_lib.c

    # Debug / telemetry
    vofa+/vofa.c

    # libs/ (ported from the upstream intuition library)
    libs/can/bsp_can.c
    libs/common/common.c
    libs/controller/adrc.c
    libs/controller/basic_controller.c
    libs/controller/controller.c
    libs/controller/lqr.c
    libs/controller/mpc.c
    libs/controller/mrac.c
    libs/controller/pid.c
    libs/controller/smc.c
    libs/cvector/cvector.c
    libs/dm_motor/DM_motor.c
    libs/dm_motor/dm_mit_control.c
    libs/fps/bsp_supervise.c
    libs/monitor/monitor.c
    libs/move_plan/hal_move_plan.c
    libs/rc/rc.c

    # NOTE: Core/Src/freertos.c and the FreeRTOS kernel (cmsis_os2, port, heap_4)
    # are added by cmake/stm32cubemx/CMakeLists.txt, which CubeMX regenerates.
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/App/test
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/vofa+

    ${CMAKE_CURRENT_SOURCE_DIR}/libs/can
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/common
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/controller
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/cvector
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/dm_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/fps
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/monitor
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/move_plan
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/rc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# 机械臂的指令入口 Gimbal_Set_Control() 目前工程里还没有调用方（遥控器 -> 关节角
# 的映射没做），-ffunction-sections + -Wl,--gc-sections 会把它整个删掉，连 JLink
# 都调不到。--undefined 强制把它当成 GC 根保留下来（函数上的 __attribute__((used))
# 在 GCC 10 上不生成 SHF_GNU_RETAIN，拦不住 --gc-sections，所以只能从链接器这边来)。
# 等有真实调用方之后这行可以删。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--undefined=Gimbal_Set_Control
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
