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
    App/arm_auto.c
    App/arm_zero.c
    App/cmd.c
    App/gimbal_arm.c
    App/gimbal_cmd.c
    App/robot_state.c
    App/test/can_loopback_test.c
    App/test/uart_echo_test.c
    App/test/uart_loopback_test.c

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
    # 源工程这个目录下还有 edge_trigger_bin.c / edge_trigger_bin_pack.c(二值量的
    # 边沿触发)，本工程零调用方，按老规矩不取(同 fast_math 的 fast_matrix.c)。
    libs/edge_trigger/edge_trigger_cont.c
    libs/encos_motor/ENCOS_motor.c
    libs/encos_motor/encos_mixed_control.c
    libs/encos_motor/encos_protocol.c
    libs/fast_math/fast_cos_f32.c
    # 源工程还有个 fast_matrix.c（fast_mat_* 那套矩阵运算），只有 robotics/ 那个
    # C++ 矩阵封装在用，本工程没移植那个框架，所以这个文件没有拿过来。
    libs/fast_math/fast_sin_f32.c
    # X5 静态重力力矩，纯 C99、零依赖(x5_gravity.h 里说得很明白：不调 libm、不用堆、
    # 没有可变的全局状态)。同目录的 x5_sdk_parameters.c 只是参数表，本工程当文档用
    # (gimbal_arm.c 里照着它填 ENCOS/达妙的量程)，零调用方，按老规矩不取。
    # 许可证是 LGPL-2.1-or-later(见各文件头的 SPDX)，静态链接进固件没问题，
    # 源文件原位留在仓库里就满足"可重新链接"的要求。
    libs/gravity_compensation/c/x5_gravity.c
    libs/fps/bsp_supervise.c
    libs/monitor/monitor.c
    libs/move_plan/curves/cubic_spline.c
    libs/move_plan/curves/linear_spline.c
    libs/move_plan/curves/quintic_spline.c
    libs/move_plan/hal_move_plan.c
    libs/move_plan/move_plan/move_plan.c
    libs/move_plan/move_plan/plan_group.c
    libs/rc/rc.c
    # 串口帧协议 + 它依赖的 Modbus CRC8/CRC16。两个库都是新加的：soft_crc 只被
    # 串口协议层(切帧/CRC/码表) + 它依赖的查表 CRC。
    # 两层的真实调用方是 App/robot_state.c(总模式状态机)；测试关掉时它也在跑，
    # 所以 --gc-sections 现在**不会**再摘掉它们(以前会，那时只有回环测试在用)。
    libs/soft_crc/soft_crc.c
    # 帧层(2026-10-08 由 uart_frame 改名而来，因为它现在两条链路都要用)。和传输无关：
    # 只负责切帧/校验/码表，字节从哪来、往哪去是上层的事。
    libs/frame/frame.c
    # 串口的传输层(DMA 收发 + .RAM_D1 缓冲都封在它里面，上层不用管内存段)。
    # 调用方：App/robot_state.c(ROBOT_LINK 选 UART 时走 USART3) + 回环测试(USART2/3，测试关掉时没有)。
    libs/uart/bsp_uart.c
    # USB CDC 的传输层。接口形状和 libs/uart **一样**(对照表在 libs/usb/bsp_usb.h 开头)，
    # 调用方也是 App/robot_state.c —— ROBOT_LINK 选 USB 时走它，选串口时它只是编进来但
    # 没人注册回调(见 App/inc/robot_state.h 顶部那段)。
    # 它依赖的 USB_DEVICE/* 和 ST 的 USB 设备中间件由 CubeMX 生成的
    # cmake/stm32cubemx/CMakeLists.txt 负责编进来，这边不用管。
    libs/usb/bsp_usb.c

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
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/edge_trigger
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/encos_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/fast_math
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/fps
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/gravity_compensation/c
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/monitor
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/move_plan
    # 源工程是把 lib/ 下每个子目录都塞进 include path 的
    # （见其 hal_lib/CMakeLists.txt 的 file(GLOB LIB_DIRS lib/*)），本工程是一个个手写，
    # 所以这两个子目录要显式加：move_plan.h 里 include 的是 "cubic_spline.h" 这种裸文件名。
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/move_plan/curves
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/move_plan/move_plan
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/rc
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/soft_crc
    # 串口传输层(BSP)。只有它自己的 .c include uart_config.h，但上层要调 BSP_UART_*，
    # 所以这个目录必须在 include path 里。
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/uart
    # frame.c 里 include 的是 "soft_crc.h" 这种裸文件名，两个目录都要在
    # include path 里(同 libs/move_plan 那两条注释说的道理)。
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/frame
    # USB 传输层(BSP)。上层要调 BSP_USB_*，所以这个目录也得在 include path 里。
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/usb
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Move_Plan_Temp_Create() 目前还没有业务代码调它：hal_move_plan.c 接上以后
# Move_Plan_Update() 是活的，但造轨迹这个入口只会被同样没人调的
# Plan_Group_Temp_Calc_And_Run() 引用，-ffunction-sections + -Wl,--gc-sections 会把它
# 整个删掉，烧进板子后连 JLink 都调不到（没法验证规划器算得对不对）。--undefined 强制
# 把它当成 GC 根保留下来（函数上的 __attribute__((used)) 在 GCC 10 上不生成
# SHF_GNU_RETAIN，拦不住 --gc-sections，所以只能从链接器这边来）。等有真实调用方之后
# 这行可以删。
#
# (Gimbal_Set_Control() 原来也在这张表里。那个函数连同"喂指令"这套接口一起删掉了 ——
#  命令层改成直接写 Gimbal.arm_control_data，所以这一项早就没用了。)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,--undefined=Move_Plan_Temp_Create
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
