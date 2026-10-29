cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
set(CMAKE_PROJECT_NAME CtrBoard-H7_ALL)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/arm_board.c
    App/arm_profile.c
    App/arm_motor.c
    App/arm_protocol.c
    App/arm_initialization.c
    App/arm.c
    App/arm_auto.c
    App/arm_zero.c
    App/cmd.c
    App/gimbal_arm.c
    App/gimbal_cmd.c
    App/robot_state.c
    App/test/can_loopback_test.c
    App/test/uart_echo_test.c
    App/test/uart_loopback_test.c
    Bsp/BMI088driver.c
    Bsp/BMI088Middleware.c
    Bsp/bsp_buzzer.c
    Bsp/bsp_user_key.c
    Bsp/lcd.c
    Bsp/w25q64.c
    Bsp/ws2812.c
    Algorithm/MahonyAHRS.c
    Algorithm/user_lib.c
    vofa+/vofa.c
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
    libs/edge_trigger/edge_trigger_cont.c
    libs/encos_motor/ENCOS_motor.c
    libs/encos_motor/encos_mixed_control.c
    libs/encos_motor/encos_protocol.c
    libs/fast_math/fast_cos_f32.c
    libs/fast_math/fast_sin_f32.c
    libs/gravity_compensation/c/arm_dynamics.c
    libs/fps/bsp_supervise.c
    libs/monitor/monitor.c
    libs/move_plan/curves/cubic_spline.c
    libs/move_plan/curves/linear_spline.c
    libs/move_plan/curves/quintic_spline.c
    libs/move_plan/hal_move_plan.c
    libs/move_plan/move_plan/move_plan.c
    libs/move_plan/move_plan/plan_group.c
    libs/rc/rc.c
    libs/soft_crc/soft_crc.c
    libs/frame/frame.c
    libs/uart/bsp_uart.c
    libs/usb/bsp_usb.c
)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/move_plan/curves
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/move_plan/move_plan
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/rc
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/soft_crc
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/frame
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/usb
)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

add_subdirectory(App/profiles)
