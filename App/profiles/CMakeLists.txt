# 查找模型生成所需的 Python 解释器。
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# 设置生成模型头文件的构建目录。
set(ARM_MODEL_DIR "${PROJECT_BINARY_DIR}/arm_models")

# 登记机型源码、URDF 模型生成规则和配置符号。
function(arm_register_profile NAME SYMBOL SOURCE URDF BASE TIP MODEL_SYMBOL HEADER)
    set(MODEL_PATH "${PROJECT_SOURCE_DIR}/libs/gravity_compensation/models/${URDF}")
    # 按 URDF 及导出器依赖生成对应模型头文件。
    add_custom_command(
        OUTPUT "${ARM_MODEL_DIR}/${HEADER}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${ARM_MODEL_DIR}"
        COMMAND "${Python3_EXECUTABLE}"
            "${PROJECT_SOURCE_DIR}/libs/gravity_compensation/c/generate_arm_model.py"
            --urdf "${MODEL_PATH}" --base "${BASE}" --tip "${TIP}"
            --symbol "${MODEL_SYMBOL}" --output "${ARM_MODEL_DIR}/${HEADER}"
        DEPENDS "${MODEL_PATH}"
            "${PROJECT_SOURCE_DIR}/libs/gravity_compensation/c/generate_arm_model.py"
            "${PROJECT_SOURCE_DIR}/libs/gravity_compensation/c/generate_dynamics_data.py"
            "${PROJECT_SOURCE_DIR}/libs/gravity_compensation/urdf.py"
            "${PROJECT_SOURCE_DIR}/libs/gravity_compensation/spatial.py"
        VERBATIM
    )
    # 将机型源码和生成模型加入固件目标。
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}"
        "${ARM_MODEL_DIR}/${HEADER}")
    # 登记机型名称到 C 配置符号的映射。
    set_property(GLOBAL PROPERTY "H7_PROFILE_${NAME}" "${SYMBOL}")
    set_property(GLOBAL APPEND PROPERTY H7_PROFILE_NAMES "${NAME}")
endfunction()

# 登记 metalarm 的配置与动力学模型。
arm_register_profile(metalarm ARM_PROFILE_METAL metal.c metal_with_gripper.urdf base_link Link6 metal_links metal_model.h)
# 登记 ARX-X5 的配置与动力学模型。
arm_register_profile(arx_x5 ARM_PROFILE_ARX_X5 arx_x5.c x5.urdf base_link link6 x5_links x5_model.h)
# 配置固件目标的生成模型头文件目录。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${ARM_MODEL_DIR}")
get_property(ARM_PROFILE_NAMES GLOBAL PROPERTY H7_PROFILE_NAMES)

# 分别解析 A、B 臂机型选择并传递配置符号。
foreach(ARM_SLOT A B)
    set(H7_ARM_${ARM_SLOT}_PROFILE "metalarm" CACHE STRING "机械臂 ${ARM_SLOT} 机型")
    set_property(CACHE H7_ARM_${ARM_SLOT}_PROFILE PROPERTY STRINGS ${ARM_PROFILE_NAMES})
    get_property(ARM_PROFILE_SYMBOL GLOBAL PROPERTY "H7_PROFILE_${H7_ARM_${ARM_SLOT}_PROFILE}")
    # 机型未登记时在配置阶段报告错误。
    if(NOT ARM_PROFILE_SYMBOL)
        message(FATAL_ERROR "无效机型: ${H7_ARM_${ARM_SLOT}_PROFILE}")
    endif()
    # 将所选机型符号传给板级绑定源码。
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ARM_${ARM_SLOT}_PROFILE=${ARM_PROFILE_SYMBOL})
endforeach()
