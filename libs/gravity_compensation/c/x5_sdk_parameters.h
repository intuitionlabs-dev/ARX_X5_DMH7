/* 只读单精度 SDK 参数表，定义关节、协议和控制标定数据。 */
#ifndef X5_SDK_PARAMETERS_H
#define X5_SDK_PARAMETERS_H

#include <stdint.h>

#define X5_SDK_JOINT_COUNT 6
#define X5_SDK_MOTOR_COUNT 7
#define X5_SDK_PROTOCOL_COUNT 2
#define X5_SDK_HOME_PROFILE_COUNT 3

enum {
    X5_SDK_HOME_DEFAULT = 0,
    X5_SDK_HOME_ROS1_REMOTE_MASTER = 1,
    X5_SDK_HOME_ROS2_V2_COLLECT = 2
};

typedef struct {
    float position_min_rad, position_max_rad, home_rad;
    float position_kp, position_kd, position_ki, protect_kd;
    float gravity_scale;
} x5_sdk_joint_parameters;

typedef struct {
    uint8_t can_id, sdk_motor_type, protocol_index, current_check_active;
    uint32_t timeout_us;

    float feedback_current_threshold;
} x5_sdk_motor_parameters;

typedef struct {
    float position_min_rad, position_max_rad;
    float velocity_min_rad_s, velocity_max_rad_s;
    float effort_min, effort_max;
    float kp_min, kp_max, kd_min, kd_max;
    uint8_t sdk_motor_type;

    uint8_t quantization_bits[5], packed_bits[5];
    uint8_t command_dlc;
} x5_sdk_protocol_parameters;

typedef struct {
    float integral_limit, integral_divisor;
    float home_acceleration, home_speed;
    float home_position_tolerance_rad, home_velocity_tolerance_rad_s;
    float position_acceleration, position_speed;

    float interpolation_argument3, interpolation_actual_dt;
    float gripper_home_done_kd;
    uint16_t overcurrent_samples, gripper_home_samples, write_motor_delay_us;
} x5_sdk_control_parameters;

typedef struct {
    /* Inactive/unknown numeric fields are zero; consult flags before use. */
    uint8_t position_limit_active, contact_check_active;
    uint16_t contact_samples;
    float position_min_rad, position_max_rad;
    float kp, kd, effort_min, effort_max, error_gain, position_bias;
    float home_velocity_rad_s, home_kd;
    float contact_feedback_threshold, contact_position_offset_rad;
} x5_sdk_gripper_parameters;

typedef struct {
    float offset_rad, feedback_min_rad, feedback_max_rad;
    float unwrap_jump_threshold_rad, unwrap_span_rad, command_wrap_span_rad;
    float feedback_recenter_span_rad;
} x5_sdk_type4_position_parameters;

typedef struct {
    float position_min_rad, position_max_rad, effort, velocity;
} x5_sdk_urdf_limit_parameters;

#ifdef __cplusplus
extern "C" {
#endif

extern const x5_sdk_joint_parameters x5_sdk_joints[6];

extern const x5_sdk_motor_parameters x5_sdk_motors[7];

extern const x5_sdk_protocol_parameters x5_sdk_protocols[2];
extern const x5_sdk_control_parameters x5_sdk_control;

extern const float x5_sdk_home_profiles[3][6];

extern const x5_sdk_gripper_parameters x5_sdk_gripper_profiles[2];
extern const x5_sdk_type4_position_parameters x5_sdk_type4_position;
extern const x5_sdk_urdf_limit_parameters x5_sdk_urdf_limits;
/* xyz (m), rpy (rad); these are not joint limits. */
extern const float x5_sdk_cartesian_min[6], x5_sdk_cartesian_max[6];

#define X5_SDK_GRIPPER_HOME_IS_RUNTIME 1

#ifdef __cplusplus
}
#endif
#endif
