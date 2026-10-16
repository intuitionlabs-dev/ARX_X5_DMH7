"""Parameters from App/inc/*_def.h and Gimbal_Create/pid_init at 21b97f9."""

from dataclasses import dataclass

from .numerics import PIDConfig
from .types import finite_tuple

JOINT_MIN = (-115.0, 0.5, 0.5, -90.0, -90.0, -112.87)
JOINT_MAX = (90.0, 206.26, 171.88, 90.0, 90.0, 119.17)
GRIPPER_MIN, GRIPPER_MAX = -142.0, 142.0
RESET_ANGLE = (0.0, 0.5, 0.5, 0.0, 0.0, 0.0)
TD_R = (1000.0, 700.0, 700.0, 2000.0, 2000.0, 2000.0)
MOTOR_IDS = (1, 2, 4, 5, 6, 7, 8)
TX_GROUPS = (0x55, 0x22, 0x22, 0x44, 0x44, 0x88, 0x88)
MOTOR_KP = (0.0, 0.0, 0.0, 50.0, 25.0, 10.0, 8.0)
MOTOR_KD = (35.0, 50.0, 50.0, 0.8, 0.8, 1.0, 0.1)
POSITION_PID = tuple(PIDConfig(kp, ki, 50, 200, 1000, irange) for kp, ki, irange in (
    (6, 0.08, 1), (4, 0, 1), (4, 0, 1), (6, 0.08, 0),
    (6, 0.08, 0), (6, 0.08, 0), (6, 0.08, 0)))
SPEED_PID = tuple(PIDConfig(output_limit=12 if i < 3 else 10) for i in range(7))
ZERO_PID = PIDConfig()


@dataclass(frozen=True)
class ArmConfig:
    direction: tuple[float, ...] = (1.0,) * 6
    reduction: tuple[float, ...] = (1.0,) * 6
    position_zero_deg: tuple[float, ...] = (0.0,) * 6
    torque_scale: tuple[float, ...] = (1.0,) * 6
    viscous_damping: tuple[float, ...] = (0.0,) * 6
    velocity_scale: tuple[float, ...] = (1.0,) * 6
    gravity_enabled: bool = True
    gravity_model: str = "2023"
    gravity_sdk_scale: bool = True
    gravity_base: tuple[float, ...] = (0.0, 0.0, -9.81)
    calibrate_encos: bool = True

    def __post_init__(self):
        for name in ("direction", "reduction", "position_zero_deg", "torque_scale",
                     "viscous_damping", "velocity_scale", "gravity_base"):
            object.__setattr__(self, name, finite_tuple(getattr(self, name),
                               3 if name == "gravity_base" else 6, name))
        if any(v not in (-1, 1) for v in self.direction):
            raise ValueError("motor directions must be +1 or -1")
        if any(v <= 0 for v in self.reduction):
            raise ValueError("motor reductions must be positive")
        if self.gravity_model not in ("2023", "master", "2025"):
            raise ValueError("unknown X5 gravity model")
