"""Metal Arm 动力学补偿参数，与 SDK config.py 一致。"""
GRAVITY_SCALES = (1.2, 1.15, 1.1, 1.15, 1.0, 1.0)
CORIOLIS_SCALES = (1.1, 1.15, 1.0, 1.0, 1.1, 1.1)
FRICTION_COEFFICIENTS = (0.15, 0.3025, 0.52, 0.42, 0.015, 0.015)
FRICTION_SPEED_LIMITS = (None, None, 0.8, 1.2, None, None)
