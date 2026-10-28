"""Metal 测试使用的六轴及夹爪反馈。"""
from math import radians
from metal_arm_control import MotorFeedback

def samples(now, arms=1, positions=(0, 0, 0, 0, 0, 0, 0), **kwargs):
    return tuple(MotorFeedback(a, i, now, position=radians(p), velocity=0, **kwargs)
                 for a in range(arms) for i, p in enumerate(positions))
