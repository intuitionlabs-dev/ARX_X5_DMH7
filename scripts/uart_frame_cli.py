#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂上位机控制台（curses 全屏面板）。UART 和 USB 两条链路都能跑。

**协议和传输无关**：帧格式、码表、版本码两条链路完全一样，所以这个脚本不区分
"串口版/ USB 版"—— 只按 USB 身份去认口子，认出哪条就用哪条（见 pick_default_port）。
板子那侧走哪条是**编译期**定死的（App/inc/robot_state.h 的 ROBOT_LINK），两边选反了
的现象是"一切都对但一个字节都没有"。

按 libs/frame/protocol_doc.md 的线协议和板子说话：

  * **周期发 0x16**（6 个关节角 + 夹爪角，deg）当控制帧，**它同时就是保活帧** ——
    板子那边 500 ms 没收到任意有效帧就进 comm_lost(damping)，而被丢掉的 0x16 也算
    有效帧（robot_state.c 的 s_last_rx_ms 在分派之前刷新），所以任何时候发都不亏。
  * **收 0x80 反馈帧**显示 mode / robot_state / 关节角 / 掉线掩码 / 夹爪角。
  * 按键步进关节目标角，目标角按 App/inc/gimbal_cmd_def.h 的软限位在**上位机**先夹住：
    板子也会夹（gimbal_cmd.c），但上位机不夹的话面板上显示的目标角就是骗人的。

为什么控制只用 0x16 不用 0x17（速度前馈）：本工程的位置环只要目标角就够；而且 j1(ENCOS)
的 kp=0/kd=35，速度前馈在那台电机上等效于一个恒定的力矩偏置（见 memory
j1-integral-window-and-ff），调试时反而不好解释。需要时再扩。

会话流程（抄 protocol_doc.md，但按板子源码核过）：
    ① 等握手（板子只认 0x01 版本包，版本码不符会**静默锁死**，只能复位板子）
    ② 离线等待（7 台电机全上线后板子自己走）
    ③ 待初始化 —— 按 i 发 0x02
    ④ 初始化中（板子自己跑一键复位 → 一键夹爪找零，约 7 s，期间必须保持保活）
    ⑤ 正常运行（**默认 damping**！按 s 发 0x12 才进 run，run 下 0x16 才被接受）
       ↑ 这一档（stop/damping）下**也能按 i 重跑一遍初始化**；跑完回 ⑤ 而不是 ③。
         （板侧闸门放开是用户要求的口径；run 档下按 i 仍会被静默丢弃。）
    ⑥ 按 z 发 0x10 设零点 → 校准，跑完回 ③
    ⑦ 500 ms 无有效帧 → comm_lost；再来一帧就回 ②
上层会用、脚本里没做的：0x11 设零回复（收了，会显示）、0x17（不做）。

用法：
    python3 scripts/uart_frame_cli.py                # 真机：自动挑口子（板子 USB 优先，
                                                     # 退回 1a86 RS485 适配器），921600
    python3 scripts/uart_frame_cli.py --port /dev/ttyACM0   # 指定口子
    python3 scripts/uart_frame_cli.py --arm-id 1     # 控制 CAN2 上那台（默认 0 = CAN1）
    python3 scripts/uart_frame_cli.py --fake         # 假板子，不接硬件就能把面板/按键/状态机走一遍
    python3 scripts/uart_frame_cli.py --selftest     # 协议层离线自测（CRC/版本码/编解码/重同步）
面板里按 h 看按键表。

**控制哪一台臂由启动参数 --arm-id 定死，一个进程只驱动一台。** 板子是双臂共用一条
上位机链路的：所有命令帧和反馈帧在帧头都带一个 arm_id 字节（0=CAN1/臂A，1=CAN2/臂B），
本进程发出的每一帧都盖自己的 arm_id，收到的帧里不是这台臂的一律不解读。于是
`--arm-id 0` / `--arm-id 1` 就是两个**完全独立**的操作手（目标角、握手状态、事件流、
500 ms 看门狗互不影响）—— 就像各自插了一块板子，换一台只要重启一次脚本。

**但一次只能开一个**：板子那侧的上位机链路是**编译期**二选一的一路（robot_state.h 的
ROBOT_LINK），物理上就这么一个口子，两个进程同时开同一个 tty 会互相抢字节、把帧拆散。
要看另一台，退出重开、带上另一个 --arm-id 就行。想在一屏里同时看/同时动两台，
那是双列面板那件事，不在这里。

**安全**：默认退出**不发** 0x14（停止），只停发 0x16 —— 目标冻结、臂保持原样，
约 500 ms 后板子自己因无帧进 damping。要立刻松劲请手动按 x，或加 --stop-on-exit。
"""

import argparse
import curses
import glob
import locale
import os
import re
import struct
import sys
import threading
import time
from collections import deque

# ============================================================================
# A 层：协议层（纯函数，可以离线 --selftest）
# ============================================================================

# crc16 = CRC-16/MODBUS(多项式 0x8408 反射式, 初值 0xFFFF)、crc8 = CRC-8/MAXIM(0x31, 初值 0xFF)。
# 两张表是从 libs/soft_crc/soft_crc.c 抠出来的，并且和板上 FrameEncode() 的输出
# **逐字节比对**过，别手改。
T16 = [0, 4489, 8978, 12955, 17956, 22445, 25910, 29887, 35912, 40385, 44890, 48851, 51820, 56293, 59774, 63735, 4225, 264, 13203, 8730, 22181, 18220, 30135, 25662, 40137, 36160, 49115, 44626, 56045, 52068, 63999, 59510, 8450, 12427, 528, 5017, 26406, 30383, 17460, 21949, 44362, 48323, 36440, 40913, 60270, 64231, 51324, 55797, 12675, 8202, 4753, 792, 30631, 26158, 21685, 17724, 48587, 44098, 40665, 36688, 64495, 60006, 55549, 51572, 16900, 21389, 24854, 28831, 1056, 5545, 10034, 14011, 52812, 57285, 60766, 64727, 34920, 39393, 43898, 47859, 21125, 17164, 29079, 24606, 5281, 1320, 14259, 9786, 57037, 53060, 64991, 60502, 39145, 35168, 48123, 43634, 25350, 29327, 16404, 20893, 9506, 13483, 1584, 6073, 61262, 65223, 52316, 56789, 43370, 47331, 35448, 39921, 29575, 25102, 20629, 16668, 13731, 9258, 5809, 1848, 65487, 60998, 56541, 52564, 47595, 43106, 39673, 35696, 33800, 38273, 42778, 46739, 49708, 54181, 57662, 61623, 2112, 6601, 11090, 15067, 20068, 24557, 28022, 31999, 38025, 34048, 47003, 42514, 53933, 49956, 61887, 57398, 6337, 2376, 15315, 10842, 24293, 20332, 32247, 27774, 42250, 46211, 34328, 38801, 58158, 62119, 49212, 53685, 10562, 14539, 2640, 7129, 28518, 32495, 19572, 24061, 46475, 41986, 38553, 34576, 62383, 57894, 53437, 49460, 14787, 10314, 6865, 2904, 32743, 28270, 23797, 19836, 50700, 55173, 58654, 62615, 32808, 37281, 41786, 45747, 19012, 23501, 26966, 30943, 3168, 7657, 12146, 16123, 54925, 50948, 62879, 58390, 37033, 33056, 46011, 41522, 23237, 19276, 31191, 26718, 7393, 3432, 16371, 11898, 59150, 63111, 50204, 54677, 41258, 45219, 33336, 37809, 27462, 31439, 18516, 23005, 11618, 15595, 3696, 8185, 63375, 58886, 54429, 50452, 45483, 40994, 37561, 33584, 31687, 27214, 22741, 18780, 15843, 11370, 7921, 3960]
T8 = [0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53]


def crc16(data):
    c = 0xFFFF
    for x in data:
        c = (c >> 8) ^ T16[(c ^ x) & 0xFF]
    return c & 0xFFFF


def crc8(data):
    c = 0xFF
    for x in data:
        c = T8[(c ^ x) & 0xFF]
    return c & 0xFF


def fnv1a32(s):
    """板子算版本码用的就是 FNV-1a 32（frame.c）。"""
    h = 0x811C9DC5
    for b in s.encode("utf-8"):
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


# 版本串抄自 libs/frame/frame_config.h 的 FRAME_VERSION_STRING。
# **版本码不硬写**，开机自己算 —— 板子上改了串这里就跟上，不用记 0xF91DD3C9。
#
# v0.2：帧头在 datalen 之后加了一字节 arm_id（payload 起点 5→6、整帧开销 7→8）。
# 这是**不兼容改版**，所以版本串必须跟着改 —— 一段 v0.1 的上位机拿 v0.2 的帧会整体
# 错位，换码能让它在 0x01 上就被板子挡下来锁死，而不是拿旧布局去解析新帧。
VERSION_STRING = "Intuition-ARX-X5-2023-v0.2"
VERSION_CODE = fnv1a32(VERSION_STRING)

# 一块板子上有几台机械臂（板上 App/inc/robot_def.h 的 ARM_NUM）。
# arm_id 就是 CAN 总线索引：0 = CAN1(臂A)、1 = CAN2(臂B)。
N_ARM = 2
# **一个进程只控制一台臂**，哪一台在启动时用 --arm-id 定死（见 parse_arm_id）。
# 这样面板/会话/反馈全链路都只有一个 arm_id，不用在运行时切换 —— 切换意味着
# 目标角、握手状态、事件流全要跟着换一份，那是"两台双列显示"那件事，不是这个开关。
DEFAULT_ARM_ID = 0


def arm_label(arm_id):
    """面板/日志里标一台臂的统一写法。"""
    return "臂%s(arm_id=%d, CAN%d)" % ("AB"[arm_id] if 0 <= arm_id < len("AB") else "?",
                                       arm_id, arm_id + 1)


def parse_arm_id(s):
    """--arm-id 的取值校验。

    板子侧只认 arm_id < ARM_NUM，越界的帧会被**静默丢弃**（连版本包都不回），
    现象就是"什么都不动、也没有字节" —— 所以这里宁可启动时直接报错，
    别让它变成一个要靠抓包才能诊断的沉默失败。
    """
    try:
        v = int(s, 0)
    except ValueError:
        raise argparse.ArgumentTypeError("--arm-id 要是整数（0..%d）" % (N_ARM - 1))
    if not 0 <= v < N_ARM:
        raise argparse.ArgumentTypeError("--arm-id 只能是 0..%d（板上 ARM_NUM=%d）"
                                         % (N_ARM - 1, N_ARM))
    return v

# 码表（libs/frame/type_id.h）
TID_VERSION = 0x01        # 版本，双向
TID_ARM_INIT = 0x02       # 初始化（空 payload）。两个入口：「待初始化」，或「正常运行」的
                          # stop/damping 档（见 robot_state.c 的 HandleArmInit）
TID_MOTOR_SET_ZERO = 0x10  # 设零点（空 payload）
TID_SET_ZERO_ACK = 0x11   # 设零回复：u8 state + u8 result[7] + u32 elapsed_ms = 12 B
TID_ARM_START = 0x12      # 进 run
TID_ARM_GRAVITY = 0x13    # 重力补偿
TID_ARM_STOP = 0x14       # 停止（松劲）
TID_ARM_DAMPING = 0x15    # 阻尼
TID_ARM_CTRL_POS = 0x16   # 位置控制：float joint_angle[6] + float gripper_angle = 28 B
TID_ARM_CTRL_POS_VEL = 0x17  # 位置 + 速度前馈（本脚本不用）
TID_ARM_RESET = 0x18      # 一键复位（空 payload），只能从「正常运行」发，不做夹爪找零
TID_ARM_FEEDBACK = 0x80   # 反馈：u8 mode,state,lost,inited + f[6] + f[6] + f + u8 temp[7] = 63 B

FB_LEN = 63
CTRL_LEN = 28
ACK_LEN = 12
EMPTY_MODE_TIDS = (TID_ARM_START, TID_ARM_GRAVITY, TID_ARM_STOP, TID_ARM_DAMPING)

# robot_state.h 的 Robot_state
STATE_NAME = {
    0: "等待握手",
    1: "离线等待",
    2: "待初始化",
    3: "初始化中",
    4: "正常运行",
    5: "通信离线",
    6: "校准零点",
    7: "复位中",
}
# STATE_NAME 的键必须和 robot_state.h 的 Robot_state 一一对上（0..ROBOT_STATE_NUM-1），
# 少一个后面全部错位、也不会报错。--selftest 会去数那个枚举的个数来核对。

# robot_def.h 的 Arm_mode（0/1 的值不能动，JLink 上那些老偏移依赖它）
MODE_NAME = {0: "stop(停止)", 1: "run(位控)", 2: "damping(阻尼)", 3: "gravity(重力补偿)"}

# arm_zero.h 的 Arm_zero_state / Arm_zero_result
ZERO_STATE_NAME = {0: "空闲", 1: "失能中", 2: "等失能确认", 3: "发设零帧", 4: "等结果", 5: "完成", 6: "超时"}
ZERO_RES_NAME = {0: "未走到", 1: "未确认", 2: "成功", 3: "失败", 4: "超时", 5: "跳过"}

# App/inc/gimbal_cmd_def.h 的软限位（deg）。--selftest 会去读那个头文件核对，
# 板子上改了限位而这里忘了改的话，自测会直接报出来。
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
JOINT_MIN = [-115.0, 0.5, 0.5, -90.0, -90.0, -112.87]
JOINT_MAX = [90.0, 206.26, 171.88, 90.0, 90.0, 119.17]
GRIPPER_MIN = -284.0 / 2.0  # 张开（找零后零点在行程中点，所以是 ∓142）
GRIPPER_MAX = 284.0 / 2.0   # 闭合
N_AXIS = 7                  # 6 关节 + 夹爪
GRIPPER = 6
AXIS_NAMES = JOINT_NAMES + ["夹爪"]
AXIS_MIN = JOINT_MIN + [GRIPPER_MIN]
AXIS_MAX = JOINT_MAX + [GRIPPER_MAX]

STEP_CHOICES = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]

# 板上 arm_auto.h 的 ARM_RESET_ANGLE：一键复位/初始化要回到的位姿（6 关节，不含夹爪）。
# 0x02 和 0x18 都用它。
RESET_POSE = (0.0, 0.5, 0.5, 0.0, 0.0, 0.0)


def axis_lo(axis):
    return AXIS_MIN[axis]


def axis_hi(axis):
    return AXIS_MAX[axis]


def clamp_axis(axis, v):
    """返回 (夹住之后的值, 是否发生了夹) —— 上位机必须自己夹，板子那边是静默夹的。"""
    lo, hi = axis_lo(axis), axis_hi(axis)
    if v < lo:
        return lo, True
    if v > hi:
        return hi, True
    return v, False


def encode(tid, payload=b"", arm_id=DEFAULT_ARM_ID):
    """组一帧：AA 55 | type | datalen | arm_id | crc8 | data | crc16(LE)，共 8+datalen 字节。

    arm_id 是"这一包发给哪台臂"（0 = CAN1，1 = CAN2），**所有 type_id 都带**。
    板子侧只认 arm_id < ARM_NUM，越界的帧直接丢弃。
    """
    head = bytearray((0xAA, 0x55, tid & 0xFF, len(payload) & 0xFF, arm_id & 0xFF))
    b = bytearray(head)
    b.append(crc8(head))
    b += payload
    c = crc16(b)
    b.append(c & 0xFF)
    b.append((c >> 8) & 0xFF)
    return bytes(b)


def encode_ctrl_pos(angles7):
    """0x16 的 payload：6 个关节角 + 夹爪角，float32 LE，deg。"""
    return struct.pack("<7f", *angles7)


def decode_feedback(p):
    if len(p) != FB_LEN:
        raise ValueError("0x80 长度 %d != %d" % (len(p), FB_LEN))
    return {
        "mode": p[0],
        "state": p[1],
        "lost": p[2],
        "inited": p[3],
        "joint": list(struct.unpack_from("<6f", p, 4)),
        "jvel": list(struct.unpack_from("<6f", p, 28)),
        "gripper": struct.unpack_from("<f", p, 52)[0],
        "temp": list(p[56:63]),
    }


def decode_setzero_ack(p):
    if len(p) != ACK_LEN:
        raise ValueError("0x11 长度 %d != %d" % (len(p), ACK_LEN))
    return {
        "state": p[0],
        "result": list(p[1:8]),
        "elapsed_ms": struct.unpack_from("<I", p, 8)[0],
    }


def expand_lost_mask(mask):
    """把 motor_lost_mask 按位展开成人看得懂的名字（bit0..5 = j1..j6，bit6 = 夹爪）。"""
    if not mask:
        return "无"
    return " ".join(AXIS_NAMES[i] for i in range(N_AXIS) if mask >> i & 1)


def expand_inited_mask(mask):
    if not mask:
        return "无"
    return " ".join(AXIS_NAMES[i] for i in range(N_AXIS) if mask >> i & 1)


class FrameParser:
    """字节流 → 帧。粘包 / 前导噪声 / 半帧 / 坏 CRC 都能重同步。

    重同步策略：找 AA 55 → 够 8+len 字节后先校 crc8，不对就把窗口**只挪一格**再找
    （而不是丢掉整帧）—— 因为 payload 里完全可能出现 AA 55。

    feed() 吐出的是 **(type_id, arm_id, payload)** 三元组：arm_id 是帧头偏移 4 那个字节，
    上层按它分流（0 = CAN1 臂 A、1 = CAN2 臂 B）。
    """

    # 帧头：0..4（AA 55 / type / datalen / arm_id）+ 偏移 5 的 crc8
    CRC8_LEN = 5
    HEADER_LEN = 6

    def __init__(self):
        self.buf = bytearray()
        self.n_ok = 0
        self.n_err_crc8 = 0
        self.n_err_crc16 = 0
        self.n_drop = 0

    def feed(self, chunk):
        out = []
        self.buf += chunk
        while True:
            i = self.buf.find(b"\xAA\x55")
            if i < 0:
                # 一个同步头都没有；最后 1 个字节可能是 AA 的前半，留着
                if len(self.buf) > 1:
                    self.n_drop += len(self.buf) - 1
                    del self.buf[:-1]
                break
            if i > 0:
                self.n_drop += i
                del self.buf[:i]
            if len(self.buf) < self.HEADER_LEN:
                break
            n = self.buf[3]
            total = 8 + n
            if len(self.buf) < total:
                break
            if crc8(self.buf[:self.CRC8_LEN]) != self.buf[self.CRC8_LEN]:
                self.n_err_crc8 += 1
                del self.buf[:1]
                continue
            if crc16(self.buf[:6 + n]) != (self.buf[6 + n] | (self.buf[7 + n] << 8)):
                self.n_err_crc16 += 1
                del self.buf[:1]
                continue
            out.append((self.buf[2], self.buf[4], bytes(self.buf[6:6 + n])))
            self.n_ok += 1
            del self.buf[:total]
        return out


# ============================================================================
# B 层：串口 + 后台读线程 + 线程间状态
# ============================================================================

BY_ID_DIR = "/dev/serial/by-id"
DEFAULT_PORT = BY_ID_DIR + "/usb-1a86_USB_Serial-if00-port0"
# 硬约束：绝不打开 J-Link 自带的 VCOM（那是探针，占着它 JLink 就连不上/烧不了了）。
# 注意**不能按名字判**：适配器那条 by-id 路径 realpath 出来也是 /dev/ttyACM*，
# 一刀切"含 ttyacm 就拒"会把好口子也拒掉。
# 所以去 /sys 读 USB 身份：J-Link = idVendor 1366 / manufacturer SEGGER / product J-Link。
SEGGER_VID = "1366"
ADAPTER_VID = "1a86"  # CH340/CH343 那类 USB 转串口/转 RS485 适配器（沁恒）

# 板子自己的 USB CDC 口。板子的 USB 设备栈是 CubeMX 生成的 USB CDC，
# 描述符里 VID:PID 就是 ST 给虚拟串口的标准值（USB_DEVICE/App/usbd_desc.c）。
BOARD_VID = "0483"
BOARD_PID = "5740"

FORBIDDEN_HINTS = ("segger", "j-link", "jlink")

# 链路类型 → 人话。板子这侧对应 App/inc/robot_state.h 的 ROBOT_LINK：
#   ROBOT_LINK_UART → 板子跑的是串口那条 → **必须**用 RS485 适配器接 USART3
#   ROBOT_LINK_USB  → 板子跑的是 USB 那条   → **必须**用板子 USB 口
# 两边选反了的现象是"一切都对但一个字节都没有"，所以下面挑口子时优先板子 USB，
# 并且在面板/日志里把选中了哪种链路写出来。
KIND_NAMES = {"USB": "板子 USB CDC", "RS485": "USB 转 RS485 适配器"}


def _read_sys(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_identity(path):
    """这个 tty 是哪个 USB 设备：往上找到有 idVendor 的那层。查不到返回 None。"""
    name = os.path.basename(os.path.realpath(path))
    d = os.path.realpath("/sys/class/tty/%s/device" % name)
    for _ in range(8):
        vid = _read_sys(os.path.join(d, "idVendor"))
        if vid:
            return {
                "idVendor": vid,
                "idProduct": _read_sys(os.path.join(d, "idProduct")) or "?",
                "product": _read_sys(os.path.join(d, "product")) or "",
                "manufacturer": _read_sys(os.path.join(d, "manufacturer")) or "",
            }
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def classify_port(ident):
    """USB 身份 → 这是哪条链路（"USB" / "RS485" / None）。

    **纯函数，不碰 /sys** —— 喂一个假的 ident 就能测，--selftest 里就是这么用的。
    顺序不在这里定，见 _pick_from()。
    """
    if ident is None:
        return None
    vid = ident["idVendor"].lower()
    if vid == BOARD_VID and ident["idProduct"].lower() == BOARD_PID:
        return "USB"
    if vid == ADAPTER_VID:
        return "RS485"
    return None


def _pick_from(candidates):
    """[(path, ident), ...] → 该用哪条。纯函数，优先级就在这里。

    板子 USB 优先、RS485 退回（用户定的）。同一档里按路径字典序，保证可复现 ——
    插两个同型号适配器时"随便挑一个"会让日志没法复现。
    """
    for want in ("USB", "RS485"):
        for path, ident in sorted(candidates, key=lambda c: c[0]):
            if classify_port(ident) == want:
                return path
    return None


def _port_candidates():
    """扫一遍机器上所有可能是链路的 tty，带上各自的 USB 身份。

    /dev/serial/by-id/* 都是指向 ttyUSB*/ttyACM* 的符号链接，所以按 realpath 去重，
    免得同一条设备出现两次（by-id 那条一般名字更可读，保留先扫到的 by-id 那条）。
    """
    out, seen = [], set()
    paths = []
    if os.path.isdir(BY_ID_DIR):
        paths += sorted(os.path.join(BY_ID_DIR, n) for n in os.listdir(BY_ID_DIR))
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        paths += sorted(glob.glob(pat))
    for p in paths:
        real = os.path.realpath(p)
        if real in seen:
            continue
        seen.add(real)
        out.append((p, usb_identity(p)))
    return out


def pick_default_port():
    """自动找那条链路。**按 USB 身份找，不按名字找**。

    名字靠不住：适配器的 by-id 名字随型号变（本机就换过一次：
    `usb-1a86_USB_Single_Serial_*` → 换 RS485 适配器之后是
    `usb-1a86_USB_Serial-if00-port0`），写死一个必然过一阵就失效。

    优先级（_pick_from）：**板子 USB(0483:5740) → RS485 适配器(1a86) → 写死那条兜底**。
    **永远排除 SEGGER(1366)** —— 它根本不是这两类，classify_port 就返回 None。
    都没有就返回写死的那条（好让报错信息里有个具体路径可看）。"""
    got = _pick_from(_port_candidates())
    if got:
        return got
    return DEFAULT_PORT


def describe_port(path):
    """给日志/面板用的一行描述。带上"这是哪种链路" —— 插错口子时一眼看得出来。"""
    ident = usb_identity(path)
    if ident is None:
        return "%s（查不到 USB 身份）" % path
    kind = classify_port(ident)
    tag = "，%s" % KIND_NAMES[kind] if kind else "，**不是链路设备**"
    return "%s（USB %s:%s %s %s%s）" % (path, ident["idVendor"], ident["idProduct"],
                                        ident["product"], ident["manufacturer"], tag)


def link_hint():
    """没找到合适口子时给的提示。两条链路各要配什么，一起说清楚。"""
    return ("跑 UART 那条(ROBOT_LINK_UART)要接 USB 转 RS485 适配器；\n"
            "  跑 USB 那条(ROBOT_LINK_USB)要把板子自己的 USB 口插上（%s:%s）。\n"
            "  用 --port 可以指定；--fake 则完全不用硬件。"
            % (BOARD_VID, BOARD_PID))


def assert_port_allowed(path):
    """--port 是用户给的，可能手滑写了探针那条。按 USB 身份判，不按名字判。"""
    if not path:
        return
    ident = usb_identity(path)
    if ident is not None:
        blob = (ident["manufacturer"] + " " + ident["product"]).lower()
        if ident["idVendor"].lower() == SEGGER_VID or any(h in blob for h in FORBIDDEN_HINTS):
            raise SystemExit(
                "拒绝打开 %s：查出来是 J-Link 探针的 VCOM 口（%s）。\n"
                "  探针要留给 JLink/烧录用（占着它连不上）。\n  %s"
                % (path, describe_port(path), link_hint()))
        return
    # 查不到身份（没有 /sys、设备还没枚举出来）：名字里带 SEGGER 的直接拒；
    # 名字是裸 ttyACM* 的也拒（宁可不连，也不赌它是探针）
    real = os.path.realpath(path)
    blob = (path + " " + real).lower()
    if any(h in blob for h in FORBIDDEN_HINTS) or \
            os.path.basename(real).lower().startswith("ttyacm"):
        raise SystemExit(
            "拒绝打开 %s：查不到它的 USB 身份，而名字像 J-Link 的探针口。\n"
            "  别赌（探针那条多半也在 ttyACM* 上）。\n  %s" % (path, link_hint()))


class SharedState:
    """读线程 → UI 线程的唯一通道。所有字段都在一把锁里改，UI 拿 snapshot() 的拷贝。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.fb = None           # 最后一帧 0x80（解好的 dict）
        self.fb_t = 0.0
        self.ack = None          # 最后一帧 0x11
        self.ack_t = 0.0
        self.board_version = None
        self.link_up = True
        self.link_err = None
        self.n_rx = 0            # 收到的合法帧数(全类型)
        self.n_rx_fb = 0
        self.rx_rate = 0.0
        self.n_tx = 0
        self.n_crc_err = 0
        self.n_drop = 0          # 解析器丢掉的噪声字节（正常应该是 0 左右）
        self.unknown_tids = {}
        self._rate_t = 0.0
        self._rate_n = 0

    def snapshot(self):
        with self.lock:
            d = dict(self.fb) if self.fb else None
            a = dict(self.ack) if self.ack else None
            return {
                "fb": d, "fb_t": self.fb_t, "ack": a, "ack_t": self.ack_t,
                "board_version": self.board_version,
                "link_up": self.link_up, "link_err": self.link_err,
                "n_rx": self.n_rx, "n_rx_fb": self.n_rx_fb, "rx_rate": self.rx_rate,
                "n_tx": self.n_tx, "n_crc_err": self.n_crc_err, "n_drop": self.n_drop,
                "unknown_tids": dict(self.unknown_tids),
            }


class Link:
    """真串口。写只在主线程（send），读在后台线程。

    `arm_id` 是**这条链路代表的那台臂**（--arm-id）：发出去的每一帧都盖这个 id，
    收进来的帧只认这个 id 的那些。收发两侧都在这一个属性上，所以不存在
    "发给了 A 却在等 B 的反馈"这种半半边界的错。
    """

    def __init__(self, port, baud, shared, arm_id=DEFAULT_ARM_ID):
        self.port = port
        self.baud = baud
        self.shared = shared
        self.arm_id = arm_id
        # 这条口子是哪种链路（"USB" / "RS485" / None）。只用来在面板/日志里标出来。
        self.kind = classify_port(usb_identity(port))
        self.ser = None
        self._alive = False
        self._thr = None

    def open(self):
        import serial  # 只有真要连的时候才需要 pyserial
        # 板子 USB(CDC) 那条上波特率是没意义的 —— CDC 是包传输，tcsetattr 会把
        # SET_LINE_CODING 交给设备，STM32 那边 CDC_Control_HS 对它是 no-op 直接返回 OK。
        # 所以照设不误，只是别据此以为它有用。
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._alive = True
        with self.shared.lock:
            self.shared.link_up = True
            self.shared.link_err = None
        self._thr = threading.Thread(target=self._reader, daemon=True)
        self._thr.start()

    def close(self):
        self._alive = False
        if self._thr is not None:
            self._thr.join(timeout=0.5)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass

    def send(self, tid, payload=b""):
        if self.ser is None:
            return False
        try:
            self.ser.write(encode(tid, payload, self.arm_id))
        except Exception as e:  # 拔线 / 写失败
            with self.shared.lock:
                self.shared.link_up = False
                self.shared.link_err = "写串口失败: %s" % e
            return False
        with self.shared.lock:
            self.shared.n_tx += 1
        return True

    def _reader(self):
        ps = FrameParser()
        while self._alive:
            try:
                chunk = self.ser.read(4096)
            except Exception as e:
                with self.shared.lock:
                    self.shared.link_up = False
                    self.shared.link_err = "读串口失败: %s" % e
                return
            if not chunk:
                continue
            try:
                frames = ps.feed(chunk)
            except Exception as e:
                with self.shared.lock:
                    self.shared.link_up = False
                    self.shared.link_err = "解析失败: %s" % e
                return
            now = time.monotonic()
            with self.shared.lock:
                self.shared.n_crc_err = ps.n_err_crc8 + ps.n_err_crc16
                self.shared.n_drop = ps.n_drop
                self.shared._rate_n += len(frames)
                if now - self.shared._rate_t >= 0.5:
                    el = now - self.shared._rate_t
                    if self.shared._rate_t > 0:
                        self.shared.rx_rate = self.shared._rate_n / el
                    self.shared._rate_t = now
                    self.shared._rate_n = 0
                for tid, arm_id, payload in frames:
                    self.shared.n_rx += 1
                    # 只有**本进程那台臂**的帧往 shared 里落 —— shared 是单台的会话状态
                    # （一个 fb、一个 ack），混进另一台的帧就会让面板在两台之间乱跳。
                    # 另一台的帧照常计入 n_rx（链路上收了多少帧是链路统计，本来就该是
                    # 全链路的），只是不解读。所以面板上"收 N 帧"是全链路的、两台都在里面，
                    # 而"0x80 M"那个才是本台的 —— 两个数不一样是正常的，不是丢帧。
                    if arm_id != self.arm_id:
                        continue
                    self.shared.fb_t = now if tid == TID_ARM_FEEDBACK else self.shared.fb_t
                    if tid == TID_ARM_FEEDBACK:
                        try:
                            self.shared.fb = decode_feedback(payload)
                            self.shared.n_rx_fb += 1
                        except ValueError:
                            pass
                    elif tid == TID_SET_ZERO_ACK:
                        try:
                            self.shared.ack = decode_setzero_ack(payload)
                            self.shared.ack_t = now
                        except ValueError:
                            pass
                    elif tid == TID_VERSION and len(payload) == 4:
                        self.shared.board_version = struct.unpack("<I", payload)[0]
                    elif tid not in (TID_VERSION, TID_ARM_FEEDBACK, TID_SET_ZERO_ACK):
                        self.shared.unknown_tids[tid] = self.shared.unknown_tids.get(tid, 0) + 1


# ============================================================================
# C 层：会话状态机（全部在 UI 线程跑，发送单写者）
# ============================================================================

class Session:
    """把「板子现在在哪个状态」从反馈帧里派生出来，不自己造状态。

    tick() 每 10 ms 调一次：
      - 能发 0x16 时（state==normal & mode==run & 没暂停）按 --ctrl-hz 发当前目标角；
      - 不能发时，150 ms 补一帧 0x01 当保活 —— 否则初始化那 7 s 会掉进 comm_lost。
    """

    KEEPALIVE_S = 0.15

    def __init__(self, link, opts):
        self.link = link
        self.opts = opts
        self.target = [0.0] * N_AXIS
        self.sel = 0
        self.step = opts.step
        self.hold = False              # 暂停 0x16 流（冻结目标，不是停机）
        self.seeded = False
        self.prev = None               # 上一帧 (state, mode, lost)
        self.last_tx = 0.0
        self.last_ctrl = 0.0
        self.t0 = time.monotonic()
        self.handshake_reported = False
        self.init_t0 = None
        self.init_told_reset = False   # 初始化流程走没走过"复位"这一步（只用来显示）
        self.reset_t0 = None           # 进了「复位中(7)」的时刻，只用来显示进度
        self.ctrl_live = False         # 上一拍真的发出去了 0x16
        self.force_init = False        # i 键第一遍提示姿态不合规，第二遍才强发
        self.events = deque(maxlen=400)
        self.dirty = True
        self.have_ever_rx = False

    # ---- 事件 ----
    def event(self, level, text):
        self.events.append((time.time(), level, text))
        self.dirty = True

    def info(self, text):
        self.event("info", text)

    def warn(self, text):
        self.event("warn", text)

    def err(self, text):
        self.event("err", text)

    # ---- 目标角 ----
    def seed_from_feedback(self, fb, why):
        self.target = list(fb["joint"]) + [fb["gripper"]]
        self.seeded = True
        self.info("目标角已对齐反馈（%s）" % why)

    def jog(self, axis, sign):
        """按键只改 target[]，实际下发由 tick 按周期做。"""
        if axis is None:
            return
        if self.fb() is None:
            self.warn("还没收到反馈帧，先别动 —— 没有反馈就无法把目标对齐当前位姿")
            return
        v = self.target[axis] + sign * self.step
        v2, clamped = clamp_axis(axis, v)
        self.target[axis] = v2
        if clamped:
            self.warn("%s 请求 %.2f 超出软限位 [%.2f, %.2f]，已停在 %.2f"
                      % (AXIS_NAMES[axis], v, axis_lo(axis), axis_hi(axis), v2))
        else:
            self.dirty = True

    def cycle_step(self, direction=1):
        i = STEP_CHOICES.index(self.step) if self.step in STEP_CHOICES else 2
        i = (i + direction) % len(STEP_CHOICES)
        self.step = STEP_CHOICES[i]
        self.info("步长 = %.2f°" % self.step)

    # ---- 读 ----
    def fb(self):
        return self.link.shared.snapshot()["fb"]

    def state_mode(self):
        snap = self.link.shared.snapshot()
        fb = snap["fb"]
        if fb is None:
            return None, None, snap
        return fb["state"], fb["mode"], snap

    # ---- 发送 ----
    def send_empty_mode(self, tid, name):
        st, mode, _ = self.state_mode()
        if st is None:
            self.warn("还没有反馈帧，不知道板子在哪一步，不发 %s" % name)
            return
        if st != 4:
            self.warn("当前是「%s」，板子不在正常运行态，%s 会被忽略（只有正常运行才认切档）"
                      % (STATE_NAME.get(st, st), name))
            return
        if self.link.send(tid):
            self.info("已发 %s（%s）" % (name, {0x12: "run", 0x13: "重力补偿",
                                               0x14: "停止/松劲", 0x15: "阻尼"}[tid]))
            self.last_tx = time.monotonic()

    def send_arm_init(self):
        """0x02。板子会先比姿态和掉线情况，不合格**不回包**（静默拒绝），所以本地上先自检。"""
        fb = self.fb()
        if fb is None:
            self.warn("还没有反馈帧，不发 0x02")
            return
        # 板子的两个入口（robot_state.c 的 HandleArmInit）：
        #   待初始化(2)，或 正常运行(4) 的 stop(0)/damping(2) 档。
        # run 档下按 i 仍被静默丢弃 —— 不让"臂正在被位控"的时候被一键动作接管。
        if not (fb["state"] == 2 or (fb["state"] == 4 and fb["mode"] in (0, 2))):
            self.warn("当前是「%s」/ %s 档，0x02 只能从「待初始化」或「正常运行」的 "
                      "stop/damping 档发（否则板子静默丢弃）"
                      % (STATE_NAME.get(fb["state"], fb["state"]),
                         MODE_NAME.get(fb["mode"], fb["mode"])))
            return
        if fb["lost"]:
            self.warn("有电机掉线（%s），0x02 多半会被拒：%s"
                      % (expand_lost_mask(fb["lost"]), "先把电机救回来"))
            return
        # 预设初始化位置 {0, 0.5, 0.5, 0, 0, 0}，容差 ROBOT_INIT_POSE_TOL_DEG
        bad = []
        for i, want in enumerate(RESET_POSE):
            d = abs(fb["joint"][i] - want)
            if d > self.opts.init_tol:
                bad.append("%s 偏 %.1f°" % (JOINT_NAMES[i], d))
        if bad:
            self.warn("姿态超出复位位姿 ±%.0f°（%s）—— 先在失能/阻尼下摆回 "
                      "{0, 0.5, 0.5, 0, 0, 0}；再按一次 i 可强发（板子判据相同，"
                      "超了照样静默丢弃）" % (self.opts.init_tol, "，".join(bad)))
            if not self.force_init:
                self.force_init = True
                return
            self.warn("（强发）")
        self.force_init = False
        if self.link.send(TID_ARM_INIT):
            self.info("已发 0x02 初始化：板子将跑「一键复位 → 一键夹爪找零」，约 7 s，别松保活")
            self.last_tx = time.monotonic()

    def send_arm_reset(self):
        """0x18 一键复位。板子的闸门是「正常运行 + 遥控器离线 + 无掉线」，不合格**不回包**，
        所以本地上先自检一遍。**板子故意不查姿态**（运行中的臂本来就该是歪的），要不要
        先确认由这里（Ui.reset_confirm）决定。"""
        fb = self.fb()
        if fb is None:
            self.warn("还没有反馈帧，不发 0x18")
            return
        if fb["state"] != 4:
            self.warn("当前是「%s」，0x18 只能从「正常运行」发（板子静默丢弃）"
                      % STATE_NAME.get(fb["state"], fb["state"]))
            return
        if fb["lost"]:
            self.warn("有电机掉线（%s），0x18 多半会被拒：先把电机救回来"
                      % expand_lost_mask(fb["lost"]))
            return
        if self.link.send(TID_ARM_RESET):
            self.info("已发 0x18 一键复位：约 2 s 回到 %s，跑完停 damping、**0x16 自动停发**"
                      "（要续位控按 s）" % " ".join("%g" % v for v in RESET_POSE))
            now = time.monotonic()
            self.last_tx = now
            self.reset_t0 = now

    def send_setzero(self):
        if self.link.send(TID_MOTOR_SET_ZERO):
            self.err("！！已发 0x10 设零点：整条臂会失能、把当前位姿写成零点 ！！")
            self.last_tx = time.monotonic()

    # ---- 每拍 ----
    def tick(self, now):
        snap = self.link.shared.snapshot()
        fb = snap["fb"]
        if fb is not None:
            self.on_feedback(fb, snap, now)
        elif now - self.t0 > self.opts.handshake_timeout and not self.handshake_reported:
            self.handshake_reported = True
            self.dirty = True

        can_ctrl = (fb is not None and fb["state"] == 4 and fb["mode"] == 1
                    and not self.hold and snap["link_up"])
        if can_ctrl:
            if now - self.last_ctrl >= 1.0 / max(1.0, self.opts.ctrl_hz):
                self.last_ctrl = now
                if self.link.send(TID_ARM_CTRL_POS, encode_ctrl_pos(self.target)):
                    self.last_tx = now
                    self.ctrl_live = True
                else:
                    self.ctrl_live = False
        else:
            self.ctrl_live = False
            if now - self.last_tx > self.KEEPALIVE_S:
                self.link.send(TID_VERSION, struct.pack("<I", VERSION_CODE))
                self.last_tx = now
        if self.init_t0 is not None and fb is not None and fb["state"] != 3:
            self.init_t0 = None
        if self.reset_t0 is not None and fb is not None and fb["state"] != 7:
            self.reset_t0 = None

    def on_feedback(self, fb, snap, now):
        if not self.have_ever_rx:
            self.have_ever_rx = True
            bv = snap["board_version"]
            if bv is None:
                self.info("已收到反馈帧（版本回复还没到 —— 板子被版本码锁死时就是一个字节都不回，"
                          "本机版本码 0x%08X 可对照）" % VERSION_CODE)
            elif bv == VERSION_CODE:
                self.info("已握手：板子版本码 0x%08X 相符（%s）" % (bv, VERSION_STRING))
            else:
                self.err("版本码不符：板子 0x%08X / 本机 0x%08X —— 板子会静默丢弃一切，"
                         "只能复位板子，或把两边版本串改回一致" % (bv, VERSION_CODE))
            self.seed_from_feedback(fb, "首帧反馈")
        st, mode, lost = fb["state"], fb["mode"], fb["lost"]
        cur = (st, mode, lost)
        if self.prev is None:
            self.prev = cur
        elif cur != self.prev:
            pst, pmode, plost = self.prev
            if st != pst:
                self.info("状态 %s → %s" % (STATE_NAME.get(pst, pst), STATE_NAME.get(st, st)))
                if st == 3:
                    self.init_t0 = now
                    self.init_told_reset = False
                if st == 7:
                    self.reset_t0 = now
                # 从 comm_lost 回来 / 初始化结束进 normal —— 这两处必须重播种，
                # 否则残留的旧目标会在进 run 的那一帧把臂拽走（joint_src 自播种只在
                # 没有有效控制帧时才跟着反馈走）。
                if st == 2 and pst in (3, 5, 6):
                    self.seed_from_feedback(fb, "重新回到待初始化")
                if st == 4 and pst == 7:
                    # 复位收尾。**必须重播种**：复位期间臂真的动过，切走之前那个目标
                    # 早过期了（Ctrl_Invalidate 也把板子那份清了），不重播种的话下一帧
                    # 0x16 会把臂按老目标再拽一次。
                    self.seed_from_feedback(fb, "复位收尾")
                    bad = [(JOINT_NAMES[i], fb["joint"][i] - want)
                           for i, want in enumerate(RESET_POSE)
                           if abs(fb["joint"][i] - want) > self.opts.init_tol]
                    if bad:
                        # 板上那个静默失败：临时规划池满时复位起了个空规划、下一拍就
                        # "跑完"，状态机以为做完了但臂没动过（robot_state.c 的 Reset_Step）。
                        self.err("复位**疑似没执行**：%s 仍偏离复位位姿超过 ±%.0f°（规划和"
                                 "还没真的跑，板上会因为规划池满静默跳过；或中途被打断）"
                                 % ("，".join("%s 偏 %.1f°" % (n, d) for n, d in bad),
                                    self.opts.init_tol))
                    else:
                        self.info("复位完成，6 个关节已到复位位姿")
                    self.reset_t0 = None
            if mode != pmode:
                good = {1: "info", 2: "warn", 0: "warn", 3: "info"}[mode] if mode in (0, 1, 2, 3) else "warn"
                text = "模式 %s → %s" % (MODE_NAME.get(pmode, pmode), MODE_NAME.get(mode, mode))
                if pst in (3, 7) and st == 4 and mode == 2:
                    # 初始化/复位收尾都会 run → damping，是**正常收尾**不是异常
                    # （异常那条是掉线把 run 打回 stop，下面 elif 那句）。别吓人。
                    good = "info"
                    text += "（一键动作收尾的正常返回，臂已松劲）"
                if mode == 1:
                    self.seed_from_feedback(fb, "进入 run")
                    self.event("info", text)
                elif pmode == 1:
                    # 最典型的：Motor_Lost_Handle 因为掉线把 arm_mode 打回 stop
                    self.event("err" if lost else good, text + "（0x16 已自动停发）")
                else:
                    self.event(good, text)
            if lost != plost:
                if lost:
                    self.err("电机掉线: %s（掩码 0x%02X）" % (expand_lost_mask(lost), lost))
                else:
                    self.info("掉线已恢复（掩码 0x%02X）" % plost)
            self.prev = cur
        elif not self.seeded:
            self.seed_from_feedback(fb, "补播种")
        # 初始化进度（只用来显示）
        if st == 3 and self.init_t0 is not None:
            self.dirty = True
            if now - self.init_t0 > 4.0 and not self.init_told_reset:
                self.init_told_reset = True
                self.info("初始化中：应在复位阶段，马上轮到夹爪找零")


# ============================================================================
# C2 层（可选）：--fake 假板子，不接硬件也能把 UI / 按键 / 状态机整条演练一遍
# ============================================================================

class FakeLink:
    """假装板子。接口和 Link 一样（open/close/send + shared），发送在主线程、造帧在后台线程。"""

    DT = 0.004  # 250 Hz

    def __init__(self, shared, opts):
        self._thr = None
        self.shared = shared
        self.opts = opts
        # 这台假板子扮演哪一台臂。**只是标出来**（面板/日志）——假板子只有一个状态机、
        # 写进 shared 的也是一份解好的 dict（里面没有 arm_id 字段），所以 0 和 1 跑起来
        # 是同一份代码。真板上那台臂由 CAN 总线物理分开，这边没有可模拟的第二条链路。
        self.arm_id = getattr(opts, "arm_id", DEFAULT_ARM_ID)
        self.lock = threading.Lock()
        self._alive = False
        self.state = 0
        self.mode = 0
        self.lost = 0
        self.inited = 0x7F
        self.joint = [0.0] * 6
        self.gripper = 0.0
        self.target = None
        self.t = 0.0
        self.init_end = None
        self.calib_end = None
        self.reset_end = None
        self.last_rx_t = 0.0   # 上一帧收到的时刻（假板子的时钟）；0 = 还没收到过

    def open(self):
        self._alive = True
        self._thr = threading.Thread(target=self._gen, daemon=True)
        self._thr.start()

    def close(self):
        self._alive = False
        if self._thr is not None:
            self._thr.join(timeout=0.5)

    def send(self, tid, payload=b""):
        with self.lock:
            with self.shared.lock:
                self.shared.n_tx += 1
            # 收到**任意**有效帧都算"上位机还在说话"，并把它从通信离线里拉回来。
            # 协议里没有心跳包，保活是隐式的（按整个帧流算）—— 所以这里不挑 type_id。
            # 注意落点是**离线等待**、不是原状态：板子会重新走一遍"电机全在线 → 待初始化"。
            self.last_rx_t = self.t
            if self.state == 5:
                self.state = 1
            if tid == TID_VERSION:
                with self.shared.lock:
                    self.shared.board_version = VERSION_CODE
            elif tid == TID_ARM_INIT and (self.state == 2
                                          or (self.state == 4 and self.mode in (0, 2))):
                # 板侧两个入口(robot_state.c 的 HandleArmInit)：待初始化(2)，或 正常运行(4)
                # 的 stop(0)/damping(2) 档 —— run 档下静默丢弃。跑完都落回 4(板子不按入口区分，
                # 见 robot_state.c 的 Init_Step)。**注意条件里读的 self.mode 必须是初始化前
                # 那个档**，所以 self.mode = 1 只能留在最后。
                self.state = 3
                self.init_end = self.t + 3.0
                self.mode = 1
            elif tid == TID_MOTOR_SET_ZERO and self.state == 2:
                # 板子的 0x10 闸门是 state==待初始化 & 遥控器离线（robot_state.c:346）
                # —— 只在待初始化受理，不在正常运行受理。假板子照抄这一条。
                self.state = 6
                self.calib_end = self.t + 1.2
            elif tid == TID_ARM_RESET and self.state == 4 and not self.lost:
                # 板子的 0x18 闸门是 state==正常运行 & 遥控器离线 & 无掉线（不查姿态）
                # —— 这里只复刻状态/掉线这两条，姿态闸门本来就故意没有（同 0x10 的做法）。
                self.state = 7
                self.mode = 1                       # 复位期间是 run（位置环要使上劲）
                self.reset_end = self.t + 2.0       # ARM_AUTO_RESET_TIME_S = 2 s
                self.target = list(RESET_POSE) + [self.gripper]
            elif tid in EMPTY_MODE_TIDS and self.state == 4 and not self.lost:
                self.mode = {TID_ARM_START: 1, TID_ARM_GRAVITY: 3,
                             TID_ARM_STOP: 0, TID_ARM_DAMPING: 2}[tid]
            elif tid == TID_ARM_CTRL_POS and len(payload) == CTRL_LEN:
                vals = list(struct.unpack("<7f", payload))
                # 板子的三重闸门：state==normal & mode==run & 遥控器离线
                if self.state == 4 and self.mode == 1:
                    self.target = vals
        return True

    def _gen(self):
        while self._alive:
            with self.lock:
                t = self.t
                # 500 ms 通信看门狗（板子 Fsm_Step 的 ROBOT_COMM_TIMEOUT_MS）：握手通过之后
                # 除了"等待握手"和"已经在离线里"两态，**任何状态**都算。这一条是"上位机
                # 卡住不发帧"的兜底 —— 也是"弹个确认框就把臂打出正常运行"的元凶：不模拟它，
                # 假板子会假装链路永远不断，面板上的状态就骗人。
                if self.last_rx_t > 0.0 and self.state not in (0, 5) and t - self.last_rx_t > 0.5:
                    self.state = 5
                    self.mode = 2   # 通信离线 = damping
                if self.state == 0 and t > 0.6:
                    self.state = 1
                elif self.state == 1 and t > 1.4:
                    self.state = 2
                    self.mode = 0
                if self.state == 3 and self.init_end is not None and t > self.init_end - 1.5:
                    # 找零把 motor_inited[夹爪] 先清 0、设零成功才置 1（真板子就是这样），
                    # 所以初始化末段 0x80 的 inited 掩码 mass 会少 bit6
                    self.inited = 0x3F
                if self.state == 3 and self.init_end is not None and t > self.init_end:
                    self.state = 4
                    self.mode = 2       # 板子到 normal 是 **damping**，要等 0x12
                    self.init_end = None
                    self.gripper = 0.0  # 找零把零点挪到行程中点，读数整体平移
                    # 找零成功 → motor_inited[夹爪] 置回 1（arm_auto.c:383）。所以真机上
                    # "正常运行(4)" 伴随的应该是 0x7F；4 配 0x3F = 找零超时/失败，夹爪
                    # ∓142 软限位没生效（Init_Step 只等 auto_mode 回 none，失败也照样进 4）
                    self.inited = 0x7F
                if self.state == 7 and self.reset_end is not None and t > self.reset_end:
                    # 复位跑完 → 正常运行 + damping（板子不自动回 run，要等 0x12）
                    self.state = 4
                    self.mode = 2
                    self.reset_end = None
                    # 真板子那条五次样条是**定时**的（2 s 正好走到位），不像上面那个
                    # 一阶滞后那样追不上就慢慢磨；这里直接落位，免得假板子自己触发
                    # "复位疑似没执行"的误报。
                    self.joint = list(RESET_POSE)
                if self.state == 6 and self.calib_end is not None and t > self.calib_end:
                    self.state = 2
                    self.mode = 0
                    self.inited = 0x7F
                    self.calib_end = None
                    with self.shared.lock:
                        self.shared.ack = {"state": 5, "result": [5, 5, 5, 2, 2, 2, 1],
                                           "elapsed_ms": 420}
                        self.shared.ack_t = time.monotonic()
                # --fake-lost：周期性地制造"电机掉线"，看板子/面板怎么反应
                if self.opts.fake_lost and self.state == 4:
                    if (int(t / 5.0) % 2) == 1:
                        if self.lost == 0:
                            self.lost = self.opts.fake_lost
                            self.mode = 0   # 板子掉线时会把 arm_mode 打回 stop
                    elif self.lost:
                        self.lost = 0
                        self.mode = 2
                # 关节一阶滞后追目标
                if self.target is not None:
                    for i in range(6):
                        d = self.target[i] - self.joint[i]
                        mx = 60.0 * self.DT
                        self.joint[i] += max(-mx, min(mx, d))
                    d = self.target[6] - self.gripper
                    mx = 200.0 * self.DT
                    self.gripper += max(-mx, min(mx, d))
                payload = struct.pack("<4B", self.mode, self.state, self.lost, self.inited)
                payload += struct.pack("<6f", *self.joint)
                payload += struct.pack("<6f", *([0.0] * 6))
                payload += struct.pack("<f", self.gripper)
                payload += bytes([36] * 7)
                fb = decode_feedback(payload)
                now = time.monotonic()
                with self.shared.lock:
                    self.shared.fb = fb
                    self.shared.fb_t = now
                    self.shared.n_rx += 1
                    self.shared.n_rx_fb += 1
                    self.shared.rx_rate = 250.0
                    self.shared.link_up = True
                self.t += self.DT
            time.sleep(self.DT)


# ============================================================================
# D 层：curses 面板
# ============================================================================

C_OK, C_WARN, C_ERR, C_DIM, C_HEAD = 1, 2, 3, 4, 5

HELP_LINES = [
    "1-6 / 0 / Tab      选中 j1..j6 / 夹爪 / 下一个",
    "← ↓ / → ↑ / + -    选中轴 ∓ 一个步长（只改目标角，实际下发按 --ctrl-hz）",
    "[ / ]              步长档 0.1 0.5 1 2 5 10（默认 1.0）",
    "c                  目标角对齐当前反馈（重播种；板子切档/掉线后建议先按一下）",
    "s / d / v / x      0x12 进 run / 0x15 阻尼 / 0x13 重力补偿 / 0x14 停止(松劲)",
    "i                  0x02 初始化（板子跑一键复位 → 一键夹爪找零，约 7 s；待初始化态、"
    "或正常运行态的 stop/damping 档发）",
    "o                  0x18 一键复位（只收臂回 {0, 0.5, 0.5, 0, 0, 0}，约 2 s；正常运行态发）",
    "z                  0x10 设零点 —— 要逐字敲 SETZERO 再敲 CONFIRM 才发！",
    "Space              暂停/继续发 0x16（冻结目标，臂保持不动，不是停机）",
    "r / e / h / q      重开串口 / 清事件 / 本帮助 / 退出（正发 0x16 时会二次确认）",
    "",
    "状态从反馈帧派生，不自己造：0x16 只在「正常运行 + run + 没暂停」时才发；",
    "其它时候每 150 ms 补一帧 0x01 保活（板子 500 ms 无有效帧就进 comm_lost）。",
    "被板子丢弃的 0x16 也算保活（板子的 s_last_rx_ms 在分派之前刷新），所以放心发。",
    "",
    "控制哪一台臂由启动参数 --arm-id 定死（0=CAN1/臂A，1=CAN2/臂B），运行中不可切换；",
    "换一台就退出重开。板子只有一路上位机链路，两个进程同时开同一个口会互相抢字节。",
    "",
    "退出不发 0x14：目标冻结、臂保持，约 500 ms 后板子自己因无帧进 damping。",
]


class Ui:
    def __init__(self, stdscr, sess, opts):
        self.scr = stdscr
        self.sess = sess
        self.opts = opts
        self.show_help = False
        self.quit = False
        self.last_draw = 0.0
        self.draw_period = 1.0 / max(1.0, opts.fb_hz)
        self._logged = set()
        self.esc_until = 0.0   # ESC 之后的 50 ms 内吞键（防方向键序列被拆成单键）
        self.logf = None
        if opts.log:
            try:
                self.logf = open(opts.log, "a", encoding="utf-8")
                self.logf.write("# uart_frame_cli 启动 %s port=%s fake=%s arm_id=%d\n"
                                % (time.strftime("%F %T"), opts.port, opts.fake, opts.arm_id))
                self.logf.flush()
            except OSError as e:
                self.logf = None
                print("警告：日志文件打不开(%s)，只在屏幕上显示" % e)

    # ---- 画 ----
    def init_colors(self):
        try:
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(C_OK, curses.COLOR_GREEN, -1)
            curses.init_pair(C_WARN, curses.COLOR_YELLOW, -1)
            curses.init_pair(C_ERR, curses.COLOR_RED, -1)
            curses.init_pair(C_DIM, curses.COLOR_CYAN, -1)
            curses.init_pair(C_HEAD, curses.COLOR_WHITE, -1)
        except curses.error:
            pass

    def attr(self, pair, extra=0):
        try:
            return curses.color_pair(pair) | extra
        except curses.error:
            return extra

    def put(self, y, x, s, a=0):
        try:
            maxy, maxx = self.scr.getmaxyx()
            if y < 0 or y >= maxy or x >= maxx:
                return
            self.scr.addstr(y, x, s[:max(0, maxx - x - 1)], a)
        except curses.error:
            pass

    def draw(self):
        scr = self.scr
        scr.erase()
        try:
            maxy, maxx = scr.getmaxyx()
        except curses.error:
            return
        if maxy < 12 or maxx < 60:
            self.put(0, 0, "窗口太小（%dx%d），至少要 12 行 60 列" % (maxy, maxx), self.attr(C_ERR))
            scr.noutrefresh()
            curses.doupdate()
            return
        snap = self.sess.link.shared.snapshot()
        fb = snap["fb"]
        y = 0

        title = "机械臂串口控制台  %s  %s  %s" % (
            arm_label(self.opts.arm_id),
            "（--fake 假板子）" if self.opts.fake else self.opts.port.split("/")[-1],
            "%d bps" % self.opts.baud)
        self.put(y, 0, title, self.attr(C_HEAD, curses.A_BOLD))
        self.put(y, maxx - 24, "vcode 0x%08X" % VERSION_CODE, self.attr(C_DIM))
        y += 1

        link = "链路 %s" % ("OK" if snap["link_up"] else "**断开**")
        if not snap["link_up"] and snap["link_err"]:
            link += " " + snap["link_err"]
        stats = "  收%u(0x80 %u, %.0f Hz)  发%u  丢字节%u  CRC错%u" % (
            snap["n_rx"], snap["n_rx_fb"], snap["rx_rate"], snap["n_tx"],
            snap["n_drop"], snap["n_crc_err"])
        self.put(y, 0, link + stats,
                 self.attr(C_ERR) if not snap["link_up"] else self.attr(C_OK))
        y += 1

        if fb is None:
            self.put(y, 0, "还没收到任何反馈帧……", self.attr(C_WARN))
            y += 1
            if not snap["link_up"]:
                self.put(y, 0, "串口断了。检查接线后按 r 重开。", self.attr(C_ERR))
            elif time.monotonic() - self.sess.t0 > self.opts.handshake_timeout:
                self.put(y, 0, "握手超时（%ds 没收到一个字节）—— 按顺序查：" % int(self.opts.handshake_timeout),
                         self.attr(C_ERR))
                for i, line in enumerate([
                    "1) 接线方向：适配器 TX → PA10(板子 RX)，适配器 RX → PA9(板子 TX)，GND 共地；",
                    "2) 波特率 921600；3) 板子上跑的是不是这份固件（比对版本码 0x%08X / 版本串 %s）；"
                    % (VERSION_CODE, VERSION_STRING),
                    "4) 版本码不符时板子**静默不回包**，只能复位板子重来；",
                    "5) 双臂板子上 --arm-id 选错了也是这样（越界 arm_id 被静默丢掉），"
                    "当前是 %s。" % arm_label(self.opts.arm_id),
                ]):
                    self.put(y + 1 + i, 0, line, self.attr(C_WARN))
        else:
            missing = ~fb["inited"] & 0x7F
            extra = ""
            if fb["lost"]:
                extra += "  掉线: " + expand_lost_mask(fb["lost"])
            if missing:
                # 夹爪的 ∓142 软限位要**一次成功的找零之后**才生效（motor_inited[夹爪]
                # 由找零自己维护，别处没人写它），所以这两台没标定好要看见。
                extra += "  未初始化: " + expand_inited_mask(missing)
                if missing >> GRIPPER & 1:
                    extra += "（夹爪软限位要一次成功设零才生效）"
            self.put(y, 0, "robot_state %s(%d)   arm_mode %s   inited 0x%02X   lost 0x%02X%s" % (
                STATE_NAME.get(fb["state"], "?"), fb["state"], MODE_NAME.get(fb["mode"], "?"),
                fb["inited"], fb["lost"], extra),
                self.attr(C_ERR) if fb["lost"] else self.attr(C_WARN) if missing else 0)
            y += 1
            stream = "控制帧 %s" % ("在发" if self.sess.ctrl_live else "停发")
            if self.sess.hold:
                stream += "（已暂停 Space）"
            stream += "  %.0f Hz  步长 %.2f°  选中 %s" % (
                self.opts.ctrl_hz, self.sess.step, AXIS_NAMES[self.sess.sel])
            if self.sess.init_t0 is not None:
                stream = "初始化中 %.1f s（复位 → 夹爪找零）  保活中" % (time.monotonic() - self.sess.init_t0)
            elif self.sess.reset_t0 is not None:
                stream = "复位中 %.1f s（目标 %s）  保活中" % (
                    time.monotonic() - self.sess.reset_t0, " ".join("%g" % v for v in RESET_POSE))
            self.put(y, 0, stream, self.attr(C_WARN) if not self.sess.ctrl_live else self.attr(C_OK))
            y += 1
            self.put(y, 0, " 轴    目标角        反馈角        偏差      软限位           温度",
                     self.attr(C_HEAD, curses.A_UNDERLINE))
            y += 1
            tgt = self.sess.target
            for i in range(N_AXIS):
                fv = fb["joint"][i] if i < 6 else fb["gripper"]
                temp = ("%3d°C" % fb["temp"][i]) if i < 6 else "  -  "
                line = "%s%-4s %11.2f %13.2f %+11.2f   [%8.1f,%8.1f] %s" % (
                    ">" if i == self.sess.sel else " ", AXIS_NAMES[i], tgt[i], fv, tgt[i] - fv,
                    axis_lo(i), axis_hi(i), temp)
                a = curses.A_REVERSE if i == self.sess.sel else 0
                if i == 6:
                    a |= self.attr(C_DIM)
                self.put(y, 0, line, a)
                y += 1
            self.put(y, 0, "jvel: " + " ".join("%+7.1f" % v for v in fb["jvel"]) + "  deg/s",
                     self.attr(C_DIM))
            y += 1

        # 事件区
        if snap["ack"] is not None and self.sess.have_ever_rx:
            ack = snap["ack"]
            self.put(y, 0, "上次设零 0x11: %s  耗时 %d ms  结果 %s" % (
                ZERO_STATE_NAME.get(ack["state"], ack["state"]), ack["elapsed_ms"],
                " ".join("%s=%s" % (AXIS_NAMES[i], ZERO_RES_NAME.get(ack["result"][i], "?"))
                         for i in range(N_AXIS))), self.attr(C_DIM))
            y += 1
            if 1 in ack["result"]:
                # 达妙"未确认"是常态（位置本来就在 0 附近，写没写进去分辨不出来），
                # 不说清楚的话每次找零完都会以为失败了。
                self.put(y, 0, "  未确认(1)=达妙本来就在读数 0 附近，无法证明写进去了 —— 不是故障",
                         self.attr(C_WARN))
                y += 1

        y += 1
        if self.show_help:
            for line in HELP_LINES:
                self.put(y, 2, line, self.attr(C_DIM))
                y += 1
        else:
            room = max(0, maxy - y - 2)
            shown = list(self.sess.events)[-room:]
            for ts, level, text in shown:
                a = {"info": 0, "warn": self.attr(C_WARN), "err": self.attr(C_ERR)}[level]
                self.put(y, 0, "%s %s" % (time.strftime("%H:%M:%S", time.localtime(ts)), text), a)
                y += 1
                if y >= maxy - 2:
                    break

        self.put(maxy - 2, 0, "1-6/0/Tab 选轴  ←↓→↑/+- 步进  [/] 步长  c 对齐  s run  d 阻尼  "
                              "x 停  v 重力  i 初始化  o 复位  z 设零  Space 暂停  h 帮助  q 退出",
                 self.attr(C_DIM))
        scr.noutrefresh()
        curses.doupdate()

    # ---- 输入 ----
    def input_line(self, prompt):
        """底部一行的手输（设零双重确认用）。自己画、自己吃键，回车返回字符串，ESC 返回 None。"""
        try:
            maxy, maxx = self.scr.getmaxyx()
        except curses.error:
            return None
        buf = ""
        while True:
            self.put(maxy - 1, 0, (" " * (maxx - 2)))
            self.put(maxy - 1, 0, prompt + buf, self.attr(C_ERR, curses.A_BOLD))
            self.scr.noutrefresh()
            curses.doupdate()
            ch = self.scr.getch()
            if ch == -1:
                # getch 超时（节拍由 --ctrl-hz 定，2~17 ms）**必须在这里把链路喂上**：
                # 本函数是阻塞的，主循环的 tick() 整段时间不跑，而 0x16 和 0x01 保活都是
                # tick() 发的 —— 一个字节都不发出去，板子 500 ms 收不到有效帧就掉进通信
                # 离线（robot_state.c 的 Fsm_Step）。等对话框关掉、保活重新发出去，板子的
                # 落点是**离线等待 → 待初始化**，也就是"按 o 弹确认的这几秒"会把「正常
                # 运行」打掉，答 y 发出去的 0x18 反而被板子按"不在正常运行"静默丢弃。
                self.sess.tick(time.monotonic())
                continue
            if ch in (10, 13, curses.KEY_ENTER):
                return buf
            if ch == 27:  # ESC
                # 也要设吞键窗：ESC 后面的序列尾巴会落到 on_key，而 'D' 在那儿 = 0x15 阻尼
                self.esc_until = time.monotonic() + 0.05
                return None
            if ch in (curses.KEY_BACKSPACE, 127, 8):
                buf = buf[:-1]
            elif 32 <= ch < 127:
                buf += chr(ch)

    def on_key(self, ch):
        s = self.sess
        # ESC 后面的字节要吞掉：终端的方向键序列（CSI/SS3）在 terminfo 对不上时会
        # 被 ncurses 拆成 ESC '[' 'D' 三个键，而 'D' 在我们的键表里 = 0x15 阻尼！
        # 也就是说"按一下方向键顺手把臂切进阻尼"是真会发生的，所以 ESC 之后 50 ms
        # 内的按键一律当序列尾巴丢掉。
        if ch == 27:
            self.esc_until = time.monotonic() + 0.05
            if self.show_help:
                self.show_help = False
                s.dirty = True
            return
        if time.monotonic() < self.esc_until:
            return
        if self.show_help:
            # 帮助是覆盖层，但**当模态用**：在帮助里按键不该顺手发出控制包
            if ch in (ord("h"), ord("?"), ord("e"), ord("q")):
                self.show_help = False
                if ch == ord("e"):
                    s.events.clear()
                    s.info("事件已清")
                elif ch == ord("q"):
                    self.quit = True
            s.dirty = True
            return
        if ch in (ord("q"), ord("Q")):
            if s.ctrl_live or s.hold:
                if self.input_line("正发/曾发控制帧，确定退出？(y/n) ").strip().lower() == "y":
                    self.quit = True
            else:
                self.quit = True
        elif ch in (ord("h"), ord("?")):
            self.show_help = not self.show_help
        elif ord("1") <= ch <= ord("6"):
            s.sel = ch - ord("1")
            s.dirty = True
        elif ch == ord("0"):
            s.sel = GRIPPER
            s.dirty = True
        elif ch == 9:  # Tab
            s.sel = (s.sel + 1) % N_AXIS
            s.dirty = True
        elif ch in (curses.KEY_LEFT, curses.KEY_DOWN, ord("-"), ord("_")):
            s.jog(s.sel, -1)
        elif ch in (curses.KEY_RIGHT, curses.KEY_UP, ord("+"), ord("=")):
            s.jog(s.sel, +1)
        elif ch == ord("["):
            s.cycle_step(-1)
        elif ch == ord("]"):
            s.cycle_step(+1)
        elif ch == ord("c"):
            fb = s.fb()
            if fb is None:
                s.warn("还没反馈帧，没法对齐")
            else:
                s.seed_from_feedback(fb, "手动 c")
        elif ch == ord(" "):
            s.hold = not s.hold
            s.info("0x16 流已%s（目标冻结，臂保持；真松劲请按 x）" % ("暂停" if s.hold else "继续"))
        elif ch in (ord("s"), ord("S")):
            s.send_empty_mode(TID_ARM_START, "0x12 启动")
        elif ch in (ord("d"), ord("D")):
            s.send_empty_mode(TID_ARM_DAMPING, "0x15 阻尼")
        elif ch in (ord("v"), ord("V")):
            s.send_empty_mode(TID_ARM_GRAVITY, "0x13 重力补偿")
        elif ch in (ord("x"), ord("X")):
            s.send_empty_mode(TID_ARM_STOP, "0x14 停止")
        elif ch in (ord("i"), ord("I")):
            s.send_arm_init()
        elif ch in (ord("o"), ord("O")):
            self.reset_confirm()
        elif ch in (ord("z"), ord("Z")):
            self.setzero_confirm()
        elif ch == ord("e"):
            s.events.clear()
            s.info("事件已清")
        elif ch == ord("r"):
            self.reopen()
        elif ch == curses.KEY_RESIZE:
            s.dirty = True

    def reset_confirm(self):
        """0x18 的姿态闸门在**上位机**这一侧（板子故意不查：运行中的臂本来就该是歪的，
        不歪就不用复位了）。偏差超过容差时弹一次 y/n 由人拍板，合格就直接发。"""
        s = self.sess
        fb = s.fb()
        if fb is not None and fb["state"] == 4 and not fb["lost"]:
            bad = [JOINT_NAMES[i] for i, want in enumerate(RESET_POSE)
                   if abs(fb["joint"][i] - want) > self.opts.init_tol]
            if bad:
                s.warn("姿态离复位位姿超过 ±%.0f°（%s）—— 复位期间是 run 档、臂会**使劲收拢**，"
                       "行程里别站人" % (self.opts.init_tol, "，".join(bad)))
                if self.input_line("确定运行一键复位？(y/n) ").strip().lower() != "y":
                    s.info("一键复位已取消")
                    return
        # 剩下的自检（没反馈/状态不对/掉线）交给 send_arm_reset 去报，避免两处文案打架
        s.send_arm_reset()

    def setzero_confirm(self):
        """0x10 是唯一会动"零点"的包，所以双重确认（逐字敲，防手滑）。"""
        s = self.sess
        fb = s.fb()
        if fb is None:
            s.warn("还没反馈帧，不发 0x10")
            return
        if fb["state"] != 2:
            s.warn("0x10 只能在「待初始化」发（现在 %s）" % STATE_NAME.get(fb["state"], fb["state"]))
            return
        s.warn("0x10 会把**当前位姿**写成零点，并覆盖电机里的原零点！")
        s.warn("当前姿态：%s（复位位姿是 0 0.5 0.5 0 0 0）"
               % " ".join("%.1f" % v for v in fb["joint"]))
        if self.input_line("第1步/2：把机械臂摆到基准位姿后逐字输入 SETZERO >") != "SETZERO":
            s.info("设零已取消（第 1 步没通过）")
            return
        if self.input_line("第2步/2：再逐字输入 CONFIRM 真正发送 0x10 >") != "CONFIRM":
            s.info("设零已取消（第 2 步没通过）")
            return
        s.send_setzero()

    def reopen(self):
        s = self.sess
        link = s.link
        try:
            link.close()
        except Exception:
            pass
        try:
            if self.opts.fake:
                link = FakeLink(link.shared, self.opts)
                link.open()
            else:
                link = Link(self.opts.port, self.opts.baud, link.shared, self.opts.arm_id)
                link.open()
            s.link = link
            s.seeded = False
            s.prev = None
            s.event("info", "串口已重开（目标角会在下一帧反馈后重新对齐）")
        except Exception as e:
            s.err("重开串口失败：%s" % e)
        s.dirty = True

    def log_events(self):
        if self.logf is None:
            return
        for ts, level, text in self.sess.events:
            key = (ts, text)
            if key in self._logged:
                continue
            self._logged.add(key)
            self.logf.write("%s %-4s %s\n" % (time.strftime("%F %T", time.localtime(ts)), level, text))
        self.logf.flush()

    def run(self):
        s = self.sess
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self.scr.keypad(True)
        # 面板节拍 = 控制周期的 1/2：发送在 tick 里做，而 tick 只在 getch 超时后才跑，
        # 节拍不比控制周期短一半就跑不满 --ctrl-hz（100 Hz 要 5 ms）。下限 2 ms，
        # 免得 --ctrl-hz 填太大时把 CPU 打满。画面刷新另由 --fb-hz 管，与这里无关。
        self.scr.timeout(max(2, int(round(500.0 / max(1.0, self.opts.ctrl_hz)))))
        curses.noecho()
        curses.cbreak()
        self.init_colors()
        self.sess.dirty = True
        while not self.quit:
            ch = self.scr.getch()
            if ch != -1:
                self.on_key(ch)
                self.sess.dirty = True
            now = time.monotonic()
            s.tick(now)
            if s.dirty or now - self.last_draw >= self.draw_period:
                self.draw()
                self.last_draw = now
                s.dirty = False
                self.log_events()
        self.log_events()


# ============================================================================
# 自测 / 假板子 / main
# ============================================================================

def selftest():
    """协议层离线自测。任何一条不过就非零退出。"""
    fails = []

    def check(name, cond, extra=""):
        print("%-46s %s %s" % (name, "OK" if cond else "**失败**", extra))
        if not cond:
            fails.append(name)

    # 1. CRC 参考值
    ref = b"123456789"
    check("crc16(参考串) == 0x6F91", crc16(ref) == 0x6F91, "实际 0x%04X" % crc16(ref))
    check("crc8(参考串) == 0x0B", crc8(ref) == 0x0B, "实际 0x%02X" % crc8(ref))

    # 2. 版本码（FNV-1a，抄板子算法）
    check("版本码 == 0xF91DD3C9", VERSION_CODE == 0xF91DD3C9, "实际 0x%08X (%s)"
          % (VERSION_CODE, VERSION_STRING))

    # 3. 几个已知帧逐字节比对（**板上 FrameEncode 的真实输出**，用小 host 程序
    #    gcc 编 libs/frame/frame.c 打出来抄的）。这一条是整条编解码链的锚：
    #    帧头布局、crc8 的覆盖范围、crc16 的落点，错一处这里就红。
    ver_frame = encode(TID_VERSION, struct.pack("<I", VERSION_CODE))
    check("版本帧字节 == 板上编码器输出",
          ver_frame.hex() == "aa550104008ec9d31df91e20", ver_frame.hex())
    init_frame = encode(TID_ARM_INIT)
    check("0x02 空包字节 == aa5502000051f088",
          init_frame.hex() == "aa5502000051f088", init_frame.hex())
    reset_frame = encode(TID_ARM_RESET)
    check("0x18 空包字节 == aa5518000071fdb6",
          reset_frame.hex() == "aa5518000071fdb6", reset_frame.hex())
    # arm_id 只改帧头那一个字节（偏移 4），crc8/crc16 都得跟着变
    check("arm_id=1 时整帧只差 arm_id 和两个 CRC",
          encode(TID_VERSION, struct.pack("<I", VERSION_CODE), 1).hex()
          == "aa55010401d0c9d31df9ef30")

    # 4. 反馈帧编解码往返（按 1 字节一片回灌，模拟真实串口分片）
    want = {"mode": 1, "state": 4, "lost": 0x01, "inited": 0x7F,
            "joint": [-12.5, 0.5, 30.25, 0.0, -45.5, 119.0], "jvel": [0.0] * 6,
            "gripper": -141.75, "temp": [36, 37, 38, 39, 40, 41, 42]}
    payload = struct.pack("<4B", want["mode"], want["state"], want["lost"], want["inited"])
    payload += struct.pack("<6f", *want["joint"]) + struct.pack("<6f", *want["jvel"])
    payload += struct.pack("<f", want["gripper"]) + bytes(want["temp"])
    check("0x80 payload 是 63 字节", len(payload) == FB_LEN, "实际 %d" % len(payload))
    wire = encode(TID_ARM_FEEDBACK, payload)
    check("0x80 整帧是 71 字节", len(wire) == 71, "实际 %d" % len(wire))
    ps = FrameParser()
    got = []
    for i in range(len(wire)):          # 1 字节一片
        got += ps.feed(wire[i:i + 1])
    check("逐字节回灌解出 1 帧", len(got) == 1, "实际 %d" % len(got))
    if got:
        check("tid == 0x80", got[0][0] == 0x80)
        check("arm_id == 0", got[0][1] == 0, "实际 %d" % got[0][1])
        d = decode_feedback(got[0][2])
        # 值都挑成 float32 能精确表示的（x.25 / x.5 / x.75），所以可以直接比相等
        ok = (d["joint"] == want["joint"] and d["jvel"] == want["jvel"]
              and d["gripper"] == want["gripper"]
              and all(d[k] == want[k] for k in ("mode", "state", "lost", "inited", "temp")))
        check("解出来的字段与原值相等", ok, "" if ok else str(d))

    # 5. 前导噪声 + AA 55 切在两次 feed 之间 + 尾部半帧
    ps = FrameParser()
    got = ps.feed(b"\x00\xffZZ" + wire[:3])          # 噪声 + 半个帧头
    got += ps.feed(wire[3:])                          # 剩下一次给完
    got += ps.feed(wire[:5])                          # 尾部留个半帧
    check("噪声/分片后仍解出 1 帧", len(got) == 1, "实际 %d" % len(got))
    check("噪声字节被计数", ps.n_drop >= 4, "n_drop=%d" % ps.n_drop)
    check("半帧被留在缓冲里", len(ps.buf) > 0, "buf=%d" % len(ps.buf))

    # 6. 坏 payload → crc16 报错，且紧随的合法帧还能解出来
    ps = FrameParser()
    bad = bytearray(wire)
    bad[10] ^= 0xFF
    got = ps.feed(bytes(bad) + wire)
    check("坏帧被计数且不吐出来", len(got) == 1 and ps.n_err_crc16 >= 1,
          "got=%d crc16err=%d" % (len(got), ps.n_err_crc16))
    check("坏帧后面的合法帧照常解出", got and got[0][0] == TID_ARM_FEEDBACK)

    # 7. 控制帧往返 + 长度
    ctrl = encode(TID_ARM_CTRL_POS, encode_ctrl_pos([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -141.5]))
    check("0x16 整帧 36 字节(8+28)", len(ctrl) == 36, "实际 %d" % len(ctrl))
    ps = FrameParser()
    got = ps.feed(ctrl)
    check("0x16 解出来字节数 == 28", got and len(got[0][2]) == CTRL_LEN)
    check("0x16 解出来的角 == 发出去的",
          got and list(struct.unpack("<7f", got[0][2])) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, -141.5])

    # 8. 版本包也算保活帧：空 payload 长度 0，整帧 8 字节
    check("版本帧 12 字节", len(ver_frame) == 12, "实际 %d" % len(ver_frame))

    # 8b. 0x18 空包往返（和 0x02 同款：整包语义只在 type_id 上）
    check("0x18 整帧 8 字节(空 payload)", len(reset_frame) == 8, "实际 %d" % len(reset_frame))
    ps = FrameParser()
    got = ps.feed(reset_frame)
    check("0x18 解出 type_id 且 payload 为空",
          len(got) == 1 and got[0][0] == TID_ARM_RESET and got[0][2] == b"")

    # 8c. arm_id 要能从字节流里原样还原（0 和 1 都试）—— 分流全靠它，
    # 解错了不会报错，只会把 B 臂的帧当成 A 臂的。
    ps = FrameParser()
    got = ps.feed(encode(TID_ARM_INIT, b"", 1) + encode(TID_ARM_INIT, b"", 0))
    check("arm_id 往返(1 和 0)",
          len(got) == 2 and got[0][1] == 1 and got[1][1] == 0
          and got[0][0] == got[1][0] == TID_ARM_INIT,
          "实际 %s" % [(g[0], g[1]) for g in got])

    # 8d. --arm-id 的取值闸门 + **发送侧真的把自己的 arm_id 盖上了**。
    # 这两条是"控制错了另一台"的唯一防线：板子对越界 arm_id 是静默丢弃，症状是
    # "一切看起来都好、状态永远不动"；而 Link.arm_id 只是一个构造参数，漏传的话
    # 代码照样跑、帧照样发（发给了臂 A），没有任何别的症状会冒出来。
    for s, want in (("0", 0), ("1", 1), ("0x1", 1), (" 1", 1)):
        check("--arm-id 接受 %r" % s, parse_arm_id(s) == want,
              "实际 %r" % (parse_arm_id(s),))
    for s in ("2", "-1", "x", "", "1.0"):
        try:
            parse_arm_id(s)
        except argparse.ArgumentTypeError:
            check("--arm-id 拒绝 %r" % s, True)
        except Exception as e:   # 别的异常说明这个闸门根本没接上
            check("--arm-id 拒绝 %r" % s, False, "抛的是 %r" % (e,))
        else:
            check("--arm-id 拒绝 %r" % s, False, "居然通过了")
    check("默认 arm_id 就是 DEFAULT_ARM_ID", parse_arm_id(str(DEFAULT_ARM_ID)) == DEFAULT_ARM_ID)

    class _CaptureSer:
        """假串口：只把写进来的字节攒着，供下面回解。"""

        def __init__(self):
            self.buf = b""

        def write(self, b):
            self.buf += b

    # 不 open()，直接把假串口塞进 ser —— 构造函数不碰 pyserial，能离线跑。
    lk = Link("/dev/null", 921600, SharedState(), 1)
    lk.ser = _CaptureSer()
    wrote = lk.send(TID_ARM_INIT)
    got = FrameParser().feed(lk.ser.buf)
    check("Link.send 盖上了构造时给的那台臂(1)",
          wrote and len(got) == 1 and got[0][0] == TID_ARM_INIT and got[0][1] == 1,
          "实际 %s" % [(g[0], g[1]) for g in got])

    class _Opts:
        pass

    o = _Opts()
    o.arm_id = 1
    check("FakeLink 记住 arm_id", FakeLink(SharedState(), o).arm_id == 1)
    check("FakeLink 没给 arm_id 时退回默认",
          FakeLink(SharedState(), _Opts()).arm_id == DEFAULT_ARM_ID)

    # 9. clamp 边界（上位机必须自己夹，否则面板显示的目标角是骗人的）
    check("clamp j1 上限", clamp_axis(0, 200) == (90.0, True))
    check("clamp j1 下限", clamp_axis(0, -200) == (-115.0, True))
    check("clamp 夹爪 ∓142", clamp_axis(GRIPPER, -999) == (-142.0, True)
          and clamp_axis(GRIPPER, 999) == (142.0, True))
    check("区间内不动", clamp_axis(3, 12.0) == (12.0, False))

    # 10. 位掩码展开
    check("lost 掩码展开", expand_lost_mask(0x41) == "j1 夹爪", expand_lost_mask(0x41))

    # 11. 软限位常数要和板上头文件一致（板子改了这里没改 = 面板骗人）
    check_limits_against_header(check)

    # 11b. 码表也要和板上头文件一致 —— 码表错了不是"不工作"，是拿 0x16 的载荷去撞别的包
    check_typeids_against_header(check)

    # 11c. 状态名同理：少一个不会报错，只会把面板显示成相邻那个错的状态
    check_states_against_header(check)

    # 12. 设零 ack 解码
    ack = decode_setzero_ack(bytes([5, 5, 5, 5, 2, 2, 2, 1]) + struct.pack("<I", 420))
    check("0x11 ack 解码", ack["state"] == 5 and ack["result"][3] == 2 and ack["elapsed_ms"] == 420)

    # 13. 自动挑口子的优先级。_pick_from / classify_port 是纯函数，喂假身份就能测 ——
    # 这个"板子 USB 优先、RS485 退回"的顺序错了会让每次连上都挑到另一个口，插错线还查不出来。
    def _ident(vid, pid, name=""):
        return {"idVendor": vid, "idProduct": pid, "product": name, "manufacturer": ""}

    board = ("/dev/ttyACM0", _ident("0483", "5740", "STM32 Virtual ComPort"))
    rs485 = ("/dev/ttyACM1", _ident("1a86", "7523", "USB Serial"))
    jlink = ("/dev/ttyACM9", _ident("1366", "0105", "J-Link"))
    check("口子识别：板子 USB", classify_port(board[1]) == "USB")
    check("口子识别：RS485 适配器", classify_port(rs485[1]) == "RS485")
    check("口子识别：探针不是链路设备", classify_port(jlink[1]) is None)
    check("口子识别：没有 USB 身份", classify_port(None) is None)
    check("挑口子：板子 USB 优先于 RS485", _pick_from([rs485, board]) == board[0])
    check("挑口子：只有 RS485 时退回它", _pick_from([jlink, rs485]) == rs485[0])
    check("挑口子：只有探针时挑不出来", _pick_from([jlink]) is None)
    check("挑口子：同档按路径排序可复现", _pick_from([board, board]) == board[0])
    # 0483 但 PID 不对（别的 ST 板子/别的 CDC 功能）不该被当成板子
    check("口子识别：0483 但非 5740 不算",
          classify_port(_ident("0483", "0001")) is None)

    print()
    if fails:
        print("自测失败 %d 项：%s" % (len(fails), "，".join(fails)))
        return 1
    print("自测全部通过（%s）" % VERSION_STRING)
    return 0


def check_limits_against_header(check):
    """读 App/inc/gimbal_cmd_def.h 的 JOINT_ANGLE_MIN/MAX，和脚本里的常数对一遍。"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        os.pardir, "App", "inc", "gimbal_cmd_def.h")
    if not os.path.exists(path):
        print("%-46s 跳过（找不到 %s）" % ("软限位与头文件一致", path))
        return
    try:
        text = open(path, encoding="utf-8", errors="replace").read()

        def arr(name):
            m = re.search(name + r"\[ARM_JOINT_NUM\]\s*=\s*\{(.*?)\}", text, re.S)
            if not m:
                raise ValueError(name)
            return [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?", m.group(1))]

        lo, hi = arr("JOINT_ANGLE_MIN"), arr("JOINT_ANGLE_MAX")
        # 夹爪是 284.0/2.0 这种表达式，单独抓
        gmax = re.search(r"GRIPPER_ANGLE_MAX\s*=\s*([^;]+);", text)
        gmin = re.search(r"GRIPPER_ANGLE_MIN\s*=\s*([^;]+);", text)
        gvals = [eval_arith(gmax.group(1)), eval_arith(gmin.group(1))] if gmax and gmin else []
        ok = (lo == JOINT_MIN and hi == JOINT_MAX
              and gvals == [GRIPPER_MAX, GRIPPER_MIN])
        check("软限位与 gimbal_cmd_def.h 一致", ok,
              "" if ok else "头文件 %s/%s/%s" % (lo, hi, gvals))
    except Exception as e:
        check("软限位与 gimbal_cmd_def.h 一致", False, "解析失败: %s" % e)


def check_typeids_against_header(check):
    """把 libs/frame/type_id.h 的码表和脚本里的常数对一遍。

    码表错了不会"报错"，只会把 0x16 的 28 字节载荷塞进一个语义完全不同的包 —— 所以值得对。
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        os.pardir, "libs", "frame", "type_id.h")
    if not os.path.exists(path):
        print("%-46s 跳过（找不到 %s）" % ("码表与 type_id.h 一致", path))
        return
    want = {
        "FRAME_TYPE_ID_VERSION": TID_VERSION,
        "FRAME_TYPE_ID_ARM_INIT": TID_ARM_INIT,
        "FRAME_TYPE_ID_MOTOR_SET_ZERO": TID_MOTOR_SET_ZERO,
        "FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK": TID_SET_ZERO_ACK,
        "FRAME_TYPE_ID_ARM_START": TID_ARM_START,
        "FRAME_TYPE_ID_ARM_GRAVITY": TID_ARM_GRAVITY,
        "FRAME_TYPE_ID_ARM_STOP": TID_ARM_STOP,
        "FRAME_TYPE_ID_ARM_DAMPING": TID_ARM_DAMPING,
        "FRAME_TYPE_ID_ARM_CTRL_POS": TID_ARM_CTRL_POS,
        "FRAME_TYPE_ID_ARM_CTRL_POS_VEL": TID_ARM_CTRL_POS_VEL,
        "FRAME_TYPE_ID_ARM_RESET": TID_ARM_RESET,
        "FRAME_TYPE_ID_ARM_FEEDBACK": TID_ARM_FEEDBACK,
    }
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
        bad = []
        for name, val in want.items():
            m = re.search(re.escape(name) + r"\s+(0x[0-9A-Fa-f]+|\d+)", text)
            if m is None:
                bad.append("%s 找不到" % name)
            elif int(m.group(1), 0) != val:
                bad.append("%s 板上=0x%02X 脚本=0x%02X" % (name, int(m.group(1), 0), val))
        check("码表与 type_id.h 一致", not bad, "；".join(bad))
    except Exception as e:
        check("码表与 type_id.h 一致", False, "解析失败: %s" % e)


def check_states_against_header(check):
    """STATE_NAME 的键必须和 robot_state.h 的 Robot_state 枚举一一对上。

    少一个（或者加了个新状态忘了往 STATE_NAME 里补）不会报错，只会让面板把状态显示成
    相邻那个 —— 比显示 "?" 更糟，因为它看着像真的。所以去数一遍枚举里有几个值。
    """
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        os.pardir, "App", "inc", "robot_state.h")
    if not os.path.exists(path):
        print("%-46s 跳过（找不到 %s）" % ("状态名与 robot_state.h 一致", path))
        return
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
        m = re.search(r"typedef enum\s*\{(.*?)\}\s*Robot_state\s*;", text, re.S)
        if m is None:
            raise ValueError("找不到 Robot_state 枚举")
        n = 0
        for line in m.group(1).splitlines():
            line = line.split("/*")[0].split("//")[0].strip()
            if not line:
                continue
            # 最后一项是 ROBOT_STATE_NUM，它不是状态
            if line.startswith("ROBOT_STATE_NUM"):
                continue
            n += 1
        ok = (sorted(STATE_NAME) == list(range(n)))
        check("状态名与 robot_state.h 一致", ok,
              "" if ok else "板上 %d 个状态，脚本 STATE_NAME 是 %s" % (n, sorted(STATE_NAME)))
    except Exception as e:
        check("状态名与 robot_state.h 一致", False, "解析失败: %s" % e)


def eval_arith(expr):
    """只认 数字 / 小数 / 加减乘除 / 括号 —— 头文件里就 '284.0/2.0' 这种。"""
    if not re.fullmatch(r"[\d\s.+\-*/()]+", expr):
        raise ValueError(expr)
    return eval(expr, {"__builtins__": {}}, {})


def build_session(opts):
    shared = SharedState()
    if opts.fake:
        link = FakeLink(shared, opts)
    else:
        link = Link(opts.port, opts.baud, shared, opts.arm_id)
    return Session(link, opts)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="按 h 看按键表；--selftest / --fake 都不需要硬件。")
    ap.add_argument("--port", default=None,
                    help="串口设备（默认自动挑：优先板子 USB 0483:5740，退回 1a86 适配器）")
    ap.add_argument("--baud", type=int, default=921600, help="波特率（默认 921600）")
    ap.add_argument("--arm-id", type=parse_arm_id, default=DEFAULT_ARM_ID,
                    help="控制哪一台臂：0=CAN1/臂A，1=CAN2/臂B（默认 %d）。一个进程只驱动一台，"
                         "换一台就退出重开（板子只有一路上位机链路，不能同时开两个）"
                         % DEFAULT_ARM_ID)
    ap.add_argument("--ctrl-hz", type=float, default=100.0, help="0x16 下发频率（默认 100）")
    ap.add_argument("--fb-hz", type=float, default=5.0, help="面板刷新率（默认 5，按键响应不受影响）")
    ap.add_argument("--step", type=float, default=1.0, help="初始步长 deg（0.1/0.5/1/2/5/10）")
    ap.add_argument("--handshake-timeout", type=float, default=3.0, help="多久没收到字节就提示握手超时")
    ap.add_argument("--init-tol", type=float, default=20.0,
                    help="0x02 自检 / 0x18 弹确认的姿态容差 deg（板子 0x02 的判据也是 20，0x18 板上不查）")
    ap.add_argument("--fake", action="store_true", help="不接硬件，用内置假板子")
    ap.add_argument("--fake-lost", type=lambda s: int(s, 0), default=0,
                    help="假板子每隔一会制造这个掉线掩码（如 0x41）")
    ap.add_argument("--selftest", action="store_true", help="协议层离线自测，不接硬件不起 curses")
    ap.add_argument("--log", default=None, help="把事件追加写到文件（用 .log 名字，*.json 被 gitignore）")
    ap.add_argument("--stop-on-exit", action="store_true",
                    help="退出前补发 0x14（默认不发：只停 0x16，约 500 ms 后板子自己进 damping）")
    opts = ap.parse_args()

    if opts.selftest:
        return selftest()

    if opts.step not in STEP_CHOICES:
        print("--step 只能是 %s 之一" % STEP_CHOICES)
        return 2
    if not opts.fake:
        opts.port = opts.port or pick_default_port()
        assert_port_allowed(opts.port)
        if not os.path.exists(opts.port):
            print("链路设备 %s 不存在。\n  %s\n  现在机器上认出来的候选："
                  % (opts.port, link_hint()))
            cands = _port_candidates()
            if cands:
                for p, ident in cands:
                    print("    %s" % describe_port(p))
            else:
                print("    （一个都没有：两条链路都没插）")
            return 2
        try:
            import serial  # noqa: F401
        except ImportError:
            print("缺 pyserial：pip3 install pyserial")
            return 2

    # curses 里要显示中文，必须在 initscr **之前**设好 locale（否则宽字符画不出来）
    locale.setlocale(locale.LC_ALL, "")

    sess = build_session(opts)
    try:
        sess.link.open()   # 串口错误要在进 curses 之前报出来
    except Exception as e:
        print("打开 %s 失败：%s" % (opts.port, e))
        if "Permission" in str(e):
            print("（多半是权限：把用户加进 dialout 组，或 sudo 跑）")
        return 2
    # 面板第一行就把"抓的是哪个设备"写清楚 —— 抓错口子是这里最容易犯的错。
    # 板子那侧走哪条链路是**编译期**定死的（App/inc/robot_state.h 的 ROBOT_LINK），
    # 两边选反了的现象是"一切都对但一个字节都没有"，所以把链路类型也标出来。
    sess.info("链路 %s" % (describe_port(opts.port) if not opts.fake
                          else "假板子（--fake，不接硬件）"))
    # 双臂共用一条链路，所以"哪一台"必须是开机第一眼就看得见的东西 —— 开错 arm_id
    # 的现象是"链路全好、状态永远不动"，光看统计分不出来。
    sess.info("控制 %s：本进程发出的每一帧都盖 arm_id=%d，收到的帧里不是这一台的一律不解读"
              % (arm_label(opts.arm_id), opts.arm_id))
    if not opts.fake and sess.link.kind == "USB":
        sess.info("板子 USB CDC：波特率 %d 在这条上无意义（CDC 是包传输）" % opts.baud)
    elif not opts.fake and sess.link.kind is None:
        sess.warn("这个口子既不是板子 USB(0483:5740)也不是 1a86 适配器，"
                  "确认板子的 ROBOT_LINK 和它是对得上的")

    ui = Ui(None, sess, opts)

    def _main(stdscr):
        ui.scr = stdscr
        ui.run()

    try:
        curses.wrapper(_main)
    finally:
        try:
            if opts.stop_on_exit:
                sess.link.send(TID_ARM_STOP)
                print("已给 %s 补发 0x14 停止。" % arm_label(opts.arm_id))
            else:
                print("已停发 %s 的 0x16（目标冻结、臂保持）。约 500 ms 后板子会因 500 ms 无"
                      "有效帧进 comm_lost → damping；要立刻松劲按 x，或用 --stop-on-exit。"
                      % arm_label(opts.arm_id))
            sess.link.close()
        except Exception:
            pass
        if ui.logf:
            ui.logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
