#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂上位机控制台（curses 全屏面板）。UART / USB 两条链路、两台臂都能跑。

**后端就是 `sdk/`**：协议、串口链路、按 arm_id 分流、闸门预检、单位换算全在那儿；
面板要的"一帧一致视图"由 [scripts/cli_backend.py](scripts/cli_backend.py) 的
`ConsoleSession.snapshot()` 给。所以本文件只剩 curses 前端 + 命令行壳 —— 改协议、
加命令只动 `sdk/`，这里最多改排版。

和板子说话的方式（口 protocol_doc.md / sdk/README）：

  * **周期发 0x16**（6 个关节角 + 夹爪角），**它同时就是保活帧** —— 板子那边 500 ms
    没收到任意有效帧就进 comm_lost(damping)，而被丢掉的 0x16 也算有效帧（robot_state.c
    的 s_last_rx_ms 在分派之前刷新），所以任何时候发都不亏。
  * **收 0x80 反馈帧**显示 mode / robot_state / 关节角 / 掉线掩码 / 夹爪角。
  * 按键步进关节目标角，目标角按板子的软限位在**上位机**先夹住：板子也会夹
    （gimbal_cmd.c），但上位机不夹的话面板上显示的目标角就是骗人的。

**单位：面板上是 SI** —— 关节 rad / rad·s⁻¹、夹爪 m（要标定，见 sdk/README「夹爪标定」）。
线上是 deg，换算在 sdk 里做。**事件日志仍然写 deg**（和固件、和别人的日志对得上），
所以"面板上 0.0175 rad、日志里 1.0°"说的是同一件事，别以为哪个错了。

会话流程（板子的状态机，详见 sdk/README「一次会话的生命周期」）：
    ① 等握手（板子只认 0x01 版本包，版本码不符会**静默锁死**，只能复位板子）
    ② 离线等待（7 台电机全上线后板子自己走）
    ③ 待初始化 —— 按 i 发 0x02
    ④ 初始化中（板子自己跑一键复位 → 一键夹爪找零，约 7 s，期间必须保持保活）
    ⑤ 正常运行（**默认 damping**！按 s 发 0x12 才进 run，run 下 0x16 才被接受）
       ↑ 这一档（stop/damping）下**也能按 i 重跑一遍初始化**；跑完回 ⑤ 而不是 ③。
         （板侧闸门放开是用户要求的口径；run 档下按 i 仍会被静默丢弃。）
    ⑥ 按 z 发 0x10 设零点 → 校准，跑完回 ③
    ⑦ 500 ms 无有效帧 → comm_lost；再来一帧就回 ②

**两台臂**：一块板子两块板卡、两条 CAN、两台臂，但上位机只有**一条链路**。所以一个
进程 = 一条链路 + 两个 `sdk.Arm`，**两台都在被驱动**（各自的目标角、各自的 0x16 流、
各自的保活计时），面板上按 `a` 切的是"在看/在操作哪一台"。两台的状态机完全独立 ——
一台还停在待初始化、另一台已经 run，是正常的（i/o/s/d/v/x 两台都要各自按一遍）。

**一次只能开一个进程**：板子的上位机链路是编译期二选一的一路（robot_state.h 的
ROBOT_LINK），物理上就一个口子，两个进程同时开同一个 tty 会互相抢字节、把帧拆散。

**安全**：默认退出**不发** 0x14（停止），只停发 0x16 —— 目标冻结、臂保持原样，
约 500 ms 后板子自己因无帧进 damping。要立刻松劲请手动按 x，或加 --stop-on-exit。

用法：
    python3 scripts/uart_frame_cli.py                # 真机：自动挑口子（板子 USB 优先，
                                                     # 退回 1a86 RS485 适配器），921600
    python3 scripts/uart_frame_cli.py --port /dev/ttyACM0   # 指定口子
    python3 scripts/uart_frame_cli.py --arm-id 1     # 启动时「当前臂」看 CAN2 那台（a 可切）
    python3 scripts/uart_frame_cli.py --fake         # 假板子，不接硬件就能把面板/按键/状态机走一遍
    python3 scripts/uart_frame_cli.py --selftest     # 离线自测（sdk + 后端），不接硬件
面板里按 h 看按键表。
"""

import argparse
import curses
import locale
import os
import sys
import time

# 直接 `python3 scripts/uart_frame_cli.py` 时 sys.path[0] 是 scripts/、**不是仓库根**，
# 所以 `import sdk` / `import cli_backend` 得自己先铺路。
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from sdk import board_defs, protocol                                  # noqa: E402
from sdk.errors import LinkError                                      # noqa: E402
from sdk.link import (assert_port_allowed, describe_port, link_hint,  # noqa: E402
                      list_links, pick_default_port)
from cli_backend import (DEFAULT_ARM_ID, DEFAULT_STEP_RAD, GRIPPER,   # noqa: E402
                         N_AXIS, STEP_CHOICES_RAD, axis_hi_rad,
                         axis_lo_rad, build_session, parse_arm_id,
                         short_label)


# ============================================================================
# D 层：curses 面板（只知道 sess.snapshot() 那一份契约，别的都在后端）
# ============================================================================

C_OK, C_WARN, C_ERR, C_DIM, C_HEAD = 1, 2, 3, 4, 5

HELP_LINES = [
    "1-6 / 0 / Tab      选中 j1..j6 / 夹爪 / 下一个",
    "← ↓ / → ↑ / + -    选中轴 ∓ 一个步长（只改目标角，实际下发按 --ctrl-hz）",
    "[ / ]              步长档（rad）0.005 0.01 0.02 0.05 0.1 0.2，默认 0.02；"
    "夹爪标定过时那一行改用米档 0.0005 0.001 0.002 0.005 0.01 0.02",
    "a                  切「当前臂」（两台都在被同时驱动，切走的那台继续跑它自己的目标角）",
    "c                  目标角对齐当前反馈（重播种；板子切档/掉线后建议先按一下）",
    "s / d / v / x      0x12 进 run / 0x15 阻尼 / 0x13 重力补偿 / 0x14 停止(松劲)",
    "i                  0x02 初始化（板子跑一键复位 → 一键夹爪找零，约 7 s；待初始化态、"
    "或正常运行态的 stop/damping 档发）",
    "o                  0x18 一键复位（只收臂回复位位姿 0/0.5/0.5/0/0/0 度，约 2 s；正常运行态发）",
    "z                  0x10 设零点 —— 要逐字敲 SETZERO 再敲 CONFIRM 才发！",
    "Space              暂停/继续发 0x16（冻结目标，臂保持不动，不是停机）",
    "r / e / h / q      重开串口 / 清事件 / 本帮助 / 退出（任一台正发 0x16 时会二次确认）",
    "",
    "**面板上的单位是 SI**：关节 rad / rad·s⁻¹、夹爪 m（要标定，见 sdk/README「夹爪标定」）；",
    "夹爪没标定时那一行显示的是**电机原始角**（rad），软限位 ∓142°。",
    "事件日志里角度一律写 deg（和固件对得上），和面板上差一个 57.3 —— 不是哪个错了。",
    "",
    "状态从反馈帧派生，不自己造：0x16 只在「正常运行 + run + 没暂停」时才发；",
    "其它时候每 150 ms 补一帧 0x01 保活（板子 500 ms 无有效帧就进 comm_lost）。",
    "被板子丢弃的 0x16 也算保活（板子的 s_last_rx_ms 在分派之前刷新），所以放心发。",
    "",
    "**两台臂各自跑各的状态机**：i/o/s/d/v/x 只作用于「当前臂」（底下那行 > 标着的那台）。",
    "一条链路上开两个进程会互相抢字节，所以一次只开这一个。",
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
                self.logf.write("# uart_frame_cli 启动 %s port=%s fake=%s 当前臂=%d\n"
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
        snap = self.sess.snapshot()
        fb = snap["fb"]
        arm_id = snap["arm_id"]
        y = 0

        title = "机械臂控制台  [当前] %s  %s  %s" % (
            board_defs.arm_label(arm_id),
            "（--fake 假板子）" if self.opts.fake else self.opts.port.split("/")[-1],
            "%d bps" % self.opts.baud)
        self.put(y, 0, title, self.attr(C_HEAD, curses.A_BOLD))
        self.put(y, maxx - 24, "vcode 0x%08X" % protocol.VERSION_CODE, self.attr(C_DIM))
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

        # 两台概览：一条链路驱动两台，所以"另一台现在什么样"必须一眼看得见
        # （> 是当前臂；它那台还在不在发、掉没掉线，全在这一行）
        seg = ["%s%s %s/%s %s %.0fHz%s" % (
            ">" if a["arm_id"] == arm_id else " ", short_label(a["arm_id"]),
            a["state_name"], a["mode_name"], "在发" if a["ctrl_live"] else "停发",
            a["rx_rate"], "  **掉线**" if a["lost"] else "") for a in snap["arms"]]
        self.put(y, 0, "两台：%s" % "  |  ".join(seg),
                 self.attr(C_WARN) if any(a["lost"] for a in snap["arms"]) else self.attr(C_DIM))
        y += 1

        if fb is None:
            self.put(y, 0, "还没收到任何反馈帧……", self.attr(C_WARN))
            y += 1
            if not snap["link_up"]:
                self.put(y, 0, "串口断了。检查接线后按 r 重开。", self.attr(C_ERR))
            elif time.monotonic() - self.sess.t0 > self.opts.handshake_timeout:
                self.put(y, 0, "握手超时（%ds 没收到一个字节）—— 按顺序查："
                         % int(self.opts.handshake_timeout), self.attr(C_ERR))
                for i, line in enumerate([
                    "1) 接线方向：适配器 TX → 板子 RX、适配器 RX → 板子 TX、GND 共地"
                    "（板子跑 UART 那条时是 RS485 半双工）；",
                    "2) 波特率 921600；3) 板子上跑的是不是这份固件（比对版本码 0x%08X / 版本串 %s）；"
                    % (protocol.VERSION_CODE, protocol.VERSION_STRING),
                    "4) 版本码不符时板子**静默不回包**，只能复位板子重来；",
                    "5) 板子的 ROBOT_LINK 是编译期二选一（UART/USB），和插的那个口子要对得上；",
                    "6) 这两条查完还没字节，看下面事件里「版本码不符」那一条。",
                ]):
                    self.put(y + 1 + i, 0, line, self.attr(C_WARN))
        else:
            missing = ~fb["inited"] & 0x7F
            extra = ""
            if fb["lost"]:
                extra += "  掉线: " + board_defs.expand_mask(fb["lost"])
            if missing:
                # 夹爪的 ∓142 软限位要**一次成功的找零之后**才生效（motor_inited[夹爪]
                # 由找零自己维护，别处没人写它），所以没标定好要看见。
                extra += "  未初始化: " + board_defs.expand_mask(missing)
                if missing >> GRIPPER & 1:
                    extra += "（夹爪软限位要一次成功设零才生效）"
            self.put(y, 0, "robot_state %s(%d)   arm_mode %s   inited 0x%02X   lost 0x%02X%s" % (
                board_defs.state_name(fb["state"]), fb["state"],
                board_defs.mode_name(fb["mode"]), fb["inited"], fb["lost"], extra),
                self.attr(C_ERR) if fb["lost"] else self.attr(C_WARN) if missing else 0)
            y += 1
            stream = "控制帧 %s" % ("在发" if self.sess.ctrl_live else "停发")
            if self.sess.hold:
                stream += "（已暂停 Space）"
            stream += "  %.0f Hz  步长 %s  选中 %s" % (
                self.opts.ctrl_hz, self.sess.step_str,
                board_defs.AXIS_NAMES[self.sess.sel])
            if self.sess.init_t0 is not None:
                stream = "初始化中 %.1f s（复位 → 夹爪找零）  保活中" \
                    % (time.monotonic() - self.sess.init_t0)
            elif self.sess.reset_t0 is not None:
                stream = "复位中 %.1f s（目标 %s rad）  保活中" % (
                    time.monotonic() - self.sess.reset_t0,
                    " ".join("%+.4f" % v for v in board_defs.RESET_POSE_RAD))
            self.put(y, 0, stream,
                     self.attr(C_WARN) if not self.sess.ctrl_live else self.attr(C_OK))
            y += 1
            self.put(y, 0, " 轴    目标角        反馈角        偏差      软限位(rad)      温度/单位",
                     self.attr(C_HEAD, curses.A_UNDERLINE))
            y += 1
            tgt = self.sess.target
            grip = self.sess.gripper_row(fb)
            for i in range(N_AXIS):
                mark = ">" if i == self.sess.sel else " "
                if i < GRIPPER:
                    fv = fb["joint"][i]
                    line = "%s%-4s %11.4f %13.4f %+11.4f   [%8.4f,%8.4f] %3d°C" % (
                        mark, board_defs.AXIS_NAMES[i], tgt[i], fv, tgt[i] - fv,
                        axis_lo_rad(i), axis_hi_rad(i), fb["temp"][i])
                else:
                    fv = grip["fb"]
                    unit = "m" if grip["calibrated"] else "raw rad"
                    if fv is None:      # 没反馈（或标定没有米数）：只画目标
                        line = "%s%-4s %11.4f %13s %11s   [%8.4f,%8.4f] %s" % (
                            mark, "夹爪", grip["tgt"], "-", "-", grip["lo"], grip["hi"], unit)
                    else:
                        line = "%s%-4s %11.4f %13.4f %+11.4f   [%8.4f,%8.4f] %s" % (
                            mark, "夹爪", grip["tgt"], fv, grip["tgt"] - fv,
                            grip["lo"], grip["hi"], unit)
                a = curses.A_REVERSE if i == self.sess.sel else 0
                if i == GRIPPER:
                    a |= self.attr(C_DIM)
                self.put(y, 0, line, a)
                y += 1
            self.put(y, 0, "jvel: " + " ".join("%+8.3f" % v for v in fb["jvel"]) + "  rad/s",
                     self.attr(C_DIM))
            y += 1
            if not grip["calibrated"]:
                # 这一行是"为什么夹爪那行的数看着怪"的答案（标定之后就没用了）
                self.put(y, 0, "夹爪未标定：那一行是**电机原始角**(rad)，软限位 ∓%.3f rad。"
                               "想要米就跑 python3 -m sdk.calibrate_gripper --arm-id %d"
                         % (board_defs.GRIPPER_ANGLE_MAX_RAD, arm_id), self.attr(C_DIM))
                y += 1

        # 事件区
        if snap["ack"] is not None and self.sess.have_ever_rx:
            ack = snap["ack"]
            self.put(y, 0, "上次设零 0x11: %s  耗时 %d ms  结果 %s" % (
                board_defs.ZERO_STATE_NAME.get(ack["state"], ack["state"]), ack["elapsed_ms"],
                " ".join("%s=%s" % (board_defs.AXIS_NAMES[i],
                                    board_defs.ZERO_RES_NAME.get(ack["result"][i], "?"))
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

        self.put(maxy - 2, 0, "a 切臂  1-6/0/Tab 选轴  ←↓→↑/+- 步进  [/] 步长  c 对齐  s run  "
                              "d 阻尼  x 停  v 重力  i 初始化  o 复位  z 设零  Space 暂停  "
                              "h 帮助  q 退出", self.attr(C_DIM))
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
                # tick() 现在是两台一起推进的，所以两台的保活都在这条路径上。
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
            if s.streaming_any():
                if self.input_line("有臂正发/曾发控制帧，确定退出？(y/n) ").strip().lower() == "y":
                    self.quit = True
            else:
                self.quit = True
        elif ch in (ord("h"), ord("?")):
            self.show_help = not self.show_help
        elif ch in (ord("a"), ord("A")):
            s.switch_arm()
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
            s.send_empty_mode(protocol.TID_ARM_START, "0x12 启动")
        elif ch in (ord("d"), ord("D")):
            s.send_empty_mode(protocol.TID_ARM_DAMPING, "0x15 阻尼")
        elif ch in (ord("v"), ord("V")):
            s.send_empty_mode(protocol.TID_ARM_GRAVITY, "0x13 重力补偿")
        elif ch in (ord("x"), ord("X")):
            s.send_empty_mode(protocol.TID_ARM_STOP, "0x14 停止")
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
        if fb is not None and fb["state"] == board_defs.STATE_NORMAL and not fb["lost"]:
            bad = [board_defs.JOINT_NAMES[i]
                   for i, want in enumerate(board_defs.RESET_POSE_RAD)
                   if abs(fb["joint"][i] - want) > self.opts.init_tol]
            if bad:
                s.warn("姿态离复位位姿超过 ±%.4f rad（%s）—— 复位期间是 run 档、臂会**使劲收拢**，"
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
        if fb["state"] != board_defs.STATE_WAIT_INIT:
            s.warn("0x10 只能在「待初始化」发（现在 %s）"
                   % board_defs.state_name(fb["state"]))
            return
        s.warn("0x10 会把**当前位姿**写成零点，并覆盖电机里的原零点！")
        s.warn("当前姿态(rad)：%s（复位位姿 %s）"
               % (" ".join("%+.4f" % v for v in fb["joint"]),
                  " ".join("%+.4f" % v for v in board_defs.RESET_POSE_RAD)))
        if self.input_line("第1步/2：把机械臂摆到基准位姿后逐字输入 SETZERO >") != "SETZERO":
            s.info("设零已取消（第 1 步没通过）")
            return
        if self.input_line("第2步/2：再逐字输入 CONFIRM 真正发送 0x10 >") != "CONFIRM":
            s.info("设零已取消（第 2 步没通过）")
            return
        s.send_setzero()

    def reopen(self):
        """重开链路（`--fake` 下就是"假板子重新上电"）。报错由后端收进事件流。"""
        self.sess.reopen()
        self.sess.dirty = True

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
# 自测 / main
# ============================================================================

def _fake_opts(**kw):
    """自测用的 opts（和 argparse 出来的那份同名同义）。"""
    d = dict(fake=True, port=None, baud=921600, arm_id=0, fake_lost=0, ctrl_hz=100.0,
             fb_hz=5.0, log=None, stop_on_exit=False, step=DEFAULT_STEP_RAD,
             init_tol=board_defs.ROBOT_INIT_POSE_TOL_RAD, handshake_timeout=3.0)
    d.update(kw)
    return argparse.Namespace(**d)


def selftest():
    """离线自测。不接硬件、不起 curses、不需要 pyserial。

    两块：**sdk 那一套**（协议 / 闸门 / 假板子，`sdk/selftest.py`），
    再**控制台后端这一套**（两台臂的接线、单位、步长、命令真的写出去了没有）。
    """
    import math

    from sdk import selftest as sdk_selftest
    from sdk.selftest import Checker
    from sdk.sim import SimLink

    import cli_backend as cb

    print("=" * 68)
    print("1) sdk 离线自测（协议 / 闸门 / 假板子）")
    print("=" * 68)
    rc1 = sdk_selftest.main([])

    print()
    print("=" * 68)
    print("2) 控制台后端自测（cli_backend，用假板子跑）")
    print("=" * 68)
    c = Checker()

    # ---- 单位与限位（rad 是面板的口径）----
    c.section("单位与软限位（rad）")
    c.check("j1 上限 = 90°", abs(cb.axis_hi_rad(0) - math.radians(90.0)) < 1e-12)
    c.check("j1 下限 = -115°", abs(cb.axis_lo_rad(0) - math.radians(-115.0)) < 1e-12)
    c.check("夹爪软限位 = ∓142°（= ∓284/2）",
            abs(cb.axis_hi_rad(GRIPPER) - math.radians(142.0)) < 1e-12
            and abs(cb.axis_lo_rad(GRIPPER) + math.radians(142.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(0, math.radians(120.0))
    c.check("夹到上限 90°", cl and abs(v - math.radians(90.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(0, math.radians(-120.0))
    c.check("夹到下限 -115°", cl and abs(v - math.radians(-115.0)) < 1e-12)
    v, cl = cb.clamp_axis_rad(3, 0.0)
    c.check("界内不夹、也不报夹过（就是拍板不要那个数）", (not cl) and v == 0.0)

    # ---- 步长档 / --arm-id 解析 ----
    c.section("步长档与 --arm-id")
    c.check("rad 档是 0.005/0.01/0.02/0.05/0.1/0.2",
            cb.STEP_CHOICES_RAD == [0.005, 0.01, 0.02, 0.05, 0.1, 0.2])
    c.check("默认 rad 档在档位里（≈ 旧版的 1.0°）",
            cb.DEFAULT_STEP_RAD in cb.STEP_CHOICES_RAD)
    c.check("米档递增且默认档在档位里",
            all(b > a > 0 for a, b in zip(cb.STEP_CHOICES_M, cb.STEP_CHOICES_M[1:]))
            and cb.DEFAULT_STEP_M in cb.STEP_CHOICES_M)
    ok = True
    for s in ("0", "1", "0x1", " 1"):
        try:
            if not 0 <= cb.parse_arm_id(s) < cb.N_ARM:
                ok = False
        except Exception:                                   # noqa: BLE001
            ok = False
    for s in ("2", "-1", "x", "", "1.0"):
        try:
            cb.parse_arm_id(s)
            ok = False          # 该拒的没拒
        except argparse.ArgumentTypeError:
            pass
    c.check("parse_arm_id 收 0/1/0x1/空格，拒 2/-1/x/空/1.0", ok)

    # ---- 两台臂的接线（走真实 SimLink：解析、分流、闸门都是真的）----
    class _RecLink(SimLink):
        """记下每一帧发出去的 tid —— 断言"该发的发了、不该发的一帧没发"。"""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.tids = []

        def send(self, tid, payload=b"", arm_id=protocol.DEFAULT_ARM_ID):
            self.tids.append((tid, arm_id))
            return super().send(tid, payload, arm_id)

    c.section("两台臂（一条链路、两个 Arm、状态机各跑各的）")
    opts = _fake_opts()
    # 假板子时钟 ×2：自测要等 0x02 的 3 s，真实时间等不起。
    # **不能再快了** —— 面板的保活是**真实时间**每 150 ms 一帧（tick 的 last_tx 用的是
    # time.monotonic），而假板子的 500 ms 看门狗按它自己的时钟走：×2 时 150 ms 实时 =
    # 300 ms 模拟（还有 40% 余量），×4 就变成 600 ms > 500，板子会一路被踢进 comm_lost
    # 卡在「离线等待」。真机演练就用默认的 ×1（面板本来该长什么样就是什么样）。
    link = _RecLink(speed=2.0)
    sess = cb.ConsoleSession(link, opts)
    link.open()
    try:
        def pump(sec, pred=None):
            """推 tick 到 pred 成立或超时（tick 是面板的唯一节拍来源）。"""
            end = time.monotonic() + sec
            while True:
                sess.tick(time.monotonic())
                if pred is not None and pred():
                    return True
                if time.monotonic() > end:
                    break
                time.sleep(0.002)
            return pred is None or pred()

        def sp(a):
            return sess.views[a].arm.snapshot()

        c.check("两台都收到反馈帧（0x80 按 arm_id 分流）",
                pump(6.0, lambda: sp(0) is not None and sp(1) is not None),
                "0x80 帧数 臂A=%s 臂B=%s"
                % (sess.views[0].arm.n_feedback, sess.views[1].arm.n_feedback))
        c.check("两台的目标角是各自独立的列表",
                sess.views[0].target is not sess.views[1].target)
        c.check("两台各自被播种（都对到自己的反馈上）",
                sess.views[0].seeded and sess.views[1].seeded)

        snap = sess.snapshot()
        fb = snap["fb"]
        c.check("snapshot 的键齐（面板的契约）",
                {"fb", "fb_t", "ack", "ack_t", "board_version", "link_up", "link_err",
                 "n_rx", "n_rx_fb", "rx_rate", "n_tx", "n_drop", "n_crc_err", "n_bad_arm",
                 "unknown_tids", "arm_id", "arms"} <= set(snap))
        c.check("fb 的键齐", {"mode", "state", "lost", "inited", "joint", "jvel", "gripper",
                            "gripper_raw_rad", "gripper_calibrated", "temp"} == set(fb))
        c.check("joint 是 6 个 rad（不是 deg）",
                len(fb["joint"]) == 6 and abs(fb["joint"][0]) <= math.pi)
        c.check("夹爪：标定标志与米数一致（没标定时 m 为 None，原始角一直在）",
                (fb["gripper"] is not None) == fb["gripper_calibrated"]
                and isinstance(fb["gripper_raw_rad"], float))
        c.check("n_rx_fb 是本臂口径（= 当前臂的 n_feedback）",
                snap["n_rx_fb"] == sess.cur.arm.n_feedback)
        c.check("arms 概览两台都在、按 id 排序",
                [a["arm_id"] for a in snap["arms"]] == [0, 1])

        # 切臂 + 点动只作用于当前臂
        before0 = list(sess.views[0].target)
        sess.switch_arm(1)
        c.check("a 键切到臂B（当前臂 = 1）", sess.cur_arm_id == 1 and sess.arm_id == 1)
        sess.jog(0, +1)
        c.check("点动只动当前臂（臂A 的目标角一动没动）",
                sess.views[0].target == before0
                and sess.views[1].target[0] != before0[0])
        c.check("sess.target 读的是当前臂那份", sess.target is sess.views[1].target)
        sess.switch_arm(0)
        c.check("再切回臂A", sess.cur_arm_id == 0 and sess.target is sess.views[0].target)
        c.check("hold 也按当前臂走（只挂臂A）",
                (sess.hold is False) and all(v.hold is False for v in sess.views.values()))

        # ---- 命令：该发的写出去、不该发的一帧都不发 ----
        c.section("命令与闸门（假板子的闸门是真的）")
        c.check("臂A 走到「待初始化」",
                pump(4.0, lambda: sp(0).robot_state == board_defs.STATE_WAIT_INIT))
        link.tids.clear()
        sess.send_arm_init()
        c.check("待初始化态按 i → 真的发出 0x02（盖当前臂的 arm_id）",
                (protocol.TID_ARM_INIT, 0) in link.tids, "发出的：%r" % (link.tids,))

        c.check("初始化跑完回到「正常运行」（板子 ~7 s）",
                pump(8.0, lambda: sp(0).robot_state == board_defs.STATE_NORMAL))
        link.tids.clear()
        sess.send_empty_mode(protocol.TID_ARM_START, "0x12 启动")
        c.check("正常运行态按 s → 发出 0x12（进 run）",
                (protocol.TID_ARM_START, 0) in link.tids)
        c.check("进 run 了（0x16 才有可能被受理）",
                pump(3.0, lambda: sp(0).mode == board_defs.MODE_RUN))

        n_warn = sum(1 for _, lv, _ in sess.events if lv == "warn")
        link.tids.clear()
        sess.send_arm_init()
        c.check("run 档下按 i → **一帧没发**（不谎报「已发」）",
                not link.tids and not sess.cur.force_init,
                "发出的：%r" % (link.tids,))
        c.check("run 档下按 i 给出恰好一条 warn（SDK 说清缺哪条闸门）",
                sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 1)
        c.check("run 档下按 i 不提供「再按一次强发」（强发也是被静默丢弃）",
                not any("可强发" in t for _, _, t in sess.events))

        n_warn = sum(1 for _, lv, _ in sess.events if lv == "warn")
        link.tids.clear()
        sess.send_setzero()
        c.check("正常运行态按 z → 一帧没发（0x10 只认待初始化）",
                not link.tids and sum(1 for _, lv, _ in sess.events if lv == "warn") == n_warn + 1)

        c.check("臂B 还停在「待初始化」（两台状态机各跑各的）",
                sp(1).robot_state == board_defs.STATE_WAIT_INIT, sp(1).robot_state_name)
        c.check("臂A 在发 0x16、臂B 没在发（控制流按台独立）",
                pump(3.0, lambda: sess.snapshot()["arms"][0]["ctrl_live"]
                     and not sess.snapshot()["arms"][1]["ctrl_live"]))
        c.check("streaming_any() 看的是两台（退出确认用）", sess.streaming_any())

        sess.switch_arm(1)
        link.tids.clear()
        sess.send_arm_init()
        c.check("臂B 按 i → 0x02 盖的是 arm_id=1（不是当前面板看的那台那么简单）",
                (protocol.TID_ARM_INIT, 1) in link.tids, "发出的：%r" % (link.tids,))
        c.check("臂B 初始化完到「正常运行」",
                pump(8.0, lambda: sp(1).robot_state == board_defs.STATE_NORMAL))
        sess.send_empty_mode(protocol.TID_ARM_START, "0x12 启动")
        c.check("臂B 进 run", pump(3.0, lambda: sp(1).mode == board_defs.MODE_RUN))
        c.check("两台都在发 0x16，各盖各的 arm_id",
                pump(3.0, lambda: sess.snapshot()["arms"][0]["ctrl_live"]
                     and sess.snapshot()["arms"][1]["ctrl_live"])
                and (protocol.TID_ARM_CTRL_POS, 0) in link.tids
                and (protocol.TID_ARM_CTRL_POS, 1) in link.tids)
        sess.switch_arm(0)

        # ---- reopen：两台都要重新播种 ----
        c.section("reopen（r 键 / 拔线重开）")
        c.check("reopen 成功", sess.reopen())
        c.check("reopen 后两台的播种都清了（等下一帧反馈）",
                all(not v.seeded for v in sess.views.values()))
        c.check("reopen 后两台都重新握手+播种（假板子重上电 → 0→1→2）",
                pump(6.0, lambda: all(v.seeded for v in sess.views.values())))

        # ---- stop_all（--stop-on-exit）----
        link.tids.clear()
        done = sess.stop_all()
        c.check("退出补 0x14：两台都发、都带各自的 arm_id",
                done == ["臂A", "臂B"]
                and (protocol.TID_ARM_STOP, 0) in link.tids
                and (protocol.TID_ARM_STOP, 1) in link.tids, "发出的：%r" % (link.tids,))
    finally:
        link.close()

    print()
    rc2 = c.summary()
    print()
    if rc1 or rc2:
        print("自测**没过**：见上面 FAIL。")
        return 1
    print("自测全部通过（sdk + 控制台后端）。")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="按 h 看按键表；--selftest / --fake 都不需要硬件。")
    ap.add_argument("--port", default=None,
                    help="串口设备（默认自动挑：优先板子 USB 0483:5740，退回 1a86 适配器）")
    ap.add_argument("--baud", type=int, default=921600, help="波特率（默认 921600）")
    ap.add_argument("--arm-id", type=parse_arm_id, default=DEFAULT_ARM_ID,
                    help="启动时「当前臂」是谁：0=CAN1/臂A，1=CAN2/臂B（默认 %d）。"
                         "**两台都在被同时驱动**，面板里按 a 随时切" % DEFAULT_ARM_ID)
    ap.add_argument("--ctrl-hz", type=float, default=100.0, help="0x16 下发频率（默认 100，每台）")
    ap.add_argument("--fb-hz", type=float, default=5.0, help="面板刷新率（默认 5，按键响应不受影响）")
    ap.add_argument("--step", type=float, default=DEFAULT_STEP_RAD,
                    help="初始步长 rad（%s；默认 %g）"
                         % (" / ".join("%g" % v for v in STEP_CHOICES_RAD), DEFAULT_STEP_RAD))
    ap.add_argument("--handshake-timeout", type=float, default=3.0, help="多久没收到字节就提示握手超时")
    ap.add_argument("--init-tol", type=float, default=board_defs.ROBOT_INIT_POSE_TOL_RAD,
                    help="姿态复核容差 rad（默认 %.4f = 板上的 %.0f°）。只用于 0x18 前的弹确认和"
                         "复位收尾的显示判据；0x02 的闸门在 sdk 里，恒为板上的 %.0f°"
                         % (board_defs.ROBOT_INIT_POSE_TOL_RAD,
                            board_defs.ROBOT_INIT_POSE_TOL_DEG,
                            board_defs.ROBOT_INIT_POSE_TOL_DEG))
    ap.add_argument("--fake", action="store_true", help="不接硬件，用内置假板子（sdk/sim.py）")
    ap.add_argument("--fake-lost", type=lambda s: int(s, 0), default=0,
                    help="假板子每隔一会给「当前臂」制造这个掉线掩码（如 0x41）")
    ap.add_argument("--selftest", action="store_true", help="离线自测，不接硬件不起 curses")
    ap.add_argument("--log", default=None, help="把事件追加写到文件（用 .log 名字，*.json 被 gitignore）")
    ap.add_argument("--stop-on-exit", action="store_true",
                    help="退出前给**两台**补发 0x14（默认不发：只停 0x16，约 500 ms 后板子自己进 damping）")
    opts = ap.parse_args()

    if opts.selftest:
        return selftest()

    if opts.step not in STEP_CHOICES_RAD:
        print("--step 只能是 %s 之一（单位是 rad）" % STEP_CHOICES_RAD)
        return 2

    if not opts.fake:
        try:
            opts.port = opts.port or pick_default_port()
            assert_port_allowed(opts.port)
        except LinkError as e:
            # 探针口在这里被拒（SDK 的文案里写了为什么），别把这条提示弄丢
            print("%s" % e)
            return 2
        if not os.path.exists(opts.port):
            print("链路设备 %s 不存在。\n  %s" % (opts.port, link_hint()))
            cands = list_links()
            if cands:
                for path, _kind, _ident in cands:
                    print("    %s" % describe_port(path))
            else:
                print("    （一个都没有：两条链路都没插）")
            return 2
        try:
            import serial  # noqa: F401
        except ImportError:
            print("缺 pyserial：pip3 install -r sdk/requirements.txt")
            return 2

    # curses 里要显示中文，必须在 initscr **之前**设好 locale（否则宽字符画不出来）
    locale.setlocale(locale.LC_ALL, "")

    sess = build_session(opts)
    try:
        sess.link.open()   # 链路错误要在进 curses 之前报出来
    except Exception as e:                                  # noqa: BLE001
        print("打开 %s 失败：%s" % (opts.port, e))
        if "Permission" in str(e):
            print("（多半是权限：把用户加进 dialout 组，或 sudo 跑）")
        return 2
    # 面板第一行就把"抓的是哪个设备"写清楚 —— 抓错口子是这里最容易犯的错。
    # 板子那侧走哪条链路是**编译期**定死的（App/inc/robot_state.h 的 ROBOT_LINK），
    # 两边选反了的现象是"一切都对但一个字节都没有"，所以把链路类型也标出来。
    sess.info("链路 %s" % (describe_port(opts.port) if not opts.fake
                          else "假板子（--fake，不接硬件）"))
    # 两台臂一条链路，所以"现在在看哪一台"必须是开机第一眼就看得见的东西
    sess.info("两台臂都在被驱动：%s、%s（一条链路、每台一个 Arm）；面板上按 a 切「当前臂」，"
              "现在看的是 %s"
              % (board_defs.arm_label(0), board_defs.arm_label(1),
                 board_defs.arm_label(opts.arm_id)))
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
                print("已给 %s 补发 0x14 停止。" % "、".join(sess.stop_all()))
            else:
                print("已停发两台的 0x16（目标冻结、臂保持）。约 500 ms 后板子会因 500 ms 无"
                      "有效帧进 comm_lost → damping；要立刻松劲按 x，或用 --stop-on-exit。")
            sess.link.close()
        except Exception:                                   # noqa: BLE001
            pass
        if ui.logf:
            ui.logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
