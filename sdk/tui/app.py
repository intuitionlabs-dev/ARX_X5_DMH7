"""curses 面板：`Ui`（画屏 / 按键 / 对话框）。只认 `session` 那份接口契约。

后端在 [session.py](session.py)：链路、闸门预检、单位换算、每拍 tick、事件策略全在
`ConsoleSession` 里。这里只管把它给的"一帧一致视图"画出来，外加**在阻塞路径里也喂 tick**
（`input_line` 里那一次）—— 板子 500 ms 收不到有效帧就掉进 comm_lost，弹着对话框干等
会把臂踢出「正常运行」。

命令行入口在 [__main__.py](__main__.py)：`python3 -m sdk.tui`。
"""

import curses
import time

from .. import board_defs, protocol
from .session import GRIPPER, N_AXIS, axis_hi_rad, axis_lo_rad, short_label


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
                self.logf.write("# sdk.tui 启动 %s port=%s fake=%s 当前臂=%d\n"
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
