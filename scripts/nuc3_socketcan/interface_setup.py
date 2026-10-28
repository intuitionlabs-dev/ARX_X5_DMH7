#!/usr/bin/env python3
"""配置 CANable 串口参数和 SocketCAN 接口。"""

import fcntl
import os
import socket
import struct
import sys
import termios
import time
import tty


def prepare(device):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        discipline = fcntl.ioctl(fd, termios.TIOCGETD, struct.pack("i", 0))
        if struct.unpack("i", discipline)[0] != 0:
            raise OSError("串口已绑定线路规程")
        fcntl.ioctl(fd, termios.TIOCEXCL)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF)
        attrs[2] &= ~termios.CRTSCTS
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        for command in (b"C\r", b"S8\r", b"M0\r", b"A0\r"):
            if os.write(fd, command) != len(command):
                raise OSError("CANable 配置命令写入不完整")
            time.sleep(.02)
        termios.tcflush(fd, termios.TCIFLUSH)
    finally:
        os.close(fd)


def wait_interface(interface):
    deadline = time.monotonic() + 3
    while True:
        try:
            socket.if_nametoindex(interface)
            break
        except OSError:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"SocketCAN 接口未出现：{interface}")
            time.sleep(.02)



if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("prepare", "wait"):
        raise SystemExit("用法：interface_setup.py prepare 串口路径 | wait 接口名称")
    {"prepare": prepare, "wait": wait_interface}[sys.argv[1]](sys.argv[2])
