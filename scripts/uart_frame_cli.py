#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容壳：控制台已经搬进 `sdk/tui/`，这里只把仓库根铺进 sys.path 再转调 `python3 -m sdk.tui`。

参数、按键、行为都和以前一模一样（`--help` 里显示的仍是 `uart_frame_cli.py`）。
新写法是 `python3 -m sdk.tui`（要在仓库根跑）；两边是同一份代码，改功能去 `sdk/tui/`。
"""
import os
import sys

# `python3 scripts/uart_frame_cli.py` 时 sys.path[0] 是 scripts/、**不是仓库根**，
# 所以 `import sdk` 得自己先铺路（正是搬进包里之后不再需要的那一步）。
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from sdk.tui.__main__ import main   # noqa: E402


if __name__ == "__main__":
    sys.exit(main(prog=os.path.basename(sys.argv[0])))
