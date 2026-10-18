# ARX X5 H7 与 PC 控制工程

本仓库包含 H7 固件、PC 控制程序和 SDK。
各组件的文档位于对应目录。

| 组件 | 目录 | 文档入口 |
| --- | --- | --- |
| H7 固件 | `App/`、`Bsp/`、`Core/` | [通信协议](libs/frame/protocol_doc.md) |
| PC 控制程序 | `arx_x5_control/` | [使用说明](arx_x5_control/README.md) |
| SDK | `sdk/` | [使用说明](sdk/README.md)、[API](sdk/API.md)、[配置](sdk/config/README.md) |
| USB-CAN 传输子包 | `arx_x5_control/usb2can/` | [接口与配置](arx_x5_control/usb2can/README.md) |
| 动力学库 | `libs/gravity_compensation/` | [使用说明](libs/gravity_compensation/README.md) |

SDK 作为 Git 子模块管理。
在仓库根目录初始化：

```bash
git submodule update --init --recursive sdk
```

PC 控制程序的 SDK 接入说明位于
[arx_x5_control/SDK_LOCAL.md](arx_x5_control/SDK_LOCAL.md)。
