# ARX X5 H7 与 PC 控制工程

本仓库包含 H7 固件、PC 控制程序和 SDK。
各组件的文档位于对应目录。

| 组件 | 目录 | 文档入口 |
| --- | --- | --- |
| H7 固件 | `App/`、`Bsp/`、`Core/` | [通信协议](libs/frame/protocol_doc.md) |
| PC 控制程序 | `sdk/arx_x5_control/` | [使用说明](sdk/arx_x5_control/README.md) |
| SDK | `sdk/` | [使用说明](sdk/README.md)、[API](sdk/API.md)、[配置](sdk/config/README.md) |
| USB-CAN 传输子包 | `sdk/arx_x5_control/usb2can/` | [接口与配置](sdk/arx_x5_control/usb2can/README.md) |
| 动力学库 | `libs/gravity_compensation/` | [使用说明](libs/gravity_compensation/README.md) |

SDK 作为 Git 子模块管理。
在仓库根目录初始化：

```bash
git submodule update --init --recursive sdk
```

PC 控制程序的 SDK 接入说明位于
[sdk/arx_x5_control/SDK_LOCAL.md](sdk/arx_x5_control/SDK_LOCAL.md)。

在 H7 工程根目录安装 PC 控制包：

```bash
python3 -m pip install './sdk[can]'
```

SDK 仓库包含独立构建配置、控制代码和动力学模型。
集成测试在安装后运行：

```bash
python3 -m unittest discover -s tests -v
```
