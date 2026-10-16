# arx_x5_control 包专属日志库

本次日志规范的范围是 `arx_x5_control`。每个独立包拥有自己的日志入口、词条和配置，不借用另一个业务包的日志库。`arx_x5_control` 使用 `arx_x5_control.log`；它不会改变 SDK、重力模块或 Python 根日志器的语言、级别和处理器。

## 一处配置，整个包生效

```python
from arx_x5_control import log

# 应用启动时配置一次；默认中文、INFO 级别，输出到标准错误流。
log.configure(language="zh", level="INFO")

# 运行中切换语言：包内已有日志对象和后台控制线程在下一条输出时使用英文。
log.configure(language="en")

# 切回中文，同时查看握手和保活等调试事件。
log.configure(language="zh", level="DEBUG")
```

这里的“全局”是**当前包内全局**：所有 `arx_x5_control.*` 模块读取同一份配置。另一个独立包应在其自己的 `log` 库中保存配置和词条，不能复用 `arx_x5_control.log`；本库拒绝创建 `sdk` 等其他包命名空间的日志对象。

支持的语言为 `zh`、`en`，级别为 `DEBUG`、`INFO`、`WARNING`、`ERROR`、`CRITICAL`。`log.get_config()` 返回不可变的 `LogConfig` 快照。非法配置会报错，原配置保持不变。重复配置不会重复添加输出处理器。

仅导入包不会输出日志。命令行入口自动调用 `configure()`；作为库使用时，由应用调用 `configure()` 启用输出，或自行安装日志处理器。

## 命令行和环境变量

两个运行入口使用相同的包级配置方式：

```bash
python3 -m arx_x5_control --demo --lang zh
python3 -m arx_x5_control --demo --lang en --log-level DEBUG
python3 -m arx_x5_control.sdk_demo --lang en

# 本包专属环境变量，只由本包的命令行入口读取。
ARX_X5_CONTROL_LOG_LANGUAGE=en ARX_X5_CONTROL_LOG_LEVEL=WARNING python3 -m arx_x5_control --demo
```

选择顺序为：显式命令行参数 > 本包环境变量 > 已有包级配置。新进程默认中文和 `INFO`。安装后的 `arx-x5-control`、`arx-x5-sdk-demo` 命令支持相同选项。普通 Python 应用通过 `log.configure()` 设置，不依赖命令行环境变量。

## 包内新增日志的写法

```python
# 以下示例应位于 arx_x5_control 包内。
from . import log

_LOG = log.get_logger(__name__)

def start_loop():
    _LOG.info("runtime.started")
```

业务模块传入**词条键和原始参数**，不拼接待输出的整句文案。新增消息时，在 [`log_messages.py`](../arx_x5_control/log_messages.py) 中同时添加中英文词条，两个版本使用相同的格式参数。例如：

```python
_LOG.warning("event.watchdog", arm_id=0)
# 中文：臂 0 命令超时，已切换到阻尼模式
# 英文：Arm 0 command timeout; switched to damping mode
```

业务代码不得直接调用 `print()`、`logging`、`warnings` 或 `sys.stdout/sys.stderr` 来输出运行日志。它们的实际输出由 `log.py` 封装。`tests/test_log.py` 包含静态检查，防止后续模块绕过该入口；其他包不在本次检查范围内。

控制核心产生的 `Event` 仍是通信数据，字段、枚举和原始 `detail` 保持原样。运行循环、本地 SDK 后端和模拟入口通过 `_LOG.event(event)` 或 `_LOG.events(events)` 翻译后输出，语言变化不会改写事件本身。既有握手/保活事件为 `DEBUG`，状态变化为 `INFO`，拒绝命令和保活超时为 `WARNING`，初始化失败和重力模型错误为 `ERROR`。

## 日志与机器数据

默认人类可读日志写入标准错误流，格式如下：

```text
INFO [arx_x5_control.sdk_link] 臂 0 初始化完成
INFO [arx_x5_control.sdk_link] Arm 0 initialization complete
```

模拟结果等机器可读数据统一使用 `log.write_data(value)` 输出到标准输出，保留 JSON 字段、数值和枚举表示，不随显示语言变化。这样可以分别保存结果和日志：

```bash
python3 -m arx_x5_control --demo --lang zh > result.jsonl 2> run.log
```

标准输出只包含最终结果 JSON；控制事件以所选语言写入日志。需要保留原来的逐事件 JSON 数据时，仍可使用已有的 `demo(..., emit=callback)` 接口：

```python
from arx_x5_control.__main__ import demo
from arx_x5_control import log

log.configure(language="zh")
records = []
summary = demo(emit=records.append)
# records 保留每条原始事件及最终结果的 JSON 字符串。
```

外部适配器或 SDK 抛出的异常对象、未知事件详情等原始数据会保留原内容，本库不猜测或改写其他包的消息。日志的本包说明文案仍按配置语言输出。

## 接入应用自己的输出目标

```python
from arx_x5_control import log

with open("control.log", "a", encoding="utf-8") as stream:
    log.configure(language="zh", stream=stream)
    # 在此运行控制程序；结束后先退出控制线程，再关闭输出文件。
    log.configure(stream=None)
```

省略 `stream` 时保留原输出目标；首次调用默认选择当时的 `sys.stderr`。`stream=None` 禁用本库自带的输出处理器，不会移除应用自己添加的处理器。

应用也可在 `logging.getLogger("arx_x5_control")` 上安装自己的 `logging.Handler`，例如队列处理器。每条记录携带 `package`、`message_key`、`language`、`message_values` 字段；控制事件记录还带有原始 `control_event`。本包日志不向根日志器传播，避免与宿主应用的处理器重复输出。

默认处理器同步输出；应用的处理器和输出目标不得长时间阻塞控制线程。需要异步写文件时，由应用配置队列处理器。每条事件的状态名、原因和正文使用同一份语言快照，切换不会在一条消息中混入两个语言版本。

## 验证

```bash
python3 -m unittest discover -s tests -p test_log.py -v
python3 -m unittest discover -s tests -v
```

日志测试覆盖包间隔离、缓存对象及后台线程切换语言、处理器重复配置、级别过滤、词条与占位符完整性、控制事件翻译、JSON 兼容性、命令行/环境变量优先级和导入时静默行为。
