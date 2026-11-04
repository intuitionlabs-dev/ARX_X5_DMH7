# Ruckig C 接口

基于 [Ruckig Community v0.15.3](https://github.com/pantor/ruckig/tree/v0.15.3)
提供独立单轴位置轨迹接口。多个实例可分别控制多个关节；轴间不做同步。
核心为 C++17，调用端只需 C11 和 `ruckig_c.h`，不依赖 HAL、RTOS 或本仓库其他库。
上游源码、提交号和 MIT 许可证保存在 `vendor/`，构建与运行均不联网。

## 单精度版本

本地求解器内核、C 接口、轨迹缓存、常量和数学函数均使用 32 位 `float`。
这是对上游的单精度移植，数值改动记录在 [vendor/UPSTREAM.md](vendor/UPSTREAM.md)。
**C ABI 已改变**：旧双精度版本和带 `minimum_duration` 字段版本的调用方，
都必须用当前头文件重新编译并重新链接，不能混用旧结构或旧共享库。

单精度端点检查按积分量级允许舍入误差；精度不再等同于上游双精度版。
采样时间使用整数周期计数乘以周期，避免逐次累加带来的漂移。
库构建启用浮点提升检查，不使用 fast-math。验证覆盖本仓库普通关节位置控制
使用的角度、约束与 2 ms 周期；其他单位、极端量级和硬实时最坏耗时需另测。

## 独立构建

```bash
cmake -S libs/ruckig -B build/ruckig-host \
  -DCMAKE_BUILD_TYPE=Release -DRUCKIG_C_BUILD_TESTS=ON
cmake --build build/ruckig-host -j
ctest --test-dir build/ruckig-host --output-on-failure
./build/ruckig-host/ruckig_c_example
```

其他 CMake 工程可直接使用：

```cmake
project(my_controller LANGUAGES C CXX)
add_subdirectory(path/to/ruckig)
add_executable(controller controller.c)
target_link_libraries(controller PRIVATE ruckig::c)
```

CMake 会自动用 C++ 链接器链接最终程序。手动构建时，用 C 编译器编译调用端，
再用 C++ 链接器链接 `libruckig_c.a`；不要将 C 调用代码作为 C++ 编译。
该静态库可通过 `CMAKE_POSITION_INDEPENDENT_CODE=ON` 链接到共享库。

## 调用

完整的纯 C 示例见 [examples/position.c](examples/position.c)。

1. `ruckig_c_create(period_s)` 创建单轴实例，分配失败或周期无效时返回 `NULL`。
2. 填写 `ruckig_c_input`：当前和目标位置、速度、加速度，以及三个正数上限。
3. 每周期调用 `ruckig_c_update()`，将输出状态交给控制器，并赋给下一周期的 `input.current`。
4. 改变目标、上限或当前状态会触发重规划。沿已有轨迹运行时只做采样。
5. 切换控制源时可调用 `ruckig_c_reset()`，下一周期从新的当前状态开始。
6. 使用完毕调用 `ruckig_c_destroy()`。

长度单位由调用者保持一致，时间单位为秒。例如使用角度时，上限依次为
deg/s、deg/s²、deg/s³。始终选择满足起终状态和运动约束的最快可行轨迹，
没有额外的最短时长参数。单轴连续时间求解直接使用 Step 1 的结果，
不再进入按最短时长选取完成时间及 Step 2 定时求解的流程。
`output.duration` 和 `output.time` 是当前轨迹时长及采样时刻。

返回 0 表示运动中，1 表示完成，负数表示错误（定义见头文件）。
完成后保持目标状态，采样时间不再增长；若目标导数非零，也返回该终态，
后续运动需由调用者提交下一段目标。普通位置控制应将目标速度、加速度设为零。
出错时输出保持原值，轨迹缓存失效；调用方必须处理错误，不能继续下发旧输出。
允许初始速度／加速度超限，以便在不丢失连续性的情况下制动恢复；这种情况下
上限不会在第一周期立即满足。轨迹没有位置边界约束，运动中反向或制动可能越过目标点。

固定单轴实例在创建后无需动态分配；更新不调用系统时钟，不抛出异常。
`ruckig_c_context_size()` 返回单实例占用（不含分配器开销）。
同一实例不能被多个线程并发访问，也不能复制；不同实例相互独立。
固件需使用支持 C++17 的工具链，链接器需要保留正常的 C++ 初始化流程。
MCU 上的最坏执行时间和任务栈余量仍需在目标硬件测量。
