#!/usr/bin/env bash
#
# Leader_f103c8t6 自动编译脚本
#
# 用法:
#   ./cm.sh                  # 默认 Debug 编译
#   ./cm.sh -t Release       # Release 编译
#   ./cm.sh -c               # 清除构建目录后重新编译
#   ./cm.sh -j 8             # 指定并行任务数
#
# 产物: build/<Debug|Release>/Leader_f103c8t6.{elf,hex,bin,map}
#
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_ROOT}"

# 从 CMakeLists.txt 读取工程名，避免硬编码
APP_NAME="$(sed -n 's/^set(CMAKE_PROJECT_NAME[[:space:]]\+\([^)]*\).*/\1/p' CMakeLists.txt | head -1)"
[ -n "${APP_NAME}" ] || { echo "错误: 无法从 CMakeLists.txt 解析 CMAKE_PROJECT_NAME" >&2; exit 1; }

BUILD_TYPE="Debug"
DO_CLEAN=0
JOBS=""

usage() {
    sed -n '3,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        -t|--type)   BUILD_TYPE="$2"; shift 2 ;;
        -c|--clean)  DO_CLEAN=1; shift ;;
        -j|--jobs)   JOBS="$2"; shift 2 ;;
        -h|--help)   usage ;;
        *) echo "未知参数: $1 (使用 -h 查看帮助)" >&2; exit 1 ;;
    esac
done

case "${BUILD_TYPE}" in
    Debug|Release) ;;
    *) echo "错误: 构建类型只能是 Debug 或 Release" >&2; exit 1 ;;
esac

BUILD_DIR="${PROJECT_ROOT}/build/${BUILD_TYPE}"

# ---------- 依赖检查 ----------
for tool in cmake ninja; do
    command -v "${tool}" >/dev/null 2>&1 || { echo "错误: 未找到 ${tool}" >&2; exit 1; }
done

# arm-none-eabi- 工具链必须可用，若不在 PATH 中则在常见位置查找
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    for candidate in "${HOME}"/Tools/gcc-arm-none-eabi-*/bin /opt/gcc-arm-none-eabi*/bin /usr/lib/gcc/arm-none-eabi/*/; do
        if [ -x "${candidate}/arm-none-eabi-gcc" ]; then
            export PATH="${candidate}:${PATH}"
            break
        fi
    done
fi
command -v arm-none-eabi-gcc >/dev/null 2>&1 || {
    echo "错误: 未找到 arm-none-eabi-gcc，请将其 bin 目录加入 PATH" >&2; exit 1; }

OBJCOPY="$(command -v arm-none-eabi-objcopy)"
SIZE="$(command -v arm-none-eabi-size)"

# ---------- 清理 ----------
if [ "${DO_CLEAN}" -eq 1 ]; then
    echo ">> 清除 ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

# ---------- 配置 ----------
if [ ! -f "${BUILD_DIR}/CMakeCache.txt" ]; then
    echo ">> 配置工程 (${BUILD_TYPE})"
    cmake --preset "${BUILD_TYPE}"
else
    echo ">> 复用已有配置 (${BUILD_TYPE})"
fi

# ---------- 编译 ----------
echo ">> 编译 ${APP_NAME} (${BUILD_TYPE})"
BUILD_ARGS=(--build --preset "${BUILD_TYPE}")
[ -n "${JOBS}" ] && BUILD_ARGS+=(-j "${JOBS}")
cmake "${BUILD_ARGS[@]}"

# ---------- 生成 hex / bin ----------
ELF="${BUILD_DIR}/${APP_NAME}.elf"
[ -f "${ELF}" ] || { echo "错误: 未生成 ${ELF}" >&2; exit 1; }

"${OBJCOPY}" -O ihex   "${ELF}" "${BUILD_DIR}/${APP_NAME}.hex"
"${OBJCOPY}" -O binary "${ELF}" "${BUILD_DIR}/${APP_NAME}.bin"

