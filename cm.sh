#!/usr/bin/env bash
# H7 固件编译与机械臂机型选择。
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_ROOT}"

# 从 CMakeLists.txt 读取工程名，避免硬编码
APP_NAME="$(sed -n 's/^set(CMAKE_PROJECT_NAME[[:space:]]\+\([^)]*\).*/\1/p' CMakeLists.txt | head -1)"
[ -n "${APP_NAME}" ] || { echo "错误: 无法从 CMakeLists.txt 解析 CMAKE_PROJECT_NAME" >&2; exit 1; }

BUILD_TYPE="Debug"
DO_CLEAN=0
JOBS=""
ARM_A=""
ARM_B=""

# 显示构建选项和各臂机型选择方式。
usage() {
    cat <<'EOF'
用法: ./cm.sh [选项]
  -t, --type Debug|Release  构建类型，默认 Debug
  -c, --clean               清除对应构建目录后重新编译
  -j, --jobs 数量           并行构建任务数
      --arm-a 机型          A 臂机型：metalarm 或 arx_x5
      --arm-b 机型          B 臂机型：metalarm 或 arx_x5
  -h, --help                显示帮助

终端运行时，未指定机型的臂通过菜单选择；回车采用已有构建配置。
首次构建默认使用 metalarm；非交互运行采用已有配置或首次默认值。
机型选择按 Debug、Release 分别保存，清理构建时沿用已选机型。

示例:
  ./cm.sh
  ./cm.sh --arm-a metalarm --arm-b metalarm
  ./cm.sh -t Release --arm-a arx_x5 --arm-b metalarm -j 8

产物: build/<Debug|Release>/<工程名>.{elf,hex,bin,map}
EOF
}

# 检查带值选项，避免缺少参数时读取未定义变量。
require_value() {
    if [ "$#" -lt 2 ] || [ -z "$2" ] || [[ "$2" == -* ]]; then
        echo "错误: $1 需要参数值 (使用 -h 查看帮助)" >&2
        exit 1
    fi
}

# 解析构建参数和各臂的显式机型选择。
while [ $# -gt 0 ]; do
    case "$1" in
        -t|--type)   require_value "$@"; BUILD_TYPE="$2"; shift 2 ;;
        -c|--clean)  DO_CLEAN=1; shift ;;
        -j|--jobs)   require_value "$@"; JOBS="$2"; shift 2 ;;
        --arm-a)    require_value "$@"; ARM_A="$2"; shift 2 ;;
        --arm-b)    require_value "$@"; ARM_B="$2"; shift 2 ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "未知参数: $1 (使用 -h 查看帮助)" >&2; exit 1 ;;
    esac
done

case "${BUILD_TYPE}" in
    Debug|Release) ;;
    *) echo "错误: 构建类型只能是 Debug 或 Release" >&2; exit 1 ;;
esac

if [ -n "${JOBS}" ] && ! [[ "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "错误: 并行任务数必须为正整数" >&2
    exit 1
fi

BUILD_DIR="${PROJECT_ROOT}/build/${BUILD_TYPE}"

# 从机型注册表读取菜单选项。
mapfile -t ARM_PROFILES < <(sed -n \
    's/^[[:space:]]*arm_register_profile([[:space:]]*\([^[:space:])]*\).*/\1/p' \
    App/profiles/CMakeLists.txt)
[ "${#ARM_PROFILES[@]}" -gt 0 ] || { echo "错误: 未找到已注册的机械臂机型" >&2; exit 1; }

# 校验机型是否已在 CMake 注册。
validate_profile() {
    local profile
    for profile in "${ARM_PROFILES[@]}"; do
        [ "$1" != "${profile}" ] || return 0
    done
    echo "错误: 无效机型 '$1'，可选值：${ARM_PROFILES[*]}" >&2
    return 1
}

# 读取对应构建目录中的机型，缺省时采用 metalarm。
cached_profile() {
    local profile=""
    if [ -f "${BUILD_DIR}/CMakeCache.txt" ]; then
        profile="$(sed -n "s/^H7_ARM_${1}_PROFILE:[^=]*=//p" "${BUILD_DIR}/CMakeCache.txt")"
    fi
    printf '%s\n' "${profile:-metalarm}"
}

# 终端菜单接受编号或机型名称，空输入保留默认值。
select_profile() {
    local slot="$1" default="$2" choice profile index
    while true; do
        printf '\n选择 %s 臂机型（默认 %s）：\n' "${slot}" "${default}" >&2
        for index in "${!ARM_PROFILES[@]}"; do
            printf '  %d) %s\n' "$((index + 1))" "${ARM_PROFILES[index]}" >&2
        done
        printf '请输入编号或机型名称，回车使用默认值: ' >&2
        if ! IFS= read -r choice; then
            echo "错误: 机型选择输入已结束" >&2
            return 1
        fi
        profile="${choice:-${default}}"
        for index in "${!ARM_PROFILES[@]}"; do
            if [ "${choice}" = "$((index + 1))" ]; then
                profile="${ARM_PROFILES[index]}"
                break
            fi
        done
        if validate_profile "${profile}"; then
            printf '%s\n' "${profile}"
            return 0
        fi
    done
}

# 在清理目录前确定机型，显式参数优先于菜单和缓存。
if [ -z "${ARM_A}" ]; then
    ARM_A="$(cached_profile A)"
    if [ -t 0 ]; then ARM_A="$(select_profile A "${ARM_A}")"; fi
fi
if [ -z "${ARM_B}" ]; then
    ARM_B="$(cached_profile B)"
    if [ -t 0 ]; then ARM_B="$(select_profile B "${ARM_B}")"; fi
fi
validate_profile "${ARM_A}"
validate_profile "${ARM_B}"

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

# ---------- 清理 ----------
if [ "${DO_CLEAN}" -eq 1 ]; then
    echo ">> 清除 ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

# ---------- 配置 ----------
echo ">> 配置工程 (${BUILD_TYPE}, A=${ARM_A}, B=${ARM_B})"
cmake --preset "${BUILD_TYPE}" \
    "-DH7_ARM_A_PROFILE=${ARM_A}" \
    "-DH7_ARM_B_PROFILE=${ARM_B}"

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

