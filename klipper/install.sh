#!/usr/bin/env bash
# tool_eddy_calibration 安装脚本
# 用法: ./install.sh [klipper目录] [--host-only]    (默认 ~/klipper)
#
# 行为:
#   1. 主机端模块安装到 klippy/extras/ (符号链接优先, 复制模式回退;
#      手动复制的旧文件备份为 *.bak)
#   2. 固件源码安装到 src/ (--host-only 跳过):
#      - sensor_ldc1612_2.c 为本插件自带 (双通道 MCU 采样模块)
#      - trigger_analog / sos_filter / sensor_ldc1612 / sensor_bulk
#        为固件自带, 缺失则报错退出 (本插件不支持旧版固件)
#   3. 幂等修补 src/Makefile 添加 sensor_ldc1612_2.c 编译条目
#      (带标记块, uninstall.sh 可完整移除)
#   4. 提示重新编译并烧录固件
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

KLIPPER_DIR=""
HOST_ONLY=0
for arg in "$@"; do
    case "$arg" in
        --host-only) HOST_ONLY=1 ;;
        *) KLIPPER_DIR="$arg" ;;
    esac
done
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"

EXTRAS_DIR="$KLIPPER_DIR/klippy/extras"
SRC_DIR="$KLIPPER_DIR/src"

if [ ! -d "$EXTRAS_DIR" ] || [ ! -d "$SRC_DIR" ]; then
    echo "错误: $KLIPPER_DIR 下未找到 klippy/extras 或 src 目录"
    echo "用法: $0 [klipper目录] [--host-only]"
    exit 1
fi

# 固件前置检查 (--host-only 跳过): 这些模块由较新的 Klipper/Kalico 自带,
# 缺失说明固件版本过旧, 直接报错退出, 不再补装
if [ "$HOST_ONLY" != 1 ]; then
    for base in sensor_ldc1612.c sensor_bulk.c \
                trigger_analog.c trigger_analog.h sos_filter.c sos_filter.h; do
        if [ ! -f "$SRC_DIR/$base" ]; then
            echo "错误: $SRC_DIR/$base 不存在, 固件版本过旧"
            echo "      本插件不支持旧版固件, 请先升级 Klipper/Kalico 再安装。"
            exit 1
        fi
    done
    if ! grep -q "config WANT_LDC1612" "$SRC_DIR/Kconfig" 2>/dev/null; then
        echo "错误: src/Kconfig 中没有 WANT_LDC1612 定义, 固件版本过旧"
        echo "      本插件不支持旧版固件, 请先升级 Klipper/Kalico 再安装。"
        exit 1
    fi
fi

MARKER_BEGIN="# Begin tool_eddy_calibration additions"
MARKER_END="# End tool_eddy_calibration additions"

# 安装单个文件: 符号链接优先, 不支持链接时复制
# $1=源文件 $2=目标文件 (内容相同则不动, 旧文件备份 *.bak)
install_file() {
    local src="$1" dst="$2"
    if [ ! -f "$src" ]; then
        echo "错误: 仓库中缺少 $src"
        exit 1
    fi
    if [ -f "$dst" ] && [ ! -L "$dst" ] && cmp -s "$src" "$dst"; then
        return 0    # 目标已是相同内容的普通文件, 不动
    fi
    if [ -L "$dst" ]; then
        rm -f "$dst"
    elif [ -e "$dst" ]; then
        mv "$dst" "$dst.bak"
        echo "已备份旧文件: $dst.bak"
    fi
    if ln -sf "$src" "$dst" 2>/dev/null && [ -L "$dst" ]; then
        echo "已安装(链接): $dst"
    else
        cp "$src" "$dst"
        echo "已安装(复制): $dst"
    fi
}

echo "== 主机端模块 =="
install_file "$SCRIPT_DIR/tool_eddy_calibration.py" \
             "$EXTRAS_DIR/tool_eddy_calibration.py" own
install_file "$SCRIPT_DIR/ldc1612_2.py" "$EXTRAS_DIR/ldc1612_2.py" own

FW_DIR="$SCRIPT_DIR/firmware"

if [ "$HOST_ONLY" = 1 ]; then
    echo ""
    echo "已按 --host-only 跳过固件安装。"
    echo "注意: 未打固件补丁时插件只能用单通道 (CHANNEL=0) 主机侧驱动。"
else
    echo ""
    echo "== 固件源码 =="
    # 本插件自带的 MCU 模块 (符号带 _2 后缀, 与原版 sensor_ldc1612.c 并存);
    # 其余依赖模块已在前置检查中确认存在 (固件自带)
    install_file "$FW_DIR/sensor_ldc1612_2.c" "$SRC_DIR/sensor_ldc1612_2.c"

    # Makefile: 删除旧标记块后, 追加缺失的编译条目 (条目已存在则跳过,
    # 避免同一 .c 被编译两次导致链接期符号重复;
    # trigger_analog.c / sos_filter.c 的条目固件 Makefile 已自带)
    makefile="$SRC_DIR/Makefile"
    sed -i "\|^${MARKER_BEGIN}$|,\|^${MARKER_END}$|d" "$makefile"
    {
        echo ""
        echo "$MARKER_BEGIN"
        line='src-$(CONFIG_WANT_LDC1612) += sensor_ldc1612_2.c'
        # 用子串匹配而非整行匹配, 兼容 CRLF 行尾的 checkout
        grep -qF "$line" "$makefile" || echo "$line"
        echo "$MARKER_END"
    } >> "$makefile"
    echo "已修补: $makefile"

    echo ""
    echo "固件源码已就位, 需要重新编译并烧录固件:"
    echo "  cd $KLIPPER_DIR"
    echo "  make menuconfig    # 确认启用 I2C (LDC1612 支持默认自动启用), 保存退出"
    echo "  make"
    echo "  # 按你的主板方式烧录, 例如: make flash FLASH_DEVICE=/dev/ttyACM0"
    echo "  烧录并重启后: TEST_LDC1612 验证传感器, TOOL_EDDY_VERSION 确认插件"
fi

echo ""
echo "安装完成。在 printer.cfg 中添加配置后执行 RESTART 生效, 例如:"
echo "  [tool_eddy_calibration my_coil]"
echo "  sensor_type: ldc1612"
echo "  coil_z: 0              # 线圈表面机器 Z (必填)"
echo "  find_coil_region: 90,90,110,110   # 找线圈的默认区域 (必填)"
echo "  # i2c_mcu / i2c_address / frequency / reg_drive_current 等"
echo "  # 传感器选项也写在本节内, 见 README.md"
echo "  # 线圈中心无需手填: TOOL_EDDY_FIND_COIL 搜索后由 G-code 保存"
echo ""
echo "如需 Moonraker 自动更新, 将以下内容追加到 moonraker.conf:"
echo "----------------------------------------------------------------"
cat "$REPO_DIR/config/moonraker.conf"
echo "----------------------------------------------------------------"
echo ""
echo "重启生效: sudo service klipper restart"
