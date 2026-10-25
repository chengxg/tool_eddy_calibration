#!/usr/bin/env bash
# tool_eddy_calibration 卸载脚本
# 用法: ./uninstall.sh [klipper目录]    (默认 ~/klipper)
#
# 行为: 移除 klippy/extras/ 与 src/ 中由 install.sh 安装的文件,
#       并还原 src/Makefile 与 src/Kconfig 中由安装脚本添加的标记块。
#       只自动移除符号链接; 普通文件 (可能含手动修改) 提示后跳过,
#       需要手动删除。
set -euo pipefail

KLIPPER_DIR="${1:-$HOME/klipper}"
EXTRAS_DIR="$KLIPPER_DIR/klippy/extras"
SRC_DIR="$KLIPPER_DIR/src"

if [ ! -d "$EXTRAS_DIR" ]; then
    echo "错误: 未找到 $EXTRAS_DIR"
    echo "用法: $0 [klipper目录]"
    exit 1
fi

MARKER_BEGIN="# Begin tool_eddy_calibration additions"
MARKER_END="# End tool_eddy_calibration additions"

remove_file() {
    local dst="$1" own="$2"
    if [ -L "$dst" ]; then
        rm -f "$dst"
        echo "已移除: $dst"
    elif [ -e "$dst" ] && [ "$own" = own ]; then
        # 本插件专属文件 (sensor_ldc1612_2.c), 复制模式安装的也直接移除
        rm -f "$dst"
        echo "已移除: $dst"
    elif [ -e "$dst" ]; then
        echo "跳过: $dst 不是符号链接 (可能是目标自带或手动修改过的文件), 请确认后手动删除"
    fi
}

echo "== 主机端模块 =="
remove_file "$EXTRAS_DIR/tool_eddy_calibration.py" own
remove_file "$EXTRAS_DIR/ldc1612_2.py" own

echo ""
echo "== 固件源码 =="
# 本插件专属 MCU 模块
remove_file "$SRC_DIR/sensor_ldc1612_2.c" own

# 还原 Makefile / Kconfig 中安装脚本添加的标记块
for target in "$SRC_DIR/Makefile" "$SRC_DIR/Kconfig"; do
    if [ -f "$target" ] && grep -q "^${MARKER_BEGIN}$" "$target"; then
        sed -i "\|^${MARKER_BEGIN}$|,\|^${MARKER_END}$|d" "$target"
        echo "已还原: $target"
    fi
done

echo ""
echo "卸载完成。记得同时:"
echo "  1. 从 printer.cfg 删除 [tool_eddy_calibration ...] 配置节"
echo "  2. 从 moonraker.conf 删除 [update_manager tool_eddy_calibration] 配置节"
echo "  3. 若曾编译过本插件的固件模块, 重新编译烧录固件即可移除:"
echo "     cd $KLIPPER_DIR && make && <按主板方式烧录>"
echo "  4. 重启: sudo service klipper restart"
