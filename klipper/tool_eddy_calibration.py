# 使用涡流传感器进行多热头XY偏移校准
# 原理：在已知线圈位置的基础上，沿多个方向扫描，检测频率峰值位置，通过几何方法重建工具中心坐标
# 该插件依赖klipper的ldc1612.py；若安装了支持多通道切换的 ldc1612_2.py
# 则优先使用其驱动（可运行时切换通道0-3），否则回退到官方单通道 ldc1612.py
# github开源地址: https://github.com/chengxg/tool_eddy_calibration
# 嘉立创开源地址: https://oshwhub.com/cxg01/project_lbabffjk
# version: V1.0.0 2026-06-01 实现基础XY偏移校准
# version: V2.0.0 2026-10-25 发布版本:
#          - 多通道线圈支持 (LDC1612/LDC1614, 通道 0-3 运行时切换,
#            每通道独立配置线圈参数)
#          - 线圈中心由 REGION 光栅搜索 (TOOL_EDDY_FIND_COIL) 定位,
#            经 G-code SAVE_VARIABLE 持久化, 校准时以 CENTER_XY 显式传入
#          - 单线圈模块兼容: coil_z 可选, 只配实际接线通道的参数
#          - 可信度判定: trusted/fitR2/deviation/edgeScans, 拟合 R² 须落在
#            [trust_min_r2, trust_max_r2) 且峰值不贴扫描线边缘
#          - 峰值搜索以扫描中段为基准, 抑制两侧杂散峰
#          - 修正抛物线拟合 b 系数行列式展开 (窗口截断时结果错误),
#            顶点越界按拟合失败处理不再钳制
#          - 扫描线/搜索区域自动钳制到运动学行程范围 (整线平移不截断),
#            粗中心贴区域边缘时不再 Move out of range
#          - Z 偏移由外部 tap 提供 (SET_TOOL_Z / [probe] 等)
# 安装方式: 见仓库 install.sh 与 README.md (git clone 后 ./install.sh)
#
# Copyright (C) 2026  b站-仁泉之子
# 本文件可在 GNU GPLv3 许可证条款下分发。
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, os, time
from . import ldc1612

# 插件版本, TOOL_EDDY_VERSION 命令与 get_status 可查询, 用于确认更新是否生效
TOOL_EDDY_VERSION = "2.0.0"

# 大范围找线圈: 响应极值偏离背景中位数的最低幅度 (Hz), 低于它视为区域内无线圈
FIND_COIL_MIN_RESPONSE_HZ = 50.0


def ensure_homed(printer, gcmd):
    """确认 XYZ 已回零, 否则报 gcode 错误"""
    curtime = printer.get_reactor().monotonic()
    kin_status = printer.lookup_object('toolhead').get_kinematics().get_status(curtime)
    if 'x' not in kin_status['homed_axes'] or \
       'y' not in kin_status['homed_axes'] or \
       'z' not in kin_status['homed_axes']:
        raise gcmd.error("Must home printer first")


def parse_center_xy(gcmd):
    """解析 CENTER_XY=x,y 参数; 未给出返回 None, 格式错误报 gcode 错误"""
    value = gcmd.get('CENTER_XY', None)
    if value is None:
        return None
    try:
        parts = [float(v.strip()) for v in value.split(',')]
        if len(parts) != 2:
            raise ValueError
    except ValueError:
        raise gcmd.error("CENTER_XY must be X,Y format (e.g., CENTER_XY=23,45)")
    return (parts[0], parts[1])


def parse_region_value(value, error):
    """解析 xmin,ymin,xmax,ymax；未给出返回 None，格式或大小错误时调用 error。"""
    if value is None:
        return None
    try:
        parts = [float(v.strip()) for v in value.split(',')]
        if len(parts) != 4:
            raise ValueError
    except ValueError:
        raise error(
            "REGION must be xmin,ymin,xmax,ymax format "
            "(e.g., REGION=10,0,30,20)")
    xmin, ymin, xmax, ymax = parts
    if xmax <= xmin or ymax <= ymin:
        raise error(
            "REGION is empty: %.3f x %.3f mm (xmax must exceed xmin, "
            "ymax must exceed ymin)" % (xmax - xmin, ymax - ymin))
    return (xmin, ymin, xmax, ymax)


def parse_region(gcmd):
    """解析 REGION=xmin,ymin,xmax,ymax G-code 参数。"""
    return parse_region_value(gcmd.get('REGION', None), gcmd.error)


def get_find_coil_region(config, option, default=None):
    """读取并校验 find_coil_region 配置项。"""
    return parse_region_value(config.get(option, None), config.error) or default



class ToolEddyCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')

        # 创建传感器（参考 probe_eddy_current.py）
        # sensor_type 仍为 "ldc1612"；若安装了支持双通道切换的 ldc1612_2.py
        # 则优先使用其驱动（可运行时切换通道0/1），否则回退到官方单通道
        # ldc1612.py（仅通道0）。
        sensor_type = config.getchoice('sensor_type', {'ldc1612': 'ldc1612'})
        self.has_ldc1612_2 = True
        try:
            from . import ldc1612_2
            sensor_class = ldc1612_2.LDC1612
        except ImportError:
            sensor_class = ldc1612.LDC1612
            self.has_ldc1612_2 = False
        self.sensor = sensor_class(config, None)

        # 扫描参数按通道分组。线圈 XY 中心不属于静态配置：首次通过
        # TOOL_EDDY_FIND_COIL REGION=... 找到后由 G-code 保存，后续每次
        # TOOL_EDDY_CALIBRATE 都以显式 CENTER_XY 为准。
        # 通道 0 用无后缀配置，通道 N (1-3) 用 N 后缀覆盖。
        # reg_drive_current 的分通道在 ldc1612_2.py 的 DriveCurrentCalibrate 里处理。
        ch0 = {
            'coil_z': config.getfloat('coil_z', None),
            'coil_inner_diameter': config.getfloat('coil_inner_diameter', 2.0,
                                                    minval=1.5, maxval=30.0),
            'scan_height': config.getfloat('scan_height', 0.5,
                                             minval=-20.0, maxval=50.0),
            'scan_speed': config.getfloat('scan_speed', 5.0,
                                            minval=0.5, maxval=20.0),
            'scan_length': config.getfloat('scan_length', 10.0,
                                             minval=2.0, maxval=20.0),
            'find_coil_region': get_find_coil_region(
                config, 'find_coil_region'),
        }
        self._channel_params = {0: ch0}
        for _ch in (1, 2, 3):
            self._channel_params[_ch] = {
                'coil_z': config.getfloat('coil_z%d' % _ch, ch0['coil_z']),
                'coil_inner_diameter': config.getfloat(
                    'coil_inner_diameter%d' % _ch, ch0['coil_inner_diameter'],
                    minval=1.5, maxval=30.0),
                'scan_height': config.getfloat('scan_height%d' % _ch,
                                                 ch0['scan_height'],
                                                 minval=-20.0, maxval=50.0),
                'scan_speed': config.getfloat('scan_speed%d' % _ch,
                                                ch0['scan_speed'],
                                                minval=0.5, maxval=20.0),
                'scan_length': config.getfloat('scan_length%d' % _ch,
                                                 ch0['scan_length'],
                                                 minval=2.0, maxval=20.0),
                'find_coil_region': get_find_coil_region(
                    config, 'find_coil_region%d' % _ch,
                    ch0['find_coil_region']),
            }

        # 非分通道参数 (各通道共用)
        self.travel_speed_xy = config.getfloat('travel_speed_xy', 100.0, minval=5.0, maxval=300.0)
        self.travel_speed_z = config.getfloat('travel_speed_z', 10.0, minval=1.0, maxval=50.0)
        self.scan_safe_z = config.getfloat('scan_safe_z', 2.0, minval=0.5, maxval=10.0)

        # 校准结果可信度判定 (结果写入 last_result 的 trusted/fitR2/deviation):
        # 结果偏离预扫描中心超过 trust_max_deviation、任一方向拟合度不在
        # [trust_min_r2, trust_max_r2) 区间 (拟合太差或好到近乎无噪声, 均
        # 视为异常)、或峰值落在扫描线边缘 (距端点不足 trust_edge_margin,
        # 此时峰值可能被截断, 拟合度再高也不可用) 均判为不可信
        self.trust_max_deviation = config.getfloat('trust_max_deviation', 1.0,
                                                    minval=0.05, maxval=10.0)
        self.trust_min_r2 = config.getfloat('trust_min_r2', 0.996,
                                             minval=0.0, maxval=1.0)
        self.trust_max_r2 = config.getfloat('trust_max_r2', 1.0,
                                             minval=0.0, maxval=1.0)
        self.trust_edge_margin = config.getfloat('trust_edge_margin', 1.0,
                                                  minval=0.1, maxval=10.0)

        # 数据保存选项
        self.save_data = config.getboolean('save_data', False)
        config_dir = os.path.dirname(os.path.abspath(
            self.printer.get_start_args()['config_file']))
        self.scan_data_dir = os.path.join(config_dir, 'tool_eddy_calibration')

        # 用活动通道 (默认通道 0) 的参数填充实例属性, 供 EddyCalibrator 读取
        self._apply_channel_params()

        # 探测结果存储（保存最后探测到的圆心坐标）
        self.last_result = {}
        self.last_coil_center = None

        # 注册命令
        self.gcode.register_command('TOOL_EDDY_CALIBRATE',
                                   self.cmd_TOOL_EDDY_CALIBRATE,
                                   desc=self.cmd_TOOL_EDDY_CALIBRATE_help)
        self.gcode.register_command('GET_TOOL_CENTER',
                                   self.cmd_GET_TOOL_CENTER,
                                   desc=self.cmd_GET_TOOL_CENTER_help)
        self.gcode.register_command('SET_TOOL_Z',
                                   self.cmd_SET_TOOL_Z,
                                   desc=self.cmd_SET_TOOL_Z_help)
        self.gcode.register_command('CLEAR_TOOL_CENTER',
                                   self.cmd_CLEAR_TOOL_CENTER,
                                   desc=self.cmd_CLEAR_TOOL_CENTER_help)
        self.gcode.register_command('TEST_LDC1612',
                                   self.cmd_TEST_LDC1612,
                                   desc=self.cmd_TEST_LDC1612_help)
        self.gcode.register_command('SET_TOOL_EDDY_CHANNEL',
                                   self.cmd_SET_TOOL_EDDY_CHANNEL,
                                   desc=self.cmd_SET_TOOL_EDDY_CHANNEL_help)
        self.gcode.register_command('TOOL_EDDY_VERSION',
                                   self.cmd_TOOL_EDDY_VERSION,
                                   desc=self.cmd_TOOL_EDDY_VERSION_help)
        self.gcode.register_command('TOOL_EDDY_FIND_COIL',
                                   self.cmd_TOOL_EDDY_FIND_COIL,
                                   desc=self.cmd_TOOL_EDDY_FIND_COIL_help)

    def _apply_channel_params(self):
        """把当前活动通道 (self.sensor.channel) 的扫描参数刷到实例属性。
        单通道 ldc1612.py 驱动无 channel 属性, 视作通道 0。
        在 __init__ 初始化及 _select_channel 切换通道后调用。"""
        ch = getattr(self.sensor, 'channel', 0)
        p = self._channel_params.get(ch, self._channel_params[0])
        self.coil_z = p['coil_z']
        self.coil_inner_diameter = p['coil_inner_diameter']
        self.scan_height = p['scan_height']
        self.scan_speed = p['scan_speed']
        self.scan_length = p['scan_length']
        self.find_coil_region = p['find_coil_region']

    def _check_coil_z(self, gcmd):
        """运行期校验: 活动通道的 coil_z 已配置 (扫描平面/安全 Z/下降曲线的
        高度换算都依赖它)。单线圈模块可只配任意一个通道, 缺失时报错指出
        该补哪一行配置。"""
        if self.coil_z is None:
            ch = getattr(self.sensor, 'channel', 0)
            suffix = '' if ch == 0 else '%d' % ch
            raise gcmd.error(
                "coil_z%s is not configured for channel %d in [%s]; add "
                "coil_z%s (the coil plane Z), or switch to a channel that "
                "has it configured (CHANNEL=)"
                % (suffix, ch, self.name, suffix))

    def _select_channel(self, gcmd):
        """从命令参数读取 CHANNEL (默认 0) 并切换到该通道, 再刷新线圈/扫描参数。
        每个需要读取传感器的命令开头调用, 无需先执行 SET_TOOL_EDDY_CHANNEL。
        通道 1-3 需 ldc1612_2.py; 单通道 ldc1612.py 仅支持通道 0。
        通道实际变化时清空缓存的工具 XY 中心；Z 是工具自身接触高度，与通道
        无关，保留。"""
        channel = gcmd.get_int('CHANNEL', 0, minval=0, maxval=3)
        if channel >= 1 and not self.has_ldc1612_2:
            raise gcmd.error(
                "Channels 1-3 require ldc1612_2.py to be installed; "
                "only the single-channel ldc1612.py is available")
        previous = getattr(self.sensor, 'channel', 0)
        if self.has_ldc1612_2:
            self.sensor.set_channel(channel)
        self._apply_channel_params()
        if channel != previous and self.last_result:
            for tool_name, result in self.last_result.items():
                self.last_result[tool_name] = {
                    'x': 0.0, 'y': 0.0,
                    'z': result.get('z', 0.0),
                }
            gcmd.respond_info(
                "%s: channel %d -> %d, cleared cached tool XY centers"
                % (self.name, previous, channel))
        return channel

    cmd_TOOL_EDDY_CALIBRATE_help = "Calibrate tool center using CENTER_XY=x,y (CHANNEL=0..3, default 0)"
    def cmd_TOOL_EDDY_CALIBRATE(self, gcmd):
        # 按命令参数 CHANNEL (默认 0) 选择通道并刷新线圈/扫描参数,
        # 无需先执行 SET_TOOL_EDDY_CHANNEL
        channel = self._select_channel(gcmd)
        # 获取工具名称（仅作为字符串标识符）
        tool_name = gcmd.get('TOOL', None)
        if tool_name is None:
            raise gcmd.error("TOOL parameter is required (e.g., TOOL=0)")

        self._check_coil_z(gcmd)
        center_xy = parse_center_xy(gcmd)
        if center_xy is None:
            raise gcmd.error(
                "CENTER_XY=x,y is required; first run TOOL_EDDY_FIND_COIL "
                "REGION=... and save its result with G-code SAVE_VARIABLE")

        # 获取扫描Z偏移（相对于scan_height，+远离线圈，-靠近线圈）
        scan_height_offset = gcmd.get_float('SCAN_HEIGHT_OFFSET', 0.0)

        # 获取重复次数
        repeats = gcmd.get_int('REPEATS', 1, minval=1, maxval=10)

        # 获取配对消除开关
        pair_cancel = gcmd.get_int('PAIR_CANCEL', 0, minval=0, maxval=1)

        # 获取扫描方向参数（角度，逗号分隔）
        dir_str = gcmd.get('DIR', '45,135')
        try:
            dir_angles = [float(a.strip()) for a in dir_str.split(',')]
        except ValueError:
            raise gcmd.error("DIR must be comma-separated angles (e.g., DIR=0,45,90,135)")
        if len(dir_angles) < 1:
            raise gcmd.error("At least 1 scan direction required (e.g., DIR=0,90)")

        toolhead = self.printer.lookup_object('toolhead')

        calibrator = EddyCalibrator(self, gcmd, scan_height_offset, repeats,
                                    bool(pair_cancel), tool_name)
        center_x, center_y, deviation, min_r2, edge_hits, trusted = \
            calibrator.run_calibration(center_xy, dir_angles)

        # 保存探测结果
        previous = self.last_result.get(tool_name, {})
        z_value = previous.get('z', 0.0)
        self.last_result[tool_name] = {
            'x': center_x,  # 测量的线圈的中心点位置
            'y': center_y,
            'z': z_value,
            'channel': channel,  # XY 校准所用的通道, Z 下降需与之一致
            'fitR2': min_r2,        # 各方向扫描拟合度的最小值 (最差一轮)
            'deviation': deviation, # 结果与初始扫描中心的偏差 (mm)
            'edgeScans': edge_hits, # 峰值落在扫描线边缘的扫描次数
            'trusted': trusted,     # False 时结果不可用, 宏/前端应丢弃
        }

        # 移动到检测到的中心点上方
        safe_z = self.coil_z + self.scan_height + self.scan_safe_z
        toolhead.manual_move([None, None, safe_z], self.travel_speed_z)
        toolhead.manual_move([center_x, center_y, None], self.travel_speed_xy)
        toolhead.wait_moves()

        # 报告结果 (含可信度; 不可信时列出原因)
        result = self.last_result[tool_name]
        if result['trusted']:
            gcmd.respond_info(
                "Tool '%s' center: X=%.4f Y=%.4f Z=%.4f (fit R²=%.4f, "
                "dev=%.3f mm, trusted)" %
                (tool_name, result['x'], result['y'], z_value,
                 result['fitR2'], result['deviation']))
        else:
            reasons = []
            if result['deviation'] > self.trust_max_deviation:
                reasons.append("deviation %.3f mm > %.3f mm"
                               % (result['deviation'], self.trust_max_deviation))
            if result['fitR2'] < self.trust_min_r2:
                reasons.append("fit R² %.4f < %.3f"
                               % (result['fitR2'], self.trust_min_r2))
            elif result['fitR2'] >= self.trust_max_r2:
                reasons.append("fit R² %.4f >= %.3f (too perfect, no noise?)"
                               % (result['fitR2'], self.trust_max_r2))
            if result['edgeScans']:
                reasons.append("%d peak(s) near scan edge"
                               % result['edgeScans'])
            gcmd.respond_info(
                "Tool '%s' center: X=%.4f Y=%.4f Z=%.4f UNTRUSTED (%s)" %
                (tool_name, result['x'], result['y'], z_value,
                 '; '.join(reasons)))

    cmd_TOOL_EDDY_FIND_COIL_help = (
        "Automatically locate the coil center: a serpentine raster over a "
        "rectangular region (GRID mm line spacing at RASTER_SPEED mm/s) "
        "finds the frequency response extremum, then orthogonal refinement "
        "scans fit the exact center. REGION=xmin,ymin,xmax,ymax overrides "
        "optional find_coil_region configuration. [GRID=1] "
        "[RASTER_SPEED=20] [CHANNEL=0]. "
        "Save the reported CENTER_XY with a G-code SAVE_VARIABLE macro. "
        "The whole region must be clear "
        "of obstacles at scan_height above the coil plane.")
    def cmd_TOOL_EDDY_FIND_COIL(self, gcmd):
        channel = self._select_channel(gcmd)
        self._check_coil_z(gcmd)
        region = parse_region(gcmd)
        if region is None:
            region = self.find_coil_region
        if region is None:
            raise gcmd.error(
                "REGION=xmin,ymin,xmax,ymax is required, or configure "
                "find_coil_region for this channel")
        xmin, ymin, xmax, ymax = region
        grid_mm = gcmd.get_float('GRID', 1.0, minval=0.5, maxval=10.0)
        raster_speed = gcmd.get_float('RASTER_SPEED', 20.0,
                                      minval=1.0, maxval=150.0)

        n_lines = max(2, int(math.ceil((ymax - ymin) / grid_mm)) + 1)
        est_time = ((n_lines * (xmax - xmin)) / raster_speed
                    + n_lines * 0.3)
        gcmd.respond_info(
            "Finding coil center: region X %.2f to %.2f, Y %.2f to %.2f "
            "(%.1f x %.1f mm), grid %.1f mm @ %.0f mm/s, %d lines, ~%.0f s, "
            "channel %d..."
            % (xmin, xmax, ymin, ymax, xmax - xmin, ymax - ymin, grid_mm,
               raster_speed, n_lines, est_time, channel))
        calibrator = EddyCalibrator(self, gcmd, tool_name='findcoil')
        center_x, center_y = calibrator.find_coil_center(
            xmin, ymin, xmax, ymax, grid_mm, raster_speed)
        self.last_coil_center = {
            'x': center_x,
            'y': center_y,
            'channel': channel,
        }

        gcmd.respond_info(
            "Coil center (channel %d): CENTER_XY=%.4f,%.4f"
            % (channel, center_x, center_y))
        gcmd.respond_info(
            "Save this CENTER_XY in your G-code macro with SAVE_VARIABLE "
            "before running TOOL_EDDY_CALIBRATE")

    cmd_GET_TOOL_CENTER_help = "Get the last detected tool center position"
    def cmd_GET_TOOL_CENTER(self, gcmd):
        tool_name = gcmd.get('TOOL', None)
        if tool_name is None:
            raise gcmd.error("TOOL parameter is required (e.g., TOOL=0)")

        if tool_name not in self.last_result:
            raise gcmd.error("Tool '%s' not calibrated yet" % tool_name)

        result = self.last_result[tool_name]
        msg = ("Tool '%s' center: X=%.4f Y=%.4f Z=%.4f"
               % (tool_name, result['x'], result['y'], result['z']))
        # XY 校准过的条目附带可信度信息; 仅 SET_TOOL_Z 初始化的条目没有
        if 'fitR2' in result:
            msg += " (fit R²=%.4f, dev=%.3f mm, %s)" % (
                result['fitR2'], result['deviation'],
                'trusted' if result['trusted'] else 'UNTRUSTED')
        gcmd.respond_info(msg)

    cmd_SET_TOOL_Z_help = "Set the Z coordinate for a tool (from external source)"
    def cmd_SET_TOOL_Z(self, gcmd):
        tool_name = gcmd.get('TOOL', None)
        if tool_name is None:
            raise gcmd.error("TOOL parameter is required (e.g., TOOL=0)")

        z = gcmd.get_float('Z', None)
        if z is None:
            raise gcmd.error("Z parameter is required")

        # 如果工具不存在，初始化数据
        if tool_name not in self.last_result:
            self.last_result[tool_name] = {
                'x': 0.0,
                'y': 0.0,
                'z': z,
            }
            gcmd.respond_info("Tool '%s' initialized with Z=%.4f" % (tool_name, z))
        else:
            # 更新 Z 值
            self.last_result[tool_name]['z'] = z

        result = self.last_result[tool_name]
        gcmd.respond_info(
            "Tool '%s' Z updated: X=%.4f Y=%.4f Z=%.4f" %
            (tool_name, result['x'], result['y'], result['z']))

    cmd_CLEAR_TOOL_CENTER_help = "Clear calibration result for a tool, or all if TOOL not specified"
    def cmd_CLEAR_TOOL_CENTER(self, gcmd):
        tool_name = gcmd.get('TOOL', None)
        if tool_name is None:
            count = len(self.last_result)
            self.last_result.clear()
            gcmd.respond_info("Cleared all %d tool(s) calibration data" % count)
        elif tool_name in self.last_result:
            del self.last_result[tool_name]
            gcmd.respond_info("Tool '%s' calibration data cleared" % tool_name)
        else:
            gcmd.respond_info("Tool '%s' has no data to clear" % tool_name)

    cmd_TEST_LDC1612_help = "Test LDC1612 sensor: collect 100ms of data and report frequency statistics (CHANNEL=0..3, default 0)"
    def cmd_TEST_LDC1612(self, gcmd):
        # 按命令参数 CHANNEL (默认 0) 选择要测试的线圈通道
        channel = self._select_channel(gcmd)
        duration = gcmd.get_float('DURATION', 0.1, minval=0.05, maxval=2.0)

        reactor = self.printer.get_reactor()
        toolhead = self.printer.lookup_object('toolhead')

        # 收集传感器数据
        is_finished = False
        collected_freqs = []
        error_count = 0

        def data_callback(msg):
            if is_finished:
                return False
            if 'data' in msg:
                for samp_time, freq, z in msg['data']:
                    if freq > 1000000:
                        collected_freqs.append(freq)
            error_count = msg.get('errors', 0)
            return True

        self.sensor.add_client(data_callback)

        # 等待传感器数据稳定
        reactor.pause(reactor.monotonic() + 0.05)
        toolhead.dwell(0.010)
        toolhead.wait_moves()

        # 清除稳定期的数据，重新开始收集
        collected_freqs.clear()

        # 收集指定时长的数据
        reactor.pause(reactor.monotonic() + duration)

        is_finished = True

        if not collected_freqs:
            gcmd.respond_info("LDC1612 test FAILED: no valid data received")
            gcmd.respond_info("  Check sensor wiring and configuration")
            return

        # 统计分析
        freq_min = min(collected_freqs)
        freq_max = max(collected_freqs)
        freq_avg = sum(collected_freqs) / len(collected_freqs)
        freq_range = freq_max - freq_min

        if len(collected_freqs) > 1:
            variance = sum((f - freq_avg)**2 for f in collected_freqs) / (len(collected_freqs) - 1)
            freq_std = math.sqrt(variance)
        else:
            freq_std = 0.0

        # 计算相邻样本的波动（更能反映短期稳定性）
        if len(collected_freqs) > 1:
            diffs = [abs(collected_freqs[i+1] - collected_freqs[i])
                     for i in range(len(collected_freqs) - 1)]
            avg_diff = sum(diffs) / len(diffs)
            max_diff = max(diffs)
        else:
            avg_diff = 0.0
            max_diff = 0.0

        sps = self.sensor.get_samples_per_second()

        gcmd.respond_info("LDC1612 sensor test results (channel %d):" % channel)
        gcmd.respond_info("  Samples: %d (expect ~%d in %.0fms @ %d sps)" %
                          (len(collected_freqs), int(sps * duration),
                           duration * 1000, sps))
        gcmd.respond_info("  Frequency: avg=%.1f Hz, min=%.1f Hz, max=%.1f Hz" %
                          (freq_avg, freq_min, freq_max))
        gcmd.respond_info("  Range: %.1f Hz (%.4f%%)" %
                          (freq_range, freq_range / freq_avg * 100 if freq_avg else 0))
        gcmd.respond_info("  Std dev: %.1f Hz" % freq_std)
        gcmd.respond_info("  Adjacent diff: avg=%.1f Hz, max=%.1f Hz" %
                          (avg_diff, max_diff))
        if error_count:
            gcmd.respond_info("  Errors: %d" % error_count)

        # 基于波动频率范围评估质量
        if freq_range < 20:
            gcmd.respond_info("  Quality: EXCELLENT - frequency range < 20Hz")
        elif freq_range < 50:
            gcmd.respond_info("  Quality: GOOD - frequency range < 50Hz")
        elif freq_range < 100:
            gcmd.respond_info("  Quality: ACCEPTABLE - frequency range < 100Hz")
        elif freq_range < 200:
            gcmd.respond_info("  Quality: NOISY - frequency range < 200Hz")
        else:
            gcmd.respond_info("  Quality: UNSTABLE - frequency range >= 200Hz")

    cmd_SET_TOOL_EDDY_CHANNEL_help = "Set the active LDC1612 coil/channel (0-3). Optional: each command also accepts CHANNEL= (default 0) directly"
    def cmd_SET_TOOL_EDDY_CHANNEL(self, gcmd):
        # 通道切换与参数刷新逻辑统一走 _select_channel; 各校准/测试命令也直接
        # 接受 CHANNEL= (默认 0), 通常无需先调用本命令。
        # 注意: 通道 2/3 仅 LDC1614 (4 通道) 芯片有物理引脚; LDC1612 (2 通道) 不存在。
        channel = self._select_channel(gcmd)
        if self.has_ldc1612_2:
            gcmd.respond_info("%s: active channel set to %d" % (self.name, channel))
        else:
            # 单通道驱动，channel 必为 0（channel >=1 已在 _select_channel 报错）
            gcmd.respond_info("%s: single-channel driver, using channel 0" % self.name)

    cmd_TOOL_EDDY_VERSION_help = "Show installed tool_eddy_calibration plugin version"
    def cmd_TOOL_EDDY_VERSION(self, gcmd):
        # 打印版本与当前驱动/通道, 用于安装或 Moonraker 更新后确认生效
        driver = 'ldc1612_2' if self.has_ldc1612_2 else 'ldc1612'
        gcmd.respond_info("tool_eddy_calibration: V%s (driver: %s, channel: %d)" %
                          (TOOL_EDDY_VERSION, driver,
                           getattr(self.sensor, 'channel', 0)))

    def get_status(self, eventtime):
        """返回状态信息，供外部查询使用"""
        status = dict(self.last_result)
        status['version'] = TOOL_EDDY_VERSION
        status['channel'] = getattr(self.sensor, 'channel', 0)
        status['has_ldc1612_2'] = self.has_ldc1612_2
        if self.last_coil_center is not None:
            status['coil_center'] = dict(self.last_coil_center)
        return status


class EddyCalibrator:
    def __init__(self, calibration, gcmd, scan_height_offset=0.0, repeats=3,
                 pair_cancel=False, tool_name=None):
        self.cal = calibration
        self.gcmd = gcmd
        self.scan_height_offset = scan_height_offset
        self.repeats = repeats
        self.pair_cancel = pair_cancel
        # 工具名用于 scan 数据文件名 (save_data 时区分不同工具的 CSV);
        # FIND_COIL 等无工具上下文的调用传 None, 文件名用 'findcoil'
        self.tool_name = tool_name
        self.printer = calibration.printer
        self.sensor = calibration.sensor

        # 获取所需对象
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')

    def run_calibration(self, center_xy, dir_angles):
        """以调用方给定的 CENTER_XY 为基准执行一次校准。"""
        # 如有需要先回零
        self._ensure_homed()

        center_x, center_y = center_xy

        # 配对消除：自动添加反方向
        if self.pair_cancel:
            full_dirs = set()
            for a in dir_angles:
                full_dirs.add(a % 360.0)
                full_dirs.add((a + 180.0) % 360.0)
            dir_angles = sorted(full_dirs)

        # 主扫描严格以本次 CENTER_XY 为基准；外部 G-code 可更新后迭代。
        initial_x, initial_y = center_x, center_y
        center_x, center_y = self._measure_center(
            center_x, center_y, dir_angles, "Scan", repeats=self.repeats)

        # 可信度判定: 结果偏离初始扫描中心太多、拟合度不在 [min_r2, max_r2)
        # 区间 (太差或好到近乎无噪声)、或峰值贴扫描线边缘 (峰值可能被截断,
        # 拟合度再高也不可用) 均判不可信
        deviation = math.hypot(center_x - initial_x, center_y - initial_y)
        min_r2 = getattr(self, '_pass_min_r2', 0.0)
        edge_hits = getattr(self, '_pass_edge_hits', 0)
        trusted = (deviation <= self.cal.trust_max_deviation
                   and self.cal.trust_min_r2 <= min_r2 < self.cal.trust_max_r2
                   and edge_hits == 0)

        return center_x, center_y, deviation, min_r2, edge_hits, trusted

    def _measure_center(self, scan_center_x, scan_center_y, dir_angles, pass_name,
                        repeats=None):
        # 多次测量取平均
        if repeats is None:
            repeats = self.repeats
        results_x = []
        results_y = []
        fit_r2_list = []   # 各方向扫描的拟合度 (拟合失败按 0 计)
        edge_hit_count = 0 # 峰值落在扫描线边缘的次数

        for repeat in range(repeats):
            peak_points = []

            for angle in dir_angles:
                peak_x, peak_y, peak_freq = self._scan_direction(
                    scan_center_x, scan_center_y, angle, repeat)
                if peak_freq is None:
                    raise self.gcmd.error("Failed to detect peak in DIR=%d direction" % int(angle))
                rad = math.radians(angle)
                cos_a, sin_a = math.cos(rad), math.sin(rad)
                peak_points.append((peak_x, peak_y, cos_a, sin_a, angle))
                # 记录拟合度; _refine_peak_position 拟合失败/被否决时为 None
                fit_r2_list.append(self._last_fit_r2
                                   if self._last_fit_r2 is not None else 0.0)
                # 边缘判定: 峰值投影到扫描方向上距扫描中心太远 (接近扫描线
                # 端点) 时峰值可能被截断, 即使拟合度高结果也不可用
                proj = ((peak_x - scan_center_x) * cos_a
                        + (peak_y - scan_center_y) * sin_a)
                if abs(proj) > (self.cal.scan_length / 2.0
                                - self.cal.trust_edge_margin):
                    edge_hit_count += 1
                    self.gcmd.respond_info(
                        "  WARNING: DIR=%d peak near scan edge "
                        "(offset %.2f mm of ±%.2f mm), result unreliable"
                        % (int(angle), proj, self.cal.scan_length / 2.0))
                fit = self._last_fit_info
                if fit is not None:
                    self.gcmd.respond_info(
                        "  DIR=%d scan peak: (%.4f, %.4f) freq=%.0f %s"
                        % (int(angle), peak_x, peak_y, peak_freq, fit))
                else:
                    self.gcmd.respond_info(
                        "  DIR=%d scan peak: (%.4f, %.4f) freq=%.0f"
                        % (int(angle), peak_x, peak_y, peak_freq))

            # 使用配对平均或LSQ从峰值投影重建中心坐标
            if self.pair_cancel:
                center_x, center_y = self._compute_center_paired_avg(peak_points)
            else:
                center_x, center_y = self._compute_center_lsq(peak_points)

            results_x.append(center_x)
            results_y.append(center_y)

            if repeats > 1:
                self.gcmd.respond_info("  Result %d: X=%.4f Y=%.4f" % (repeat + 1, center_x, center_y))

        # 计算平均值和标准差
        avg_x = sum(results_x) / len(results_x)
        avg_y = sum(results_y) / len(results_y)

        # 计算标准差
        std_x = math.sqrt(sum((x - avg_x)**2 for x in results_x) / len(results_x))
        std_y = math.sqrt(sum((y - avg_y)**2 for y in results_y) / len(results_y))

        # 显示统计信息
        if repeats > 1:
            self.gcmd.respond_info("%s statistics: X=%.4f±%.4f Y=%.4f±%.4f (n=%d)" %
                                  (pass_name, avg_x, std_x, avg_y, std_y, repeats))

        # 本轮质量信息存实例属性, run_calibration 在主扫描轮之后读取
        self._pass_min_r2 = min(fit_r2_list) if fit_r2_list else 0.0
        self._pass_edge_hits = edge_hit_count

        return avg_x, avg_y

    def find_coil_center(self, xmin, ymin, xmax, ymax, grid_mm, raster_speed):
        """大范围自动找线圈中心: 矩形区域内以 grid_mm 行距、raster_speed
        行进速度蛇形光栅粗扫, 定位频率响应极值, 再经 0°/90° 正交精扫
        (抛物线拟合, 仍用慢速 scan_speed) 得到精确中心。返回 (x, y)。
        前提: 扫描平面 (coil_z + scan_height) 在整个区域内安全无障碍。"""
        self._ensure_homed()

        # 区域钳制到行程内 (宏收缩区域已钳过, 这里兜底配置/命令区域越界)
        amin_x, amin_y, amax_x, amax_y = self._axis_limits()
        c_xmin, c_ymin = max(xmin, amin_x), max(ymin, amin_y)
        c_xmax, c_ymax = min(xmax, amax_x), min(ymax, amax_y)
        if c_xmin >= c_xmax or c_ymin >= c_ymax:
            raise self.gcmd.error(
                "Search region [%.1f,%.1f]-[%.1f,%.1f] has no overlap with "
                "axis limits [%.1f,%.1f]-[%.1f,%.1f]"
                % (xmin, ymin, xmax, ymax, amin_x, amin_y, amax_x, amax_y))
        if (c_xmin, c_ymin, c_xmax, c_ymax) != (xmin, ymin, xmax, ymax):
            self.gcmd.respond_info(
                "  Note: search region clamped to axis limits: "
                "X %.2f to %.2f, Y %.2f to %.2f"
                % (c_xmin, c_xmax, c_ymin, c_ymax))
            xmin, ymin, xmax, ymax = c_xmin, c_ymin, c_xmax, c_ymax

        # 行距均匀覆盖 [ymin, ymax], 末行压在边界上
        n_lines = max(2, int(math.ceil((ymax - ymin) / grid_mm)) + 1)
        spacing = (ymax - ymin) / (n_lines - 1)

        scan_z = self.cal.coil_z + self.cal.scan_height
        safe_z = scan_z + self.cal.scan_safe_z

        # 从区域一角进入, 蛇形光栅 (行间不抬升, 整个区域同高)
        self._move_to_position(xmin, ymin, safe_z)
        self._move_to_position(xmin, ymin, scan_z)

        samples = []
        for i in range(n_lines):
            y = ymin + i * spacing
            sx, ex = (xmin, xmax) if i % 2 == 0 else (xmax, xmin)
            self.toolhead.manual_move([sx, y, None], self.cal.travel_speed_xy)
            self.toolhead.wait_moves()
            samples.extend(
                self._collect_scan_data(sx, y, ex, y, speed=raster_speed))
            if (i + 1) % 10 == 0 and i + 1 < n_lines:
                self.gcmd.respond_info("  Raster progress: %d/%d lines"
                                       % (i + 1, n_lines))

        # 抬升脱离扫描平面
        self.toolhead.manual_move([None, None, safe_z], self.cal.travel_speed_z)
        self.toolhead.wait_moves()

        if len(samples) < 20:
            raise self.gcmd.error(
                "Coarse raster collected only %d valid samples; check sensor "
                "wiring and sample rate" % (len(samples),))

        # 极性判定: 极值偏离背景中位数的幅度, 谷底形取最小样本, 峰值形取最大样本
        freqs = sorted(f for _, _, f in samples)
        median = freqs[len(freqs) // 2]
        low = min(samples, key=lambda s: s[2])
        high = max(samples, key=lambda s: s[2])
        dip = median - low[2]
        bump = high[2] - median
        if max(dip, bump) < FIND_COIL_MIN_RESPONSE_HZ:
            raise self.gcmd.error(
                "No coil response found in the scanned region (max deviation "
                "%.1f Hz); enlarge RANGE, check CHANNEL, or check scan_height"
                % (max(dip, bump),))
        coarse = low if dip >= bump else high
        self.gcmd.respond_info(
            "  Raster: %.1f x %.1f mm, %d lines, %d samples, %s response "
            "%.1f Hz, coarse center (%.4f, %.4f)"
            % (xmax - xmin, ymax - ymin, n_lines, len(samples),
               'valley' if dip >= bump else 'peak', max(dip, bump),
               coarse[0], coarse[1]))

        # 正交精扫: 0° 扫描定 X, 90° 扫描定 Y (复用 _scan_direction 的
        # 抛物线拟合与日志)
        peak_x, _, freq_x = self._scan_direction(coarse[0], coarse[1], 0)
        _, peak_y, freq_y = self._scan_direction(coarse[0], coarse[1], 90)
        if freq_x is None or freq_y is None:
            raise self.gcmd.error(
                "Refinement scan found no peak near the coarse center")
        return peak_x, peak_y

    def _compute_center_lsq(self, peak_points):
        """使用LSQ从各方向峰值投影重建中心坐标
        peak_points: [(peak_x, peak_y, cos_θ, sin_θ, angle_deg), ...]
        每个方向的峰值投影到其扫描方向轴上，用LSQ求解中心
        """
        # 计算各方向的投影: proj_i = peak_x·cos(θ_i) + peak_y·sin(θ_i)
        projections = []  # [(cos_θ, sin_θ, projection_value)]
        for px, py, c, s, angle in peak_points:
            proj = px * c + py * s
            projections.append((c, s, proj))

        if not projections:
            return peak_points[0][0], peak_points[0][1]

        # LSQ求解: tx·cos(θ_i) + ty·sin(θ_i) = proj_i
        a11 = sum(c * c for c, s, _ in projections)
        a12 = sum(c * s for c, s, _ in projections)
        a22 = sum(s * s for c, s, _ in projections)
        c1 = sum(c * p for c, s, p in projections)
        c2 = sum(s * p for c, s, p in projections)

        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-12:
            avg_x = sum(p / c if abs(c) > abs(s) else 0.0
                       for c, s, p in projections) / len(projections)
            avg_y = sum(p / s if abs(s) > abs(c) else 0.0
                       for c, s, p in projections) / len(projections)
            return avg_x, avg_y

        tx = (a22 * c1 - a12 * c2) / det
        ty = (a11 * c2 - a12 * c1) / det
        return tx, ty

    def _compute_center_paired_avg(self, peak_points):
        """使用配对平均消除运动滞后，再用LSQ重建中心坐标
        peak_points: [(peak_x, peak_y, cos_θ, sin_θ, angle_deg), ...]
        将 θ 和 θ+180 配对，取投影的平均值消除滞后
        """
        by_angle = {}
        for px, py, c, s, angle in peak_points:
            norm_angle = angle % 360.0
            by_angle[norm_angle] = (px, py)

        paired_projections = []
        used = set()

        for angle, (px, py) in by_angle.items():
            if angle in used:
                continue
            opposite = (angle + 180.0) % 360.0
            rad = math.radians(angle)
            cos_a = math.cos(rad)
            sin_a = math.sin(rad)

            if opposite in by_angle and opposite not in used:
                ox, oy = by_angle[opposite]
                proj_fwd = px * cos_a + py * sin_a
                proj_bwd = ox * cos_a + oy * sin_a
                proj_avg = (proj_fwd + proj_bwd) / 2.0
                paired_projections.append((cos_a, sin_a, proj_avg))
                used.add(angle)
                used.add(opposite)
            else:
                proj = px * cos_a + py * sin_a
                paired_projections.append((cos_a, sin_a, proj))

        if not paired_projections:
            return peak_points[0][0], peak_points[0][1]

        # LSQ求解
        a11 = sum(c * c for c, s, _ in paired_projections)
        a12 = sum(c * s for c, s, _ in paired_projections)
        a22 = sum(s * s for c, s, _ in paired_projections)
        c1 = sum(c * p for c, s, p in paired_projections)
        c2 = sum(s * p for c, s, p in paired_projections)

        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-12:
            avg_x = sum(p / c if abs(c) > abs(s) else 0.0
                       for c, s, p in paired_projections) / len(paired_projections)
            avg_y = sum(p / s if abs(s) > abs(c) else 0.0
                       for c, s, p in paired_projections) / len(paired_projections)
            return avg_x, avg_y

        tx = (a22 * c1 - a12 * c2) / det
        ty = (a11 * c2 - a12 * c1) / det
        return tx, ty

    def _ensure_homed(self):
        """确保打印机已回零"""
        ensure_homed(self.printer, self.gcmd)

    def _axis_limits(self):
        """返回运动学行程范围 (xmin, ymin, xmax, ymax)"""
        curtime = self.printer.get_reactor().monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(curtime)
        amin = kin_status['axis_minimum']
        amax = kin_status['axis_maximum']
        return amin.x, amin.y, amax.x, amax.y

    def _clamp_scan_line(self, start_x, start_y, end_x, end_y, cos_a, sin_a):
        """把扫描线整体沿扫描方向平移, 使两端点都落回行程范围内,
        长度与方向保持不变; 行程内放不下时报 gcode 错误。
        找线圈精扫的粗中心可能贴在区域边缘, 起点越界不钳制会直接
        Move out of range; 平移后峰值若不在本扫描线上, 由拟合阶段的
        no-peak 报错兜底。返回 (start_x, start_y, end_x, end_y)。"""
        amin_x, amin_y, amax_x, amax_y = self._axis_limits()
        # 沿 (cos_a, sin_a) 方向的可行平移量 t 的区间, 两轴约束取交集
        t_lo, t_hi = -1e9, 1e9
        for name, d, s, e, amin, amax in (
                ('X', cos_a, start_x, end_x, amin_x, amax_x),
                ('Y', sin_a, start_y, end_y, amin_y, amax_y)):
            if abs(d) < 1e-9:
                # 扫描线在该轴上恒定, 只能要求已在范围内
                if s < amin - 1e-9 or s > amax + 1e-9:
                    raise self.gcmd.error(
                        "Scan line at %s=%.3f is outside axis limits "
                        "[%.3f, %.3f]" % (name, s, amin, amax))
                continue
            lo, hi = (s, e) if s <= e else (e, s)
            if d > 0:
                cur_lo, cur_hi = (amin - lo) / d, (amax - hi) / d
            else:
                cur_lo, cur_hi = (amax - hi) / d, (amin - lo) / d
            t_lo = max(t_lo, cur_lo)
            t_hi = min(t_hi, cur_hi)
        if t_lo > t_hi:
            raise self.gcmd.error(
                "Scan length %.1fmm does not fit within axis limits"
                % self.cal.scan_length)
        # 本来就在范围内则不动 (t=0), 否则取离 0 最近的边界平移
        t = min(max(0.0, t_lo), t_hi)
        if t != 0.0:
            self.gcmd.respond_info(
                "  Note: scan line shifted %.3fmm along scan direction "
                "to stay within axis limits" % t)
        return (start_x + t * cos_a, start_y + t * sin_a,
                end_x + t * cos_a, end_y + t * sin_a)

    def _scan_direction(self, center_x, center_y, angle_deg, repeat_idx=None):
        """沿指定角度方向扫描，经过中心点
        angle_deg: 扫描方向角度（度），0=X+, 90=Y+
        起点为中心点偏移 -L/2 方向，终点为 +L/2 方向
        repeat_idx: 本轮重复序号 (从 0 计), 仅用于 scan 数据文件名
        """
        half_len = self.cal.scan_length / 2.0
        rad = math.radians(angle_deg)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)

        start_x = center_x - half_len * cos_a
        start_y = center_y - half_len * sin_a
        end_x = center_x + half_len * cos_a
        end_y = center_y + half_len * sin_a

        # 两端点钳制到行程内 (整线平移, 见 _clamp_scan_line)
        start_x, start_y, end_x, end_y = self._clamp_scan_line(
            start_x, start_y, end_x, end_y, cos_a, sin_a)

        # 文件名含方向+轮次, tool/channel 由 _save_scan_data 统一拼入
        scan_name = "DIR_%d" % int(angle_deg)
        if repeat_idx is not None:
            scan_name += "_r%d" % repeat_idx

        # 计算实际 Z 坐标（相对 coil_z，+远离线圈，-靠近线圈）
        effective_scan_height = self.cal.scan_height + self.scan_height_offset
        # scan_z 不能低于 coil_z，防止碰撞线圈
        if effective_scan_height < 0:
            self.gcmd.respond_info(
                "  WARNING: scan_z would be below coil_z, "
                "clamping effective offset from %.4f to 0" %
                effective_scan_height)
            effective_scan_height = 0.0
        scan_z = self.cal.coil_z + effective_scan_height
        safe_z = self.cal.coil_z + effective_scan_height + self.cal.scan_safe_z

        # 移动到起始位置（安全 Z 高度）
        self._move_to_position(start_x, start_y, safe_z)
        # 降到扫描高度
        self._move_to_position(start_x, start_y, scan_z)

        # 采集数据
        samples = self._collect_scan_data(start_x, start_y, end_x, end_y)

        # 抬高 Z
        self._move_to_position(end_x, end_y, safe_z)

        # 找峰值并保存数据
        return self._find_peak(samples, scan_name)

    def _move_to_position(self, x, y, z):
        """移动工具头到指定位置，XY和Z使用不同速度"""
        # 先移动X再移动Y（快速），再移动Z（慢速）
        self.toolhead.manual_move([x, None, None], self.cal.travel_speed_xy)
        self.toolhead.manual_move([None, y, None], self.cal.travel_speed_xy)
        self.toolhead.manual_move([None, None, z], self.cal.travel_speed_z)
        self.toolhead.wait_moves()

    def _collect_scan_data(self, start_x, start_y, end_x, end_y, speed=None):
        """
        在直线扫描过程中采集频率数据
        返回：[(x, y, freq), ...]
        speed 缺省用精扫速度; 粗扫可传更快的速度
        """
        # 计算扫描参数
        if speed is None:
            speed = self.cal.scan_speed
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        scan_time = distance / speed

        # 开始传感器数据采集
        reactor = self.printer.get_reactor()

        # 设置数据采集回调
        is_finished = False
        collected_samples = []
        def data_callback(msg):
            if is_finished:
                return False  # 返回 False 以从 client_cbs 中自动移除
            if 'data' in msg:
                collected_samples.append(msg)
            return True

        # 先注册回调，确保不会遗漏任何数据
        self.sensor.add_client(data_callback)

        # 等待一下，让传感器数据稳定且回调生效
        reactor.pause(reactor.monotonic() + 0.05)

        try:
            # 添加短暂延迟，确保运动队列准备好
            self.toolhead.dwell(0.010)

            # 获取当前最后运动时间作为基准（打印时间）
            move_start_print_time = self.toolhead.get_last_move_time()

            # 启动扫描移动
            self.toolhead.manual_move([end_x, end_y, None], speed)
            self.toolhead.wait_moves()

            # 稍等，确保最后的数据到达
            reactor.pause(reactor.monotonic() + 0.2)

        finally:
            is_finished = True

        # 处理采集到的样本
        # 使用运动开始的打印时间作为基准
        samples = []

        # 统计信息
        skipped_before = 0
        skipped_after = 0
        skipped_invalid = 0

        for msg in collected_samples:
            for samp_time, freq, z in msg['data']:
                # 跳过无效的低频样本（传感器启动时的噪声）
                if freq < 1000000:
                    skipped_invalid += 1
                    continue

                relative_time = samp_time - move_start_print_time

                if relative_time < -0.05 or relative_time > scan_time + 0.05:
                    if relative_time < 0:
                        skipped_before += 1
                    else:
                        skipped_after += 1
                    continue

                if relative_time < 0:
                    skipped_before += 1
                    continue

                t = relative_time / scan_time
                x = start_x + t * (end_x - start_x)
                y = start_y + t * (end_y - start_y)

                samples.append((x, y, freq))

        if len(samples) < 10:
            self.gcmd.respond_info("  WARNING: only %d valid samples" % len(samples))

        return samples

    def _auto_detect_peak_type(self, freq_data):
        """自动检测信号是峰值形还是谷底形
        比较中心区域与边缘区域的频率均值：
        - 中心高、边缘低 → peak（大线圈特征）
        - 中心低、边缘高 → valley（小线圈特征）
        返回：'peak' 或 'valley'
        """
        n = len(freq_data)
        # 边缘区域取前后15%
        edge_margin = max(1, n // 7)
        edge_freqs = freq_data[:edge_margin] + freq_data[-edge_margin:]
        edge_avg = sum(edge_freqs) / len(edge_freqs)

        # 中心区域取中间30%
        center_start = n * 35 // 100
        center_end = n * 65 // 100
        if center_end <= center_start:
            center_end = center_start + 1
        center_freqs = freq_data[center_start:center_end]
        center_avg = sum(center_freqs) / len(center_freqs)

        detected = 'peak' if center_avg > edge_avg else 'valley'
        return detected, center_avg, edge_avg

    def _find_peak(self, samples, scan_name=None):
        """
        检测工具产生的频率极值点（峰值或谷值）
        peak模式：中心处频率最高（大线圈）
        valley模式：中心处频率最低（小线圈）
        auto模式：自动检测信号形态
        返回：(peak_x, peak_y, peak_freq)
        """
        if len(samples) < 10:
            self.gcmd.respond_info("  ERROR: Not enough samples (%d)" % len(samples))
            return None, None, None

        freq_data = [s[2] for s in samples]

        # 保存数据到文件用于分析（如果启用）
        if scan_name and self.cal.save_data:
            self._save_scan_data(samples, scan_name)

        # 自动检测peak_type（每次扫描都重新检测）
        detected_type, center_avg, edge_avg = self._auto_detect_peak_type(freq_data)
        auto_detect_info = (detected_type, center_avg, edge_avg)

        # 排除边缘区域（前15%和后15%）
        margin = (len(freq_data) * 15) // 100
        search_start = max(1, margin)
        search_end = min(len(freq_data) - 1, len(freq_data) - margin)

        # 峰值搜索以扫描中段 (中间50%) 为基准: 扫描中心就是传入的 CENTER_XY
        # (该工具自己的线圈中心), 真实峰值必在中段附近; 两侧的杂散响应
        # (邻近线圈/PCB 走线/固定件) 即使幅度更大也只作提示, 不参与定位
        center_start = max(search_start, len(freq_data) // 4)
        center_end = min(search_end, len(freq_data) * 3 // 4)
        if center_end <= center_start:
            center_start, center_end = search_start, search_end

        find_min = detected_type == 'valley'
        def find_extreme(lo, hi):
            idx = lo
            best = freq_data[lo]
            for i in range(lo + 1, hi):
                if (freq_data[i] < best) if find_min else (freq_data[i] > best):
                    best = freq_data[i]
                    idx = i
            return idx, best

        best_idx, best_freq = find_extreme(center_start, center_end)

        # 全范围 (15%~85%) 内若有更强且位置不同的极值, 视为被过滤的旁峰,
        # 打印其位置供排查 (不改变中段结果)
        wide_idx, wide_freq = find_extreme(search_start, search_end)
        if (wide_idx != best_idx
                and ((wide_freq < best_freq) if find_min
                     else (wide_freq > best_freq))):
            side = samples[wide_idx]
            self.gcmd.respond_info(
                "  Note: side %s at (%.3f, %.3f) %.1f Hz suppressed, "
                "using central %s at (%.3f, %.3f) %.1f Hz"
                % (detected_type, side[0], side[1], wide_freq,
                   detected_type, samples[best_idx][0], samples[best_idx][1],
                   best_freq))

        # 在极值点附近使用抛物线拟合以提升亚采样精度
        peak_x, peak_y = self._refine_peak_position(samples, best_idx, detected_type,
                                                       auto_detect_info)

        return peak_x, peak_y, freq_data[best_idx]

    def _save_scan_data(self, samples, scan_name):
        """保存扫描数据到CSV文件"""
        if not self.cal.save_data:
            return

        try:
            os.makedirs(self.cal.scan_data_dir, exist_ok=True)
            # 文件名区分 tool / channel / 方向 / 轮次:
            # tool_eddy_scan_tool{tool}_ch{channel}_{scan_name}.csv
            # (FIND_COIL 的精扫无工具上下文, tool 段为 findcoil)
            tool = self.tool_name if self.tool_name else 'findcoil'
            channel = getattr(self.sensor, 'channel', 0)
            filename = os.path.join(
                self.cal.scan_data_dir,
                "tool_eddy_scan_tool%s_ch%d_%s.csv" % (tool, channel, scan_name))
            sps = self.sensor.get_samples_per_second()
            with open(filename, "w") as f:
                # 写入参数行（以#开头，便于CSV兼容）
                f.write("# tool=%s,channel=%d,scan_length=%.1f,scan_speed=%.1f,"
                        "coil_inner_diameter=%.1f,peak_type=auto,"
                        "scan_height=%.1f,sps=%.0f\n" % (
                            tool, channel, self.cal.scan_length,
                            self.cal.scan_speed, self.cal.coil_inner_diameter,
                            self.cal.scan_height, sps))
                f.write("x_mm,y_mm,frequency_hz\n")
                for x, y, freq in samples:
                    f.write("%.4f,%.4f,%.1f\n" % (x, y, freq))
        except OSError as e:
            raise self.gcmd.error(
                "Unable to save scan data to %s: %s" %
                (self.cal.scan_data_dir, str(e)))

    def _refine_peak_position(self, samples, peak_idx, detected_type,
                                auto_detect_info=None):
        """
        使用加权最小二乘法对峰值附近点进行抛物线拟合，细化峰值位置
        detected_type: 'peak' 或 'valley'，用于判断抛物线开口方向
        """
        # 拟合质量信息存到实例属性, 由调用方 (_measure_center) 合并进
        # "DIR=... scan peak" 一行输出; _last_fit_r2 为数值拟合度, 供可信度
        # 判定取最小值。拟合失败/被否决时保持 None (视为 0)
        self._last_fit_info = None
        self._last_fit_r2 = None
        n_samples = len(samples)

        # 获取峰值附近的点用于抛物线拟合
        # 根据线圈内径计算窗口大小：取内径-0.5mm范围内的数据点
        effective_radius = (self.cal.coil_inner_diameter - 0.5) / 2.0
        sps = self.sensor.get_samples_per_second()
        if sps > 0.0 and self.cal.scan_speed > 0.0:
            samples_per_mm = sps / self.cal.scan_speed
            half_window = int(samples_per_mm * effective_radius)
        else:
            half_window = 50

        start_idx = max(0, peak_idx - half_window)
        end_idx = min(n_samples, peak_idx + half_window + 1)

        # 如果样本不足，使用全部可用样本
        if end_idx - start_idx < 3:
            return samples[peak_idx][0], samples[peak_idx][1]

        # 提取窗口内的数据
        window_samples = samples[start_idx:end_idx]
        n = len(window_samples)

        # 归一化 x 坐标，使峰值索引为 0
        # 这样窗口中心对应 x=0
        peak_local_idx = peak_idx - start_idx
        x_coords = []
        freqs = []
        weights = []

        for i, (x, y, freq) in enumerate(window_samples):
            # 归一化坐标，峰值位置为 0
            x_norm = float(i - peak_local_idx)
            x_coords.append(x_norm)
            freqs.append(freq)

            # 使用高斯权重，越靠近峰值权重越大
            # 标准差设为窗口的一半
            sigma = half_window / 2.0
            weight = math.exp(-(x_norm * x_norm) / (2.0 * sigma * sigma))
            weights.append(weight)

        # 使用加权最小二乘法拟合抛物线 y = a*x^2 + b*x + c
        # 权重为 w_i

        w_sum = sum(weights)
        w_sum_x = sum(w * x for w, x in zip(weights, x_coords))
        w_sum_x2 = sum(w * x * x for w, x in zip(weights, x_coords))
        w_sum_x3 = sum(w * x * x * x for w, x in zip(weights, x_coords))
        w_sum_x4 = sum(w * x * x * x * x for w, x in zip(weights, x_coords))
        w_sum_y = sum(w * y for w, y in zip(weights, freqs))
        w_sum_xy = sum(w * x * y for w, x, y in zip(weights, x_coords, freqs))
        w_sum_x2y = sum(w * x * x * y for w, x, y in zip(weights, x_coords, freqs))

        # 构建加权正规方程的系数矩阵行列式
        det = (w_sum_x4 * (w_sum_x2 * w_sum - w_sum_x * w_sum_x) -
               w_sum_x3 * (w_sum_x3 * w_sum - w_sum_x * w_sum_x2) +
               w_sum_x2 * (w_sum_x3 * w_sum_x - w_sum_x2 * w_sum_x2))

        if abs(det) < 1e-10:
            return samples[peak_idx][0], samples[peak_idx][1]

        # 计算 a
        det_a = (w_sum_x2y * (w_sum_x2 * w_sum - w_sum_x * w_sum_x) -
                 w_sum_xy * (w_sum_x3 * w_sum - w_sum_x * w_sum_x2) +
                 w_sum_y * (w_sum_x3 * w_sum_x - w_sum_x2 * w_sum_x2))
        a = det_a / det

        # 计算 b (Cramer: 第2列换成矩量 [w_sum_x2y, w_sum_xy, w_sum_y];
        # 旧展开把 wx3 与 wx2y 放错了位置, 窗口被截断时 b 是错值,
        # 顶点 -b/(2a) 随之报废 -- 对称窗口下两种展开碰巧相等)
        det_b = (w_sum_x4 * (w_sum_xy * w_sum - w_sum_x * w_sum_y) -
                 w_sum_x2y * (w_sum_x3 * w_sum - w_sum_x * w_sum_x2) +
                 w_sum_x2 * (w_sum_x3 * w_sum_y - w_sum_xy * w_sum_x2))
        b = det_b / det

        # 计算 c
        det_c = (w_sum_x4 * (w_sum_x2 * w_sum_y - w_sum_x * w_sum_xy) -
                 w_sum_x3 * (w_sum_x3 * w_sum_y - w_sum_x * w_sum_x2y) +
                 w_sum_x2 * (w_sum_x3 * w_sum_xy - w_sum_x2 * w_sum_x2y))
        c = det_c / det

        # 计算加权 R² 拟合度
        y_pred = [a * x * x + b * x + c for x in x_coords]
        w_mean = w_sum_y / w_sum
        ss_res = sum(w * (y - yp) ** 2 for w, y, yp in zip(weights, freqs, y_pred))
        ss_tot = sum(w * (y - w_mean) ** 2 for w, y in zip(weights, freqs))
        r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        self._last_fit_info = "R²=%.4f (n=%d) peak_type=%s" % (r_squared, n, detected_type)
        self._last_fit_r2 = r_squared

        # 检查拟合质量
        if abs(a) < 1e-10:
            # 抛物线过平 (无有效峰形), 否决拟合, 可信度按 0 计
            self._last_fit_r2 = 0.0
            return samples[peak_idx][0], samples[peak_idx][1]

        # peak模式：a<0为开口向下(峰值)，valley模式：a>0为开口向上(谷值)
        find_min = detected_type == 'valley'
        if find_min:
            if a < 0:
                # 开口方向与信号形态矛盾, 否决拟合, 可信度按 0 计
                self._last_fit_r2 = 0.0
                return samples[peak_idx][0], samples[peak_idx][1]
        else:
            if a > 0:
                self._last_fit_r2 = 0.0
                return samples[peak_idx][0], samples[peak_idx][1]

        x_peak = -b / (2.0 * a)

        # 顶点偏离最强采样点超过半个窗口说明拟合没找到真实峰
        # (可能峰值被截断/杂散峰混入)。旧实现把 x_peak 钳回窗口内,
        # 把失败拟合变成貌似合理的数字; 现在按拟合失败处理:
        # 退回原始采样点, 可信度归 0, 由 trusted 机制拦截
        max_offset = half_window * 0.5
        if abs(x_peak) > max_offset:
            self._last_fit_r2 = 0.0
            return samples[peak_idx][0], samples[peak_idx][1]

        # 转换回全局索引
        peak_global_idx = peak_local_idx + x_peak

        # 插值到实际位置 (x_peak 已在窗口内, 索引必然合法)
        peak_idx_floor = int(peak_global_idx)
        if peak_idx_floor < 0:
            peak_idx_floor = 0
        if peak_idx_floor >= n - 1:
            peak_idx_floor = n - 2
        t = peak_global_idx - peak_idx_floor

        # 从窗口样本中获取实际坐标
        p1 = window_samples[peak_idx_floor]
        p2 = window_samples[peak_idx_floor + 1]
        peak_x = p1[0] + t * (p2[0] - p1[0])
        peak_y = p1[1] + t * (p2[1] - p1[1])

        return peak_x, peak_y


def load_config_prefix(config):
    return ToolEddyCalibration(config)
