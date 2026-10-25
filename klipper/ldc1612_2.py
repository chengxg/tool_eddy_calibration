# Support for reading frequency samples from ldc1612
#
# Copyright (C) 2020-2026  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
from . import bus, bulk_sensor

MIN_MSG_TIME = 0.100

BATCH_UPDATES = 0.100

LDC1612_ADDR = 0x2a

DEFAULT_LDC1612_FREQ = 12000000
SETTLETIME = 0.0002
DRIVECUR = 15
DEGLITCH = 0x05 # 10 Mhz

LDC1612_MANUF_ID = 0x5449
LDC1612_DEV_ID = 0x3055

# LDC1612 per-channel registers = base + channel (RCOUNT0=0x08,
# DRIVE_CURRENT0=0x1e, DATA_CH0_MSB=0x00+2*channel). This variant reads a
# single selectable channel (0 or 1), switchable at runtime via the
# SET_LDC1612_CHANNEL gcode command; the per-instance addresses are computed
# in LDC1612.__init__ and updated on switch.
REG_RCOUNT_BASE = 0x08
REG_OFFSET_BASE = 0x0c
REG_SETTLECOUNT_BASE = 0x10
REG_CLOCK_DIVIDERS_BASE = 0x14
REG_DRIVE_CURRENT_BASE = 0x1e
REG_ERROR_CONFIG = 0x19
REG_CONFIG = 0x1a
REG_MUX_CONFIG = 0x1b
REG_MANUFACTURER_ID = 0x7e
REG_DEVICE_ID = 0x7f

# ---- Register bit-field reference (verified against the LDC1612 datasheet) ----
#
# CONFIG (0x1a):
#   [15:14] ACTIVE_CHAN       00=CH0, 01=CH1, 10=CH2, 11=CH3. Selects the
#                             continuous-conversion channel when AUTOSCAN_EN=0.
#                             Per-instance value = (channel << 14) = active_chan.
#   [13]    SLEEP_MODE_EN      0=active, 1=sleep (reset=1). Code leaves it 0.
#   [12]    RP_OVERRIDE_EN     1=use the programmed CHx_INIT_IDRIVE (DRIVE_CURRENT
#                             register) instead of the auto-computed value.
#   [11]    SENSOR_ACTIVATE_SEL 0=full-current activation, 1=low-power. Code=0.
#   [10]    AUTO_AMP_DIS       1=disable auto-amplitude correction, use INIT_IDRIVE
#                             (high precision). 0=auto (used during calibration).
#   [9]     REF_CLK_SRC        0=internal oscillator, 1=external CLKIN. Code=1
#                             (clock_freq is the external reference frequency).
#   [8]     RESERVED           must be 0.
#   [7]     INTB_DIS           0=INTB asserts on STATUS update.
#   [6]     HIGH_CURRENT_DRV   0=normal current (<=1.5mA).
#   [5:0]   RESERVED           must be 0b000001 (bit0 set); code writes 0x001.
#
# MUX_CONFIG (0x1b):
#   [15]    AUTOSCAN_EN        0=single channel (ACTIVE_CHAN selects), 1=autoscan.
#                             Code=0, so ACTIVE_CHAN drives channel selection.
#   [14:13] RR_SEQUENCE        autoscan channel order (unused in single-channel).
#   [12:3]  RESERVED           keep reset values (field reset 0x0040 -> bit 9 set).
#                             Code = 0x0208 (= default 0x020F with DEGLITCH
#                             cleared) | DEGLITCH: it preserves the power-on
#                             reserved bits and only sets DEGLITCH below.
#   [2:0]   DEGLITCH           input filter: 0x1=1MHz, 0x4=3.3MHz, 0x5=10MHz,
#                             0x7=33MHz. Code=0x05 (10MHz) via the DEGLITCH const.
#
# STATUS (0x18) - read-only; the MCU driver gates reads on the per-channel bit:
#   [15:14] ERR_CHAN           channel that flagged an error.
#   [13:8]  ERR_UR/OR/WD/AHE/ALE/ZC  conversion error flags (under/over-range,
#                             watchdog, amplitude high/low, zero-count).
#   [6]     DRDY               new conversion ready (any channel).
#   [3]     UNREADCONV0        CH0 has an unread conversion.
#   [2]     UNREADCONV1        CH1 has an unread conversion.
#   [1]     UNREADCONV2        CH2 (LDC1614 only).
#   [0]     UNREADCONV3        CH3 (LDC1614 only).
#   => UNREADCONV_n = bit (3 - n); cleared by reading DATA_CHx. The MCU bulk
#      query gates on this bit (sensor_ldc1612_2.c STATUS_DATA_READY(ch)).
#
# DRIVE_CURRENT (0x1e + channel):
#   [15:11] CHx_INIT_IDRIVE    initial drive current the code writes
#                             (get_drive_current() << 11); used when
#                             RP_OVERRIDE_EN=1 / AUTO_AMP_DIS=1.
#   [10:6]  CHx_IDRIVE         drive current used in the last conversion; read
#                             back during calibration ((reg >> 6) & 0x1f).
#
# ERROR_CONFIG (0x19): (0x1f << 11) | 1 enables conversion-error reporting (the
#   STATUS error flags above) to assert INTB.

# Tool for determining appropriate DRIVE_CURRENT register
class DriveCurrentCalibrate:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.sensor = sensor
        # 按通道的驱动电流。通道 0 -> 'reg_drive_current' (无后缀),
        # 通道 N (1-3) -> 'reg_drive_currentN'; 未配置则回退到通道 0 的值。
        dc0 = config.getint("reg_drive_current", DRIVECUR, minval=0, maxval=31)
        self.drive_cur = {0: dc0}
        for ch in (1, 2, 3):
            self.drive_cur[ch] = config.getint("reg_drive_current%d" % ch, dc0,
                                              minval=0, maxval=31)
        self.name = config.get_name()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("LDC_CALIBRATE_DRIVE_CURRENT",
                                   "CHIP", self.name.split()[-1],
                                   self.cmd_LDC_CALIBRATE,
                                   desc=self.cmd_LDC_CALIBRATE_help)
    def get_drive_current(self, channel=None):
        # 返回指定通道 (默认活动通道) 的驱动电流; 未配置的通道回退到通道 0。
        if channel is None:
            channel = self.sensor.channel
        return self.drive_cur.get(channel, self.drive_cur[0])
    cmd_LDC_CALIBRATE_help = "Calibrate LDC1612 DRIVE_CURRENT register"
    def cmd_LDC_CALIBRATE(self, gcmd):
        is_in_progress = True
        def handle_batch(msg):
            return is_in_progress
        self.sensor.add_client(handle_batch)
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(0.100)
        toolhead.wait_moves()
        old_config = self.sensor.read_reg(REG_CONFIG)
        # Select this instance's channel so the chip auto-calibrates that
        # channel's drive current and stores it in DRIVE_CURRENT<channel>.
        # RP_OVERRIDE_EN(12)=0 + AUTO_AMP_DIS(10)=0 enable auto-amplitude
        # correction; REF_CLK_SRC(9)=1 (external clock); ACTIVE_CHAN=channel.
        self.sensor.set_reg(REG_CONFIG, 0x001 | (1<<9) | self.sensor.active_chan)
        toolhead.wait_moves()
        toolhead.dwell(0.100)
        toolhead.wait_moves()
        reg_drive_current = self.sensor.read_reg(self.sensor.reg_drive_current)
        self.sensor.set_reg(REG_CONFIG, old_config)
        is_in_progress = False
        # Report found value to user
        drive_cur = (reg_drive_current >> 6) & 0x1f  # CHx_IDRIVE [10:6] readback
        ch = self.sensor.channel
        key = "reg_drive_current" if ch == 0 else "reg_drive_current%d" % ch
        gcmd.respond_info(
            "%s: %s (channel %d): %d\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, key, ch, drive_cur))
        # 立即更新内存值, 使本次会话后续 _write_chip_config 即用新值 (无需重启)
        self.drive_cur[ch] = drive_cur
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, key, "%d" % (drive_cur,))

# Interface class to LDC1612 mcu support
class LDC1612:
    def __init__(self, config, calibration=None):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.calibration = calibration
        # Active channel (0-3), switchable at runtime via the
        # SET_LDC1612_CHANNEL gcode command (not pinned in config). Per-channel
        # register addresses and the ACTIVE_CHAN config field are derived from
        # this; defaults to 0 until switched. Channels 2-3 exist only on LDC1614.
        self.channel = 0
        self.reg_rcount = REG_RCOUNT_BASE + self.channel
        self.reg_offset = REG_OFFSET_BASE + self.channel
        self.reg_settlecount = REG_SETTLECOUNT_BASE + self.channel
        self.reg_clock_dividers = REG_CLOCK_DIVIDERS_BASE + self.channel
        self.reg_drive_current = REG_DRIVE_CURRENT_BASE + self.channel
        self.active_chan = self.channel << 14
        self.dccal = DriveCurrentCalibrate(config, self)
        # Register runtime channel-switch command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_LDC1612_CHANNEL", "CHIP", self.name,
                                   self.cmd_SET_LDC1612_CHANNEL,
                                   desc=self.cmd_SET_LDC1612_CHANNEL_help)
        gcode.register_mux_command("SET_LDC1612_DATA_RATE", "CHIP", self.name,
                                   self.cmd_SET_LDC1612_DATA_RATE,
                                   desc=self.cmd_SET_LDC1612_DATA_RATE_help)
        # Setup mcu sensor_ldc1612 bulk query code
        self.i2c = bus.MCU_I2C_from_config(config,
                                           default_addr=LDC1612_ADDR,
                                           default_speed=400000)
        self.mcu = mcu = self.i2c.get_mcu()
        self._sensor_errors = {}
        self.oid = oid = mcu.create_oid()
        self.query_ldc1612_cmd = None
        self.clock_freq = config.getint("frequency", DEFAULT_LDC1612_FREQ,
                                        2000000, 40000000)
        # 采样频率 (Hz), 经 RCOUNT=f_clk/(16*rate) 换算为转换时间。约束:
        # RCOUNT 不得溢出 16-bit (下限, 随 frequency 而定, 40MHz 时约 38Hz),
        # 且不得小于 SETTLECOUNT (转换时间 >= 稳定时间, 上限约 5kHz)。
        # 可配置也可运行时 SET_LDC1612_DATA_RATE 修改。
        self.data_rate = config.getint("data_rate", 400, minval=1)
        msg = self._validate_data_rate(self.data_rate)
        if msg is not None:
            raise config.error("ldc1612 %s: %s" % (self.name, msg))
        self._measuring = False
        # Determine sensor divider (want 4*max_hz < clock_ref)
        max_hz = config.getfloat("max_sensor_hz", 5000000., 3000000., 20000000.)
        self.sensor_div = int(math.ceil(4. * max_hz / self.clock_freq))
        self.freq_conv = float(self.clock_freq * self.sensor_div) / (1<<28)
        if self.calibration is not None:
            cal_freqs, cal_zpos = self.calibration.get_calibration()
            if cal_freqs and max(cal_freqs) > max_hz:
                pconfig = self.printer.lookup_object("configfile")
                pconfig.runtime_warning(
                    "ldc1612 %s: Should set 'max_sensor_hz' to at least %d"
                    % (self.name, math.ceil(max(cal_freqs))))
        # Configure intb and mcu object
        if config.get('intb_pin', None) is not None:
            ppins = config.get_printer().lookup_object("pins")
            pin_params = ppins.lookup_pin(config.get('intb_pin'))
            if pin_params['chip'] != mcu:
                raise config.error("ldc1612 intb_pin must be on same mcu")
            mcu.add_config_cmd(
                "config_ldc1612_2_with_intb oid=%d i2c_oid=%d intb_pin=%s"
                % (oid, self.i2c.get_oid(), pin_params['pin']))
        else:
            mcu.add_config_cmd("config_ldc1612_2 oid=%d i2c_oid=%d"
                               % (oid, self.i2c.get_oid()))
        mcu.add_config_cmd("query_ldc1612_2 oid=%d rest_ticks=0"
                           % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        # Bulk sample message reading
        chip_smooth = self.data_rate * BATCH_UPDATES * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, ">I")
        self.last_error_count = 0
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._finish_measurements, BATCH_UPDATES)
        hdr = ('time', 'frequency', 'z')
        self.batch_bulk.add_mux_endpoint("ldc1612_2/dump_ldc1612_2", "sensor",
                                         self.name, {'header': hdr})
    def setup_trigger_analog(self, trigger_analog_oid):
        self.mcu.add_config_cmd(
            "ldc1612_2_attach_trigger_analog oid=%d trigger_analog_oid=%d"
            % (self.oid, trigger_analog_oid), is_init=True)
    def _build_config(self):
        cmdqueue = self.i2c.get_command_queue()
        self.query_ldc1612_cmd = self.mcu.lookup_command(
            "query_ldc1612_2 oid=%c rest_ticks=%u", cq=cmdqueue)
        self.set_channel_cmd = self.mcu.lookup_command(
            "set_ldc1612_2_channel oid=%c channel=%c", cq=cmdqueue)
        self.ffreader.setup_query_command("query_status_ldc1612_2 oid=%c",
                                          oid=self.oid, cq=cmdqueue)
        errors = self.mcu.get_enumerations().get("ldc1612_2_error:", {})
        self._sensor_errors = {v: k for k, v in errors.items()}
    cmd_SET_LDC1612_CHANNEL_help = (
        "Set the active LDC1612 channel (0-3) at runtime. "
        "Channels 2-3 exist only on the LDC1614 (4-channel) chip.")
    def set_channel(self, channel):
        # Update host-side channel state (register addresses + ACTIVE_CHAN),
        # tell the MCU which DATA register / STATUS bit to use, then
        # reconfigure the chip for the new channel. No-op if already on
        # `channel`. Exposed so host modules (e.g. tool_eddy_calibration)
        # driving an ldc1612_2 instance can switch channels directly.
        # channel 0-3: register math (REG_BASE+ch, ch<<14) and the MCU
        # STATUS_DATA_READY(ch) bit all cover the full 4-channel range.
        if channel == self.channel:
            return
        self.channel = channel
        self.reg_rcount = REG_RCOUNT_BASE + self.channel
        self.reg_offset = REG_OFFSET_BASE + self.channel
        self.reg_settlecount = REG_SETTLECOUNT_BASE + self.channel
        self.reg_clock_dividers = REG_CLOCK_DIVIDERS_BASE + self.channel
        self.reg_drive_current = REG_DRIVE_CURRENT_BASE + self.channel
        self.active_chan = self.channel << 14
        self.set_channel_cmd.send([self.oid, self.channel])
        self._write_chip_config()
    def cmd_SET_LDC1612_CHANNEL(self, gcmd):
        channel = gcmd.get_int("CHANNEL", minval=0, maxval=3)
        if channel == self.channel:
            gcmd.respond_info("%s: channel already %d" % (self.name, channel))
            return
        self.set_channel(channel)
        gcmd.respond_info("%s: channel set to %d" % (self.name, channel))
    def _validate_data_rate(self, data_rate):
        """校验采样频率是否落在 RCOUNT 约束内, 返回错误消息或 None"""
        rcount = int(self.clock_freq / (16. * data_rate) + 0.5)
        settlecount = int(SETTLETIME * self.clock_freq / 16. + .5)
        if rcount > 0xffff:
            return ("data_rate %d too low for frequency %d "
                    "(RCOUNT %d overflows 16-bit; minimum rate is %.0f)"
                    % (data_rate, self.clock_freq, rcount,
                       math.ceil(self.clock_freq / (16. * 0xffff))))
        if rcount < settlecount:
            return ("data_rate %d too high for frequency %d "
                    "(RCOUNT %d below SETTLECOUNT %d; maximum rate is %.0f)"
                    % (data_rate, self.clock_freq, rcount, settlecount,
                       math.floor(self.clock_freq
                               / (16. * settlecount))))
        return None
    cmd_SET_LDC1612_DATA_RATE_help = (
        "Set the LDC1612 sample rate (Hz) at runtime. RCOUNT constraints "
        "are validated; applies to the active channel and subsequent "
        "measurements. RATE=400 restores the configured default.")
    def set_data_rate(self, data_rate):
        # 运行时修改采样频率。校验后更新内存值; 测量进行中则同时刷新
        # RCOUNT 寄存器与 MCU 轮询周期, 空闲时只更新内存值, 下次测量
        # 启动时 _write_chip_config 统一写入。
        msg = self._validate_data_rate(data_rate)
        if msg is not None:
            raise self.printer.command_error(msg)
        self.data_rate = data_rate
        if not self._measuring:
            return
        if not self.mcu.is_fileoutput():
            self.set_reg(self.reg_rcount,
                         int(self.clock_freq / (16. * data_rate) + 0.5))
        if self.query_ldc1612_cmd is not None:
            rest_ticks = self.mcu.seconds_to_clock(0.5 / data_rate)
            self.query_ldc1612_cmd.send([self.oid, rest_ticks])
    def cmd_SET_LDC1612_DATA_RATE(self, gcmd):
        rate = gcmd.get_int("RATE", minval=1)
        self.set_data_rate(rate)
        gcmd.respond_info("%s: data_rate set to %d" % (self.name, rate))
    def _write_chip_config(self):
        # Write per-channel + shared registers and ACTIVE_CHAN for the active
        # channel. Called at measurement start and on runtime channel switch.
        rcount = self.clock_freq / (16. * self.data_rate)
        self.set_reg(self.reg_rcount, int(rcount + 0.5))
        self.set_reg(self.reg_offset, 0)
        self.set_reg(self.reg_settlecount, int(SETTLETIME*self.clock_freq/16. + .5))
        self.set_reg(self.reg_clock_dividers, (self.sensor_div << 12) | 1)
        self.set_reg(REG_ERROR_CONFIG, (0x1c << 11) | 1)
        self.set_reg(REG_MUX_CONFIG, 0x0208 | DEGLITCH)  # AUTOSCAN_EN=0 (single ch), DEGLITCH=10MHz
        # CONFIG: bit0 reserved(must-set), RP_OVERRIDE_EN(12)=use INIT_IDRIVE,
        # AUTO_AMP_DIS(10)=fixed drive current, REF_CLK_SRC(9)=external clock,
        # ACTIVE_CHAN(15:14)=channel; SLEEP_MODE_EN(13)=0 (awake).
        self.set_reg(REG_CONFIG, 0x001 | (1<<12) | (1<<10) | (1<<9) | self.active_chan)
        # DRIVE_CURRENT[15:11] = CHx_INIT_IDRIVE (used because RP_OVERRIDE_EN=1)
        self.set_reg(self.reg_drive_current, self.dccal.get_drive_current() << 11)
    def get_mcu(self):
        return self.i2c.get_mcu()
    def get_samples_per_second(self):
        return self.data_rate
    def read_reg(self, reg):
        if self.mcu.is_fileoutput():
            return 0
        params = self.i2c.i2c_read([reg], 2)
        response = bytearray(params['response'])
        return (response[0] << 8) | response[1]
    def set_reg(self, reg, val, minclock=0):
        self.i2c.i2c_write([reg, (val >> 8) & 0xff, val & 0xff],
                           minclock=minclock)
    def add_client(self, cb):
        self.batch_bulk.add_client(cb)
    def lookup_sensor_error(self, error):
        return self._sensor_errors.get(error, "Unknown ldc1612 error")
    def convert_raw_to_frequency(self, raw_value):
        return raw_value * self.freq_conv
    def convert_frequency_to_raw(self, freq):
        return int(freq / self.freq_conv + 0.5)
    # Measurement decoding
    def _convert_samples(self, samples):
        freq_conv = self.freq_conv
        count = 0
        errors = {}
        def log_once(msg):
            if not errors.get(msg, 0):
                errors[msg] = 0
            errors[msg] += 1
        for ptime, val in samples:
            mv = val & 0x0fffffff
            if val > 0x03ffffff or val == 0x0:
                self.last_error_count += 1
                if (val >> 16 & 0xffff) == 0xffff:
                    # Encoded error from sensor_ldc1612_2.c
                    log_once(self.lookup_sensor_error(val & 0xffff))
                    continue
                error_bits = (val >> 28) & 0x0f
                if error_bits & 0x8 or mv == 0x0000000:
                    log_once("Frequency under valid range")
                if error_bits & 0x4 or mv > 0x3ffffff:
                    type = "hard" if error_bits & 0x4 else "soft"
                    log_once("Frequency over valid %s range" % (type))
                if error_bits & 0x2:
                    log_once("Conversion Watchdog timeout")
                if error_bits & 0x1:
                    log_once("Amplitude Low/High warning")
            samples[count] = (round(ptime, 6), round(freq_conv * mv, 3), 999.9)
            count += 1
        del samples[count:]
        for msg in errors:
            logging.error("%s: %s (%d)" % (self.name, msg, errors[msg]))
    # Start, stop, and process message batches
    def _start_measurements(self):
        self._measuring = True
        # In case of miswiring, testing LDC1612 device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        manuf_id = self.read_reg(REG_MANUFACTURER_ID)
        dev_id = self.read_reg(REG_DEVICE_ID)
        if ((manuf_id != LDC1612_MANUF_ID or dev_id != LDC1612_DEV_ID)
            and not self.mcu.is_fileoutput()):
            raise self.printer.command_error(
                "Invalid ldc1612 id (got %x,%x vs %x,%x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty ldc1612 chip."
                % (manuf_id, dev_id, LDC1612_MANUF_ID, LDC1612_DEV_ID))
        # Configure chip for the active channel
        self._write_chip_config()
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(0.5 / self.data_rate)
        self.query_ldc1612_cmd.send([self.oid, rest_ticks])
        logging.info("LDC1612 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.ffreader.note_start()
        self.last_error_count = 0
    def _finish_measurements(self):
        self._measuring = False
        # Halt bulk reading
        self.query_ldc1612_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("LDC1612 finished '%s' measurements", self.name)
    def _process_batch(self, eventtime):
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        if not samples:
            return {}
        if self.calibration is not None:
            self.calibration.apply_calibration(samples)
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.ffreader.get_last_overflows()}
