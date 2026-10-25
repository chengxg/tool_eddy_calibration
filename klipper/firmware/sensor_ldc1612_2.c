// Support for eddy current sensor data from ldc1612/ldc1614 chip
// Selectable-channel variant: reads DATA_CH<channel> (channel 0-3, channels
// 2-3 on LDC1614 only, switched at runtime via the SET_LDC1612_CHANNEL
// gcode command) instead of the original's fixed DATA_CH0.
//
// All symbols, command names and the error enumeration are suffixed _2 so
// this file compiles ALONGSIDE the original sensor_ldc1612.c (which keeps
// reading channel 0). The matching host module is ldc1612_2.py.
//
// Copyright (C) 2023 Alan.Ma <tech@biqu3d.com>
// Copyright (C) 2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "i2ccmds.h" // i2cdev_oid_lookup
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report
#include "trigger_analog.h" // trigger_analog_update

enum {
    LDC_PENDING = 1<<0, LDC_HAVE_INTB = 1<<1,
};

struct ldc1612_2 {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint8_t flags;
    uint8_t channel;
    uint8_t data_msb;
    struct sensor_bulk sb;
    struct gpio_in intb_pin;
    struct trigger_analog *ta;
};

static struct task_wake ldc1612_2_wake;

// Internal errors transmitted in sample reports (or trsync error)
enum {
    SE_SENSOR_ERROR, SE_I2C_STATUS, SE_I2C_DATA, SE_INVALID_DATA
};

DECL_ENUMERATION("ldc1612_2_error:", "SENSOR_REPORTS_ERROR", SE_SENSOR_ERROR);
DECL_ENUMERATION("ldc1612_2_error:", "I2C_STATUS_ERROR", SE_I2C_STATUS);
DECL_ENUMERATION("ldc1612_2_error:", "I2C_DATA_ERROR", SE_I2C_DATA);
DECL_ENUMERATION("ldc1612_2_error:", "INVALID_READ_DATA", SE_INVALID_DATA);

// Check if the intb line is "asserted"
static int
check_intb_asserted(struct ldc1612_2 *ld)
{
    return !gpio_in_read(ld->intb_pin);
}

// Query ldc1612 data
static uint_fast8_t
ldc1612_2_event(struct timer *timer)
{
    struct ldc1612_2 *ld = container_of(timer, struct ldc1612_2, timer);
    if (ld->flags & LDC_PENDING)
        ld->sb.possible_overflows++;
    if (!(ld->flags & LDC_HAVE_INTB) || check_intb_asserted(ld)) {
        ld->flags |= LDC_PENDING;
        sched_wake_task(&ldc1612_2_wake);
    }
    ld->timer.waketime += ld->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_ldc1612_2(uint32_t *args)
{
    struct ldc1612_2 *ld = oid_alloc(args[0], command_config_ldc1612_2
                                      , sizeof(*ld));
    ld->timer.func = ldc1612_2_event;
    ld->i2c = i2cdev_oid_lookup(args[1]);
    // Channel starts at 0; switched at runtime via
    // command_set_ldc1612_2_channel. DATA_CHx_MSB = 0x00 + 2*channel.
    ld->channel = 0;
    ld->data_msb = 0;
}
DECL_COMMAND(command_config_ldc1612_2, "config_ldc1612_2 oid=%c i2c_oid=%c");

void
command_config_ldc1612_2_with_intb(uint32_t *args)
{
    command_config_ldc1612_2(args);
    struct ldc1612_2 *ld = oid_lookup(args[0], command_config_ldc1612_2);
    ld->intb_pin = gpio_in_setup(args[2], 1);
    ld->flags = LDC_HAVE_INTB;
}
DECL_COMMAND(command_config_ldc1612_2_with_intb,
             "config_ldc1612_2_with_intb oid=%c i2c_oid=%c intb_pin=%c");

// Switch the active channel at runtime. Updates which DATA register and STATUS
// data-ready bit the bulk query path uses. The host reconfigures the chip
// (ACTIVE_CHAN + per-channel registers) around this call.
void
command_set_ldc1612_2_channel(uint32_t *args)
{
    struct ldc1612_2 *ld = oid_lookup(args[0], command_config_ldc1612_2);
    ld->channel = args[1];
    ld->data_msb = 2 * ld->channel;
}
DECL_COMMAND(command_set_ldc1612_2_channel,
             "set_ldc1612_2_channel oid=%c channel=%c");

void
ldc1612_2_attach_trigger_analog(uint32_t *args) {
    struct ldc1612_2 *ld = oid_lookup(args[0], command_config_ldc1612_2);
    ld->ta = trigger_analog_oid_lookup(args[1]);
}
#if CONFIG_WANT_TRIGGER_ANALOG
DECL_COMMAND(ldc1612_2_attach_trigger_analog,
    "ldc1612_2_attach_trigger_analog oid=%c trigger_analog_oid=%c");
#endif

#define DATA_ERROR_AMPLITUDE (1L << 28)

// Chip registers. The per-channel DATA register address is set at config time
// and switched at runtime (ld->data_msb = 2*channel); the LSB register follows
// at ld->data_msb+1.
#define REG_STATUS    0x18

// Read a register on the ldc1612
static int
read_reg(struct ldc1612_2 *ld, uint8_t reg, uint8_t *res)
{
    return i2c_dev_read(ld->i2c, sizeof(reg), &reg, 2, res);
}

// Read the status register on the ldc1612
static int
read_reg_status(struct ldc1612_2 *ld, uint16_t *status)
{
    uint8_t data_status[2];
    int ret = read_reg(ld, REG_STATUS, data_status);
    *status = (data_status[0] << 8) | data_status[1];
    return ret;
}

// Per-channel "data ready" bit in STATUS (0x18): UNREADCONV_n = bit (3 - n),
// i.e. CH0->bit3, CH1->bit2, CH2->bit1, CH3->bit0. Derived from the channel-0
// code (UNREADCONV0 = bit3) and the former channel-2 mask (which included
// bit1). The bit is cleared by reading the channel's DATA register, so it
// reliably indicates fresh data for the configured channel (single-channel
// mode, ACTIVE_CHAN = channel).
#define STATUS_DATA_READY(ch) (1 << (3 - (ch)))
#define BYTES_PER_SAMPLE 4

static void
report_sample_error(struct ldc1612_2 *ld, int error_code)
{
    trigger_analog_note_error(ld->ta, error_code);

    uint8_t *d = &ld->sb.data[ld->sb.data_count];
    d[0] = 0xff;
    d[1] = 0xff;
    d[2] = 0;
    d[3] = error_code;
}

// Query ldc1612 data
static void
ldc1612_2_query(struct ldc1612_2 *ld, uint8_t oid)
{
    // Check if data available (and clear INTB line)
    uint16_t status;
    int ret = read_reg_status(ld, &status);
    irq_disable();
    ld->flags &= ~LDC_PENDING;
    irq_enable();
    if (ret) {
        report_sample_error(ld, SE_I2C_STATUS);
        goto out;
    }
    if (!(status & STATUS_DATA_READY(ld->channel)))
        // No data available
        return;

    // Read channel frequency
    uint8_t *d = &ld->sb.data[ld->sb.data_count];
    ret |= read_reg(ld, ld->data_msb, &d[0]);
    ret |= read_reg(ld, ld->data_msb + 1, &d[2]);

    if (ret) {
        report_sample_error(ld, SE_I2C_DATA);
        goto out;
    }
    if (d[0] == 0xff && d[1] == 0xff) {
        // Invalid data from sensor (conflict with internal error indicator)
        report_sample_error(ld, SE_INVALID_DATA);
        goto out;
    }

    // Check for homing trigger
    uint32_t raw_data = (((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16)
                         | ((uint32_t)d[2] << 8) | ((uint32_t)d[3]));
    if (raw_data & 0xe0000000)
        trigger_analog_note_error(ld->ta, SE_SENSOR_ERROR);
    else
        trigger_analog_update(ld->ta, raw_data & 0x0fffffff);

out:
    ld->sb.data_count += BYTES_PER_SAMPLE;
    // Flush local buffer if needed
    if (ld->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(ld->sb.data))
        sensor_bulk_report(&ld->sb, oid);
}

void
command_query_ldc1612_2(uint32_t *args)
{
    struct ldc1612_2 *ld = oid_lookup(args[0], command_config_ldc1612_2);

    sched_del_timer(&ld->timer);
    ld->flags &= ~LDC_PENDING;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    ld->rest_ticks = args[1];
    sensor_bulk_reset(&ld->sb);
    irq_disable();
    ld->timer.waketime = timer_read_time() + ld->rest_ticks;
    sched_add_timer(&ld->timer);
    irq_enable();
}
DECL_COMMAND(command_query_ldc1612_2, "query_ldc1612_2 oid=%c rest_ticks=%u");

void
command_query_status_ldc1612_2(uint32_t *args)
{
    struct ldc1612_2 *ld = oid_lookup(args[0], command_config_ldc1612_2);

    if (ld->flags & LDC_HAVE_INTB) {
        // Check if a sample is pending in the chip via the intb line
        irq_disable();
        uint32_t time = timer_read_time();
        int p = check_intb_asserted(ld);
        irq_enable();
        sensor_bulk_status(&ld->sb, args[0], time, 0, p ? BYTES_PER_SAMPLE : 0);
        return;
    }

    // Query sensor to see if a sample is pending
    uint16_t status;
    uint32_t time1 = timer_read_time();
    int ret = read_reg_status(ld, &status);
    uint32_t time2 = timer_read_time();

    if (ret)
        // Query error - don't send response - host will retry
        return;
    uint32_t fifo = status & STATUS_DATA_READY(ld->channel) ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&ld->sb, args[0], time1, time2-time1, fifo);
}
DECL_COMMAND(command_query_status_ldc1612_2, "query_status_ldc1612_2 oid=%c");

void
ldc1612_2_task(void)
{
    if (!sched_check_wake(&ldc1612_2_wake))
        return;
    uint8_t oid;
    struct ldc1612_2 *ld;
    foreach_oid(oid, ld, command_config_ldc1612_2) {
        uint_fast8_t flags = ld->flags;
        if (!(flags & LDC_PENDING))
            continue;
        ldc1612_2_query(ld, oid);
    }
}
DECL_TASK(ldc1612_2_task);
