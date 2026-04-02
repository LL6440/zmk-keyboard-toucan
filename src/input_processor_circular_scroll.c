/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll input processor — REL-event based, sliding-window detection.
 *
 * Uses a ring-buffer of recent velocity vectors.  Circular motion is detected
 * by comparing the CURRENT vector against the vector COMPARE_OFFSET samples
 * back.  At ~140 Hz this comparison spans ~55 ms of motion, giving a reliable
 * angular-velocity signal even for slow, smooth circles.
 */

#define DT_DRV_COMPAT zmk_input_processor_circular_scroll

#include <drivers/input_processor.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define RAD_TO_DEG (180.0f / (float)M_PI)
#define IABS(x)    ((x) < 0 ? -(x) : (x))

/* ── tuneable constants ─────────────────────────────────────────────────── */

/* Ring-buffer size for velocity history.  Must be > COMPARE_OFFSET.       */
#define HIST_SIZE       16

/* Compare current velocity against the velocity this many samples back.    *
 * At ~140 Hz, COMPARE_OFFSET=8 ≈ 55 ms → typically 10-25° of rotation    *
 * for a comfortable circle pace, giving a clear cross-product signal.      */
#define COMPARE_OFFSET   8

/* Minimum per-report speed (|dx|+|dy|) to participate in detection.       */
#define MIN_SPEED        6

/* |sin(angle)| between v[now] and v[now-COMPARE_OFFSET] must exceed this. */
/* sin(15°) ≈ 0.26 — well above noise for the window above.               */
#define CIRC_SIN_THRESHOLD   0.22f

/* Consecutive circular samples (each spanning COMPARE_OFFSET+1 reports)   *
 * required to enter scroll mode.                                           */
#define CIRC_ENTER       3

/* Consecutive non-circular samples to exit scroll mode.                   */
#define CIRC_EXIT        5

/* Degrees of accumulated arc per scroll tick (overridden by DTS).         */
#define DEFAULT_SCROLL_DEG  12

/* ── types ──────────────────────────────────────────────────────────────── */

enum circ_axis { AXIS_UNKNOWN = 0, AXIS_VERTICAL, AXIS_HORIZONTAL };

struct circular_scroll_config {
    /* Kept for DTS compat — values no longer actively used except          *
     * scroll_angle_deg (emission rate) and invert_* flags.                 */
    uint16_t pressure_threshold;
    uint16_t x_max;
    uint16_t y_max;
    uint16_t inner_ring_pct;
    uint16_t sector_half_angle_deg;
    uint16_t activation_angle_deg;
    uint16_t scroll_angle_deg;
    bool invert_vertical;
    bool invert_horizontal;
};

struct circular_scroll_data {
    /* Velocity ring-buffer */
    int32_t vx[HIST_SIZE];
    int32_t vy[HIST_SIZE];
    int     head;        /* index of next slot to write                     */
    int     filled;      /* number of valid entries (capped at HIST_SIZE)   */

    /* Current-report accumulator */
    int32_t curr_dx;
    bool    have_x;

    /* Touch + scroll state */
    bool           touch_active;
    bool           scroll_mode;
    enum circ_axis scroll_axis;
    float          angle_accum;

    /* Detection counters */
    int   enter_count;
    int   exit_count;
    float circ_sign;

    /* Axis selection (dominant motion during warm-up) */
    float abs_x_sum;
    float abs_y_sum;
};

/* ── helpers ─────────────────────────────────────────────────────────────── */

static void circ_reset(struct circular_scroll_data *d) {
    d->head        = 0;
    d->filled      = 0;
    d->curr_dx     = 0;
    d->have_x      = false;
    d->touch_active = false;
    d->scroll_mode = false;
    d->scroll_axis = AXIS_UNKNOWN;
    d->angle_accum = 0.0f;
    d->enter_count = 0;
    d->exit_count  = 0;
    d->circ_sign   = 0.0f;
    d->abs_x_sum   = 0.0f;
    d->abs_y_sum   = 0.0f;
}

/* Push a new velocity sample into the ring buffer. */
static void push_vel(struct circular_scroll_data *d, int32_t dx, int32_t dy)
{
    d->vx[d->head] = dx;
    d->vy[d->head] = dy;
    d->head = (d->head + 1) % HIST_SIZE;
    if (d->filled < HIST_SIZE) d->filled++;
}

/* Retrieve sample at (head - 1 - offset), i.e. offset samples before now. */
static void get_past(const struct circular_scroll_data *d, int offset,
                     int32_t *ox, int32_t *oy)
{
    int idx = (d->head - 1 - offset + HIST_SIZE * 2) % HIST_SIZE;
    *ox = d->vx[idx];
    *oy = d->vy[idx];
}

/* ── per-report processing ───────────────────────────────────────────────── */

static int32_t process_vector(struct circular_scroll_data *d,
                              const struct circular_scroll_config *cfg,
                              int32_t dx, int32_t dy)
{
    int speed = IABS(dx) + IABS(dy);

    /* Always accumulate axis-selection data while not yet in scroll mode.  */
    if (!d->scroll_mode) {
        d->abs_x_sum += IABS(dx);
        d->abs_y_sum += IABS(dy);
    }

    push_vel(d, dx, dy);

    /* Need enough history to compare against. */
    if (d->filled <= COMPARE_OFFSET || speed < MIN_SPEED) {
        return 0;
    }

    /* Compare current velocity against COMPARE_OFFSET samples back. */
    int32_t px, py;
    get_past(d, COMPARE_OFFSET, &px, &py);

    float cross = (float)px * dy - (float)py * dx;
    float m1    = sqrtf((float)px*px + (float)py*py);
    float m2    = sqrtf((float)dx*dx + (float)dy*dy);

    if (m1 < 1.0f || m2 < 1.0f) { return 0; }

    float norm_cross = cross / (m1 * m2);
    /* Also compute swept angle (signed, radians→degrees) for accumulation. */
    float dot        = (float)px*dx + (float)py*dy;
    float swept_deg  = atan2f(cross, dot + 1e-6f) * RAD_TO_DEG;

    bool is_circular = (IABS((int)(norm_cross * 1000)) >
                        (int)(CIRC_SIN_THRESHOLD * 1000));

    if (!d->scroll_mode) {
        if (is_circular) {
            float new_sign = (norm_cross > 0.0f) ? 1.0f : -1.0f;
            if (d->enter_count == 0 || d->circ_sign == new_sign) {
                d->enter_count++;
                d->circ_sign = new_sign;
            } else {
                d->enter_count = 1;
                d->circ_sign   = new_sign;
            }
            if (d->enter_count >= CIRC_ENTER) {
                d->scroll_mode = true;
                d->exit_count  = 0;
                d->angle_accum = 0.0f;
                d->scroll_axis = (d->abs_y_sum >= d->abs_x_sum)
                                 ? AXIS_VERTICAL : AXIS_HORIZONTAL;
            }
        } else {
            d->enter_count = 0;
        }
        return 0;
    }

    /* ── Scroll mode ── */
    if (!is_circular) {
        /* Keep the current mode latched until finger lift. */
        d->exit_count++;
        return 0;
    }
    d->exit_count = 0;

    d->angle_accum += swept_deg;

    int32_t scroll_deg = (cfg->scroll_angle_deg > 0)
                         ? cfg->scroll_angle_deg : DEFAULT_SCROLL_DEG;

    if (IABS((int)d->angle_accum) < scroll_deg) { return 0; }

    int32_t ticks  = (int32_t)(d->angle_accum / (float)scroll_deg);
    d->angle_accum -= (float)ticks * (float)scroll_deg;
    return ticks;
}

/* ── main event handler ──────────────────────────────────────────────────── */

static int circular_scroll_handle_event(const struct device *dev,
                                        struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state)
{
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct circular_scroll_config *cfg = dev->config;
    struct circular_scroll_data         *data = dev->data;

    if (event->type == INPUT_EV_ABS) {
        if (event->code == INPUT_ABS_Z) {
            if (event->value < cfg->pressure_threshold) {
                circ_reset(data);
            } else if (!data->touch_active) {
                data->touch_active = true;
                data->curr_dx = 0;
                data->have_x = false;
                data->angle_accum = 0.0f;
                data->enter_count = 0;
                data->exit_count = 0;
                data->circ_sign = 0.0f;
                data->abs_x_sum = 0.0f;
                data->abs_y_sum = 0.0f;
            }
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->curr_dx = event->value;
        data->have_x  = true;
        /* Once scroll mode is entered, keep the cursor frozen until lift. */
        return data->scroll_mode ? ZMK_INPUT_PROC_STOP : ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        int32_t dx = data->have_x ? data->curr_dx : 0;
        int32_t dy = event->value;
        data->have_x  = false;
        data->curr_dx = 0;

        int32_t ticks = process_vector(data, cfg, dx, dy);

        if (!data->scroll_mode) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (ticks == 0) {
            /* Keep swallowing pointer motion while finger remains down. */
            return ZMK_INPUT_PROC_STOP;
        }

        /* Convert the REL_Y event into a scroll event. */
        if (data->scroll_axis == AXIS_VERTICAL) {
            if (cfg->invert_vertical) { ticks = -ticks; }
            event->code  = INPUT_REL_WHEEL;
            event->value = ticks;
        } else {
            if (cfg->invert_horizontal) { ticks = -ticks; }
            event->code  = INPUT_REL_HWHEEL;
            event->value = ticks;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

/* ── driver boilerplate ──────────────────────────────────────────────────── */

static int circular_scroll_init(const struct device *dev) {
    circ_reset(dev->data);
    return 0;
}

static const struct zmk_input_processor_driver_api circular_scroll_api = {
    .handle_event = circular_scroll_handle_event,
};

#define CIRC_SCROLL_INST(n)                                                    \
    static struct circular_scroll_data circ_scroll_data_##n = {};              \
    static const struct circular_scroll_config circ_scroll_cfg_##n = {        \
        .pressure_threshold    = DT_INST_PROP(n, pressure_threshold),         \
        .x_max                 = DT_INST_PROP(n, x_max),                      \
        .y_max                 = DT_INST_PROP(n, y_max),                      \
        .inner_ring_pct        = DT_INST_PROP(n, inner_ring_pct),             \
        .sector_half_angle_deg = DT_INST_PROP(n, sector_half_angle_deg),      \
        .activation_angle_deg  = DT_INST_PROP(n, activation_angle_deg),       \
        .scroll_angle_deg      = DT_INST_PROP(n, scroll_angle_deg),           \
        .invert_vertical       = DT_INST_PROP(n, invert_vertical),            \
        .invert_horizontal     = DT_INST_PROP(n, invert_horizontal),          \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, circular_scroll_init, NULL,                       \
                          &circ_scroll_data_##n, &circ_scroll_cfg_##n,        \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &circular_scroll_api);

DT_INST_FOREACH_STATUS_OKAY(CIRC_SCROLL_INST)
