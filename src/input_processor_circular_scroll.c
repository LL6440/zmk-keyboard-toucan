/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll input processor — REL-event based, sliding-window detection.
 *
 * State machine:
 *   IDLE  → (circular motion detected) → SCROLL  → (finger lift) → IDLE
 *   IDLE  → (non-circular motion)      → CURSOR   → (finger lift) → IDLE
 *
 * Once an axis/mode is chosen it is LOCKED until the finger is lifted.
 * Finger lift is detected by N consecutive frames all below LIFT_SPEED.
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

#define HIST_SIZE           16
#define COMPARE_OFFSET       8   /* samples back to compare against          */

/* Min speed to participate in detection (avoids noise near lift).          */
#define MIN_SPEED_DETECT     6
/* Speed below which a frame counts toward "finger lift".                   */
#define LIFT_SPEED           3
/* Consecutive frames at LIFT_SPEED to declare finger lifted & reset.       */
#define LIFT_FRAMES          6

/* Min consecutive circular samples to enter scroll mode.                   */
#define CIRC_ENTER           3
/* Min consecutive non-circular samples to commit to CURSOR mode            *
 * (only applies during the IDLE detection phase, before mode is locked).   */
#define CURSOR_ENTER         5

/* |sin(angle)| threshold for circular detection.                           */
#define CIRC_SIN_THRESHOLD   0.22f

#define DEFAULT_SCROLL_DEG   12

/* ── state machine ───────────────────────────────────────────────────────── */

enum circ_state {
    STATE_IDLE = 0,   /* waiting to determine mode                          */
    STATE_SCROLL,     /* circular scroll mode — locked until lift           */
    STATE_CURSOR,     /* plain cursor mode   — locked until lift            */
};

enum circ_axis { AXIS_VERTICAL = 0, AXIS_HORIZONTAL };

/* ── config / data ───────────────────────────────────────────────────────── */

struct circular_scroll_config {
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
    int     head;
    int     filled;

    /* Current-report REL_X accumulator */
    int32_t curr_dx;
    bool    have_x;

    /* State machine */
    enum circ_state state;
    enum circ_axis  scroll_axis;

    /* Scroll angle accumulator */
    float angle_accum;

    /* Detection counters (IDLE phase only) */
    int   circ_count;    /* consecutive circular samples                    */
    int   cursor_count;  /* consecutive non-circular samples                */
    float circ_sign;     /* +1 or -1 — rotation direction                  */

    /* Axis-selection accumulators (IDLE phase only) */
    float abs_x_sum;
    float abs_y_sum;

    /* Finger-lift detector */
    int   lift_count;    /* consecutive slow frames                         */
};

/* ── helpers ─────────────────────────────────────────────────────────────── */

static void full_reset(struct circular_scroll_data *d) {
    d->head         = 0;
    d->filled       = 0;
    d->curr_dx      = 0;
    d->have_x       = false;
    d->state        = STATE_IDLE;
    d->angle_accum  = 0.0f;
    d->circ_count   = 0;
    d->cursor_count = 0;
    d->circ_sign    = 0.0f;
    d->abs_x_sum    = 0.0f;
    d->abs_y_sum    = 0.0f;
    d->lift_count   = 0;
}

static void push_vel(struct circular_scroll_data *d, int32_t dx, int32_t dy) {
    d->vx[d->head] = dx;
    d->vy[d->head] = dy;
    d->head = (d->head + 1) % HIST_SIZE;
    if (d->filled < HIST_SIZE) d->filled++;
}

static void get_past(const struct circular_scroll_data *d, int offset,
                     int32_t *ox, int32_t *oy) {
    int idx = (d->head - 1 - offset + HIST_SIZE * 2) % HIST_SIZE;
    *ox = d->vx[idx];
    *oy = d->vy[idx];
}

/* ── per-report processing ───────────────────────────────────────────────── */

/*
 * Returns the number of scroll ticks to emit (0 = none).
 * Also updates d->state.
 */
static int32_t process_vector(struct circular_scroll_data *d,
                              const struct circular_scroll_config *cfg,
                              int32_t dx, int32_t dy)
{
    int speed = IABS(dx) + IABS(dy);

    /* ── Finger-lift detection (runs in all states) ── */
    if (speed <= LIFT_SPEED) {
        d->lift_count++;
        if (d->lift_count >= LIFT_FRAMES) {
            full_reset(d);
            /* Don't push this slow sample; start fresh next time. */
            return 0;
        }
    } else {
        d->lift_count = 0;
    }

    push_vel(d, dx, dy);

    /* ── Locked states: no mode change allowed ── */

    if (d->state == STATE_CURSOR) {
        /* Plain cursor mode — just pass through, no scroll. */
        return 0;
    }

    if (d->state == STATE_SCROLL) {
        /* Scroll mode locked. Compute angle and emit ticks. */
        if (d->filled <= COMPARE_OFFSET || speed < MIN_SPEED_DETECT) {
            return 0;
        }
        int32_t px, py;
        get_past(d, COMPARE_OFFSET, &px, &py);

        float cross    = (float)px * dy - (float)py * dx;
        float dot      = (float)px * dx + (float)py * dy;
        float swept    = atan2f(cross, dot + 1e-6f) * RAD_TO_DEG;

        d->angle_accum += swept;

        int32_t scroll_deg = (cfg->scroll_angle_deg > 0)
                             ? cfg->scroll_angle_deg : DEFAULT_SCROLL_DEG;

        if (IABS((int)d->angle_accum) < scroll_deg) { return 0; }

        int32_t ticks   = (int32_t)(d->angle_accum / (float)scroll_deg);
        d->angle_accum -= (float)ticks * (float)scroll_deg;
        return ticks;
    }

    /* ── STATE_IDLE: determine mode ── */

    d->abs_x_sum += IABS(dx);
    d->abs_y_sum += IABS(dy);

    if (d->filled <= COMPARE_OFFSET || speed < MIN_SPEED_DETECT) {
        return 0;
    }

    int32_t px, py;
    get_past(d, COMPARE_OFFSET, &px, &py);

    float m1 = sqrtf((float)px*px + (float)py*py);
    float m2 = sqrtf((float)dx*dx + (float)dy*dy);
    if (m1 < 1.0f || m2 < 1.0f) { return 0; }

    float cross      = (float)px * dy - (float)py * dx;
    float norm_cross = cross / (m1 * m2);

    bool is_circular = (IABS((int)(norm_cross * 1000)) >
                        (int)(CIRC_SIN_THRESHOLD * 1000));

    if (is_circular) {
        float new_sign = (norm_cross > 0.0f) ? 1.0f : -1.0f;
        if (d->circ_count == 0 || d->circ_sign == new_sign) {
            d->circ_count++;
            d->circ_sign = new_sign;
        } else {
            /* Direction flipped — reset circular counter */
            d->circ_count = 1;
            d->circ_sign  = new_sign;
        }
        d->cursor_count = 0;

        if (d->circ_count >= CIRC_ENTER) {
            d->state       = STATE_SCROLL;
            d->scroll_axis = (d->abs_y_sum >= d->abs_x_sum)
                             ? AXIS_VERTICAL : AXIS_HORIZONTAL;
            d->angle_accum = 0.0f;
        }
    } else {
        d->cursor_count++;
        d->circ_count = 0;
        d->circ_sign  = 0.0f;

        if (d->cursor_count >= CURSOR_ENTER) {
            d->state = STATE_CURSOR;
        }
    }

    return 0;
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

    const struct circular_scroll_config *cfg  = dev->config;
    struct circular_scroll_data         *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->curr_dx = event->value;
        data->have_x  = true;
        /* In scroll mode, suppress X cursor movement. */
        return (data->state == STATE_SCROLL)
               ? ZMK_INPUT_PROC_STOP : ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        int32_t dx = data->have_x ? data->curr_dx : 0;
        int32_t dy = event->value;
        data->have_x  = false;
        data->curr_dx = 0;

        int32_t ticks = process_vector(data, cfg, dx, dy);

        if (data->state != STATE_SCROLL || ticks == 0) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

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
    full_reset(dev->data);
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
