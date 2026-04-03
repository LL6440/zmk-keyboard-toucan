/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll — direction-at-touch-start detection.
 *
 * Mode selection:
 *   Phase 1 – SETTLING (first SETTLE_FRAMES valid frames):
 *     Accumulate the initial direction vector of the finger.
 *
 *   Phase 2 – CONFIRMING:
 *     Compare each new motion vector against the initial direction.
 *     If the direction CURVES (cross-product large) → SCROLL mode.
 *       Scroll axis = initial dominant direction:
 *         |dy| > |dx|  →  VERTICAL scroll   (finger started at ~3h or ~9h)
 *         |dx| > |dy|  →  HORIZONTAL scroll  (finger started at ~6h or ~12h)
 *     If direction stays STRAIGHT → CURSOR mode.
 *
 *   Once a mode is chosen it is locked until the finger is lifted.
 *
 * Finger-lift detection: LIFT_FRAMES consecutive frames with speed ≤ LIFT_SPEED.
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

/* Ring-buffer for scroll-angle accumulation. */
#define HIST_SIZE           16
#define COMPARE_OFFSET       8   /* samples back for swept-angle computation */

/* Minimum |dx|+|dy| to count as valid motion (filter pad noise). */
#define MIN_SPEED            5

/* Finger-lift: LIFT_FRAMES consecutive frames with speed ≤ LIFT_SPEED → reset. */
#define LIFT_SPEED           3
#define LIFT_FRAMES          6

/* Phase 1 – initial direction settling. */
#define SETTLE_FRAMES        7   /* valid frames to accumulate initial dir   */

/* Phase 2 – curvature confirmation. */
/* |sin(angle between initial_dir and current motion)| threshold.           */
#define CURVE_SIN_MIN        0.28f
/* Consecutive curved frames required to enter SCROLL mode. */
#define CIRC_CONFIRM         3
/* Consecutive straight frames required to enter CURSOR mode. */
#define STRAIGHT_CONFIRM     8

/* Degrees of arc per scroll tick (overridden by DTS scroll-angle-deg). */
#define DEFAULT_SCROLL_DEG   12

/* ── types ──────────────────────────────────────────────────────────────── */

enum circ_state {
    STATE_IDLE = 0,   /* settling or confirming — no mode locked yet */
    STATE_SCROLL,     /* circular scroll, locked until lift           */
    STATE_CURSOR,     /* plain cursor, locked until lift              */
};

enum circ_axis { AXIS_VERTICAL = 0, AXIS_HORIZONTAL };

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
    /* Velocity ring-buffer (for scroll-angle accumulation). */
    int32_t vx[HIST_SIZE];
    int32_t vy[HIST_SIZE];
    int     head;
    int     filled;

    /* REL_X accumulator (paired with the following REL_Y). */
    int32_t curr_dx;
    bool    have_x;

    /* State machine. */
    enum circ_state state;
    enum circ_axis  scroll_axis;

    /* Phase 1 – settle. */
    int     settle_count;     /* valid frames accumulated so far     */
    int32_t settle_sum_dx;    /* accumulated dx during settle        */
    int32_t settle_sum_dy;    /* accumulated dy during settle        */

    /* Phase 2 – confirm. Normalised initial direction vector. */
    float   init_nx;          /* x component (unit vector)           */
    float   init_ny;          /* y component (unit vector)           */
    int     circ_count;       /* consecutive curved frames           */
    int     straight_count;   /* consecutive straight frames         */

    /* Scroll angle accumulator (used in STATE_SCROLL). */
    float   angle_accum;

    /* Finger-lift detector. */
    int     lift_count;

    /* Accumulated position since finger-down (for logging). */
    int32_t pos_x;
    int32_t pos_y;
    int     log_frame;   /* kept for compat, unused */
};

/* ── helpers ─────────────────────────────────────────────────────────────── */

static void full_reset(struct circular_scroll_data *d) {
    d->head          = 0;
    d->filled        = 0;
    d->curr_dx       = 0;
    d->have_x        = false;
    d->state         = STATE_IDLE;
    d->scroll_axis   = AXIS_VERTICAL;
    d->settle_count  = 0;
    d->settle_sum_dx = 0;
    d->settle_sum_dy = 0;
    d->init_nx       = 0.0f;
    d->init_ny       = 0.0f;
    d->circ_count    = 0;
    d->straight_count= 0;
    d->angle_accum   = 0.0f;
    d->lift_count    = 0;
    d->pos_x         = 0;
    d->pos_y         = 0;
    d->log_frame     = 0;
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

static int32_t process_vector(struct circular_scroll_data *d,
                              const struct circular_scroll_config *cfg,
                              int32_t dx, int32_t dy)
{
    int speed = IABS(dx) + IABS(dy);

    /* ── Finger-lift detection (all states) ── */
    if (speed <= LIFT_SPEED) {
        d->lift_count++;
        if (d->lift_count >= LIFT_FRAMES) {
            LOG_ERR("CIRC -> LIFT final_pos=(%d,%d)", (int)d->pos_x, (int)d->pos_y);
            full_reset(d);
        }
        return 0;
    }
    d->lift_count = 0;

    /* Accumulate position for logging. */
    d->pos_x += dx;
    d->pos_y += dy;

    /* ── Locked: CURSOR ── */
    if (d->state == STATE_CURSOR) {
        return 0;  /* pass events through normally */
    }

    /* ── Locked: SCROLL ── */
    if (d->state == STATE_SCROLL) {
        if (d->filled <= COMPARE_OFFSET || speed < MIN_SPEED) {
            return 0;
        }
        int32_t px, py;
        get_past(d, COMPARE_OFFSET, &px, &py);

        float cross = (float)px * dy - (float)py * dx;
        float dot   = (float)px * dx + (float)py * dy;
        float swept = atan2f(cross, dot + 1e-6f) * RAD_TO_DEG;

        d->angle_accum += swept;

        int32_t sdeg = (cfg->scroll_angle_deg > 0)
                       ? cfg->scroll_angle_deg : DEFAULT_SCROLL_DEG;

        if (IABS((int)d->angle_accum) < sdeg) { return 0; }

        int32_t ticks   = (int32_t)(d->angle_accum / (float)sdeg);
        d->angle_accum -= (float)ticks * (float)sdeg;
        return ticks;
    }

    /* ── STATE_IDLE: phase 1 (settle) or phase 2 (confirm) ── */

    if (speed < MIN_SPEED) { return 0; }

    /* Phase 1 — accumulate initial direction. */
    if (d->settle_count < SETTLE_FRAMES) {
        d->settle_sum_dx += dx;
        d->settle_sum_dy += dy;
        d->settle_count++;

        if (d->settle_count == SETTLE_FRAMES) {
            /* Compute and store normalised initial direction. */
            float mag = sqrtf((float)d->settle_sum_dx * d->settle_sum_dx +
                              (float)d->settle_sum_dy * d->settle_sum_dy);
            if (mag > 0.5f) {
                d->init_nx = d->settle_sum_dx / mag;
                d->init_ny = d->settle_sum_dy / mag;
                /* Choose scroll axis from dominant component of initial dir. */
                if (IABS((int)(d->init_ny * 1000)) >=
                    IABS((int)(d->init_nx * 1000))) {
                    d->scroll_axis = AXIS_VERTICAL;
                } else {
                    d->scroll_axis = AXIS_HORIZONTAL;
                }
            } else {
                /* Barely moved — restart settle. */
                d->settle_count  = 0;
                d->settle_sum_dx = 0;
                d->settle_sum_dy = 0;
            }
        }
        return 0;
    }

    /* Phase 2 — check if motion curves relative to initial direction. */
    float cur_mag = sqrtf((float)dx*dx + (float)dy*dy);
    if (cur_mag < 1.0f) { return 0; }

    /* Cross product of initial_dir (unit) with current motion (unit). */
    float cross_n = d->init_nx * (dy / cur_mag) - d->init_ny * (dx / cur_mag);
    bool curved   = (IABS((int)(cross_n * 1000)) > (int)(CURVE_SIN_MIN * 1000));

    if (curved) {
        d->circ_count++;
        d->straight_count = 0;
        if (d->circ_count >= CIRC_CONFIRM) {
            d->state       = STATE_SCROLL;
            d->angle_accum = 0.0f;
            LOG_ERR("CIRC -> SCROLL axis=%s pos=(%d,%d) init=(%.2f,%.2f)",
                    (d->scroll_axis == AXIS_VERTICAL) ? "VERT" : "HORIZ",
                    (int)d->pos_x, (int)d->pos_y,
                    (double)d->init_nx, (double)d->init_ny);
        }
    } else {
        d->straight_count++;
        d->circ_count = 0;
        if (d->straight_count >= STRAIGHT_CONFIRM) {
            d->state = STATE_CURSOR;
            LOG_ERR("CIRC -> CURSOR pos=(%d,%d) init=(%.2f,%.2f)",
                    (int)d->pos_x, (int)d->pos_y,
                    (double)d->init_nx, (double)d->init_ny);
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
        /* Suppress X cursor movement once in scroll mode. */
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
