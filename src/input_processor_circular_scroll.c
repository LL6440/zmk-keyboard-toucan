/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll — direction-at-touch-start detection (REL-event based).
 *
 * The Cirque module emits REL_X / REL_Y events directly.  No ABS events
 * are available, so position is inferred from the initial motion direction.
 *
 * Mode selection at each touch:
 *   Phase 1 – SETTLING (SETTLE_FRAMES valid frames):
 *     Accumulate the initial direction vector.
 *
 *   Phase 2 – CONFIRMING:
 *     Compare each new motion vector against the established initial direction.
 *     • Direction CURVES  → SCROLL (axis = initial dominant direction)
 *         |dy| ≥ |dx|  → VERTICAL   (finger at ~3h or ~9h)
 *         |dx| >  |dy| → HORIZONTAL (finger at ~6h or ~12h)
 *     • Direction STRAIGHT → CURSOR
 *
 *   Mode locked until finger lift (LIFT_FRAMES consecutive slow frames).
 */

#define DT_DRV_COMPAT zmk_input_processor_circular_scroll

#include <drivers/input_processor.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
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

/* Ring-buffer for scroll-angle accumulation (must be > COMPARE_OFFSET). */
#define HIST_SIZE           16
#define COMPARE_OFFSET       8

/* Minimum |dx|+|dy| to count as valid motion. */
#define MIN_SPEED            4

/* Fallback finger-lift detection when ABS_Z is not available. */
#define LIFT_SPEED           2
#define LIFT_FRAMES          8
/* If REL events stop for this long, treat the next REL event as a new touch. */
#define IDLE_LIFT_MS         50

/* ABS_Z debouncing / hysteresis for stable touch release. */
#define ABS_Z_UP_MARGIN      3   /* release only when z <= threshold - margin */
#define ABS_Z_UP_FRAMES      4   /* consecutive low-Z frames required */

/* Phase 1: accumulate initial direction over SETTLE_FRAMES valid frames. */
#define SETTLE_FRAMES        8

/* Phase 2: curvature detection thresholds. */
#define CURVE_SIN_MIN        0.25f   /* |sin| of angle vs initial dir       */
#define CIRC_CONFIRM         3       /* consecutive curved frames → SCROLL   */
#define STRAIGHT_CONFIRM    10       /* consecutive straight frames → CURSOR */

/* Degrees of arc per scroll tick (overridden by DTS scroll-angle-deg). */
#define DEFAULT_SCROLL_DEG  12

/* ── state machine ───────────────────────────────────────────────────────── */

enum circ_state { STATE_IDLE = 0, STATE_SCROLL, STATE_CURSOR };
enum circ_axis  { AXIS_VERTICAL = 0, AXIS_HORIZONTAL };

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
    /* Velocity ring-buffer (for scroll-angle accumulation). */
    int32_t vx[HIST_SIZE];
    int32_t vy[HIST_SIZE];
    int     head;
    int     filled;

    /* Current-report REL_X accumulator. */
    int32_t curr_dx;
    bool    have_x;

    /* State machine. */
    enum circ_state state;
    enum circ_axis  scroll_axis;
    float           angle_accum;

    /* Phase 1 – settle. */
    int     settle_count;
    int32_t settle_sum_dx;
    int32_t settle_sum_dy;

    /* Phase 2 – confirm. Normalised initial direction. */
    float   init_nx;
    float   init_ny;
    int     circ_count;
    int     straight_count;

    /* Touch state from ABS_Z when available. */
    bool    touch_active;
    bool    pressure_supported;
    int32_t last_z;
    int     low_z_count;

    /* Fallback finger-lift detector (used only without ABS_Z). */
    int     lift_count;
    int64_t last_rel_ms;
};

/* ── helpers ─────────────────────────────────────────────────────────────── */

static void gesture_reset(struct circular_scroll_data *d) {
    d->head           = 0;
    d->filled         = 0;
    d->curr_dx        = 0;
    d->have_x         = false;
    d->state          = STATE_IDLE;
    d->scroll_axis    = AXIS_VERTICAL;
    d->angle_accum    = 0.0f;
    d->settle_count   = 0;
    d->settle_sum_dx  = 0;
    d->settle_sum_dy  = 0;
    d->init_nx        = 0.0f;
    d->init_ny        = 0.0f;
    d->circ_count     = 0;
    d->straight_count = 0;
    d->lift_count     = 0;
    d->last_rel_ms     = 0;
}

static void full_reset(struct circular_scroll_data *d) {
    gesture_reset(d);
    d->touch_active       = false;
    d->pressure_supported = false;
    d->last_z             = 0;
    d->low_z_count        = 0;
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

    /* ── Finger-lift detection ── */
    if (d->pressure_supported) {
        /* With ABS_Z support, never unlock on slowdown alone. */
        if (!d->touch_active || speed <= LIFT_SPEED) {
            return 0;
        }
        d->lift_count = 0;
    } else {
        /* Fallback for builds where only REL events are observed. */
        if (speed <= LIFT_SPEED) {
            d->lift_count++;
            if (d->lift_count >= LIFT_FRAMES) {
                if (d->state != STATE_IDLE) {
                    LOG_ERR("CIRC lift (was %s)",
                            d->state == STATE_SCROLL ? "SCROLL" : "CURSOR");
                }
                gesture_reset(d);
                d->touch_active = false;
            }
            return 0;
        }
        d->lift_count = 0;
    }

    push_vel(d, dx, dy);

    /* ── Locked: CURSOR ── */
    if (d->state == STATE_CURSOR) {
        return 0;
    }

    /* ── Locked: SCROLL ── */
    if (d->state == STATE_SCROLL) {
        if (d->filled <= COMPARE_OFFSET || speed < MIN_SPEED) return 0;

        int32_t px, py;
        get_past(d, COMPARE_OFFSET, &px, &py);

        float cross  = (float)px * dy - (float)py * dx;
        float dot    = (float)px * dx + (float)py * dy;
        float swept  = atan2f(cross, dot + 1e-6f) * RAD_TO_DEG;

        d->angle_accum += swept;

        int32_t sdeg = (cfg->scroll_angle_deg > 0)
                       ? cfg->scroll_angle_deg : DEFAULT_SCROLL_DEG;

        if (IABS((int)d->angle_accum) < sdeg) return 0;

        int32_t ticks   = (int32_t)(d->angle_accum / (float)sdeg);
        d->angle_accum -= (float)ticks * (float)sdeg;
        return ticks;
    }

    /* ── STATE_IDLE: phase 1 (settle) or phase 2 (confirm) ── */
    if (speed < MIN_SPEED) return 0;

    /* Phase 1 */
    if (d->settle_count < SETTLE_FRAMES) {
        d->settle_sum_dx += dx;
        d->settle_sum_dy += dy;
        d->settle_count++;

        if (d->settle_count == SETTLE_FRAMES) {
            float mag = sqrtf((float)d->settle_sum_dx * d->settle_sum_dx +
                              (float)d->settle_sum_dy * d->settle_sum_dy);
            if (mag > 0.5f) {
                d->init_nx = d->settle_sum_dx / mag;
                d->init_ny = d->settle_sum_dy / mag;
                d->scroll_axis = (IABS((int)(d->init_ny * 1000)) >=
                                  IABS((int)(d->init_nx * 1000)))
                                 ? AXIS_VERTICAL : AXIS_HORIZONTAL;
            } else {
                /* Barely moved — restart settle. */
                d->settle_count  = 0;
                d->settle_sum_dx = 0;
                d->settle_sum_dy = 0;
            }
        }
        return 0;
    }

    /* Phase 2: curvature detection */
    float cur_mag = sqrtf((float)dx*dx + (float)dy*dy);
    if (cur_mag < 1.0f) return 0;

    float cross_n = d->init_nx * (dy / cur_mag)
                  - d->init_ny * (dx / cur_mag);
    bool curved = (IABS((int)(cross_n * 1000)) > (int)(CURVE_SIN_MIN * 1000));

    if (curved) {
        d->circ_count++;
        d->straight_count = 0;
        if (d->circ_count >= CIRC_CONFIRM) {
            d->state       = STATE_SCROLL;
            d->angle_accum = 0.0f;
            LOG_ERR("CIRC -> SCROLL %s init=(%.2f,%.2f)",
                    d->scroll_axis == AXIS_VERTICAL ? "VERT" : "HORIZ",
                    (double)d->init_nx, (double)d->init_ny);
        }
    } else {
        d->straight_count++;
        d->circ_count = 0;
        if (d->straight_count >= STRAIGHT_CONFIRM) {
            d->state = STATE_CURSOR;
            LOG_ERR("CIRC -> CURSOR init=(%.2f,%.2f)",
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

    if (event->type == INPUT_EV_ABS && event->code == INPUT_ABS_Z) {
        bool was_touching = data->touch_active;
        int32_t up_threshold = MAX((int32_t)cfg->pressure_threshold - ABS_Z_UP_MARGIN, 0);

        data->pressure_supported = true;
        data->last_z = event->value;

        if (!was_touching) {
            if (event->value >= cfg->pressure_threshold) {
                gesture_reset(data);
                data->touch_active = true;
                data->low_z_count = 0;
                LOG_ERR("CIRC touch-down z=%d", event->value);
            }
            return ZMK_INPUT_PROC_STOP;
        }

        if (event->value <= up_threshold) {
            data->low_z_count++;
            if (data->low_z_count >= ABS_Z_UP_FRAMES) {
                if (data->state != STATE_IDLE) {
                    LOG_ERR("CIRC touch-up z=%d low=%d/%d (was %s)",
                            event->value, data->low_z_count, ABS_Z_UP_FRAMES,
                            data->state == STATE_SCROLL ? "SCROLL" : "CURSOR");
                }
                gesture_reset(data);
                data->touch_active = false;
                data->low_z_count = 0;
            }
        } else {
            data->low_z_count = 0;
        }

        return ZMK_INPUT_PROC_STOP;
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->pressure_supported) {
        int64_t now = k_uptime_get();
        if (data->last_rel_ms > 0 && (now - data->last_rel_ms) > IDLE_LIFT_MS) {
            if (data->state != STATE_IDLE) {
                LOG_ERR("CIRC idle-lift dt=%lldms (was %s)",
                        (long long)(now - data->last_rel_ms),
                        data->state == STATE_SCROLL ? "SCROLL" : "CURSOR");
            }
            gesture_reset(data);
        }
        data->last_rel_ms = now;
    }

    if (data->pressure_supported && !data->touch_active) {
        data->have_x = false;
        data->curr_dx = 0;
        return ZMK_INPUT_PROC_STOP;
    }

    if (event->code == INPUT_REL_X) {
        data->curr_dx = event->value;
        data->have_x  = true;
        return (data->state == STATE_SCROLL)
               ? ZMK_INPUT_PROC_STOP : ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        int32_t dx = data->have_x ? data->curr_dx : 0;
        int32_t dy = event->value;
        data->have_x  = false;
        data->curr_dx = 0;

        int32_t ticks = process_vector(data, cfg, dx, dy);

        /* While scroll mode is locked, never let REL_Y fall through as pointer
         * motion.  If no wheel tick is ready yet, swallow the event and keep
         * waiting for more arc accumulation.
         */
        if (data->state == STATE_SCROLL && ticks == 0) {
            return ZMK_INPUT_PROC_STOP;
        }

        if (data->state != STATE_SCROLL) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (data->scroll_axis == AXIS_VERTICAL) {
            if (cfg->invert_vertical) ticks = -ticks;
            event->code  = INPUT_REL_WHEEL;
            event->value = ticks;
        } else {
            if (cfg->invert_horizontal) ticks = -ticks;
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
