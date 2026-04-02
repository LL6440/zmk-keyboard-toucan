/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll input processor — REL-event based implementation.
 *
 * Detects circular gestures on the trackpad by analysing the cross-product
 * of consecutive velocity vectors (REL_X, REL_Y pairs).  No absolute
 * position or pressure information is required.
 *
 * When circular motion is detected the processor consumes the REL_X/REL_Y
 * events and emits REL_WHEEL (vertical) or REL_HWHEEL (horizontal) instead.
 *
 * Scroll axis is selected from the dominant direction of the FIRST motion
 * at the start of each circular gesture:
 *   - dominant Y  →  vertical scroll  (finger starting at 3 h or 9 h)
 *   - dominant X  →  horizontal scroll (finger starting at 6 h or 12 h)
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

#define IABS(x) ((x) < 0 ? -(x) : (x))

/* ── tuneable constants ─────────────────────────────────────────────────── */

/* Minimum per-report speed (sum |dx|+|dy|) to participate in detection.   */
#define MIN_SPEED 2

/* |sin(angle_between_vectors)| must exceed this to count as "circular".   */
/* sin(15°) ≈ 0.26 — raise to require tighter circles, lower to be lenient */
#define CIRC_SIN_THRESHOLD 0.20f

/* Consecutive circular samples required to enter scroll mode.             */
#define CIRC_ENTER 3

/* Consecutive non-circular samples required to exit scroll mode.          */
#define CIRC_EXIT  4

/* Degrees of accumulated circular arc to emit one scroll tick.            */
/* (overridden by DTS scroll-angle-deg if > 0)                             */
#define DEFAULT_SCROLL_DEG 15

/* ── types ──────────────────────────────────────────────────────────────── */

enum circ_axis { AXIS_UNKNOWN = 0, AXIS_VERTICAL, AXIS_HORIZONTAL };

struct circular_scroll_config {
    /* kept from old DTS binding so the .yaml / .dtsi don't need to change */
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
    /* Current-report accumulated REL deltas */
    int32_t  curr_dx;
    int32_t  curr_dy;
    bool     have_x;

    /* Previous-report velocity vector */
    int32_t  prev_dx;
    int32_t  prev_dy;
    bool     have_prev;

    /* Scroll-mode state */
    bool          scroll_mode;
    enum circ_axis scroll_axis;
    float          angle_accum;   /* degrees accumulated since last tick  */

    /* Axis selection during first samples */
    float abs_x_sum;
    float abs_y_sum;
    int   axis_samples;

    /* Circular detection counters */
    int   enter_count;   /* consecutive circular samples                  */
    int   exit_count;    /* consecutive non-circular samples in scroll mode */
    float circ_sign;     /* +1 or -1 — rotation direction                 */
};

/* ── helpers ─────────────────────────────────────────────────────────────── */

static inline float vec_mag_sq(int32_t dx, int32_t dy) {
    return (float)dx * dx + (float)dy * dy;
}

static inline float vec_cross(int32_t ax, int32_t ay, int32_t bx, int32_t by) {
    return (float)ax * by - (float)ay * bx;
}

static inline float vec_dot(int32_t ax, int32_t ay, int32_t bx, int32_t by) {
    return (float)ax * bx + (float)ay * by;
}

static void circ_reset(struct circular_scroll_data *d) {
    d->curr_dx     = 0;
    d->curr_dy     = 0;
    d->have_x      = false;
    d->prev_dx     = 0;
    d->prev_dy     = 0;
    d->have_prev   = false;
    d->scroll_mode = false;
    d->scroll_axis = AXIS_UNKNOWN;
    d->angle_accum = 0.0f;
    d->abs_x_sum   = 0.0f;
    d->abs_y_sum   = 0.0f;
    d->axis_samples= 0;
    d->enter_count = 0;
    d->exit_count  = 0;
    d->circ_sign   = 0.0f;
}

/* ── per-report processing ───────────────────────────────────────────────── */

/*
 * Called when we have a complete (curr_dx, curr_dy) pair.
 * Updates scroll_mode, angle_accum, scroll_axis.
 * Returns the scroll ticks to emit (0 = none).
 */
static int32_t process_vector(struct circular_scroll_data *d,
                              const struct circular_scroll_config *cfg)
{
    int32_t dx = d->curr_dx;
    int32_t dy = d->curr_dy;

    int speed = IABS(dx) + IABS(dy);

    if (!d->have_prev || speed < MIN_SPEED) {
        /* First sample, or finger barely moved — just store, don't detect. */
        d->have_prev = true;
        d->prev_dx   = dx;
        d->prev_dy   = dy;
        if (!d->scroll_mode) {
            d->enter_count = 0;
        }
        return 0;
    }

    /* Compute normalised cross product  sin(angle_between_vectors). */
    float cross     = vec_cross(d->prev_dx, d->prev_dy, dx, dy);
    float mag_prev  = sqrtf(vec_mag_sq(d->prev_dx, d->prev_dy));
    float mag_curr  = sqrtf(vec_mag_sq(dx, dy));
    float norm_cross = cross / (mag_prev * mag_curr + 1e-6f);

    /* Angle swept this sample (signed, degrees). */
    float dot        = vec_dot(d->prev_dx, d->prev_dy, dx, dy);
    float swept_deg  = atan2f(cross, dot + 1e-6f) * RAD_TO_DEG;

    d->prev_dx = dx;
    d->prev_dy = dy;

    bool is_circular = (IABS((int)(norm_cross * 1000)) > (int)(CIRC_SIN_THRESHOLD * 1000));

    if (!d->scroll_mode) {
        /* Accumulate axis-selection data */
        d->abs_x_sum   += IABS(dx);
        d->abs_y_sum   += IABS(dy);
        d->axis_samples++;

        if (is_circular) {
            float new_sign = (norm_cross > 0.0f) ? 1.0f : -1.0f;
            if (d->enter_count == 0 || d->circ_sign == new_sign) {
                d->enter_count++;
                d->circ_sign = new_sign;
            } else {
                /* Direction changed — restart. */
                d->enter_count = 1;
                d->circ_sign   = new_sign;
            }

            if (d->enter_count >= CIRC_ENTER) {
                /* Enter scroll mode. */
                d->scroll_mode = true;
                d->exit_count  = 0;
                d->angle_accum = 0.0f;
                /* Choose axis from accumulated motion direction. */
                if (d->abs_y_sum >= d->abs_x_sum) {
                    d->scroll_axis = AXIS_VERTICAL;
                } else {
                    d->scroll_axis = AXIS_HORIZONTAL;
                }
            }
        } else {
            d->enter_count = 0;
        }
        return 0; /* Still in cursor mode — no scroll yet. */
    }

    /* ── Scroll mode ── */
    if (!is_circular) {
        d->exit_count++;
        if (d->exit_count >= CIRC_EXIT) {
            /* Motion no longer circular — go back to cursor mode. */
            circ_reset(d);
        }
        return 0;
    }
    d->exit_count = 0;

    /* Accumulate swept angle (positive = one direction, negative = other). */
    d->angle_accum += swept_deg;

    int32_t scroll_deg = (d->scroll_axis == AXIS_VERTICAL)
        ? ((cfg->scroll_angle_deg > 0) ? cfg->scroll_angle_deg : DEFAULT_SCROLL_DEG)
        : ((cfg->scroll_angle_deg > 0) ? cfg->scroll_angle_deg : DEFAULT_SCROLL_DEG);

    if (IABS((int)d->angle_accum) < scroll_deg) {
        return 0;
    }

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

    /* Only handle REL events. */
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->curr_dx = event->value;
        data->have_x  = true;
        /* In scroll mode suppress X cursor movement. */
        if (data->scroll_mode) {
            return ZMK_INPUT_PROC_STOP;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        data->curr_dy = event->value;
        if (!data->have_x) {
            data->curr_dx = 0;
        }
        data->have_x = false;

        int32_t ticks = process_vector(data, cfg);

        /* Reset accumulators for next report. */
        data->curr_dx = 0;
        data->curr_dy = 0;

        if (!data->scroll_mode || ticks == 0) {
            /* Cursor mode or no tick yet — pass Y through. */
            return ZMK_INPUT_PROC_CONTINUE;
        }

        /* Emit scroll event by modifying this REL_Y event in-place. */
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
    struct circular_scroll_data *data = dev->data;
    circ_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api circular_scroll_api = {
    .handle_event = circular_scroll_handle_event,
};

#define CIRC_SCROLL_INST(n)                                                    \
    static struct circular_scroll_data circ_scroll_data_##n = {};              \
    static const struct circular_scroll_config circ_scroll_cfg_##n = {        \
        .pressure_threshold   = DT_INST_PROP(n, pressure_threshold),          \
        .x_max                = DT_INST_PROP(n, x_max),                       \
        .y_max                = DT_INST_PROP(n, y_max),                       \
        .inner_ring_pct       = DT_INST_PROP(n, inner_ring_pct),              \
        .sector_half_angle_deg= DT_INST_PROP(n, sector_half_angle_deg),       \
        .activation_angle_deg = DT_INST_PROP(n, activation_angle_deg),        \
        .scroll_angle_deg     = DT_INST_PROP(n, scroll_angle_deg),            \
        .invert_vertical      = DT_INST_PROP(n, invert_vertical),             \
        .invert_horizontal    = DT_INST_PROP(n, invert_horizontal),           \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, circular_scroll_init, NULL,                       \
                          &circ_scroll_data_##n, &circ_scroll_cfg_##n,        \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &circular_scroll_api);

DT_INST_FOREACH_STATUS_OKAY(CIRC_SCROLL_INST)
