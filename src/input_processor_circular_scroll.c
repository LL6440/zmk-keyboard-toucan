/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll input processor — REL-event based, sliding-window detection.
 *
 * Gesture selection happens in two stages:
 *   1) The touch must START near a cardinal point on the outer ring.
 *      - near 3h or 9h  => candidate vertical scroll
 *      - near 6h or 12h => candidate horizontal scroll
 *   2) The FIRST meaningful motion must move along the ring (tangential).
 *      A radial first motion keeps the whole contact as normal pointer motion.
 *
 * Once a contact is classified as scroll, it stays scroll until finger lift.
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
#define HIST_SIZE       16
#define COMPARE_OFFSET   8
#define MIN_SPEED        6
#define CIRC_SIN_THRESHOLD   0.22f
#define CIRC_ENTER       3
#define DEFAULT_SCROLL_DEG  12
#define DECISION_SPEED      10
#define DECISION_MARGIN      4.0f

/* ── types ──────────────────────────────────────────────────────────────── */
enum circ_axis { AXIS_UNKNOWN = 0, AXIS_VERTICAL, AXIS_HORIZONTAL };

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
    int32_t vx[HIST_SIZE];
    int32_t vy[HIST_SIZE];
    int     head;
    int     filled;

    int32_t curr_dx;
    bool    have_x;

    bool           touch_active;
    bool           scroll_mode;
    enum circ_axis scroll_axis;      /* selected mode; locked until lift */
    enum circ_axis candidate_axis;   /* sector-based candidate before first motion decision */
    float          angle_accum;

    int   enter_count;
    float circ_sign;

    int32_t abs_x;
    int32_t abs_y;
    bool    have_abs_x;
    bool    have_abs_y;
    bool    start_decided;

    float start_radial_x;
    float start_radial_y;
    float start_tangent_x;
    float start_tangent_y;
    int32_t decision_dx;
    int32_t decision_dy;
};

/* ── helpers ─────────────────────────────────────────────────────────────── */
static void circ_reset(struct circular_scroll_data *d) {
    d->head         = 0;
    d->filled       = 0;
    d->curr_dx      = 0;
    d->have_x       = false;
    d->touch_active = false;
    d->scroll_mode  = false;
    d->scroll_axis  = AXIS_UNKNOWN;
    d->candidate_axis = AXIS_UNKNOWN;
    d->angle_accum  = 0.0f;
    d->enter_count  = 0;
    d->circ_sign    = 0.0f;
    d->abs_x        = 0;
    d->abs_y        = 0;
    d->have_abs_x   = false;
    d->have_abs_y   = false;
    d->start_decided = false;
    d->start_radial_x = 0.0f;
    d->start_radial_y = 0.0f;
    d->start_tangent_x = 0.0f;
    d->start_tangent_y = 0.0f;
    d->decision_dx = 0;
    d->decision_dy = 0;
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

static float smallest_angle_delta_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static enum circ_axis configure_start_sector(const struct circular_scroll_config *cfg,
                                             struct circular_scroll_data *d,
                                             int32_t x, int32_t y) {
    float cx = (float)cfg->x_max / 2.0f;
    float cy = (float)cfg->y_max / 2.0f;
    float dx_screen = (float)x - cx;
    float dy_screen = (float)y - cy;
    float dy_up = -dy_screen; /* convert screen-down to math-up for angles */

    float r = sqrtf(dx_screen * dx_screen + dy_screen * dy_screen);
    float min_half = ((float)MIN(cfg->x_max, cfg->y_max)) / 2.0f;
    float min_r = ((float)cfg->inner_ring_pct / 100.0f) * min_half;
    if (r < min_r) {
        return AXIS_UNKNOWN;
    }

    float radial_x = dx_screen / r;
    float radial_y = dy_screen / r;
    d->start_radial_x = radial_x;
    d->start_radial_y = radial_y;
    d->start_tangent_x = -radial_y;
    d->start_tangent_y = radial_x;

    float ang = atan2f(dy_up, dx_screen) * RAD_TO_DEG;
    float half = (float)cfg->sector_half_angle_deg;

    if (fabsf(smallest_angle_delta_deg(ang, 0.0f)) <= half ||
        fabsf(smallest_angle_delta_deg(ang, 180.0f)) <= half ||
        fabsf(smallest_angle_delta_deg(ang, -180.0f)) <= half) {
        return AXIS_VERTICAL;   /* 3h or 9h */
    }
    if (fabsf(smallest_angle_delta_deg(ang, -90.0f)) <= half ||
        fabsf(smallest_angle_delta_deg(ang, 90.0f)) <= half) {
        return AXIS_HORIZONTAL; /* 6h or 12h */
    }
    return AXIS_UNKNOWN;
}

/* ── per-report processing ───────────────────────────────────────────────── */
static bool decide_gesture_start(struct circular_scroll_data *d) {
    if (d->candidate_axis == AXIS_UNKNOWN) {
        d->scroll_axis = AXIS_UNKNOWN;
        d->start_decided = true;
        return true;
    }

    float total_dx = (float)d->decision_dx;
    float total_dy = (float)d->decision_dy;
    int total_speed = IABS(d->decision_dx) + IABS(d->decision_dy);
    if (total_speed < DECISION_SPEED) {
        return false;
    }

    float tangential = fabsf(total_dx * d->start_tangent_x + total_dy * d->start_tangent_y);
    float radial = fabsf(total_dx * d->start_radial_x + total_dy * d->start_radial_y);

    if ((tangential - radial) >= DECISION_MARGIN) {
        d->scroll_axis = d->candidate_axis;
        d->scroll_mode = true;
        d->start_decided = true;
        d->angle_accum = 0.0f;
        d->enter_count = 0;
        d->circ_sign = 0.0f;
        return true;
    }

    if ((radial - tangential) >= DECISION_MARGIN) {
        d->scroll_axis = AXIS_UNKNOWN;
        d->start_decided = true;
        return true;
    }

    return false;
}

static int32_t process_vector(struct circular_scroll_data *d,
                              const struct circular_scroll_config *cfg,
                              int32_t dx, int32_t dy) {
    int speed = IABS(dx) + IABS(dy);

    push_vel(d, dx, dy);

    if (!d->start_decided) {
        d->decision_dx += dx;
        d->decision_dy += dy;
        if (!decide_gesture_start(d)) {
            return 0;
        }
    }

    /* Pointer-only gesture: never try to capture scroll. */
    if (d->scroll_axis == AXIS_UNKNOWN) {
        return 0;
    }

    /* Not enough information yet. */
    if (d->filled <= COMPARE_OFFSET || speed < MIN_SPEED) {
        return 0;
    }

    int32_t px, py;
    get_past(d, COMPARE_OFFSET, &px, &py);

    float cross = (float)px * dy - (float)py * dx;
    float m1    = sqrtf((float)px*px + (float)py*py);
    float m2    = sqrtf((float)dx*dx + (float)dy*dy);
    if (m1 < 1.0f || m2 < 1.0f) { return 0; }

    float norm_cross = cross / (m1 * m2);
    float dot        = (float)px*dx + (float)py*dy;
    float swept_deg  = atan2f(cross, dot + 1e-6f) * RAD_TO_DEG;
    bool is_circular = (IABS((int)(norm_cross * 1000)) >
                        (int)(CIRC_SIN_THRESHOLD * 1000));

    if (!d->scroll_mode) {
        if (!is_circular) {
            d->enter_count = 0;
            return 0;
        }

        float new_sign = (norm_cross > 0.0f) ? 1.0f : -1.0f;
        if (d->enter_count == 0 || d->circ_sign == new_sign) {
            d->enter_count++;
            d->circ_sign = new_sign;
        } else {
            d->enter_count = 1;
            d->circ_sign   = new_sign;
        }

        d->angle_accum += swept_deg;

        float activation_deg = (cfg->activation_angle_deg > 0)
                               ? (float)cfg->activation_angle_deg : 10.0f;
        if (d->enter_count >= CIRC_ENTER && fabsf(d->angle_accum) >= activation_deg) {
            d->scroll_mode = true;
            d->angle_accum = 0.0f;
        }
        return 0;
    }

    /* Scroll mode stays latched until lift, but only circular motion emits ticks. */
    if (!is_circular) {
        return 0;
    }

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
                                        struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct circular_scroll_config *cfg = dev->config;
    struct circular_scroll_data         *data = dev->data;

    if (event->type == INPUT_EV_ABS) {
        switch (event->code) {
        case INPUT_ABS_X:
            data->abs_x = event->value;
            data->have_abs_x = true;
            if (data->touch_active && !data->start_decided && data->have_abs_y) {
                data->candidate_axis = configure_start_sector(cfg, data, data->abs_x, data->abs_y);
            }
            return ZMK_INPUT_PROC_CONTINUE;

        case INPUT_ABS_Y:
            data->abs_y = event->value;
            data->have_abs_y = true;
            if (data->touch_active && !data->start_decided && data->have_abs_x) {
                data->candidate_axis = configure_start_sector(cfg, data, data->abs_x, data->abs_y);
            }
            return ZMK_INPUT_PROC_CONTINUE;

        case INPUT_ABS_Z:
            if (event->value < cfg->pressure_threshold) {
                circ_reset(data);
            } else if (!data->touch_active) {
                data->touch_active  = true;
                data->curr_dx       = 0;
                data->have_x        = false;
                data->scroll_mode   = false;
                data->scroll_axis   = AXIS_UNKNOWN;
                data->candidate_axis = AXIS_UNKNOWN;
                data->angle_accum   = 0.0f;
                data->enter_count   = 0;
                data->circ_sign     = 0.0f;
                data->have_abs_x    = false;
                data->have_abs_y    = false;
                data->start_decided = false;
                data->start_radial_x = 0.0f;
                data->start_radial_y = 0.0f;
                data->start_tangent_x = 0.0f;
                data->start_tangent_y = 0.0f;
                data->decision_dx = 0;
                data->decision_dy = 0;
            }
            return ZMK_INPUT_PROC_CONTINUE;

        default:
            return ZMK_INPUT_PROC_CONTINUE;
        }
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->curr_dx = event->value;
        data->have_x  = true;
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
            return ZMK_INPUT_PROC_STOP;
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

static int circular_scroll_init(const struct device *dev) {
    circ_reset(dev->data);
    return 0;
}

static const struct zmk_input_processor_driver_api circular_scroll_api = {
    .handle_event = circular_scroll_handle_event,
};

#define CIRC_SCROLL_INST(n)                                                    \
    static struct circular_scroll_data circ_scroll_data_##n = {};              \
    static const struct circular_scroll_config circ_scroll_cfg_##n = {         \
        .pressure_threshold    = DT_INST_PROP(n, pressure_threshold),          \
        .x_max                 = DT_INST_PROP(n, x_max),                       \
        .y_max                 = DT_INST_PROP(n, y_max),                       \
        .inner_ring_pct        = DT_INST_PROP(n, inner_ring_pct),              \
        .sector_half_angle_deg = DT_INST_PROP(n, sector_half_angle_deg),       \
        .activation_angle_deg  = DT_INST_PROP(n, activation_angle_deg),        \
        .scroll_angle_deg      = DT_INST_PROP(n, scroll_angle_deg),            \
        .invert_vertical       = DT_INST_PROP(n, invert_vertical),             \
        .invert_horizontal     = DT_INST_PROP(n, invert_horizontal),           \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, circular_scroll_init, NULL,                       \
                          &circ_scroll_data_##n, &circ_scroll_cfg_##n,         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                          &circular_scroll_api);

DT_INST_FOREACH_STATUS_OKAY(CIRC_SCROLL_INST)
