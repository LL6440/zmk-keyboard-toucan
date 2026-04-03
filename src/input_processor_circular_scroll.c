/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll input processor — ABS-angle based.
 *
 * Gesture selection:
 *   1) The touch must START near a cardinal point on the outer ring.
 *      - near 3h or 9h  => candidate vertical scroll
 *      - near 6h or 12h => candidate horizontal scroll
 *   2) The FIRST meaningful motion decides the gesture for the whole contact.
 *      - tangential first motion => lock scroll on candidate axis
 *      - radial / non-ring first motion => lock normal pointer
 *
 * Once a contact is classified, it stays in that mode until finger lift.
 *
 * In scroll mode, scroll ticks are computed from ABS angular motion around the
 * pad center. This is much more stable than REL-pair heuristics when the user
 * slows down or briefly pauses without lifting the finger.
 */

#define DT_DRV_COMPAT zmk_input_processor_circular_scroll

#include <drivers/input_processor.h>
#include <stdbool.h>
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
#define DEFAULT_SCROLL_DEG  12
#define DECISION_SPEED      10
#define DECISION_MARGIN      4.0f

enum circ_axis { AXIS_UNKNOWN = 0, AXIS_VERTICAL, AXIS_HORIZONTAL };
enum gesture_mode { GESTURE_UNDECIDED = 0, GESTURE_POINTER, GESTURE_SCROLL };

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
    bool touch_active;

    int32_t abs_x;
    int32_t abs_y;
    bool    have_abs_x;
    bool    have_abs_y;

    int32_t start_x;
    int32_t start_y;
    bool    have_start_point;

    enum gesture_mode mode;
    enum circ_axis    scroll_axis;
    enum circ_axis    candidate_axis;

    float start_radial_x;
    float start_radial_y;
    float start_tangent_x;
    float start_tangent_y;

    float last_angle_deg;
    bool  have_last_angle;
    float angle_accum;
};

static void circ_reset(struct circular_scroll_data *d) {
    d->touch_active = false;
    d->abs_x = 0;
    d->abs_y = 0;
    d->have_abs_x = false;
    d->have_abs_y = false;
    d->start_x = 0;
    d->start_y = 0;
    d->have_start_point = false;
    d->mode = GESTURE_UNDECIDED;
    d->scroll_axis = AXIS_UNKNOWN;
    d->candidate_axis = AXIS_UNKNOWN;
    d->start_radial_x = 0.0f;
    d->start_radial_y = 0.0f;
    d->start_tangent_x = 0.0f;
    d->start_tangent_y = 0.0f;
    d->last_angle_deg = 0.0f;
    d->have_last_angle = false;
    d->angle_accum = 0.0f;
}

static float smallest_angle_delta_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static float current_angle_deg(const struct circular_scroll_config *cfg,
                               int32_t x, int32_t y) {
    float cx = (float)cfg->x_max / 2.0f;
    float cy = (float)cfg->y_max / 2.0f;
    float dx = (float)x - cx;
    float dy_up = cy - (float)y; /* screen-down -> math-up */
    return atan2f(dy_up, dx) * RAD_TO_DEG;
}

static enum circ_axis configure_start_sector(const struct circular_scroll_config *cfg,
                                             struct circular_scroll_data *d,
                                             int32_t x, int32_t y) {
    float cx = (float)cfg->x_max / 2.0f;
    float cy = (float)cfg->y_max / 2.0f;
    float dx_screen = (float)x - cx;
    float dy_screen = (float)y - cy;
    float dy_up = -dy_screen;

    float r = sqrtf(dx_screen * dx_screen + dy_screen * dy_screen);
    float min_half = ((float)MIN(cfg->x_max, cfg->y_max)) / 2.0f;
    float min_r = ((float)cfg->inner_ring_pct / 100.0f) * min_half;
    if (r < min_r) {
        return AXIS_UNKNOWN;
    }

    d->start_radial_x = dx_screen / r;
    d->start_radial_y = dy_screen / r;
    d->start_tangent_x = -d->start_radial_y;
    d->start_tangent_y = d->start_radial_x;

    float ang = atan2f(dy_up, dx_screen) * RAD_TO_DEG;
    float half = (float)cfg->sector_half_angle_deg;

    if (fabsf(smallest_angle_delta_deg(ang, 0.0f)) <= half ||
        fabsf(smallest_angle_delta_deg(ang, 180.0f)) <= half ||
        fabsf(smallest_angle_delta_deg(ang, -180.0f)) <= half) {
        return AXIS_VERTICAL; /* 3h or 9h */
    }

    if (fabsf(smallest_angle_delta_deg(ang, -90.0f)) <= half ||
        fabsf(smallest_angle_delta_deg(ang, 90.0f)) <= half) {
        return AXIS_HORIZONTAL; /* 6h or 12h */
    }

    return AXIS_UNKNOWN;
}

static bool maybe_lock_gesture(struct circular_scroll_data *d) {
    if (!d->have_start_point || !d->have_abs_x || !d->have_abs_y) {
        return false;
    }

    if (d->mode != GESTURE_UNDECIDED) {
        return true;
    }

    if (d->candidate_axis == AXIS_UNKNOWN) {
        d->mode = GESTURE_POINTER;
        return true;
    }

    float total_dx = (float)(d->abs_x - d->start_x);
    float total_dy = (float)(d->abs_y - d->start_y);
    int total_speed = IABS((int32_t)total_dx) + IABS((int32_t)total_dy);
    if (total_speed < DECISION_SPEED) {
        return false;
    }

    float tangential = fabsf(total_dx * d->start_tangent_x + total_dy * d->start_tangent_y);
    float radial = fabsf(total_dx * d->start_radial_x + total_dy * d->start_radial_y);

    if ((tangential - radial) >= DECISION_MARGIN) {
        d->mode = GESTURE_SCROLL;
        d->scroll_axis = d->candidate_axis;
        d->have_last_angle = false;
        d->angle_accum = 0.0f;
        return true;
    }

    if ((radial - tangential) >= DECISION_MARGIN) {
        d->mode = GESTURE_POINTER;
        return true;
    }

    return false;
}

static int circular_scroll_handle_event(const struct device *dev,
                                        struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct circular_scroll_config *cfg = dev->config;
    struct circular_scroll_data *data = dev->data;

    if (event->type != INPUT_EV_ABS) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    switch (event->code) {
    case INPUT_ABS_Z:
        if (event->value < cfg->pressure_threshold) {
            circ_reset(data);
        } else if (!data->touch_active) {
            data->touch_active = true;
            data->mode = GESTURE_UNDECIDED;
            data->scroll_axis = AXIS_UNKNOWN;
            data->candidate_axis = AXIS_UNKNOWN;
            data->have_start_point = false;
            data->have_last_angle = false;
            data->angle_accum = 0.0f;
        }
        return ZMK_INPUT_PROC_CONTINUE;

    case INPUT_ABS_X:
        data->abs_x = event->value;
        data->have_abs_x = true;
        break;

    case INPUT_ABS_Y:
        data->abs_y = event->value;
        data->have_abs_y = true;
        break;

    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->touch_active || !data->have_abs_x || !data->have_abs_y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->have_start_point) {
        data->start_x = data->abs_x;
        data->start_y = data->abs_y;
        data->have_start_point = true;
        data->candidate_axis = configure_start_sector(cfg, data, data->start_x, data->start_y);

        if (data->candidate_axis == AXIS_UNKNOWN) {
            data->mode = GESTURE_POINTER;
            return ZMK_INPUT_PROC_CONTINUE;
        }

        /* Candidate scroll gesture: hold events until first motion decides. */
        return ZMK_INPUT_PROC_STOP;
    }

    if (data->mode == GESTURE_UNDECIDED) {
        if (!maybe_lock_gesture(data)) {
            return ZMK_INPUT_PROC_STOP;
        }

        if (data->mode == GESTURE_POINTER) {
            return ZMK_INPUT_PROC_CONTINUE;
        }
    }

    if (data->mode == GESTURE_POINTER) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Scroll mode: swallow ABS pointer events and emit wheel from angular delta. */
    float ang = current_angle_deg(cfg, data->abs_x, data->abs_y);
    if (!data->have_last_angle) {
        data->last_angle_deg = ang;
        data->have_last_angle = true;
        return ZMK_INPUT_PROC_STOP;
    }

    float delta = smallest_angle_delta_deg(ang, data->last_angle_deg);
    data->last_angle_deg = ang;
    data->angle_accum += delta;

    int32_t scroll_deg = (cfg->scroll_angle_deg > 0)
                             ? cfg->scroll_angle_deg
                             : DEFAULT_SCROLL_DEG;

    if (IABS((int32_t)data->angle_accum) < scroll_deg) {
        return ZMK_INPUT_PROC_STOP;
    }

    int32_t ticks = (int32_t)(data->angle_accum / (float)scroll_deg);
    data->angle_accum -= (float)ticks * (float)scroll_deg;

    if (ticks == 0) {
        return ZMK_INPUT_PROC_STOP;
    }

    event->type = INPUT_EV_REL;
    if (data->scroll_axis == AXIS_VERTICAL) {
        if (cfg->invert_vertical) {
            ticks = -ticks;
        }
        event->code = INPUT_REL_WHEEL;
        event->value = ticks;
    } else {
        if (cfg->invert_horizontal) {
            ticks = -ticks;
        }
        event->code = INPUT_REL_HWHEEL;
        event->value = ticks;
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
