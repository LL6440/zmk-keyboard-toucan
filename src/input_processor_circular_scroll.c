/*
 * SPDX-License-Identifier: MIT
 *
 * Circular scroll with gesture lock.
 *
 * Behavior:
 *  - touch must start near 3h/9h (vertical candidate) or 6h/12h (horizontal candidate)
 *  - first meaningful motion decides:
 *      tangential motion => lock scroll on candidate axis
 *      radial motion     => lock normal pointer
 *  - once locked, mode never changes until finger lift
 *
 * Scroll emission intentionally keeps the proven vector-delta accumulation path
 * from the earlier working version, because it reacted correctly on hardware.
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
#define DECISION_SPEED 10
#define DECISION_MARGIN 4.0f

enum circular_scroll_mode {
    CIRCULAR_SCROLL_MODE_NONE = 0,
    CIRCULAR_SCROLL_MODE_POINTER,
    CIRCULAR_SCROLL_MODE_CANDIDATE_VERTICAL,
    CIRCULAR_SCROLL_MODE_CANDIDATE_HORIZONTAL,
    CIRCULAR_SCROLL_MODE_SCROLL_VERTICAL,
    CIRCULAR_SCROLL_MODE_SCROLL_HORIZONTAL,
};

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
    int32_t x;
    int32_t y;
    int32_t z;

    bool touch_active;
    bool start_ready;
    bool have_prev_vec;
    bool captured;
    bool fresh_x;
    bool fresh_y;
    bool have_x;
    bool have_y;
    enum circular_scroll_mode mode;

    int32_t start_x;
    int32_t start_y;
    float start_radial_x;
    float start_radial_y;
    float start_tangent_x;
    float start_tangent_y;

    int32_t prev_dx;
    int32_t prev_dy;
    float angle_accum_deg;
};

static void circular_scroll_reset(struct circular_scroll_data *data) {
    data->z = 0;
    data->touch_active = false;
    data->start_ready = false;
    data->have_prev_vec = false;
    data->captured = false;
    data->fresh_x = false;
    data->fresh_y = false;
    data->have_x = false;
    data->have_y = false;
    data->mode = CIRCULAR_SCROLL_MODE_NONE;
    data->start_x = 0;
    data->start_y = 0;
    data->start_radial_x = 0.0f;
    data->start_radial_y = 0.0f;
    data->start_tangent_x = 0.0f;
    data->start_tangent_y = 0.0f;
    data->prev_dx = 0;
    data->prev_dy = 0;
    data->angle_accum_deg = 0.0f;
}

static inline float absf_local(float v) { return v < 0 ? -v : v; }

static bool point_in_outer_ring(const struct circular_scroll_config *cfg, int32_t dx, int32_t dy) {
    float rx = (float)cfg->x_max * 0.5f;
    float ry = (float)cfg->y_max * 0.5f;
    float x = (float)dx;
    float y = (float)dy;
    float norm = (x * x) / (rx * rx) + (y * y) / (ry * ry);
    float inner = (float)cfg->inner_ring_pct / 100.0f;
    return norm >= (inner * inner);
}

static float angle_diff_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f) {
        d -= 360.0f;
    }
    while (d < -180.0f) {
        d += 360.0f;
    }
    return d;
}

static float point_angle_deg(int32_t dx, int32_t dy) {
    return atan2f((float)dy, (float)dx) * RAD_TO_DEG;
}

static enum circular_scroll_mode choose_start_mode(const struct circular_scroll_config *cfg,
                                                   int32_t dx, int32_t dy) {
    if (!point_in_outer_ring(cfg, dx, dy)) {
        return CIRCULAR_SCROLL_MODE_POINTER;
    }

    float angle = atan2f((float)dy, (float)dx) * RAD_TO_DEG;
    float sector = (float)cfg->sector_half_angle_deg;

    if (absf_local(angle_diff_deg(angle, 0.0f)) <= sector ||
        absf_local(angle_diff_deg(angle, 180.0f)) <= sector) {
        return CIRCULAR_SCROLL_MODE_CANDIDATE_VERTICAL;
    }

    if (absf_local(angle_diff_deg(angle, 90.0f)) <= sector ||
        absf_local(angle_diff_deg(angle, -90.0f)) <= sector) {
        return CIRCULAR_SCROLL_MODE_CANDIDATE_HORIZONTAL;
    }

    return CIRCULAR_SCROLL_MODE_POINTER;
}

static void circular_scroll_prepare_start(const struct circular_scroll_config *cfg,
                                          struct circular_scroll_data *data) {
    int32_t center_x = (int32_t)cfg->x_max / 2;
    int32_t center_y = (int32_t)cfg->y_max / 2;
    int32_t dx = data->x - center_x;
    int32_t dy = data->y - center_y;

    data->mode = choose_start_mode(cfg, dx, dy);
    data->start_ready = true;
    data->start_x = data->x;
    data->start_y = data->y;
    data->have_prev_vec = true;
    data->prev_dx = dx;
    data->prev_dy = dy;
    data->angle_accum_deg = 0.0f;
    data->captured = false;

    float len = sqrtf((float)dx * (float)dx + (float)dy * (float)dy);
    if (len > 0.0f) {
        data->start_radial_x = (float)dx / len;
        data->start_radial_y = (float)dy / len;
        data->start_tangent_x = -data->start_radial_y;
        data->start_tangent_y = data->start_radial_x;
    } else {
        data->start_radial_x = 0.0f;
        data->start_radial_y = 0.0f;
        data->start_tangent_x = 0.0f;
        data->start_tangent_y = 0.0f;
    }
}

static float delta_angle_deg(int32_t prev_dx, int32_t prev_dy, int32_t dx, int32_t dy) {
    float cross = (float)prev_dx * (float)dy - (float)prev_dy * (float)dx;
    float dot = (float)prev_dx * (float)dx + (float)prev_dy * (float)dy;
    return atan2f(cross, dot) * RAD_TO_DEG;
}

static bool maybe_lock_candidate(struct circular_scroll_data *data, int32_t dx, int32_t dy) {
    if (data->mode != CIRCULAR_SCROLL_MODE_CANDIDATE_VERTICAL &&
        data->mode != CIRCULAR_SCROLL_MODE_CANDIDATE_HORIZONTAL) {
        return true;
    }

    float total_dx = (float)(data->x - data->start_x);
    float total_dy = (float)(data->y - data->start_y);
    float tangential = absf_local(total_dx * data->start_tangent_x +
                                  total_dy * data->start_tangent_y);
    float radial = absf_local(total_dx * data->start_radial_x +
                              total_dy * data->start_radial_y);
    int total_speed = (int)(absf_local(total_dx) + absf_local(total_dy));
    if (total_speed < DECISION_SPEED) {
        return false;
    }

    if ((tangential - radial) >= DECISION_MARGIN) {
        if (data->mode == CIRCULAR_SCROLL_MODE_CANDIDATE_VERTICAL) {
            data->mode = CIRCULAR_SCROLL_MODE_SCROLL_VERTICAL;
            LOG_DBG("cscroll lock: vertical");
        } else {
            data->mode = CIRCULAR_SCROLL_MODE_SCROLL_HORIZONTAL;
            LOG_DBG("cscroll lock: horizontal");
        }
        data->captured = false;
        data->angle_accum_deg = 0.0f;
        data->prev_dx = dx;
        data->prev_dy = dy;
        data->have_prev_vec = true;
        return false;
    }

    if ((radial - tangential) >= DECISION_MARGIN) {
        data->mode = CIRCULAR_SCROLL_MODE_POINTER;
        LOG_DBG("cscroll lock: pointer");
        return true;
    }

    return false;
}

static int circular_scroll_handle_motion(const struct device *dev, struct input_event *event,
                                         const struct circular_scroll_config *cfg,
                                         struct circular_scroll_data *data) {
    int32_t center_x = (int32_t)cfg->x_max / 2;
    int32_t center_y = (int32_t)cfg->y_max / 2;
    int32_t dx = data->x - center_x;
    int32_t dy = data->y - center_y;

    if (!data->touch_active) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->start_ready) {
        if (!data->fresh_x || !data->fresh_y) {
            return ZMK_INPUT_PROC_STOP;
        }

        circular_scroll_prepare_start(cfg, data);

        if (data->mode == CIRCULAR_SCROLL_MODE_POINTER) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        return ZMK_INPUT_PROC_STOP;
    }

    if (data->mode == CIRCULAR_SCROLL_MODE_POINTER || data->mode == CIRCULAR_SCROLL_MODE_NONE) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->mode == CIRCULAR_SCROLL_MODE_CANDIDATE_VERTICAL ||
        data->mode == CIRCULAR_SCROLL_MODE_CANDIDATE_HORIZONTAL) {
        if (maybe_lock_candidate(data, dx, dy)) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (data->mode == CIRCULAR_SCROLL_MODE_POINTER) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (data->mode == CIRCULAR_SCROLL_MODE_CANDIDATE_VERTICAL ||
            data->mode == CIRCULAR_SCROLL_MODE_CANDIDATE_HORIZONTAL) {
            return ZMK_INPUT_PROC_STOP;
        }

        return ZMK_INPUT_PROC_STOP;
    }

    if (!data->have_prev_vec) {
        data->have_prev_vec = true;
        data->prev_dx = dx;
        data->prev_dy = dy;
        return ZMK_INPUT_PROC_STOP;
    }

    float delta = delta_angle_deg(data->prev_dx, data->prev_dy, dx, dy);
    data->prev_dx = dx;
    data->prev_dy = dy;
    data->angle_accum_deg += delta;

    if (!data->captured) {
        if (absf_local(data->angle_accum_deg) < (float)MAX(cfg->activation_angle_deg, 1)) {
            return ZMK_INPUT_PROC_STOP;
        }
        data->captured = true;
    }

    int32_t step = 0;
    int32_t threshold = MAX(cfg->scroll_angle_deg, 1);

    if (absf_local(data->angle_accum_deg) >= (float)threshold) {
        step = (int32_t)(data->angle_accum_deg / (float)threshold);
        data->angle_accum_deg -= (float)step * (float)threshold;
    }

    if (step == 0) {
        return ZMK_INPUT_PROC_STOP;
    }

    event->type = INPUT_EV_REL;
    if (data->mode == CIRCULAR_SCROLL_MODE_SCROLL_VERTICAL) {
        if (cfg->invert_vertical) {
            step = -step;
        }
        event->code = INPUT_REL_WHEEL;
        event->value = step;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (cfg->invert_horizontal) {
        step = -step;
    }
    event->code = INPUT_REL_HWHEEL;
    event->value = step;
    return ZMK_INPUT_PROC_CONTINUE;
}

static int circular_scroll_handle_event(const struct device *dev, struct input_event *event,
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
    case INPUT_ABS_X:
        data->x = event->value;
        data->have_x = true;
        if (data->touch_active && !data->start_ready) {
            data->fresh_x = true;
        }
        return circular_scroll_handle_motion(dev, event, cfg, data);

    case INPUT_ABS_Y:
        data->y = event->value;
        data->have_y = true;
        if (data->touch_active && !data->start_ready) {
            data->fresh_y = true;
        }
        return circular_scroll_handle_motion(dev, event, cfg, data);

    case INPUT_ABS_Z:
        data->z = event->value;

        if (data->z < cfg->pressure_threshold) {
            circular_scroll_reset(data);
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (!data->touch_active) {
            data->touch_active = true;
            data->start_ready = false;
            data->have_prev_vec = false;
            data->captured = false;
            data->fresh_x = data->have_x;
            data->fresh_y = data->have_y;
            data->mode = CIRCULAR_SCROLL_MODE_NONE;
            data->angle_accum_deg = 0.0f;

            if (data->fresh_x && data->fresh_y) {
                circular_scroll_prepare_start(cfg, data);
            }
        }

        if (data->mode == CIRCULAR_SCROLL_MODE_POINTER ||
            data->mode == CIRCULAR_SCROLL_MODE_NONE) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        return ZMK_INPUT_PROC_STOP;

    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }
}

static int circular_scroll_init(const struct device *dev) {
    struct circular_scroll_data *data = dev->data;
    circular_scroll_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api circular_scroll_driver_api = {
    .handle_event = circular_scroll_handle_event,
};

#define CIRCULAR_SCROLL_INST(n)                                                              \
    static struct circular_scroll_data circular_scroll_data_##n = {};                        \
    static const struct circular_scroll_config circular_scroll_config_##n = {                \
        .pressure_threshold = DT_INST_PROP(n, pressure_threshold),                           \
        .x_max = DT_INST_PROP(n, x_max),                                                     \
        .y_max = DT_INST_PROP(n, y_max),                                                     \
        .inner_ring_pct = DT_INST_PROP(n, inner_ring_pct),                                   \
        .sector_half_angle_deg = DT_INST_PROP(n, sector_half_angle_deg),                     \
        .activation_angle_deg = DT_INST_PROP(n, activation_angle_deg),                       \
        .scroll_angle_deg = DT_INST_PROP(n, scroll_angle_deg),                               \
        .invert_vertical = DT_INST_PROP(n, invert_vertical),                                 \
        .invert_horizontal = DT_INST_PROP(n, invert_horizontal),                             \
    };                                                                                       \
    DEVICE_DT_INST_DEFINE(n, circular_scroll_init, NULL, &circular_scroll_data_##n,         \
                          &circular_scroll_config_##n, POST_KERNEL,                          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &circular_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CIRCULAR_SCROLL_INST)
