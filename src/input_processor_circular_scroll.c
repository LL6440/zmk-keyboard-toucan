/*
 * SPDX-License-Identifier: MIT
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
#define CIRC_RELEASE_DEBOUNCE 3

enum circular_scroll_mode {
    CIRCULAR_SCROLL_MODE_NONE = 0,
    CIRCULAR_SCROLL_MODE_VERTICAL,
    CIRCULAR_SCROLL_MODE_HORIZONTAL,
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
    enum circular_scroll_mode mode;

    int32_t prev_dx;
    int32_t prev_dy;
    float angle_accum_deg;
    uint8_t release_debounce;
};

static void circular_scroll_reset(struct circular_scroll_data *data) {
    data->z = 0;
    data->touch_active = false;
    data->start_ready = false;
    data->have_prev_vec = false;
    data->captured = false;
    data->mode = CIRCULAR_SCROLL_MODE_NONE;
    data->prev_dx = 0;
    data->prev_dy = 0;
    data->angle_accum_deg = 0.0f;
    data->release_debounce = 0;
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

static enum circular_scroll_mode choose_start_mode(const struct circular_scroll_config *cfg,
                                                   int32_t dx, int32_t dy) {
    if (!point_in_outer_ring(cfg, dx, dy)) {
        return CIRCULAR_SCROLL_MODE_NONE;
    }

    float angle = atan2f((float)dy, (float)dx) * RAD_TO_DEG;
    float sector = (float)cfg->sector_half_angle_deg;

    if (absf_local(angle) <= sector) {
        return CIRCULAR_SCROLL_MODE_VERTICAL; /* 3h */
    }

    if (absf_local(angle - 90.0f) <= sector) {
        return CIRCULAR_SCROLL_MODE_HORIZONTAL; /* 6h */
    }

    return CIRCULAR_SCROLL_MODE_NONE;
}

static float delta_angle_deg(int32_t prev_dx, int32_t prev_dy, int32_t dx, int32_t dy) {
    float cross = (float)prev_dx * (float)dy - (float)prev_dy * (float)dx;
    float dot = (float)prev_dx * (float)dx + (float)prev_dy * (float)dy;
    return atan2f(cross, dot) * RAD_TO_DEG;
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
        data->mode = choose_start_mode(cfg, dx, dy);
        data->start_ready = true;
        data->have_prev_vec = true;
        data->prev_dx = dx;
        data->prev_dy = dy;
        data->angle_accum_deg = 0.0f;

        if (data->mode == CIRCULAR_SCROLL_MODE_NONE) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        return ZMK_INPUT_PROC_STOP;
    }

    if (data->mode == CIRCULAR_SCROLL_MODE_NONE) {
        return ZMK_INPUT_PROC_CONTINUE;
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

    if (data->mode == CIRCULAR_SCROLL_MODE_VERTICAL) {
        if (cfg->invert_vertical) {
            step = -step;
        }
        event->type = INPUT_EV_REL;
        event->code = INPUT_REL_WHEEL;
        event->value = step;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (cfg->invert_horizontal) {
        step = -step;
    }
    event->type = INPUT_EV_REL;
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
        return circular_scroll_handle_motion(dev, event, cfg, data);

    case INPUT_ABS_Y:
        data->y = event->value;
        return circular_scroll_handle_motion(dev, event, cfg, data);

    case INPUT_ABS_Z:
        data->z = event->value;

        if (data->z < cfg->pressure_threshold) {
            if (data->touch_active && data->release_debounce < CIRC_RELEASE_DEBOUNCE) {
                data->release_debounce++;
            }

            if (data->touch_active && data->release_debounce < CIRC_RELEASE_DEBOUNCE) {
                return (data->mode == CIRCULAR_SCROLL_MODE_NONE)
                           ? ZMK_INPUT_PROC_CONTINUE
                           : ZMK_INPUT_PROC_STOP;
            }

            circular_scroll_reset(data);
            return ZMK_INPUT_PROC_CONTINUE;
        }

        data->release_debounce = 0;

        if (!data->touch_active) {
            data->touch_active = true;
            data->start_ready = false;
            data->have_prev_vec = false;
            data->captured = false;
            data->mode = CIRCULAR_SCROLL_MODE_NONE;
            data->angle_accum_deg = 0.0f;
            data->release_debounce = 0;
        }

        if (data->mode == CIRCULAR_SCROLL_MODE_NONE) {
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
    DEVICE_DT_INST_DEFINE(n, circular_scroll_init, NULL, &circular_scroll_data_##n,          \
                          &circular_scroll_config_##n, POST_KERNEL,                          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &circular_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CIRCULAR_SCROLL_INST)
