/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_edge_scroll

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum edge_scroll_mode {
    EDGE_SCROLL_NONE = 0,
    EDGE_SCROLL_VERTICAL,
    EDGE_SCROLL_HORIZONTAL,
};

struct edge_scroll_config {
    uint16_t right_zone_pct;
    uint16_t top_zone_pct;
    uint16_t pressure_threshold;
    uint16_t scroll_div;
    uint16_t x_max;
    uint16_t y_max;
    bool invert_vertical;
    bool invert_horizontal;
};

struct edge_scroll_data {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t last_x;
    int32_t last_y;
    int32_t rem;
    bool touch_active;
    enum edge_scroll_mode mode;
};

static void edge_scroll_reset(struct edge_scroll_data *data) {
    data->z = 0;
    data->last_x = data->x;
    data->last_y = data->y;
    data->rem = 0;
    data->touch_active = false;
    data->mode = EDGE_SCROLL_NONE;
}

static inline int32_t pct_value(uint32_t max_value, uint32_t pct) {
    return (int32_t)((max_value * pct) / 100U);
}

static int edge_scroll_handle_event(const struct device *dev, struct input_event *event,
                                    uint32_t param1, uint32_t param2,
                                    struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct edge_scroll_config *cfg = dev->config;
    struct edge_scroll_data *data = dev->data;

    if (event->type != INPUT_EV_ABS) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    switch (event->code) {
    case INPUT_ABS_X:
        data->x = event->value;

        if (!data->touch_active) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (data->mode != EDGE_SCROLL_HORIZONTAL) {
            return data->mode == EDGE_SCROLL_NONE ? ZMK_INPUT_PROC_CONTINUE : ZMK_INPUT_PROC_STOP;
        }

        data->rem += data->x - data->last_x;
        data->last_x = data->x;

        {
            int32_t step = data->rem / (int32_t)MAX(cfg->scroll_div, 1);
            data->rem -= step * (int32_t)MAX(cfg->scroll_div, 1);

            if (cfg->invert_horizontal) {
                step = -step;
            }

            if (step == 0) {
                return ZMK_INPUT_PROC_STOP;
            }

            event->code = INPUT_REL_HWHEEL;
            event->value = step;
            event->type = INPUT_EV_REL;
            return ZMK_INPUT_PROC_CONTINUE;
        }

    case INPUT_ABS_Y:
        data->y = event->value;

        if (!data->touch_active) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (data->mode != EDGE_SCROLL_VERTICAL) {
            return data->mode == EDGE_SCROLL_NONE ? ZMK_INPUT_PROC_CONTINUE : ZMK_INPUT_PROC_STOP;
        }

        data->rem += data->y - data->last_y;
        data->last_y = data->y;

        {
            int32_t step = data->rem / (int32_t)MAX(cfg->scroll_div, 1);
            data->rem -= step * (int32_t)MAX(cfg->scroll_div, 1);

            if (cfg->invert_vertical) {
                step = -step;
            }

            if (step == 0) {
                return ZMK_INPUT_PROC_STOP;
            }

            event->code = INPUT_REL_WHEEL;
            event->value = step;
            event->type = INPUT_EV_REL;
            return ZMK_INPUT_PROC_CONTINUE;
        }

    case INPUT_ABS_Z: {
        int32_t prev_z = data->z;
        data->z = event->value;

        if (data->z < cfg->pressure_threshold) {
            edge_scroll_reset(data);
            return ZMK_INPUT_PROC_STOP;
        }

        if (!data->touch_active && prev_z < cfg->pressure_threshold) {
            int32_t right_start = (int32_t)cfg->x_max - pct_value(cfg->x_max, cfg->right_zone_pct);
            int32_t top_end = pct_value(cfg->y_max, cfg->top_zone_pct);

            data->touch_active = true;
            data->last_x = data->x;
            data->last_y = data->y;
            data->rem = 0;

            if (data->x >= right_start) {
                data->mode = EDGE_SCROLL_VERTICAL;
            } else if (data->y <= top_end) {
                data->mode = EDGE_SCROLL_HORIZONTAL;
            } else {
                data->mode = EDGE_SCROLL_NONE;
            }
        }

        return data->mode == EDGE_SCROLL_NONE ? ZMK_INPUT_PROC_CONTINUE : ZMK_INPUT_PROC_STOP;
    }

    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }
}

static int edge_scroll_init(const struct device *dev) {
    struct edge_scroll_data *data = dev->data;

    edge_scroll_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api edge_scroll_driver_api = {
    .handle_event = edge_scroll_handle_event,
};

#define EDGE_SCROLL_INST(n)                                                                       \
    static struct edge_scroll_data edge_scroll_data_##n = {};                                     \
    static const struct edge_scroll_config edge_scroll_config_##n = {                             \
        .right_zone_pct = DT_INST_PROP(n, right_zone_pct),                                        \
        .top_zone_pct = DT_INST_PROP(n, top_zone_pct),                                            \
        .pressure_threshold = DT_INST_PROP(n, pressure_threshold),                                \
        .scroll_div = DT_INST_PROP(n, scroll_div),                                                \
        .x_max = DT_INST_PROP(n, x_max),                                                          \
        .y_max = DT_INST_PROP(n, y_max),                                                          \
        .invert_vertical = DT_INST_PROP(n, invert_vertical),                                      \
        .invert_horizontal = DT_INST_PROP(n, invert_horizontal),                                  \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, edge_scroll_init, NULL, &edge_scroll_data_##n,                       \
                          &edge_scroll_config_##n, POST_KERNEL,                                   \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &edge_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EDGE_SCROLL_INST)
