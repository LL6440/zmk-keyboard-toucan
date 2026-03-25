/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_absolute_rel_pointer

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct abs_rel_pointer_config {
    uint16_t pressure_threshold;
    uint16_t x_div;
    uint16_t y_div;
    uint16_t max_rel;
    bool invert_x;
    bool invert_y;
};

struct abs_rel_pointer_data {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t last_x;
    int32_t last_y;
    int32_t rem_x;
    int32_t rem_y;
    bool touch_active;
    bool x_anchored;
    bool y_anchored;
};

static void abs_rel_pointer_reset(struct abs_rel_pointer_data *data) {
    data->z = 0;
    data->last_x = data->x;
    data->last_y = data->y;
    data->rem_x = 0;
    data->rem_y = 0;
    data->touch_active = false;
    data->x_anchored = false;
    data->y_anchored = false;
}

static int32_t clamp_step(int32_t value, int32_t max_abs) {
    if (value > max_abs) {
        return max_abs;
    }
    if (value < -max_abs) {
        return -max_abs;
    }
    return value;
}

static int abs_rel_pointer_handle_event(const struct device *dev, struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct abs_rel_pointer_config *cfg = dev->config;
    struct abs_rel_pointer_data *data = dev->data;

    if (event->type != INPUT_EV_ABS) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    switch (event->code) {
    case INPUT_ABS_X: {
        data->x = event->value;
        LOG_INF("PAD X=%d Z=%d", data->x, data->z);

        if (!data->touch_active) {
            return ZMK_INPUT_PROC_STOP;
        }

        if (!data->x_anchored) {
            data->last_x = data->x;
            data->x_anchored = true;
            return ZMK_INPUT_PROC_STOP;
        }

        data->rem_x += data->x - data->last_x;
        data->last_x = data->x;

        int32_t step = data->rem_x / (int32_t)MAX(cfg->x_div, 1);
        data->rem_x -= step * (int32_t)MAX(cfg->x_div, 1);

        if (cfg->invert_x) {
            step = -step;
        }

        step = clamp_step(step, cfg->max_rel);

        if (step == 0) {
            return ZMK_INPUT_PROC_STOP;
        }

        event->code = INPUT_REL_X;
        event->value = step;
        event->type = INPUT_EV_REL;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    case INPUT_ABS_Y: {
        data->y = event->value;
        LOG_INF("PAD Y=%d Z=%d", data->y, data->z);

        if (!data->touch_active) {
            return ZMK_INPUT_PROC_STOP;
        }

        if (!data->y_anchored) {
            data->last_y = data->y;
            data->y_anchored = true;
            return ZMK_INPUT_PROC_STOP;
        }

        data->rem_y += data->y - data->last_y;
        data->last_y = data->y;

        int32_t step = data->rem_y / (int32_t)MAX(cfg->y_div, 1);
        data->rem_y -= step * (int32_t)MAX(cfg->y_div, 1);

        if (cfg->invert_y) {
            step = -step;
        }

        step = clamp_step(step, cfg->max_rel);

        if (step == 0) {
            return ZMK_INPUT_PROC_STOP;
        }

        event->code = INPUT_REL_Y;
        event->value = step;
        event->type = INPUT_EV_REL;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    case INPUT_ABS_Z:
        data->z = event->value;
        LOG_INF("PAD Z=%d X=%d Y=%d", data->z, data->x, data->y);

        if (data->z < cfg->pressure_threshold) {
            abs_rel_pointer_reset(data);
            return ZMK_INPUT_PROC_STOP;
        }

        if (!data->touch_active) {
            data->touch_active = true;
            data->x_anchored = false;
            data->y_anchored = false;
            data->rem_x = 0;
            data->rem_y = 0;
        }

        return ZMK_INPUT_PROC_STOP;

    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }
}

static int abs_rel_pointer_init(const struct device *dev) {
    struct abs_rel_pointer_data *data = dev->data;

    abs_rel_pointer_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api abs_rel_pointer_driver_api = {
    .handle_event = abs_rel_pointer_handle_event,
};

#define ABS_REL_POINTER_INST(n)                                                                  \
    static struct abs_rel_pointer_data abs_rel_pointer_data_##n = {};                            \
    static const struct abs_rel_pointer_config abs_rel_pointer_config_##n = {                    \
        .pressure_threshold = DT_INST_PROP(n, pressure_threshold),                               \
        .x_div = DT_INST_PROP(n, x_div),                                                         \
        .y_div = DT_INST_PROP(n, y_div),                                                         \
        .max_rel = DT_INST_PROP(n, max_rel),                                                     \
        .invert_x = DT_INST_PROP(n, invert_x),                                                   \
        .invert_y = DT_INST_PROP(n, invert_y),                                                   \
    };                                                                                           \
    DEVICE_DT_INST_DEFINE(n, abs_rel_pointer_init, NULL, &abs_rel_pointer_data_##n,              \
                          &abs_rel_pointer_config_##n, POST_KERNEL,                              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &abs_rel_pointer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ABS_REL_POINTER_INST)
