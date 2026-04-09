/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_touch_tap

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct touch_tap_config {
    uint16_t pressure_threshold;
    uint16_t max_tap_ms;
    uint16_t move_threshold;
};

struct touch_tap_data {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t start_x;
    int32_t start_y;
    int64_t down_ts;
    bool have_x;
    bool have_y;
    bool start_valid;
    bool touch_active;
    bool moved;
};

static void touch_tap_reset(struct touch_tap_data *data) {
    data->z = 0;
    data->start_x = 0;
    data->start_y = 0;
    data->down_ts = 0;
    data->start_valid = false;
    data->touch_active = false;
    data->moved = false;
}

static inline int32_t abs_local(int32_t v) { return v < 0 ? -v : v; }

static void touch_tap_maybe_set_start(struct touch_tap_data *data) {
    if (!data->touch_active || data->start_valid || !data->have_x || !data->have_y) {
        return;
    }

    data->start_x = data->x;
    data->start_y = data->y;
    data->start_valid = true;
}

static void touch_tap_track_motion(const struct touch_tap_config *cfg,
                                   struct touch_tap_data *data) {
    if (!data->touch_active) {
        return;
    }

    touch_tap_maybe_set_start(data);
    if (!data->start_valid) {
        return;
    }

    if (abs_local(data->x - data->start_x) > (int32_t)cfg->move_threshold ||
        abs_local(data->y - data->start_y) > (int32_t)cfg->move_threshold) {
        data->moved = true;
    }
}

static int touch_tap_handle_event(const struct device *dev, struct input_event *event,
                                  uint32_t param1, uint32_t param2,
                                  struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct touch_tap_config *cfg = dev->config;
    struct touch_tap_data *data = dev->data;

    if (event->type != INPUT_EV_ABS) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    switch (event->code) {
    case INPUT_ABS_X:
        data->x = event->value;
        data->have_x = true;
        touch_tap_track_motion(cfg, data);
        return ZMK_INPUT_PROC_CONTINUE;

    case INPUT_ABS_Y:
        data->y = event->value;
        data->have_y = true;
        touch_tap_track_motion(cfg, data);
        return ZMK_INPUT_PROC_CONTINUE;

    case INPUT_ABS_Z: {
        data->z = event->value;

        if (data->z >= cfg->pressure_threshold) {
            if (!data->touch_active) {
                data->touch_active = true;
                data->moved = false;
                data->start_valid = false;
                data->down_ts = k_uptime_get();
                touch_tap_maybe_set_start(data);
            }
            return ZMK_INPUT_PROC_CONTINUE;
        }

        if (!data->touch_active) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        int64_t elapsed = k_uptime_get() - data->down_ts;
        bool is_tap = data->start_valid && !data->moved && elapsed <= (int64_t)cfg->max_tap_ms;

        touch_tap_reset(data);

        if (!is_tap) {
            return ZMK_INPUT_PROC_CONTINUE;
        }

        event->type = INPUT_EV_KEY;
        event->code = INPUT_BTN_TOUCH;
        event->value = 1;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }
}

static int touch_tap_init(const struct device *dev) {
    struct touch_tap_data *data = dev->data;
    data->have_x = false;
    data->have_y = false;
    touch_tap_reset(data);
    return 0;
}

static const struct zmk_input_processor_driver_api touch_tap_driver_api = {
    .handle_event = touch_tap_handle_event,
};

#define TOUCH_TAP_INST(n)                                                                    \
    static struct touch_tap_data touch_tap_data_##n = {};                                    \
    static const struct touch_tap_config touch_tap_config_##n = {                            \
        .pressure_threshold = DT_INST_PROP(n, pressure_threshold),                           \
        .max_tap_ms = DT_INST_PROP(n, max_tap_ms),                                           \
        .move_threshold = DT_INST_PROP(n, move_threshold),                                   \
    };                                                                                       \
    DEVICE_DT_INST_DEFINE(n, touch_tap_init, NULL, &touch_tap_data_##n,                      \
                          &touch_tap_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &touch_tap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TOUCH_TAP_INST)
