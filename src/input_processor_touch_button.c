/*
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_touch_button

#include <drivers/input_processor.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>

#ifndef BTN_TOUCH
#define BTN_TOUCH 0x14a
#endif

struct touch_button_config {
    uint16_t pressure_threshold;
};

struct touch_button_data {
    bool touch_active;
};

static void touch_button_reset(struct touch_button_data *data) {
    data->touch_active = false;
}

static int touch_button_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct touch_button_config *cfg = dev->config;
    struct touch_button_data *data = dev->data;

    if (event->type != INPUT_EV_ABS || event->code != INPUT_ABS_Z) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->touch_active && event->value >= cfg->pressure_threshold) {
        data->touch_active = true;
        event->type = INPUT_EV_KEY;
        event->code = BTN_TOUCH;
        event->value = 1;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->touch_active && event->value < cfg->pressure_threshold) {
        data->touch_active = false;
        event->type = INPUT_EV_KEY;
        event->code = BTN_TOUCH;
        event->value = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return ZMK_INPUT_PROC_STOP;
}

static int touch_button_init(const struct device *dev) {
    touch_button_reset(dev->data);
    return 0;
}

static const struct zmk_input_processor_driver_api touch_button_driver_api = {
    .handle_event = touch_button_handle_event,
};

#define TOUCH_BUTTON_INST(n)                                                                    \
    static struct touch_button_data touch_button_data_##n = {};                                 \
    static const struct touch_button_config touch_button_config_##n = {                         \
        .pressure_threshold = DT_INST_PROP(n, pressure_threshold),                              \
    };                                                                                          \
    DEVICE_DT_INST_DEFINE(n, touch_button_init, NULL, &touch_button_data_##n,                  \
                          &touch_button_config_##n, POST_KERNEL,                                \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &touch_button_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TOUCH_BUTTON_INST)
