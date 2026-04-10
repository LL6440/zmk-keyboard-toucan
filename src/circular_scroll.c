/*
 * Circular gesture scroll input processor for ZMK.
 *
 * Detects circular motion on a trackpad and converts it to scroll events.
 * Works on all layers — no layer restriction.
 *
 * === Why angular momentum, not two-phase direction comparison ===
 *
 * A naive approach comparing the direction vector in two consecutive
 * windows fails for any realistic scroll speed: at 1 rev/3 s the
 * direction rotates only ~7° per 60 ms window, far below any useful
 * threshold.
 *
 * Instead we accumulate the *angular momentum* of the finger relative
 * to its path from touch start:
 *
 *   tang += cum_x * dy - cum_y * dx    (cross product of position x velocity)
 *
 * For circular motion this sum grows as r^2 * (angle swept).
 * For a straight line, cum and (dx,dy) are always parallel, so tang = 0.
 * This works reliably at any speed, even with small integer deltas.
 *
 * Example (r=40, v=3 units/event, 15 detect events):
 *   angle swept ~= 3*15/40 = 1.1 rad  ->  tang ~= 1.1 * 1600 = 1760
 *   straight line of the same length   ->  tang ~= 0
 *
 * Default tang-threshold = 200, comfortably between these two values.
 *
 * === Axis detection ===
 * The dominant direction of accumulated movement during detection
 * (sum_dx, sum_dy) determines the scroll axis:
 *   |sum_dy| >= |sum_dx|  ->  VERTICAL scroll
 *   |sum_dx| >  |sum_dy|  ->  HORIZONTAL scroll
 *
 * A purely radial motion (e.g. 3h toward center) accumulates no angular
 * momentum -> stays in POINTER mode.
 *
 * === Scroll calculation ===
 * After mode is locked the same tangential formula drives scroll.
 * Ticks are emitted when |scroll_accum| >= scroll_threshold.
 * scroll_threshold = r_sq * scroll_divisor / 1000, giving
 * ~= 2*pi*1000 / scroll_divisor ticks per full revolution.
 * Default divisor 419 -> ~= 15 ticks/revolution.
 *
 * Clockwise rotation produces positive tangential values (positive WHEEL).
 * Use invert-vertical / invert-horizontal if the direction is wrong.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * ZMK input processor API defined locally: zmk/input_processor.h is an
 * internal header not always reachable from external modules.
 * Layout must match zmk/app/src/pointing/input_listener.c.
 */
struct zmk_input_processor_state {
    uint8_t current_layer;
};

__subsystem struct zmk_input_processor_driver_api {
    int (*handle_event)(const struct device *dev,
                        struct input_event *event,
                        uint32_t param1, uint32_t param2,
                        struct zmk_input_processor_state *state);
};

#define ZMK_INPUT_PROC_CONTINUE 0

#ifndef INPUT_BTN_TOUCH
#define INPUT_BTN_TOUCH 0x14au
#endif

#define DT_DRV_COMPAT zmk_input_processor_circular_scroll

/* ── state machine ────────────────────────────────────────────────────── */

enum cs_mode {
    CS_MODE_DETECT,
    CS_MODE_POINTER,
    CS_MODE_SCROLL_V,
    CS_MODE_SCROLL_H,
};

struct cs_data {
    enum cs_mode mode;
    int32_t  sum_dx, sum_dy;
    int32_t  cum_x, cum_y;
    int64_t  tang_detect;
    int16_t  detect_count;
    int32_t  pending_dx;
    int64_t  scroll_accum;
    int32_t  scroll_threshold;
    int64_t  last_event_ms;
};

struct cs_config {
    uint8_t  detect_samples;
    int32_t  tang_threshold;
    int32_t  scroll_divisor;
    int32_t  touch_timeout_ms;
    bool     invert_vertical;
    bool     invert_horizontal;
};

/* ── helpers ──────────────────────────────────────────────────────────── */

static void cs_reset(struct cs_data *data, int64_t now)
{
    memset(data, 0, sizeof(*data));
    data->mode          = CS_MODE_DETECT;
    data->last_event_ms = now;
}

static void cs_classify(struct cs_data *data, const struct cs_config *cfg)
{
    int64_t abs_tang = data->tang_detect < 0 ? -data->tang_detect : data->tang_detect;

    LOG_DBG("CS classify: tang=%lld threshold=%d dir=(%d,%d)",
            data->tang_detect, cfg->tang_threshold,
            data->sum_dx, data->sum_dy);

    if (abs_tang < (int64_t)cfg->tang_threshold) {
        data->mode = CS_MODE_POINTER;
        LOG_DBG("CS: -> POINTER");
        return;
    }

    int32_t abs_sdx = data->sum_dx < 0 ? -data->sum_dx : data->sum_dx;
    int32_t abs_sdy = data->sum_dy < 0 ? -data->sum_dy : data->sum_dy;

    data->mode = (abs_sdy >= abs_sdx) ? CS_MODE_SCROLL_V : CS_MODE_SCROLL_H;
    LOG_DBG("CS: -> %s", data->mode == CS_MODE_SCROLL_V ? "SCROLL_V" : "SCROLL_H");

    int64_t r_sq = (int64_t)data->cum_x * data->cum_x
                 + (int64_t)data->cum_y * data->cum_y;
    int32_t thr = (int32_t)((r_sq * (int64_t)cfg->scroll_divisor) / 1000);
    data->scroll_threshold = thr < 8 ? 8 : thr;
    data->scroll_accum     = 0;

    LOG_DBG("CS: r_sq=%lld scroll_threshold=%d", r_sq, data->scroll_threshold);
}

/* ── event handler ────────────────────────────────────────────────────── */

static int cs_handle_event(const struct device *dev,
                           struct input_event *event,
                           uint32_t param1, uint32_t param2,
                           struct zmk_input_processor_state *state)
{
    struct cs_data         *data = dev->data;
    const struct cs_config *cfg  = dev->config;
    int64_t                 now  = k_uptime_get();

    if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_TOUCH) {
        cs_reset(data, now);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool is_x = (event->code == INPUT_REL_X);
    bool is_y = (event->code == INPUT_REL_Y);
    if (!is_x && !is_y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (data->last_event_ms != 0 &&
        (now - data->last_event_ms) > (int64_t)cfg->touch_timeout_ms) {
        cs_reset(data, now);
        LOG_DBG("CS: timeout -> new touch");
    }
    data->last_event_ms = now;

    if (data->mode == CS_MODE_POINTER) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (is_x) {
        data->pending_dx = event->value;
        event->value = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* --- Y event: process paired (dx, dy) --- */
    int32_t dx = data->pending_dx;
    int32_t dy = event->value;
    data->pending_dx = 0;

    switch (data->mode) {

    case CS_MODE_DETECT:
        data->sum_dx += dx;
        data->sum_dy += dy;
        data->tang_detect += (int64_t)data->cum_x * dy
                           - (int64_t)data->cum_y * dx;
        data->cum_x += dx;
        data->cum_y += dy;
        data->detect_count++;
        event->value = 0;
        if (data->detect_count >= cfg->detect_samples) {
            cs_classify(data, cfg);
        }
        break;

    case CS_MODE_SCROLL_V: {
        int64_t tangential = (int64_t)data->cum_x * dy
                           - (int64_t)data->cum_y * dx;
        data->cum_x += dx;
        data->cum_y += dy;
        data->scroll_accum += tangential;

        int32_t ticks = 0;
        if (data->scroll_accum >= data->scroll_threshold) {
            ticks = (int32_t)(data->scroll_accum / data->scroll_threshold);
            data->scroll_accum -= (int64_t)ticks * data->scroll_threshold;
        } else if (data->scroll_accum <= -(int64_t)data->scroll_threshold) {
            ticks = (int32_t)(data->scroll_accum / data->scroll_threshold);
            data->scroll_accum -= (int64_t)ticks * data->scroll_threshold;
        }
        event->code  = INPUT_REL_WHEEL;
        event->value = cfg->invert_vertical ? -ticks : ticks;
        break;
    }

    case CS_MODE_SCROLL_H: {
        int64_t tangential = (int64_t)data->cum_x * dy
                           - (int64_t)data->cum_y * dx;
        data->cum_x += dx;
        data->cum_y += dy;
        data->scroll_accum += tangential;

        int32_t ticks = 0;
        if (data->scroll_accum >= data->scroll_threshold) {
            ticks = (int32_t)(data->scroll_accum / data->scroll_threshold);
            data->scroll_accum -= (int64_t)ticks * data->scroll_threshold;
        } else if (data->scroll_accum <= -(int64_t)data->scroll_threshold) {
            ticks = (int32_t)(data->scroll_accum / data->scroll_threshold);
            data->scroll_accum -= (int64_t)ticks * data->scroll_threshold;
        }
        event->code  = INPUT_REL_HWHEEL;
        event->value = cfg->invert_horizontal ? -ticks : ticks;
        break;
    }

    default:
        break;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

/* ── driver boilerplate ───────────────────────────────────────────────── */

static int cs_init(const struct device *dev) { return 0; }

static const struct zmk_input_processor_driver_api cs_driver_api = {
    .handle_event = cs_handle_event,
};

#define CS_INST(n)                                                              \
    static struct cs_data cs_data_##n = {.mode = CS_MODE_DETECT};              \
    static const struct cs_config cs_config_##n = {                             \
        .detect_samples    = DT_INST_PROP(n, detect_samples),                   \
        .tang_threshold    = DT_INST_PROP(n, tang_threshold),                   \
        .scroll_divisor    = DT_INST_PROP(n, scroll_divisor),                   \
        .touch_timeout_ms  = DT_INST_PROP(n, touch_timeout_ms),                 \
        .invert_vertical   = DT_INST_PROP(n, invert_vertical),                  \
        .invert_horizontal = DT_INST_PROP(n, invert_horizontal),                \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, cs_init, NULL, &cs_data_##n, &cs_config_##n,      \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,     \
                          &cs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CS_INST)
