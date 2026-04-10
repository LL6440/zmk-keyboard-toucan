/*
 * Circular gesture scroll input processor for ZMK.
 *
 * Detects circular motion on a trackpad and converts it to scroll events.
 * No layer restriction — gesture works on all layers.
 *
 * === Gesture detection ===
 * The processor accumulates movement in two consecutive phases of
 * `detect-samples` events each.  If the angle between the two phase
 * vectors is greater than ~60 ° (i.e. |cos| < 0.5), the motion is
 * considered circular.  The axis (vertical / horizontal) is determined
 * by the dominant direction of the first phase vector.
 *
 *   - First phase mostly vertical   → vertical scroll mode
 *   - First phase mostly horizontal → horizontal scroll mode
 *   - Straight / diagonal motion    → pointer mode (no scroll)
 *
 * This naturally handles "finger at 3h going radially inward (left)" as
 * pointer mode, because straight-line motion produces no direction change
 * between the two phases.
 *
 * === Scroll calculation ===
 * Once scroll mode is locked the processor computes the *tangential*
 * component of each XY movement relative to the accumulated position
 * vector from the touch start:
 *
 *   tangential = cum_x * dy − cum_y * dx
 *
 * For a perfect circle of radius r, this sums to 2π·r² per full
 * revolution.  The scroll threshold is set to r²·scroll_divisor/1000,
 * which gives ≈ 2π·1000/scroll_divisor ticks per revolution.
 * With the default divisor of 419 that is ≈ 15 ticks/rev.
 *
 * Clockwise rotation produces positive tangential values.  By default
 * the processor emits positive WHEEL / HWHEEL values for clockwise.
 * Use `invert-vertical` / `invert-horizontal` if the direction is wrong
 * for your system.
 *
 * === Touch detection ===
 * The processor handles explicit INPUT_BTN_TOUCH events AND uses a
 * configurable inactivity timeout to detect touch end (covers drivers
 * that do not emit BTN_TOUCH).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * ── ZMK input processor API (defined locally) ────────────────────────────
 *
 * zmk/input_processor.h is an internal ZMK header that is not always
 * reachable from external modules.  We reproduce the two structs we need
 * here; their layout must match zmk/app/src/input_listener.c exactly.
 * If ZMK ever changes this API the symptom would be a hard-fault at runtime.
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

/* BTN_TOUCH (Linux / Zephyr HID code 0x14a). Guard against redefinition. */
#ifndef INPUT_BTN_TOUCH
#define INPUT_BTN_TOUCH 0x14au
#endif

#define DT_DRV_COMPAT zmk_input_processor_circular_scroll

/* ── state machine ────────────────────────────────────────────────────── */

enum cs_mode {
    CS_MODE_DETECT_1, /* accumulating phase-1 direction vector          */
    CS_MODE_DETECT_2, /* accumulating phase-2 direction vector          */
    CS_MODE_POINTER,  /* straight motion — pass events through unchanged */
    CS_MODE_SCROLL_V, /* vertical scroll locked                          */
    CS_MODE_SCROLL_H, /* horizontal scroll locked                        */
};

struct cs_data {
    enum cs_mode mode;

    /* Phase-1 direction accumulator */
    int32_t a1x, a1y;
    int16_t phase1_count;

    /* Phase-2 direction accumulator */
    int32_t a2x, a2y;
    int16_t phase2_count;

    /* Cumulative position from touch start (for tangential calc) */
    int32_t cum_x, cum_y;

    /* Buffered X delta — paired with the next Y event */
    int32_t pending_dx;

    /* Fractional scroll accumulator and per-tick threshold */
    int64_t scroll_accum;
    int64_t scroll_threshold;

    /* Timestamp of the last received event (for timeout-based reset) */
    int64_t last_event_ms;
};

struct cs_config {
    uint8_t  detect_samples;     /* events per detection phase            */
    int32_t  scroll_divisor;     /* threshold = r² * divisor / 1000      */
    int32_t  touch_timeout_ms;   /* inactivity → treat as new touch       */
    bool     invert_vertical;    /* invert WHEEL sign                     */
    bool     invert_horizontal;  /* invert HWHEEL sign                    */
};

/* ── helpers ──────────────────────────────────────────────────────────── */

static void cs_reset(struct cs_data *data, int64_t now)
{
    memset(data, 0, sizeof(*data));
    data->mode           = CS_MODE_DETECT_1;
    data->last_event_ms  = now;
}

/*
 * Called after both detection phases are complete.
 * Examines a1/a2 vectors and sets data->mode accordingly.
 */
static void cs_determine_mode(struct cs_data *data,
                               const struct cs_config *cfg)
{
    int64_t mag1sq = (int64_t)data->a1x * data->a1x
                   + (int64_t)data->a1y * data->a1y;
    int64_t mag2sq = (int64_t)data->a2x * data->a2x
                   + (int64_t)data->a2y * data->a2y;

    /* Require a minimum of movement in each phase */
    if (mag1sq < 4 || mag2sq < 4) {
        data->mode = CS_MODE_POINTER;
        LOG_DBG("CS: too little movement → pointer");
        return;
    }

    int64_t dot    = (int64_t)data->a1x * data->a2x
                   + (int64_t)data->a1y * data->a2y;
    int64_t dot_sq = dot * dot;

    /*
     * Perpendicularity test: |cos(θ)| < 0.5  ↔  |dot|² < mag1² × mag2² / 4
     * We rearrange to avoid the division overflowing int64:
     *   4 × dot² < mag1² × mag2²
     */
    if ((4 * dot_sq) < (mag1sq * mag2sq)) {
        /* Circular motion confirmed — pick axis from phase-1 direction */
        int32_t abs_a1x = data->a1x < 0 ? -data->a1x : data->a1x;
        int32_t abs_a1y = data->a1y < 0 ? -data->a1y : data->a1y;

        if (abs_a1y >= abs_a1x) {
            data->mode = CS_MODE_SCROLL_V;
            LOG_DBG("CS: circular → VERTICAL scroll");
        } else {
            data->mode = CS_MODE_SCROLL_H;
            LOG_DBG("CS: circular → HORIZONTAL scroll");
        }

        /* Lock the scroll threshold from the current radius */
        int64_t r_sq = (int64_t)data->cum_x * data->cum_x
                     + (int64_t)data->cum_y * data->cum_y;
        data->scroll_threshold = (r_sq * cfg->scroll_divisor) / 1000;

        /* Clamp to a sensible minimum so tiny circles still scroll */
        if (data->scroll_threshold < 8) {
            data->scroll_threshold = 8;
        }

        data->scroll_accum = 0;
        LOG_DBG("CS: r_sq=%lld threshold=%lld", r_sq, data->scroll_threshold);
    } else {
        data->mode = CS_MODE_POINTER;
        LOG_DBG("CS: straight motion → pointer");
    }
}

/* ── event handler ────────────────────────────────────────────────────── */

static int cs_handle_event(const struct device *dev,
                           struct input_event *event,
                           uint32_t param1, uint32_t param2,
                           struct zmk_input_processor_state *state)
{
    struct cs_data             *data = dev->data;
    const struct cs_config     *cfg  = dev->config;
    int64_t                     now  = k_uptime_get();

    /* ── 1. Explicit touch button (BTN_TOUCH) ───────────────────────── */
    if (event->type == INPUT_EV_KEY &&
        event->code == INPUT_BTN_TOUCH) {
        if (event->value != 0) {
            cs_reset(data, now);
            LOG_DBG("CS: BTN_TOUCH down → DETECT_1");
        } else {
            cs_reset(data, now);
            /* Start fresh; next movement will begin a new detection */
            LOG_DBG("CS: BTN_TOUCH up → reset");
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* ── 2. Only process relative motion events ─────────────────────── */
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    bool is_x = (event->code == INPUT_REL_X);
    bool is_y = (event->code == INPUT_REL_Y);

    if (!is_x && !is_y) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* ── 3. Timeout-based touch detection ───────────────────────────── */
    if (data->last_event_ms != 0 &&
        (now - data->last_event_ms) > cfg->touch_timeout_ms) {
        cs_reset(data, now);
        LOG_DBG("CS: timeout → new touch, DETECT_1");
    }
    data->last_event_ms = now;

    /* ── 4. Pointer mode — pass through unchanged ───────────────────── */
    if (data->mode == CS_MODE_POINTER) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* ── 5. Buffer X; process paired (dx, dy) on Y event ───────────── */
    if (is_x) {
        /*
         * In detection and scroll modes we suppress the raw X event by
         * zeroing its value.  The event still propagates through the
         * remainder of the processor chain with value = 0, which is
         * harmless (scaling 0 yields 0, etc.).
         */
        data->pending_dx = event->value;
        event->value = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* --- Y event from here on --- */
    int32_t dx = data->pending_dx;
    int32_t dy = event->value;
    data->pending_dx = 0;

    switch (data->mode) {

    /* ── Detection phase 1 ───────────────────────────────────────────── */
    case CS_MODE_DETECT_1:
        data->a1x += dx;
        data->a1y += dy;
        data->cum_x += dx;
        data->cum_y += dy;
        data->phase1_count++;
        event->value = 0; /* eat movement during detection */

        if (data->phase1_count >= cfg->detect_samples) {
            data->mode = CS_MODE_DETECT_2;
            LOG_DBG("CS: phase1 done (a1=%d,%d) → DETECT_2",
                    data->a1x, data->a1y);
        }
        break;

    /* ── Detection phase 2 ───────────────────────────────────────────── */
    case CS_MODE_DETECT_2:
        data->a2x += dx;
        data->a2y += dy;
        data->cum_x += dx;
        data->cum_y += dy;
        data->phase2_count++;
        event->value = 0; /* eat movement during detection */

        if (data->phase2_count >= cfg->detect_samples) {
            cs_determine_mode(data, cfg);
            /* If we fell into pointer mode, pending events were already
             * zeroed — acceptable; the brief detection window (~100-200 ms)
             * is the price of gesture recognition. */
        }
        break;

    /* ── Vertical scroll mode ────────────────────────────────────────── */
    case CS_MODE_SCROLL_V: {
        /*
         * Tangential component of (dx, dy) relative to the accumulated
         * position vector (cum_x, cum_y):
         *
         *   tangential = cum_x · dy − cum_y · dx
         *
         * Positive tangential ↔ clockwise rotation.
         * Update cum *after* computing tangential to use the pre-move
         * position.
         */
        int64_t tangential = (int64_t)data->cum_x * dy
                           - (int64_t)data->cum_y * dx;
        data->cum_x += dx;
        data->cum_y += dy;
        data->scroll_accum += tangential;

        int32_t ticks = 0;
        if (data->scroll_accum >= data->scroll_threshold) {
            ticks = (int32_t)(data->scroll_accum / data->scroll_threshold);
            data->scroll_accum -= (int64_t)ticks * data->scroll_threshold;
        } else if (data->scroll_accum <= -data->scroll_threshold) {
            ticks = (int32_t)(data->scroll_accum / data->scroll_threshold);
            data->scroll_accum -= (int64_t)ticks * data->scroll_threshold;
        }

        event->code  = INPUT_REL_WHEEL;
        event->value = cfg->invert_vertical ? -ticks : ticks;
        break;
    }

    /* ── Horizontal scroll mode ──────────────────────────────────────── */
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
        } else if (data->scroll_accum <= -data->scroll_threshold) {
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

static int cs_init(const struct device *dev)
{
    return 0;
}

static const struct zmk_input_processor_driver_api cs_driver_api = {
    .handle_event = cs_handle_event,
};

#define CS_INST(n)                                                              \
    static struct cs_data cs_data_##n = {                                       \
        .mode          = CS_MODE_DETECT_1,                                      \
        .last_event_ms = 0,                                                     \
    };                                                                          \
    static const struct cs_config cs_config_##n = {                             \
        .detect_samples   = DT_INST_PROP(n, detect_samples),                    \
        .scroll_divisor   = DT_INST_PROP(n, scroll_divisor),                    \
        .touch_timeout_ms = DT_INST_PROP(n, touch_timeout_ms),                  \
        .invert_vertical  = DT_INST_PROP(n, invert_vertical),                   \
        .invert_horizontal = DT_INST_PROP(n, invert_horizontal),                \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, cs_init, NULL,                                     \
                          &cs_data_##n, &cs_config_##n,                         \
                          POST_KERNEL,                                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                   \
                          &cs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CS_INST)
