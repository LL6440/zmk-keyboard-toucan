/*
 * Circular gesture scroll input processor for ZMK.
 * Works on all layers — no layer restriction.
 *
 * === Root cause of the "back-and-forth" bug (now fixed) ===
 *
 * The tangential formula  tang = cum_x * dy - cum_y * dx
 * computes angular momentum about the ORIGIN (touch-start point).
 *
 * If the touch-start is OUTSIDE or ON the circle the user traces
 * (which is always the case: the user places their finger at the edge
 * of the pad and traces a circle whose centre is somewhere else),
 * then the tangential oscillates between positive and negative on
 * each revolution. This produces the observed back-and-forth scroll.
 *
 * === Fix: use the circle centre as origin ===
 *
 * The time-average of any point on a circle is the circle's centre.
 * We maintain an exponential moving average (EMA) of cum_x/cum_y:
 *
 *   ema_x = ema_x * (W-1)/W + cum_x
 *   ref_x  = ema_x / W               <- circle-centre estimate
 *
 * Then:  tang = (cum_x - ref_x) * dy - (cum_y - ref_y) * dx
 *
 * As the finger goes around, ema converges to the circle centre.
 * After the first half-turn (~W events lag) tang stays positive
 * for CW and negative for CCW, cleanly driving the scroll.
 *
 * W = EMA_SHIFT (power of 2 for cheap division): default 8.
 * ema_x is stored as W * actual_ref_x to avoid float.
 *
 * === Detection ===
 * During the first detect-samples Y-events we accumulate:
 *   tang_detect += cum_x * dy - cum_y * dx  (origin = touch start)
 * At that early stage the finger has moved < quarter circle, so
 * origin-on-circle is fine: the tangential is always >= 0 and easily
 * crosses tang-threshold.
 * The dominant direction (sum_dx vs sum_dy) determines the axis.
 *
 * === Scroll magnitude ===
 * scroll_threshold = r_sq * scroll_divisor / 1000
 * where r_sq is estimated at mode-lock time (using EMA centre).
 * Ticks/revolution ~= 2*pi*1000 / scroll_divisor
 * Default divisor 419 -> ~15 ticks/revolution.
 *
 * === Touch detection ===
 * The Cirque Pinnacle in relative mode does not emit BTN_TOUCH events.
 * Two timeouts:
 *   - DETECT / POINTER : touch_timeout_ms (200 ms)
 *   - SCROLL locked    : scroll_lock_timeout_ms (500 ms)
 * Only a genuine finger lift (no events for the timeout period) resets
 * the mode; a brief pause during circular motion does not.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * ZMK input processor API — defined locally because zmk/input_processor.h
 * is not always reachable from external modules.
 * Layout must match zmk/app/src/pointing/input_listener.c.
 */
struct zmk_input_processor_state { uint8_t current_layer; };
__subsystem struct zmk_input_processor_driver_api {
    int (*handle_event)(const struct device *dev, struct input_event *event,
                        uint32_t p1, uint32_t p2,
                        struct zmk_input_processor_state *state);
};
#define ZMK_INPUT_PROC_CONTINUE 0

#ifndef INPUT_BTN_TOUCH
#define INPUT_BTN_TOUCH 0x14au
#endif

#define DT_DRV_COMPAT zmk_input_processor_circular_scroll

/* Shift for EMA: ema stored as (2^EMA_SHIFT * actual_value) */
#define EMA_SHIFT 3   /* window = 8 events */

/* ── state machine ───────────────────────────────────────────────────── */

enum cs_mode { CS_MODE_DETECT, CS_MODE_POINTER, CS_MODE_SCROLL_V, CS_MODE_SCROLL_H };

struct cs_data {
    enum cs_mode mode;

    /* Accumulated position from touch start */
    int32_t cum_x, cum_y;

    /* EMA of cum (stored scaled by 2^EMA_SHIFT) — converges to circle centre */
    int32_t ema_x, ema_y;

    /* Detection accumulators */
    int32_t  sum_dx, sum_dy;   /* direction sum for axis detection */
    int64_t  tang_detect;      /* angular momentum during detection */
    int16_t  detect_count;

    /* Pending X delta (paired with next Y event) */
    int32_t  pending_dx;

    /* Scroll state */
    int64_t  scroll_accum;
    int32_t  scroll_threshold;

    /* Timing */
    int64_t  last_event_ms;
};

struct cs_config {
    uint8_t  detect_samples;
    int32_t  tang_threshold;
    int32_t  scroll_divisor;
    int32_t  touch_timeout_ms;
    int32_t  scroll_lock_timeout_ms;
    bool     invert_vertical;
    bool     invert_horizontal;
};

/* ── helpers ─────────────────────────────────────────────────────────── */

static void cs_reset(struct cs_data *d, int64_t now)
{
    memset(d, 0, sizeof(*d));
    d->mode          = CS_MODE_DETECT;
    d->last_event_ms = now;
}

static void cs_classify(struct cs_data *d, const struct cs_config *cfg)
{
    int64_t abs_tang = d->tang_detect < 0 ? -d->tang_detect : d->tang_detect;

    LOG_DBG("CS classify: tang=%lld threshold=%d dir=(%d,%d)",
            d->tang_detect, cfg->tang_threshold, d->sum_dx, d->sum_dy);

    if (abs_tang < (int64_t)cfg->tang_threshold) {
        d->mode = CS_MODE_POINTER;
        LOG_DBG("CS: -> POINTER");
        return;
    }

    int32_t adx = d->sum_dx < 0 ? -d->sum_dx : d->sum_dx;
    int32_t ady = d->sum_dy < 0 ? -d->sum_dy : d->sum_dy;
    d->mode = (ady >= adx) ? CS_MODE_SCROLL_V : CS_MODE_SCROLL_H;
    LOG_DBG("CS: -> %s", d->mode == CS_MODE_SCROLL_V ? "SCROLL_V" : "SCROLL_H");

    /* Estimate r² from EMA centre: ref = ema / 8 */
    int32_t ref_x = d->ema_x >> EMA_SHIFT;
    int32_t ref_y = d->ema_y >> EMA_SHIFT;
    int32_t rx    = d->cum_x - ref_x;
    int32_t ry    = d->cum_y - ref_y;
    int64_t r_sq  = (int64_t)rx * rx + (int64_t)ry * ry;

    int32_t thr = (int32_t)((r_sq * (int64_t)cfg->scroll_divisor) / 1000);
    d->scroll_threshold = thr < 8 ? 8 : thr;
    d->scroll_accum     = 0;

    LOG_DBG("CS: r_sq=%lld scroll_threshold=%d ref=(%d,%d)",
            r_sq, d->scroll_threshold, ref_x, ref_y);
}

/* ── event handler ───────────────────────────────────────────────────── */

static int cs_handle_event(const struct device *dev,
                           struct input_event *event,
                           uint32_t p1, uint32_t p2,
                           struct zmk_input_processor_state *state)
{
    struct cs_data         *d   = dev->data;
    const struct cs_config *cfg = dev->config;
    int64_t                 now = k_uptime_get();

    /* BTN_TOUCH: always reset (finger lift / new touch) */
    if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_TOUCH) {
        cs_reset(d, now);
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

    /* Timeout: two thresholds depending on whether scroll is locked */
    bool locked = (d->mode == CS_MODE_SCROLL_V || d->mode == CS_MODE_SCROLL_H);
    int64_t tmo = locked ? (int64_t)cfg->scroll_lock_timeout_ms
                         : (int64_t)cfg->touch_timeout_ms;
    if (d->last_event_ms != 0 && (now - d->last_event_ms) > tmo) {
        cs_reset(d, now);
        LOG_DBG("CS: timeout (%s) -> reset", locked ? "locked" : "detect");
    }
    d->last_event_ms = now;

    /* POINTER: pass through */
    if (d->mode == CS_MODE_POINTER) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* Buffer X; process paired (dx, dy) on Y event.
     * In non-pointer modes we suppress the raw X/Y by zeroing them;
     * scroll events are emitted in their place on the Y event. */
    if (is_x) {
        d->pending_dx = event->value;
        event->value  = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t dx = d->pending_dx;
    int32_t dy = event->value;
    d->pending_dx = 0;

    switch (d->mode) {

    /* ── Detection ─────────────────────────────────────────────────── */
    case CS_MODE_DETECT:
        d->sum_dx += dx;
        d->sum_dy += dy;

        /* tang from touch-start origin (fine for the first quarter circle) */
        d->tang_detect += (int64_t)d->cum_x * dy - (int64_t)d->cum_y * dx;

        /* Update position */
        d->cum_x += dx;
        d->cum_y += dy;

        /* Update EMA: ema = ema * 7/8 + cum  (stored as 8 * actual) */
        d->ema_x = d->ema_x - (d->ema_x >> EMA_SHIFT) + d->cum_x;
        d->ema_y = d->ema_y - (d->ema_y >> EMA_SHIFT) + d->cum_y;

        d->detect_count++;
        event->value = 0;

        if (d->detect_count >= cfg->detect_samples) {
            cs_classify(d, cfg);
        }
        break;

    /* ── Scroll (vertical or horizontal) ───────────────────────────── */
    case CS_MODE_SCROLL_V:
    case CS_MODE_SCROLL_H: {

        /* Update position */
        d->cum_x += dx;
        d->cum_y += dy;

        /* Update EMA centre estimate */
        d->ema_x = d->ema_x - (d->ema_x >> EMA_SHIFT) + d->cum_x;
        d->ema_y = d->ema_y - (d->ema_y >> EMA_SHIFT) + d->cum_y;

        /* Position relative to estimated circle centre */
        int32_t ref_x = d->ema_x >> EMA_SHIFT;
        int32_t ref_y = d->ema_y >> EMA_SHIFT;
        int32_t rx    = d->cum_x - ref_x;
        int32_t ry    = d->cum_y - ref_y;

        /*
         * Tangential component using PREVIOUS position (before this event).
         * We already added dx/dy to cum, so previous = current - delta.
         */
        int32_t prev_rx = rx - dx;
        int32_t prev_ry = ry - dy;
        int64_t tangential = (int64_t)prev_rx * dy - (int64_t)prev_ry * dx;

        d->scroll_accum += tangential;

        int32_t ticks = 0;
        if (d->scroll_accum >= d->scroll_threshold) {
            ticks = (int32_t)(d->scroll_accum / d->scroll_threshold);
            d->scroll_accum -= (int64_t)ticks * d->scroll_threshold;
        } else if (d->scroll_accum <= -(int64_t)d->scroll_threshold) {
            ticks = (int32_t)(d->scroll_accum / d->scroll_threshold);
            d->scroll_accum -= (int64_t)ticks * d->scroll_threshold;
        }

        if (d->mode == CS_MODE_SCROLL_V) {
            event->code  = INPUT_REL_WHEEL;
            event->value = cfg->invert_vertical   ? -ticks : ticks;
        } else {
            event->code  = INPUT_REL_HWHEEL;
            event->value = cfg->invert_horizontal ? -ticks : ticks;
        }
        break;
    }

    default:
        break;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

/* ── driver boilerplate ──────────────────────────────────────────────── */

static int cs_init(const struct device *dev) { return 0; }

static const struct zmk_input_processor_driver_api cs_driver_api = {
    .handle_event = cs_handle_event,
};

#define CS_INST(n)                                                                  \
    static struct cs_data cs_data_##n = {.mode = CS_MODE_DETECT};                  \
    static const struct cs_config cs_config_##n = {                                 \
        .detect_samples         = DT_INST_PROP(n, detect_samples),                  \
        .tang_threshold         = DT_INST_PROP(n, tang_threshold),                  \
        .scroll_divisor         = DT_INST_PROP(n, scroll_divisor),                  \
        .touch_timeout_ms       = DT_INST_PROP(n, touch_timeout_ms),                \
        .scroll_lock_timeout_ms = DT_INST_PROP(n, scroll_lock_timeout_ms),          \
        .invert_vertical        = DT_INST_PROP(n, invert_vertical),                 \
        .invert_horizontal      = DT_INST_PROP(n, invert_horizontal),               \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(n, cs_init, NULL, &cs_data_##n, &cs_config_##n,          \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,         \
                          &cs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CS_INST)
