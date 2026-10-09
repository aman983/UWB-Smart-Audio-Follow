/*
 * Scans for the tag, smooths its RSSI with an exponential moving average{EWMA},
 * and reports over serial:
 *   PROX,<node>,<raw_rssi>,<filtered_rssi>,<ms>   every tag advertisement
 *   WAKE,<node>,<ms>                             tag entered the room -> start UWB
 *   SLEEP,<node>,<ms>                            tag left the room    -> stop UWB
 *   LOST,<node>,<ms>                             tag not heard for a while
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

/* ---- Per-board setting: 1, 2 or 3 ---- */
#ifndef NODE_ID
#define NODE_ID 1
#endif

/* Tag identifier - must match the tag firmware */
#define TAG_COMPANY_LO 0xFF
#define TAG_COMPANY_HI 0xFF
#define TAG_GROUP      0x15
#define TAG_ID         0x01

/* Trigger tuning ("tag is in the same room" detection).
 * Hysteresis: switch ON above RSSI_ON, switch OFF only below RSSI_OFF.
 * The gap between them stops WAKE from firing repeatedly when the
 * RSSI hovers around one threshold. Calibrated from 0.5-5 m tests
 * (5 m with rotation: filtered mean -68, min -78). Check RSSI_OFF
 * against measurements from the neighbouring room. */
//#define RSSI_ON          (-70)   /* dBm */
//#define RSSI_OFF         (-76)   /* dBm */

#define RSSI_ON          (-55)   /* dBm */
#define RSSI_OFF         (-65)   /* dBm */

#define TRIGGER_SAMPLES  5       /* ~0.5 s above RSSI_ON  -> WAKE  */
#define OFF_SAMPLES      10      /* ~1 s below RSSI_OFF   -> SLEEP */
#define TAG_LOST_MS      1000    /* no advertisement for this long -> LOST */
#define EMA_ALPHA        0.3f    /* higher = reacts faster, noisier */

/* Scan timing in units of 0.625 ms. window == interval -> continuous. */
#define SCAN_INTERVAL    0x0060  /* 60 ms */
#define SCAN_WINDOW      0x0060  /* 60 ms */

static const struct gpio_dt_spec led_0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios); /* heartbeat */
static const struct gpio_dt_spec led_1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios); /* tag close */

static float    rssi_ema;
static bool     ema_valid;
static int      above_count;
static int      below_count;
static bool     candidate;
static uint32_t last_seen_ms;

/* Called once per AD element; sets *found if this is our tag */
static bool ad_parse_cb(struct bt_data *data, void *user_data)
{
    bool *found = user_data;

    if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 4 &&
        data->data[0] == TAG_COMPANY_LO && data->data[1] == TAG_COMPANY_HI &&
        data->data[2] == TAG_GROUP     && data->data[3] == TAG_ID) {
        *found = true;
        return false;   /* stop parsing */
    }
    return true;        /* continue with next element */
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *buf)
{
    bool found = false;

    bt_data_parse(buf, ad_parse_cb, &found);
    if (!found) {
        return;
    }

    uint32_t now = k_uptime_get_32();
    last_seen_ms = now;

    /* Exponential moving average of RSSI */
    if (!ema_valid) {
        rssi_ema = rssi;
        ema_valid = true;
    } else {
        rssi_ema = EMA_ALPHA * rssi + (1.0f - EMA_ALPHA) * rssi_ema;
    }

    /* Hysteresis state machine. Several samples in a row are required
     * in both directions so one RSSI spike can't change the state. */
    if (!candidate) {
        above_count = (rssi_ema > RSSI_ON) ? above_count + 1 : 0;
        if (above_count >= TRIGGER_SAMPLES) {
            candidate = true;
            below_count = 0;
            /* Later: this becomes the wake request to the server -> start UWB */
            printk("WAKE,%d,%u\n", NODE_ID, now);
        }
    } else {
        below_count = (rssi_ema < RSSI_OFF) ? below_count + 1 : 0;
        if (below_count >= OFF_SAMPLES) {
            candidate = false;
            above_count = 0;
            /* Later: tell the server UWB can stop for this node */
            printk("SLEEP,%d,%u\n", NODE_ID, now);
        }
    }
    gpio_pin_set_dt(&led_1, candidate);

    printk("PROX,%d,%d,%d,%u\n", NODE_ID, rssi, (int)rssi_ema, now);
}

int main(void)
{
    int err;

    if (!gpio_is_ready_dt(&led_0) || !gpio_is_ready_dt(&led_1)) {
        printk("LEDs not ready\n");
        return 0;
    }
    gpio_pin_configure_dt(&led_0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_1, GPIO_OUTPUT_INACTIVE);

    err = bt_enable(NULL);
    if (err) {
        printk("bt_enable failed (%d)\n", err);
        return 0;
    }

    struct bt_le_scan_param p = {
        .type     = BT_LE_SCAN_TYPE_PASSIVE,
        .options  = BT_LE_SCAN_OPT_NONE,   /* no duplicate filter: we want every adv */
        .interval = SCAN_INTERVAL,
        .window   = SCAN_WINDOW,
    };

    err = bt_le_scan_start(&p, scan_cb);
    if (err) {
        printk("Scan start failed (%d)\n", err);
        return 0;
    }
    printk("ANCHOR %d: scanning\n", NODE_ID);

    while (1) {
        /* Tag-lost timeout */
        if (ema_valid && (k_uptime_get_32() - last_seen_ms) > TAG_LOST_MS) {
            printk("LOST,%d,%u\n", NODE_ID, k_uptime_get_32());
            ema_valid = false;
            above_count = 0;
            below_count = 0;
            candidate = false;
            gpio_pin_set_dt(&led_1, 0);
        }
        gpio_pin_toggle_dt(&led_0);   /* heartbeat */
        k_msleep(200);
    }
    return 0;
}