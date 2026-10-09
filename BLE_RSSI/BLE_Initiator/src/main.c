/*
 * Advertises a fixed identifier in manufacturer-specific data so the
 *  Responder can recognise this and measure its RSSI.
 *
 * LED0 flashes briefly every time an advertisement is actually sent.
 *
 * How: the advertising set is started for exactly ONE advertising event.
 * When the controller has sent it, the .sent callback fires -> we flash
 * the LED and schedule the next advertisement. So the application, not
 * the controller, decides the interval.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/random/random.h>

/* Identifier: company ID 0xFFFF (test), group 0x15, tag 0x01.
 * Must match the values in the anchor firmware. */
#define TAG_COMPANY_LO 0xFF
#define TAG_COMPANY_HI 0xFF
#define TAG_GROUP      0x15
#define TAG_ID         0x01

/* Time between advertisements. Experiment variable:
 * shorter = faster detection, longer = lower power. */
#define ADV_INTERVAL_MS  100

/* Random extra delay 0..ADV_JITTER_MS, like BLE's own advDelay.
 * Prevents repeated collisions with other BLE devices. */
#define ADV_JITTER_MS    10

/* LED on-time per advertisement */
#define LED_FLASH_MS     20

/* Set to 0 before power measurements (LED + prints cost current) */
#define DEBUG_LED        1

static const struct gpio_dt_spec led_0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
                  TAG_COMPANY_LO, TAG_COMPANY_HI, TAG_GROUP, TAG_ID),
};

static struct bt_le_ext_adv *adv;
static uint32_t adv_count;

/* ---------- LED off after a short flash ---------- */
static void led_off_fn(struct k_work *work)
{
    gpio_pin_set_dt(&led_0, 0);
}
static K_WORK_DELAYABLE_DEFINE(led_off_work, led_off_fn);

/* ---------- Start one advertising event ---------- */
static void adv_start_fn(struct k_work *work)
{
    /* timeout 0, exactly 1 advertising event (sent on channels 37, 38, 39) */
    int err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_PARAM(0, 1));
    if (err) {
        printk("adv start failed (%d)\n", err);
    }
}
static K_WORK_DELAYABLE_DEFINE(adv_work, adv_start_fn);

/* ---------- Called by the stack after the advertisement was sent ---------- */
static void adv_sent_cb(struct bt_le_ext_adv *instance,
                        struct bt_le_ext_adv_sent_info *info)
{
    adv_count++;

#if DEBUG_LED
    gpio_pin_set_dt(&led_0, 1);
    k_work_reschedule(&led_off_work, K_MSEC(LED_FLASH_MS));
#endif

    uint32_t delay = ADV_INTERVAL_MS + (sys_rand32_get() % (ADV_JITTER_MS + 1));
    k_work_reschedule(&adv_work, K_MSEC(delay));
}

static const struct bt_le_ext_adv_cb adv_cb = {
    .sent = adv_sent_cb,
};

int main(void)
{
    int err;

    if (!gpio_is_ready_dt(&led_0)) {
        printk("LED not ready\n");
        return 0;
    }
    gpio_pin_configure_dt(&led_0, GPIO_OUTPUT_INACTIVE);

    err = bt_enable(NULL);
    if (err) {
        printk("bt_enable failed (%d)\n", err);
        return 0;
    }

    /* No BT_LE_ADV_OPT_EXT_ADV -> legacy advertising PDUs, so the
     * anchors' normal scanner still receives them. */
    err = bt_le_ext_adv_create(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY,
                                               BT_GAP_ADV_FAST_INT_MIN_2,
                                               BT_GAP_ADV_FAST_INT_MAX_2,
                                               NULL),
                               &adv_cb, &adv);
    if (err) {
        printk("adv create failed (%d)\n", err);
        return 0;
    }

    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("adv set data failed (%d)\n", err);
        return 0;
    }

    k_work_reschedule(&adv_work, K_NO_WAIT);
    printk("TAG: advertising every ~%d ms\n", ADV_INTERVAL_MS);

    while (1) {
#if DEBUG_LED
        k_sleep(K_SECONDS(10));
        printk("TAG: %u advertisements sent\n", adv_count);
#else
        k_sleep(K_FOREVER);
#endif
    }
    return 0;
}