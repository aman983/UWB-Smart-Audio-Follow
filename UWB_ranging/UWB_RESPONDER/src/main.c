/*
 * NES Group 15 - UWB test (Stage 2)
 *
 * A stand-alone project to get the DW3110 UWB chip working, separate from
 * the BLE tag/anchor projects. Work through the modes in order:
 *
 *   MODE_DEVID      Read the chip's device ID over SPI. One board.
 *                   Proves SPI, reset and the devicetree overlay are right.
 *   MODE_INITIATOR  Anchor side of single-sided two-way ranging (SS-TWR).
 *   MODE_RESPONDER  Tag side of SS-TWR. Flash this on the second board.
 *
 */

/* Fixed names for the three modes. DO NOT CHANGE these three numbers. */
#define MODE_DEVID      0
#define MODE_INITIATOR  1
#define MODE_RESPONDER  2

/* ====================================================================
 * >>> CHOOSE THE MODE HERE: change only the word at the end of this line
 *     to MODE_DEVID, MODE_INITIATOR or MODE_RESPONDER <<<
 * ==================================================================== */
#define TEST_MODE MODE_RESPONDER

#include <zephyr/kernel.h>
#include <string.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

/* ---------- radio configuration (must be identical on both boards) ---------- */

static dwt_config_t config = {
    .chan            = 5,                 /* channel 5: lower current than 9 */
    .txPreambLength  = DWT_PLEN_128,
    .rxPAC           = DWT_PAC8,
    .txCode          = 9,
    .rxCode          = 9,
    .sfdType         = DWT_SFD_DW_8,
    .dataRate        = DWT_BR_6M8,
    .phrMode         = DWT_PHRMODE_STD,
    .phrRate         = DWT_PHRRATE_STD,
    .sfdTO           = (128 + 1 + 8 - 8), /* preamble + 1 + SFD - PAC */
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,
    .pdoaMode        = DWT_PDOA_M0,
};

/* Qorvo's recommended TX spectrum settings for channel 5 */
static dwt_txconfig_t txconfig = {
    .PGdly   = 0x34,
    .power   = 0xfdfdfdfd,
    .PGcount = 0,
};

/* Antenna delay: default value from Qorvo's examples. It adds a constant
 * offset (typically some tens of cm) until you calibrate it per board. */
#define TX_ANT_DLY 16398
#define RX_ANT_DLY 16398

/* ---------- timing ---------- */

/* 1 UWB microsecond (uus) = 512/499.2 us. In device time units: */
#define UUS_TO_DWT_TIME 63898
#define SPEED_OF_LIGHT  299702547.0   /* m/s in air */

/* Responder: time from receiving the poll to sending the response.
 * Generous, so slow SPI + polling can keep up. Lower it later. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 1000

/* Initiator: when to switch the receiver on after the poll, and for how long.
 * The window must contain the response sent POLL_RX_TO_RESP_TX_DLY_UUS later. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 600


#define RESP_RX_TIMEOUT_UUS        1000

#define RANGING_PERIOD_MS 200

/* ---------- messages ---------- */

/* 802.15.4 data frame: frame control, sequence number, PAN ID, dst, src,
 * function code. The last 2 bytes are the CRC, added by the chip. */
static uint8_t poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'G', 'A', '1',
                             0xE0, 0, 0};
static uint8_t resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '1', 'T', 'G',
                             0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define SEQ_IDX          2
#define COMMON_LEN       10   /* bytes compared to recognise a message */
#define RESP_POLL_RX_IDX 10   /* 4 bytes: when the responder got the poll */
#define RESP_RESP_TX_IDX 14   /* 4 bytes: when the responder sent the response */

static uint8_t rx_buf[24];

/* ---------- helpers ---------- */

static void put_ts(uint8_t *field, uint64_t ts)
{
    for (int i = 0; i < 4; i++) {
        field[i] = (uint8_t)(ts >> (8 * i));
    }
}

static uint32_t get_ts(const uint8_t *field)
{
    uint32_t ts = 0;

    for (int i = 0; i < 4; i++) {
        ts |= (uint32_t)field[i] << (8 * i);
    }
    return ts;
}

/* Wait until one of the status bits in 'mask' is set; returns the status.
 * Gives up after 'timeout_ms' and returns 0. */
static uint32_t wait_status(uint32_t mask, uint32_t timeout_ms)
{
    uint32_t status;
    uint32_t start = k_uptime_get_32();

    while (!((status = dwt_readsysstatuslo()) & mask)) {
        if ((k_uptime_get_32() - start) > timeout_ms) {
            return 0;
        }
    }
    return status;
}

/* Reset the chip and read its ID. Returns 0 on success. */
static int uwb_probe(void)
{
    if (dw3000_hw_init() != 0) {
        printk("UWB: hw init failed (overlay missing or SPI not ready?)\n");
        return -1;
    }
    dw3000_hw_reset();
    k_msleep(5);

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        printk("UWB: probe failed - no DW3xxx chip answered on SPI\n");
        return -1;
    }

    uint32_t id = dwt_readdevid();

    printk("UWB: device ID 0x%08x %s\n", id,
           ((id & 0xFFFFFF00) == 0xDECA0300) ? "(OK)" : "(UNEXPECTED)");
    return ((id & 0xFFFFFF00) == 0xDECA0300) ? 0 : -1;
}

/* Full radio setup for ranging. Returns 0 on success. */
__unused static int uwb_setup(void)
{
    if (uwb_probe() != 0) {
        return -1;
    }

    /* Wait for the chip to reach IDLE_RC before talking to it further */
    for (int i = 0; !dwt_checkidlerc(); i++) {
        if (i > 100) {
            printk("UWB: chip never reached IDLE_RC\n");
            return -1;
        }
        k_msleep(1);
    }

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        printk("UWB: dwt_initialise failed\n");
        return -1;
    }
    dw3000_spi_speed_fast();

    if (dwt_configure(&config) != DWT_SUCCESS) {
        printk("UWB: dwt_configure failed (PLL did not lock)\n");
        return -1;
    }
    dwt_configuretxrf(&txconfig);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* D13 on the board now flashes on every UWB transmit / receive */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    printk("UWB: configured, channel %d\n", config.chan);
    return 0;
}

/* =====================================================================
 *                          INITIATOR (anchor)
 * ===================================================================== */
#if TEST_MODE == MODE_INITIATOR

#define MODE_NAME  "INITIATOR (anchor)"
#define mode_init  uwb_setup

static void run(void)
{
    uint8_t  seq = 0;
    uint32_t ok = 0, fail = 0;

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    while (1) {
        /* 1. Send the poll; the receiver switches on automatically after it */
        poll_msg[SEQ_IDX] = seq++;
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
        dwt_writetxfctrl(sizeof(poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* 2. Wait for the response, a timeout or an error */
        uint32_t status = wait_status(DWT_INT_RXFCG_BIT_MASK |
                                      SYS_STATUS_ALL_RX_TO |
                                      SYS_STATUS_ALL_RX_ERR, 100);
        bool valid = false;

        if (status == 0) {
            dwt_forcetrxoff();   /* chip reported nothing at all */
        }

        if (status & DWT_INT_RXFCG_BIT_MASK) {
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            uint8_t  rng = 0;
            uint16_t len = dwt_getframelength(&rng);

            if (len == sizeof(resp_msg)) {
                dwt_readrxdata(rx_buf, len, 0);
                rx_buf[SEQ_IDX] = 0;
                if (memcmp(rx_buf, resp_msg, COMMON_LEN) == 0) {
                    /* 3. Four timestamps -> time of flight -> distance */
                    uint32_t poll_tx = dwt_readtxtimestamplo32();
                    uint32_t resp_rx = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
                    uint32_t poll_rx = get_ts(&rx_buf[RESP_POLL_RX_IDX]);
                    uint32_t resp_tx = get_ts(&rx_buf[RESP_RESP_TX_IDX]);

                    /* The two boards' clocks differ slightly; correct for it */
                    double clk = (double)dwt_readclockoffset() /
                                 (double)(1UL << 26);
                    int32_t round_trip = (int32_t)(resp_rx - poll_tx);
                    int32_t reply_time = (int32_t)(resp_tx - poll_rx);
                    double tof = ((round_trip - reply_time * (1.0 - clk)) / 2.0) *
                                 DWT_TIME_UNITS;
                    int dist_cm = (int)(tof * SPEED_OF_LIGHT * 100.0);

                    ok++;
                    valid = true;
                    /* RANGE,<distance cm>,<status>,<uptime ms>,<ok>,<failed> */
                    printk("RANGE,%d,OK,%u,%u,%u\n", dist_cm,
                           k_uptime_get_32(), ok, fail);
                }
            }
        } else {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        if (!valid) {
            fail++;
            /* TIMEOUT  = nothing heard
             * ERROR    = something heard but it was corrupted
             * BADFRAME = a good frame, but not the expected response */
            printk("RANGE,-1,%s,%u,%u,%u\n",
                   (status & DWT_INT_RXFCG_BIT_MASK) ? "BADFRAME" :
                   (status & SYS_STATUS_ALL_RX_ERR)  ? "ERROR" : "TIMEOUT",
                   k_uptime_get_32(), ok, fail);
        }

        k_msleep(RANGING_PERIOD_MS);
    }
}

/* =====================================================================
 *                           RESPONDER (tag)
 * ===================================================================== */
#elif TEST_MODE == MODE_RESPONDER

#define MODE_NAME  "RESPONDER (tag)"
#define mode_init  uwb_setup

static void run(void)
{
    /* polls   = valid polls received      replies = responses sent
     * late    = response not sent: the scheduled time had already passed
     * rx_err  = something received but corrupted
     * other   = a good frame that was not our poll */
    uint32_t polls = 0, replies = 0, late = 0, rx_err = 0, other = 0;
    uint32_t last_print = 0;

    while (1) {
        /* Status line every 2 s, printed while the radio is idle */
        if ((k_uptime_get_32() - last_print) >= 2000) {
            last_print = k_uptime_get_32();
            printk("RESP,polls=%u,replies=%u,late=%u,rx_err=%u,other=%u\n",
                   polls, replies, late, rx_err, other);
        }

        /* 1. Listen for a poll (this test listens all the time) */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status = wait_status(DWT_INT_RXFCG_BIT_MASK |
                                      SYS_STATUS_ALL_RX_ERR, 500);

        if (status == 0) {
            dwt_forcetrxoff();   /* nothing heard: restart the receiver */
            continue;
        }
        if (!(status & DWT_INT_RXFCG_BIT_MASK)) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
            rx_err++;
            continue;
        }
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

        uint8_t  rng = 0;
        uint16_t len = dwt_getframelength(&rng);

        if (len != sizeof(poll_msg)) {
            other++;
            continue;
        }
        dwt_readrxdata(rx_buf, len, 0);

        uint8_t seq = rx_buf[SEQ_IDX];

        rx_buf[SEQ_IDX] = 0;
        if (memcmp(rx_buf, poll_msg, COMMON_LEN) != 0) {
            other++;
            continue;
        }
        polls++;

        /* 2. Schedule the response a fixed delay after the poll arrived */
        uint8_t ts[5];

        dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);

        uint64_t poll_rx = 0;

        for (int i = 4; i >= 0; i--) {
            poll_rx = (poll_rx << 8) | ts[i];
        }

        uint32_t resp_tx_time =
            (uint32_t)((poll_rx +
                        (uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME) >> 8);

        dwt_setdelayedtrxtime(resp_tx_time);

        /* The chip ignores the lowest bit of the start time */
        uint64_t resp_tx = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) +
                           TX_ANT_DLY;

        /* 3. Put both timestamps in the response and send it */
        resp_msg[SEQ_IDX] = seq;
        put_ts(&resp_msg[RESP_POLL_RX_IDX], poll_rx);
        put_ts(&resp_msg[RESP_RESP_TX_IDX], resp_tx);
        dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
        dwt_writetxfctrl(sizeof(resp_msg), 0, 1);

        if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
            wait_status(DWT_INT_TXFRS_BIT_MASK, 10);
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            replies++;
        } else {
            /* Too slow: the scheduled time had already passed.
             * If this counter grows, raise POLL_RX_TO_RESP_TX_DLY_UUS
             * (and the initiator's RX delay) on BOTH boards. */
            late++;
        }
    }
}

/* =====================================================================
 *                             DEVICE ID
 * ===================================================================== */
#else

#define MODE_NAME  "DEVICE ID only"
#define mode_init  uwb_probe

static void run(void)
{
    while (1) {
        k_sleep(K_SECONDS(5));
        printk("UWB: device ID 0x%08x\n", dwt_readdevid());
    }
}

#endif

int main(void)
{
    k_msleep(500);   /* let the serial terminal connect */
    printk("\nNES15 UWB test, mode: %s\n", MODE_NAME);

    if (mode_init() != 0) {
        printk("UWB: setup FAILED, stopping\n");
        return 0;
    }
    run();
    return 0;
}