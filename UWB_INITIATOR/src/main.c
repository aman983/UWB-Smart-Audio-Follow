#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>

#include <dw3000_spi.h>
#include <dw3000_hw.h>

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

static uint32_t wait_status(uint32_t mask, uint32_t timeout_ms)
{
        uint32_t status, start = k_uptime_get_32();

        while (!((status = dwt_readsysstatuslo()) & mask ))
        {
                if((k_uptime_get_32() - start) > timeout_ms)
                {
                        return 0;
                }
        } 
        return status;
}


static int uwb_probe(void)
{
        if(dw3000_hw_init() != 0)
        {
                printk("UWB HW init Error !!\n");
                return -1;
        }
        dw3000_hw_reset();
        k_msleep(5);

        if(dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS)
        {
                printk("UWB failed CHIP does not answer !!!");
                return -1;
        }
        uint32_t dev_id = dwt_readdevid();
 
        printk("UWB: device ID 0x%08x %s\n", dev_id,
           ((dev_id & 0xFFFFFF00) == 0xDECA0300) ? "(OK)" : "(UNEXPECTED)");
    return ((dev_id & 0xFFFFFF00) == 0xDECA0300) ? 0 : -1;
}


__unused static int uwb_setup(void)
{
        if(uwb_probe() != 0){
                return -1;
        }
        // wait for chip to reach idle
        for (int i=0; !dwt_checkidlerc(); i++){
                if(i>100){
                        printk("UWB: chip IDLE ERROR \n");
                        return -1;
                }
                k_msleep(1);
        }
        if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
                printk("UWB: dwt_initialise ERROR\n");
                return -1;
        }
        dw3000_spi_speed_fast();

        if(dwt_configure(&config) != DWT_SUCCESS)
        {
                printk("UWB: dwt configure ERROR !!\n");
                return -1;
        }
        

        dwt_configuretxrf(&txconfig);
        dwt_setrxantennadelay(RX_ANT_DLY);
        dwt_settxantennadelay(TX_ANT_DLY);

        // Blink led on tx and Rx
        dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

        printk("UWB: configured, channel $d\n", config.chan);
        return 0;
}


static void Node_initiator_run(void)
{
        uint8_t seq = 0;
        uint32_t ok =0, fail = 0;

        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

        while(1)
        {
                /* send the poll the recievier starts to respond automatically */

                poll_msg[SEQ_IDX] = seq++;
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
                dwt_writetxfctrl(sizeof(poll_msg), 0, 1);
                dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                /* wait for response or a timeout */

                uint32_t status = wait_status(DWT_INT_RXFCG_BIT_MASK |
                                              SYS_STATUS_ALL_RX_TO | 
                                              SYS_STATUS_ALL_RX_ERR, 100);
                
                bool valid = false;

                if(status == 0){
                        dwt_forcetrxoff(); /* chip reported nothing at all */
                        printk("Timeout reached !!! \n");
                }

                if(status & DWT_INT_RXFCG_BIT_MASK){
                        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

                        uint8_t rng = 0;
                        uint16_t len = dwt_getframelength(&rng);

                        if(len == sizeof(resp_msg)){
                                dwt_readrxdata(rx_buf, len, 0);
                                rx_buf[SEQ_IDX] = 0;

                                if(memcmp(rx_buf, resp_msg, COMMON_LEN) == 0){

                                        uint32_t poll_tx = dwt_readtxtimestamplo32();
                                        uint32_t resp_rx = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
                                        uint32_t poll_rx = get_ts(&rx_buf[RESP_POLL_RX_IDX]);
                                        uint32_t resp_tx = get_ts(&rx_buf[RESP_RESP_TX_IDX]);

                                        /* correct for clk drift */
                                        double clk = (double) dwt_readclockoffset() /
                                                     (double) (1UL << 26);
                                        
                                        int32_t round_trip = (int32_t)(resp_rx - poll_tx);
                                        int32_t reply_time = (int32_t)(resp_tx - poll_rx);
                                        double tof = ((round_trip - reply_time * (1.0 - clk)) / 2.0) * 
                                                       DWT_TIME_UNITS;
                                        int dist_cm = (int)(tof * SPEED_OF_LIGHT * 100.0);

                                        ok++;

                                        valid = true;

                                        printk("RANGE   => %d\nTime    => %u\nOK      => %u\nFailed  =>%u\n",
                                        dist_cm, k_uptime_get_32(), ok, fail);
                                }
                        }else{
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
}


int main(void)
{
        k_msleep(500);
        printk("\n UWB INITIATOR \n");
        if(uwb_setup() != 0){
                printk("UWB init failed !!!\n");
                return 0;
        }
        Node_initiator_run();
        return 0;
}
