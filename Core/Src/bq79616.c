/*
 * bq79616.c  –  BQ79616 battery monitor driver (implementation)
 *
 * FILE LOCATION: Core/Src/bq79616.c
 *
 * This file implements the low-level communication with the
 * BQ79616 chip over USART6.  Think of it like a library that
 * hides all the messy UART framing and CRC details from you.
 *
 * How communication works (simplified):
 *   1. Build a "command frame" (a small byte array) that says
 *      what register you want to read or write.
 *   2. Append a CRC-16 checksum so the chip can verify the data.
 *   3. Send the frame over UART TX.
 *   4. Wait for the chip to send back a "response frame" on UART RX.
 *   5. Verify the response CRC, then extract the data bytes.
 *
 * FRAME FORMAT (learned from debugging + your working partid_test.c):
 *
 *   TX command (single read, 1 byte):
 *     [INIT] [DEV] [REG_HI] [REG_LO] [COUNT] [CRC_LO] [CRC_HI]
 *       0x80  addr   reg>>8   reg&FF    0x00    crc&FF   crc>>8
 *
 *   RX response (single read, 1 byte):
 *     [INIT] [DEV] [REG_HI] [REG_LO] [DATA] [CRC_LO] [CRC_HI]
 *       0x00  addr   reg>>8   reg&FF   value   crc&FF   crc>>8
 *     Total: 7 bytes.  Data is at index 4.
 *     CRC verified by running CRC over all 7 bytes — result = 0x0000.
 *
 * CRC ALGORITHM:
 *   CRC-16-IBM, reflected (LSB-first), polynomial 0xA001,
 *   init 0xFFFF.  Matching your proven partid_test.c exactly.
 *
 * DEBUG OUTPUT:
 *   All debug messages are sent out USART1.
 *   Connect a USB-serial adapter to PA9 (TX) to see them.
 */

#include "bq79616.h"
#include "main.h"       /* Gives us GPIO defines, HAL functions          */
#include "board.h"      /* uart1_dma_send for DMA terminal output        */
#include <string.h>     /* For memset, strlen                            */
#include <stdio.h>      /* For snprintf                                  */

/* ================================================================
 *  EXTERNAL HANDLES
 *
 *  huart6 is the UART connected to the BQ79616 (1 Mbps).
 *  Terminal output uses uart1_dma_send() from board.c, so
 *  huart1 is no longer needed directly in this file.
 * ================================================================ */
extern UART_HandleTypeDef huart6;

/* ================================================================
 *  GLOBAL DEBUG FLAG
 *
 *  Set this to true to see all BQ79616 driver debug output.
 *  Set to false to silence it — useful once your driver is
 *  working and you want a clean terminal for application data.
 *
 *  You can toggle it from application.c at any time:
 *    sys_debug = false;  // Hush the driver
 *    bq79616_init(0x01);
 *    sys_debug = true;   // Turn debug back on
 * ================================================================ */
bool sys_debug = true;

/* One-line fault summary — updated by bq79616_get_status().
 * Read by the display engine via bq79616_get_fault_line(). */
static char fault_line_buf[64] = "FAULT: ---";

/* ================================================================
 *  DEBUG HELPERS
 *
 *  dbg() sends a string out USART1 so you can see what's
 *  happening on a serial terminal (like Arduino Serial Monitor).
 *  Uses DMA so the CPU is freed while bytes are clocked out.
 * ================================================================ */
static void dbg(const char *msg)
{
    if (!sys_debug) { return; }  /* Silent when debug is off */
    uart1_dma_send((const uint8_t *)msg,
                   (uint16_t)strlen(msg));
}

/*
 * dbg_hex  –  Print a label and a hex byte, e.g. "PARTID = 0x21\r\n"
 */
static void dbg_hex(const char *label, uint8_t val)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "%s0x%02X\r\n", label, val);
    dbg(buf);
}

/*
 * dbg_frame  –  Dump a byte array in hex, e.g. "TX: 80 01 05 00 00 34 23"
 */
static void dbg_frame(const char *label, const uint8_t *data, uint16_t len)
{
    char buf[80];
    int pos;

    pos = snprintf(buf, sizeof(buf), "%s", label);

    for (uint16_t i = 0; i < len && pos < 70; i++)
    {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
    }

    snprintf(buf + pos, sizeof(buf) - pos, "\r\n");
    dbg(buf);
}

/* ================================================================
 *  PRIVATE (STATIC) HELPER FUNCTIONS
 *  These are only used inside this file – not visible outside.
 * ================================================================ */

/*
 * bq79616_crc16  –  Calculate CRC-16 for a byte array
 *
 * Identical to the algorithm in your working partid_test.c.
 *
 * Algorithm details:
 *   - Reflected CRC-16-IBM (LSB-first processing)
 *   - Polynomial: 0xA001  (reflected form of 0x8005)
 *   - Initial value: 0xFFFF
 *   - Process each byte starting from bit 0 (LSB)
 *   - No final XOR or byte-swap
 *
 * Two ways to use it:
 *   1. TX: compute CRC over payload, append result to frame
 *   2. RX: compute CRC over ENTIRE frame (including CRC bytes).
 *          If the frame is valid, the result is 0x0000.
 *
 * Parameters:
 *   data – pointer to the byte array to checksum
 *   len  – number of bytes in the array
 *
 * Returns:
 *   Raw 16-bit CRC value (not byte-swapped)
 */
static uint16_t bq79616_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;   /* Start with all bits set              */
    uint16_t i;
    uint8_t  bit;

    for (i = 0; i < len; i++)
    {
        /*
         * XOR the next byte into the LOW 8 bits of the CRC.
         * (Reflected: we work from the LSB side, not the MSB.)
         */
        crc ^= (uint16_t)data[i];

        /* Process each of the 8 bits in this byte, LSB first */
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                /*
                 * Bottom bit is 1: shift RIGHT and XOR
                 * with the reflected polynomial 0xA001.
                 */
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                /* Bottom bit is 0, just shift right */
                crc = crc >> 1;
            }
        }
    }

    return crc;   /* Raw reflected result, no swap */
}


/*
 * bq79616_flush_rx  –  Discard any stale bytes in the UART RX FIFO
 *
 * After the wake ping (GPIO toggling), there may be garbage
 * bytes sitting in the UART receive register from electrical
 * noise during the pin transition.  If we don't flush them,
 * HAL_UART_Receive would pick them up as the start of our
 * response and everything would be misaligned.
 *
 * Think of it like clearing the Arduino Serial buffer with
 * while(Serial.available()) Serial.read();
 */
static void bq79616_flush_rx(void)
{
    /*
     * Read and discard bytes as long as the RX-Not-Empty flag
     * is set.  RDR is the Receive Data Register on STM32G0.
     */
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE))
    {
        (void)huart6.Instance->RDR;
    }

    /* Also clear any overrun error that might have built up */
    __HAL_UART_CLEAR_OREFLAG(&huart6);
}


/*
 * bq79616_wake_ping  –  Send a WAKE ping to bring the chip to ACTIVE mode
 *
 * The BQ79616 starts in SHUTDOWN mode (lowest power).
 * To wake it up, we must pull its RX pin LOW for at least 2 ms
 * (the datasheet calls this tHLD_WAKE = 2 to 2.5 ms).
 *
 * Since the BQ79616's RX pin is connected to our USART6 TX (PC0),
 * we temporarily turn PC0 into a regular GPIO output, pull it low,
 * wait, then pull it high again.  After that, we re-initialise
 * USART6 so it can do normal UART communication again.
 */
static void bq79616_wake_ping(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* STEP 1: Turn off USART6 so we can manually control the TX pin */
    HAL_UART_DeInit(&huart6);

    /* STEP 2: Configure PC0 (USART6_TX) as a plain GPIO output */
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* STEP 3: Pull LOW for ~3 ms (wake ping) then release HIGH */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(BQ_WAKE_PING_LOW_MS);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    /* STEP 4: Wait for chip power-up (tSU(WAKE_SHUT) max 10 ms) */
    HAL_Delay(BQ_WAKE_SETTLE_MS);

    /* STEP 5: Re-initialise USART6 for normal 1 Mbps comms */
    huart6.Instance                    = USART6;
    huart6.Init.BaudRate               = 1000000;
    huart6.Init.WordLength             = UART_WORDLENGTH_8B;
    huart6.Init.StopBits               = UART_STOPBITS_1;
    huart6.Init.Parity                 = UART_PARITY_NONE;
    huart6.Init.Mode                   = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart6.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart6.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart6);

    /* STEP 6: Flush any junk from the RX FIFO */
    bq79616_flush_rx();
}


/*
 * bq79616_single_read  –  Read one register byte from a specific device
 *
 * TX command frame (7 bytes):
 *   [INIT=0x80] [DEV] [REG_HI] [REG_LO] [COUNT=0x00] [CRC_LO] [CRC_HI]
 *
 * RX response frame (7 bytes for 1-byte read):
 *   [INIT=0x00] [DEV] [REG_HI] [REG_LO] [DATA] [CRC_LO] [CRC_HI]
 *
 * CRC is verified by running CRC over the entire 7-byte response.
 * A valid frame produces CRC == 0x0000.  This matches the
 * technique in your proven partid_test.c.
 *
 * Parameters:
 *   dev_addr – 6-bit device address (0x01 for your chip)
 *   reg_addr – 16-bit register address
 *   out_data – pointer to store the single byte we read back
 *
 * Returns:
 *   true  – got a valid response with good CRC
 *   false – UART timeout or CRC mismatch
 */
static bool bq79616_single_read(uint8_t dev_addr, uint16_t reg_addr,
                                uint8_t *out_data)
{
    /* ---------- Build the 7-byte command frame ---------- */

    uint8_t  cmd[7];
    uint16_t crc;

    cmd[0] = BQ_FRAME_TYPE_CMD | BQ_REQ_SINGLE_READ;  /* 0x80 */
    cmd[1] = dev_addr & 0x3F;
    cmd[2] = (uint8_t)(reg_addr >> 8);       /* REG high byte */
    cmd[3] = (uint8_t)(reg_addr & 0xFF);     /* REG low byte  */
    cmd[4] = 0x00;                           /* Read 1 byte   */

    /* CRC over bytes 0..4, then append low-byte-first
     * (matching your partid_test.c convention)             */
    crc    = bq79616_crc16(cmd, 5);
    cmd[5] = (uint8_t)(crc & 0xFF);          /* CRC low byte  */
    cmd[6] = (uint8_t)(crc >> 8);            /* CRC high byte */

    dbg_frame("  TX: ", cmd, 7);

    /* ---------- Send the command ---------- */

    bq79616_flush_rx();

    if (HAL_UART_Transmit(&huart6, cmd, sizeof(cmd),
                          BQ_UART_TIMEOUT_MS) != HAL_OK)
    {
        dbg("  TX failed!\r\n");
        return false;
    }

    /* ---------- Receive the 7-byte response ---------- */

    /*
     * Response layout for a 1-byte single read:
     *   Byte 0: INIT  (0x00 = response, 1 data byte)
     *   Byte 1: DEV   (echoed device address)
     *   Byte 2: REG_HI (echoed register high byte)
     *   Byte 3: REG_LO (echoed register low byte)
     *   Byte 4: DATA  (the register value we want)
     *   Byte 5: CRC_LO
     *   Byte 6: CRC_HI
     */
    uint8_t resp[7];
    memset(resp, 0, sizeof(resp));

    if (HAL_UART_Receive(&huart6, resp, sizeof(resp),
                         BQ_UART_TIMEOUT_MS) != HAL_OK)
    {
        dbg("  RX timeout (no response)\r\n");
        return false;
    }

    dbg_frame("  RX: ", resp, 7);

    /* ---------- Verify the response CRC ---------- */

    /*
     * Run CRC over the ENTIRE 7-byte response (data + CRC bytes).
     * If the frame is intact, the result is exactly 0x0000.
     * This is the same verification method your partid_test.c uses.
     */
    uint16_t check = bq79616_crc16(resp, 7);

    if (check != 0x0000)
    {
        char buf[48];
        snprintf(buf, sizeof(buf),
                 "  CRC fail! check=0x%04X\r\n", check);
        dbg(buf);
        return false;
    }

    /* ---------- Extract the data byte ---------- */

    *out_data = resp[4];   /* Data is at index 4 (after INIT, DEV, REG) */

    return true;
}


/*
 * bq79616_single_write  –  Write one byte to a register on a specific device
 *
 * TX command frame (7 bytes):
 *   [INIT=0x90] [DEV] [REG_HI] [REG_LO] [DATA] [CRC_LO] [CRC_HI]
 *
 * Write commands produce no response from the chip.
 * To confirm the write took, the caller reads the register back.
 *
 * Parameters:
 *   dev_addr – 6-bit device address
 *   reg_addr – 16-bit register address to write to
 *   data     – the single byte value to write
 *
 * Returns:
 *   true  – UART transmit completed OK
 *   false – UART transmit failed
 */
static bool bq79616_single_write(uint8_t dev_addr, uint16_t reg_addr,
                                 uint8_t data)
{
    uint8_t  cmd[7];
    uint16_t crc;

    cmd[0] = BQ_FRAME_TYPE_CMD | BQ_REQ_SINGLE_WRITE;  /* 0x90 */
    cmd[1] = dev_addr & 0x3F;
    cmd[2] = (uint8_t)(reg_addr >> 8);
    cmd[3] = (uint8_t)(reg_addr & 0xFF);
    cmd[4] = data;

    crc    = bq79616_crc16(cmd, 5);
    cmd[5] = (uint8_t)(crc & 0xFF);          /* CRC low byte  */
    cmd[6] = (uint8_t)(crc >> 8);            /* CRC high byte */

    dbg_frame("  TX: ", cmd, 7);

    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(&huart6, cmd, sizeof(cmd),
                               BQ_UART_TIMEOUT_MS);

    /* Small delay for the chip to process the write */
    HAL_Delay(1);

    return (status == HAL_OK);
}


/*
 * bq79616_broadcast_write  –  Write one byte to ALL devices (no addressing)
 *
 * A broadcast write hits every device on the bus regardless of
 * what address they have.  The datasheet says this command works
 * "without auto-addressing" — meaning we can use it immediately
 * after a wake ping, before we even know the chip's address.
 *
 * This is critical for the safety timeout: we want the shutdown
 * timer armed ASAP after wake, before anything else can go wrong.
 *
 * TX frame (6 bytes — note: NO device address byte):
 *   [INIT=0xD0] [REG_HI] [REG_LO] [DATA] [CRC_LO] [CRC_HI]
 *
 *   INIT = 0xD0:
 *     Bit 7   = 1      (command frame)
 *     Bits 6-4 = 101   (Broadcast Write)
 *     Bit 3   = 0      (reserved)
 *     Bits 2-0 = 000   (DATA_SIZE = 1 byte)
 *
 * No response is returned for any write command.
 *
 * Parameters:
 *   reg_addr – 16-bit register address to write to
 *   data     – the single byte value to write
 *
 * Returns:
 *   true  – UART transmit completed OK
 *   false – UART transmit failed
 */
static bool bq79616_broadcast_write(uint16_t reg_addr, uint8_t data)
{
    uint8_t  cmd[6];     /* 4 payload bytes + 2 CRC bytes            */
    uint16_t crc;

    cmd[0] = BQ_FRAME_TYPE_CMD | BQ_REQ_BROADCAST_WRITE;  /* 0xD0  */
    cmd[1] = (uint8_t)(reg_addr >> 8);       /* REG high byte        */
    cmd[2] = (uint8_t)(reg_addr & 0xFF);     /* REG low byte         */
    cmd[3] = data;                           /* The value to write   */

    /* CRC over bytes 0..3, then append low-byte-first */
    crc    = bq79616_crc16(cmd, 4);
    cmd[4] = (uint8_t)(crc & 0xFF);          /* CRC low byte         */
    cmd[5] = (uint8_t)(crc >> 8);            /* CRC high byte        */

    dbg_frame("  TX(bcast): ", cmd, 6);

    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(&huart6, cmd, sizeof(cmd),
                               BQ_UART_TIMEOUT_MS);

    /* Small delay for the chip to process the write */
    HAL_Delay(1);

    return (status == HAL_OK);
}


/* ================================================================
 *  SHARED HELPERS (used by init AND GPIO functions)
 * ================================================================ */

/*
 * bq79616_update_otp_crc  –  Sync the customer OTP CRC after shadow register writes
 *
 * WHEN TO CALL:
 *   After writing ANY NVM shadow register in the range 0x0000–0x0035.
 *   These include GPIO_CONF1-4, COMM_TIMEOUT_CONF, ACTIVE_CELL,
 *   OV_THRESH, UV_THRESH, OTUT_THRESH, FAULT_MSK1/2, etc.
 *
 * WHY:
 *   The BQ79616 continuously computes a CRC over all customer OTP
 *   shadow registers and compares the result against CUST_CRC_HI/LO.
 *   If you change any shadow register, the computed CRC changes but
 *   the stored expected CRC doesn't → CUST_CRC fault fires.
 *
 * HOW:
 *   1. Read CUST_CRC_RSLT_HI/LO (the device's freshly computed CRC)
 *   2. Write those values into CUST_CRC_HI/LO (the expected CRC)
 *   Now both sides agree and the fault clears (or never fires).
 *
 * Parameters:
 *   dev_addr – the device address (0x01 per your OTP config)
 *
 * Returns:
 *   true  – CRC updated successfully
 *   false – a read or write failed
 */
static bool bq79616_update_otp_crc(uint8_t dev_addr)
{
    uint8_t crc_hi = 0;
    uint8_t crc_lo = 0;

    /* Read what the device computed for the current shadow state */
    if (!bq79616_single_read(dev_addr, BQ_REG_CUST_CRC_RSLT_HI, &crc_hi) ||
        !bq79616_single_read(dev_addr, BQ_REG_CUST_CRC_RSLT_LO, &crc_lo))
    {
        dbg("  CRC: could not read RSLT registers!\r\n");
        return false;
    }

    /* Write it into the expected-CRC registers so they match */
    if (!bq79616_single_write(dev_addr, BQ_REG_CUST_CRC_HI, crc_hi) ||
        !bq79616_single_write(dev_addr, BQ_REG_CUST_CRC_LO, crc_lo))
    {
        dbg("  CRC: could not write HI/LO registers!\r\n");
        return false;
    }

    return true;
}


/*
 * bq79616_gpio_conf_reg  –  Return the GPIO_CONF register address for a pin
 *
 * The 8 GPIO pins are packed 2 per register across 4 registers:
 *   GPIO_CONF1 (0x0E): GPIO1 in bits [2:0], GPIO2 in bits [5:3]
 *   GPIO_CONF2 (0x0F): GPIO3 in bits [2:0], GPIO4 in bits [5:3]
 *   GPIO_CONF3 (0x10): GPIO5 in bits [2:0], GPIO6 in bits [5:3]
 *   GPIO_CONF4 (0x11): GPIO7 in bits [2:0], GPIO8 in bits [5:3]
 *
 * Parameters:
 *   pin   – GPIO pin number 1 to 8
 *   shift – pointer to receive the bit-shift (0 or 3)
 *
 * Returns:
 *   register address (0x000E to 0x0011)
 */
static uint16_t bq79616_gpio_conf_reg(uint8_t pin, uint8_t *shift)
{
    /*
     * Odd pins (1,3,5,7) sit in bits [2:0] → shift = 0
     * Even pins (2,4,6,8) sit in bits [5:3] → shift = 3
     */
    *shift = (pin % 2 == 0) ? 3 : 0;

    /*
     * Pins 1-2 → CONF1 (0x0E), pins 3-4 → CONF2 (0x0F), etc.
     * Formula: base address + (pin-1)/2
     */
    return BQ_REG_GPIO_CONF1 + ((pin - 1) / 2);
}


/* ================================================================
 *  PUBLIC FUNCTIONS
 * ================================================================ */

/*
 * bq79616_init  –  Wake the BQ79616, arm safety timeout, then verify comms
 *
 * IMPORTANT: The order of operations is designed to minimise the
 * time window where the chip is awake but has no shutdown timer.
 *
 *   Step 1: Wake ping (SHUTDOWN → ACTIVE)
 *   Step 2: IMMEDIATELY broadcast-write COMM_TIMEOUT_CONF = 0x0B
 *           This arms the 10 s → SHUTDOWN safety timer.
 *           Uses broadcast (not single-device) so it works without
 *           knowing the chip's address — no auto-addressing needed.
 *   Step 3: Read PARTID to verify the chip is a BQ79616
 *   Step 4: Read back COMM_TIMEOUT_CONF to confirm the write took
 *   Step 5: Fix customer OTP CRC (our write to COMM_TIMEOUT_CONF
 *           changed a CRC-covered register — we must update the
 *           expected CRC to match the device's computed CRC)
 *   Step 6: Clear all latched faults (DRST from wake + CUST_CRC)
 *   Step 7: Verify FAULT_SUMMARY == 0x00 (NFAULT pin released)
 *
 * Why this order matters:
 *   After a wake ping, the COMM_TIMEOUT_CONF register loads from
 *   OTP, which defaults to 0x00 (timeout DISABLED).  If the MCU
 *   loses power between the wake ping and our timeout write, the
 *   chip would stay in ACTIVE mode forever, draining the battery.
 *   By making the broadcast write the very first thing after wake,
 *   we shrink that vulnerable window to just a few microseconds
 *   (the time for one 6-byte UART frame at 1 Mbps = ~60 µs).
 *
 * For absolute protection (zero-risk), you could programme the
 * timeout value into OTP so it's always armed on power-up.  But
 * OTP is one-time-programmable and cannot be undone, so this
 * software approach is the practical choice during development.
 *
 * LOCATION: Called from application_setup() in application.c:
 *     bool ok = bq79616_init(0x01);
 *
 * Parameters:
 *   dev_addr – the device address (0x01 per your OTP config)
 *
 * Returns:
 *   true  – chip is alive, identified, AND timeout is verified
 *   false – any step failed
 */
bool bq79616_init(uint8_t dev_addr)
{
    uint8_t part_id  = 0x00;
    uint8_t readback = 0x00;

    dbg("\r\n=== BQ79616 INIT START ===\r\n");

    /* ---- Step 1: Wake ping ---- */
    dbg("Step 1: Wake ping...\r\n");
    bq79616_wake_ping();
    dbg("  Done.\r\n");

    /*
     * ---- Step 2: Arm the safety timeout IMMEDIATELY ----
     *
     * This is a broadcast write — it hits every device on the
     * bus regardless of address, and works right after wake
     * without auto-addressing.
     *
     * COMM_TIMEOUT_CONF = 0x0B means:
     *   CTL_ACT  = 1   (action = SHUTDOWN, not just SLEEP)
     *   CTL_TIME = 011 (10-second countdown)
     *
     * From this point on, if the MCU dies or stops talking,
     * the BQ79616 will shut itself down after 10 seconds.
     * Every valid UART frame we send resets the countdown.
     */
    dbg("Step 2: Broadcast-write safety timeout...\r\n");

    if (!bq79616_broadcast_write(BQ_REG_COMM_TIMEOUT_CONF,
                                 BQ_COMM_TIMEOUT_10S_SHTDN))
    {
        dbg("  FAIL: Broadcast write did not transmit!\r\n");
        return false;
    }

    dbg("  Safety timer armed (10 s -> SHUTDOWN).\r\n");

    /* ---- Step 3: Read PARTID (0x0500), expect 0x21 ---- */
    dbg_hex("Step 3: Read PARTID at addr ", dev_addr);

    if (!bq79616_single_read(dev_addr, BQ_REG_PARTID, &part_id))
    {
        dbg("  FAIL: No valid response!\r\n");
        return false;
    }

    dbg_hex("  PARTID = ", part_id);

    if (part_id != BQ79616_PARTID_VALUE)
    {
        dbg("  FAIL: Wrong part ID!\r\n");
        return false;
    }

    dbg("  OK\r\n");

    /*
     * ---- Step 4: Read back COMM_TIMEOUT_CONF to verify ----
     *
     * We must confirm the broadcast write actually took.
     * init() cannot return true unless this safety net is armed.
     */
    dbg("Step 4: Verify COMM_TIMEOUT_CONF...\r\n");

    if (!bq79616_single_read(dev_addr,
                             BQ_REG_COMM_TIMEOUT_CONF,
                             &readback))
    {
        dbg("  FAIL: Read-back failed!\r\n");
        return false;
    }

    dbg_hex("  Read back = ", readback);

    if (readback != BQ_COMM_TIMEOUT_10S_SHTDN)
    {
        dbg("  FAIL: Expected 0x0B!\r\n");
        return false;
    }

    dbg("  Timeout verified.\r\n");

    /*
     * ---- Step 5: Fix the customer OTP CRC ----
     *
     * Our broadcast write to COMM_TIMEOUT_CONF (0x0019) changed a
     * CRC-covered shadow register.  We must update the expected CRC
     * so the background checker doesn't fire CUST_CRC fault.
     * This uses the shared helper that GPIO functions also call.
     */
    dbg("Step 5: Fix customer OTP CRC...\r\n");

    if (!bq79616_update_otp_crc(dev_addr))
    {
        dbg("  FAIL: CRC update failed!\r\n");
        return false;
    }

    dbg("  CRC updated.\r\n");

    /*
     * ---- Step 6: Clear all latched faults ----
     *
     * After wake from SHUTDOWN, several faults are expected:
     *   - FAULT_SYS[DRST] = digital reset occurred (always set)
     *   - FAULT_OTP[CUST_CRC] = was set before we fixed it above
     *
     * We clear ALL fault groups by writing 0xFF to both reset
     * registers.  Each bit in FAULT_RST1/RST2 clears a group
     * of fault registers.  The bits self-clear after writing.
     *
     * FAULT_RST1 (0x0331):
     *   bit 7 = RST_PROT  (clears FAULT_PROT1/2)
     *   bit 6 = RST_UT    (clears FAULT_UT)
     *   bit 5 = RST_OT    (clears FAULT_OT)
     *   bit 4 = RST_UV    (clears FAULT_UV1/2)
     *   bit 3 = RST_OV    (clears FAULT_OV1/2)
     *   bit 2 = RST_COMP  (clears all FAULT_COMP_*)
     *   bit 1 = RST_SYS   (clears FAULT_SYS → clears DRST)
     *   bit 0 = RST_PWR   (clears FAULT_PWR1/2/3)
     *
     * FAULT_RST2 (0x0332):
     *   bit 6 = RST_OTP_CRC   (clears CUST_CRC, FACT_CRC)
     *   bit 5 = RST_OTP_DATA  (clears SEC_DET, DED_DET)
     *   bit 4 = RST_COMM3_FCOMM
     *   bit 3 = RST_COMM3_FTONE
     *   bit 2 = RST_COMM3_HB
     *   bit 1 = RST_COMM2
     *   bit 0 = RST_COMM1
     *
     * This also de-asserts the NFAULT pin (if no active fault
     * condition persists).
     */
    dbg("Step 6: Clear all latched faults...\r\n");

    if (!bq79616_single_write(dev_addr, BQ_REG_FAULT_RST1, 0xFF))
    {
        dbg("  FAIL: Could not write FAULT_RST1!\r\n");
        return false;
    }

    if (!bq79616_single_write(dev_addr, BQ_REG_FAULT_RST2, 0xFF))
    {
        dbg("  FAIL: Could not write FAULT_RST2!\r\n");
        return false;
    }

    dbg("  Faults cleared.\r\n");

    /*
     * ---- Step 7: Verify FAULT_SUMMARY is clean ----
     *
     * Short delay to let the background CRC checker re-run,
     * then read FAULT_SUMMARY.  It should be 0x00 now.
     * If any bit is still set, a real fault persists.
     */
    dbg("Step 7: Verify FAULT_SUMMARY...\r\n");
    HAL_Delay(5);   /* Give CRC checker time to re-evaluate */

    if (!bq79616_single_read(dev_addr, BQ_REG_FAULT_SUMMARY, &readback))
    {
        dbg("  FAIL: Could not read FAULT_SUMMARY!\r\n");
        return false;
    }

    dbg_hex("  FAULT_SUMMARY = ", readback);

    if (readback != 0x00)
    {
        dbg("  WARNING: Faults persist after clear!\r\n");
        dbg("  Run bq79616_get_status() for details.\r\n");
    }
    else
    {
        dbg("  All clear.\r\n");
    }

    dbg("=== BQ79616 INIT OK ===\r\n\r\n");

    return true;
}


/* ================================================================
 *  FAULT STATUS DECODE HELPERS (static / private)
 *
 *  Each helper reads one or more sub-registers for a fault group
 *  and prints which bits are set.  These are only called when
 *  FAULT_SUMMARY tells us there IS a fault in that group.
 * ================================================================ */

/*
 * decode_bits  –  Check each bit in a register value and print its name
 *
 * This is like a mini lookup table.  You give it:
 *   - val:   the byte you read from a register
 *   - names: an array of 8 strings, one per bit (bit 7 first)
 *   - count: how many bits to check (usually 8)
 *
 * For any bit that is SET (= 1), it prints "    <name>\r\n".
 * If a name is NULL, that bit is reserved and we skip it.
 *
 * Example: if val = 0x09 and names[7]="TSHUT", names[0]="TWARN",
 * it would print both of those.
 */
static void decode_bits(uint8_t val, const char * const names[], uint8_t count)
{
    uint8_t i;

    for (i = 0; i < count; i++)
    {
        /*
         * Check from the top bit down: bit 7, then 6, etc.
         * (0x80 >> 0) = 0x80 = bit 7
         * (0x80 >> 1) = 0x40 = bit 6  ... and so on
         */
        if ((val & (0x80 >> i)) && names[i] != NULL)
        {
            dbg("    ");
            dbg(names[i]);
            dbg("\r\n");
        }
    }
}


/*
 * decode_cell_faults  –  Print which cells (1-16) have a fault
 *
 * The BQ79616 splits 16 cells across two registers:
 *   reg1 = cells 16..9  (bit 7 = cell 16, bit 0 = cell 9)
 *   reg2 = cells 8..1   (bit 7 = cell 8,  bit 0 = cell 1)
 *
 * label is something like "OV" or "UV" so we print e.g. "OV Cell 3".
 */
static void decode_cell_faults(uint8_t reg1, uint8_t reg2, const char *label)
{
    char buf[32];
    uint8_t cell;

    /* reg1: bit 7 = cell 16, bit 0 = cell 9 */
    for (cell = 16; cell >= 9; cell--)
    {
        if (reg1 & (1 << (cell - 9)))
        {
            snprintf(buf, sizeof(buf), "    %s Cell %u\r\n", label, cell);
            dbg(buf);
        }
    }

    /* reg2: bit 7 = cell 8, bit 0 = cell 1 */
    for (cell = 8; cell >= 1; cell--)
    {
        if (reg2 & (1 << (cell - 1)))
        {
            snprintf(buf, sizeof(buf), "    %s Cell %u\r\n", label, cell);
            dbg(buf);
        }
    }
}


/*
 * decode_gpio_faults  –  Print which GPIOs (1-8) have a fault
 *
 * Single register: bit 7 = GPIO8, bit 0 = GPIO1.
 * label is "OT" or "UT".
 */
static void decode_gpio_faults(uint8_t val, const char *label)
{
    char buf[32];
    uint8_t gpio;

    for (gpio = 8; gpio >= 1; gpio--)
    {
        if (val & (1 << (gpio - 1)))
        {
            snprintf(buf, sizeof(buf), "    %s GPIO%u\r\n", label, gpio);
            dbg(buf);
        }
    }
}


/* ================================================================
 *  PUBLIC: bq79616_get_status
 * ================================================================ */

/*
 * bq79616_get_status  –  Read all fault registers and print any active faults
 *
 * This function follows the BQ79616 fault hierarchy:
 *
 *   1. Read FAULT_SUMMARY (the top-level register).
 *      Each bit represents a GROUP of related faults.
 *
 *   2. For any group that is flagged, drill down into the
 *      lower-level registers and print every individual fault.
 *
 *   3. If FAULT_SUMMARY is 0x00 (all clear), just print "No faults".
 *
 * The fault groups in FAULT_SUMMARY are (bit 7 → bit 0):
 *   FAULT_PROT     – hardware protector (comparator) failures
 *   FAULT_COMP_ADC – ADC comparison diagnostic failures
 *   FAULT_OTP      – OTP memory CRC or load errors
 *   FAULT_COMM     – UART / daisy-chain communication errors
 *   FAULT_OTUT     – overtemperature / undertemperature
 *   FAULT_OVUV     – cell overvoltage / undervoltage
 *   FAULT_SYS      – system: digital reset, timeout, thermal shutdown
 *   FAULT_PWR      – internal power supply rail failures
 *
 * NOTE: After a fresh wake from SHUTDOWN, FAULT_SYS[DRST] (digital
 * reset) is always set.  This is normal — it just means the chip
 * was reset.  The function flags it but it's expected behaviour.
 *
 * LOCATION: Call from application_loop() in application.c, or
 * anywhere you want a status check:
 *
 *     bool healthy = bq79616_get_status(0x01);
 *     if (!healthy)
 *     {
 *         // Handle faults...
 *     }
 *
 * Parameters:
 *   dev_addr – the device address (0x01 per your OTP config)
 *
 * Returns:
 *   true  – FAULT_SUMMARY == 0x00, no faults at all
 *   false – one or more faults detected (details printed to USART1)
 */
bool bq79616_get_status(uint8_t dev_addr)
{
    uint8_t summary = 0xFF;  /* Pre-fill so a read failure looks like faults */
    uint8_t reg_val = 0x00;
    uint8_t reg_val2 = 0x00;

    dbg("\r\n=== BQ79616 FAULT STATUS ===\r\n");

    /* ---- Read the top-level summary register ---- */

    if (!bq79616_single_read(dev_addr, BQ_REG_FAULT_SUMMARY, &summary))
    {
        dbg("  FAIL: Could not read FAULT_SUMMARY!\r\n");
        dbg("=== STATUS READ FAILED ===\r\n\r\n");
        snprintf(fault_line_buf, sizeof(fault_line_buf),
                 "FAULT: READ FAILED");
        return false;
    }

    dbg_hex("  FAULT_SUMMARY = ", summary);

    /* Quick exit if no faults at all */
    if (summary == 0x00)
    {
        dbg("  All clear — no faults.\r\n");
        dbg("=== STATUS OK ===\r\n\r\n");
        snprintf(fault_line_buf, sizeof(fault_line_buf),
                 "FAULT: OK");
        return true;
    }

    /* -----------------------------------------------------------
     * Drill down into each flagged group.
     * We only read sub-registers for groups that have a fault.
     * ----------------------------------------------------------- */

    /*
     * BIT 0: FAULT_PWR  –  Internal power supply faults
     *
     * These indicate problems with the chip's own power rails:
     * AVDD, DVDD, CVDD, TSREF, NEG5V, reference voltages.
     * A fault here usually means a hardware problem on the PCB.
     */
    if (summary & 0x01)
    {
        dbg("\r\n  [FAULT_PWR] Power supply fault:\r\n");

        /* FAULT_PWR1 (0x0552) */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_PWR1, &reg_val))
        {
            static const char * const pwr1_names[8] = {
                "CVSS_OPEN",       /* bit 7 */
                "DVSS_OPEN",       /* bit 6 */
                "REFHM_OPEN",      /* bit 5 */
                "CVDD_UV",         /* bit 4 */
                "CVDD_OV",         /* bit 3 */
                "DVDD_OV",         /* bit 2 */
                "AVDD_OSC",        /* bit 1 */
                "AVDD_OV"          /* bit 0 */
            };
            dbg_hex("    PWR1 = ", reg_val);
            decode_bits(reg_val, pwr1_names, 8);
        }

        /* FAULT_PWR2 (0x0553) */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_PWR2, &reg_val))
        {
            static const char * const pwr2_names[8] = {
                NULL,              /* bit 7: reserved */
                "PWRBIST_FAIL",    /* bit 6 */
                NULL,              /* bit 5: reserved */
                "REFH_OSC",        /* bit 4 */
                "NEG5V_UV",        /* bit 3 */
                "TSREF_OSC",       /* bit 2 */
                "TSREF_UV",        /* bit 1 */
                "TSREF_OV"         /* bit 0 */
            };
            dbg_hex("    PWR2 = ", reg_val);
            decode_bits(reg_val, pwr2_names, 8);
        }

        /* FAULT_PWR3 (0x0554) */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_PWR3, &reg_val))
        {
            if (reg_val & 0x08)    /* bit 3 = AVDDUV_DRST */
            {
                dbg_hex("    PWR3 = ", reg_val);
                dbg("    AVDDUV_DRST (AVDD UV caused reset)\r\n");
            }
        }
    }

    /*
     * BIT 1: FAULT_SYS  –  System-level events
     *
     * Includes digital reset (DRST) which is NORMAL after wake,
     * communication timeouts (CTL/CTS), thermal shutdown/warning,
     * LFO oscillator fault, and GPIO8 fault input.
     */
    if (summary & 0x02)
    {
        dbg("\r\n  [FAULT_SYS] System fault:\r\n");

        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_SYS, &reg_val))
        {
            static const char * const sys_names[8] = {
                "LFO (oscillator out of range)",        /* bit 7 */
                NULL,                                   /* bit 6: reserved */
                "GPIO (fault input on GPIO8)",          /* bit 5 */
                "DRST (digital reset occurred)",        /* bit 4 */
                "CTL (long comm timeout)",              /* bit 3 */
                "CTS (short comm timeout warning)",     /* bit 2 */
                "TSHUT (thermal shutdown!)",             /* bit 1 */
                "TWARN (thermal warning)"               /* bit 0 */
            };
            dbg_hex("    SYS = ", reg_val);
            decode_bits(reg_val, sys_names, 8);

            /* DRST is expected after every wake from SHUTDOWN */
            if ((reg_val & 0x10) && !(reg_val & 0xEF))
            {
                dbg("    (DRST alone is normal after wake)\r\n");
            }
        }
    }

    /*
     * BIT 2: FAULT_OVUV  –  Cell overvoltage / undervoltage
     *
     * These are the critical battery safety faults.
     * OV means a cell exceeded the OV threshold.
     * UV means a cell dropped below the UV threshold.
     * Each cell (1-16) has its own flag.
     */
    if (summary & 0x04)
    {
        dbg("\r\n  [FAULT_OVUV] Cell voltage fault:\r\n");

        /* OV: split across FAULT_OV1 (cells 16-9) and FAULT_OV2 (cells 8-1) */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_OV1, &reg_val) &&
            bq79616_single_read(dev_addr, BQ_REG_FAULT_OV2, &reg_val2))
        {
            if (reg_val || reg_val2)
            {
                dbg("    Overvoltage detected:\r\n");
                decode_cell_faults(reg_val, reg_val2, "OV");
            }
        }

        /* UV: split across FAULT_UV1 (cells 16-9) and FAULT_UV2 (cells 8-1) */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_UV1, &reg_val) &&
            bq79616_single_read(dev_addr, BQ_REG_FAULT_UV2, &reg_val2))
        {
            if (reg_val || reg_val2)
            {
                dbg("    Undervoltage detected:\r\n");
                decode_cell_faults(reg_val, reg_val2, "UV");
            }
        }
    }

    /*
     * BIT 3: FAULT_OTUT  –  GPIO overtemperature / undertemperature
     *
     * Each GPIO pin (1-8) can have an external thermistor.
     * OT means the temperature exceeded the OT comparator threshold.
     * UT means the temperature dropped below the UT threshold.
     */
    if (summary & 0x08)
    {
        dbg("\r\n  [FAULT_OTUT] Temperature fault:\r\n");

        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_OT, &reg_val))
        {
            if (reg_val)
            {
                dbg("    Overtemperature detected:\r\n");
                decode_gpio_faults(reg_val, "OT");
            }
        }

        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_UT, &reg_val))
        {
            if (reg_val)
            {
                dbg("    Undertemperature detected:\r\n");
                decode_gpio_faults(reg_val, "UT");
            }
        }
    }

    /*
     * BIT 4: FAULT_COMM  –  Communication faults
     *
     * Three sub-registers cover UART, daisy-chain (COMH/COML),
     * and heartbeat / fault-tone detection.
     */
    if (summary & 0x10)
    {
        dbg("\r\n  [FAULT_COMM] Communication fault:\r\n");

        /* FAULT_COMM1 (0x0530) – UART faults */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_COMM1, &reg_val))
        {
            static const char * const comm1_names[8] = {
                NULL,                          /* bit 7: reserved */
                NULL,                          /* bit 6: reserved */
                "UART_TR (TX response fault)",  /* bit 5 */
                "UART_RR (RX response fault)",  /* bit 4 */
                "UART_RC (RX command fault)",    /* bit 3 */
                NULL,                          /* bit 2: reserved */
                "COMMCLR_DET (clear detected)", /* bit 1 */
                "STOP_DET (stop detected)"      /* bit 0 */
            };
            dbg_hex("    COMM1 = ", reg_val);
            decode_bits(reg_val, comm1_names, 8);
        }

        /* FAULT_COMM2 (0x0531) – Daisy-chain faults */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_COMM2, &reg_val))
        {
            static const char * const comm2_names[8] = {
                "COML_TR (COML TX response)",  /* bit 7 */
                "COML_RR (COML RX response)",  /* bit 6 */
                "COML_RC (COML RX command)",   /* bit 5 */
                "COML_BIT (COML bit error)",   /* bit 4 */
                "COMH_TR (COMH TX response)",  /* bit 3 */
                "COMH_RR (COMH RX response)",  /* bit 2 */
                "COMH_RC (COMH RX command)",   /* bit 1 */
                "COMH_BIT (COMH bit error)"    /* bit 0 */
            };
            dbg_hex("    COMM2 = ", reg_val);
            decode_bits(reg_val, comm2_names, 8);
        }

        /* FAULT_COMM3 (0x0532) – Heartbeat / Fault tone */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_COMM3, &reg_val))
        {
            static const char * const comm3_names[8] = {
                NULL,                                  /* bit 7: reserved */
                NULL,                                  /* bit 6: reserved */
                NULL,                                  /* bit 5: reserved */
                NULL,                                  /* bit 4: reserved */
                "FCOMM_DET (stack fault forwarded)",   /* bit 3 */
                "FTONE_DET (fault tone received)",     /* bit 2 */
                "HB_FAIL (heartbeat missing)",         /* bit 1 */
                "HB_FAST (heartbeat too fast)"         /* bit 0 */
            };
            dbg_hex("    COMM3 = ", reg_val);
            decode_bits(reg_val, comm3_names, 8);
        }
    }

    /*
     * BIT 5: FAULT_OTP  –  OTP memory faults
     *
     * CRC mismatches or load errors in the one-time-programmable
     * memory.  If CUSTLDERR or FACTLDERR is set, the device
     * configuration may not be reliable.
     */
    if (summary & 0x20)
    {
        dbg("\r\n  [FAULT_OTP] OTP memory fault:\r\n");

        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_OTP, &reg_val))
        {
            static const char * const otp_names[8] = {
                NULL,                                       /* bit 7: reserved */
                "DED_DET (double error in OTP)",            /* bit 6 */
                "SEC_DET (single error corrected in OTP)",  /* bit 5 */
                "CUST_CRC (customer CRC error)",            /* bit 4 */
                "FACT_CRC (factory CRC error)",             /* bit 3 */
                "CUSTLDERR (customer OTP load error!)",     /* bit 2 */
                "FACTLDERR (factory OTP load error!)",      /* bit 1 */
                "GBLOVERR (OTP overvoltage error!)"         /* bit 0 */
            };
            dbg_hex("    OTP = ", reg_val);
            decode_bits(reg_val, otp_names, 8);
        }
    }

    /*
     * BIT 6: FAULT_COMP_ADC  –  ADC comparison diagnostic faults
     *
     * These fire when the main ADC and auxiliary ADC disagree
     * beyond a threshold.  There are many sub-registers (VCCB,
     * VCOW, CBOW, CBFET, GPIO, MISC).  We just flag the group
     * here — the individual registers can be read if needed.
     */
    if (summary & 0x40)
    {
        dbg("\r\n  [FAULT_COMP_ADC] ADC comparison fault detected.\r\n");
        dbg("    (Main vs AUX ADC disagreement — check\r\n");
        dbg("     FAULT_COMP_VCCB/VCOW/CBOW/CBFET/GPIO/MISC\r\n");
        dbg("     registers for details.)\r\n");
    }

    /*
     * BIT 7: FAULT_PROT  –  Protector (comparator) faults
     *
     * These indicate failures in the OV/UV/OT/UT hardware
     * comparator paths or the BIST self-test of those paths.
     */
    if (summary & 0x80)
    {
        dbg("\r\n  [FAULT_PROT] Protector fault:\r\n");

        /* FAULT_PROT1 (0x053A) – Parity check */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_PROT1, &reg_val))
        {
            static const char * const prot1_names[8] = {
                NULL, NULL, NULL, NULL, NULL,           /* bits 7-3: reserved */
                "TPARITY_FAIL (temp parity error)",    /* bit 2 */
                "VPARITY_FAIL (voltage parity error)", /* bit 1 */
                NULL                                   /* bit 0: reserved */
            };
            dbg_hex("    PROT1 = ", reg_val);
            decode_bits(reg_val, prot1_names, 8);
        }

        /* FAULT_PROT2 (0x053B) – BIST failures */
        if (bq79616_single_read(dev_addr, BQ_REG_FAULT_PROT2, &reg_val))
        {
            static const char * const prot2_names[8] = {
                NULL,                                       /* bit 7: reserved */
                "BIST_ABORT (protector BIST aborted)",     /* bit 6 */
                "TPATH_FAIL (temp signal path fail)",      /* bit 5 */
                "VPATH_FAIL (voltage signal path fail)",   /* bit 4 */
                "UTCOMP_FAIL (UT comparator BIST fail)",   /* bit 3 */
                "OTCOMP_FAIL (OT comparator BIST fail)",   /* bit 2 */
                "OVCOMP_FAIL (OV comparator BIST fail)",   /* bit 1 */
                "UVCOMP_FAIL (UV comparator BIST fail)"    /* bit 0 */
            };
            dbg_hex("    PROT2 = ", reg_val);
            decode_bits(reg_val, prot2_names, 8);
        }
    }

    /* ---- Summary footer ---- */
    dbg("\r\n=== FAULTS PRESENT ===\r\n\r\n");

    snprintf(fault_line_buf, sizeof(fault_line_buf),
             "FAULT: 0x%02X %s%s%s%s%s%s%s",
             summary,
             (summary & 0x01) ? "PWR "  : "",
             (summary & 0x02) ? "SYS "  : "",
             (summary & 0x04) ? "OV "   : "",
             (summary & 0x08) ? "UV "   : "",
             (summary & 0x10) ? "OT "   : "",
             (summary & 0x20) ? "COMP " : "",
             (summary & 0x40) ? "PROT " : "");

    return false;   /* At least one fault group was flagged */
}


/* ================================================================
 *  GPIO FUNCTIONS (Arduino-style)
 *
 *  The BQ79616 has 8 GPIO pins.  Each one has a 3-bit config field
 *  packed into one of four registers (two pins per register):
 *
 *    GPIO_CONF1 (0x0E): bits [2:0]=GPIO1, bits [5:3]=GPIO2
 *    GPIO_CONF2 (0x0F): bits [2:0]=GPIO3, bits [5:3]=GPIO4
 *    GPIO_CONF3 (0x10): bits [2:0]=GPIO5, bits [5:3]=GPIO6
 *    GPIO_CONF4 (0x11): bits [2:0]=GPIO7, bits [5:3]=GPIO8
 *
 *  The 3-bit codes are:
 *    000 = disabled (high-Z)
 *    001 = ADC + OTUT input (thermistor)
 *    010 = ADC only input
 *    011 = digital input
 *    100 = output HIGH
 *    101 = output LOW
 *    110 = ADC + weak pull-up
 *    111 = ADC + weak pull-down
 *
 *  IMPORTANT: These registers are NVM shadow registers (address
 *  range 0x0000–0x0035), so every write triggers the CRC update.
 *
 *  NOTE on outputs: There is no separate "output data" register.
 *  The mode code itself IS the output state:
 *    100 = output HIGH,  101 = output LOW.
 *  So digital_write() changes the config bits to toggle the pin.
 * ================================================================ */

/*
 * bq79616_pin_mode  –  Configure a GPIO pin as INPUT, OUTPUT, or ADC
 *
 * Works like Arduino's pinMode() but for the BQ79616's GPIOs.
 *
 * Usage (in application.c):
 *   bq79616_pin_mode(0x01, 3, BQ_OUTPUT);  // GPIO3 as output
 *   bq79616_pin_mode(0x01, 1, BQ_INPUT);   // GPIO1 as digital input
 *   bq79616_pin_mode(0x01, 5, BQ_ADC);     // GPIO5 as ADC input
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   pin      – GPIO number, 1 to 8
 *   mode     – BQ_INPUT (0x03), BQ_OUTPUT (0x04), or BQ_ADC (0x02)
 *
 * Returns:
 *   true  – pin configured and CRC updated
 *   false – invalid pin, bad mode, or comms failure
 */
bool bq79616_pin_mode(uint8_t dev_addr, uint8_t pin, uint8_t mode)
{
    uint8_t  shift;      /* Bit position within the register (0 or 3) */
    uint16_t reg_addr;   /* Which GPIO_CONFn register to modify       */
    uint8_t  reg_val;    /* Current register contents                 */
    char     buf[48];

    /* Validate pin number: must be 1 to 8 */
    if (pin < 1 || pin > 8)
    {
        dbg("  pin_mode: invalid pin (must be 1-8)\r\n");
        return false;
    }

    /* Validate mode: only INPUT, OUTPUT, or ADC allowed */
    if (mode != BQ_INPUT && mode != BQ_OUTPUT && mode != BQ_ADC)
    {
        dbg("  pin_mode: invalid mode\r\n");
        return false;
    }

    /* Work out which register and which 3-bit field to modify */
    reg_addr = bq79616_gpio_conf_reg(pin, &shift);

    /* Read the current register value (preserves the other pin's config) */
    if (!bq79616_single_read(dev_addr, reg_addr, &reg_val))
    {
        dbg("  pin_mode: read failed\r\n");
        return false;
    }

    /*
     * Clear the 3-bit field for our pin, then set the new mode.
     *
     * Example for GPIO2 (shift=3):
     *   mask = 0x07 << 3 = 0x38 = 0b00111000
     *   ~mask             = 0xC7 = 0b11000111
     *   reg_val & ~mask   → clears bits [5:3], keeps everything else
     *   | (mode << shift) → inserts the new 3-bit code
     */
    reg_val = (reg_val & ~(0x07 << shift)) | (mode << shift);

    /* Write back the modified register */
    if (!bq79616_single_write(dev_addr, reg_addr, reg_val))
    {
        dbg("  pin_mode: write failed\r\n");
        return false;
    }

    /*
     * Update the customer OTP CRC.
     * GPIO_CONF1-4 are NVM shadow registers, so the running CRC
     * changed when we wrote the register.  We must tell the device
     * the new expected CRC or CUST_CRC fault fires.
     */
    if (!bq79616_update_otp_crc(dev_addr))
    {
        dbg("  pin_mode: CRC update failed\r\n");
        return false;
    }

    snprintf(buf, sizeof(buf), "  GPIO%u -> mode 0x%02X\r\n", pin, mode);
    dbg(buf);

    return true;
}


/*
 * bq79616_digital_write  –  Set a GPIO output HIGH or LOW
 *
 * Works like Arduino's digitalWrite().
 * The pin MUST already be configured as BQ_OUTPUT via pin_mode().
 *
 * On the BQ79616, there's no separate output data register.
 * Instead the 3-bit mode code determines the state:
 *   100 = output HIGH
 *   101 = output LOW
 *
 * So this function reads the config register, changes the mode
 * code, writes it back, and updates the CRC.
 *
 * Usage (in application.c):
 *   bq79616_digital_write(0x01, 3, 1);  // GPIO3 = HIGH
 *   bq79616_digital_write(0x01, 3, 0);  // GPIO3 = LOW
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   pin      – GPIO number, 1 to 8
 *   state    – 1 for HIGH, 0 for LOW
 *
 * Returns:
 *   true  – output set and CRC updated
 *   false – invalid pin or comms failure
 */
bool bq79616_digital_write(uint8_t dev_addr, uint8_t pin, uint8_t state)
{
    uint8_t  shift;
    uint16_t reg_addr;
    uint8_t  reg_val;
    uint8_t  code;

    /* Validate pin number */
    if (pin < 1 || pin > 8)
    {
        dbg("  digital_write: invalid pin (must be 1-8)\r\n");
        return false;
    }

    /*
     * Pick the 3-bit code:
     *   state != 0 → 0x04 (100 = output HIGH)
     *   state == 0 → 0x05 (101 = output LOW)
     */
    code = (state) ? 0x04 : 0x05;

    /* Work out which register and field */
    reg_addr = bq79616_gpio_conf_reg(pin, &shift);

    /* Read current register (preserves the other pin) */
    if (!bq79616_single_read(dev_addr, reg_addr, &reg_val))
    {
        dbg("  digital_write: read failed\r\n");
        return false;
    }

    /* Clear old 3-bit field, insert new code */
    reg_val = (reg_val & ~(0x07 << shift)) | (code << shift);

    /* Write it back */
    if (!bq79616_single_write(dev_addr, reg_addr, reg_val))
    {
        dbg("  digital_write: write failed\r\n");
        return false;
    }

    /* Update CRC (NVM shadow register was changed) */
    if (!bq79616_update_otp_crc(dev_addr))
    {
        dbg("  digital_write: CRC update failed\r\n");
        return false;
    }

    return true;
}


/*
 * bq79616_digital_read  –  Read the digital state of a GPIO pin
 *
 * Works like Arduino's digitalRead().
 * The pin should be configured as BQ_INPUT via pin_mode() first,
 * but it also works for output pins (reads back the driven state).
 *
 * The digital state of all 8 GPIOs is in the GPIO_STAT register
 * (0x052A), one bit per pin:
 *   bit 0 = GPIO1, bit 1 = GPIO2, ... bit 7 = GPIO8
 *
 * Usage (in application.c):
 *   int val = bq79616_digital_read(0x01, 1);  // Read GPIO1
 *   if (val == 1) { ... }
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   pin      – GPIO number, 1 to 8
 *
 * Returns:
 *    1 – pin is HIGH
 *    0 – pin is LOW
 *   -1 – error (invalid pin or comms failure)
 */
int bq79616_digital_read(uint8_t dev_addr, uint8_t pin)
{
    uint8_t stat = 0;

    /* Validate pin number */
    if (pin < 1 || pin > 8)
    {
        dbg("  digital_read: invalid pin (must be 1-8)\r\n");
        return -1;
    }

    /* Read the GPIO_STAT register — all 8 pins in one byte */
    if (!bq79616_single_read(dev_addr, BQ_REG_GPIO_STAT, &stat))
    {
        dbg("  digital_read: read failed\r\n");
        return -1;
    }

    /*
     * Extract the bit for our pin.
     * GPIO1 is bit 0, GPIO2 is bit 1, ... GPIO8 is bit 7.
     * Shift right by (pin-1) and mask with 1.
     */
    return (stat >> (pin - 1)) & 0x01;
}


/* ================================================================
 *  ADC / ANALOGUE FUNCTIONS
 *
 *  The BQ79616 has a 16-bit Main ADC that measures cell voltages,
 *  GPIO voltages, TSREF, and die temperature in a round-robin
 *  cycle (~192 μs per cycle).  Each GPIO gets one slot per 8
 *  round-robin cycles, so all 8 GPIOs are measured every ~1.5 ms.
 *
 *  ADC results are 16-bit signed (2's complement) in two registers:
 *    GPIO*_HI (high byte) and GPIO*_LO (low byte).
 *  Conversion to microvolts: raw_decimal × 152.59 μV/LSB.
 *
 *  The reset/no-data value is 0x8000 (−32768), which means the
 *  ADC hasn't measured that channel yet.
 *
 *  WORKFLOW:
 *    1. bq79616_tsref_enable()  — turn on TSREF LDO (needed for
 *       thermistor bias, also good practice before any GPIO ADC)
 *    2. bq79616_pin_mode(pin, BQ_ADC) — configure pin for ADC
 *    3. bq79616_adc_start()     — start continuous Main ADC
 *    4. HAL_Delay(2)            — wait for first full round-robin
 *    5. bq79616_analog_read(pin) — read result in millivolts
 *       (repeat as often as needed)
 *    6. bq79616_adc_stop()      — stop Main ADC when done
 * ================================================================ */

/*
 * bq79616_tsref_enable  –  Turn on the TSREF LDO output
 *
 * TSREF is a ~5 V reference output used to bias NTC thermistors.
 * It's off by default after wake.  You must enable it before
 * taking any GPIO ADC measurements if thermistors are connected.
 *
 * The enable bit is CONTROL2[TSREF_EN] (bit 0) at address 0x030A.
 * This is a normal RW register (NOT an NVM shadow register), so
 * no CRC update is needed.
 *
 * After enabling, the datasheet requires a 1.35 ms wait for the
 * LDO to settle before taking measurements.  We wait 2 ms to
 * be safe.
 *
 * Usage (in application.c, after init):
 *   bq79616_tsref_enable(0x01);
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *
 * Returns:
 *   true  – TSREF enabled and settled
 *   false – comms failure
 */
bool bq79616_tsref_enable(uint8_t dev_addr)
{
    uint8_t reg_val = 0;

    dbg("TSREF: Enabling LDO...\r\n");

    /*
     * Read CONTROL2 first to preserve any other bits.
     * CONTROL2 (0x030A) layout:
     *   bit 7:2 = RSVD (reserved, keep as-is)
     *   bit 1   = SEND_HW_RESET (cleared on read, should be 0)
     *   bit 0   = TSREF_EN
     */
    if (!bq79616_single_read(dev_addr, BQ_REG_CONTROL2, &reg_val))
    {
        dbg("  TSREF: read CONTROL2 failed\r\n");
        return false;
    }

    dbg_hex("  CONTROL2 before = ", reg_val);

    /* Set bit 0 (TSREF_EN = 1) */
    reg_val |= 0x01;

    if (!bq79616_single_write(dev_addr, BQ_REG_CONTROL2, reg_val))
    {
        dbg("  TSREF: write CONTROL2 failed\r\n");
        return false;
    }

    /*
     * Wait for the TSREF LDO to settle.
     * Datasheet says 1.35 ms minimum; we use 2 ms for margin.
     */
    HAL_Delay(2);

    /* Read back to confirm */
    if (!bq79616_single_read(dev_addr, BQ_REG_CONTROL2, &reg_val))
    {
        dbg("  TSREF: read-back failed\r\n");
        return false;
    }

    dbg_hex("  CONTROL2 after  = ", reg_val);

    if (!(reg_val & 0x01))
    {
        dbg("  TSREF: FAIL — bit did not stick!\r\n");
        return false;
    }

    dbg("  TSREF: Enabled and settled.\r\n");
    return true;
}


/*
 * bq79616_adc_start  –  Start the Main ADC in continuous mode
 *
 * Writes to ADC_CTRL1 (0x030D) to begin continuous round-robin
 * measurements.  Once started, the ADC runs forever until you
 * call bq79616_adc_stop().
 *
 * ADC_CTRL1 register layout:
 *   bit 7:5 = RSVD
 *   bit 4   = LPF_BB_EN   (bus bar low-pass filter, leave 0)
 *   bit 3   = LPF_VCELL_EN (cell voltage LPF, leave 0)
 *   bit 2   = MAIN_GO     (1 = start, self-clears on read)
 *   bit 1:0 = MAIN_MODE   (00=stop, 01=8RR, 10=continuous)
 *
 * We write 0x06 = 0b00000110:
 *   MAIN_GO   = 1  (start)
 *   MAIN_MODE = 10 (continuous)
 *
 * This is a normal RW register — no CRC update needed.
 *
 * After starting, wait at least 2 ms before the first
 * analog_read() so all GPIO channels complete one measurement.
 *
 * Usage (in application.c):
 *   bq79616_adc_start(0x01);
 *   HAL_Delay(2);  // Wait for first round-robin to complete
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *
 * Returns:
 *   true  – ADC started
 *   false – comms failure
 */
bool bq79616_adc_start(uint8_t dev_addr)
{
    dbg("ADC: Starting continuous mode...\r\n");

    /*
     * 0x06 = MAIN_GO (bit 2) | MAIN_MODE = 10 (bits 1:0)
     * This kicks off the continuous round-robin.
     */
    if (!bq79616_single_write(dev_addr, BQ_REG_ADC_CTRL1, 0x06))
    {
        dbg("  ADC: write ADC_CTRL1 failed\r\n");
        return false;
    }

    dbg("  ADC: Running.\r\n");
    return true;
}


/*
 * bq79616_adc_stop  –  Stop the Main ADC
 *
 * Writes ADC_CTRL1 with MAIN_MODE = 00 (stop) and MAIN_GO = 1
 * to apply the new setting.  The ADC halts after the current
 * conversion finishes.
 *
 * Usage (in application.c):
 *   bq79616_adc_stop(0x01);
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *
 * Returns:
 *   true  – ADC stopped
 *   false – comms failure
 */
bool bq79616_adc_stop(uint8_t dev_addr)
{
    dbg("ADC: Stopping...\r\n");

    /*
     * 0x04 = MAIN_GO (bit 2) | MAIN_MODE = 00 (stop)
     * The GO bit tells the device to sample the new mode setting.
     */
    if (!bq79616_single_write(dev_addr, BQ_REG_ADC_CTRL1, 0x04))
    {
        dbg("  ADC: write ADC_CTRL1 failed\r\n");
        return false;
    }

    dbg("  ADC: Stopped.\r\n");
    return true;
}


/*
 * bq79616_analog_read  –  Read a GPIO pin's ADC value in millivolts
 *
 * Works like Arduino's analogRead() but returns millivolts instead
 * of a raw count.  For example, 3300 means 3.3 V.
 *
 * PREREQUISITES:
 *   - Pin must be configured as BQ_ADC via bq79616_pin_mode()
 *   - Main ADC must be running via bq79616_adc_start()
 *   - Wait at least 2 ms after start for first valid readings
 *
 * HOW IT WORKS:
 *   Each GPIO has two result registers (HI and LO byte) that
 *   hold the latest 16-bit signed ADC sample.  The addresses
 *   follow a pattern:
 *     GPIO1_HI = 0x058E, GPIO1_LO = 0x058F
 *     GPIO2_HI = 0x0590, GPIO2_LO = 0x0591
 *     ...
 *     GPIOn_HI = 0x058E + (n-1)*2
 *
 *   Reading HI first "locks" LO from updating, so we get a
 *   consistent 16-bit snapshot (the device does this for us).
 *
 *   The raw value is 2's complement (int16_t).  We convert to
 *   millivolts using the datasheet LSB size of 152.59 μV:
 *     millivolts = raw * 15259 / 100000   (integer maths)
 *
 *   The reset value 0x8000 (−32768) means "no data" — either
 *   the ADC hasn't run yet or the pin isn't configured for ADC.
 *   We treat this as an error.
 *
 * Usage (in application.c):
 *   int32_t mv = bq79616_analog_read(0x01, 5);  // Read GPIO5
 *   if (mv >= 0)
 *   {
 *       // mv is the voltage in millivolts, e.g. 3300 = 3.3 V
 *   }
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   pin      – GPIO number, 1 to 8
 *
 * Returns:
 *   >= 0 : voltage in millivolts (e.g. 3300 = 3.3 V)
 *   -1   : error (invalid pin, comms failure, or no data)
 */
int32_t bq79616_analog_read(uint8_t dev_addr, uint8_t pin)
{
    uint8_t  hi_byte = 0;
    uint8_t  lo_byte = 0;
    uint16_t hi_addr;
    uint16_t lo_addr;
    int16_t  raw;
    int32_t  mv;

    /* Validate pin number */
    if (pin < 1 || pin > 8)
    {
        dbg("  analog_read: invalid pin (must be 1-8)\r\n");
        return -1;
    }

    /*
     * Calculate the register addresses for this pin.
     * GPIO1_HI = 0x058E, GPIO1_LO = 0x058F
     * Each subsequent GPIO is 2 addresses higher.
     */
    hi_addr = BQ_REG_GPIO1_HI + ((uint16_t)(pin - 1) * 2);
    lo_addr = hi_addr + 1;

    /*
     * Read HI byte FIRST.
     * This is critical — reading HI locks the LO register from
     * updating until we also read LO.  Guarantees we get both
     * bytes from the same ADC sample.
     */
    if (!bq79616_single_read(dev_addr, hi_addr, &hi_byte))
    {
        dbg("  analog_read: HI read failed\r\n");
        return -1;
    }

    if (!bq79616_single_read(dev_addr, lo_addr, &lo_byte))
    {
        dbg("  analog_read: LO read failed\r\n");
        return -1;
    }

    /*
     * Combine into a 16-bit signed value.
     * The BQ79616 stores results in 2's complement.
     * Example: 0x6666 = +26214 → about 4000 mV
     *          0x8000 = −32768 → "no data" sentinel
     */
    raw = (int16_t)((uint16_t)hi_byte << 8 | lo_byte);

    /*
     * Check for the "no data" sentinel value.
     * The ADC resets all result registers to 0x8000 when started.
     * If we still read 0x8000, either:
     *   - The ADC hasn't been started (call bq79616_adc_start)
     *   - The pin isn't configured for ADC (call bq79616_pin_mode)
     *   - Not enough time has passed since start
     */
    if (raw == (int16_t)0x8000)
    {
        dbg("  analog_read: no data (0x8000) — ADC not ready?\r\n");
        return -1;
    }

    /*
     * Convert raw ADC count to millivolts.
     *
     * Datasheet spec: VLSB_GPIO = 152.59 μV per LSB
     *
     * So:  microvolts = raw × 152.59
     *      millivolts = raw × 152.59 / 1000
     *                 = raw × 15259 / 100000
     *
     * Using integer maths with +50000 for rounding:
     *   mv = (raw * 15259 + 50000) / 100000
     *
     * Overflow check: max raw = 32767
     *   32767 × 15259 = 500,049,653  → fits in int32_t
     */
    mv = ((int32_t)raw * 15259 + 50000) / 100000;

    return mv;
}


/* ================================================================
 *  CELL VOLTAGE FUNCTIONS
 *
 *  The BQ79616 supports 6 to 16 series cells.  The number of
 *  active cells is configured in ACTIVE_CELL[NUM_CELL3:0]:
 *    0x0 = 6S, 0x1 = 7S, ... 0xA = 16S.
 *
 *  Cell voltage results come from the Main ADC.  The registers
 *  are in REVERSE order — Cell 16 has the lowest address:
 *    VCELL16_HI = 0x0568
 *    VCELL15_HI = 0x056A
 *    ...
 *    VCELL1_HI  = 0x0586
 *    Formula: VCELLn_HI = 0x0568 + (16 - n) * 2
 *
 *  The LSB size for cell voltages is 190.73 μV (VLSB_ADC).
 *  This is different from the GPIO LSB of 152.59 μV.
 * ================================================================ */


/* ================================================================
 * bq79616_set_adc_offset  –  Apply an offset correction to the
 *                             Main ADC for all cell channels.
 *
 * The BQ79616's MAIN_ADC_CAL2 register has a 7-bit signed
 * OFFSET field (bits 6:0) with 0.19073 mV per LSB.  The chip
 * applies this correction automatically to every Main ADC
 * conversion result.
 *
 * HOW TO CALIBRATE
 * ----------------
 *   1. Let the pack rest (no balancing, no load) for 5+ minutes.
 *   2. Read all 16 cells with the BQ79616 (terminal printout).
 *   3. Read all 16 cells with a calibrated multimeter.
 *   4. Compute the AVERAGE difference:
 *        offset = average(meter - BQ)  for all cells
 *   5. Pass that value as offset_mv.
 *
 * Example:  BQ reads 4037, meter reads 4032 on average.
 *           offset = 4032 - 4037 = -5 mV.
 *           Call: bq79616_set_adc_offset(0x01, -5);
 *
 * This is an NVM shadow register — writing it requires a
 * CRC update to avoid CUST_CRC faults.  The gain field
 * (GAINH in bit 7 of CAL2, GAINL in CAL1) is preserved.
 *
 * Parameters:
 *   dev_addr  – device address (0x01)
 *   offset_mv – signed correction in millivolts (-12 to +12).
 *               Negative means BQ was reading too high.
 *
 * Returns:
 *   true  – offset written and CRC updated
 *   false – value out of range or comms failure
 * ================================================================ */
bool bq79616_set_adc_offset(uint8_t dev_addr, int8_t offset_mv)
{
    char    buf[80];
    uint8_t cal2_old;         /* Current MAIN_ADC_CAL2 value        */
    uint8_t cal2_new;         /* Value we'll write back              */
    int8_t  offset_lsb;       /* Offset in ADC LSBs (0.19073 mV)    */
    uint8_t offset_field;     /* 7-bit 2's complement for register   */

    dbg("ADC offset: Calibrating...\r\n");

    /* ---- Validate range ----
     *
     * The 7-bit signed field can hold -64 to +63 LSBs.
     * At 0.19073 mV/LSB, that's roughly -12.2 to +12.0 mV.
     * We clamp to ±12 mV for safety.
     */
    if (offset_mv < -12 || offset_mv > 12)
    {
        dbg("  ERROR: offset must be -12 to +12 mV\r\n");
        return false;
    }

    /* ---- Convert millivolts to LSBs ----
     *
     * 1 LSB = 0.19073 mV, so:
     *   LSBs = offset_mv / 0.19073
     *        = offset_mv * 1000000 / 190730
     *
     * We use integer arithmetic with rounding:
     *   LSBs = (offset_mv * 10000 + sign*953) / 1907
     *
     * For -5 mV: (-50000 - 953) / 1907 = -26.7 → -26
     * The rounding biases toward zero which is conservative.
     */
    {
        int32_t numer = (int32_t)offset_mv * 10000;

        /* Add half-divisor toward zero for rounding */
        if (numer >= 0)
        {
            numer += 953;   /* 1907 / 2 ≈ 953 */
        }
        else
        {
            numer -= 953;
        }

        offset_lsb = (int8_t)(numer / 1907);
    }

    /* Clamp to 7-bit signed range just in case */
    if (offset_lsb > 63)
    {
        offset_lsb = 63;
    }
    if (offset_lsb < -64)
    {
        offset_lsb = -64;
    }

    /* ---- Convert to 7-bit register field ----
     *
     * OFFSET[6:0] is 7-bit 2's complement.
     * Mask to 7 bits to strip any sign extension.
     */
    offset_field = (uint8_t)offset_lsb & 0x7F;

    snprintf(buf, sizeof(buf),
             "  Offset: %d mV -> %d LSBs (%.2f mV actual)\r\n",
             (int)offset_mv, (int)offset_lsb,
             (double)offset_lsb * 0.19073);
    dbg(buf);

    /* ---- Read current MAIN_ADC_CAL2 ----
     *
     * We need to preserve bit 7 (GAINH) which is the
     * MSB of the separate 9-bit gain correction field.
     * Only bits 6:0 (OFFSET) get overwritten.
     */
    if (!bq79616_single_read(dev_addr,
                             BQ_REG_MAIN_ADC_CAL2, &cal2_old))
    {
        dbg("  ERROR: could not read MAIN_ADC_CAL2\r\n");
        return false;
    }

    snprintf(buf, sizeof(buf),
             "  MAIN_ADC_CAL2 before = 0x%02X\r\n",
             cal2_old);
    dbg(buf);

    /* ---- Build new register value ----
     *
     * Keep GAINH (bit 7) from the old value.
     * Replace OFFSET[6:0] with our new value.
     */
    cal2_new = (cal2_old & 0x80) | offset_field;

    /* ---- Write MAIN_ADC_CAL2 ----
     *
     * This is an NVM shadow register.  The chip applies
     * the offset immediately to ADC conversions.
     */
    if (!bq79616_single_write(dev_addr,
                              BQ_REG_MAIN_ADC_CAL2, cal2_new))
    {
        dbg("  ERROR: could not write MAIN_ADC_CAL2\r\n");
        return false;
    }

    snprintf(buf, sizeof(buf),
             "  MAIN_ADC_CAL2 after  = 0x%02X\r\n",
             cal2_new);
    dbg(buf);

    /* ---- Update customer CRC ----
     *
     * Since MAIN_ADC_CAL2 is in the NVM shadow region,
     * the chip recalculates its CRC and compares against
     * CUST_CRC_HI/LO.  If they don't match, it asserts
     * a CUST_CRC fault.  So we read the new computed CRC
     * and write it back, same as for OV/UV thresholds.
     */
    {
        uint8_t crc_hi, crc_lo;

        if (!bq79616_single_read(dev_addr,
                                 BQ_REG_CUST_CRC_RSLT_HI,
                                 &crc_hi))
        {
            dbg("  ERROR: CRC read failed\r\n");
            return false;
        }
        if (!bq79616_single_read(dev_addr,
                                 BQ_REG_CUST_CRC_RSLT_LO,
                                 &crc_lo))
        {
            dbg("  ERROR: CRC read failed\r\n");
            return false;
        }

        bq79616_single_write(dev_addr,
                             BQ_REG_CUST_CRC_HI, crc_hi);
        bq79616_single_write(dev_addr,
                             BQ_REG_CUST_CRC_LO, crc_lo);
    }

    dbg("  ADC offset: Applied and CRC updated.\r\n");
    return true;
}


/*
 * bq79616_set_active_cells  –  Set how many cells are connected
 *
 * Writes the ACTIVE_CELL register (0x0003) which is an NVM shadow
 * register — CRC update is required afterwards.
 *
 * The register field NUM_CELL[3:0] uses an offset encoding:
 *   0x0 = 6 cells, 0x1 = 7 cells, ... 0xA = 16 cells.
 *   So the value written = cells - 6.
 *
 * This tells the BQ79616 which VC channels are active.  Inactive
 * channels (above the count) are ignored by the ADC — their
 * result registers stay at the 0x8000 reset value.
 *
 * IMPORTANT: If the Main ADC is already running, you must
 * restart it (stop + start) after changing active cells for
 * the new configuration to take effect.
 *
 * Usage (in application.c, after init):
 *   bq79616_set_active_cells(0x01, 10);  // 10S configuration
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   cells    – number of cells, 6 to 16
 *
 * Returns:
 *   true  – register written and CRC updated
 *   false – invalid count or comms failure
 */
bool bq79616_set_active_cells(uint8_t dev_addr, uint8_t cells)
{
    uint8_t reg_val;
    char    buf[48];

    /* Validate: BQ79616 supports 6 to 16 cells */
    if (cells < 6 || cells > 16)
    {
        dbg("  set_active_cells: invalid (must be 6-16)\r\n");
        return false;
    }

    /*
     * NUM_CELL encoding: value = cells - 6.
     * SPARE bits [7:4] must stay 0.
     */
    reg_val = cells - 6;

    snprintf(buf, sizeof(buf),
             "Cells: Setting %uS (reg=0x%02X)...\r\n", cells, reg_val);
    dbg(buf);

    /* Write ACTIVE_CELL register */
    if (!bq79616_single_write(dev_addr, BQ_REG_ACTIVE_CELL, reg_val))
    {
        dbg("  set_active_cells: write failed\r\n");
        return false;
    }

    /*
     * ACTIVE_CELL (0x0003) is an NVM shadow register in the
     * CRC-covered range (0x0000 to 0x0035).  Update the CRC
     * so CUST_CRC fault doesn't fire.
     */
    if (!bq79616_update_otp_crc(dev_addr))
    {
        dbg("  set_active_cells: CRC update failed\r\n");
        return false;
    }

    /* Read back to confirm */
    if (!bq79616_single_read(dev_addr, BQ_REG_ACTIVE_CELL, &reg_val))
    {
        dbg("  set_active_cells: read-back failed\r\n");
        return false;
    }

    snprintf(buf, sizeof(buf),
             "  ACTIVE_CELL = 0x%02X (%uS confirmed)\r\n",
             reg_val, (reg_val & 0x0F) + 6);
    dbg(buf);

    return true;
}


/*
 * bq79616_read_cell_voltages  –  Read all active cell voltages
 *
 * Reads the ACTIVE_CELL register to find out how many cells are
 * configured, then reads each cell's ADC result and converts
 * to millivolts.
 *
 * The results are stored in mv_out[] with index 0 = Cell 1,
 * index 1 = Cell 2, and so on.  Only active cells are read;
 * the rest of the array is untouched.
 *
 * PREREQUISITES:
 *   - Call bq79616_set_active_cells() to configure cell count
 *   - Call bq79616_adc_start() and wait at least 2 ms
 *
 * REGISTER LAYOUT:
 *   The VCELL registers are stored in reverse order:
 *     VCELL16_HI = 0x0568, VCELL16_LO = 0x0569
 *     VCELL15_HI = 0x056A, VCELL15_LO = 0x056B
 *     ...
 *     VCELL1_HI  = 0x0586, VCELL1_LO  = 0x0587
 *
 *   Formula: VCELLn_HI = 0x0568 + (16 - n) * 2
 *
 * CONVERSION:
 *   Cell voltage LSB = 190.73 μV (VLSB_ADC)
 *   millivolts = raw * 19073 / 100000
 *
 *   Overflow check: max raw 32767 × 19073 = 625,094,591
 *   → fits comfortably in int32_t.
 *
 * Usage (in application.c):
 *   int32_t cells_mv[BQ_MAX_CELLS];
 *   int count = bq79616_read_cell_voltages(0x01, cells_mv, BQ_MAX_CELLS);
 *   if (count > 0)
 *   {
 *       for (int i = 0; i < count; i++)
 *       {
 *           // cells_mv[i] is Cell(i+1) voltage in mV
 *       }
 *   }
 *
 * Parameters:
 *   dev_addr  – device address (0x01)
 *   mv_out    – array to receive cell voltages in millivolts
 *               (must be large enough for all active cells)
 *   max_cells – size of the mv_out array (safety limit)
 *
 * Returns:
 *   > 0 : number of cells successfully read
 *   -1  : error (comms failure or bad configuration)
 */
int bq79616_read_cell_voltages(uint8_t dev_addr,
                                int32_t mv_out[],
                                uint8_t max_cells)
{
    uint8_t  active_reg = 0;
    uint8_t  num_cells;
    uint8_t  hi_byte, lo_byte;
    uint16_t hi_addr, lo_addr;
    int16_t  raw;
    int32_t  mv;
    uint8_t  i;
    char     buf[48];

    /* ---- Read ACTIVE_CELL to find out how many cells ---- */
    if (!bq79616_single_read(dev_addr, BQ_REG_ACTIVE_CELL, &active_reg))
    {
        dbg("  read_cells: could not read ACTIVE_CELL\r\n");
        return -1;
    }

    /*
     * NUM_CELL[3:0] = register value.
     * Actual cells = value + 6.
     * Clamp to valid range just in case.
     */
    num_cells = (active_reg & 0x0F) + 6;
    if (num_cells > 16)
    {
        num_cells = 16;
    }

    /* Don't overflow the caller's array */
    if (num_cells > max_cells)
    {
        num_cells = max_cells;
    }

    snprintf(buf, sizeof(buf), "Reading %u cell voltages...\r\n", num_cells);
    dbg(buf);

    /* ---- Read each active cell ---- */
    for (i = 0; i < num_cells; i++)
    {
        /*
         * Cell number is (i + 1).  Cell 1 is at the HIGHEST
         * address and Cell 16 at the LOWEST.
         *
         * VCELLn_HI = 0x0568 + (16 - n) * 2
         * For Cell 1: 0x0568 + 15*2 = 0x0586  ✓
         * For Cell 16: 0x0568 + 0*2 = 0x0568  ✓
         */
        hi_addr = BQ_REG_VCELL16_HI + (uint16_t)(16 - (i + 1)) * 2;
        lo_addr = hi_addr + 1;

        /* Read HI first (locks LO from updating) */
        if (!bq79616_single_read(dev_addr, hi_addr, &hi_byte))
        {
            snprintf(buf, sizeof(buf),
                     "  Cell%u: HI read failed\r\n", i + 1);
            dbg(buf);
            return -1;
        }

        if (!bq79616_single_read(dev_addr, lo_addr, &lo_byte))
        {
            snprintf(buf, sizeof(buf),
                     "  Cell%u: LO read failed\r\n", i + 1);
            dbg(buf);
            return -1;
        }

        /* Combine into 16-bit signed value */
        raw = (int16_t)((uint16_t)hi_byte << 8 | lo_byte);

        /* Check for "no data" sentinel */
        if (raw == (int16_t)0x8000)
        {
            snprintf(buf, sizeof(buf),
                     "  Cell%u: no data (0x8000)\r\n", i + 1);
            dbg(buf);
            mv_out[i] = -1;   /* Mark as invalid */
            continue;          /* Keep going for other cells */
        }

        /*
         * Convert to millivolts.
         * VLSB_ADC = 190.73 μV per LSB
         * millivolts = raw × 190.73 / 1000
         *            = raw × 19073 / 100000
         *
         * With rounding: (raw * 19073 + 50000) / 100000
         */
        mv = ((int32_t)raw * 19073 + 50000) / 100000;
        mv_out[i] = mv;

        snprintf(buf, sizeof(buf),
                 "  Cell%u = %ld mV\r\n", i + 1, (long)mv);
        dbg(buf);
    }

    return (int)num_cells;
}


/* ================================================================
 *  PROTECTOR THRESHOLD FUNCTIONS
 *
 *  The BQ79616 has hardware comparators that continuously monitor
 *  cell voltages against OV (overvoltage) and UV (undervoltage)
 *  thresholds.  These comparators are independent of the ADC —
 *  they work even if the ADC fails.
 *
 *  Both registers are NVM shadow registers, so CRC update is
 *  required after writing.
 *
 *  IMPORTANT: After changing a threshold, you must send
 *  OVUV_GO = 1 for the comparators to pick up the new value.
 *  The functions below set the register but do NOT auto-start
 *  the OVUV protector — that's a separate step you control.
 * ================================================================ */

/*
 * bq79616_set_ov_thresh  –  Set the cell overvoltage threshold
 *
 * Writes OV_THRESH (0x0009), an NVM shadow register.
 *
 * The OV comparator has three non-contiguous voltage ranges,
 * all in 25 mV steps:
 *
 *   Range 1:  2700 – 3000 mV  (codes 0x02 – 0x0E)
 *   Range 2:  3500 – 3800 mV  (codes 0x12 – 0x1E)
 *   Range 3:  4175 – 4475 mV  (codes 0x22 – 0x2E)
 *
 * Gaps between ranges (3001–3499 and 3801–4174) are invalid.
 * Values outside all ranges will be rejected.
 *
 * The input millivolt value is rounded DOWN to the nearest
 * 25 mV step within the matching range.
 *
 * NOTE: After setting, you must issue OVUV_GO = 1 for the
 * comparator to use the new threshold.
 *
 * Usage (in application.c):
 *   bq79616_set_ov_thresh(0x01, 4200);  // OV at 4.200 V
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   mv       – threshold in millivolts (e.g. 4200)
 *
 * Returns:
 *   true  – threshold set and CRC updated
 *   false – value out of range or comms failure
 */
bool bq79616_set_ov_thresh(uint8_t dev_addr, uint16_t mv)
{
    uint8_t code;
    char    buf[64];

    /*
     * Work out which of the three ranges this value falls in,
     * then calculate the 6-bit register code.
     *
     * Range 1: 2700–3000 mV → code = (mv-2700)/25 + 0x02
     * Range 2: 3500–3800 mV → code = (mv-3500)/25 + 0x12
     * Range 3: 4175–4475 mV → code = (mv-4175)/25 + 0x22
     */
    if (mv >= 2700 && mv <= 3000)
    {
        code = (uint8_t)((mv - 2700) / 25) + 0x02;
    }
    else if (mv >= 3500 && mv <= 3800)
    {
        code = (uint8_t)((mv - 3500) / 25) + 0x12;
    }
    else if (mv >= 4175 && mv <= 4475)
    {
        code = (uint8_t)((mv - 4175) / 25) + 0x22;
    }
    else
    {
        /* Value falls in a gap or is out of range entirely */
        snprintf(buf, sizeof(buf),
                 "  set_ov: %u mV out of range!\r\n", mv);
        dbg(buf);
        dbg("  Valid: 2700-3000, 3500-3800, 4175-4475 mV\r\n");
        return false;
    }

    /* Tell the user what we're actually setting (after rounding) */
    snprintf(buf, sizeof(buf),
             "OV: Setting %u mV (code=0x%02X)...\r\n", mv, code);
    dbg(buf);

    /* Write the register (spare bits [7:6] = 0) */
    if (!bq79616_single_write(dev_addr, BQ_REG_OV_THRESH, code))
    {
        dbg("  set_ov: write failed\r\n");
        return false;
    }

    /* CRC update — OV_THRESH (0x0009) is an NVM shadow register */
    if (!bq79616_update_otp_crc(dev_addr))
    {
        dbg("  set_ov: CRC update failed\r\n");
        return false;
    }

    /* Read back to confirm */
    {
        uint8_t readback = 0;

        if (!bq79616_single_read(dev_addr, BQ_REG_OV_THRESH, &readback))
        {
            dbg("  set_ov: read-back failed\r\n");
            return false;
        }

        snprintf(buf, sizeof(buf),
                 "  OV_THRESH = 0x%02X (confirmed)\r\n", readback);
        dbg(buf);
    }

    dbg("  NOTE: Send OVUV_GO=1 for new threshold to take effect.\r\n");
    return true;
}


/*
 * bq79616_set_uv_thresh  –  Set the cell undervoltage threshold
 *
 * Writes UV_THRESH (0x000A), an NVM shadow register.
 *
 * The UV comparator has one contiguous range in 50 mV steps:
 *
 *   1200 – 3100 mV  (codes 0x00 – 0x26)
 *
 * Values outside this range will be rejected.
 *
 * The input millivolt value is rounded DOWN to the nearest
 * 50 mV step.
 *
 * NOTE: After setting, you must issue OVUV_GO = 1 for the
 * comparator to use the new threshold.
 *
 * Usage (in application.c):
 *   bq79616_set_uv_thresh(0x01, 2800);  // UV at 2.800 V
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   mv       – threshold in millivolts (e.g. 2800)
 *
 * Returns:
 *   true  – threshold set and CRC updated
 *   false – value out of range or comms failure
 */
bool bq79616_set_uv_thresh(uint8_t dev_addr, uint16_t mv)
{
    uint8_t code;
    char    buf[64];

    /* Validate range: 1200 to 3100 mV */
    if (mv < 1200 || mv > 3100)
    {
        snprintf(buf, sizeof(buf),
                 "  set_uv: %u mV out of range!\r\n", mv);
        dbg(buf);
        dbg("  Valid: 1200-3100 mV (50 mV steps)\r\n");
        return false;
    }

    /*
     * UV_THR encoding is simple and contiguous:
     *   code 0x00 = 1200 mV
     *   code 0x01 = 1250 mV
     *   ...
     *   code 0x26 = 3100 mV
     *
     * Formula: code = (mv - 1200) / 50
     * Integer division rounds down to nearest 50 mV step.
     */
    code = (uint8_t)((mv - 1200) / 50);

    /* Clamp to max code just in case of rounding edge */
    if (code > 0x26)
    {
        code = 0x26;
    }

    snprintf(buf, sizeof(buf),
             "UV: Setting %u mV (code=0x%02X)...\r\n", mv, code);
    dbg(buf);

    /* Write the register (spare bits [7:6] = 0) */
    if (!bq79616_single_write(dev_addr, BQ_REG_UV_THRESH, code))
    {
        dbg("  set_uv: write failed\r\n");
        return false;
    }

    /* CRC update — UV_THRESH (0x000A) is an NVM shadow register */
    if (!bq79616_update_otp_crc(dev_addr))
    {
        dbg("  set_uv: CRC update failed\r\n");
        return false;
    }

    /* Read back to confirm */
    {
        uint8_t readback = 0;

        if (!bq79616_single_read(dev_addr, BQ_REG_UV_THRESH, &readback))
        {
            dbg("  set_uv: read-back failed\r\n");
            return false;
        }

        snprintf(buf, sizeof(buf),
                 "  UV_THRESH = 0x%02X (confirmed)\r\n", readback);
        dbg(buf);
    }

    dbg("  NOTE: Send OVUV_GO=1 for new threshold to take effect.\r\n");
    return true;
}


/* ================================================================
 *  CELL BALANCING
 *
 *  The BQ79616 has internal MOSFET switches (CBFETs) across each
 *  cell that can discharge higher-voltage cells through external
 *  balancing resistors.  This "passive balancing" brings all
 *  cells closer to the same voltage.
 *
 *  In AUTO mode the chip automatically alternates between odd
 *  and even numbered cells (they share current paths so can't
 *  all be on at once).  The swap time between odd/even groups
 *  is the "duty" setting.
 *
 *  This function does a SMART start:
 *    1. Reads all current cell voltages via the Main ADC
 *    2. Finds the lowest-voltage cell
 *    3. Only enables balancing on cells that are MORE than
 *       delta_mv above the minimum (the others don't need it)
 *    4. Sets VCB_DONE_THRESH as an absolute voltage floor —
 *       no cell will be balanced below this voltage
 *    5. Starts the OVUV protector in round-robin (needed for
 *       VCB_DONE to work, and also activates OV/UV protection
 *       if you've already set those thresholds)
 *    6. Fires BAL_GO with AUTO_BAL and FLTSTOP_EN enabled
 *
 *  Balancing timers are set to 120 minutes for all enabled
 *  cells.  The expectation is that your main loop calls this
 *  function periodically (every few minutes) to re-evaluate
 *  which cells still need balancing.  VCB_DONE acts as an
 *  additional safety stop per-cell.
 *
 *  REGISTER SUMMARY (all RW, none are NVM — no CRC needed):
 *
 *  CB_CELLn_CTRL (0x0318 + (16-n)) — per-cell balance timer
 *    TIME[4:0]:  0x00=stop, 0x01=10s, 0x02=30s, 0x03=60s,
 *                0x04=300s, 0x05–0x10 = 10–120 min (10 min steps),
 *                0x11–0x1F = 150–600 min (30 min steps)
 *
 *  VCB_DONE_THRESH (0x032A) — stop-balancing voltage floor
 *    CB_THR[5:0]: 0x00=disabled, 0x01=2450 mV … 0x3F=4000 mV
 *                 (25 mV steps).  Requires OVUV_GO=1 to latch.
 *
 *  BAL_CTRL1 (0x032E) — DUTY[2:0] odd/even swap time
 *  BAL_CTRL2 (0x032F) — AUTO_BAL, BAL_GO, FLTSTOP_EN
 *  OVUV_CTRL (0x032C) — OVUV_GO, OVUV_MODE (round robin)
 *  BAL_STAT  (0x0552) — CB_RUN, CB_DONE status (read only)
 *
 * ================================================================ */

/*
 * bq79616_auto_balance  –  Smart-start automatic cell balancing
 *
 * Reads cell voltages, identifies which cells are higher than
 * the lowest cell by more than delta_mv, enables balancing on
 * those cells only, and starts the BQ79616's auto-balance mode
 * which swaps odd/even cells at the duty interval.
 *
 * If no cells need balancing (all within delta_mv of each other)
 * the function skips and returns true without starting.
 *
 * PREREQUISITES:
 *   - bq79616_set_active_cells() has been called
 *   - Main ADC is running (bq79616_adc_start)
 *   - Wait at least 2 ms after ADC start for valid data
 *
 * Usage (in application.c):
 *
 *   // Balance cells, stop if any cell drops below 2900 mV,
 *   // stop balancing a cell once it's within 10 mV of the
 *   // lowest cell, swap odd/even every 30 seconds.
 *
 *   bq79616_auto_balance(0x01, 2900, 10, BQ_DUTY_30S);
 *
 * Parameters:
 *   dev_addr – device address (0x01)
 *   floor_mv – absolute minimum cell voltage for balancing
 *              (VCB_DONE threshold, range 2450–4000 mV)
 *              If a cell voltage drops below this, its
 *              balancing stops automatically.
 *   delta_mv – voltage difference threshold in millivolts.
 *              Only cells that are more than delta_mv above
 *              the lowest cell will be balanced.  When all
 *              cells are within delta_mv, balancing is done.
 *              Typical value: 5–20 mV.
 *   duty     – odd/even swap time code (use BQ_DUTY_* defines)
 *              BQ_DUTY_5S (0) through BQ_DUTY_30M (7)
 *
 * Returns:
 *   true  – balancing started (or skipped because cells
 *           are already balanced within delta_mv)
 *   false – communication error
 */
bool bq79616_auto_balance(uint8_t  dev_addr,
                          uint16_t floor_mv,
                          uint16_t delta_mv,
                          uint8_t  duty)
{
    int32_t  cells_mv[BQ_MAX_CELLS];  /* Millivolt readings           */
    int      num_cells;                /* Number of active cells       */
    int32_t  min_mv;                   /* Lowest cell voltage found    */
    int32_t  target_mv;               /* min_mv + delta_mv            */
    uint8_t  bal_count = 0;            /* How many cells need balance  */
    uint16_t cb_addr;                  /* CB_CELL register address     */
    uint8_t  vcb_code;                 /* VCB_DONE register code       */
    uint8_t  stat;                     /* BAL_STAT readback            */
    char     buf[64];
    int      i;

    dbg("=== AUTO BALANCE START ===\r\n");

    /* ---- Step 0: Read current balance status (info only) ----
     *
     * BAL_STAT (0x052B):
     *   bit 7 = INVALID_CBCONF
     *   bit 3 = CB_RUN   (balancing active)
     *   bit 2 = ABORTFLT (fault killed session)
     *   bit 0 = CB_DONE  (all cells finished)
     *
     * We ALWAYS re-evaluate and re-send BAL_GO, even if
     * balancing is already running.  This is necessary because
     * as the pack charges, new cells cross above floor_mv and
     * need to be added to the balancing set.  The only way to
     * add them is to update the timers and re-send BAL_GO.
     *
     * The brief BAL_GO restart (fraction of a second gap) is
     * imperceptible.  The earlier "flash" problem was caused
     * by VCB_DONE instantly killing cells below floor — that
     * is now fixed by the floor_mv check in Step 3.
     */
    if (bq79616_single_read(dev_addr, BQ_REG_BAL_STAT, &stat))
    {
        snprintf(buf, sizeof(buf),
                 "  BAL_STAT = 0x%02X", stat);
        dbg(buf);

        /* Log what we see for diagnostics */
        if (stat & 0x08) { dbg(" [CB_RUN]"); }
        if (stat & 0x04) { dbg(" [ABORTFLT]"); }
        if (stat & 0x01) { dbg(" [CB_DONE]"); }
        if (stat == 0x00) { dbg(" [idle]"); }
        dbg("\r\n");
    }

    /* ---- Step 1: Read current cell voltages ----
     *
     * The Main ADC must already be running.  This gives us
     * a snapshot of where each cell sits right now.
     */
    num_cells = bq79616_read_cell_voltages(dev_addr,
                                           cells_mv,
                                           BQ_MAX_CELLS);
    if (num_cells < 6)
    {
        dbg("  balance: could not read cell voltages\r\n");
        return false;
    }

    /* ---- Step 2: Find the lowest cell voltage ----
     *
     * We'll use this as the reference — only cells that are
     * significantly above this minimum will be discharged.
     */
    min_mv = cells_mv[0];
    for (i = 1; i < num_cells; i++)
    {
        /*
         * Skip cells that returned -1 (no data / error).
         * A cell reading of -1 means the ADC had no valid
         * sample for that channel yet.
         */
        if (cells_mv[i] >= 0 && cells_mv[i] < min_mv)
        {
            min_mv = cells_mv[i];
        }
    }

    /* target_mv is the "good enough" voltage.  Any cell at
     * or below this level doesn't need balancing.
     */
    target_mv = min_mv + (int32_t)delta_mv;

    snprintf(buf, sizeof(buf),
             "  Min cell = %ld mV, target = %ld mV\r\n",
             (long)min_mv, (long)target_mv);
    dbg(buf);

    /* ---- Step 3: Set per-cell balancing timers ----
     *
     * CB_CELLn_CTRL address = 0x0318 + (16 - n)
     * Cell 1  → 0x0327 (highest address)
     * Cell 16 → 0x0318 (lowest address)
     *
     * A cell gets a 120-minute timer ONLY if it is:
     *   a) above target_mv  (min + delta — needs equalising)
     *   b) above floor_mv   (VCB_DONE threshold)
     *
     * Without check (b), VCB_DONE would instantly stop cells
     * that are between target and floor, causing the chip to
     * set CB_DONE=1 immediately and never actually balance.
     *
     * 120 minutes is generous — your main loop re-calls this
     * function periodically to re-evaluate.
     */
    for (i = 0; i < num_cells; i++)
    {
        cb_addr = BQ_REG_CB_CELL16_CTRL
                  + (uint16_t)(16 - (i + 1));

        if (cells_mv[i] > target_mv
            && cells_mv[i] > (int32_t)floor_mv
            && cells_mv[i] >= 0)
        {
            /* This cell is above both thresholds — enable CBFET */
            if (!bq79616_single_write(dev_addr, cb_addr, 0x10))
            {
                dbg("  balance: CB_CELL write failed\r\n");
                return false;
            }

            snprintf(buf, sizeof(buf),
                     "  Cell%d: %ld mV -> BALANCE\r\n",
                     i + 1, (long)cells_mv[i]);
            dbg(buf);
            bal_count++;
        }
        else
        {
            /* Cell is either within delta of min, or below
             * floor — make sure its timer is off.
             */
            if (!bq79616_single_write(dev_addr, cb_addr, 0x00))
            {
                dbg("  balance: CB_CELL write failed\r\n");
                return false;
            }

            snprintf(buf, sizeof(buf),
                     "  Cell%d: %ld mV -> skip\r\n",
                     i + 1, (long)cells_mv[i]);
            dbg(buf);
        }
    }

    /* Zero out any unused cell slots above num_cells so
     * leftover timers from a previous session don't cause
     * balancing on unconnected channels.
     */
    for (i = num_cells; i < 16; i++)
    {
        cb_addr = BQ_REG_CB_CELL16_CTRL
                  + (uint16_t)(16 - (i + 1));
        bq79616_single_write(dev_addr, cb_addr, 0x00);
    }

    /* ---- Clear any latched faults (always) ----
     *
     * NVM shadow writes (active cells, OV/UV thresholds) can
     * briefly cause CRC mismatches that latch CUST_CRC faults.
     * We clear them here unconditionally — even if no cells
     * need balancing — so the 5-second status check in the
     * main loop doesn't show stale faults.
     *
     * FAULT_RST1 (0x0331): clears PWR, SYS, OVUV, OTUT
     * FAULT_RST2 (0x0332): clears COMM, OTP, COMP, PROT
     */
    bq79616_single_write(dev_addr, BQ_REG_FAULT_RST1, 0xFF);
    bq79616_single_write(dev_addr, BQ_REG_FAULT_RST2, 0xFF);
    dbg("  Latched faults cleared.\r\n");

    /* ---- If all cells are already balanced, skip ---- */
    if (bal_count == 0)
    {
        snprintf(buf, sizeof(buf),
                 "  All cells within %u mV — balanced.\r\n",
                 delta_mv);
        dbg(buf);
        dbg("=== AUTO BALANCE SKIP ===\r\n");
        return true;  /* Success — nothing to do */
    }

    snprintf(buf, sizeof(buf),
             "  %u of %d cells selected for balancing.\r\n",
             bal_count, num_cells);
    dbg(buf);

    /* ---- Step 4: Set VCB_DONE voltage floor ----
     *
     * VCB_DONE_THRESH (0x032A) tells the chip: "If any cell
     * drops below this voltage, stop balancing THAT cell."
     *
     * Encoding:
     *   0x00 = disabled (no floor)
     *   0x01 = 2450 mV
     *   ...each step is +25 mV...
     *   0x3F = 4000 mV
     *
     * Formula: code = (floor_mv - 2450) / 25 + 1
     *
     * This register needs OVUV_GO=1 afterwards to latch.
     */
    if (floor_mv < 2450)
    {
        floor_mv = 2450;
    }
    if (floor_mv > 4000)
    {
        floor_mv = 4000;
    }

    vcb_code = (uint8_t)((floor_mv - 2450) / 25) + 1;
    if (vcb_code > 0x3F)
    {
        vcb_code = 0x3F;
    }

    snprintf(buf, sizeof(buf),
             "  VCB_DONE floor = %u mV (code=0x%02X)\r\n",
             floor_mv, vcb_code);
    dbg(buf);

    if (!bq79616_single_write(dev_addr, BQ_REG_VCB_DONE_THRESH, vcb_code))
    {
        dbg("  balance: VCB_DONE write failed\r\n");
        return false;
    }

    /* ---- Step 5: Set duty cycle (odd/even swap time) ----
     *
     * BAL_CTRL1 (0x032E), DUTY[2:0] in bits 2:0.
     *
     * This controls how long the odd-numbered CBFETs stay on
     * before the chip switches to the even-numbered ones.
     * For example BQ_DUTY_30S means: odd cells balance for
     * 30 s, then even cells balance for 30 s, then repeat.
     */
    if (duty > 7) { duty = 7; }

    if (!bq79616_single_write(dev_addr, BQ_REG_BAL_CTRL1, duty))
    {
        dbg("  balance: BAL_CTRL1 write failed\r\n");
        return false;
    }

    snprintf(buf, sizeof(buf), "  Duty code = %u\r\n", duty);
    dbg(buf);

    /* ---- Step 6: Start OVUV protector (round-robin) ----
     *
     * OVUV_CTRL (0x032C):
     *   bits 1:0 = OVUV_MODE = 01  (round-robin)
     *   bit 2    = OVUV_GO   = 1   (start / latch settings)
     *
     * Write value = 0x05
     *
     * This is REQUIRED for VCB_DONE detection to work.
     * As a bonus, it also activates OV/UV fault protection
     * if you've already configured those thresholds with
     * bq79616_set_ov_thresh() / bq79616_set_uv_thresh().
     */
    if (!bq79616_single_write(dev_addr, BQ_REG_OVUV_CTRL, 0x05))
    {
        dbg("  balance: OVUV_CTRL write failed\r\n");
        return false;
    }
    dbg("  OVUV protector started (round-robin).\r\n");

    /* Let OVUV comparators settle before firing BAL_GO */
    HAL_Delay(5);

    /* ---- Step 7: Fire BAL_GO with auto-balance ----
     *
     * BAL_CTRL2 (0x032F):
     *   bit 0 = AUTO_BAL   = 1  (chip handles odd/even swap)
     *   bit 1 = BAL_GO     = 1  (start now, self-clearing)
     *   bit 2 = BAL_ACT[0] = 0  }
     *   bit 3 = BAL_ACT[1] = 0  } no action when done
     *   bit 4 = OTCB_EN    = 0  (no thermal CB pause)
     *   bit 5 = FLTSTOP_EN = 1  (stop all CB on any fault)
     *   bit 6 = CB_PAUSE   = 0
     *
     * Value = 0b00100011 = 0x23
     */
    if (!bq79616_single_write(dev_addr, BQ_REG_BAL_CTRL2, 0x23))
    {
        dbg("  balance: BAL_CTRL2 write failed\r\n");
        return false;
    }

    /* ---- Step 8: Verify balancing has started ----
     *
     * BAL_STAT (0x052B) is a read-only status register:
     *   bit 7 = INVALID_CBCONF  (1 = bad config, rejected)
     *   bit 3 = CB_RUN          (1 = balancing active)
     *   bit 2 = ABORTFLT        (1 = stopped by fault)
     *   bit 0 = CB_DONE         (1 = all cells finished)
     *
     * Give the chip 10 ms to process everything before
     * reading the status.
     */
    HAL_Delay(10);

    if (!bq79616_single_read(dev_addr, BQ_REG_BAL_STAT, &stat))
    {
        dbg("  balance: could not read BAL_STAT\r\n");
        return false;
    }

    snprintf(buf, sizeof(buf), "  BAL_STAT = 0x%02X\r\n", stat);
    dbg(buf);

    /* Check for invalid configuration (bit 7) */
    if (stat & 0x80)
    {
        dbg("  ERROR: INVALID_CBCONF — check cell config!\r\n");
        dbg("=== AUTO BALANCE FAILED ===\r\n");
        return false;
    }

    /* Check for fault-abort (bit 2) */
    if (stat & 0x04)
    {
        dbg("  ERROR: ABORTFLT — a fault stopped balancing!\r\n");

        /* Read FAULT_SUMMARY so we can see what fault fired */
        {
            uint8_t fsum = 0;

            bq79616_single_read(dev_addr, BQ_REG_FAULT_SUMMARY,
                                &fsum);
            snprintf(buf, sizeof(buf),
                     "  FAULT_SUMMARY = 0x%02X\r\n", fsum);
            dbg(buf);
        }

        dbg("=== AUTO BALANCE FAILED ===\r\n");
        return false;
    }

    /* Check CB_RUN (bit 3) */
    if (stat & 0x08)
    {
        dbg("  Balancing is running.\r\n");
    }
    else
    {
        /* Not running — read DEV_STAT to check OVUV_RUN */
        dbg("  WARNING: CB_RUN not set.\r\n");

        {
            uint8_t dev_st = 0;

            bq79616_single_read(dev_addr, 0x052C, &dev_st);
            snprintf(buf, sizeof(buf),
                     "  DEV_STAT = 0x%02X (OVUV_RUN=bit3)\r\n",
                     dev_st);
            dbg(buf);
        }
    }

    dbg("=== AUTO BALANCE OK ===\r\n");
    return true;
}



/* ================================================================
 * FILE-SCOPE STATE for manual balance state machine.
 *
 * These are outside the function so that bq79616_balance_stop()
 * can reset the state machine when the user shuts down.
 * ================================================================ */
static uint8_t  mb_state      = 0;   /* Current state (MB_* defines)   */
static uint32_t mb_timer      = 0;   /* Timestamp for timed waits      */
static int32_t  mb_clean_mv[BQ_MAX_CELLS]; /* Last ground-truth read   */
static int      mb_num_cells  = 0;   /* How many cells were read       */
static int32_t  mb_spread     = 0;   /* Last computed max-min spread   */
static uint8_t  mb_odd_count  = 0;   /* Cells enabled in odd phase     */
static uint8_t  mb_even_count = 0;   /* Cells enabled in even phase    */
static uint32_t mb_last_print = 0;   /* Rate limiter for status msgs   */
static int32_t  mb_target_mv  = 0;   /* Saved target for even phase    */
static bool     mb_from_done  = true;  /* Starts true so first entry uses
                                       * delta_resume_mv + debounce.
                                       * Set false once actively balancing,
                                       * set true when entering DONE or
                                       * after balance_stop(). */

/* Buffer to hold the latest balance status line for the display
 * function to read.  Updated every balance cycle.  The display
 * function calls bq79616_balance_get_status_line() to copy it. */
static char     mb_status_buf[128] = "BAL: waiting";
static int32_t  mb_min_mv     = 0;   /* Cached min cell voltage       */
static int32_t  mb_max_mv     = 0;   /* Cached max cell voltage       */
static uint16_t mb_bal_mask   = 0;   /* Bitmask: bit N = cell N+1 balancing */

/* ---- Debounce counters ----
 *
 * Both the DONE and RESUME decisions require the qualifying
 * condition to be met for MB_DEBOUNCE_READS consecutive
 * ground-truth reads before acting.  This prevents noise
 * spikes from stopping or starting a balance cycle.
 *
 * mb_done_count:   Increments each MB_READ where spread <=
 *                  delta_mv while actively balancing.  Resets
 *                  to 0 when spread > delta_mv.  When it hits
 *                  MB_DEBOUNCE_READS, balancing stops (DONE).
 *                  Balancing continues during the countdown
 *                  so cells keep converging.
 *
 * mb_resume_count: Increments each MB_READ where spread >
 *                  delta_resume_mv while in DONE state.  Resets
 *                  to 0 when spread drops back below.  When it
 *                  hits MB_DEBOUNCE_READS, balancing resumes.
 *
 * At ~3.6 seconds per balance cycle (one ground-truth read),
 * 10 reads = ~36 seconds of consistent data before acting.
 */
#define MB_DEBOUNCE_READS  BQ_BALANCE_DEBOUNCE_READS
static uint8_t  mb_done_count   = 0;
static uint8_t  mb_resume_count = 0;


/* ================================================================
 * bq79616_manual_balance()
 *
 * MCU-controlled cell balancing with ground-truth reads.
 *
 * HOW IT WORKS
 * ------------
 * This function is a NON-BLOCKING state machine.  Call it from
 * your main loop as often as you like -- it returns immediately
 * most of the time (just checking a timer).
 *
 *   INIT --> READ --> ODD_PHASE --> GAP --> EVEN_PHASE --> SETTLE --+
 *              ^                                                    |
 *              +----------------------------------------------------+
 *
 * GROUND-TRUTH READS (simple and reliable)
 * -----------------------------------------
 * Every cycle begins with a full simultaneous read of all 16
 * cells.  No mosaic, no pipelining.  The 1500 ms SETTLE period
 * before each read allows the external Q2 bootstrap FETs to
 * discharge:
 *
 *   Even cells' Q2: off for 1500 ms (SETTLE only)
 *     Vgs(1500) = 3.8 x e^(-1500/470) = 0.15 V — dead OFF.
 *
 *   Odd cells' Q2: off for 100 + 1000 + 1500 = 2600 ms
 *     Vgs(2600) = 3.8 x e^(-2600/470) = 0.02 V — dead OFF.
 *
 * Because every convergence check uses a simultaneous read,
 * there is no drift between odd and even halves.  This
 * prevents the false convergence seen with the mosaic
 * approach on small cells where balance current is a
 * significant C-rate.
 *
 * CELL SELECTION (outlier targeting)
 * -----------------------------------
 * Only cells that are HIGH OUTLIERS get balanced:
 *
 *   target_mv = min_cell + delta_mv
 *
 * A cell is balanced only if BOTH conditions are true:
 *   - cell voltage > target_mv  (it's a high outlier)
 *   - cell voltage > floor_mv   (safety floor)
 *
 * Cells within delta_mv of the lowest cell are LEFT ALONE.
 *
 * PHASE ASSIGNMENT (strict odd/even, NO adjacent cells)
 * -----------------------------------------------------
 * Per TI engineer guidance: with external FET balancing,
 * adjacent cells must NEVER be balanced simultaneously.
 * DEV_CONF[NO_ADJ_CB] is left at default (1).
 *
 *   Odd phase:  C1, C3, C5, C7, C9, C11, C13, C15
 *   Even phase: C2, C4, C6, C8, C10, C12, C14, C16
 *
 * TIMING PER CYCLE
 * ----------------
 *   Read:       ~5 ms   (UART read, all cells clean)
 *   Odd phase:  1000 ms (odd balancers ON)
 *   Gap:         100 ms (all OFF, CBFETs switch)
 *   Even phase: 1000 ms (even balancers ON)
 *   Settle:     1500 ms (all OFF, Q2 FET discharge)
 *   ---------
 *   Total:  ~3605 ms per cycle
 *   Active: 2000 ms (55% duty cycle)
 *
 * PARAMETERS
 * ----------
 *   dev_addr       - BQ79616 device address (usually 0x01)
 *   floor_mv       - Safety floor.  Never balance below this.
 *   delta_mv       - Stop threshold.  Declare DONE when
 *                     max-min <= delta_mv.
 *   delta_resume_mv - Restart threshold (hysteresis).  After
 *                     DONE, only resume balancing if spread
 *                     exceeds this value.  Must be >= delta_mv.
 *                     Example: delta_mv=4, delta_resume_mv=6
 *                     means stop at 4 mV, restart at 7+ mV.
 *
 * RETURNS
 * -------
 *   BQ_BAL_ACTIVE (1)  - Balancers firing
 *   BQ_BAL_DONE   (0)  - Pack balanced (spread <= delta)
 *   BQ_BAL_IDLE   (2)  - In a gap/settle or no cells qualify
 *   BQ_BAL_ERROR  (-1) - Communication failure
 * ================================================================ */
int bq79616_manual_balance(uint8_t  dev_addr,
                           uint16_t floor_mv,
                           uint16_t delta_mv,
                           uint16_t delta_resume_mv)
{
    /* ---- State machine states ---- */
    #define MB_INIT        0  /* One-time hardware setup            */
    #define MB_READ        1  /* Ground-truth read of all cells     */
    #define MB_ODD_PHASE   2  /* Odd balancers ON for 1000 ms       */
    #define MB_GAP         3  /* 100 ms all off (odd to even)       */
    #define MB_EVEN_PHASE  4  /* Even balancers ON for 1000 ms      */
    #define MB_SETTLE      5  /* 1500 ms all off (Q2 FET discharge) */
    #define MB_DONE        6  /* Pack balanced, periodic re-check   */

    /* ---- Timing constants (milliseconds) ---- */
    #define MB_PHASE_MS   1000  /* Balance phase duration           */
    #define MB_GAP_MS      100  /* Dead time between odd and even   */
    #define MB_SETTLE_MS  1500  /* Q2 FET discharge before read     */
    #define MB_DONE_MS    5000  /* Re-check interval when done      */

    /* ---- Local variables ---- */
    uint32_t now = HAL_GetTick();
    int      result = BQ_BAL_IDLE;
    int      i;

    /* Silence frame-level debug during fast state machine */
    bool saved_debug = sys_debug;
    sys_debug = false;

    switch (mb_state)
    {
        /* ================================================
         * MB_INIT: One-time hardware setup (first call).
         *
         * NO_ADJ_CB is left at its default value of 1.
         * The chip will reject any BAL_GO that has
         * adjacent cells enabled -- safety guard.
         * ================================================ */
        case MB_INIT:
        {
            if (!bq79616_balance_prepare_manual(dev_addr))
            {
                sys_debug = saved_debug;
                return BQ_BAL_ERROR;
            }

            HAL_Delay(5);

            mb_state = MB_READ;
            /* Fall through to READ */
        }
        /* FALLTHROUGH */

        /* ================================================
         * MB_READ: Read ALL 16 cells with nothing
         * balancing.  Every read is ground truth — no
         * mosaic, no stale halves.
         *
         * 1. Read all 16 cells simultaneously.
         * 2. Check if spread <= delta (balanced?).
         * 3. If not, calculate target and set up the
         *    odd-phase balancers.
         *
         * All Q2 FETs have been off since SETTLE ended
         * (1500 ms for even cells, 2600 ms for odd cells).
         * ================================================ */
        case MB_READ:
        {
            int32_t  min_mv, max_mv;
            uint16_t cb_addr;

            /* ---- Read all cells (ground truth) ---- */
            mb_num_cells = bq79616_read_cell_voltages(
                               dev_addr,
                               mb_clean_mv,
                               BQ_MAX_CELLS);

            if (mb_num_cells < 6)
            {
                sys_debug = saved_debug;
                return BQ_BAL_ERROR;
            }

            /* ---- Find min/max ---- */
            min_mv = mb_clean_mv[0];
            max_mv = mb_clean_mv[0];

            for (i = 1; i < mb_num_cells; i++)
            {
                if (mb_clean_mv[i] < min_mv)
                {
                    min_mv = mb_clean_mv[i];
                }
                if (mb_clean_mv[i] > max_mv)
                {
                    max_mv = mb_clean_mv[i];
                }
            }

            mb_spread = max_mv - min_mv;

            /* ---- Already balanced? ----
             *
             * Two debounce counters prevent noise from causing
             * false stops or false resumes:
             *
             * STOP (active → DONE):
             *   spread <= delta_mv for MB_DEBOUNCE_READS
             *   consecutive reads.  Balancing continues during
             *   the countdown so cells keep converging.
             *
             * RESUME (DONE → active):
             *   spread > delta_resume_mv for MB_DEBOUNCE_READS
             *   consecutive reads.
             *
             * The hysteresis gap (delta_mv=4 vs delta_resume_mv=15)
             * is the primary defence.  The 10-read counters are a
             * secondary filter for noise that exceeds the gap.
             *
             * At ~3.6s per cycle, 10 reads ≈ 36 seconds.
             * A single noise spike resets the counter to 0.
             */
            {
                int32_t threshold = mb_from_done
                    ? (int32_t)delta_resume_mv
                    : (int32_t)delta_mv;

                if (mb_spread <= threshold)
                {
                    /* Spread is within limits.  Reset the
                     * resume counter (streak broken). */
                    mb_resume_count = 0;

                    if (mb_from_done)
                    {
                        /* Already in DONE — stay there.
                         * No done-count needed, already stopped. */
                        mb_min_mv = min_mv;
                        mb_max_mv = max_mv;
                        mb_timer  = now;
                        mb_state  = MB_DONE;
                        result    = BQ_BAL_DONE;

                        snprintf(mb_status_buf, sizeof(mb_status_buf),
                                 "BAL DONE  spread=%ld  "
                                 "min=%ld  max=%ld",
                                 (long)mb_spread,
                                 (long)min_mv, (long)max_mv);
                        mb_last_print = now;
                        break;
                    }

                    /* Actively balancing — count consecutive
                     * reads where spread is at or below delta_mv.
                     * Keep balancing during the countdown so cells
                     * continue converging toward the target. */
                    mb_done_count++;

                    if (mb_done_count < MB_DEBOUNCE_READS)
                    {
                        /* Not enough consecutive reads yet.
                         * Show countdown, fall through to
                         * continue balancing this cycle. */
                        snprintf(mb_status_buf, sizeof(mb_status_buf),
                                 "BAL spread=%ld  "
                                 "(done in %u reads)",
                                 (long)mb_spread,
                                 (unsigned)(MB_DEBOUNCE_READS
                                            - mb_done_count));
                        mb_last_print = now;

                        /* DON'T break — fall through to target
                         * calculation and keep balancing */
                    }
                    else
                    {
                        /* 10 consecutive reads within delta.
                         * Pack is genuinely balanced — stop. */
                        mb_done_count = 0;

                        /* Zero all CB timers (safety) */
                        for (i = 0; i < 16; i++)
                        {
                            cb_addr = BQ_REG_CB_CELL16_CTRL
                                      + (uint16_t)(16 - (i + 1));
                            bq79616_single_write(dev_addr,
                                                 cb_addr, 0x00);
                        }
                        bq79616_single_write(dev_addr,
                                             BQ_REG_BAL_CTRL2,
                                             0x22);

                        mb_odd_count  = 0;
                        mb_even_count = 0;
                        mb_bal_mask   = 0;
                        mb_min_mv     = min_mv;
                        mb_max_mv     = max_mv;
                        mb_timer      = now;
                        mb_state      = MB_DONE;
                        result        = BQ_BAL_DONE;

                        snprintf(mb_status_buf, sizeof(mb_status_buf),
                                 "BAL DONE  spread=%ld  "
                                 "min=%ld  max=%ld",
                                 (long)mb_spread,
                                 (long)min_mv, (long)max_mv);
                        mb_last_print = now;
                        break;
                    }
                }
                else
                {
                    /* Spread exceeds threshold.  Reset the
                     * done counter (streak broken). */
                    mb_done_count = 0;

                    if (mb_from_done)
                    {
                        /* In DONE state — count consecutive
                         * reads above delta_resume_mv. */
                        mb_resume_count++;

                        if (mb_resume_count < MB_DEBOUNCE_READS)
                        {
                            /* Not enough reads yet — stay
                             * in DONE, keep counting. */
                            snprintf(mb_status_buf,
                                     sizeof(mb_status_buf),
                                     "BAL DONE  spread=%ld  "
                                     "(resume in %u reads)",
                                     (long)mb_spread,
                                     (unsigned)(MB_DEBOUNCE_READS
                                                - mb_resume_count));

                            mb_min_mv = min_mv;
                            mb_max_mv = max_mv;
                            mb_timer  = now;
                            mb_state  = MB_DONE;
                            result    = BQ_BAL_DONE;
                            mb_last_print = now;
                            break;
                        }

                        /* 10 consecutive reads above threshold.
                         * Genuine drift — resume balancing. */
                        mb_resume_count = 0;

                        snprintf(mb_status_buf, sizeof(mb_status_buf),
                                 "BAL RESUME  spread=%ld",
                                 (long)mb_spread);
                    }
                }
            }

            /* If we reach here, we're actively balancing
             * (either never stopped, or just resumed).
             * Clear the from-done flag so future convergence
             * checks use the tighter delta_mv threshold. */
            mb_from_done = false;

            /* ---- Calculate target ----
             *
             * target = min + delta.  Only cells ABOVE this
             * threshold get discharged.  Cells within
             * delta_mv of the lowest cell are left alone.
             */
            mb_target_mv = min_mv + (int32_t)delta_mv;

            /* ---- Pre-scan: count odd/even cells that need balancing ----
             *
             * These counts are used below to skip the odd or even
             * phase entirely if no cells in that group qualify.
             * The per-cell detail is now shown by B markers on the
             * display, so we no longer build cell-list strings here.
             */
            mb_odd_count  = 0;
            mb_even_count = 0;

            for (i = 0; i < mb_num_cells; i++)
            {
                bool needs_bal =
                    (mb_clean_mv[i] > mb_target_mv) &&
                    (mb_clean_mv[i] > (int32_t)floor_mv);

                if (!needs_bal)
                {
                    continue;
                }

                /* index 0 = C1 (odd), index 1 = C2 (even) */
                if ((i % 2) == 0)
                {
                    mb_odd_count++;
                }
                else
                {
                    mb_even_count++;
                }
            }

            /* ---- Status print ---- */
            if ((now - mb_last_print) >= 1000U)
            {
                mb_last_print = now;

                snprintf(mb_status_buf, sizeof(mb_status_buf),
                         "BAL spread=%ld  min=%ld  max=%ld",
                         (long)mb_spread,
                         (long)min_mv, (long)max_mv);
            }

            /* ---- Write ODD cell timers and start ----
             *
             * Set CB timer = 0x01 (minimum, ~126 ms) for
             * cells that need balancing.  The MCU controls
             * duration via the state machine, not the chip
             * timer.  0x00 for cells that don't need it.
             *
             * Also build mb_bal_mask — bit N set means cell
             * N+1 is being balanced this cycle.  We clear it
             * first and rebuild from scratch each MB_READ so
             * the mask always reflects the latest decisions.
             */
            mb_bal_mask = 0;   /* Start fresh each cycle */
            mb_min_mv   = min_mv;
            mb_max_mv   = max_mv;

            for (i = 0; i < mb_num_cells; i++)
            {
                cb_addr = BQ_REG_CB_CELL16_CTRL
                          + (uint16_t)(16 - (i + 1));

                bool needs_bal =
                    (mb_clean_mv[i] > mb_target_mv) &&
                    (mb_clean_mv[i] > (int32_t)floor_mv);

                if ((i % 2) == 0 && needs_bal)
                {
                    /* Odd cell — balance now */
                    bq79616_single_write(dev_addr,
                                         cb_addr, 0x01);
                    mb_bal_mask |= (uint16_t)(1U << i);
                }
                else
                {
                    bq79616_single_write(dev_addr,
                                         cb_addr, 0x00);
                }
            }

            /* Zero any unused cell slots (cells > num) */
            for (i = mb_num_cells; i < 16; i++)
            {
                cb_addr = BQ_REG_CB_CELL16_CTRL
                          + (uint16_t)(16 - (i + 1));
                bq79616_single_write(dev_addr,
                                     cb_addr, 0x00);
            }

            /* Skip odd phase if no odd cells qualify */
            if (mb_odd_count == 0)
            {
                mb_timer = now;
                mb_state = MB_GAP;
                result   = BQ_BAL_IDLE;
                break;
            }

            /* Fire BAL_GO for odd phase */
            bq79616_single_write(dev_addr,
                                 BQ_REG_FAULT_RST1, 0xFF);
            bq79616_single_write(dev_addr,
                                 BQ_REG_FAULT_RST2, 0xFF);
            bq79616_single_write(dev_addr,
                                 BQ_REG_BAL_CTRL2, 0x22);

            mb_timer = now;
            mb_state = MB_ODD_PHASE;
            result   = BQ_BAL_ACTIVE;
            break;
        }

        /* ================================================
         * MB_ODD_PHASE: Odd balancers are ON for 1000 ms.
         *
         * Cells C1,C3,C5,C7,C9,C11,C13,C15 are being
         * discharged through their external Q2 FETs.
         * No reading happens here — just wait.
         * ================================================ */
        case MB_ODD_PHASE:
        {
            if ((now - mb_timer) < MB_PHASE_MS)
            {
                result = BQ_BAL_ACTIVE;
                break;
            }

            /* Stop all balancers.
             *
             * Per datasheet Section 8.3.3.3.2:
             * "MCU can force stop cell balancing by zeroing
             *  out the balancing timer setting and issuing
             *  [BAL_GO] = 1."
             *
             * Config registers are only sampled on BAL_GO.
             * Changing them mid-session has NO effect until
             * the next BAL_GO.  So we:
             *   1. Zero all 16 CB_CELL timer registers.
             *   2. Fire BAL_GO again — chip re-reads the
             *      (now zero) timers and immediately stops.
             */
            for (i = 0; i < 16; i++)
            {
                uint16_t cb_addr = BQ_REG_CB_CELL16_CTRL
                                   + (uint16_t)(16 - (i + 1));
                bq79616_single_write(dev_addr, cb_addr, 0x00);
            }

            /* Re-issue BAL_GO so chip samples the zeroed timers */
            bq79616_single_write(dev_addr,
                                 BQ_REG_BAL_CTRL2, 0x22);

            mb_timer    = now;
            mb_state    = MB_GAP;
            mb_bal_mask = 0;   /* Odd FETs off — clear B markers */
            result      = BQ_BAL_IDLE;
            break;
        }

        /* ================================================
         * MB_GAP: All balancers OFF for 100 ms.
         *
         * This gap ensures the internal CBFETs have fully
         * switched off before the even-phase CBFETs fire.
         * Internal CBFETs switch in microseconds, but
         * 100 ms gives a comfortable margin.
         *
         * During this gap, set up even-phase CB timers
         * using the SAME target calculated in MB_READ.
         * mb_clean_mv[] hasn't changed since the read.
         * ================================================ */
        case MB_GAP:
        {
            if ((now - mb_timer) < MB_GAP_MS)
            {
                result = BQ_BAL_IDLE;
                break;
            }

            /* ---- Write EVEN cell timers ----
             *
             * Re-scan mb_clean_mv[] with the same target
             * from the READ state.  The voltages haven't
             * changed (no reads since then) so the same
             * cells will qualify.
             *
             * Add even cell bits to mb_bal_mask.  Clear the mask
             * first because odd-phase FETs are now OFF — only
             * the even cells about to turn on should show "B".
             */
            {
                uint16_t cb_addr;

                mb_bal_mask = 0;   /* Odd FETs are off now */

                for (i = 0; i < mb_num_cells; i++)
                {
                    cb_addr = BQ_REG_CB_CELL16_CTRL
                              + (uint16_t)(16 - (i + 1));

                    bool needs_bal =
                        (mb_clean_mv[i] > mb_target_mv) &&
                        (mb_clean_mv[i] > (int32_t)floor_mv);

                    if ((i % 2) == 1 && needs_bal)
                    {
                        bq79616_single_write(dev_addr,
                                             cb_addr, 0x01);
                        mb_bal_mask |= (uint16_t)(1U << i);
                    }
                    else
                    {
                        bq79616_single_write(dev_addr,
                                             cb_addr, 0x00);
                    }
                }

                for (i = mb_num_cells; i < 16; i++)
                {
                    cb_addr = BQ_REG_CB_CELL16_CTRL
                              + (uint16_t)(16 - (i + 1));
                    bq79616_single_write(dev_addr,
                                         cb_addr, 0x00);
                }

                /* Skip even phase if no even cells */
                if (mb_even_count == 0)
                {
                    mb_timer = now;
                    mb_state = MB_SETTLE;
                    result   = BQ_BAL_IDLE;
                    break;
                }

                /* Fire BAL_GO for even phase */
                bq79616_single_write(dev_addr,
                                     BQ_REG_FAULT_RST1, 0xFF);
                bq79616_single_write(dev_addr,
                                     BQ_REG_FAULT_RST2, 0xFF);
                bq79616_single_write(dev_addr,
                                     BQ_REG_BAL_CTRL2, 0x22);

                mb_timer = now;
                mb_state = MB_EVEN_PHASE;
                result   = BQ_BAL_ACTIVE;
            }
            break;
        }

        /* ================================================
         * MB_EVEN_PHASE: Even balancers are ON for 1000 ms.
         *
         * Cells C2,C4,C6,C8,C10,C12,C14,C16 are being
         * discharged.  No reading happens here.
         * ================================================ */
        case MB_EVEN_PHASE:
        {
            if ((now - mb_timer) < MB_PHASE_MS)
            {
                result = BQ_BAL_ACTIVE;
                break;
            }

            /* Stop all balancers.
             *
             * Same as odd phase: zero all CB_CELL timers,
             * then re-issue BAL_GO so the chip re-samples
             * the zeroed values and actually stops.
             * Critical here because the settle period is
             * 1500 ms — without this, the chip keeps the
             * even CBFETs active the entire time.
             */
            for (i = 0; i < 16; i++)
            {
                uint16_t cb_addr = BQ_REG_CB_CELL16_CTRL
                                   + (uint16_t)(16 - (i + 1));
                bq79616_single_write(dev_addr, cb_addr, 0x00);
            }

            /* Re-issue BAL_GO so chip samples the zeroed timers */
            bq79616_single_write(dev_addr,
                                 BQ_REG_BAL_CTRL2, 0x22);

            mb_timer    = now;
            mb_state    = MB_SETTLE;
            mb_bal_mask = 0;   /* All FETs off — clear B markers */
            result      = BQ_BAL_IDLE;
            break;
        }

        /* ================================================
         * MB_SETTLE: All balancers OFF for 1500 ms.
         *
         * This is the critical settle period that allows
         * external Q2 bootstrap FETs to discharge before
         * the next voltage read.
         *
         *   Even cells' Q2: off for 1500 ms
         *     Vgs(1500) = 3.8 x e^(-1500/470) = 0.15 V
         *
         *   Odd cells' Q2: off for 100+1000+1500 = 2600 ms
         *     Vgs(2600) = 3.8 x e^(-2600/470) = 0.02 V
         *
         * After settle, go to MB_READ for a clean read.
         * ================================================ */
        case MB_SETTLE:
        {
            if ((now - mb_timer) < MB_SETTLE_MS)
            {
                result = BQ_BAL_IDLE;
                break;
            }

            /* Settle complete — go read all cells */
            mb_state = MB_READ;
            result   = BQ_BAL_IDLE;
            break;
        }

        /* ================================================
         * MB_DONE: Pack is balanced (spread <= delta).
         * Wait 5 seconds then re-check with a fresh read.
         * ================================================ */
        case MB_DONE:
        {
            if ((now - mb_timer) < MB_DONE_MS)
            {
                result = BQ_BAL_DONE;
                break;
            }

            /* Re-check time — go to READ for fresh data.
             * Set the from-done flag so MB_READ uses the
             * looser delta_resume_mv threshold instead of
             * the tight delta_mv.  This is the hysteresis.
             */
            mb_from_done = true;
            mb_state = MB_READ;
            result   = BQ_BAL_IDLE;
            break;
        }

        default:
        {
            mb_state = MB_INIT;
            result   = BQ_BAL_ERROR;
            break;
        }
    }

    sys_debug = saved_debug;
    return result;
}


/* ================================================================
 * bq79616_balance_apply_mask()
 *
 * Writes the requested balance mask into the CB_CELL timer
 * registers and fires BAL_GO so the chip samples the new values.
 * The caller must ensure the mask only enables non-adjacent cells.
 * ================================================================ */
bool bq79616_balance_apply_mask(uint8_t  dev_addr,
                                uint16_t cell_mask,
                                uint8_t  num_cells)
{
    uint16_t cb_addr;
    uint16_t valid_mask;
    int      i;

    if (num_cells == 0U)
    {
        return false;
    }

    if (num_cells > BQ_MAX_CELLS)
    {
        num_cells = BQ_MAX_CELLS;
    }

    valid_mask = (num_cells >= 16U)
                 ? 0xFFFFU
                 : (uint16_t)((1U << num_cells) - 1U);
    cell_mask &= valid_mask;

    for (i = 0; i < 16; i++)
    {
        uint8_t timer = 0x00U;

        cb_addr = BQ_REG_CB_CELL16_CTRL
                  + (uint16_t)(16 - (i + 1));

        if ((cell_mask & (uint16_t)(1U << i)) != 0U)
        {
            timer = 0x01U;
        }

        if (!bq79616_single_write(dev_addr, cb_addr, timer))
        {
            return false;
        }
    }

    bq79616_single_write(dev_addr, BQ_REG_FAULT_RST1, 0xFF);
    bq79616_single_write(dev_addr, BQ_REG_FAULT_RST2, 0xFF);
    bq79616_single_write(dev_addr, BQ_REG_BAL_CTRL2, 0x22);

    mb_bal_mask = cell_mask;
    return true;
}


/* ================================================================
 * bq79616_balance_prepare_manual()
 *
 * Mirrors the one-time setup that the old manual balance state
 * machine performed in MB_INIT so the distributed balancer can
 * reuse the same BQ79616 configuration.
 * ================================================================ */
bool bq79616_balance_prepare_manual(uint8_t dev_addr)
{
    if (!bq79616_single_write(dev_addr, BQ_REG_VCB_DONE_THRESH, 0x00U))
    {
        return false;
    }

    if (!bq79616_single_write(dev_addr, BQ_REG_OVUV_CTRL, 0x05U))
    {
        return false;
    }

    if (!bq79616_single_write(dev_addr, BQ_REG_FAULT_RST1, 0xFFU))
    {
        return false;
    }

    if (!bq79616_single_write(dev_addr, BQ_REG_FAULT_RST2, 0xFFU))
    {
        return false;
    }

    return bq79616_balance_stop(dev_addr);
}


/* ================================================================
 * bq79616_balance_stop()
 *
 * Emergency / clean stop -- zeroes all 16 CB_CELL timer registers,
 * stops any running balance session, and resets the state machine.
 *
 * Returns true on success, false if a register write fails.
 * ================================================================ */
bool bq79616_balance_stop(uint8_t dev_addr)
{
    uint16_t cb_addr;
    int      i;

    dbg("=== BALANCE STOP ===\r\n");

    /* Zero all 16 CB_CELL timer registers first */
    for (i = 0; i < 16; i++)
    {
        cb_addr = BQ_REG_CB_CELL16_CTRL
                  + (uint16_t)(16 - (i + 1));

        if (!bq79616_single_write(dev_addr, cb_addr, 0x00))
        {
            dbg("  balance_stop: CB_CELL write failed\r\n");
            return false;
        }
    }

    /* Re-issue BAL_GO so chip re-samples the zeroed timers.
     *
     * Per datasheet: "MCU can force stop cell balancing by
     * zeroing out the balancing timer setting and issuing
     * [BAL_GO] = 1."  Config registers are only sampled
     * on BAL_GO — changes have no effect mid-session.
     */
    bq79616_single_write(dev_addr, BQ_REG_BAL_CTRL2, 0x22);

    mb_state         = 0;  /* MB_INIT */
    mb_from_done     = true;  /* Next start uses delta_resume_mv */
    mb_bal_mask      = 0;  /* No cells balancing */
    mb_done_count    = 0;  /* Reset debounce counters */
    mb_resume_count  = 0;

    dbg("  All CB timers zeroed, state machine reset.\r\n");
    return true;
}


/* ================================================================
 * bq79616_balance_get_voltages()
 *
 * Copies the best-known CLEAN cell voltage mosaic from the
 * manual balance state machine's internal buffer.
 *
 * These voltages are a MOSAIC -- each cell was read at a
 * different time, but always when that cell's external Q2
 * MOSFET was confirmed OFF:
 *
 *   - Odd cells (C1,C3,C5...) are updated at 950 ms into
 *     the EVEN balance phase (Q2 off for 1050+ ms).
 *   - Even cells (C2,C4,C6...) are updated at 950 ms into
 *     the ODD balance phase (Q2 off for 1050+ ms).
 *
 * All values represent TRUE open-circuit voltages with no
 * balance current contamination.
 *
 * Returns: number of cells copied into mv_out[]
 *          0 if no data yet (balance hasn't run first READ)
 * ================================================================ */
int bq79616_balance_get_voltages(int32_t *mv_out,
                                 uint8_t  max_cells)
{
    int count;
    int i;

    if (mb_num_cells == 0) { return 0; }

    count = (mb_num_cells < (int)max_cells)
            ? mb_num_cells : (int)max_cells;

    for (i = 0; i < count; i++)
    {
        mv_out[i] = mb_clean_mv[i];
    }

    return count;
}

/* ================================================================
 * bq79616_balance_get_status_line()
 *
 * Returns a pointer to the balance state machine's last status
 * string.  This is updated internally by bq79616_manual_balance()
 * every ~1 second during active balancing.
 *
 * The caller should NOT modify or free the returned string.
 * It points to a static buffer inside bq79616.c.
 *
 * Typical values:
 *   "BAL: waiting"
 *   "BAL s=6 t=4017 O:[C1,C9]-2 E:[C8]-1 mn=4013 mx=4019"
 *   "BAL DONE  spread=4 mV  min=4015  max=4019"
 *   "BAL RESUME  spread=7 mV  (threshold=6)"
 * ================================================================ */
const char *bq79616_balance_get_status_line(void)
{
    return mb_status_buf;
}

/* ================================================================
 * bq79616_get_fault_line()
 *
 * Returns a pointer to the one-line fault summary string.
 * Updated by bq79616_get_status().
 *
 * Typical values:
 *   "FAULT: OK"
 *   "FAULT: 0x0C OV UV"
 *   "FAULT: READ FAILED"
 * ================================================================ */
const char *bq79616_get_fault_line(void)
{
    return fault_line_buf;
}


/* ================================================================
 * bq79616_balance_get_mask()
 *
 * Returns a 16-bit bitmask showing which cells are currently
 * being balanced.  Bit 0 = Cell 1, bit 15 = Cell 16.
 * A set bit means that cell's external MOSFET is being driven.
 *
 * The mask is updated each balance cycle in MB_READ (odd cells)
 * and MB_GAP (even cells), and cleared when balancing enters
 * DONE or is stopped via bq79616_balance_stop().
 *
 * Usage in display code:
 *   uint16_t mask = bq79616_balance_get_mask();
 *   if (mask & (1U << i))  ->  cell i+1 is balancing
 * ================================================================ */
uint16_t bq79616_balance_get_mask(void)
{
    return mb_bal_mask;
}


/* ================================================================
 * bq79616_balance_get_min_mv / max_mv / spread_mv
 *
 * Return the cached min, max, and spread (max-min) from the
 * most recent ground-truth cell voltage read in MB_READ.
 * Updated every balance cycle (~3.6 seconds).
 * ================================================================ */
int32_t bq79616_balance_get_min_mv(void)
{
    return mb_min_mv;
}

int32_t bq79616_balance_get_max_mv(void)
{
    return mb_max_mv;
}

int32_t bq79616_balance_get_spread_mv(void)
{
    return mb_spread;
}
