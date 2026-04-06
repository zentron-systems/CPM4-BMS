//bq79616.h created Tuesday 11/02/2026 at 18:20
#ifndef INC_BQ79616_H_
#define INC_BQ79616_H_



#include "main.h"    /* Pulls in the correct HAL for STM32G0 */
#include <stdbool.h>
#include <stdint.h>

/* ===== Hardware mapping from IOC =====
   PC0  -> USART6_TX_BQ79616  (to BQ RX)
   PC1  -> USART6_RX_BQ79616  (from BQ TX) */
#define BQ_UART_HANDLE            huart6
#define BQ_UART_TX_GPIO_Port      GPIOC
#define BQ_UART_TX_Pin            GPIO_PIN_0

/* Provided by CubeMX in usart.c */
extern UART_HandleTypeDef BQ_UART_HANDLE;

/* ===== Register addresses (from datasheet Section 8.5) ===== */
#define BQ_REG_PARTID             0x0500  /* Read-only, expected 0x21      */
#define BQ_REG_COMM_TIMEOUT_CONF  0x0019  /* NVM shadow, comm timeout cfg  */

/* Customer OTP CRC registers (Section 8.5.4.3.13-16) */
#define BQ_REG_CUST_CRC_HI       0x0036  /* NVM: host-written expected CRC high */
#define BQ_REG_CUST_CRC_LO       0x0037  /* NVM: host-written expected CRC low  */
#define BQ_REG_CUST_CRC_RSLT_HI  0x050C  /* Read-only: device-computed CRC high */
#define BQ_REG_CUST_CRC_RSLT_LO  0x050D  /* Read-only: device-computed CRC low  */

/* Fault reset registers (Section 8.5.4.12.3-4) */
#define BQ_REG_FAULT_RST1        0x0331  /* Write bits to clear fault groups 1  */
#define BQ_REG_FAULT_RST2        0x0332  /* Write bits to clear fault groups 2  */

/* ===== Fault status registers (datasheet Section 8.5.4.13) ===== */
#define BQ_REG_FAULT_SUMMARY      0x052D  /* Top-level: 1 bit per fault group  */
#define BQ_REG_FAULT_COMM1        0x0530  /* UART faults                       */
#define BQ_REG_FAULT_COMM2        0x0531  /* Daisy-chain COMH/COML faults      */
#define BQ_REG_FAULT_COMM3        0x0532  /* Heartbeat / fault-tone / FCOMM    */
#define BQ_REG_FAULT_OTP          0x0535  /* OTP CRC / load errors             */
#define BQ_REG_FAULT_SYS          0x0536  /* System: reset, timeout, thermal   */
#define BQ_REG_FAULT_PROT1        0x053A  /* Protector parity faults           */
#define BQ_REG_FAULT_PROT2        0x053B  /* Protector BIST faults             */
#define BQ_REG_FAULT_OV1          0x053C  /* Overvoltage cells 16..9           */
#define BQ_REG_FAULT_OV2          0x053D  /* Overvoltage cells 8..1            */
#define BQ_REG_FAULT_UV1          0x053E  /* Undervoltage cells 16..9          */
#define BQ_REG_FAULT_UV2          0x053F  /* Undervoltage cells 8..1           */
#define BQ_REG_FAULT_OT           0x0540  /* Overtemp GPIO 8..1                */
#define BQ_REG_FAULT_UT           0x0541  /* Undertemp GPIO 8..1               */
#define BQ_REG_FAULT_PWR1         0x0552  /* Power rail: AVDD, DVDD, CVDD      */
#define BQ_REG_FAULT_PWR2         0x0553  /* Power rail: TSREF, NEG5V, REFH    */
#define BQ_REG_FAULT_PWR3         0x0554  /* Power rail: AVDD UV reset         */

/* ===== Expected register values ===== */
#define BQ79616_PARTID_VALUE      0x21    /* PARTID == 0x21 means BQ79616  */

/* COMM_TIMEOUT_CONF value we write during init:
 *   Bit 7     = 0   (spare)
 *   Bits 6-4  = 000 (CTS short timeout disabled)
 *   Bit 3     = 1   (CTL_ACT: go to SHUTDOWN, not SLEEP)
 *   Bits 2-0  = 011 (CTL_TIME: 10-second long timeout)
 *   Combined  = 0x0B                                       */
#define BQ_COMM_TIMEOUT_10S_SHTDN 0x0B

/* ===== Frame-building constants (datasheet Table 8-10) ===== */
#define BQ_FRAME_TYPE_CMD         0x80    /* Bit 7 = 1: command from host  */
#define BQ_REQ_SINGLE_READ        0x00    /* REQ_TYPE = 000                */
#define BQ_REQ_SINGLE_WRITE       0x10    /* REQ_TYPE = 001                */
#define BQ_REQ_BROADCAST_WRITE    0x50    /* REQ_TYPE = 101                */

/* ===== Protector threshold registers (Section 8.5.4.8) ===== */
#define BQ_REG_OV_THRESH          0x0009  /* NVM: OV_THR[5:0] in bits 5:0  */
#define BQ_REG_UV_THRESH          0x000A  /* NVM: UV_THR[5:0] in bits 5:0  */
#define BQ_REG_OVUV_CTRL         0x032C  /* RW: OVUV_GO, OVUV_MODE[1:0]  */

/* ===== Balancing registers (Section 8.5.4.7) ===== */
#define BQ_REG_CB_CELL16_CTRL     0x0318  /* RW: CB timer, Cell16 (lowest) */
#define BQ_REG_CB_CELL1_CTRL      0x0327  /* RW: CB timer, Cell1 (highest) */
/* Formula: CB_CELLn_CTRL = 0x0318 + (16 - n) */
#define BQ_REG_VCB_DONE_THRESH    0x032A  /* RW: stop-balancing voltage    */
#define BQ_REG_BAL_CTRL1          0x032E  /* RW: DUTY[2:0] odd/even swap  */
#define BQ_REG_BAL_CTRL2          0x032F  /* RW: BAL_GO, AUTO_BAL, etc.   */
#define BQ_REG_BAL_STAT           0x052B  /* RO: CB_RUN, CB_DONE, etc.    */

/* ===== GPIO configuration registers (Section 8.5.4.9) ===== */
#define BQ_REG_GPIO_CONF1         0x000E  /* NVM: GPIO1[2:0], GPIO2[2:0]   */
#define BQ_REG_GPIO_CONF2         0x000F  /* NVM: GPIO3[2:0], GPIO4[2:0]   */
#define BQ_REG_GPIO_CONF3         0x0010  /* NVM: GPIO5[2:0], GPIO6[2:0]   */
#define BQ_REG_GPIO_CONF4         0x0011  /* NVM: GPIO7[2:0], GPIO8[2:0]   */
#define BQ_REG_GPIO_STAT          0x052A  /* Read-only: digital state GPIO1-8 */

/* ===== Control registers (Section 8.5.4.3) ===== */
#define BQ_REG_DEV_CONF           0x0002  /* NVM: NO_ADJ_CB, NFAULT_EN etc */
#define BQ_REG_ACTIVE_CELL        0x0003  /* NVM: NUM_CELL[3:0] in bits 3:0 */
#define BQ_REG_CONTROL2           0x030A  /* RW: TSREF_EN, SEND_HW_RESET   */

/* ===== ADC control registers (Section 8.5.4.5) ===== */
#define BQ_REG_ADC_CTRL1          0x030D  /* RW: MAIN_GO, MAIN_MODE[1:0]   */

/* ===== ADC calibration registers (Section 8.5.4.5.3-4) =====
 *
 * MAIN_ADC_CAL1 (0x001B): GAINL[7:0]
 *   Lower 8 bits of the 9-bit gain correction.
 *   Step size: 0.0031%.  Range: -0.78125% to +0.7782%.
 *
 * MAIN_ADC_CAL2 (0x001C): [GAINH | OFFSET[6:0]]
 *   Bit 7:   GAINH — MSB (bit 8) of the 9-bit gain field.
 *   Bits 6:0: OFFSET[6:0] — 7-bit signed offset correction.
 *   Step size: 0.19073 mV.  Range: -12.207 mV to +12.016 mV.
 *
 * Both are NVM shadow registers — writing them requires
 * a CRC update (same as OV/UV threshold writes).
 * The chip applies these corrections automatically to
 * every Main ADC conversion result.
 */
#define BQ_REG_MAIN_ADC_CAL1     0x001B  /* NVM: GAINL[7:0]               */
#define BQ_REG_MAIN_ADC_CAL2     0x001C  /* NVM: GAINH | OFFSET[6:0]      */

/* ===== Cell voltage result registers (Section 8.5.4.6.1-16) =====
 *
 * Stored in REVERSE order: Cell16 is at the lowest address.
 *   VCELL16_HI = 0x0568,  VCELL1_HI = 0x0586
 *   Formula: VCELLn_HI = 0x0568 + (16 - n) * 2
 *
 * Cell voltage LSB = 190.73 μV (VLSB_ADC), different from GPIO's 152.59 μV.
 * Results are 16-bit signed 2's complement.  Reset value 0x8000 = no data.
 */
#define BQ_REG_VCELL16_HI         0x0568  /* Highest cell = lowest address  */
#define BQ_REG_VCELL1_HI          0x0586  /* Lowest cell = highest address  */

/* ===== GPIO ADC result registers (Section 8.5.4.6.19-26) =====
 *
 * Each GPIO has a HI and LO byte.  Results are 16-bit signed
 * (2's complement).  Conversion: millivolts = raw * 152.59 / 1000.
 * Reset value 0x8000 means "no data yet" (ADC hasn't run).
 *
 * IMPORTANT: Always read HI first — this locks LO from updating
 * until LO is also read.  Guarantees an atomic 16-bit sample.
 */
#define BQ_REG_GPIO1_HI           0x058E  /* GPIO1 ADC result high byte    */
#define BQ_REG_GPIO1_LO           0x058F  /* GPIO1 ADC result low byte     */
/* GPIO2 = 0x0590/91, GPIO3 = 0x0592/93, ... GPIO8 = 0x059C/9D
 * Formula: GPIOn_HI = 0x058E + (n-1)*2                         */

/* ===== GPIO pin mode values (Arduino-style) =====
 *
 * These map to the 3-bit codes in GPIO_CONF registers:
 *   000 = disabled (high-Z)  ← not exposed, use for future if needed
 *   001 = ADC + OTUT input   ← not exposed separately
 *   010 = ADC only input     ← BQ_ADC
 *   011 = digital input      ← BQ_INPUT
 *   100 = output high        ← BQ_OUTPUT (default state = high)
 *   101 = output low         ← used internally by digital_write
 *   110 = ADC + weak pull-up ← not exposed separately
 *   111 = ADC + weak pull-dn ← not exposed separately
 */
#define BQ_INPUT    0x03    /* Digital input  (code 011) */
#define BQ_OUTPUT   0x04    /* Digital output (code 100 = high) */
#define BQ_ADC      0x02    /* ADC only input (code 010) */

/* ===== Timing (from datasheet Section 7.6) ===== */
#define BQ_WAKE_PING_LOW_MS       3       /* tHLD_WAKE min is 2 ms         */
#define BQ_WAKE_SETTLE_MS         15      /* tSU(WAKE_SHUT) max is 10 ms   */
#define BQ_UART_TIMEOUT_MS        10      /* HAL receive/transmit timeout  */

/* ===== Global debug flag =====
 *
 * Set sys_debug = true to see all BQ79616 driver debug output.
 * Set sys_debug = false to silence it (only your application
 * prints will appear on the terminal).
 * Defined in bq79616.c, accessible from anywhere.
 */
extern bool sys_debug;

/* ===== Public API ===== */
bool bq79616_init(uint8_t dev_id);       /* Wake, verify PARTID, arm 10 s safety timeout */
bool bq79616_get_status(uint8_t dev_id); /* Read all fault regs, print details, true=no faults */
bool bq79616_shutdown(uint8_t dev_id);   /* Shutdown ping, confirm loss of comms          */

/* GPIO functions (Arduino-style) */
bool bq79616_pin_mode(uint8_t dev_id, uint8_t pin, uint8_t mode);
bool bq79616_digital_write(uint8_t dev_id, uint8_t pin, uint8_t state);
int  bq79616_digital_read(uint8_t dev_id, uint8_t pin);

/* ADC / analogue functions */
bool    bq79616_tsref_enable(uint8_t dev_id);    /* Enable TSREF LDO (thermistor bias)  */
bool    bq79616_adc_start(uint8_t dev_id);       /* Start Main ADC continuous mode       */
bool    bq79616_adc_stop(uint8_t dev_id);        /* Stop Main ADC                        */
int32_t bq79616_analog_read(uint8_t dev_id, uint8_t pin); /* Read GPIO in mV, -1 on err */

/* ADC calibration
 *
 * Writes an offset correction to MAIN_ADC_CAL2[OFFSET].
 * The chip applies this to every Main ADC conversion.
 *   offset_mv: signed millivolt correction (-12 to +12).
 *              Negative = readings were too high.
 *              Example: meter reads 4032, BQ reads 4037
 *                       → offset_mv = -5
 */
bool bq79616_set_adc_offset(uint8_t dev_id, int8_t offset_mv);

/* Cell voltage functions */
#define BQ_MAX_CELLS  16   /* Maximum cells the BQ79616 supports   */
bool bq79616_set_active_cells(uint8_t dev_id, uint8_t cells);
int  bq79616_read_cell_voltages(uint8_t dev_id, int32_t mv_out[], uint8_t max_cells);

/* Protector threshold functions */
bool bq79616_set_ov_thresh(uint8_t dev_id, uint16_t mv);
bool bq79616_set_uv_thresh(uint8_t dev_id, uint16_t mv);

/* Cell balancing functions
 *
 * Duty codes for odd/even swap time (BAL_CTRL1[DUTY2:0]):
 *   BQ_DUTY_5S   = 0  →   5 seconds
 *   BQ_DUTY_10S  = 1  →  10 seconds
 *   BQ_DUTY_30S  = 2  →  30 seconds
 *   BQ_DUTY_60S  = 3  →  60 seconds
 *   BQ_DUTY_5M   = 4  →   5 minutes
 *   BQ_DUTY_10M  = 5  →  10 minutes
 *   BQ_DUTY_20M  = 6  →  20 minutes
 *   BQ_DUTY_30M  = 7  →  30 minutes
 */
#define BQ_DUTY_5S    0
#define BQ_DUTY_10S   1
#define BQ_DUTY_30S   2
#define BQ_DUTY_60S   3
#define BQ_DUTY_5M    4
#define BQ_DUTY_10M   5
#define BQ_DUTY_20M   6
#define BQ_DUTY_30M   7

bool bq79616_auto_balance(uint8_t dev_id,
                          uint16_t floor_mv,
                          uint16_t delta_mv,
                          uint8_t  duty);

/* Shared pack-balance policy values.
 *
 * The old local balancer and the new master-coordinated balancer
 * both use the same thresholds so behaviour stays consistent. */
#define BQ_BALANCE_DELTA_MV          4U
#define BQ_BALANCE_RESUME_MV         6U
#define BQ_BALANCE_DEBOUNCE_READS   10U

/* ===== MCU-controlled manual balance (Section 8.5.4.7) =====
 *
 * Unlike auto_balance (where the chip handles odd/even swapping),
 * manual_balance gives the MCU full control.  Call it at a fixed
 * rate (e.g. every 200 ms from your main loop).  Each call:
 *
 *   1. Reads all cell voltages (silently, no debug flood)
 *   2. Checks if the pack is already balanced (spread <= delta)
 *   3. Picks cells above floor_mv in the current phase (odd/even)
 *   4. Fires BAL_GO to turn on those CBFETs
 *   5. Toggles phase for the next call
 *
 * Return codes tell you what happened:
 */
#define BQ_BAL_DONE     0   /* Spread <= delta — pack is balanced     */
#define BQ_BAL_ACTIVE   1   /* CBFETs firing this cycle               */
#define BQ_BAL_IDLE     2   /* No cells above floor, but spread>delta */
#define BQ_BAL_ERROR   (-1) /* Communication failure                  */

int  bq79616_manual_balance(uint8_t  dev_id,
                            uint16_t floor_mv,
                            uint16_t delta_mv,
                            uint16_t delta_resume_mv);

/* Prepare the BQ79616 for MCU-driven balance masks by disabling the
 * hardware VCB_DONE floor and clearing any leftover CB timers/faults. */
bool bq79616_balance_prepare_manual(uint8_t dev_id);

/* Apply a specific per-cell balance mask immediately.
 * Bit 0 = Cell 1, bit 15 = Cell 16.  The caller is responsible
 * for ensuring no adjacent cells are enabled together. */
bool bq79616_balance_apply_mask(uint8_t  dev_id,
                                uint16_t cell_mask,
                                uint8_t  num_cells);

bool bq79616_balance_stop(uint8_t dev_id);

/* Get the last CLEAN (unloaded) cell voltages from the
 * manual balance state machine.  These are only valid after
 * manual_balance() has completed at least one READ cycle.
 * Returns number of cells copied, or 0 if no data yet.
 */
int  bq79616_balance_get_voltages(int32_t *mv_out,
                                  uint8_t  max_cells);

/* Returns a pointer to the balance status string (static buffer).
 * Updated every ~1 second by bq79616_manual_balance().
 * Do NOT modify or free the returned pointer. */
const char *bq79616_balance_get_status_line(void);

/* Returns a pointer to the one-line fault summary string.
 * Updated by bq79616_get_status().  Do NOT modify or free. */
const char *bq79616_get_fault_line(void);

/* Returns a 16-bit bitmask of cells currently balancing.
 * Bit 0 = Cell 1, bit 15 = Cell 16.  Cleared when DONE. */
uint16_t bq79616_balance_get_mask(void);

/* Cached min/max/spread from the last ground-truth read.
 * Updated each balance cycle (~3.6s). */
int32_t bq79616_balance_get_min_mv(void);
int32_t bq79616_balance_get_max_mv(void);
int32_t bq79616_balance_get_spread_mv(void);

#endif /* INC_BQ79616_H_ */
