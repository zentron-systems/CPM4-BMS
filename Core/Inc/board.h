//board.h created Sunday 15/02/2026 at 11:05 updated Sunday 15/02/2026 at 16:58

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#include <stdbool.h>
#include <stdint.h>

/* ===================================================================
 *  Board identity — always set by the CONFIG_ADDRESS jumpers
 *
 *  Address 0 = master board (runs CAN bus, controls contactors)
 *  Address 1..7 = slave boards (daisy-chained via BQ79616 comms)
 *
 *  CONFIG_ADDRESS_J1 (PB9)  = bit 2 (MSB)
 *  CONFIG_ADDRESS_J2 (PC12) = bit 1
 *  CONFIG_ADDRESS_J3 (PC2)  = bit 0 (LSB)
 *
 *  Expected slaves = how many slave boards are daisy-chained.
 *  NUM_SLAVES_J2 (PC3) = bit 2 (MSB)
 *  NUM_SLAVES_J1 (PA0) = bit 1
 *  NUM_SLAVES_J0 (PA1) = bit 0 (LSB)
 * =================================================================== */
#define BOARD_ADDR_MASTER  0
#define BOARD_ADDR_SLAVE1  1
#define BOARD_ADDR_SLAVE2  2
#define BOARD_ADDR_SLAVE3  3
#define BOARD_ADDR_SLAVE4  4
#define BOARD_ADDR_SLAVE5  5
#define BOARD_ADDR_SLAVE6  6
#define BOARD_ADDR_SLAVE7  7

/* Returns the board's address (0 = master, 1..7 = slave). */
uint8_t  board_get_address(void);

/* Returns how many slave boards are expected on the daisy chain.
 * Only meaningful on the master board (address 0). */
uint8_t  board_get_num_expected_slaves(void);

/* Refresh board identity from the physical jumpers.
 * Safe to call anytime after GPIO init. */
void     board_refresh_identity_from_jumpers(void);

/* Returns how many cells this board is configured to monitor
 * (6..16) based on the active jumper/default path. */
uint8_t  board_get_active_cell_count(void);

/* Returns the balance floor voltage in mV, derived from the
 * chemistry/mode jumpers and used by the balancing logic. */
uint16_t board_get_bal_floor_mv(void);

/* Update the runtime-only config values from the hardware jumpers
 * without touching the BQ79616.  Used by master-only boards so the
 * pack-level logic still uses the same floor/cell-count policy. */
void     board_load_runtime_config_from_jumpers(void);

/* Apply the same runtime defaults as board_set_defaults() but
 * without writing any BQ79616 registers. */
void     board_load_runtime_defaults(void);


/* ===================================================================
 *  board_setup_from_config_jumpers
 *
 *  Reads ALL hardware config jumpers on the PCB and configures
 *  the BQ79616 and MCU warning thresholds accordingly:
 *
 *    - Board address         (CONFIG_ADDRESS_J1..J3)
 *    - Expected slave count  (NUM_SLAVES_J2..J0)
 *    - Number of active cells (CONFIG_CELLS_J1..J4)
 *    - Chemistry type         (CONFIG_ION_LFP)
 *    - OV / UV thresholds     (CONFIG_HICAP_LONGLIFE + chemistry)
 *    - MCU warning thresholds (derived from OV/UV)
 *    - Balance floor voltage  (chemistry-dependent)
 *
 *  Must be called AFTER bq79616_init() and BEFORE bq79616_adc_start()
 *  because it writes NVM shadow registers (ACTIVE_CELL, OV_THRESH,
 *  UV_THRESH) which are sampled when the ADC starts.
 *
 *  Parameters:
 *    dev_id  - BQ79616 device address (usually 0x01)
 *
 *  Returns:
 *    true  - all settings applied successfully
 *    false - a BQ79616 call failed (check debug output)
 * =================================================================== */
bool board_setup_from_config_jumpers(uint8_t dev_id);

/* ===================================================================
 *  board_set_defaults
 *
 *  Hardcoded configuration for when CONFIG_NO_APP is open
 *  (app/PC will manage the board).  Sets safe Li-Ion Long-Life defaults:
 *    - 16 cells, OV=4175, UV=3100
 *    - MCU warnings: OV=4150, BAL=4085, UV=3150
 *    - Board identity still comes from the jumpers
 *    - Balance floor = 3800 mV
 *
 *  Must be called AFTER bq79616_init() and BEFORE bq79616_adc_start().
 * =================================================================== */
bool board_set_defaults(uint8_t dev_id);


/* ===================================================================
 *  board_contactor_startup
 *
 *  Closes the battery contactors in the correct sequence to safely
 *  connect the high-voltage bus.  Based on the Nissan Leaf charge
 *  turn-on timing diagram.
 *
 *  Each contactor is driven by a TPS1H200A-Q1 smart high-side switch.
 *  The FAULT output of each driver is open-drain, active LOW:
 *    HIGH = no fault (normal)
 *    LOW  = fault (overload, short-to-GND, open load, or thermal)
 *
 *  Sequence:
 *    1. Enable diagnostics on all 3 drivers  (PA15 HIGH)
 *    2. Close negative contactor              (PC9  HIGH)
 *       - check for driver fault on PD2
 *    3. Wait 200 ms
 *    4. Close precharge contactor             (PD3  HIGH)
 *       - check for driver fault on PD4
 *    5. Wait for HV bus to charge             (PB14 goes HIGH)
 *       - 500 ms timeout from precharge close
 *       - if timeout: abort and open everything
 *    6. Close positive contactor              (PD5  HIGH)
 *       - check for driver fault on PC8
 *    7. Wait 100 ms then open precharge       (PD3  LOW)
 *
 *  If any TPS1H200A reports a fault at any step, the sequence is
 *  aborted and all contactors are opened.
 *
 *  PIN MAP (from CPM-4-16.ioc):
 *    PA15 = DIAG_EN_CONTACTORS    (output, enables diag on all 3)
 *    PC9  = MCU_CONTACTOR-        (output, negative contactor IN)
 *    PD5  = MCU_CONTACTOR+        (output, positive contactor IN)
 *    PD3  = MCU_PRECHARGE         (output, precharge contactor IN)
 *    PD2  = MCU_CONTACTOR-FAULT   (input,  negative FAULT pin)
 *    PC8  = MCU_CONTACTOR+FAULT   (input,  positive FAULT pin)
 *    PD4  = MCU_PRECHARGE_FAULT   (input,  precharge FAULT pin)
 *    PB14 = MCU_HV_PRESENT        (input,  HIGH = bus charged)
 *
 *  Returns:
 *    true  - all contactors closed, HV bus live
 *    false - fault or timeout, all contactors opened safely
 * =================================================================== */
bool board_contactor_startup(void);


/* ===================================================================
 *  board_contactor_shutdown
 *
 *  Opens the battery contactors in the correct sequence to safely
 *  disconnect the high-voltage bus.
 *
 *  Sequence:
 *    1. Open positive contactor   (PD5  LOW)
 *    2. Wait 50 ms
 *    3. Open precharge            (PD3  LOW)  (in case it was left on)
 *    4. Wait 50 ms
 *    5. Open negative contactor   (PC9  LOW)
 *    6. Disable diagnostics       (PA15 LOW)
 *
 *  No return value - shutdown always completes.
 * =================================================================== */
void board_contactor_shutdown(void);


/* ===================================================================
 *  MCU-level voltage warning outputs
 *
 *  Soft warnings on MCU GPIOs, separate from the BQ79616's hardware
 *  OV/UV fault protector.  These are early-warning indicators:
 *
 *    PA5 = MCU_OVER_VOLTAGE   HIGH when any cell >= ov_warn
 *    PA4 = MCU_BALANCING      HIGH when any cell >= bal_warn
 *    PA6 = MCU_UNDER_VOLTAGE  HIGH when any cell <= uv_warn
 *
 *  Non-latching, no hysteresis.  Pin goes LOW as soon as
 *  no cell meets the condition any more.
 * =================================================================== */

/* Set the overvoltage warning threshold (mV).
 * Any cell >= this value -> PA5 (MCU_OVER_VOLTAGE) goes HIGH. */
void board_set_ov_warn_thresh(uint16_t mv);

/* Set the balance warning threshold (mV).
 * Any cell >= this value -> PA4 (MCU_BALANCING) goes HIGH. */
void board_set_bal_warn_thresh(uint16_t mv);

/* Set the undervoltage warning threshold (mV).
 * Any cell <= this value -> PA6 (MCU_UNDER_VOLTAGE) goes HIGH. */
void board_set_uv_warn_thresh(uint16_t mv);

/* Check all cells against the three warning thresholds and
 * drive the corresponding GPIOs HIGH or LOW.
 *
 * Parameters:
 *   cell_mv   - array of cell voltages in millivolts
 *   num_cells - number of valid entries in cell_mv
 *
 * Call this every main loop pass.  It only reads the cached
 * voltage array (no UART traffic) so it's essentially free. */
void board_check_voltage_warnings(const int32_t cell_mv[],
                                  uint8_t       num_cells);


/* ===================================================================
 *  Insulation resistance measurement
 *
 *  Measures the isolation between the HV battery and the vehicle
 *  chassis using a voltage-divider circuit on BQ79616 GPIO6 ADC.
 *
 *  Circuit overview:
 *    CELL_TOP --- R268(180MΩ) --+-- R269(180MΩ) --- GNDHV
 *                                |
 *                           chassis GND
 *                                |
 *                           R270(400MΩ)
 *                                |
 *                        ISOTEST_DIV_NODE --- R271(10k) --- GPIO6
 *                                |                           |
 *                           R272(2.74MΩ)               C270(3.3nF)
 *                                |                           |
 *                              GNDHV                       GNDHV
 *
 *  R268/R269 bias chassis to V_pack/2 (known reference point).
 *  R270/R272 scale that down to the BQ ADC's 0-5V range.
 *  If insulation degrades, the chassis voltage shifts from
 *  V_pack/2 and the function back-calculates the fault resistance.
 *
 *  Parameters:
 *    dev_id    - BQ79616 device address (0x01)
 *    cell_mv   - array of cell voltages in millivolts
 *    num_cells - number of valid entries in cell_mv
 *
 *  Prints results to debug UART.  Call periodically (e.g. every
 *  5 seconds) from application_loop().
 * =================================================================== */
void board_measure_insulation(uint8_t        dev_id,
                              const int32_t  cell_mv[],
                              uint8_t        num_cells);

/* ===================================================================
 *  USART1 DMA transmit helpers
 *
 *  All terminal output (debug prints, status screen) goes through
 *  these functions instead of blocking HAL_UART_Transmit().
 *
 *  How it works:
 *    - A single static 1536-byte buffer lives in board.c
 *    - uart1_dma_send() copies your data into that buffer and
 *      fires a DMA transfer.  The CPU is free immediately.
 *    - uart1_dma_buf() lets you build a large message (like the
 *      status screen) directly inside the DMA buffer, then
 *      uart1_dma_send_buf() fires DMA with no copy.
 *    - uart1_dma_wait() blocks until the current DMA finishes.
 *      Only needed if you must guarantee the data has been sent
 *      before continuing (e.g., right before power-off).
 *
 *  If a new send is requested while the previous DMA is still
 *  running, the function waits for it to complete first.  This
 *  is rare (only happens if you send faster than 115200 bps can
 *  drain) and is still much better than blocking on every byte.
 *
 *  Requires CubeMX: USART1 TX DMA channel + USART1 global IRQ.
 * =================================================================== */

#define UART1_TX_SIZE  1536  /* DMA buffer size in bytes */

/* Send a string via DMA.  Copies len bytes from data into the
 * internal DMA buffer, then fires DMA.  Safe to call with
 * stack-local source buffers — the copy happens before return.
 * Waits for any in-progress DMA to finish before starting. */
void uart1_dma_send(const uint8_t *data, uint16_t len);

/* Returns a pointer to the internal DMA buffer so you can
 * build content directly inside it with snprintf(), avoiding
 * a copy.  Waits for any in-progress DMA to finish first.
 * Use uart1_dma_send_buf() to send when you're done building. */
uint8_t *uart1_dma_buf(void);

/* Fire DMA to send the first 'len' bytes of the internal buffer.
 * Call this after building content via uart1_dma_buf().
 * Does NOT wait — returns immediately while DMA runs. */
void uart1_dma_send_buf(uint16_t len);

/* Block until any in-progress DMA transfer completes.
 * Call before power-off or when you need to guarantee the
 * terminal has received everything. */
void uart1_dma_wait(void);


/* Result-line getters — return pointers to static buffers
 * containing the last formatted measurement result.
 * Used by the display engine in application.c to build the
 * status screen.  Do NOT modify or free the returned strings.
 */
const char *board_get_iso_line(void);
const char *board_get_bal_temp_line(void);
const char *board_get_batt_temp_line(void);


/* ===================================================================
 *  NTC temperature measurement
 *
 *  Reads Murata NCP18XH103F03RB NTC thermistors (10 kΩ at 25°C,
 *  B25/50 = 3380 K) connected via the TSREF voltage divider
 *  circuit on BQ79616 GPIO pins.
 *
 *  Circuit: TSREF(5V) → R_fixed(10k) → junction → NTC → GNDHV
 *           GPIO pin reads the junction voltage through an RC filter.
 *
 *  All five thermistor circuits (GPIO1-5) are wired identically.
 * =================================================================== */

/* Error sentinel value — returned when a reading is invalid.
 * -99°C is clearly impossible for any real battery or board
 * temperature, so the caller can check: if (temp == -99) ... */
#define NTC_ERROR_C  (-99)


/* board_get_balance_temp  –  Read the balance board NTC on GPIO5.
 *
 *  Returns temperature in °C, or NTC_ERROR_C (-99) on failure.
 *  Also prints the result to the debug terminal.
 */
int16_t board_get_balance_temp(uint8_t dev_id);


/* board_get_battery_temps  –  Read all four battery NTCs
 *                              (GPIO1 through GPIO4).
 *
 *  Fills temps_out[0..3] with temperatures in °C.
 *  Failed readings get NTC_ERROR_C (-99).
 *  Also prints all four results on one line to the terminal.
 */
void board_get_battery_temps(uint8_t dev_id, int16_t temps_out[4]);


/* ===================================================================
 *  FDCAN1 (Control Bus) — heartbeat
 *
 *  Every board broadcasts a heartbeat frame on FDCAN1 every
 *  1 second.  CAN ID = 0x1B0 + board_address.  The master
 *  board (address 0) also listens for heartbeats from slaves
 *  and prints their addresses on the terminal.
 *
 *  Per BMS_CANBUS_Architecture.md, heartbeat frame (8 bytes):
 *    Byte 0   : Board ID (0–7)
 *    Byte 1   : Board state (0=OK, 1=Warning, 2=Fault)
 *    Byte 2   : Active cell count (6–16)
 *    Byte 3–4 : Board min cell V, mV, big-endian uint16
 *    Byte 5–6 : Board max cell V, mV, big-endian uint16
 *    Byte 7   : Rolling counter (0–255, wraps)
 *
 *  Pins:
 *    FDCAN1_TX = PD1,  FDCAN1_RX = PD0
 *    FDCAN2_TX = PB13, FDCAN2_RX = PB12
 *
 *  Requires CubeMX changes — see comments in board.c.
 * =================================================================== */

/* Initialise FDCAN1 — configure RX filter for heartbeat IDs
 * (0x1B0–0x1B7), then start the peripheral.
 * Call once from application_setup() AFTER CubeMX's
 * MX_FDCAN1_Init() has already run.
 * Returns true if FDCAN started successfully. */
bool board_can_init(void);
bool board_can2_init(void);

/* Build and transmit a heartbeat frame on FDCAN1.
 * Uses this board's address for the CAN ID (0x1B0 + addr).
 * Call every 1000 ms from application_loop().
 *
 * Parameters:
 *   min_cell_mv - lowest cell voltage in mV (0 if unknown)
 *   max_cell_mv - highest cell voltage in mV (0 if unknown)
 *   cell_count  - number of active cells (6–16)
 *   state       - board state: 0=OK, 1=Warning, 2=Fault
 *
 * Returns true if the frame was queued for transmission. */
bool board_can_tx_heartbeat(int32_t  min_cell_mv,
                            int32_t  max_cell_mv,
                            uint8_t  cell_count,
                            uint8_t  state);
bool board_can2_tx_heartbeat(int32_t  min_cell_mv,
                             int32_t  max_cell_mv,
                             uint8_t  cell_count,
                             uint8_t  state);
bool board_can2_tx_has_free_slot(void);

/* Send the four CAN2 cell-voltage info-bus frames:
 *   0x200 + N  -> cells 1..4
 *   0x210 + N  -> cells 5..8
 *   0x220 + N  -> cells 9..12
 *   0x230 + N  -> cells 13..16
 *
 * Each cell is encoded as an unsigned big-endian millivolt value.
 * Unused cells above cell_count are sent as 0. */
bool board_can2_tx_cell_voltage_frame(const int32_t cell_mv[],
                                      uint8_t       cell_count,
                                      uint8_t       frame_index);
bool board_can2_tx_cell_voltages(const int32_t cell_mv[],
                                 uint8_t       cell_count);

/* CAN2 distributed balancing support.
 *
 * Status frame (0x400 + board_id) lets the master know whether the
 * slave is in a clean-read window and which balance phase is active.
 * Command frame (0x410 + board_id) carries the 16-bit per-cell mask
 * plus the clean-read sequence number that command was based on. */
bool board_can2_tx_balance_status(uint16_t active_mask,
                                  uint8_t  balance_state,
                                  uint8_t  clean_cycle,
                                  uint8_t  spread_mv,
                                  uint8_t  cell_count,
                                  uint8_t  flags);

bool board_can2_tx_balance_command(uint8_t  board_id,
                                   uint16_t balance_mask,
                                   uint8_t  clean_cycle);

/* Poll FDCAN1 RX FIFO for a received heartbeat frame.
 * If a frame is waiting, copies the 8-byte payload into
 * rx_data[] and returns the board ID (0–7).
 * If the FIFO is empty, returns -1.
 *
 * Call every loop pass.  Non-blocking — returns instantly
 * if no message is available. */
int8_t board_can_rx_heartbeat(uint8_t rx_data[8]);
int8_t board_can2_rx_heartbeat(uint8_t rx_data[8]);

/* Poll one raw standard-ID frame from CAN2 RX FIFO 0.
 * Returns true when a frame was read and fills std_id/rx_data.
 * Returns false if the FIFO is empty or the frame was invalid. */
bool board_can2_rx_message(uint32_t *std_id,
                           uint8_t   rx_data[8]);


#endif /* INC_BOARD_H_ */

//My board has config jumpers which should be used to set everything up at boot.
//Cell count should be set using CONFIG_CELLS_J1 to J4 in CPM-4-16.ioc attached to this project
//shorted jumper = 1, open jumper = 0

//number of cells   J1  J2  J3  J4
//		6			0	0	0	0
//		7			0	0	0	1
//		8			0	0	1	0
//		9			0	0	1	1
//		10			0	1	0	0
//		11			0	1	0	1
//		12			0	1	1	0
//		13			0	1	1	1
//		14			1	0	0	0
//		15			1	0	0	1
//		16			1	0	1	0

//CONFIG_ION_LFP 1 = lithium ion, 0 = lifepo4

//CONFIG_HICAP_LONGLIFE sets the ov and uv thresholds depending on what CONFIG_ION_LFP is set to
//for lithium ion hicap set ov to 4225 uv to 2850
//for lithium ion longlife set ov to 4175 uv to 3450
//for lifepo4 hicap set ov to 3650 uv to 2500
//for lifepo4 longlife set ov to 3500 uv to 2950
