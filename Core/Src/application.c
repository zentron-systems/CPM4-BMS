//application.c created Tuesday 11/02/2026 at 17:09 updated Wednesday 18/02/2026
#include "application.h"
#include "digital_io.h"
#include "main.h"
#include "bq79616.h"
#include <stdio.h>
#include <string.h>
#include "board.h"       /* Add this at the top with your other includes */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* ---- LED one-shot and PD9 low-hold detection (driven by TIM3 @ 10 ms) ---- */
static volatile uint8_t  g_led_ticks_10ms   = 0;   /* LED remaining time in 10 ms ticks */
static volatile uint8_t  g_pd9_shutdown_req = 0;   /* set by ISR; handled in main context */
static uint16_t          g_pd9_low_ticks    = 0;   /* consecutive LOW samples (10 ms units) */
static uint8_t           g_pd9_armed        = 1;   /* one-shot latch; re-arms on PD9 HIGH */

/* ---- PD8 key-start detection (driven by TIM3 @ 10 ms) ----
 *
 * PD8 = MCU_KEY_START.  Goes HIGH while the ignition key is held
 * in the "start" position.  We require it to be held HIGH for at
 * least 200 ms (20 x 10 ms ticks) before we trigger a startup
 * request.  This debounces the signal and prevents accidental
 * triggers from brief contact bounce.
 *
 * The one-shot latch (g_pd8_armed) prevents repeated triggers
 * while the key is held.  It re-arms when PD8 goes LOW (key
 * released), so the user must release and hold again to retry.
 */
static volatile uint8_t  g_pd8_startup_req  = 0;   /* set by ISR; handled in main context */
static uint16_t          g_pd8_high_ticks   = 0;   /* consecutive HIGH samples (10 ms)    */
static uint8_t           g_pd8_armed        = 1;   /* one-shot latch; re-arms on PD8 LOW  */

/* ---- Contactor state ----
 *
 * Tracks whether the contactors are currently closed (live) or
 * open (shutdown).  This prevents board_contactor_startup() from
 * being called when the contactors are already on, which would
 * be pointless and could confuse the TPS1H200A fault logic.
 *
 * 0 = contactors are OFF (safe to start)
 * 1 = contactors are ON  (ignore further start requests)
 */
static volatile uint8_t  g_contactors_live  = 0;

/* ---- NFAULT latching fault ----
 *
 * BQ79616_NFAULT on PC11.  When this pin triggers, the
 * contactors are opened immediately and g_nfault_shutdown
 * latches to 1.  The board must be power-cycled to clear it.
 *
 * 0 = normal operation
 * 1 = NFAULT tripped — contactors forced open, balancing
 *     stopped, no recovery without power cycle
 */
static volatile uint8_t  g_nfault_shutdown  = 0;

/* ---- Balance thermal protection ----
 *
 * Balancing is stopped if the balance board temperature
 * (NTC on GPIO5) reaches BAL_TEMP_STOP_C (60°C) and only
 * resumes when it cools to BAL_TEMP_RESUME_C (50°C).
 * The 10°C hysteresis prevents rapid on/off cycling.
 *
 * 0 = balancing allowed (temperature is OK)
 * 1 = balancing stopped (too hot, waiting to cool)
 */
#define BAL_TEMP_STOP_C    60   /* Stop balancing at this temp  */
#define BAL_TEMP_RESUME_C  50   /* Resume when cooled to here   */
static uint8_t  g_bal_thermal_stop  = 0;

/* ---- CAN heartbeat tracker ----
 *
 * Updated every loop pass by the RX poll.  The display screen
 * reads these to show which boards are alive on the CAN bus.
 *
 * g_can_last_seen[n]    = HAL_GetTick() when board n's heartbeat
 *                          was last received.  0 = never seen.
 * g_can_last_counter[n] = rolling counter from the last heartbeat.
 * g_can_last_state[n]   = board state (0=OK, 1=Warn, 2=Fault).
 */
static uint32_t g_can_last_seen[8]    = {0};
static uint8_t  g_can_last_counter[8] = {0};
static uint8_t  g_can_last_state[8]   = {0};
static uint8_t  g_can_last_cell_count[8] = {0};
static uint32_t g_can2_last_seen[8]    = {0};
static uint8_t  g_can2_last_counter[8] = {0};
static uint8_t  g_can2_last_state[8]   = {0};
static uint32_t g_can2_cell_last_seen[8]    = {0};
static uint8_t  g_can2_cell_group_mask[8]   = {0};
static int32_t  g_can2_cell_mv[8][BQ_MAX_CELLS] = {{0}};
static uint32_t g_can2_balance_last_seen[8]   = {0};
static uint16_t g_can2_balance_active_mask[8] = {0};
static uint8_t  g_can2_balance_state[8]       = {0};
static uint8_t  g_can2_balance_clean_cycle[8] = {0};
static uint8_t  g_can2_balance_spread_mv[8]   = {0};
static uint8_t  g_can2_balance_cell_count[8]  = {0};
static uint8_t  g_can2_balance_flags[8]       = {0};
static uint8_t  g_master_last_used_clean_cycle[8] = {0};
static uint16_t g_master_pending_balance_mask[8] = {0};
static uint8_t  g_master_pending_balance_cycle[8] = {0};
static uint32_t g_master_pending_balance_last_tx_ms[8] = {0};
static uint8_t  g_master_pending_balance_valid[8] = {0};

static uint8_t  g_slave_balance_state = 0;
static uint32_t g_slave_balance_timer_ms = 0;
static uint32_t g_slave_last_clean_read_ms = 0;
static uint32_t g_slave_last_can2_tx_ms = 0;
static int32_t  g_slave_clean_cells[BQ_MAX_CELLS] = {0};
static int32_t  g_slave_can2_snapshot_cells[BQ_MAX_CELLS] = {0};
static uint8_t  g_slave_clean_cell_count = 0;
static uint8_t  g_slave_can2_snapshot_cell_count = 0;
static uint8_t  g_slave_clean_cycle = 0;
static uint8_t  g_slave_clean_valid = 0;
static uint8_t  g_slave_can2_status_pending = 0;
static uint8_t  g_slave_can2_next_frame = 4U;
static uint16_t g_slave_requested_mask = 0;
static uint16_t g_slave_active_mask = 0;
static int32_t  g_slave_clean_min_mv = 0;
static int32_t  g_slave_clean_max_mv = 0;
static int32_t  g_slave_clean_spread_mv = 0;

static uint8_t  g_master_balance_from_done = 1;
static uint8_t  g_master_balance_done_count = 0;
static uint8_t  g_master_balance_resume_count = 0;
static int32_t  g_master_balance_min_mv = 0;
static int32_t  g_master_balance_max_mv = 0;
static int32_t  g_master_balance_spread_mv = 0;
static int32_t  g_master_balance_target_mv = 0;

#define CAN_TIMEOUT_MS         3000U
#define CAN2_ID_CELLS_BASE     0x200U
#define CAN2_ID_CELLS_LAST     0x237U
#define CAN2_CELL_FRAME_STEP   0x10U
#define CAN2_CELL_FRAME_COUNT  4U
#define CAN2_CELLS_PER_FRAME   4U
#define CAN2_ID_BAL_STATUS_BASE 0x400U
#define CAN2_ID_BAL_CMD_BASE    0x410U
#define CAN2_ID_BAL_LAST        0x417U

#define SLAVE_CAN2_TX_MS       500U
#define SLAVE_BAL_PHASE_MS     500U
#define SLAVE_BAL_SETTLE_MS   1500U
#define MASTER_BAL_CMD_RETRY_MS 100U

#define REMOTE_BAL_READY       0U
#define REMOTE_BAL_ODD         1U
#define REMOTE_BAL_EVEN        2U
#define REMOTE_BAL_SETTLE      3U
#define REMOTE_BAL_BLOCKED     4U

#define REMOTE_BAL_FLAG_THERMAL_STOP  0x01U
#define REMOTE_BAL_FLAG_NFAULT        0x02U
#define REMOTE_BAL_FLAG_CLEAN_VALID   0x04U

static uint16_t application_cell_count_mask(uint8_t cell_count)
{
    if (cell_count >= 16U)
    {
        return 0xFFFFU;
    }

    if (cell_count == 0U)
    {
        return 0x0000U;
    }

    return (uint16_t)((1U << cell_count) - 1U);
}

static uint8_t application_slave_balance_flags(void)
{
    uint8_t flags = 0U;

    if (g_bal_thermal_stop)
    {
        flags |= REMOTE_BAL_FLAG_THERMAL_STOP;
    }
    if (g_nfault_shutdown)
    {
        flags |= REMOTE_BAL_FLAG_NFAULT;
    }
    if (g_slave_clean_valid)
    {
        flags |= REMOTE_BAL_FLAG_CLEAN_VALID;
    }

    return flags;
}

static void application_update_slave_clean_cache(const int32_t cell_mv[],
                                                 uint8_t       cell_count,
                                                 uint32_t      now_ms)
{
    int32_t min_mv;
    int32_t max_mv;
    uint8_t i;

    if (cell_count == 0U)
    {
        return;
    }

    memset(g_slave_clean_cells, 0, sizeof(g_slave_clean_cells));

    min_mv = cell_mv[0];
    max_mv = cell_mv[0];

    for (i = 0; i < cell_count; i++)
    {
        g_slave_clean_cells[i] = cell_mv[i];

        if (cell_mv[i] < min_mv) { min_mv = cell_mv[i]; }
        if (cell_mv[i] > max_mv) { max_mv = cell_mv[i]; }
    }

    g_slave_clean_cell_count = cell_count;
    g_slave_clean_min_mv     = min_mv;
    g_slave_clean_max_mv     = max_mv;
    g_slave_clean_spread_mv  = max_mv - min_mv;
    g_slave_clean_valid      = 1U;
    g_slave_last_clean_read_ms = now_ms;
    g_slave_clean_cycle++;
}

static bool application_slave_take_clean_read(uint8_t  configured_cell_count,
                                              uint32_t now_ms)
{
    int32_t cells_mv[BQ_MAX_CELLS];
    int     read_count;

    memset(cells_mv, 0, sizeof(cells_mv));

    read_count = bq79616_read_cell_voltages(0x01,
                                            cells_mv,
                                            BQ_MAX_CELLS);
    if (read_count < (int)configured_cell_count)
    {
        return false;
    }

    if (read_count > (int)configured_cell_count)
    {
        read_count = (int)configured_cell_count;
    }

    application_update_slave_clean_cache(cells_mv,
                                         (uint8_t)read_count,
                                         now_ms);
    return true;
}

static void application_slave_schedule_can2_snapshot(uint32_t now_ms)
{
    g_slave_last_can2_tx_ms = now_ms;
    g_slave_can2_status_pending = 1U;

    if (g_slave_clean_valid && g_slave_clean_cell_count > 0U)
    {
        memcpy(g_slave_can2_snapshot_cells,
               g_slave_clean_cells,
               sizeof(g_slave_can2_snapshot_cells));
        g_slave_can2_snapshot_cell_count = g_slave_clean_cell_count;
        g_slave_can2_next_frame = 0U;
    }
    else
    {
        memset(g_slave_can2_snapshot_cells,
               0,
               sizeof(g_slave_can2_snapshot_cells));
        g_slave_can2_snapshot_cell_count = 0U;
        g_slave_can2_next_frame = CAN2_CELL_FRAME_COUNT;
    }
}

static void application_slave_flush_can2(void)
{
    if (!board_can2_tx_has_free_slot())
    {
        return;
    }

    if (g_slave_can2_status_pending)
    {
        if (board_can2_tx_balance_status(g_slave_active_mask,
                                         g_slave_balance_state,
                                         g_slave_clean_cycle,
                                         (g_slave_clean_spread_mv > 255)
                                         ? 255U
                                         : (uint8_t)g_slave_clean_spread_mv,
                                         g_slave_clean_cell_count,
                                         application_slave_balance_flags()))
        {
            g_slave_can2_status_pending = 0U;
        }

        return;
    }

    if (g_slave_can2_next_frame >= CAN2_CELL_FRAME_COUNT ||
        g_slave_can2_snapshot_cell_count == 0U)
    {
        return;
    }

    if (board_can2_tx_cell_voltage_frame(g_slave_can2_snapshot_cells,
                                         g_slave_can2_snapshot_cell_count,
                                         g_slave_can2_next_frame))
    {
        g_slave_can2_next_frame++;
    }
}

static void application_master_queue_balance_command(uint8_t  board_id,
                                                     uint16_t balance_mask,
                                                     uint8_t  clean_cycle)
{
    if (board_id > 7U)
    {
        return;
    }

    g_master_pending_balance_mask[board_id]  = balance_mask;
    g_master_pending_balance_cycle[board_id] = clean_cycle;
    g_master_pending_balance_last_tx_ms[board_id] = 0U;
    g_master_pending_balance_valid[board_id] = 1U;
    g_master_last_used_clean_cycle[board_id] = clean_cycle;
}

static void application_master_flush_balance_commands(uint32_t now_ms)
{
    uint8_t expected_slaves;
    uint8_t board_id;

    if (board_get_address() != BOARD_ADDR_MASTER)
    {
        return;
    }

    if (!board_can2_tx_has_free_slot())
    {
        return;
    }

    expected_slaves = board_get_num_expected_slaves();

    for (board_id = 1U; board_id <= expected_slaves; board_id++)
    {
        if (g_master_pending_balance_valid[board_id] != 0U)
        {
            bool accepted =
                (g_can2_balance_clean_cycle[board_id]
                 != g_master_pending_balance_cycle[board_id]) ||
                (g_can2_balance_state[board_id] != REMOTE_BAL_READY) ||
                (g_can2_balance_active_mask[board_id] != 0U);

            if (accepted)
            {
                g_master_pending_balance_valid[board_id] = 0U;
                continue;
            }
        }

        if (!g_master_pending_balance_valid[board_id])
        {
            continue;
        }

        if ((now_ms - g_master_pending_balance_last_tx_ms[board_id])
            < MASTER_BAL_CMD_RETRY_MS)
        {
            continue;
        }

        if (!board_can2_tx_has_free_slot())
        {
            return;
        }

        if (board_can2_tx_balance_command(board_id,
                                          g_master_pending_balance_mask[board_id],
                                          g_master_pending_balance_cycle[board_id]))
        {
            g_master_pending_balance_last_tx_ms[board_id] = now_ms;
        }

        break;
    }
}

static bool application_cache_can2_balance_status(uint32_t      can_id,
                                                  const uint8_t rx_data[8],
                                                  uint32_t      now_ms)
{
    uint8_t board_id;

    if (can_id < CAN2_ID_BAL_STATUS_BASE ||
        can_id >= CAN2_ID_BAL_CMD_BASE)
    {
        return false;
    }

    board_id = (uint8_t)(can_id - CAN2_ID_BAL_STATUS_BASE);
    if (board_id > 7U)
    {
        return false;
    }

    g_can2_balance_last_seen[board_id]   = now_ms;
    g_can2_last_seen[board_id]           = now_ms;

    if (g_can2_balance_clean_cycle[board_id] != rx_data[3])
    {
        g_can2_cell_group_mask[board_id] = 0U;
    }

    g_can2_balance_active_mask[board_id] =
        (uint16_t)(((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1]);
    g_can2_balance_state[board_id]       = rx_data[2];
    g_can2_balance_clean_cycle[board_id] = rx_data[3];
    g_can2_balance_spread_mv[board_id]   = rx_data[4];
    g_can2_balance_cell_count[board_id]  = rx_data[5];
    g_can2_balance_flags[board_id]       = rx_data[6];
    g_can_last_cell_count[board_id]      = rx_data[5];

    return true;
}

static void application_slave_handle_balance_command(uint32_t can_id,
                                                     const uint8_t rx_data[8],
                                                     uint8_t configured_cell_count)
{
    uint8_t  board_id;
    uint16_t mask;

    if (can_id < CAN2_ID_BAL_CMD_BASE || can_id > CAN2_ID_BAL_LAST)
    {
        return;
    }

    board_id = (uint8_t)(can_id - CAN2_ID_BAL_CMD_BASE);
    if (board_id != board_get_address())
    {
        return;
    }

    if (g_slave_balance_state != REMOTE_BAL_READY || !g_slave_clean_valid)
    {
        return;
    }

    if (rx_data[2] != g_slave_clean_cycle)
    {
        return;
    }

    mask = (uint16_t)(((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1]);
    mask &= application_cell_count_mask(configured_cell_count);

    {
        uint8_t i;
        for (i = 0; i < configured_cell_count; i++)
        {
            if (g_slave_clean_cells[i] <= (int32_t)board_get_bal_floor_mv())
            {
                mask &= (uint16_t)~(1U << i);
            }
        }
    }

    g_slave_requested_mask = mask;
}

static void application_slave_service_balance(uint32_t now_ms,
                                              uint8_t  configured_cell_count)
{
    uint16_t odd_mask;
    uint16_t even_mask;
    bool     blocked = (g_bal_thermal_stop != 0U) || (g_nfault_shutdown != 0U);

    if (blocked)
    {
        if (g_slave_active_mask != 0U)
        {
            (void)bq79616_balance_stop(0x01);
            g_slave_active_mask = 0U;
        }

        g_slave_requested_mask = 0U;
        g_slave_balance_state  = REMOTE_BAL_BLOCKED;
    }
    else if (g_slave_balance_state == REMOTE_BAL_BLOCKED)
    {
        g_slave_balance_state = REMOTE_BAL_SETTLE;
        g_slave_balance_timer_ms = now_ms;
    }

    odd_mask  = g_slave_requested_mask & 0x5555U;
    even_mask = g_slave_requested_mask & 0xAAAAU;

    switch (g_slave_balance_state)
    {
        case REMOTE_BAL_READY:
            if (g_slave_requested_mask != 0U)
            {
                if (odd_mask != 0U)
                {
                    if (bq79616_balance_apply_mask(0x01,
                                                   odd_mask,
                                                   configured_cell_count))
                    {
                        g_slave_active_mask = odd_mask;
                        g_slave_balance_state = REMOTE_BAL_ODD;
                        g_slave_balance_timer_ms = now_ms;
                    }
                }
                else if (even_mask != 0U)
                {
                    if (bq79616_balance_apply_mask(0x01,
                                                   even_mask,
                                                   configured_cell_count))
                    {
                        g_slave_active_mask = even_mask;
                        g_slave_balance_state = REMOTE_BAL_EVEN;
                        g_slave_balance_timer_ms = now_ms;
                    }
                }
                else
                {
                    g_slave_requested_mask = 0U;
                }
            }

            if (g_slave_balance_state == REMOTE_BAL_READY &&
                (((now_ms - g_slave_last_clean_read_ms) >= SLAVE_CAN2_TX_MS)
                || !g_slave_clean_valid)
               )
            {
                (void)application_slave_take_clean_read(configured_cell_count,
                                                        now_ms);
            }
            break;

        case REMOTE_BAL_ODD:
            if ((now_ms - g_slave_balance_timer_ms) >= SLAVE_BAL_PHASE_MS)
            {
                (void)bq79616_balance_stop(0x01);
                g_slave_active_mask = 0U;

                if (even_mask != 0U)
                {
                    if (bq79616_balance_apply_mask(0x01,
                                                   even_mask,
                                                   configured_cell_count))
                    {
                        g_slave_active_mask = even_mask;
                        g_slave_balance_state = REMOTE_BAL_EVEN;
                        g_slave_balance_timer_ms = now_ms;
                    }
                    else
                    {
                        g_slave_requested_mask = 0U;
                        g_slave_balance_state = REMOTE_BAL_SETTLE;
                        g_slave_balance_timer_ms = now_ms;
                    }
                }
                else
                {
                    g_slave_requested_mask = 0U;
                    g_slave_balance_state = REMOTE_BAL_SETTLE;
                    g_slave_balance_timer_ms = now_ms;
                }
            }
            break;

        case REMOTE_BAL_EVEN:
            if ((now_ms - g_slave_balance_timer_ms) >= SLAVE_BAL_PHASE_MS)
            {
                (void)bq79616_balance_stop(0x01);
                g_slave_active_mask = 0U;
                g_slave_requested_mask = 0U;
                g_slave_balance_state = REMOTE_BAL_SETTLE;
                g_slave_balance_timer_ms = now_ms;
            }
            break;

        case REMOTE_BAL_SETTLE:
            if ((now_ms - g_slave_balance_timer_ms) >= SLAVE_BAL_SETTLE_MS)
            {
                if (application_slave_take_clean_read(configured_cell_count,
                                                      now_ms))
                {
                    g_slave_balance_state = REMOTE_BAL_READY;
                }
            }
            break;

        case REMOTE_BAL_BLOCKED:
        default:
            break;
    }

    if ((now_ms - g_slave_last_can2_tx_ms) >= SLAVE_CAN2_TX_MS)
    {
        application_slave_schedule_can2_snapshot(now_ms);
    }
}

static void application_master_service_balance(uint32_t now_ms)
{
    uint8_t expected_slaves = board_get_num_expected_slaves();
    uint8_t board_id;
    bool    all_ready = true;
    bool    all_new = true;
    bool    have_any = false;
    int32_t min_mv = 0;
    int32_t max_mv = 0;

    if (board_get_address() != BOARD_ADDR_MASTER || expected_slaves == 0U)
    {
        return;
    }

    for (board_id = 1U; board_id <= expected_slaves; board_id++)
    {
        uint8_t  cell_count = g_can2_balance_cell_count[board_id];
        uint8_t  cell_index;
        uint8_t  needed_groups;
        uint8_t  expected_group_mask;

        if (cell_count < 1U || cell_count > BQ_MAX_CELLS)
        {
            cell_count = g_can_last_cell_count[board_id];
        }

        if (cell_count < 1U || cell_count > BQ_MAX_CELLS)
        {
            all_ready = false;
            break;
        }

        if (g_can_last_seen[board_id] == 0U ||
            (now_ms - g_can_last_seen[board_id]) >= CAN_TIMEOUT_MS ||
            g_can2_balance_last_seen[board_id] == 0U ||
            (now_ms - g_can2_balance_last_seen[board_id]) >= CAN_TIMEOUT_MS ||
            g_can2_cell_last_seen[board_id] == 0U ||
            (now_ms - g_can2_cell_last_seen[board_id]) >= CAN_TIMEOUT_MS)
        {
            all_ready = false;
            break;
        }

        needed_groups = (uint8_t)((cell_count + 3U) / 4U);
        expected_group_mask =
            (uint8_t)application_cell_count_mask(needed_groups);
        if ((g_can2_cell_group_mask[board_id] & expected_group_mask)
            != expected_group_mask)
        {
            all_ready = false;
            break;
        }

        if (g_can2_balance_state[board_id] != REMOTE_BAL_READY ||
            (g_can2_balance_flags[board_id] & REMOTE_BAL_FLAG_CLEAN_VALID) == 0U)
        {
            all_ready = false;
            break;
        }

        if (g_can2_balance_clean_cycle[board_id]
            == g_master_last_used_clean_cycle[board_id])
        {
            all_new = false;
        }

        for (cell_index = 0U; cell_index < cell_count; cell_index++)
        {
            int32_t cell_mv = g_can2_cell_mv[board_id][cell_index];

            if (cell_mv <= 0)
            {
                all_ready = false;
                break;
            }

            if (!have_any)
            {
                min_mv = cell_mv;
                max_mv = cell_mv;
                have_any = true;
            }
            else
            {
                if (cell_mv < min_mv) { min_mv = cell_mv; }
                if (cell_mv > max_mv) { max_mv = cell_mv; }
            }
        }

        if (!all_ready)
        {
            break;
        }
    }

    if (!all_ready || !all_new || !have_any)
    {
        return;
    }

    g_master_balance_min_mv = min_mv;
    g_master_balance_max_mv = max_mv;
    g_master_balance_spread_mv = max_mv - min_mv;

    if (g_master_balance_spread_mv <=
        (g_master_balance_from_done ? (int32_t)BQ_BALANCE_RESUME_MV
                                    : (int32_t)BQ_BALANCE_DELTA_MV))
    {
        g_master_balance_resume_count = 0U;

        if (g_master_balance_from_done)
        {
            for (board_id = 1U; board_id <= expected_slaves; board_id++)
            {
                g_master_last_used_clean_cycle[board_id] =
                    g_can2_balance_clean_cycle[board_id];
            }
            return;
        }

        g_master_balance_done_count++;
        if (g_master_balance_done_count >= BQ_BALANCE_DEBOUNCE_READS)
        {
            g_master_balance_done_count = 0U;
            g_master_balance_from_done = 1U;

            for (board_id = 1U; board_id <= expected_slaves; board_id++)
            {
                g_master_last_used_clean_cycle[board_id] =
                    g_can2_balance_clean_cycle[board_id];
            }
            return;
        }
    }
    else
    {
        g_master_balance_done_count = 0U;

        if (g_master_balance_from_done)
        {
            g_master_balance_resume_count++;
            if (g_master_balance_resume_count
                < BQ_BALANCE_DEBOUNCE_READS)
            {
                for (board_id = 1U; board_id <= expected_slaves; board_id++)
                {
                    g_master_last_used_clean_cycle[board_id] =
                        g_can2_balance_clean_cycle[board_id];
                }
                return;
            }

            g_master_balance_resume_count = 0U;
        }
    }

    g_master_balance_from_done = 0U;
    g_master_balance_target_mv = min_mv + (int32_t)BQ_BALANCE_DELTA_MV;

    for (board_id = 1U; board_id <= expected_slaves; board_id++)
    {
        uint8_t  cell_count = g_can2_balance_cell_count[board_id];
        uint8_t  cell_index;
        uint16_t mask = 0U;

        if (cell_count < 1U || cell_count > BQ_MAX_CELLS)
        {
            cell_count = g_can_last_cell_count[board_id];
        }

        for (cell_index = 0U; cell_index < cell_count; cell_index++)
        {
            int32_t cell_mv = g_can2_cell_mv[board_id][cell_index];

            if (cell_mv > g_master_balance_target_mv &&
                cell_mv > (int32_t)board_get_bal_floor_mv())
            {
                mask |= (uint16_t)(1U << cell_index);
            }
        }

        if (mask != 0U)
        {
            application_master_queue_balance_command(board_id,
                                                     mask,
                                                     g_can2_balance_clean_cycle[board_id]);
        }
        else
        {
            g_master_last_used_clean_cycle[board_id] =
                g_can2_balance_clean_cycle[board_id];
        }
    }
}

static bool application_cache_can2_cell_frame(uint32_t       can_id,
                                              const uint8_t  rx_data[8],
                                              uint32_t       now_ms)
{
    uint8_t board_id;
    uint8_t frame_index;
    uint8_t slot;

    if (can_id < CAN2_ID_CELLS_BASE || can_id > CAN2_ID_CELLS_LAST)
    {
        return false;
    }

    board_id = (uint8_t)(can_id & 0x0FU);
    frame_index = (uint8_t)((can_id - CAN2_ID_CELLS_BASE)
                            / CAN2_CELL_FRAME_STEP);

    if (board_id > 7U || frame_index >= CAN2_CELL_FRAME_COUNT)
    {
        return false;
    }

    g_can2_cell_last_seen[board_id] = now_ms;
    g_can2_last_seen[board_id]      = now_ms;
    g_can2_cell_group_mask[board_id] |= (uint8_t)(1U << frame_index);

    for (slot = 0; slot < CAN2_CELLS_PER_FRAME; slot++)
    {
        uint8_t cell_index = (uint8_t)(frame_index * CAN2_CELLS_PER_FRAME
                                       + slot);
        g_can2_cell_mv[board_id][cell_index] =
            (int32_t)(((uint16_t)rx_data[slot * 2U] << 8)
                      | (uint16_t)rx_data[slot * 2U + 1U]);
    }

    return true;
}

static int application_append_remote_cell_line(char     *scr,
                                               int       pos,
                                               uint8_t   board_id,
                                               uint32_t  now_ms)
{
    uint8_t  expected_cells = g_can_last_cell_count[board_id];
    uint8_t  cell_index;
    uint8_t  frame_mask;
    bool     fresh;

    if (expected_cells < 1U || expected_cells > BQ_MAX_CELLS)
    {
        expected_cells = BQ_MAX_CELLS;
    }

    if (pos >= (int)UART1_TX_SIZE)
    {
        return (int)UART1_TX_SIZE - 1;
    }

    fresh = (g_can2_cell_last_seen[board_id] != 0U)
            && ((now_ms - g_can2_cell_last_seen[board_id]) < CAN_TIMEOUT_MS);

    if (!fresh)
    {
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "S%u(%u): waiting for CAN2 cell frames\033[K\r\n",
                        board_id, expected_cells);
        return (pos < (int)UART1_TX_SIZE) ? pos
                                          : ((int)UART1_TX_SIZE - 1);
    }

    frame_mask = g_can2_cell_group_mask[board_id];
    pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                    "S%u(%u):", board_id, expected_cells);

    for (cell_index = 0; cell_index < expected_cells; cell_index++)
    {
        bool known = (frame_mask & (uint8_t)(1U << (cell_index / 4U))) != 0U;

        if (pos >= (int)UART1_TX_SIZE)
        {
            return (int)UART1_TX_SIZE - 1;
        }

        if (known && g_can2_cell_mv[board_id][cell_index] > 0)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            " %4ld",
                            (long)g_can2_cell_mv[board_id][cell_index]);
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            " ----");
        }
    }

    pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                    "\033[K\r\n");
    return (pos < (int)UART1_TX_SIZE) ? pos
                                      : ((int)UART1_TX_SIZE - 1);
}

static bool application_has_local_bq79616(void)
{
    return (board_get_address() != BOARD_ADDR_MASTER);
}


void application_setup() // Called once at startup
{
    sys_debug = false;
    digital_write(PA8, HIGH); // keeps the power on after MCU boots

    /* Board identity must always come from the address/slave jumpers,
     * even if the BQ init later fails and even if CONFIG_NO_APP is low. */
    board_refresh_identity_from_jumpers();

    if (application_has_local_bq79616())
    {
        // Do init first so the timer ISR doesn't fight our status indication.
        bool ok = bq79616_init(0x01);

        if (ok)
        {

        // Success: show a solid ON for 1 second, then proceed.
        digital_write(PC13, HIGH);
        HAL_Delay(1000);
        digital_write(PC13, LOW);
        bq79616_get_status(0x01);
        bq79616_tsref_enable(0x01);

        /* Configure GPIO1-4 as ADC inputs for the four battery
         * temperature NTC sensors.  Each circuit is identical:
         *   TSREF(5V) → 10kΩ → junction → NTC(10k@25°C) → GNDHV
         *   GPIO reads the junction voltage through an RC filter.
         *
         * Must be set BEFORE bq79616_adc_start() so the Main ADC
         * includes all GPIO channels in its round-robin cycle.
         */
        bq79616_pin_mode(0x01, 1, BQ_ADC);    /* Battery temp 1 */
        bq79616_pin_mode(0x01, 2, BQ_ADC);    /* Battery temp 2 */
        bq79616_pin_mode(0x01, 3, BQ_ADC);    /* Battery temp 3 */
        bq79616_pin_mode(0x01, 4, BQ_ADC);    /* Battery temp 4 */

        /* GPIO5: Balance board temperature NTC (same circuit) */
        bq79616_pin_mode(0x01, 5, BQ_ADC);

        /* Configure GPIO6 as ADC input for insulation resistance
         * measurement.  The circuit uses R268/R269 (180MΩ each)
         * to bias chassis GND to V_pack/2, then R270/R272 divide
         * that voltage down to the BQ's 0-5V ADC range.
         *
         * Must be set BEFORE bq79616_adc_start() so the Main ADC
         * includes GPIO6 in its continuous round-robin cycle.
         */
        bq79616_pin_mode(0x01, 6, BQ_ADC);

        /* ---- Board configuration ----
         *
         * CONFIG_NO_APP on PC10 selects the configuration source:
         *   fitted -> GPIO HIGH -> standalone / no app
         *   open   -> GPIO LOW  -> app-managed defaults
         *
         *   HIGH = Standalone mode.  All settings come from the
         *          hardware config jumpers on the PCB.  The board
         *          can be deployed in the field without a PC or app.
         *          Reads: address, slave count, cell count, chemistry,
         *          capacity mode.  Derives: OV, UV, MCU warnings,
         *          balance floor.
         *
         *   LOW  = App-managed mode.  Uses hardcoded safe defaults
         *          (16S Li-Ion Long-Life).  A PC or app will send
         *          the real configuration later via CAN or USB.
         *
         * Both paths set: active cells, OV/UV thresholds, MCU warning
         * thresholds, and balance floor.  Board identity for CAN
         * (address / expected slaves) always comes from the jumpers.
         */
         if (digital_read(PC10) == HIGH)
        {
            /* Standalone — read jumpers for all settings */
            if (!board_setup_from_config_jumpers(0x01))
            {
                /* Config failed — flash error and stop.
                 * The board will stay powered but won't start
                 * the ADC or balance algorithm. */
                for (int i = 0; i < 10; ++i)
                {
                    digital_write(PC13, HIGH);
                    HAL_Delay(100);
                    digital_write(PC13, LOW);
                    HAL_Delay(100);
                }
                return;
            }
        }
        else
        {
            /* App-managed — safe defaults for now */
            if (!board_set_defaults(0x01))
            {
                for (int i = 0; i < 10; ++i)
                {
                    digital_write(PC13, HIGH);
                    HAL_Delay(100);
                    digital_write(PC13, LOW);
                    HAL_Delay(100);
                }
                return;
            }
        }

        /* ADC offset calibration.
         *
         * The BQ79616's Main ADC reads ~5 mV high on all
         * channels compared to a calibrated multimeter.
         * This writes MAIN_ADC_CAL2[OFFSET] = -5 mV so the
         * chip subtracts 5 mV from every conversion result.
         *
         * IMPORTANT: Must be called BEFORE bq79616_adc_start()
         * because the chip samples calibration registers when
         * the ADC begins — changes mid-run are ignored.
         *
         * To re-calibrate: rest the pack, compare all 16
         * cells against a meter, compute average(meter - BQ),
         * and update the value here.
         */
        bq79616_set_adc_offset(0x01, -5);

        bq79616_adc_start(0x01);

        /* The old local balancer performed a one-time MB_INIT pass
         * before it ever switched CB timers.  The new master/slave
         * flow still needs that same BQ setup even though the mask
         * now arrives over CAN2. */
        (void)bq79616_balance_prepare_manual(0x01);

        }
        else
        {
            // Failure: flash 5 times quickly, then proceed (or stay here if fatal).
            for (int i = 0; i < 5; ++i)
            {
                digital_write(PC13, HIGH);
                HAL_Delay(150);
                digital_write(PC13, LOW);
                HAL_Delay(150);
            }
            // If you want to stop the app on failure, you can return or loop here.
            // return;
        }
    }
    else
    {
        /* Master boards have no local BQ79616 fitted, so keep all
         * BQ-derived warning outputs inactive and leave USART6 idle. */
        if (digital_read(PC10) == HIGH)
        {
            board_load_runtime_config_from_jumpers();
        }
        else
        {
            board_load_runtime_defaults();
        }

        digital_write(PA5, LOW);
        digital_write(PA4, LOW);
        digital_write(PA6, LOW);
    }

    /* ---- Initialise FDCAN1 (Control Bus) ----
     *
     * Start CAN OUTSIDE the BQ79616 success/fail branch so the
     * board can heartbeat even if its BQ is dead — the master
     * will see it as alive with state=Fault.
     *
     * MX_FDCAN1_Init() has already been called by CubeMX in
     * main.c.  board_can_init() configures the RX filter for
     * heartbeat IDs (0x1B0–0x1B7) and starts the peripheral.
     */
    (void)board_can_init();
    (void)board_can2_init();

    // Start your periodic timers only after the indication is done
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim2);
}


void application_loop()  /* Called repeatedly from main() */
{
    bool board_has_bq = application_has_local_bq79616();
    uint8_t configured_cell_count = board_get_active_cell_count();

    /* ================================================================
     *  ANSI terminal display engine
     *
     *  Two modes of operation:
     *
     *  debug = false (normal):
     *    - Refreshes every 500 ms
     *    - Paints a fixed status screen that updates IN PLACE
     *      (no scrolling — cursor returns to top each cycle)
     *    - All BQ79616 reads done silently (no TX/RX noise)
     *
     *  debug = true (development):
     *    - Refreshes every 5 seconds
     *    - Same status screen at the top
     *    - Full fault decode with TX/RX frames below a separator
     *
     *  The balance algorithm continues running on every loop pass
     *  regardless of display timing.  It maintains its own clean
     *  voltage reads internally (taken with all balancers OFF).
     *
     *  The display reads LIVE voltages (may show balance sag) to
     *  let you see real-time current draw on individual cells.
     *
     *  ANSI escape codes used:
     *    \033[H    = cursor to row 1, column 1 (home)
     *    \033[K    = erase from cursor to end of line
     *    \033[J    = erase from cursor to end of screen
     * ================================================================ */

    /* ---- NFAULT emergency shutdown (every loop pass) ----
     *
     * PC11 = BQ79616_NFAULT.  This is the BQ79616's hardware
     * fault output.  If it goes HIGH while the contactors are
     * engaged, we must immediately:
     *   1. Stop balancing (prevent further cell discharge)
     *   2. Open all contactors (isolate the HV bus)
     *   3. Latch the fault — only a power cycle clears it
     *
     * This check runs BEFORE anything else in the loop because
     * a hardware fault takes priority over all other operations.
     *
     * The latching flag (g_nfault_shutdown) prevents the balance
     * state machine and contactor startup from running.  The
     * display engine shows "NFAULT: TRIPPED" on screen so you
     * know the board needs a power cycle.
     */
    if (board_has_bq && !g_nfault_shutdown)
    {
        if (digital_read(PC11) == HIGH)
        {
            g_nfault_shutdown = 1;

            /* Stop balancing immediately */
            bq79616_balance_stop(0x01);

            /* Only run the contactor shutdown sequence if
             * they are actually engaged.  If they're already
             * off, there's nothing to open. */
            if (g_contactors_live)
            {
                board_contactor_shutdown();
                g_contactors_live = 0;
            }
        }
    }

    /* ---- Handle shutdown request from PD9 (key-off held 500 ms) ---- */
    if (g_pd9_shutdown_req)
    {
        g_pd9_shutdown_req = 0;            /* consume request */

        /* Wait for any in-flight DMA screen update to finish
         * before we cut power — otherwise the last frame gets
         * truncated mid-byte on the terminal. */
        uart1_dma_wait();

        board_contactor_shutdown();         /* open all contactors safely */
        g_contactors_live = 0;             /* mark contactors as OFF */
        digital_write(PA8, LOW);           /* release power hold */
    }

    /* ---- Handle startup request from PD8 (key-start held 200 ms) ---- */
    if (g_pd8_startup_req)
    {
        g_pd8_startup_req = 0;             /* consume request */

        /* Block startup if NFAULT has latched — power cycle required */
        if ((g_contactors_live == 0) && (!g_nfault_shutdown))
        {
            if (board_contactor_startup())
            {
                g_contactors_live = 1;     /* mark contactors as ON */
            }
        }
    }

    /* ---- Get current time for all periodic checks ---- */
    uint32_t now_ms = HAL_GetTick();

    /* ---- Distributed balance execution (slaves) ----
     *
     * Slave boards no longer make balancing decisions locally.
     * They wait for a per-cell mask from the master, execute the
     * odd half for 500 ms, then the even half for 500 ms, then
     * settle long enough for the next clean read.
     *
     * The master uses the same floor/delta/hysteresis values the
     * old local balancer used; the slave just executes the mask. */
    if (board_has_bq)
    {
        application_slave_service_balance(now_ms, configured_cell_count);
    }

    /* Voltage warnings are now checked inside the display refresh
     * (Phase 1) using fresh live voltages, not here.  The old code
     * used bq79616_balance_get_voltages() which only updates every
     * ~3.6 seconds — too slow.  The BQ's hardware OVUV comparator
     * would trip at 4175 mV before the MCU warning at 4150 mV ever
     * saw the new voltage.  Checking against live reads every 500 ms
     * gives the 25 mV gap enough time to actually work. */


    /* ---- FDCAN1 heartbeat TX (every 1000 ms) ----
     *
     * All boards (master and slaves) broadcast their heartbeat
     * on the control bus.  CAN ID = 0x1B0 + board_address.
     *
     * Uses the cached min/max from the balance state machine.
     * These update every ~3.6s which is fine for a 1s heartbeat.
     * Cell count comes from the active jumper/default config.
     * State is always 0 (OK) for this test. */
    {
        static uint32_t last_hb_ms = 0;

        if ((now_ms - last_hb_ms) >= 1000U)
        {
            last_hb_ms = now_ms;

            /* Get cached min/max from the balance state machine.
             * Master boards have no local BQ, so publish 0/unknown. */
            int32_t hb_min = 0;
            int32_t hb_max = 0;

            if (board_has_bq && g_slave_clean_valid)
            {
                hb_min = g_slave_clean_min_mv;
                hb_max = g_slave_clean_max_mv;
            }

            board_can_tx_heartbeat(hb_min, hb_max,
                                   configured_cell_count,
                                   0);     /* state = OK */
            board_can2_tx_heartbeat(hb_min, hb_max,
                                    configured_cell_count,
                                    0);     /* state = OK */
        }
    }

    if (board_has_bq)
    {
        application_slave_flush_can2();
    }

    /* ---- FDCAN1 heartbeat RX (every loop pass) ----
     *
     * Non-blocking poll — returns instantly if the FIFO is empty.
     * Drains all pending heartbeats and updates a static tracker
     * so the display screen can show which boards are alive.
     *
     * The tracker is an array of 8 entries (one per possible
     * board address).  Each entry stores the last-seen tick
     * and rolling counter.  The display uses this to show
     * which boards are responding. */
    {
        uint8_t rx_data[8];
        int8_t  sender;

        while ((sender = board_can_rx_heartbeat(rx_data)) >= 0)
        {
            /* Clamp to valid board range (0–7) */
            if (sender >= 0 && sender <= 7)
            {
                g_can_last_seen[sender]    = now_ms;
                g_can_last_counter[sender] = rx_data[7];
                g_can_last_state[sender]   = rx_data[1];
                g_can_last_cell_count[sender] = rx_data[2];
            }
        }
    }

    /* ---- FDCAN2 info-bus RX (every loop pass) ---- */
    {
        uint32_t can2_id;
        uint8_t rx_data[8];

        while (board_can2_rx_message(&can2_id, rx_data))
        {
            if (can2_id >= 0x1B0U && can2_id <= 0x1B7U)
            {
                uint8_t sender = (uint8_t)(can2_id - 0x1B0U);

                g_can2_last_seen[sender]    = now_ms;
                g_can2_last_counter[sender] = rx_data[7];
                g_can2_last_state[sender]   = rx_data[1];
                g_can_last_cell_count[sender] = rx_data[2];
            }
            else
            {
                if (!application_cache_can2_cell_frame(can2_id,
                                                       rx_data,
                                                       now_ms))
                {
                    if (!application_cache_can2_balance_status(can2_id,
                                                               rx_data,
                                                               now_ms))
                    {
                        application_slave_handle_balance_command(can2_id,
                                                                 rx_data,
                                                                 configured_cell_count);
                    }
                }
            }
        }
    }

    application_master_service_balance(now_ms);
    application_master_flush_balance_commands(now_ms);

    /* ---- Periodic display refresh ----
     *
     * Interval depends on debug mode:
     *   debug=false: 500 ms  (fast, silent, in-place)
     *   debug=true:  5000 ms (slower, with TX/RX frames below)
     */
    {
        static uint32_t last_display_ms = 0;

        uint32_t refresh_ms = sys_debug ? 5000U : 500U;

        if ((now_ms - last_display_ms) < refresh_ms)
        {
            return;   /* Not time yet — exit early */
        }

        last_display_ms = now_ms;
    }

    /* ===============================================================
     *  If we get here, it's time to refresh the display.
     *
     *  Phase 1: Collect all sensor data SILENTLY.
     *           Temporarily suppress debug so register reads
     *           don't produce TX/RX noise that would break
     *           the clean status screen.
     *
     *  Phase 2: Paint the status screen from the collected data.
     *           Uses ANSI cursor-home to redraw in place.
     *
     *  Phase 3: (debug=true only) Print the full fault decode
     *           with TX/RX frames below a separator line.
     * =============================================================== */

    /* ---- Phase 1: Collect data silently ---- */
    bool saved_debug = sys_debug;
    sys_debug = false;      /* Suppress all TX/RX prints */

    /* Cell voltages — try a live read for fresh values.
     *
     * Most of the time this succeeds and gives voltages that
     * are only 500 ms old.  If it fails (USART6 timeout due
     * to collision with the balance state machine's own reads
     * or BAL_GO writes), fall back to the cached clean values
     * from the last MB_READ cycle.
     *
     * This way the display always has data — never shows
     * "waiting for cell data" — and updates as fast as
     * possible without blocking the balance algorithm.
     */
    int32_t  disp_cells[BQ_MAX_CELLS];
    int      cell_count = 0;
    int16_t  batt_temps[4];
    int16_t  bal_temp_c = NTC_ERROR_C;

    memset(disp_cells, 0, sizeof(disp_cells));
    memset(batt_temps, 0, sizeof(batt_temps));

    if (board_has_bq)
    {
        cell_count = bq79616_read_cell_voltages(0x01, disp_cells,
                                                 BQ_MAX_CELLS);

    if (cell_count < (int)configured_cell_count && g_slave_clean_valid)
    {
        /* Live read failed — use the last clean no-balance snapshot. */
        memcpy(disp_cells, g_slave_clean_cells, sizeof(g_slave_clean_cells));
        cell_count = g_slave_clean_cell_count;
    }

    /* ---- Voltage warning check (every display refresh) ----
     *
     * Uses the freshest available cell voltages (live read if
     * it succeeded, cached balance data as fallback).  Runs
     * every 500 ms (debug=false) or 5 s (debug=true), which
     * is fast enough for the 25 mV gap between the MCU warning
     * threshold (4150 mV) and the BQ hardware fault (4175 mV)
     * to actually fire first during charging.
     *
     * Drives MCU GPIO pins:
     *   PA5 HIGH if any cell >= 4150 mV  (overvoltage warning)
     *   PA4 HIGH if any cell >= 4085 mV  (balance warning)
     *   PA6 HIGH if any cell <= 3150 mV  (undervoltage warning)
     */
    if (cell_count > 0)
    {
        board_check_voltage_warnings(disp_cells,
                                     (uint8_t)cell_count);
    }

    /* Fault status — updates the one-line summary buffer */
    bq79616_get_status(0x01);

    /* Temperatures — update their static result buffers */
    bal_temp_c = board_get_balance_temp(0x01);
    board_get_battery_temps(0x01, batt_temps);

    /* Insulation resistance — updates its static result buffer */
    if (cell_count > 0)
    {
        board_measure_insulation(0x01, disp_cells,
                                 (uint8_t)cell_count);
    }
    }
    else
    {
        digital_write(PA5, LOW);
        digital_write(PA4, LOW);
        digital_write(PA6, LOW);
    }

    sys_debug = saved_debug;   /* Restore debug flag */

    /* ---- Balance thermal protection (hysteresis) ----
     *
     * Check the balance board temperature AFTER it has been
     * read (Phase 1 above) and BEFORE painting the display.
     *
     * State transitions:
     *   g_bal_thermal_stop = 0 (running):
     *     if bal_temp >= 60°C → stop balancing, set flag = 1
     *
     *   g_bal_thermal_stop = 1 (stopped):
     *     if bal_temp <= 50°C → clear flag = 0, balancing
     *     resumes on the next loop pass (gated at line ~300)
     *
     * The 10°C hysteresis band prevents rapid on/off cycling
     * that would occur if both thresholds were the same value.
     *
     * If the temperature read failed (NTC_ERROR_C = -99), we
     * leave the current state unchanged — don't stop on a bad
     * reading, and don't resume on a bad reading either.
     */
    if (board_has_bq && (bal_temp_c != NTC_ERROR_C))
    {
        if (!g_bal_thermal_stop)
        {
            /* Currently running — check if too hot */
            if (bal_temp_c >= BAL_TEMP_STOP_C)
            {
                g_bal_thermal_stop = 1;

                /* Stop balancing immediately — zero all CB timers
                 * and re-issue BAL_GO so the chip sees the zeros */
                bq79616_balance_stop(0x01);
            }
        }
        else
        {
            /* Currently stopped — check if cool enough */
            if (bal_temp_c <= BAL_TEMP_RESUME_C)
            {
                g_bal_thermal_stop = 0;
                /* Balancing resumes on the next loop pass via
                 * the remote balance state machine above. */
            }
        }
    }

    /* ---- Phase 2: Paint the status screen ----
     *
     * Build the entire screen in one buffer and send it in
     * a single UART transmit.  This minimises flicker because
     * PuTTY receives and renders the whole frame at once.
     *
     * ANSI codes:
     *   \033[H  = cursor home (top-left corner)
     *   \033[K  = clear from cursor to end of this line
     *   \033[J  = clear from cursor to end of screen
     *
     * Every line ends with \033[K to erase leftover characters
     * from the previous paint (e.g. if a value got shorter).
     * The last line ends with \033[J to wipe anything below.
     */
    {
        uint8_t addr = board_get_address();
        uint8_t addr_j1 = (addr >> 2) & 0x1U;
        uint8_t addr_j2 = (addr >> 1) & 0x1U;
        uint8_t addr_j3 = addr & 0x1U;
        uint8_t no_app = (digital_read(PC10) == HIGH) ? 1U : 0U;

        /* Refresh the physical identity each paint so the display shows
         * the actual jumper state even if the BQ path never completed. */
        board_refresh_identity_from_jumpers();
        addr = board_get_address();
        addr_j1 = (addr >> 2) & 0x1U;
        addr_j2 = (addr >> 1) & 0x1U;
        addr_j3 = addr & 0x1U;

        /* Get the DMA buffer — waits for any in-progress
         * transfer to complete before returning the pointer.
         * We build the screen directly inside this buffer
         * with snprintf calls, then fire DMA with no copy. */
        char *scr = (char *)uart1_dma_buf();
        int  pos = 0;

        /* Cursor home — redraws on top of previous frame */
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "\033[H");

        /* Line 1: Fault status */
        if (board_has_bq)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "%s\033[K\r\n",
                            bq79616_get_fault_line());
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "MASTER: showing slave cell voltages from CAN2\033[K\r\n");
        }

        /* Line 2: Column header */
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "Cell voltages (mV):\033[K\r\n");

        /* Lines 3-18: Cell voltages (one per line)
         *
         * If a cell is currently being balanced, a "B" marker
         * appears after its voltage so you can see at a glance
         * which MOSFETs are active.  The mask comes from the
         * balance state machine — bit 0 = C1, bit 15 = C16. */
        if (board_has_bq && cell_count >= (int)configured_cell_count)
        {
            uint16_t bal_mask = g_slave_active_mask;
            int i;

            for (i = 0; i < (int)configured_cell_count; i++)
            {
                /* Check if this cell's bit is set in the mask */
                const char *mark = (bal_mask & (1U << i))
                                   ? " B" : "";

                pos += snprintf(&scr[pos],
                                UART1_TX_SIZE - (size_t)pos,
                                "C%02d: %ld%s\033[K\r\n",
                                i + 1, (long)disp_cells[i],
                                mark);
            }
        }
        else if (addr == BOARD_ADDR_MASTER)
        {
            uint8_t remote_board;

            for (remote_board = 1; remote_board <= 7U; remote_board++)
            {
                pos = application_append_remote_cell_line(scr,
                                                          pos,
                                                          remote_board,
                                                          now_ms);
            }
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "(local BQ79616 not present on this board)\033[K\r\n");
        }

        /* Blank separator line */
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "\033[K\r\n");

        /* Balance status + live min/max/spread.
         *
         * The BAL/BAL DONE label reflects whether any FETs
         * are currently active (from the balance mask).
         * Min/max/spread are computed here from the fresh
         * disp_cells[] that update every 500 ms, so the
         * numbers always match the voltages shown above. */
        if (board_has_bq && cell_count >= (int)configured_cell_count)
        {
            int32_t d_min = disp_cells[0];
            int32_t d_max = disp_cells[0];
            int di;
            const char *bal_label = "BAL READY";

            for (di = 1; di < (int)configured_cell_count; di++)
            {
                if (disp_cells[di] < d_min) { d_min = disp_cells[di]; }
                if (disp_cells[di] > d_max) { d_max = disp_cells[di]; }
            }

            if (g_slave_balance_state == REMOTE_BAL_ODD)
            {
                bal_label = "BAL ODD";
            }
            else if (g_slave_balance_state == REMOTE_BAL_EVEN)
            {
                bal_label = "BAL EVEN";
            }
            else if (g_slave_balance_state == REMOTE_BAL_SETTLE)
            {
                bal_label = "BAL SETTLE";
            }
            else if (g_slave_balance_state == REMOTE_BAL_BLOCKED)
            {
                bal_label = "BAL BLOCKED";
            }

            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "%s  spread=%ld  min=%ld  "
                            "max=%ld\033[K\r\n",
                            bal_label,
                            (long)(d_max - d_min),
                            (long)d_min, (long)d_max);
        }
        else if (addr == BOARD_ADDR_MASTER)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "PACK BAL: spread=%ld min=%ld max=%ld target=%ld\033[K\r\n",
                            (long)g_master_balance_spread_mv,
                            (long)g_master_balance_min_mv,
                            (long)g_master_balance_max_mv,
                            (long)g_master_balance_target_mv);
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "BAL: N/A (master has no local BQ79616)\033[K\r\n");
        }

        /* PD8/contactor status line */
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "PD8 pin=%u ticks=%u armed=%u "
                        "req=%u live=%u\033[K\r\n",
                        (unsigned)(digital_read(PD8) == HIGH),
                        g_pd8_high_ticks,
                        g_pd8_armed,
                        g_pd8_startup_req,
                        g_contactors_live);

        /* Battery temperature line */
        if (board_has_bq)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "%s\033[K\r\n",
                            board_get_batt_temp_line());
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "BATT TEMP: N/A (master has no local BQ79616)\033[K\r\n");
        }

        /* Balance board temperature line */
        if (board_has_bq)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "%s\033[K\r\n",
                            board_get_bal_temp_line());
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "BAL TEMP: N/A (master has no local BQ79616)\033[K\r\n");
        }

        /* Insulation resistance line */
        if (board_has_bq)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "%s\033[K\r\n",
                            board_get_iso_line());
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "ISO: N/A (master has no local BQ79616)\033[K\r\n");
        }

        /* NFAULT status line — shows PC11 pin state and
         * whether the latching fault has been tripped.
         * If tripped, the board needs a power cycle. */
        if (!board_has_bq)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "NFAULT: N/A (master has no local BQ79616)\033[K\r\n");
        }
        else if (g_nfault_shutdown)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "NFAULT: TRIPPED — power cycle "
                            "required\033[K\r\n");
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "NFAULT: OK  (PC11=%u)\033[K\r\n",
                            (unsigned)(digital_read(PC11) == HIGH));
        }

        /* Balance thermal status — shows whether balancing is
         * stopped due to overtemperature on the balance board */
        if (!board_has_bq)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "BAL THERMAL: N/A (master has no local BQ79616)\033[K\r\n");
        }
        else if (g_bal_thermal_stop)
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "BAL THERMAL: STOPPED (>=%dC, "
                            "resume at <=%dC)\033[K\r\n",
                            BAL_TEMP_STOP_C,
                            BAL_TEMP_RESUME_C);
        }
        else
        {
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "BAL THERMAL: OK\033[K\r\n");
        }

        /* Board identity line — address and slave info */
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "BOARD: addr=%u (%s)  slaves=%u\033[K\r\n",
                        addr,
                        (addr == 0) ? "MASTER" : "SLAVE",
                        board_get_num_expected_slaves());

        /* Raw jumper snapshot so address problems are visible inside
         * the in-place terminal redraw, not only in one-shot debug logs. */
        pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                        "JUMPERS: A[J1=%u J2=%u J3=%u] NO_APP=%u\033[K\r\n",
                        addr_j1, addr_j2, addr_j3, no_app);

        /* CAN bus status — show all 8 board slots.
         *
         * Each board shows one of:
         *   "OK(nn)"    = heartbeat received within 3s, counter=nn
         *   "WN(nn)"    = heartbeat with state=Warning
         *   "FT(nn)"    = heartbeat with state=Fault
         *   "--"        = no heartbeat received (or timed out)
         *
         * The 3-second timeout matches the architecture spec:
         * "If a heartbeat goes missing for >3 seconds, the master
         *  declares a communication fault." */
        {
            static const char *state_tags[] = {"OK", "WN", "FT"};
            int ci;
            char node_label[12];

            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "CAN1:");

            for (ci = 0; ci < 8; ci++)
            {
                if (ci == 0)
                {
                    snprintf(node_label, sizeof(node_label), "Master");
                }
                else
                {
                    snprintf(node_label,
                             sizeof(node_label),
                             "Slave%d",
                             ci);
                }

                if (g_can_last_seen[ci] != 0 &&
                    (now_ms - g_can_last_seen[ci]) < 3000U)
                {
                    /* Board is alive — show state and counter */
                    uint8_t st = g_can_last_state[ci];
                    if (st > 2) { st = 2; }  /* Clamp to valid */

                    pos += snprintf(&scr[pos],
                                    UART1_TX_SIZE - (size_t)pos,
                                    " %s=%s(%u)",
                                    node_label,
                                    state_tags[st],
                                    g_can_last_counter[ci]);
                }
                else
                {
                    /* Board not seen or timed out */
                    pos += snprintf(&scr[pos],
                                    UART1_TX_SIZE - (size_t)pos,
                                    " %s=--", node_label);
                }
            }

            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "\033[K\r\n");

            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "CAN2:");

            for (ci = 0; ci < 8; ci++)
            {
                if (ci == 0)
                {
                    snprintf(node_label, sizeof(node_label), "Master");
                }
                else
                {
                    snprintf(node_label,
                             sizeof(node_label),
                             "Slave%d",
                             ci);
                }

                if (g_can2_last_seen[ci] != 0 &&
                    (now_ms - g_can2_last_seen[ci]) < 3000U)
                {
                    uint8_t st = g_can2_last_state[ci];
                    if (st > 2) { st = 2; }

                    pos += snprintf(&scr[pos],
                                    UART1_TX_SIZE - (size_t)pos,
                                    " %s=%s(%u)",
                                    node_label,
                                    state_tags[st],
                                    g_can2_last_counter[ci]);
                }
                else
                {
                    pos += snprintf(&scr[pos],
                                    UART1_TX_SIZE - (size_t)pos,
                                    " %s=--", node_label);
                }
            }

            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "\033[K\r\n");
        }

        if (!saved_debug)
        {
            /* debug=false: erase everything below the status
             * block so no old debug output lingers on screen. */
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "\033[J");
        }
        else
        {
            /* debug=true: print separator before debug area */
            pos += snprintf(&scr[pos], UART1_TX_SIZE - (size_t)pos,
                            "--- DEBUG ---\033[K\r\n"
                            "\033[J");
        }

        /* Clamp to buffer size */
        if (pos >= (int)UART1_TX_SIZE)
        {
            pos = (int)UART1_TX_SIZE - 1;
        }

        /* Fire DMA — returns immediately, CPU is free while
         * the ~700-900 bytes are clocked out at 115200 bps
         * (~60-80 ms of background transmission). */
        uart1_dma_send_buf((uint16_t)pos);
    }

    /* ---- Phase 3: Debug dump (only when sys_debug = true) ----
     *
     * Re-read FAULT_SUMMARY with debug enabled so the full
     * TX/RX frames and fault decode appear below the separator.
     *
     * Only the fault status is re-read (1-3 register reads).
     * Cell voltages, temperatures, and ISO are NOT re-read —
     * their results are already in the status block above.
     */
    if (saved_debug && board_has_bq)
    {
        /* bq79616_get_status() will print the full fault decode
         * including TX/RX frames via dbg() calls.  Since sys_debug
         * is currently true, these flow straight to UART1 and
         * appear below the "--- DEBUG ---" separator. */
        bq79616_get_status(0x01);
    }
}


/* Called every 10 ms from TIM3 ISR */
static inline void pd9_sample_10ms(void)
{
    DigitalLevel s = digital_read(PD9);

    if (s == LOW)
    {
        if (g_pd9_low_ticks < 0xFFFF)
        {
            g_pd9_low_ticks++;
        }

        /* 50 * 10 ms = 500 ms hold */
        if (g_pd9_armed && g_pd9_low_ticks >= 50)
        {
            g_pd9_shutdown_req = 1;  /* request shutdown in main thread */
            g_pd9_armed = 0;         /* fire once per hold; re-arms when PD9 goes HIGH */
        }
    }
    else
    {
        g_pd9_low_ticks = 0;
        g_pd9_armed     = 1;
    }
}


/* Called every 10 ms from TIM3 ISR
 *
 * Mirrors the PD9 shutdown sampler but works in reverse:
 *   - PD8 (MCU_KEY_START) goes HIGH when the key is in "start"
 *   - We count consecutive HIGH samples
 *   - After 20 ticks (200 ms) we set g_pd8_startup_req
 *   - The one-shot latch prevents repeat triggers while held
 *   - Releasing the key (PD8 goes LOW) re-arms the latch
 *
 * The main loop checks g_pd8_startup_req and also checks
 * g_contactors_live before actually calling startup, so even
 * if the ISR sets the flag, it won't do anything if the
 * contactors are already closed.
 */
static inline void pd8_sample_10ms(void)
{
    DigitalLevel s = digital_read(PD8);

    if (s == HIGH)
    {
        /* Key is being held in start position - count up */
        if (g_pd8_high_ticks < 0xFFFF)
        {
            g_pd8_high_ticks++;
        }

        /* 20 * 10 ms = 200 ms hold required */
        if (g_pd8_armed && g_pd8_high_ticks >= 20)
        {
            g_pd8_startup_req = 1;   /* request startup in main thread */
            g_pd8_armed = 0;         /* fire once; re-arms when PD8 goes LOW */
        }
    }
    else
    {
        /* Key released - reset counter and re-arm the one-shot */
        g_pd8_high_ticks = 0;
        g_pd8_armed      = 1;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 1 Hz event: begin 10 ms LED pulse */
        digital_write(PC13, HIGH);
        g_led_ticks_10ms = 1;   /* 1 tick @ 10 ms; TIM3 will clear it */
    }
    else if (htim->Instance == TIM3)
    {
        /* ===== 10 ms heartbeat ===== */

        /* LED one-shot handling */
        if (g_led_ticks_10ms)
        {
            g_led_ticks_10ms--;
            if (g_led_ticks_10ms == 0)
            {
                digital_write(PC13, LOW);
            }
        }

        /* PD9 low-hold sampler (500 ms trigger -> shutdown) */
        pd9_sample_10ms();

        /* PD8 high-hold sampler (200 ms trigger -> startup) */
        pd8_sample_10ms();
    }
}
