/**
 * @file    app_can.c
 * @brief   CAN protocol — FDCAN1 500 kbps, status broadcast, command parse
 *
 * Status frame: TX every 10 ms on CAN_ID_STATUS_BASE + node_id (8 bytes)
 * Command frame: RX on CAN_ID_COMMAND (8 bytes), parsed into CAN_Command_t
 * Watchdog: 50 ms warning, 200 ms fault (only in RUN/DERATE after first cmd)
 */

#include "app_can.h"
#include "app_adc.h"
#include "app_protection.h"
#include "app_statemachine.h"
#include "main.h"

/* ------------------------------------------------------------------ */
/*  Module State                                                       */
/* ------------------------------------------------------------------ */

static uint8_t       s_node_id       = 0U;
static uint32_t      s_last_tx_tick  = 0U;
static uint32_t      s_last_cmd_tick = 0U;
static uint8_t       s_cmd_received  = 0U;   /* First command seen flag  */
static uint8_t       s_watchdog_warned = 0U; /* Warning already issued   */
static CAN_Command_t s_command       = {0};
static uint32_t      s_busoff_count  = 0U;

/* ------------------------------------------------------------------ */
/*  Static Helpers                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief  Read DIP switch GPIO pins (active-low) to get 3-bit node ID
 * @return Node ID 0–7
 */
static uint8_t dip_switch_read(void)
{
    uint8_t id = 0U;

    if (HAL_GPIO_ReadPin(DIP_SW_PORT, DIP_SW0_PIN) == GPIO_PIN_RESET)
    {
        id |= 0x01U;
    }
    if (HAL_GPIO_ReadPin(DIP_SW_PORT, DIP_SW1_PIN) == GPIO_PIN_RESET)
    {
        id |= 0x02U;
    }
    if (HAL_GPIO_ReadPin(DIP_SW_PORT, DIP_SW2_PIN) == GPIO_PIN_RESET)
    {
        id |= 0x04U;
    }

    return id;
}

/**
 * @brief  CAN watchdog — only active after first command AND in RUN/DERATE
 *
 * 50 ms without command: broadcast warning fault event.
 * 200 ms without command: enter FAULT_CAN_TIMEOUT.
 */
static void can_watchdog_check(void)
{
    if (s_cmd_received == 0U)
    {
        return;
    }

    AppState_t state = App_SM_GetState();
    if ((state != STATE_RUN) && (state != STATE_DERATE))
    {
        s_watchdog_warned = 0U;
        return;
    }

    uint32_t elapsed = HAL_GetTick() - s_last_cmd_tick;

    if (elapsed >= CAN_WATCHDOG_SHUTDOWN_MS)
    {
        Fault_Enter(FAULT_CAN_TIMEOUT);
        s_watchdog_warned = 0U;
    }
    else if ((elapsed >= CAN_WATCHDOG_WARN_MS) && (s_watchdog_warned == 0U))
    {
        App_CAN_BroadcastFault((uint8_t)FAULT_CAN_TIMEOUT,
                               (uint8_t)FAULT_SEV_WARNING);
        s_watchdog_warned = 1U;
    }
}

/**
 * @brief  Check FDCAN protocol status for bus-off condition
 */
static void can_busoff_check(void)
{
    FDCAN_ProtocolStatusTypeDef psr;

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr) != HAL_OK)
    {
        return;
    }

    if (psr.BusOff != 0U)
    {
        s_busoff_count++;
        /* FDCAN auto-retransmission is enabled — recovery is automatic
         * after 128 × 11 recessive bits. Log the event. */
        App_CAN_BroadcastFault(0xFEU, (uint8_t)s_busoff_count);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void App_CAN_Init(void)
{
    s_node_id      = dip_switch_read();
    s_last_tx_tick = HAL_GetTick();
    s_last_cmd_tick = HAL_GetTick();
    s_cmd_received = 0U;
    s_watchdog_warned = 0U;
    s_busoff_count = 0U;
    s_command.valid = 0U;
}

uint8_t App_CAN_GetNodeID(void)
{
    return s_node_id;
}

const CAN_Command_t *App_CAN_GetCommand(void)
{
    return &s_command;
}

uint8_t App_CAN_IsWatchdogExpired(void)
{
    if (s_cmd_received == 0U)
    {
        return 0U;
    }

    uint32_t elapsed = HAL_GetTick() - s_last_cmd_tick;
    return (elapsed >= CAN_WATCHDOG_SHUTDOWN_MS) ? 1U : 0U;
}

uint8_t App_CAN_IsWatchdogWarning(void)
{
    if (s_cmd_received == 0U)
    {
        return 0U;
    }

    uint32_t elapsed = HAL_GetTick() - s_last_cmd_tick;
    return (elapsed >= CAN_WATCHDOG_WARN_MS) ? 1U : 0U;
}

/**
 * @brief  Broadcast current state over FDCAN1
 * @param  state  Current AppState_t value (0-9)
 */
void App_CAN_BroadcastState(uint8_t state)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[1];

    header.Identifier          = CAN_ID_STATUS_BASE + (uint32_t)s_node_id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_1;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    data[0] = state;

    /* Silently discard if Tx FIFO is full */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

/**
 * @brief  Broadcast fault event over FDCAN1
 * @param  fault_id  FaultSource_t value
 * @param  severity  FaultSeverity_t value
 *
 * Sends 3-byte frame on CAN_ID_DIAG_RSP: 0xFF marker + fault_id + severity
 */
void App_CAN_BroadcastFault(uint8_t fault_id, uint8_t severity)
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[3];

    header.Identifier          = CAN_ID_DIAG_RSP;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_3;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    data[0] = 0xFFU;    /* Fault event marker */
    data[1] = fault_id;
    data[2] = severity;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

/**
 * @brief  Broadcast 8-byte status telemetry frame
 *
 * Layout: [0-1] I_out  uint16 BE, 0.1 A/LSB
 *         [2-3] V_out  uint16 BE, 0.1 V/LSB
 *         [4]   Faults uint8 bitmask (lower 5 bits of g_fault_pending)
 *         [5]   Temp   uint8, 1 degC/LSB, +40 offset (max of 4 sensors)
 *         [6]   State  uint8, AppState_t enum (0-9)
 *         [7]   Reserved 0x00
 */
void App_CAN_BroadcastStatus(void)
{
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];

    /* Scale current: clamp negative to 0, then 0.1 A/LSB */
    float i_out = adc->i_out;
    if (i_out < 0.0f)
    {
        i_out = 0.0f;
    }
    uint16_t i_scaled = (uint16_t)(i_out * (float)CAN_CURRENT_SCALE);

    /* Scale voltage: 0.1 V/LSB */
    uint16_t v_scaled = (uint16_t)(adc->v_out * (float)CAN_VOLTAGE_SCALE);

    /* Fault bitmask — lower 5 bits of hardware fault pending register */
    uint8_t faults = (uint8_t)(g_fault_pending & 0x1FU);

    /* Temperature — max of 4 sensors, +40 offset, clamp to uint8 */
    float t_max = adc->t_sic_pfc;
    if (adc->t_sic_llc > t_max)
    {
        t_max = adc->t_sic_llc;
    }
    if (adc->t_magnetics > t_max)
    {
        t_max = adc->t_magnetics;
    }
    if (adc->t_ambient > t_max)
    {
        t_max = adc->t_ambient;
    }
    float t_offset = t_max + (float)CAN_TEMP_OFFSET;
    if (t_offset < 0.0f)
    {
        t_offset = 0.0f;
    }
    if (t_offset > 255.0f)
    {
        t_offset = 255.0f;
    }
    uint8_t temp = (uint8_t)t_offset;

    /* Pack big-endian */
    data[0] = (uint8_t)(i_scaled >> 8U);
    data[1] = (uint8_t)(i_scaled & 0xFFU);
    data[2] = (uint8_t)(v_scaled >> 8U);
    data[3] = (uint8_t)(v_scaled & 0xFFU);
    data[4] = faults;
    data[5] = temp;
    data[6] = (uint8_t)App_SM_GetState();
    data[7] = 0x00U;

    header.Identifier          = CAN_ID_STATUS_BASE + (uint32_t)s_node_id;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
}

/**
 * @brief  Drain Rx FIFO and process all pending messages
 *
 * Handles CAN_ID_COMMAND and CAN_ID_DIAG_REQ frames.
 */
static void can_process_rx(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* Drain all pending messages from Rx FIFO 0 */
    while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                   &rx_header, rx_data) == HAL_OK)
    {
        if (rx_header.Identifier == CAN_ID_COMMAND)
        {
            /* Check target: must be our node_id or broadcast (0xFF) */
            uint8_t target = rx_data[5];
            if ((target != s_node_id) && (target != 0xFFU))
            {
                continue;
            }

            /* Unpack command frame (big-endian) */
            s_command.v_ref  = ((uint16_t)rx_data[0] << 8U)
                             | (uint16_t)rx_data[1];
            s_command.i_ref  = ((uint16_t)rx_data[2] << 8U)
                             | (uint16_t)rx_data[3];
            s_command.enable = rx_data[4] & 0x01U;
            s_command.valid  = 1U;

            /* Reset watchdog */
            s_last_cmd_tick   = HAL_GetTick();
            s_cmd_received    = 1U;
            s_watchdog_warned = 0U;

            /* If enable bit set and we're in STANDBY, request enable */
            if ((s_command.enable != 0U)
                && (App_SM_GetState() == STATE_STANDBY))
            {
                App_SM_RequestEnable();
            }
        }
        else if (rx_header.Identifier == CAN_ID_DIAG_REQ)
        {
            if (rx_data[0] == CAN_DIAG_CMD_CLEAR_FAULTS)
            {
                App_Protection_DiagClear();
            }
        }
        else
        {
            /* Unrecognized frame — ignore */
        }
    }
}

/**
 * @brief  Main CAN tick — call from main loop
 *
 * Processes Rx, broadcasts 10 ms status, checks watchdog and bus-off.
 */
void App_CAN_Tick(void)
{
    /* Drain and process all pending Rx messages */
    can_process_rx();

    /* 10 ms periodic status broadcast */
    uint32_t now = HAL_GetTick();
    if ((now - s_last_tx_tick) >= CAN_STATUS_PERIOD_MS)
    {
        s_last_tx_tick = now;
        App_CAN_BroadcastStatus();
    }

    /* Watchdog and bus-off checks */
    can_watchdog_check();
    can_busoff_check();
}
