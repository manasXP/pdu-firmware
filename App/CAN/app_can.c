/**
 * @file    app_can.c
 * @brief   CAN protocol — FDCAN1 500 kbps, status broadcast, command parse
 */

#include "app_can.h"
#include "app_protection.h"
#include "main.h"

static uint8_t  s_node_id = 0;
static uint32_t s_last_rx_tick = 0;

void App_CAN_Init(void)
{
    /* TODO: Read DIP switch for node ID */
    s_node_id = 0;

    /* TODO: Configure FDCAN1 Rx filters, Tx mailbox */
    /* TODO: Start FDCAN */
}

uint8_t App_CAN_GetNodeID(void)
{
    return s_node_id;
}

/**
 * @brief  Broadcast current state over FDCAN1
 * @param  state  Current AppState_t value (0–9)
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
 * @brief  Poll Rx FIFO for diagnostic request frames
 *
 * Checks for frames on CAN_ID_DIAG_REQ (0x7E0).
 * If data[0] == CAN_DIAG_CMD_CLEAR_FAULTS, clears all faults.
 */
void App_CAN_ProcessDiagRequest(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    /* Poll Rx FIFO 0 for pending messages */
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
                                &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    s_last_rx_tick = HAL_GetTick();

    /* Check for diagnostic request */
    if (rx_header.Identifier == CAN_ID_DIAG_REQ)
    {
        if (rx_data[0] == CAN_DIAG_CMD_CLEAR_FAULTS)
        {
            App_Protection_DiagClear();
        }
    }
}

void App_CAN_Tick(void)
{
    /* Process incoming diagnostic requests */
    App_CAN_ProcessDiagRequest();

    /* TODO: 10 ms status broadcast (V_out, I_out, faults, temp, state) */
    /* TODO: CAN watchdog: warn at 50 ms, shutdown at 200 ms */
    (void)s_last_rx_tick;
}
