/**
 * @file    app_can.c
 * @brief   CAN protocol — FDCAN1 500 kbps, status broadcast, command parse
 */

#include "app_can.h"
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

void App_CAN_Tick(void)
{
    /* TODO: 10 ms status broadcast (V_out, I_out, faults, temp, state) */
    /* TODO: Check Rx for command frames */
    /* TODO: CAN watchdog: warn at 50 ms, shutdown at 200 ms */
    (void)s_last_rx_tick;
}
