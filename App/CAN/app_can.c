/**
 * @file    app_can.c
 * @brief   CAN protocol — FDCAN1 500 kbps, status broadcast, command parse
 */

#include "app_can.h"

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

void App_CAN_BroadcastState(uint8_t state)
{
    /* TODO: Build CAN frame with node ID + state, transmit */
    (void)state;
}

void App_CAN_Tick(void)
{
    /* TODO: 10 ms status broadcast (V_out, I_out, faults, temp, state) */
    /* TODO: Check Rx for command frames */
    /* TODO: CAN watchdog: warn at 50 ms, shutdown at 200 ms */
}
