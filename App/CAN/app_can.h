/**
 * @file    app_can.h
 * @brief   CAN protocol — status/command frames, master FSM, stacking
 */

#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>

/* CAN frame IDs (11-bit standard) */
#define CAN_ID_STATUS_BASE       0x100  /* + node_id */
#define CAN_ID_COMMAND           0x010
#define CAN_ID_MASTER_ANNOUNCE   0x001
#define CAN_ID_DISCOVERY_REQ     0x0F0
#define CAN_ID_DISCOVERY_RSP     0x0F1
#define CAN_ID_DIAG_REQ          0x7E0
#define CAN_ID_DIAG_RSP          0x7E8

/* Watchdog thresholds */
#define CAN_WATCHDOG_WARN_MS     50
#define CAN_WATCHDOG_SHUTDOWN_MS 200

void    App_CAN_Init(void);
void    App_CAN_BroadcastState(uint8_t state);
void    App_CAN_Tick(void);
uint8_t App_CAN_GetNodeID(void);

#endif /* APP_CAN_H */
