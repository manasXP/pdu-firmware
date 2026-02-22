/**
 * @file    app_can.h
 * @brief   CAN protocol — status/command frames, master FSM, stacking
 */

#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>

/* CAN frame IDs (11-bit standard) */
#define CAN_ID_STATUS_BASE       0x100U  /* + node_id */
#define CAN_ID_COMMAND           0x010U
#define CAN_ID_MASTER_ANNOUNCE   0x001U
#define CAN_ID_DISCOVERY_REQ     0x0F0U
#define CAN_ID_DISCOVERY_RSP     0x0F1U
#define CAN_ID_DIAG_REQ          0x7E0U
#define CAN_ID_DIAG_RSP          0x7E8U

/* Diagnostic command IDs */
#define CAN_DIAG_CMD_CLEAR_FAULTS 0x01U

/* Watchdog thresholds */
#define CAN_WATCHDOG_WARN_MS     50U
#define CAN_WATCHDOG_SHUTDOWN_MS 200U

/* Status frame scaling */
#define CAN_VOLTAGE_SCALE        10U    /* 0.1 V/LSB */
#define CAN_CURRENT_SCALE        10U    /* 0.1 A/LSB */
#define CAN_TEMP_OFFSET          40     /* +40 degC offset */

/** @brief  Parsed command frame from master / charger controller */
typedef struct
{
    uint16_t v_ref;     /* Voltage reference (0.1 V/LSB) */
    uint16_t i_ref;     /* Current reference (0.1 A/LSB) */
    uint8_t  enable;    /* bit0 = enable request */
    uint8_t  valid;     /* 1 = at least one command received */
} CAN_Command_t;

void             App_CAN_Init(void);
void             App_CAN_BroadcastState(uint8_t state);
void             App_CAN_BroadcastFault(uint8_t fault_id, uint8_t severity);
void             App_CAN_BroadcastStatus(void);
void             App_CAN_Tick(void);
uint8_t          App_CAN_GetNodeID(void);
const CAN_Command_t *App_CAN_GetCommand(void);
uint8_t          App_CAN_IsWatchdogExpired(void);

#endif /* APP_CAN_H */
