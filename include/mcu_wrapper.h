#ifndef MCU_H
#define MCU_H
#include "SPISlave_T4.h"
#include "pins.h"
#include <Arduino.h>
#include "config.h"
#include "bcc.h"

extern uint8_t transBuf[5];  // Adjust size as per your protocol
extern uint8_t recvBuf[RX_BUF_SIZE];
extern uint16_t recvTrCnt;

void BCC_MCU_WaitUs(uint32_t delay);
void BCC_MCU_WaitMs(uint16_t delay);
bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs);
bool BCC_MCU_TimeoutExpired(void);
void BCC_MCU_Assert(const bool x);
bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[]);
bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt);
void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value);
void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value);
void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value);
uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance);
#endif