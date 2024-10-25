#include "mcu_wrapper.h"

void BCC_MCU_WaitMs(uint16_t delay) {
  delayMicroseconds((uint32_t)(0.001 * delay));
}

void BCC_MCU_WaitUs(uint32_t delay) {
  delayMicroseconds(delay);
}

bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs){
  Serial.print("BCC_MCU_StartTimeout not used\n");
  return BCC_STATUS_SUCCESS;
}

bool BCC_MCU_TimeoutExpired(void){
  return false;
}

void BCC_MCU_Assert(const bool x) {
  if (!x) Serial.print("BCC assertion failed\n");
}

bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[]){
      Serial.print("BCC_MCU_TransferSpi not used\n");
      return BCC_STATUS_SUCCESS;
}

bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt) {
  Serial.printf("drvInstance %u transferTpl\n", drvInstance); // what does drvInstance do?
//   BCC_TX_SPI->transfer(transBuf, 5);
  for (uint16_t rxCount = 0; rxCount < recvTrCnt; rxCount++) {
    uint32_t startTime = millis();
    while (!spiRxComplete) {
      if (millis() - startTime > 10) {
        Serial.printf("SPI RX frame %u Timeout\n", rxCount);
        return BCC_STATUS_COM_TIMEOUT;
      }
    }
    Serial.println(spiRxIdx);
    for (uint8_t i = 0; i < spiRxIdx; i++) {
      Serial.print(spiRx[i], HEX); Serial.print(" ");
      recvBuf[rxCount*5 + i] = spiRx[i];
    }
    Serial.println();
    spiRxComplete = 0;
    spiRxIdx = 0;
  }
  return BCC_STATUS_SUCCESS;
}

void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value) {
  digitalWriteFast(PIN_BCC_TX_CS, value);
}

void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value) {
  digitalWriteFast(PIN_BCC_TX_RST, value);
}

void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value) {
  digitalWriteFast(PIN_BCC_EN, value);
}

uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance) {
  return digitalReadFast(PIN_BCC_INT);
}