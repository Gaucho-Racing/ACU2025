#include "mcu_wrapper.h"

void BCC_MCU_WaitMs(uint16_t delay) {
  for (uint16_t i = 0; i < delay; i++) {
    delayMicroseconds(1000);
  }
}

void BCC_MCU_WaitUs(uint32_t delay) {
  delayMicroseconds(delay);
}

extern uint32_t BCC_MCU_Timeout_Start;
bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs){
  //Serial.print("BCC_MCU_StartTimeout not used\n");
  BCC_MCU_Timeout_Start = micros();
  return BCC_STATUS_SUCCESS;
}

bool BCC_MCU_TimeoutExpired(void){
  return (micros() - BCC_MCU_Timeout_Start) > 1000;
}

void BCC_MCU_Assert(const bool x) {
  if (!x) Serial.print("BCC assertion failed\n");
}

bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[]){
      Serial.print("BCC_MCU_TransferSpi not used\n");
      return BCC_STATUS_SUCCESS;
}

extern SPIClass* BCC_TX_SPI;
bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt) {
  //Serial.printf("drvInstance %u transferTpl\n", drvInstance); // what does drvInstance do?
  //Serial.printf("recvTrCnt:%d\n", recvTrCnt);
  // send
  BCC_TX_SPI->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  digitalWriteFast(PIN_BCC_TX_CS, LOW);
  uint8_t buf[BCC_MSG_SIZE];
  memcpy(buf, txBuf, 6);
  delayNanoseconds(1500);
  BCC_TX_SPI->transfer(buf, BCC_MSG_SIZE);
  delayNanoseconds(1000);
  digitalWriteFast(PIN_BCC_TX_CS, HIGH);
  BCC_TX_SPI->endTransaction();
  // receive
  for (uint16_t rxCount = 0; rxCount < recvTrCnt; rxCount++) {
    uint32_t startTime = millis();
    while ((!spiRxComplete) || spiRxIdx < BCC_MSG_SIZE) {
      if (millis() - startTime > 10) {
        Serial.printf("SPI RX frame %u Timeout\n", rxCount);
        return BCC_STATUS_COM_TIMEOUT;
      }
    }
    for (uint8_t i = 0; i < BCC_MSG_SIZE; i++) {
      rxBuf[rxCount*BCC_MSG_SIZE + i] = spiRx[i];
    }
    // Serial.print("txData:");
    // for (uint8_t i = 0; i < spiRxIdx; i++) {
    //   Serial.print(txBuf[i], HEX); Serial.print(" ");
    // }
    // Serial.print("  rxData:");
    // for (uint8_t i = 0; i < spiRxIdx; i++) {
    //   Serial.print(rxBuf[rxCount*BCC_MSG_SIZE + i], HEX); Serial.print(" ");
    // }
    // Serial.printf("spiRxIdx:%d\n", spiRxIdx);
    // Serial.println();
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