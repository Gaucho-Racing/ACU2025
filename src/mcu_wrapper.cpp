#include "mcu_wrapper.h"

void BCC_MCU_WaitMs(uint16_t delay) {
  for (uint16_t i = 0; i < delay; i++) {
    delayMicroseconds(1000);
  }
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

extern SPIClass* BCC_TX_SPI;
bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt) {
  //Serial.printf("drvInstance %u transferTpl\n", drvInstance); // what does drvInstance do?
  Serial.printf("recvTrCnt:%d\n", recvTrCnt);
  bool sent = false;
  for (uint16_t rxCount = 0; rxCount < recvTrCnt; rxCount++) {
    // send
    BCC_TX_SPI->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
    digitalWriteFast(PIN_BCC_TX_CS, LOW);
    uint8_t buf[BCC_MSG_SIZE];
    memcpy(buf, txBuf, 6);
    delayNanoseconds(1500);
    // if (sent) {
    //   buf[0] = 0;
    //   buf[1] = 0;
    //   buf[2] = 0;
    //   buf[3] = 0;
    //   buf[4] = 0;
    //   buf[5] = 0;
    // }
    BCC_TX_SPI->transfer(buf, BCC_MSG_SIZE);
    sent = true;
    // BCC_TX_SPI->transfer(buf[BCC_MSG_IDX_DATA_H]);
    // BCC_TX_SPI->transfer(buf[BCC_MSG_IDX_DATA_L]);
    // BCC_TX_SPI->transfer(buf[BCC_MSG_IDX_ADDR]);
    // BCC_TX_SPI->transfer(buf[BCC_MSG_IDX_CID]);
    // BCC_TX_SPI->transfer(buf[BCC_MSG_IDX_CNT_CMD]);
    // BCC_TX_SPI->transfer(buf[BCC_MSG_IDX_CRC]);
    delayNanoseconds(1000);
    digitalWriteFast(PIN_BCC_TX_CS, HIGH);
    BCC_TX_SPI->endTransaction();

    // receive
    uint32_t startTime = millis();
    while (!spiRxComplete || spiRxIdx < BCC_MSG_SIZE) {
      if (millis() - startTime > 10) {
        Serial.printf("SPI RX frame %u Timeout\n", rxCount);
        return BCC_STATUS_COM_TIMEOUT;
      }
    }
    Serial.printf("spiRxIdx:%d\n", spiRxIdx);
    for (uint8_t i = 0; i < spiRxIdx; i++) {
      rxBuf[rxCount*BCC_MSG_SIZE + i] = spiRx[i];
    }
    // rxBuf[rxCount*BCC_MSG_SIZE + BCC_MSG_IDX_DATA_H] = spiRx[0];
    // rxBuf[rxCount*BCC_MSG_SIZE + BCC_MSG_IDX_DATA_L] = spiRx[1];
    // rxBuf[rxCount*BCC_MSG_SIZE + BCC_MSG_IDX_ADDR] = spiRx[2];
    // rxBuf[rxCount*BCC_MSG_SIZE + BCC_MSG_IDX_CID] = spiRx[3];
    // rxBuf[rxCount*BCC_MSG_SIZE + BCC_MSG_IDX_CNT_CMD] = spiRx[4];
    // rxBuf[rxCount*BCC_MSG_SIZE + BCC_MSG_IDX_CRC] = spiRx[5];
    Serial.print("txData:");
    for (uint8_t i = 0; i < spiRxIdx; i++) {
      Serial.print(txBuf[i], HEX); Serial.print(" ");
    }
    Serial.print("  rxData:");
    for (uint8_t i = 0; i < spiRxIdx; i++) {
      Serial.print(rxBuf[rxCount*BCC_MSG_SIZE + i], HEX); Serial.print(" ");
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