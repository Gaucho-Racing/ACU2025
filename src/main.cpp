#include <Arduino.h>
#include <SPI.h>
#include "bcc.h"

SPIClass* BCC_TX_SPI;
SPIClass* BCC_RX_SPI;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
}

void loop() {
  Serial.print("Please work\n");
  // put your main code here, to run repeatedly:
}

void BCC_MCU_WaitMs(uint16_t ms) {
  delay(ms);
}

void BCC_MCU_WaitUs(uint32_t us) {
  delayMicroseconds(us);
}

void BCC_MCU_Assert(bool x) {
  if (!x) Serial.print("BCC assertion failed\n");
}

bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[], uint8_t recvBuf[], uint16_t recvTrCnt) {
  BCC_TX_SPI->transfer(transBuf, 5);
  // STUB: figure out SPI slave mode
  for (uint8_t i = 0; i < recvTrCnt) {
    recvBuf[i] = 0;
    recvBuf[i+1] = 0;
    recvBuf[i+2] = 0;
    recvBuf[i+3] = 0;
    recvBuf[i+4] = 0;
  }
}