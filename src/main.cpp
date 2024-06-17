#include <Arduino.h>
#include <SPI.h>
#include "bcc.h"
#include "SPISlave_T4.h" // hope this thing works
#include "pins.h"


SPIClass* BCC_TX_SPI = &SPI1;
SPISlave_T4<&SPI, SPI_8_BITS> BCC_RX_SPI;
uint32_t spiRx[10];
volatile int spiRxIdx;
volatile int spiRxComplete = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  BCC_TX_SPI->begin();
  BCC_RX_SPI.begin();
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

bcc_status_t BCC_MCU_TransferSpi(uint8_t drvInstance, uint8_t transBuf[], uint8_t recvBuf[]) {
  // shouldn't be called (using Tpl mode), but written anyway
  Serial.print("This shouldn't happen\n");
  BCC_TX_SPI->transfer(transBuf, recvBuf, 5);
  return BCC_STATUS_SUCCESS;
}

bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[], uint8_t recvBuf[], uint16_t recvTrCnt) {
  Serial.printf("drvInstance %u transferTpl\n", drvInstance); // what does drvInstance do?
  BCC_TX_SPI->transfer(transBuf, 5);
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

void BCC_MCU_WriteCsbPin(uint8_t drvInstance, uint8_t value) {
  digitalWriteFast(PIN_BCC_TX_CS, value);
}

void BCC_MCU_WriteRstPin(uint8_t drvInstance, uint8_t value) {
  //digitalWriteFast(PIN_BCC_TX_RST, value);
}

void BCC_MCU_WriteEnPin(uint8_t drvInstance, uint8_t value) {
  digitalWriteFast(PIN_BCC_EN, value);
}

uint32_t BCC_MCU_ReadIntbPin(uint8_t drvInstance) {
  return digitalReadFast(PIN_BCC_INT);
}