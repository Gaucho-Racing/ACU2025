#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "states.h"
#include "Battery.h"
#include "mcu_wrapper.h"
#include "bcc.h"
#include "SPISlave_T4.h"

// Battery
Battery * battery;
bcc_status_t bccError;
bcc_drv_data_t drv_data; // contains cellMap, rxBuf, msgCenter
bool bccInitialized = false; 
bool loopBackOk = true; 

uint8_t transBuf[5];  // Adjust size as per your protocol
uint8_t recvBuf[RX_BUF_SIZE];
uint16_t recvTrCnt;

// State
States state;

// SPI
SPIClass* BCC_TX_SPI = &SPI1;
SPISlave_T4<&SPI, SPI_8_BITS> BCC_RX_SPI; // 8-bit data mode
uint32_t spiRx[10]; // Array to store received SPI data.
volatile int spiRxIdx; //  Index for received SPI data
volatile int spiRxComplete = 0; // Flag to indicate if SPI reception is complete

void nop(bcc_drv_config_t * const drvConfig);

void setup() {

    Serial.begin(1000000);
    BCC_TX_SPI->begin(); // init SPI bus for transmission
    BCC_RX_SPI.begin(); // init the SPI bus for reception (slave mode)

    // Initialize BCC functions
    battery->drvConfig.commMode = BCC_MODE_TPL;
    battery->drvConfig.drvInstance = 0U;
    battery->drvConfig.devicesCnt = NUM_TOTAL_IC;

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        battery->drvConfig.device[i] = BCC_DEVICE_MC33771C;
        battery->drvConfig.cellCnt[i] = NUM_CELL_IC;
    }
    battery->drvConfig.loopBack = true;

    state = STANDBY;
    Serial.println("State => Standby");
}

void loop() {
  Serial.println("Please work\n");
  // nop(&(battery->drvConfig));
  // put your main code here, to run repeatedly:
  switch (state)
  {
    case STANDBY:
      bccError = BCC_Init(&(battery->drvConfig));
      if(bccError != bcc_status_t::BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
      }
      else state = PRECHARGE;
      battery->init();
      break;

    case PRECHARGE:
      // preChargeState();
      break;

    case CHARGE:
      // chargeState();
      break;

    case NORMAL:
      // normalState();
      break;

    case SHUTDOWN:
      // shutdownState();
      break;

    default:
      // state = SHUTDOWN;
      delay(10000);
      break;
    
    
    battery->readDeviceMeasurements();
    // nop(&(battery->drvConfig));
    battery->printDeviceMeasurements();

    bccError = BCC_Sleep(&(battery->drvConfig));
    for (uint8_t i = 0; i < 5000000; i++);
    BCC_WakeUp(&(battery->drvConfig));
    
    #ifdef DEBUG
      if(millis() - prev_mill > 500){
        prev_mill = millis();
        debug();
      }
    #endif
  }
}

void nop(bcc_drv_config_t * const drvConfig) {
    for (uint8_t cid = 1; cid <= drvConfig->devicesCnt; cid++) {
        bcc_cid_t chosen = (bcc_cid_t)cid;
        BCC_SendNop(drvConfig, chosen);
    }
}