#include "pins.h" // defining pin configurations
#include "config.h" // defining configuration values

#include <bcc_com_c.h>
#include "bcc_c.h" // Real version

#include <Battery.h>
#include <SPI.h>
#include "SPISlave_T4.h" // SPI slave library for Teensy 4

Battery * battery;
SPIClass* BCC_TX_SPI = &SPI1;
SPISlave_T4<&SPI, SPI_8_BITS> BCC_RX_SPI; // 8-bit data mode
uint32_t spiRx[10]; // Array to store received SPI data.
volatile int spiRxIdx; //  Index for received SPI data
volatile int spiRxComplete = 0; // Flag to indicate if SPI reception is complete

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  BCC_TX_SPI->begin(); // init SPI bus for transmission
  BCC_RX_SPI.begin(); // init the SPI bus for reception (slave mode)

  bcc_status_t errors = battery->init(); // initialize BCC Controller

  if(!errors != BCC_STATUS_SUCCESS){ // system check
    battery->state = SHUTDOWN;
    D_PRINT("State => Shutdown");
  }
  else{
    battery->state = STANDBY;
    D_PRINT("State => Standby");
  }
}

void loop() {
  Serial.print("Please work\n");
  // put your main code here, to run repeatedly:
  switch (battery->state)
  {
    case STANDBY:
      // standByState();
      break;

    case PRECHARGE:
      // preChargeState();
      break;

    case CHARGE:
      // chargeState();  //TODO
      break;

    case NORMAL:
      // normalState();
      break;

    case SHUTDOWN:
      // shutdownState();
      break;

    default:
      // state = SHUTDOWN;
      // D_L1("Uh oh u dummy, u've entered a non-existent state");
      // delay(10000);
      break;
    
    battery->readDeviceMeasurements();
    // dump data to CAN bus
    // dump data to fan controller

    #ifdef DEBUG
      if(millis() - prev_mill > 500){
        prev_mill = millis();
        debug();
      }
    #endif
  }
}