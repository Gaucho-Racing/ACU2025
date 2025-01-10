#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "states.h"
#include "Battery.h"
#include "mcu_wrapper.h"
#include "bcc.h"
#include "SPISlave_T4.h"
#include "debug.h"

#define DEBUG

// Battery
Battery* battery = new Battery;
bcc_status_t bccError;
bcc_drv_data_t drv_data; // contains cellMap, rxBuf, msgCenter
bool bccInitialized = false; 
bool loopBackOk = true; 

// State
States state;

// SPI
SPIClass* BCC_TX_SPI = &SPI1;
SPISlave_T4<&SPI, SPI_8_BITS> BCC_RX_SPI; // 8-bit data mode
uint32_t spiRx[10]; // Array to store received SPI data.
volatile int spiRxIdx; //  Index for received SPI data
volatile int spiRxComplete = 0; // Flag to indicate if SPI reception is complete

// BCC MCU timeout thingy
uint32_t BCC_MCU_Timeout_Start;

void precharge(){

}

void standby(){
    
}

bool systemCheck(){
  Serial.println("Checking Battery...");
  battery->check_acu();
  battery->check_battery();
  return true;
  
  /*
  acu.checkACU(startup);

  //D_L1("Checking Battery");

  battery.checkBattery(fullCheck);

  //D_L1("System Check Done");
  //D_L1();

  digitalWrite(PIN_AMS_OK, (acu.errs & ERR_OverTemp) == 0);
  return acu.errs != 0;
  */

}

void shutdown(bool randomBoolean){
  Serial.println("Shutting down cell balancing...");
  battery->toggleCellBalancing(true, false, BCC_CID_UNASSIG, 0);
  // set TS inactive
}

void setup() {
  Serial.begin(1000000);

  pinMode(PIN_BCC_TX_CS, OUTPUT);
  pinMode(PIN_BCC_EN, OUTPUT);
  pinMode(PIN_BCC_INT, INPUT);
  digitalWrite(PIN_BCC_TX_CS, HIGH);
  digitalWrite(PIN_BCC_EN, LOW);

  Serial.println("Init SPIs");
  BCC_TX_SPI->begin(); // init SPI bus for transmission
  BCC_RX_SPI.begin(); // init the SPI bus for reception (slave mode)

  // Initialize BCC functions
  Serial.println("Init BCC functions");
  battery->drvConfig.commMode = BCC_MODE_TPL;
  battery->drvConfig.drvInstance = 0U;
  battery->drvConfig.devicesCnt = NUM_TOTAL_IC;
  Serial.println("Init devices");
  for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
      battery->drvConfig.device[i] = BCC_DEVICE_MC33771C;
      battery->drvConfig.cellCnt[i] = NUM_CELL_IC;
  }
  battery->drvConfig.loopBack = false;

  state = STANDBY;
  Serial.println("BCC_Init...");
  bccError = BCC_Init(&(battery->drvConfig));
  while (bccError != bcc_status_t::BCC_STATUS_SUCCESS){
    Serial.print(" failed: ");
    print_bcc_status(bccError);
    delay(1000);
    Serial.print("\n\n\n");
    bccError = BCC_Init(&(battery->drvConfig));
  }
  Serial.println("success");

  Serial.println("Battery init...");
  battery->init();
}

uint32_t prev_mill = 0;
void loop() {
  // Serial.print("Please work\n");
  // put your main code here, to run repeatedly:
  switch (state)
  {
    case STANDBY:
      // get tsCurrent();
      // systemCheck();
      break;

    case PRECHARGE:
      // preChargeState();
      break;

    case CHARGE:
      // chargeState();
      break;

    case DRIVE:
      // driveState();
      break;

    case SHUTDOWN:
      // shutdown(true);
      break;

    default:
      state = SHUTDOWN;
      break;
  }
    
  battery->readDeviceMeasurements();
  bccError = BCC_Sleep(&(battery->drvConfig));
  delay(200);
  BCC_WakeUp(&(battery->drvConfig));
  
  #ifdef DEBUG
    if(millis() - prev_mill > 500){
      prev_mill = millis();
      debug(battery);
    }
  #endif
}