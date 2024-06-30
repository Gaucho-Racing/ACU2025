#include <Arduino.h>
#include "bcc.h"
#include "pins.h"
#include "Battery.h"
#include "SPISlave_T4.h"

#define MC33771C

static const bcc_init_reg_t init_regs[INIT_REG_CNT] = {
    {MC33771C_GPIO_CFG1_OFFSET, MC33771C_GPIO_CFG1_POR_VAL, GPIO_CFG1},
    {MC33771C_GPIO_CFG2_OFFSET, MC33771C_GPIO_CFG2_POR_VAL, GPIO_CFG2},
    {MC33771C_TH_ALL_CT_OFFSET, MC33771C_TH_ALL_CT_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT14_OFFSET, MC33771C_TH_CT14_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT13_OFFSET, MC33771C_TH_CT13_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT12_OFFSET, MC33771C_TH_CT12_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT11_OFFSET, MC33771C_TH_CT11_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT10_OFFSET, MC33771C_TH_CT10_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT9_OFFSET, MC33771C_TH_CT9_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT8_OFFSET, MC33771C_TH_CT8_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT7_OFFSET, MC33771C_TH_CT7_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT6_OFFSET, MC33771C_TH_CT6_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT5_OFFSET, MC33771C_TH_CT5_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT4_OFFSET, MC33771C_TH_CT4_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT3_OFFSET, MC33771C_TH_CT3_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT2_OFFSET, MC33771C_TH_CT2_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_CT1_OFFSET, MC33771C_TH_CT1_POR_VAL, PRM_CELL_MAX_VOLT},
    {MC33771C_TH_AN6_OT_OFFSET, MC33771C_TH_AN6_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN5_OT_OFFSET, MC33771C_TH_AN5_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN4_OT_OFFSET, MC33771C_TH_AN4_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN3_OT_OFFSET, MC33771C_TH_AN3_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN2_OT_OFFSET, MC33771C_TH_AN2_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN1_OT_OFFSET, MC33771C_TH_AN1_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN0_OT_OFFSET, MC33771C_TH_AN0_OT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN6_UT_OFFSET, MC33771C_TH_AN6_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN5_UT_OFFSET, MC33771C_TH_AN5_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN4_UT_OFFSET, MC33771C_TH_AN4_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN3_UT_OFFSET, MC33771C_TH_AN3_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN2_UT_OFFSET, MC33771C_TH_AN2_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN1_UT_OFFSET, MC33771C_TH_AN1_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_TH_AN0_UT_OFFSET, MC33771C_TH_AN0_UT_POR_VAL, CELL_MAX_TEMP},
    {MC33771C_CB1_CFG_OFFSET, MC33771C_CB1_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB2_CFG_OFFSET, MC33771C_CB2_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB3_CFG_OFFSET, MC33771C_CB3_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB4_CFG_OFFSET, MC33771C_CB4_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB5_CFG_OFFSET, MC33771C_CB5_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB6_CFG_OFFSET, MC33771C_CB6_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB7_CFG_OFFSET, MC33771C_CB7_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB8_CFG_OFFSET, MC33771C_CB8_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB9_CFG_OFFSET, MC33771C_CB9_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB10_CFG_OFFSET, MC33771C_CB10_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB11_CFG_OFFSET, MC33771C_CB11_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB12_CFG_OFFSET, MC33771C_CB12_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB13_CFG_OFFSET, MC33771C_CB13_CFG_POR_VAL, CBX_CFG},
    {MC33771C_CB14_CFG_OFFSET, MC33771C_CB14_CFG_POR_VAL, CBX_CFG}
};

Battery * battery;
bcc_status_t bccError;
// bcc_drv_data_t drv_data; // contains cellMap, rxBuf, msgCenter

SPIClass* BCC_TX_SPI = &SPI1;
SPISlave_T4<&SPI, SPI_8_BITS> BCC_RX_SPI; // 8-bit data mode
uint32_t spiRx[10]; // Array to store received SPI data.
volatile int spiRxIdx; //  Index for received SPI data
volatile int spiRxComplete = 0; // Flag to indicate if SPI reception is complete

bcc_drv_config_t drvConfig; 
bool bccInitialized = false; 
bool loopBackOk = true; 

void setup() {

    Serial.begin(1000000);
    BCC_TX_SPI->begin(); // init SPI bus for transmission
    BCC_RX_SPI.begin(); // init the SPI bus for reception (slave mode)

    // Initialize BCC functions
    drvConfig.commMode = BCC_MODE_TPL;
    drvConfig.drvInstance = 0U;
    drvConfig.devicesCnt = NUM_TOTAL_IC;

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        drvConfig.device[i] = BCC_DEVICE_MC33771C;
        drvConfig.cellCnt[i] = NUM_CELL_IC;
    }
    drvConfig.loopBack = true;

    battery->state = STANDBY;
    Serial.println("State => Standby");
}

void loop() {
  Serial.println("Please work\n");
  battery->nop(&drvConfig);
  // put your main code here, to run repeatedly:
  switch (battery->state)
  {
    case STANDBY:
      bccError = BCC_Init(&drvConfig);
      if(bccError != bcc_status_t::BCC_STATUS_SUCCESS){
        battery->state = SHUTDOWN;
      }
      else battery->state = PRECHARGE;
      battery->init(&drvConfig, init_regs);
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
    
    
    battery->readDeviceMeasurements(&drvConfig);
    // battery->nop();
    battery->printDeviceMeasurements(&drvConfig);

    bccError = BCC_Sleep(&drvConfig);
    for (uint8_t i = 0; i < 5000000; i++);
    BCC_WakeUp(&drvConfig);
    
    #ifdef DEBUG
      if(millis() - prev_mill > 500){
        prev_mill = millis();
        debug();
      }
    #endif
  }
}