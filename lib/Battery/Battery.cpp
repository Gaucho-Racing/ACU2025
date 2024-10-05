#include "Battery.h"
#include <bcc_tpl.h>

Battery::Battery(){ }

bcc_status_t Battery::init(){
    // setup drvConfig
    drvConfig->drvInstance = 0U;
    drvConfig->devicesCnt = NUM_TOTAL_IC;

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        drvConfig->device[i] = DEVICE;
        drvConfig->cellCnt[i] = NUM_CELL_IC;
    }
    drvConfig->commMode = MODE;

    // init BCC, TPL, Regs
    bcc_status_t errors = BCC_Init(drvConfig); 
    
    if(errors != BCC_STATUS_SUCCESS){
        Serial.println("BCC Init failed");
        state = SHUTDOWN;
        return errors;
    }
    // disable cell balancing first
    cellBalancing(true, false);

    bcc_status_t errors = readDeviceMeasurements();
    BCC_MCU_WaitUs(500);

    Serial.printf("cell_OV_Threshold: %5.03f, cell_UV_Threshold: %5.03f\n", this->maxCellVolt, this->minCellVolt);
    if(errors != BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
        return errors;
    }

    // diagnose cell voltages
    errors = checkVoltage();
    if(errors != bcc_status_t::BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
        return errors;
    }

    // diagnose cell temp
    Serial.printf("cell_OT_Threshold: %5.03f, cell_UT_Threshold: %5.03f\n", CELL_MAX_TEMP, CELL_MIN_TEMP);
    errors = checkTemperature();

    if(errors != bcc_status_t::BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
        return errors;
    }

    // enable cell balancing
    cellBalancing(true, true);
    return bcc_status_t::BCC_STATUS_SUCCESS;
}

// setting to change cellBalancing of all cells
void Battery::cellBalancing(bool init = false, bool enable){
    PRINT("Setting Cell Balancing");

    //turn on/off balancing
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        if(init == true) BCC_CB_Enable(drvConfig, bcc_cid_t(i), enable);
        else BCC_CB_Pause(drvConfig, bcc_cid_t(i), enable);
    }
}

// Measures stuff, no checs yet
bcc_status_t Battery::readDeviceMeasurements() {
    
    uint32_t * measurements;
    int16_t * temps;
    bcc_status_t error;
    
    // Get Cell/Stack Voltages in Volts & IC temps in Celcius
    for(uint8_t i = 0; i < NUM_CELL_IC; i++){

        error = BCC_Meas_GetCellVoltages(this->drvConfig, bcc_cid_t(i), measurements);
        BCC_MCU_WaitUs(500);

        if(error != BCC_STATUS_SUCCESS){
            this->state = SHUTDOWN;
            return error;
        }

        for(uint8_t j = 0; j < NUM_TOTAL_IC; j++){
            this->cellVoltage[i*NUM_TOTAL_IC + j] = measurements[j] * 0.001;
        }
        bzero(measurements, NUM_TOTAL_IC);

        error = BCC_Meas_GetStackVoltage(this->drvConfig, bcc_cid_t(i), measurements);
        BCC_MCU_WaitUs(500);
        if(error != BCC_STATUS_SUCCESS){
            this->state = SHUTDOWN;
            return error;
        }
        this->stackVoltage[i] = (*measurements) * 0.001;
        bzero(measurements, 1);

        error = BCC_Meas_GetIcTemperature(this->drvConfig, bcc_cid_t(i), bcc_temp_unit_t::BCC_TEMP_CELSIUS, temps);
        BCC_MCU_WaitUs(500);
        this->cellTemp[i] = *temps;
        bzero(temps, 1);

    }
    return BCC_STATUS_SUCCESS;
}

bcc_status_t Battery::checkVoltage(bool fullCheck = false) const {
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < NUM_CELL_IC; j++){
            if(this->cellVoltage[i*NUM_CELL_IC + j] > CELL_MAX_VOLT){
                Serial.println("OV failure");
                // do sth
            }
            else if(this->cellVoltage[i*NUM_CELL_IC + j] < CELL_MIN_VOLT){
                Serial.println("UV failure");
                // do sth
            }
        }
    }
    return bcc_status_t::BCC_STATUS_SUCCESS;
}

bcc_status_t Battery::checkTemperature(bool fullCheck = false) const {
    for(int i = 0; i < (NUM_CELL_IC); i++){
        
        if(this->cellTemp[i] > CELL_MAX_TEMP){
            D_PRINT("OT detected");
            return bcc_status_t::BCC_STATUS_PARAM_RANGE;
        }
        if(this->cellTemp[i]){
            D_PRINT("UT detected");
            return bcc_status_t::BCC_STATUS_PARAM_RANGE;
        }
    }
    return bcc_status_t::BCC_STATUS_SUCCESS;
}

bool Battery::checkStatus() {

    uint16_t* status; // status of all devices
    bool faults = false;
    for(int i = 0; i <= NUM_TOTAL_IC; i++){

        bcc_status_t error = BCC_Fault_GetStatus(drvConfig, (bcc_cid_t)i, status);
        this->faultStatus[i] = bcc_fault_status_t(*status);
        
        if(error != BCC_STATUS_SUCCESS){
            bcc_errors[i] = 1;
            return error;
        }
        if(*status != NULL) faults = true;
    }
    return faults;
}

bool Battery::checkCommunication(){
    bool flag = false;
    for(uint8_t i = BCC_CID_DEV1; i < BCC_CID_DEV15; i++){
        bcc_status_t status = BCC_VerifyComTpl(drvConfig, (bcc_cid_t)i);
        if(status != BCC_STATUS_SUCCESS){
            dev_com_errors[i] = 1;
            flag = true;
        }
    }
    return flag;
}
