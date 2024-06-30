#include "Battery.h"
#pragma once
Battery::Battery(){}

void Battery::toggleCellBalancing(bcc_drv_config_t* const drvConfig, byte enable){
    Serial.println("Setting Cell Balancing");

    //turn on/off balancing
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        if(true) BCC_CB_Enable(drvConfig, bcc_cid_t(i), enable);
        else BCC_CB_Pause(drvConfig, bcc_cid_t(i), enable);
    }

    Serial.println("Successfully updated Cell Balancing Settings");
}

void Battery::init(bcc_drv_config_t* const drvConfig, const bcc_init_reg_t * init_regs){

    this->options = new Type(true, true, true, true);
    this->bccInitialized = false;

    // init BCC, TPL, Regs
    bcc_status_t errors;

    this->init_registers(drvConfig, init_regs);
    if (errors != BCC_STATUS_SUCCESS) return;

    this->clear_faults(drvConfig);
    if(errors != BCC_STATUS_SUCCESS) return;

    this->bccInitialized = true;

    // disable cell balancing first
    toggleCellBalancing(drvConfig, 1);

    readDeviceMeasurements(drvConfig);
    BCC_MCU_WaitUs(500);

    Serial.printf("cell_OV_Threshold: %5.03f, cell_UV_Threshold: %5.03f\n", this->maxCellVolt, this->minCellVolt);
    if(errors != BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
        return;
    }

    // diagnose cell voltages
    checkVoltage(drvConfig);
    if(errors != bcc_status_t::BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
        return;
    }

    // diagnose cell temp
    Serial.printf("cell_OT_Threshold: %5.03f, cell_UT_Threshold: %5.03f\n", CELL_MAX_TEMP, CELL_MIN_TEMP);
    checkTemperature(drvConfig);

    if(errors != bcc_status_t::BCC_STATUS_SUCCESS){
        state = SHUTDOWN;
        return;
    }

    // enable cell balancing
    toggleCellBalancing(drvConfig, 1);
    return;
}


void Battery::readDeviceMeasurements(bcc_drv_config_t* const drvConfig) {
    
    uint32_t measurements[NUM_TOTAL_IC];
    int16_t temps[1];
    bcc_status_t error;

    toggleCellBalancing(drvConfig, 0);
    bzero(measurements, NUM_TOTAL_IC);
    bzero(temps, 1);
    // Get Cell/Stack Voltages in Volts & IC temps in Celcius
    for(uint8_t i = 0; i < NUM_CELL_IC; i++){

        if(this->options->voltage){
            error = BCC_Meas_GetCellVoltages(drvConfig, bcc_cid_t(i), measurements);
            BCC_MCU_WaitUs(500);

            if(error != BCC_STATUS_SUCCESS){
                this->state = SHUTDOWN;
                toggleCellBalancing(drvConfig, 1);
                return;
            }

            for(uint8_t j = 0; j < NUM_TOTAL_IC; j++){
                this->cellVoltage[i*NUM_TOTAL_IC + j] = measurements[j] * 0.001;
            }
            bzero(measurements, NUM_TOTAL_IC);

            error = BCC_Meas_GetStackVoltage(drvConfig, bcc_cid_t(i), measurements);

            BCC_MCU_WaitUs(500);
            if(error != BCC_STATUS_SUCCESS){
                this->state = SHUTDOWN;
                toggleCellBalancing(drvConfig, 1);
                return;
            }
            this->stackVoltage[i] = (*measurements) * 0.001;
            bzero(measurements, 1);
        }

        if(this->options->temp){
            error = BCC_Meas_GetIcTemperature(drvConfig, bcc_cid_t(i), bcc_temp_unit_t::BCC_TEMP_CELSIUS, temps);
            BCC_MCU_WaitUs(500);
            this->cellTemp[i] = *temps;
            bzero(temps, 1);
        }

        if(this->options->fuse){

        }
    }
    toggleCellBalancing(drvConfig, 0);
    return;
}

void Battery::checkTemperature(bcc_drv_config_t* const drvConfig) {
    for(int i = 0; i < (NUM_CELL_IC); i++){
        
        if(this->cellTemp[i] > CELL_MAX_TEMP){
            Serial.println("OT detected");
            return;
        }
        if(this->cellTemp[i]){
            Serial.println("UT detected");
            return;
        }
    }
}

void Battery::checkVoltage(bcc_drv_config_t* const drvConfig) {
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < NUM_CELL_IC; j++){
            if(this->cellVoltage[i*NUM_CELL_IC + j] > PRM_CELL_MAX_VOLT){
                Serial.println("OV failure");
                // do sth
            }
            else if(this->cellVoltage[i*NUM_CELL_IC + j] < PRM_CELL_MIN_VOLT){
                Serial.println("UV failure");
                // do sth
            }
        }
    }
}

bool Battery::checkStatus(bcc_drv_config_t* const drvConfig) {

    uint16_t* status = nullptr; // status of all devices
    bool faults = false;
    for(int i = 0; i <= NUM_TOTAL_IC; i++){

        bcc_status_t error = BCC_Fault_GetStatus(drvConfig, (bcc_cid_t)i, status);
        this->faultStatus[i] = NONE;
        
        if(error != BCC_STATUS_SUCCESS){
            bcc_errors[i] = 1;
        }
        if(*status != BCC_STATUS_SUCCESS) faults = true;
    }
    return faults;
}

void Battery::printDeviceMeasurements(bcc_drv_config_t* const drvConfig) {
    for(uint8_t cell = 0; cell < NUM_CELL_IC * NUM_TOTAL_IC; cell++){
        if(cell % NUM_CELL_IC == 0){
            Serial.println('\n');
        }
        Serial.printf("CELL[Cell %d: %5.03f", cell, this->cellVoltage[cell]);
    }

    for(uint8_t cell = 0; cell < NUM_CELL_IC; cell++){
         Serial.printf("Stack Voltage [Cell %d: %5.03f", cell, this->stackVoltage[cell]);
    }

    for(uint8_t cell = 0; cell < NUM_CELL_IC; cell++){
         Serial.printf("Cell Temp [Cell %d: %5.03f", cell, this->cellTemp[cell]);
    }
}

bool Battery::system_check(bcc_drv_config_t* const drvConfig, bool startup){
    this->options->fuse = true;
    this->options->temp = true;
    this->options->voltage = true;
    this->readDeviceMeasurements(drvConfig);
    return true;
}

void Battery::nop(bcc_drv_config_t* const drvConfig) {
    for (uint8_t cid = 1; cid <= drvConfig->devicesCnt; cid++) 
        BCC_SendNop(drvConfig, (bcc_cid_t)(cid));
}

void Battery::init_registers(bcc_drv_config_t* const drvConfig, const bcc_init_reg_t* init_regs)
{
    uint8_t cid, i;
    bcc_status_t status;

    for (cid = 1; cid <= drvConfig->devicesCnt; cid++)
    {
        for (i = 0; i < INIT_REG_CNT; i++)
        {
            if (init_regs[i].value != init_regs[i].defaultVal)
            {
                status = BCC_Reg_Write(drvConfig, (bcc_cid_t)cid,
                        init_regs[i].address, init_regs[i].value);
                    if(status != BCC_STATUS_SUCCESS) return;
            }
        }
    }
}

void Battery::clear_faults(bcc_drv_config_t* const drvConfig)
{
    bcc_status_t status = bcc_status_t::BCC_STATUS_SUCCESS;
    uint8_t cid;

    for (cid = 1; cid <= NUM_CELL_IC; cid++)
    {
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_OV);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_UV);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_COMM);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT1);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT2);
        status = BCC_Fault_ClearStatus(drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT3);
    }
    if(status != BCC_STATUS_SUCCESS) return;
}
