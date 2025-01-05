#include "Battery.h"
#include "bcc.h"
#include "bcc_communication.h"
#include "bcc_diagnostics.h"
#include "debug.h"

void clear_faults(bcc_drv_config_t * drvConfig)
{
    bcc_status_t * status = new bcc_status_t();
    for (uint8_t ic = 1; ic <= NUM_CELL_IC; ic++)
    {
        bcc_cid_t cid = (bcc_cid_t)ic;
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_CELL_OV);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_CELL_UV);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_CB_OPEN);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_CB_SHORT);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_GPIO_STATUS);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_AN_OT_UT);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_GPIO_SHORT);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_COMM);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_FAULT1);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_FAULT2);
        *status = BCC_Fault_ClearStatus(drvConfig, cid, bcc_fault_status_t::BCC_FS_FAULT3);
        if(*status != BCC_STATUS_SUCCESS) return;
    }
}

Battery::Battery(){}

void Battery::toggleCellBalancing(bool enable){
    //turn on/off balancing
    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){
        bcc_cid_t cid = bcc_cid_t(i+1);
        if(enable) {
            bcc_status_t status = BCC_CB_Enable(&drvConfig, cid, 1);
            Serial.print("Enable cell balancing: "); print_bcc_status(status);
        }
        else {
            bcc_status_t status = BCC_CB_Pause(&drvConfig, cid, 1);
            Serial.print("Disable cell balancing: "); print_bcc_status(status);
        }
    }
}

void Battery::init(){

    // init BCC, TPL, Regs
    bcc_status_t errors = BCC_STATUS_SUCCESS;

    this->init_registers();
    clear_faults(&drvConfig);

    // disable cell balancing first
    toggleCellBalancing(0);

    readDeviceMeasurements();
    BCC_MCU_WaitUs(500);

    Serial.printf("cell_OV_Threshold: %5.03f, cell_UV_Threshold: %5.03f\n", CELL_MAX_VOLT, CELL_MIN_VOLT);
    if(errors != BCC_STATUS_SUCCESS){
        // state = SHUTDOWN;
        return;
    }

    // diagnose cell voltages
    checkVoltage();
    if(errors != bcc_status_t::BCC_STATUS_SUCCESS){
        // state = SHUTDOWN;
        return;
    }

    // diagnose cell temp
    // Serial.printf("cell_OT_Threshold: %5.03f, cell_UT_Threshold: %5.03f\n", CELL_MAX_TEMP, CELL_MIN_TEMP);
    checkTemperature();

    if(errors != bcc_status_t::BCC_STATUS_SUCCESS){
        // state = SHUTDOWN;
        return;
    }

    // enable cell balancing
    //toggleCellBalancing(1);
    return;
}


void Battery::readDeviceMeasurements() {
    
    uint32_t measurements[NUM_CELL_IC];
    int16_t temp_measures[NUM_CELL_IC];
    // int16_t temps[1];
    bcc_status_t error;

    //toggleCellBalancing(0);
    bzero(measurements, NUM_CELL_IC);
    // bzero(temps, 1);

    for(uint8_t i = 0; i < NUM_TOTAL_IC; i++){

        if(true){ // get cell & stack voltage
            //Serial.print("GetCellVoltage: "); 
            BCC_Meas_StartAndWait(&drvConfig, bcc_cid_t(i+1), BCC_AVG_1);
            error = BCC_Meas_GetCellVoltages(&drvConfig, bcc_cid_t(i+1), measurements);
            if(error != BCC_STATUS_SUCCESS){
                // this->state = SHUTDOWN;
                //toggleCellBalancing(&drvConfig);
                return;
            }
            //print_bcc_status(error);

            for(uint8_t j = 0; j < NUM_CELL_IC; j++){
                this->cellVoltage[i*NUM_CELL_IC + j] = measurements[j] * 1e-6f;
            }

            //Serial.print("GetStackVoltage: "); 
            bzero(measurements, NUM_TOTAL_IC);
            error = BCC_Meas_GetStackVoltage(&drvConfig, bcc_cid_t(i+1), measurements);
            if(error != BCC_STATUS_SUCCESS){
                // this->state = SHUTDOWN;
                //toggleCellBalancing(&drvConfig);
                return;
            }
            //print_bcc_status(error);
            //this->stackVoltage[i] = measurements[0];
        }

        if(true){ // get IC temperature
            //Serial.print("GetIcTemperature: "); 
            bzero(temp_measures, NUM_CELL_IC);
            error = BCC_Meas_GetIcTemperature(&drvConfig, bcc_cid_t(i+1), BCC_TEMP_CELSIUS, temp_measures);
            //print_bcc_status(error);
            this->icTemp[i] = temp_measures[i] * 0.1f; // when printing make sure to multiply by 0.1
            // bzero(temps, 1);
        }

        if(true){
            Serial.print("GetCellTemperature: ");
            for(uint8_t j = 0; j < 32; j++) { // TODO: fix this loop
                uint8_t readByte;
                error = BCC_EEPROM_Read(&drvConfig, bcc_cid_t(i+1), j+1, &readByte);
                Serial.print(readByte); print_bcc_status(error);
                if (error == BCC_STATUS_SUCCESS) this->cellTemp[i*NUM_CELL_IC + j] = (float)readByte;
            }
        }
    }
    //toggleCellBalancing(0);
    return;
}

void Battery::checkTemperature() {
    for(int i = 0; i < NUM_TOTAL_IC; i++){
        for(int j = 0; j < (NUM_CELL_IC); i++){
            if(this->cellTemp[i*NUM_CELL_IC + j] > CELL_MAX_TEMP){
                Serial.println("OT detected");
                return;
            }
            if(this->cellTemp[i*NUM_CELL_IC + j] < CELL_MIN_TEMP){
                Serial.println("UT detected");
                return;
            }
        }
    }
}

void Battery::checkVoltage() {
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
}

bool Battery::checkStatus() {

    uint16_t* status = nullptr; // status of all devices
    bool faults = false;
    for(int i = 0; i <= NUM_TOTAL_IC; i++){

        bcc_status_t error = BCC_Fault_GetStatus(&drvConfig, (bcc_cid_t)i, status);        
        if(error != BCC_STATUS_SUCCESS){
            faults = true;
        }
    }
    return faults;
}

void Battery::printDeviceMeasurements() {
    for(uint8_t cell = 0; cell < NUM_CELL_IC * NUM_TOTAL_IC; cell++){
        if(cell % NUM_CELL_IC == 0){
            Serial.printf("\nSegment %u: ", cell / NUM_CELL_IC);
        }
        Serial.printf("%d:%5.03f ", cell, this->cellVoltage[cell]);
    }

    // for(uint8_t cell = 0; cell < NUM_CELL_IC; cell++){
    //      Serial.printf("Stack Voltage [Cell %d: %5.03f", cell, this->stackVoltage[cell]);
    // }

    // for(uint8_t cell = 0; cell < NUM_CELL_IC; cell++){
    //      Serial.printf("Cell Temp [Cell %d: %5.03f", cell, this->cellTemp[cell]);
    // }
}

bool Battery::system_check(bool startup){
    this->readDeviceMeasurements();
    return true;
}

void Battery::init_registers()
{
    uint8_t cid, i;
    bcc_status_t status;

    for (cid = 1; cid <= drvConfig.devicesCnt; cid++)
    {
        for (i = 0; i < INIT_REG_CNT; i++)
        {
            if (init_regs[i].value != init_regs[i].defaultVal)
            {
                status = BCC_Reg_Write(&drvConfig, (bcc_cid_t)cid,
                        init_regs[i].address, init_regs[i].value);
                    if(status != BCC_STATUS_SUCCESS) return;
            }
        }
    }
}