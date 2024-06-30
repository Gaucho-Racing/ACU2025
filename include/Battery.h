#ifndef BATTERY_H
#define BATTERY_H
#pragma once
#include <Arduino.h>
#include "config.h"
#include "bcc.h"
#include "states.h"
#include "pins.h"

struct Type {
    bool voltage;
    bool temp;
    bool fuse;
    bool soc;
    Type(bool a, bool b, bool c, bool d): voltage(a), temp(b), fuse(c), soc(d){}
};

typedef struct
{
    const uint8_t address;
    const uint16_t defaultVal;
    const float value;
} bcc_init_reg_t;


enum bcc_status
{
    INIT_ERROR,
    UV_ERROR,
    OV_ERROR,
    NONE,
};

class Battery
{
private:
    uint16_t devConf[NUM_CELL_IC][INIT_REG_CNT];
    uint8_t dev_com_errors[15]; // <=> checkCommunication (comm errors), ignore 0th idx
    int bcc_errors[13]; // connect with bcc_fault_status_t (bcc errors) && bcc_status_t (error codes)
public:
    float minCellVolt, maxCellVolt;
    float minChargeVolt, maxChargeVolt;

    float batVoltage, batSOC;
    float max_output_current;

    Type * options;
    uint32_t cellVoltage[NUM_CELL_IC * NUM_TOTAL_IC]; // V
    uint32_t stackVoltage[NUM_CELL_IC];
    bcc_status faultStatus[NUM_CELL_IC];
    float cellTemp[NUM_CELL_IC];

    float balTemp[NUM_CELL_IC * NUM_TOTAL_IC]; 
    float cellSOC[NUM_CELL_IC * NUM_TOTAL_IC];
    bool bccInitialized;
    
    States state;
    
    Battery();
    void init(bcc_drv_config_t* const drvConfig, const bcc_init_reg_t * init_regs);
    
    // big functions
    bool system_check(bcc_drv_config_t* const drvConfig, bool startup);
    void check_acu(bcc_drv_config_t* const drvConfig);
    void check_battery(bcc_drv_config_t* const drvConfig);
    
    // read data from bcc
    void readDeviceMeasurements(bcc_drv_config_t* const drvConfig);

    // self checks
    void checkTemperature(bcc_drv_config_t* const drvConfig);
    void checkVoltage(bcc_drv_config_t* const drvConfig);
    void checkFuse(bcc_drv_config_t* const drvConfig);
    bool checkStatus(bcc_drv_config_t* const drvConfig);

    // debugging statements
    void nop(bcc_drv_config_t* const drvConfig);

    void toggleCellBalancing(bcc_drv_config_t* const drvConfig, byte enable);
    void printDeviceMeasurements(bcc_drv_config_t* const drvConfig);
    void init_registers(bcc_drv_config_t* const drvConfig, const bcc_init_reg_t* init_regs);
    void clear_faults(bcc_drv_config_t* const drvConfig);
};

#endif