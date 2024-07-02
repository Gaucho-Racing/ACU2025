#ifndef BATTERY_H
#define BATTERY_H
#include <bcc.h>
#include <bcc_spi.h>
#include <Arduino.h>
#include <config.h>

class Battery
{
private:
    /* data */
    bcc_drv_config_t * drvConfig;
    uint16_t devConf[NUM_CELL_IC][BCC_INIT_CONF_REG_CNT];
    uint8_t dev_com_errors[15]; // <=> checkCommunication (comm errors), ignore 0th idx
    int bcc_errors[13]; // connect with bcc_fault_status_t (bcc errors) && bcc_status_t (error codes)
public:
    float cellVoltage[140];
    float cellTemp[140];
    float balTemp[140]; 
    float cellSOC[140];
    
    Battery();
    void shutDown();
    bool checkBattery();
    bool checkCellTemp();
    bool cellBalancing();
    bool checkCommunication();
    bool checkStatus(bcc_cid_t cid);
    bool measureCell(bcc_cid_t cluster, uint8_t cell_idx);
    void printErrorCode(bcc_status_t error,  bcc_cid_t cid);
    
};
#endif