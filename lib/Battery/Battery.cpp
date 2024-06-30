#include "Battery.h"

Battery::Battery()
{
    // setup drvConfig
    drvConfig->drvInstance = 0;
    drvConfig->commMode = bcc_mode_t::BCC_MODE_SPI;
    drvConfig->devicesCnt = 14;
    for(uint8_t i = 0; i < 14; i++){
        drvConfig->device[i] = bcc_device_t::BCC_DEVICE_MC33771;
    }
    for(uint8_t i = 0; i < 14; i++){
        drvConfig->cellCnt[i] = NUM_CELL_IC;
    }
    
    // init BCC
    bcc_status_t status = BCC_Init(drvConfig, devConf); 
    
    if(status != BCC_STATUS_SUCCESS){
        Serial.println("BCC Init failed");
        shutDown();
    }

    // normal mode to all devices
    BCC_WakeUp(drvConfig);

    // enable cell balancing
    for(int i = 0; i < (NUM_TOTAL_IC); i++){
        for (int j = 0; j < NUM_CELL_IC; j++){
            BCC_CB_SetIndividual(drvConfig, (bcc_cid_t)i, j, 1, CELL_BALANCE_TIMER);
            // BCC_CB_Enable(drvConfig, (bcc_cid_t)i, 1); --> not sure what the diff is btwn ^^
        }
    }
}

bool Battery::measureCell(bcc_cid_t cluster, uint8_t cell_idx){
    bcc_status_t status = BCC_STATUS_SUCCESS;
    if((status = BCC_CB_Pause(drvConfig, cluster, true)) != BCC_STATUS_SUCCESS){
        Serial.print('Error while pausing cell balancing');
        printErrorCode(status, cluster);
        return false;
    }
    // start measurement
    /** DO STUFF HERE */
    // end measurement

    if((status = BCC_CB_Pause(drvConfig, cluster, false)) != BCC_STATUS_SUCCESS){
        Serial.print('Error while resuming cell balancing');
        printErrorCode(status, cluster);
        return false;
    }
    return true;
}

/// @brief Checks the status of the battery
/// @param cid Cluster Identification Address
/// @return true if there are errors, false otherwise
bool Battery::checkStatus(bcc_cid_t cid){
    uint16_t status[14]; // status of all devices
    for(int i = 1; i <= 15; i++){
        bcc_status_t errors = BCC_Fault_GetStatus(drvConfig, cid,status);
        if(errors != BCC_STATUS_SUCCESS){
            bcc_errors[i] = 1;
            printErrorCode(errors, cid);
        }
    }
    return bcc_errors > 0;
}
/// @brief Shuts down
void Battery::shutDown(){
    // disable cell balancing for all cells
    for(int i = 0; i < 15; i++){
        BCC_CB_Enable(drvConfig, (bcc_cid_t)i, 0);
    }
}

/// @brief Checks the communication between the BCC and the MCU
/// @param drvConfig Pointer to driver instance configuration.
/// @return true if communication is successful, false otherwise
/// @note which device lost comm is stored in dev_com_errors
bool Battery::checkCommunication(){
    bool flag = false;
    for(uint8_t i = BCC_CID_DEV1; i < BCC_CID_DEV15; i++){
        bcc_status_t status = BCC_VerifyComSpi(drvConfig, (bcc_cid_t)i);
        if(status != BCC_STATUS_SUCCESS){
            dev_com_errors[i] = 1;
            flag = true;
        }
    }
    return flag;
}

bool Battery::checkBattery(){
    return true;
}

bool Battery::checkCellTemp(){
    for(int i = 1; i < 15; i++){
        // BCC_REG_CELL_OV_FLT_ADDR & CID
    }
    return false;
}

/// @brief Prints the error code givenstatus & cid
/// @param error 
/// @param cid 
void Battery::printErrorCode(bcc_status_t error, bcc_cid_t cid){
    switch (error)
    {
    case BCC_STATUS_SUCCESS:
        Serial.printf("Success for cid: %d\n", cid);
        break;
    case BCC_STATUS_SPI_INIT:
        Serial.printf("SPI Init Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_SPI_BUSY:
        Serial.printf("SPI Busy Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_PARAM_RANGE:
        Serial.printf("Parameter Range Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_CRC:
        Serial.printf("CRC Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_COM_TAG_ID:
        Serial.printf("Communication Tag ID Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_COM_RC:
        Serial.printf("Communication RC Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_COM_TIMEOUT:
        Serial.printf("Communication Timeout Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_DIAG_FAIL:
        Serial.printf("Diagnostic Failure Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_EEPROM_ERROR:
        Serial.printf("EEPROM Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_EEPROM_PRESENT:
        Serial.printf("EEPROM Present Error for cid: %d\n", cid);
        break;
    case BCC_STATUS_NULL_RESP:
        Serial.printf("Null Response Error for cid: %d\n", cid);
        break;
    default:
        Serial.printf("Unknown Error for cid: %d\n", cid);
        break;
    }
}

/* NOTES: 
// bcc_measurements_t measurements; // self-explanatory
// typedef enum
// {
//     BCC_FS_CELL_OV        = 0U,   /*!< CT overvoltage fault (register CELL_OV_FLT). */
//     BCC_FS_CELL_UV        = 1U,   /*!< CT undervoltage fault (register CELL_UV_FLT). */
//     BCC_FS_CB_OPEN        = 2U,   /*!< Open CB fault (register CB_OPEN_FLT). */
//     BCC_FS_CB_SHORT       = 3U,   /*!< Short CB fault (register CB_SHORT_FLT). */
//     BCC_FS_GPIO_STATUS    = 4U,   /*!< GPIO status (register GPIO_STS). */
//     BCC_FS_AN_OT_UT       = 5U,   /*!< AN undertemperature and overtemperature (register AN_OT_UT_FLT). */
//     BCC_FS_GPIO_SHORT     = 6U,   /*!< GPIO short and analog inputs open load detection
//                                        (register GPIO_SHORT_Anx_OPEN_STS). */
//     BCC_FS_COMM           = 7U,   /*!< Number of communication errors detected (register COM_STATUS). */
//     BCC_FS_FAULT1         = 8U,   /*!< Fault status (register FAULT1_STATUS). */
//     BCC_FS_FAULT2         = 9U,   /*!< Fault status (register FAULT2_STATUS). */
//     BCC_FS_FAULT3         = 10U   /*!< Fault status (register FAULT3_STATUS). 
// } bcc_fault_status_t; 


