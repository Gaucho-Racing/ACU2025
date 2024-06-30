#ifndef BATTERY_H
#define BATTERY_H
// #include <bcc_spi.h>
#include <Arduino.h>
#include <config.h>
#include <bcc_diagnostics.h>

#include <bcc_mc33771c.h>
#include "bcc_c.h"
#define D_PRINT(x) printf(x) // DEBUG MODE enabled
#define PRINT(x) Serial.println(x)

class Battery
{
private:
    /* data */
    bcc_drv_data_t * drvData; // Driver internal data
    uint16_t devConf[NUM_CELL_IC][INIT_REG_CNT];

    uint8_t transBuf[5];  // Adjust size as per your protocol
    uint8_t recvBuf[RX_BUF_SIZE];
    uint16_t recvTrCnt;

    uint8_t dev_com_errors[15]; // <=> checkCommunication (comm errors), ignore 0th idx
    int bcc_errors[13]; // connect with bcc_fault_status_t (bcc errors) && bcc_status_t (error codes)
public:
    bcc_drv_config_t * drvConfig; // Driver configuration
    float minCellVolt, maxCellVolt;
    float minChargeVolt, maxChargeVolt;
    float batVoltage, batSOC;

    uint32_t cellVoltage[NUM_CELL_IC * NUM_TOTAL_IC]; // V
    uint32_t stackVoltage[NUM_CELL_IC];
    float cellTemp[NUM_CELL_IC];
    bcc_fault_status_t faultStatus[NUM_CELL_IC];

    float balTemp[NUM_CELL_IC * NUM_TOTAL_IC]; 
    float cellSOC[NUM_CELL_IC * NUM_TOTAL_IC];
    
    States state;
    
    Battery();
    bcc_status_t init();
    bool resetDischarge();
    void cellBalancing(bool init = false, bool enable);
    
    
    // read data from bcc
    bcc_status_t readDeviceMeasurements();
    bcc_status_t measureCellVolts(byte cluster, byte cell);
    bcc_status_t measureCellTemp(byte cluster);
    bcc_status_t getTotalVoltage();

    // self checks
    bcc_status_t checkTemperature(bool fullCheck = false) const;
    bcc_status_t checkVoltage(bool fullCheck = false)const;
    bcc_status_t checkFuse(bool fullCheck = false) const;
    bool checkCommunication();
    bool checkStatus();

    // send data to others
    bool updateVoltage();
    bool updateTemp();
    float updateSOC();

    // debugging statements
    void printErrorCode(bcc_status_t error,  bcc_cid_t cid);
    
};

/*
static bcc_status_t updateMeasurements(void)
{
    bcc_status_t error;
    uint16_t measurements[BCC_MEAS_CNT]; /* Array needed to store all measured values.
    Step 1: Obtain raw measurements. 
    error = BCC_Meas_StartConversion(&drvConfig, BCC_CID_DEV1);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    do
    {
        error = BCC_Meas_IsConverting(&drvConfig, BCC_CID_DEV1, &completed);
        if (error != BCC_STATUS_SUCCESS)
        {
          return error;
        }
    } while (!completed);


    /* Step 2: Convert raw measurements to appropriate units. * /
    error = BCC_Meas_GetRawValues(&drvConfig, BCC_CID_DEV1, measurements);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* You can use bcc_measurements_t enumeration to index array with raw values. * /
    /* Useful macros can be found in bcc.h or bcc_MC3377x.h. * /

    /* Measured ISENSE in [uV]. * /
    isenseVoltageUV = BCC_GET_ISENSE_VOLT(measurements[BCC_MSR_ISENSE1], measurements[BCC_MSR_ISENSE2]);
    /* Measured ISENSE in [mA]. Value of shunt resistor is used. * /
    isenseCurrentMA = BCC_GET_ISENSE_AMP(DEMO_RSHUNT, measurements[BCC_MSR_ISENSE1],
            measurements[BCC_MSR_ISENSE2]);
    /* Stack voltage in [uV]. * /
    stackVoltageUV = BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]);
    /* Cells voltage in [uV]. * /
    cell1VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1]);
    cell2VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT2]);
    cell3VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT3]);
    cell4VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT4]);
    cell5VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT5]);
    cell6VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT6]);
    cell7VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT7]);
    cell8VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT8]);
    cell9VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT9]);
    cell10VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT10]);
    cell11VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT11]);
    cell12VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT12]);
    cell13VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT13]);
    cell14VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT14]);

    /* For calculation of temperature on GPIO[0-6] in analog mode with use of NTC resistor. * /
    /* To measure temperature selected GPIOs have to be set to analog input AM or RM mode! * /
    error = getNtcCelsius(measurements[BCC_MSR_AN0], &an0TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN1], &an1TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN2], &an2TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN3], &an3TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN4], &an4TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN5], &an5TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN6], &an6TempDegC);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* IC temperature measurement in degC (register MEAS_IC_TEMP). * /
    icTempDegC = (int16_t)(BCC_GET_IC_TEMP(measurements[BCC_MSR_ICTEMP]));

    /* ADCIA Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1A). * /
    vBGADC1AVoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1A]);
    /* ADCIB Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1B). * /
    vBGADC1BVoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1B]);

    return BCC_STATUS_SUCCESS;
}
*/
#endif