#include "debug.h"
#include "Battery.h"
#include "config.h"
#include <Arduino.h>
// Key Test Functions for BCC

// Voltage Measurement
/************** INFO **************
 * Continuously monitors the voltage of each cell
 * to ensure they are within safe operating limits.
 * Test: All cell voltages are within safe limits
 **********************************/
int voltage_measurement_test(Battery *bty){
    float_t min_volt = __FLT_MAX__, max_volt = __FLT_MIN__;
    Serial.println("Cell Voltage: --------------------------");
    for(int i = 0; i < (NUM_TOTAL_IC); i++){
        
        Serial.printf("Row %d => ", i);
        for (int j = 0; j < NUM_CELL_IC; j++){
            
            Serial.printf("[Cell %d: ", j);
            min_volt = min(min_volt, bty->cellVoltage[i]);
            max_volt = max(max_volt, bty->cellVoltage[i]);
            if(bty->cellVoltage[i] < PRM_CELL_MIN_VOLT){
                Serial.printf("%5.03f | Under] ", bty->cellVoltage[i]);

            } else if (bty->cellVoltage[i] > PRM_CELL_MAX_VOLT){
                Serial.printf("%5.03f | Over] ", bty->cellVoltage[i]);
            }
            else{
                Serial.printf("%5.03f] ", bty->cellVoltage[i]);
            }
        }
        Serial.println();
        Serial.printf("Min Volt: %5.03f | Max Volt: %5.03f\n", min_volt, max_volt);
    }
    Serial.println("-----------------------------------------");
    return 1;
}

// Temperature Measurement
/************** INFO **************
 * Measures the temperature of each cell or the entire
 * battery pack to prevent overheating.
 **********************************/
int temperature_measurement_test(Battery *bty){
    float_t min_temp = __FLT_MAX__, max_temp = __FLT_MIN__;
    Serial.println("Cell Temp: ------------------------------");
    for(int i = 0; i < (NUM_TOTAL_IC); i++){
        
        Serial.printf("Row %d => ", i);
        for (int j = 0; j < NUM_CELL_IC; j++){
            
            Serial.printf("[Cell %d: ", j);
            min_temp = min(min_temp, bty->cellTemp[i]);
            max_temp = max(max_temp, bty->cellTemp[i]);

            if(bty->cellTemp[i] < CELL_MIN_TEMP){
                Serial.printf("%5.01f | Under] ", bty->cellTemp[i] * 0.1);

            } else if (bty->cellTemp[i] > CELL_MAX_TEMP){
                Serial.printf("%5.01f | Over] ", bty->cellTemp[i] * 0.1);
            }
            else{
                Serial.printf("%5.01f] ", bty->cellTemp[i]);
            }
        }
        Serial.println();
        Serial.printf("Min temp: %5.01f | Max temp: %5.01f\n", min_temp, max_temp);
    }
    Serial.println("-----------------------------------------");
    return 1;
}

// Ball Temp Measurement
/************** INFO **************
 * Measures the ball temp
 * Test: All cell voltages are within safe limits
 **********************************/
int ball_temp_measurement_test(Battery * bty){
    float_t min_temp = __FLT_MAX__, max_temp = __FLT_MIN__;
    Serial.println("Ball Temp: ------------------------------");
    for(int i = 0; i < (NUM_TOTAL_IC); i++){
        
        Serial.printf("Row %d => ", i);
        for (int j = 0; j < NUM_CELL_IC; j++){
            
            Serial.printf("[Cell %d: ", j);
            min_temp = min(min_temp, bty->balTemp[i]);
            max_temp = max(max_temp, bty->balTemp[i]);

            if(bty->balTemp[i] < MIN_BALL_TEMP){
                Serial.printf("%5.01f | Under] ", bty->balTemp[i]);

            } else if (bty->balTemp[i] > MAX_BALL_TEMP){
                Serial.printf("%5.01f | Over] ", bty->balTemp[i]);
            }
            else{
                Serial.printf("%5.01f] ", bty->balTemp[i]);
            }
        }
        Serial.println();
        Serial.printf("Min ball_temp: %5.01f | Max ball_temp: %5.01f\n", min_temp, max_temp);
    }
    Serial.println("-----------------------------------------");
    return 1;
}

// Cell Balancing Test
/************** INFO **************
 * Turns first cell's cell balancing on then off
 * Test: Don't wanna turn too many off
 **********************************/
int turn_single_cell_balancing_on(Battery * bty){

    Serial.println("Cell Balancing Test: ------------------------------");
    
    uint8_t count = 50;
    bty->toggleCellBalancing(false, true, BCC_CID_DEV1, 0);
    Serial.println("Turn on cell balancing for one cell");

    while(count > 0){
        uint8_t value = bty->cellBalancing[0];
        bty->readDeviceMeasurements();
        if(value == 0){
            Serial.printf("[Cell %d | Off] ", value);

        } else if (value == 255){
            Serial.printf("[Cell %d | On] ", value);
        }
        else{
            Serial.printf("[Cell %d | Invalid] ", value);
        }
        count--;
        Serial.printf("Voltage: %5.03f, Temp: %5.03f\n", bty->cellVoltage[0], bty->cellTemp[0]);
        delay(1000);
    }

    bty->toggleCellBalancing(false, false, BCC_CID_DEV1, 0);
    Serial.println("Turn off cell balancing for one cell");

    count = 50;
    while(count > 0){
        bty->readDeviceMeasurements();
        uint8_t value = bty->cellBalancing[0];
        if(value == 0){
            Serial.printf("[Cell %d | Off] ", value);

        } else if (value == 255){
            Serial.printf("[Cell %d | On] ", value);
        }
        else{
            Serial.printf("[Cell %d | Invalid] ", value);
        }
        count--;
        Serial.printf("Voltage: %5.03f, Temp: %5.03f\n", bty->cellVoltage[0], bty->cellTemp[0]);
        delay(1000);
    }

    
    Serial.println("Cell Balancing Test Done: ------------------------------");
    return 1;
}

int debug(Battery *bty){
    voltage_measurement_test(bty);
    temperature_measurement_test(bty);
    turn_single_cell_balancing_on(bty);
    return 0;
}

    // BCC_STATUS_SUCCESS        = 0U,   /*!< No error. */
    // BCC_STATUS_PARAM_RANGE    = 1U,   /*!< Parameter out of range. */
    // BCC_STATUS_SPI_FAIL       = 2U,   /*!< Fail in the SPI communication. */
    // BCC_STATUS_COM_TIMEOUT    = 3U,   /*!< Communication timeout. */
    // BCC_STATUS_COM_ECHO       = 4U,   /*!< Received "echo" frame from MC33664 does not correspond
    //                                        to the sent frame. */
    // BCC_STATUS_COM_CRC        = 5U,   /*!< Wrong CRC in the received SPI frame. */
    // BCC_STATUS_COM_MSG_CNT    = 6U,   /*!< Received frame has a valid CRC but the message counter
    //                                        value does not match to the expected one. */
    // BCC_STATUS_COM_NULL       = 7U,   /*!< Received frame has a valid CRC but all bit-fields
    //                                        except CRC and message counter are zero. This occurs only
    //                                        in SPI communication mode: during the very first message
    //                                        or as a response to an invalid request from MCU. */
    // BCC_STATUS_DIAG_FAIL      = 8U,   /*!< It is not allowed to enter diagnostic mode. */
    // BCC_STATUS_EEPROM_ERROR   = 9U,   /*!< An error occurred during the communication to EEPROM. */
    // BCC_STATUS_EEPROM_PRESENT = 10U,  /*!< No EEPROM detected. */
    // BCC_STATUS_DATA_RDY       = 11U,  /*!< A new sequence of conversions is currently running. */
    // BCC_STATUS_TIMEOUT_START  = 12U   /*!< An error reported in BCC_MCU_StartTimeout function. */
void print_bcc_status(bcc_status_t bccStatus){
    switch (bccStatus)
    {
    case BCC_STATUS_SUCCESS:
        Serial.print("Success\n");
        break;
    case BCC_STATUS_PARAM_RANGE:
        Serial.print("Parameter out of range\n");
        break;
    case BCC_STATUS_SPI_FAIL:
        Serial.print("SPI failed\n");
        break;
    case BCC_STATUS_COM_TIMEOUT:
        Serial.print("communication timeout\n");
        break;
    case BCC_STATUS_COM_ECHO:
        Serial.print("Echo frame doesn't correspond to sent frame\n");
        break;
    case BCC_STATUS_COM_CRC:
        Serial.print("CRC error\n");
        break;
    case BCC_STATUS_COM_MSG_CNT:
        Serial.print("Message counter mismatch\n");
        break;
    case BCC_STATUS_COM_NULL:
        Serial.print("NULL message\n");
        break;
    case BCC_STATUS_DIAG_FAIL:
        Serial.print("Diagnoctic mode not allowed\n");
        break;
    case BCC_STATUS_EEPROM_ERROR:
        Serial.print("EEPROM communication error\n");
        break;
    case BCC_STATUS_EEPROM_PRESENT:
        Serial.print("EEPROM device not detected\n");
        break;
    case BCC_STATUS_DATA_RDY:
        Serial.print("New convertion already running\n");
        break;
    case BCC_STATUS_TIMEOUT_START:
        Serial.print("BCC_MCU_StartTimeout function error\n");
    default:
        Serial.print("Unknown status\n");
        break;
    }
}

// Cell Balancing - Active
/************** INFO **************
 * Actively redistributes charge among
 * cells to ensure uniform charge levels.
 **********************************/  

// Cell Balancing - Passive
/************** INFO **************
 * Uses resistors to dissipate 
 * excess energy from overcharged cells.
 **********************************/ 

// Cell Balancing - Passive
/************** INFO **************
 * Uses resistors to dissipate 
 * excess energy from overcharged cells.
 **********************************/ 

// Overcharge Protection
/************** INFO **************
 * Prevents cells from being charged 
 * beyond their maximum voltage.
 **********************************/ 

// Overdischarge Protection
/************** INFO **************
 * Prevents cells from being discharged 
 * below their minimum voltage.
 **********************************/

// OverTemp Protection
/************** INFO **************
 * Shuts down or limits operation if cells 
 * exceed safe temperature ranges.
 **********************************/

// Short Circuit Protection
/************** INFO **************
 * Detects and responds to short circuits 
 * to prevent damage and hazards.
 **********************************/

// Data Transmission
/************** INFO **************
 * Sends data on cell voltages, temps, etc
 * to a central BMS or external devices.
 * Commands: RCV cmds from BMS (balancing, etc)
 **********************************/