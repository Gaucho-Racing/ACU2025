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
                Serial.printf("%5.01f | Under] ", bty->cellTemp[i]);

            } else if (bty->cellTemp[i] > CELL_MAX_TEMP){
                Serial.printf("%5.01f | Over] ", bty->cellTemp[i]);
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

int debug(Battery *bty){
    voltage_measurement_test(bty);
    temperature_measurement_test(bty);
    return 0;
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