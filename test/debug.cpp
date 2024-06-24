#include "main.cpp"
#include "Battery.h"
#include "config.h"

Battery* bty;

bool init_battery_test(){
    bty = new Battery();
    for(int i = 0; i < 140; i++){ // zero-init
        bty->cellVoltage[i] = 0;
        bty->cellTemp[i] = 0;
        bty->balTemp[i] = 0;
        bty->cellSOC[i] = 0;
    }
    return true;
}

int debug(){
    init_battery_test();
    voltage_measurement_test();
    return 0;
}

// Key Test Functions for BCC

// Voltage Measurement
/************** INFO **************
 * Continuously monitors the voltage of each cell
 * to ensure they are within safe operating limits.

 * Test: All cell voltages are within safe limits
 **********************************/
int voltage_measurement_test(){
    Serial.println("Cell Voltage: --------------------------");
    for(int i = 0; i < (NUM_TOTAL_IC * NUM_CELL_IC); i++){
        
        Serial.printf("Row %d => ", i);
        for (int j = 0; j < NUM_CELL_IC; j++){
            
            Serial.printf("[Cell %d: ", j);
            
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
    }
    Serial.println("-----------------------------------------");
    return 1;
}

// Temperature Measurement
/************** INFO **************
 * Measures the temperature of each cell or the entire
 * battery pack to prevent overheating.
 **********************************/

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