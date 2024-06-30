#ifndef CONFIG_H
#define CONFIG_H

#pragma once
#define NUM_TOTAL_IC 10
#define NUM_CELL_IC 14

#define PRM_CELL_MAX_VOLT 4.2
#define PRM_CELL_MIN_VOLT 2.8

#define INIT_REG_CNT 45
#define GPIO_CFG1 0.0
#define GPIO_CFG2 0.0
#define CBX_CFG 0.0
#define RX_BUF_SIZE 8

#define NUM_TOTAL_IC 10
#define NUM_CELL_IC 14

#define CELL_MIN_TEMP 0.0 // to set later
#define CELL_MAX_TEMP 1000.0 // to set later

#define MIN_BALL_TEMP 0.0 // to set later
#define MAX_BALL_TEMP 1000.0 // to set later

#define D_PRINT(x) printf(x) // DEBUG MODE enabled
#define PRINT(x) Serial.println(x)

#endif