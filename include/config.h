#ifndef CONFIG_H
#define CONFIG_H

#define NUM_TOTAL_IC 10
#define NUM_CELL_IC 14

#define PRM_CELL_MAX_VOLT 4.2
#define PRM_CELL_MIN_VOLT 2.8

/*! @brief Cell map for 7 cells connected to MC33771. */
#define BCC_CM_MC33771_7CELLS     0x380FU
/*! @brief Cell map for 3 cells connected to MC33772. */
#define BCC_CM_MC33772_3CELLS     0x0023U

#define CELL_BALANCE_TIMER 0x0A // some random val

#define PRM_CELL_MIN_TEMP __FLT_MIN__ // to set later
#define PRM_CELL_MAX_TEMP __FLT_MAX__ // to set later

#define PRM_MIN_BALL_TEMP __FLT_MIN__ // to set later
#define PRM_MAX_BALL_TEMP __FLT_MAX__ // to set later

#endif