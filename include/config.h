#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG 0
#define ISOLATE 1

#define NUM_TOTAL_IC 10
#define NUM_CELL_IC 14

#define CELL_MAX_VOLT 4.2
#define CELL_MIN_VOLT 2.8

/*! @brief Cell map for 7 cells connected to MC33771. */
#define BCC_CM_MC33771_7CELLS     0x380FU
/*! @brief Cell map for 3 cells connected to MC33772. */
#define BCC_CM_MC33772_3CELLS     0x0023U

#define CELL_BALANCE_TIMER 0x0A // some random val

#define CELL_MIN_TEMP __FLT_MIN__ // to set later
#define CELL_MAX_TEMP __FLT_MAX__ // to set later

#define MIN_BALL_TEMP __FLT_MIN__ // to set later
#define MAX_BALL_TEMP __FLT_MAX__ // to set later

#define INIT_REG_CNT 0
#define RX_BUF_SIZE BCC_RX_BUF_SIZE_TPL
#define DEVICE bcc_device_t::BCC_DEVICE_MC33771C
#define MODE bcc_mode_t::BCC_MODE_TPL

bcc_cc_data_t coulumb_counter; 
// int32_t ccAccumulator;      /*!< Coulomb counting accumulator with V_2RES resolution. */
// uint16_t nbSamples;          /*!< Number of samples accumulated in the Coulomb counter. */

bcc_drv_data_t driver_internal_data;
// uint16_t cellMap[BCC_DEVICE_CNT_MAX];    /*!< Bit map of used cells of each BCC device. */
// uint8_t msgCntr[BCC_DEVICE_CNT_MAX + 1]; /*!< Last received value of Message counter (values 0-15).
                                           // MsgCntr[0] contains Message counter of CID=0. */
// uint8_t rxBuf[BCC_RX_BUF_SIZE_TPL];      /*!< Buffer for receiving data in TPL mode. */

bcc_drv_config_t driver_config; // should be declared within battery

enum States {
    STANDBY,
    SHUTDOWN,
    PRECHARGE,
    CHARGE,
    NORMAL,
};

#endif