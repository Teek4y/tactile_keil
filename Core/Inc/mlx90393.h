#include <stdio.h>
#include "i2c.h"

#define OP_CONF 0x1813//0x1813

#define BIST 0x0000 // 0x0000
#define Z_SERIES 0x0000 //0x0000
#define GAIN_SEL 0x0000 //0x0070
#define HALLCONF 0x000C //0x000C
#define CONF00 (BIST | Z_SERIES | GAIN_SEL | HALLCONF)

#define TRIG_INT 0x0000
#define COMM_MODE 0x0000
#define WOC_DIFF 0x0000
#define EXT_TRG 0x0000
#define TCMP_EN 0x0000
#define BURST_SEL 0x0000
#define BURST_DATARATE 0x0000
#define CONF01 (TRIG_INT | COMM_MODE | WOC_DIFF | EXT_TRG | TCMP_EN | BURST_SEL | BURST_DATARATE)

#define OSR2 0x0000
#define RES_Z (0x0<<9)  //[0x0]|0x1|0x2|0x3
#define RES_Y (0x0<<7)  //[0x0]|0x1|0x2|0x3
#define RES_X (0x0<<5)  //[0x0]|0x1|0x2|0x3
#define DIG_FILT 0x0000
#define OSR 0x0000
// #define CONF02 = OSR2 | RES_Z | RES_Y | RES_X | DIG_FILT | OSR
#define CONF02 0x1813
extern const uint8_t mlx_addr[12];
extern uint8_t * stat;
extern uint8_t data[9];
extern uint8_t readfromreg[3];
extern uint8_t cmd;
extern uint16_t cmd_temp;



typedef struct mlx{
	uint8_t addr;
	uint8_t status;
	uint8_t gain;
	uint8_t hallconf;
	int16_t X_raw;
	int16_t Y_raw;
	int16_t Z_raw;
	uint16_t T_raw;
	float X;
	float Y;
	float Z;
	float T;
	float Mag;
} MLX;

static const float mlx90393_lsb_lookup[2][8][4][2] = {

    /* HALLCONF = 0xC (default) */
    {
    // |    RES = 0   |  |   RES = 1  |  |   RES = 2  |  |   RES = 3  |
    // |SENSxy |SENSz |  |SENSxy|SENSz|  |SENSxy|SENSz|  |SENSxy|SENSz|
        /* GAIN_SEL = 0, 5x gain */
        {{0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}, {6.009, 9.680}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}, {4.840, 7.744}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.451, 0.726}, {0.901, 1.452}, {1.803, 2.904}, {3.605, 5.808}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.376, 0.605}, {0.751, 1.210}, {1.502, 2.420}, {3.004, 4.840}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}, {2.403, 3.872}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.250, 0.403}, {0.501, 0.807}, {1.001, 1.613}, {2.003, 3.227}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.200, 0.323}, {0.401, 0.645}, {0.801, 1.291}, {1.602, 2.581}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.150, 0.242}, {0.300, 0.484}, {0.601, 0.968}, {1.202, 1.936}},
    },

    /* HALLCONF = 0x0 */
    {
    // |    RES = 0   |  |   RES = 1  |  |   RES = 2  |  |   RES = 3  |
    // |SENSxy |SENSz |  |SENSxy|SENSz|  |SENSxy|SENSz|  |SENSxy|SENSz|
        /* GAIN_SEL = 0, 5x gain */
        {{0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}, {6.292, 10.137}},
        /* GAIN_SEL = 1, 4x gain */
        {{0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}, {5.034, 8.109}},
        /* GAIN_SEL = 2, 3x gain */
        {{0.472, 0.760}, {0.944, 1.521}, {1.888, 3.041}, {3.775, 6.082}},
        /* GAIN_SEL = 3, 2.5x gain */
        {{0.393, 0.634}, {0.787, 1.267}, {1.573, 2.534}, {3.146, 5.068}},
        /* GAIN_SEL = 4, 2x gain */
        {{0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}, {2.517, 4.055}},
        /* GAIN_SEL = 5, 1.667x gain */
        {{0.262, 0.422}, {0.524, 0.845}, {1.049, 1.689}, {2.097, 3.379}},
        /* GAIN_SEL = 6, 1.333x gain */
        {{0.210, 0.338}, {0.419, 0.676}, {0.839, 1.352}, {1.678, 2.703}},
        /* GAIN_SEL = 7, 1x gain */
        {{0.157, 0.253}, {0.315, 0.507}, {0.629, 1.014}, {1.258, 2.027}},
    }};

/** Lookup table for conversion time based on [DIF_FILT][OSR].
 */
static const float mlx90393_tconv[8][4] = {
  //|        OSR          |
  //| 0  || 1 | | 2 | | 3 |
    /* DIG_FILT = 0 */
    {1.27, 1.84, 3.00, 5.30},
    /* DIG_FILT = 1 */
    {1.46, 2.23, 3.76, 6.84},
    /* DIG_FILT = 2 */
    {1.84, 3.00, 5.30, 9.91},
    /* DIG_FILT = 3 */
    {2.61, 4.53, 8.37, 16.05},
    /* DIG_FILT = 4 */
    {4.15, 7.60, 14.52, 28.34},
    /* DIG_FILT = 5 */
    {7.22, 13.75, 26.80, 52.92},
    /* DIG_FILT = 6 */
    {13.36, 26.04, 51.38, 102.07},
    /* DIF_FILT = 7 */
    {25.65, 50.61, 100.53, 200.37},
};

/**
 * @brief SB--Start Burst Mode
 * @note The ASIC will have a programmable data rate at which it will operate. This data rate implies 
 * auto-wakeup and sequencing of the ASIC, flagging that data is ready on a dedicated pin (INT/DRDY). 
 * The maximum data rate corresponds to continuous burst mode, and is a function of the chosen 
 * measurement axes. For non-continuous burst modes, the time during which the ASIC has a counter 
 * running but is not doing an actual conversion is called the Standby mode (STBY).
 * @param mlx_addr sensor address(7bit)
 * @retval 1: sensor refuses entering burst	mode
 * @retval HAL_Status
 */
uint8_t mlx_burst_mode(uint8_t mlx_addr);

/**
 * @brief SW--Start Wake-up on Change Mode
 * @note This mode is similar to the burst mode in the sense that the device will be auto-sequencing, 
 * with the difference that the measured component(s) is/are compared with a reference and in case the 
 * difference is bigger than a user-defined threshold, the DRDY signal is set on the designated pin. The user 
 * can select which axes and/or temperature fall under this cyclic check, and which thresholds are allowed.
 * @param mlx_addr sensor address(7bit)
 * @param x whether to obtain x-axis
 * @param y whether to obtain y-axis
 * @param z whether to obtain z-axis
 * @param t whether to obtain temperature
 * @retval 1: sensor refuses entering wOC mode
 * @retval HAL_Status
 */
uint8_t mlx_wake_on_change(uint8_t mlx_addr, uint8_t x, uint8_t y, uint8_t z, uint8_t t);

/**
 * @brief SM--Start Single Measurement Mode
 * @note The master will ask for data, waking up the sensor to make a single conversion, immediately
 *  followed by an automatic return to sleep mode (IDLE) until the next polling of the master. 
 * @param mlx_addr sensor address(7bit)
 * @param x whether to obtain x-axis
 * @param y whether to obtain y-axis
 * @param z whether to obtain z-axis
 * @param t whether to obtain temperature
 * @retval 1: sensor refuses entering SM mode
 * @retval HAL_Status
 */
uint8_t mlx_single_measure(uint8_t mlx_addr, uint8_t x, uint8_t y, uint8_t z, uint8_t t);

/**
 * @brief RM--Read Measurement
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_read_measure(uint8_t mlx_addr);

/**
 * @brief RR--Read Register
 * @note 
 * @param mlx_addr sensor address(7bit)
 * @param reg Target register to read from. reg = register addr <<2
 * @retval HAL_Status
 */
uint8_t mlx_read_reg(uint8_t mlx_addr, uint8_t reg);

/**
 * @brief WR--Write Register
 * @note
 * @param mlx_addr sensor address(7bit)
 * @param reg Target register to write to.
 * @param writetoreg Data to write to the specified register
 * @retval HAL_Status
 */
uint8_t mlx_write_reg(uint8_t mlx_addr, uint8_t reg, uint16_t writetoreg);

/**
 * @brief EX--Exit Mode
 * @note When receiving an EX instruction, the sensor exits current measure and goes to IDLE.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */ 
uint8_t mlx_exit(uint8_t mlx_addr);

/**
 * @brief HR--Memory Recall
 * @note To read from volatile memory, first recall from non-volatile memory.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_mem_recall(uint8_t mlx_addr);

/**
 * @brief HS--Memory Store
 * @note To apply register configurations for good, call HS to store contents from volatile memory to non-volatile one.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_mem_store(uint8_t mlx_addr);

/**
 * @brief RT--Reset
 * @note It takes sometime for the sensor to fully recover from RT.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_reset(uint8_t mlx_addr);

/**
 * @brief mlx_config: config sensor registers.
 * @note
 * @param 
 * @retval HAL_Status
 */
uint8_t mlx_config(void);

/**
 * @brief mlx_init: read out the sensors
 * @note
 * @param 
 * @retval HAL_Status
 */
uint8_t mlx_init(void);

/**
 * @brief mlx_calc: do the math
 * @note 
 * @param 
 */
uint8_t mlx_calc(MLX* mlx);