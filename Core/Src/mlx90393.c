#include <stdio.h>
#include "i2c.h"

#include "mlx90393.h"

const uint8_t mlx_addr[12]={	
	0x0C, 0x0E, 0x0F, 
	0x10, 0x12, 0x13,
	0x14, 0x16, 0x17,
	0x18, 0x1A, 0x1B
};

const uint16_t mlx_default[10]=
{0x007C, 0x0000, 0x0000, 0x0000, 0x0000,
 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
uint8_t cmd = 0;

uint8_t data[9] = {0,0,0,0,0,0,0,0,0};
uint8_t * stat = data;
uint8_t readfromreg[3] = {0,0,0};
uint16_t cmd_temp = 0;

struct mlx mlxarray[12];


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
uint8_t mlx_burst_mode(uint8_t mlx_addr){
	uint8_t code = 0;
	cmd = 0x1F;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	// if(stat < 0x80){
	// 	return 1;
	// }
	return code;
}


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
uint8_t mlx_wake_on_change(uint8_t mlx_addr, uint8_t x, uint8_t y, uint8_t z, uint8_t t){
	uint8_t code = 0;
	cmd = 0x20 | t | x<<1 | y<<2 | z<<3;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	// if((stat<<1) < 0x80){
	// 	return 1;
	// }
	return code;
}

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
uint8_t mlx_single_measure(uint8_t mlx_addr, uint8_t x, uint8_t y, uint8_t z, uint8_t t){
	uint8_t code = 0;
	cmd = 0x30 | t | x<<1 | y<<2 | z<<3;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	// if((stat<<2) < 0x80){
	// 	return 1;
	// }
	return code;
}

/**
 * @brief RM--Read Measurement
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_read_measure(uint8_t mlx_addr){
	uint8_t code = 0;
	cmd = 0x4F;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, data, 9, HAL_MAX_DELAY);
	return code;
} 

/**
 * @brief RR--Read Register
 * @note 
 * @param mlx_addr sensor address(7bit)
 * @param reg Target register to read from. reg = register addr <<2
 * @retval HAL_Status
 */
uint8_t mlx_read_reg(uint8_t mlx_addr, uint8_t reg){
	uint8_t code = 0;
	cmd = 0x50;
	cmd_temp = cmd;
	cmd_temp = ((uint16_t)cmd)<<8 | (reg<<2);
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd_temp, 3, readfromreg, 3, HAL_MAX_DELAY);
	return code;
}

/**
 * @brief WR--Write Register
 * @note
 * @param mlx_addr sensor address(7bit)
 * @param reg Target register to write to.
 * @param writetoreg Data to write to the specified register
 * @retval HAL_Status
 */
uint8_t mlx_write_reg(uint8_t mlx_addr, uint8_t reg, uint16_t writetoreg){
	uint8_t code = 0;
	cmd = 0x60;
	uint8_t data_from_master[4] = {cmd, (uint8_t)(writetoreg>>8), (uint8_t)(writetoreg), reg<<2};
	code = HAL_I2C_Master_Transmit(&hi2c1, mlx_addr<<1, data_from_master, 4, HAL_MAX_DELAY);
	return code;
}

/**
 * @brief EX--Exit Mode
 * @note When receiving an EX instruction, the sensor exits current measure and goes to IDLE.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */ 
uint8_t mlx_exit(uint8_t mlx_addr){
	uint8_t code = 0;
	cmd = 0x80;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	return code;
}

/**
 * @brief HR--Memory Recall
 * @note To read from volatile memory, first recall from non-volatile memory.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_mem_recall(uint8_t mlx_addr){
	uint8_t code = 0;
	cmd = 0xD0;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	return code;
}

/**
 * @brief HS--Memory Store
 * @note To apply register configurations for good, call HS to store contents from volatile memory to non-volatile one.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_mem_store(uint8_t mlx_addr){
	uint8_t code = 0;
	cmd = 0xE0;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	HAL_Delay(15);
	return code;
}

/**
 * @brief RT--Reset
 * @note It takes sometime for the sensor to fully recover from RT.
 * @param mlx_addr sensor address(7bit)
 * @retval HAL_Status
 */
uint8_t mlx_reset(uint8_t mlx_addr){
	uint8_t code = 0;
	mlx_exit(mlx_addr);
	HAL_Delay(1);
	cmd = 0xF0;
	code = HAL_I2C_Mem_Read(&hi2c1, mlx_addr<<1, cmd, 1, stat, 1, HAL_MAX_DELAY);
	HAL_Delay(2);
	return code;
}


/**
 * @brief mlx_config: config sensor registers.
 * @note
 * @param 
 * @retval HAL_Status
 */
uint8_t mlx_config(void){
	uint8_t code = 0;
	uint8_t i = 0;

	for(i = 0; i<12; i++){
		code = mlx_write_reg(mlx_addr[i], 0x00, CONF00);
		if(code != HAL_OK){
			return code*0x10+i;
		}
		HAL_Delay(2);
  	}
	for(i = 0; i<12; i++){
		code = mlx_write_reg(mlx_addr[i], 0x01, CONF01);
		if(code != HAL_OK){
			return code*0x20+i;
		}
		HAL_Delay(2);
  	}
	for(i = 0; i<12; i++){
		code = mlx_write_reg(mlx_addr[i], 0x02, CONF02);
		if(code != HAL_OK){
			return code*0x30+i;
		}
		HAL_Delay(2);
  	}
	return code;
}

/**
 * @brief mlx_init: Start burst mode for all sensors.
 */
uint8_t mlx_init(void){
	uint8_t code = 0;
	uint8_t i = 0;

	for(i=0;i<12;i++){
		code = mlx_burst_mode(mlx_addr[i]);
		if(code != HAL_OK){
			return -code*10-i;
		}
	  	HAL_Delay(2);
	}
	return code;
}

/**
 * @brief mlx_calc: do the math
 * @note 
 * @param 
 */
uint8_t mlx_calc(MLX* mlx){
	mlx->status = data[0];
	mlx->T_raw = ((uint16_t)data[1])<<8|data[2];
	mlx->X_raw = ((uint16_t)data[3])<<8|data[4];
	mlx->Y_raw = ((uint16_t)data[5])<<8|data[6];
	mlx->Z_raw = ((uint16_t)data[7])<<8|data[8];
	mlx->T = (mlx->T_raw-46244)/45.2F + 25.0F;
	mlx->X = (mlx->X_raw) * mlx90393_lsb_lookup[0][7][0][0];
	mlx->Y = (mlx->Y_raw) * mlx90393_lsb_lookup[0][7][0][0];
	mlx->Z = (mlx->Z_raw) * mlx90393_lsb_lookup[0][7][0][1];
	return 0;
}