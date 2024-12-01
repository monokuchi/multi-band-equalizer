/*
 * CS4272.c
 *
 *	CS4272 Stereo Audio Codec Driver
 *
 *
 *  Created on: Jul 17, 2024
 *      Author: john
 */



/*
 * INCLUDES
 */
#include "CS4272.h"

#include <string.h>





const uint8_t CS4272_REGISTERS[CS4272_NUM_REGISTERS] = {
															CS4272_REG_MODE_CTRL_1,
															CS4272_REG_DAC_CTRL,
															CS4272_REG_DAC_VOL_MIX_CTRL,
															CS4272_REG_DAC_CH_A_VOL_CTRL,
															CS4272_REG_DAC_CH_B_VOL_CTRL,
															CS4272_REG_ADC_CTRL,
															CS4272_REG_MODE_CTRL_2,
															CS4272_REG_CHIP_ID
													   };


uint8_t CS4272_REGISTERS_INIT_CONFIG[CS4272_NUM_REGISTERS] = {
																  0x21, 	  						            // Single Speed Mode, MCLK/LRCK=512 and SCLK/LRCK=64, Slave Mode, DAC Digital Interface Format (I2S, 24 bit)
																  0x80, 									    // Auto Mute
																  0x29, 									    // Soft Ramp, ATAPI Channel Mixing (AOUTA=aL, AOUTB=bR)
																  0x00,
																  0x00,
																  0x10,											// ADC Digital Interface Format (I2S, 24 bit)
																  0x02, 					   				    // Set Control Port Enable, and clear Power Down bit
																  ((CS4272_CHIP_ID << 4) | CS4272_CHIP_REV)     // Device part and revision number
														     };



/*
 * FUNCTIONS
 */
void CS4272_Reset()
{
	// Hold nRST low until the power supply, MCLK, and LRCK are stable
	HAL_GPIO_WritePin(CS4272_NRST_GPIO_PORT, CS4272_NRST_GPIO_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
	// Bring nRST high to turn the device on again
	HAL_GPIO_WritePin(CS4272_NRST_GPIO_PORT, CS4272_NRST_GPIO_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
}


uint8_t CS4272_Init(CS4272 *dev, I2C_HandleTypeDef *i2c)
{
	/*
	 * Set CS4272 struct parameters
	 */
	dev->i2c_handle = i2c;
	// Reset register_map
	memset(dev->register_map, 0, CS4272_NUM_REGISTERS);


	/*
	 * Variables needed for initialization procedure
	 */
	HAL_StatusTypeDef status = HAL_OK;
	// Stores the data we pull from the registers
	uint8_t reg_data = 0x00;



	/*
	 * Get access to Control Port Mode via the power up sequence (Pg 27)
	 */
	CS4272_Reset();



	/*
	 * Activate the Control Port and put the codec in power down mode
	 */
	reg_data = 0x03;
	status = CS4272_Write_Register(dev, CS4272_REG_MODE_CTRL_2, &reg_data);
	if (status != HAL_OK) { return 255; }


	/*
	 * Check the codec Chip ID/Revision to see if we are communicating with the right device
	 */
	status = CS4272_Read_Register(dev, CS4272_REG_CHIP_ID, &reg_data);
	if (status != HAL_OK || reg_data != ((CS4272_CHIP_ID << 4) | CS4272_CHIP_REV)) { return 254; }



	/*
	 * Write the desired initialization register values
	 */
	for (size_t i=0; i<CS4272_NUM_REGISTERS-1; ++i)
	{
		// Write the corresponding initialization configuration value into the register
		status = CS4272_Write_Register(dev, CS4272_REGISTERS[i], &CS4272_REGISTERS_INIT_CONFIG[i]);
		if (status != HAL_OK) { return i+1; }

		// Read back what we just wrote and double check the contents
		status = CS4272_Read_Register(dev, CS4272_REGISTERS[i], &reg_data);
		if (status != HAL_OK || reg_data != CS4272_REGISTERS_INIT_CONFIG[i]) { return i+1; }
	}


	/*
	 * Update the register_map with the current state of all the registers
	 */
	status = CS4272_Read_All_Registers(dev, dev->register_map);
	if (status != HAL_OK) { return 253; }


	/*
	 * Return 0 to indicate everything went well)
	 */
	return 0;
}


HAL_StatusTypeDef CS4272_Read_Register(CS4272 *dev, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->i2c_handle, CS4272_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	HAL_Delay(1);
	return status;
}


HAL_StatusTypeDef CS4272_Write_Register(CS4272 *dev, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->i2c_handle, CS4272_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	HAL_Delay(1);
	return status;
}

HAL_StatusTypeDef CS4272_Read_All_Registers(CS4272 *dev, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_OK;
	for (size_t i=0; i<CS4272_NUM_REGISTERS; ++i)
	{
		status = CS4272_Read_Register(dev, CS4272_REGISTERS[i], &data[i]);
		if (status != HAL_OK) { return status; }
	}
	return status;
}

HAL_StatusTypeDef CS4272_Write_All_Registers(CS4272 *dev, uint8_t *data)
{
	HAL_StatusTypeDef status = HAL_OK;
	for (size_t i=0; i<CS4272_NUM_REGISTERS; ++i)
	{
		status = CS4272_Write_Register(dev, CS4272_REGISTERS[i], &data[i]);
		if (status != HAL_OK) { return status; }
	}
	return status;
}
