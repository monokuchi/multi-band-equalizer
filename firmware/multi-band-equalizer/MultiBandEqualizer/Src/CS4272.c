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
#include <string.h>

#include "CS4272.h"




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



/*
 * FUNCTIONS
 */
uint8_t CS4272_Init(CS4272 *dev, I2C_HandleTypeDef *i2c)
{
	/*
	 * Set CS4272 struct parameters
	 */
	dev->i2c_handle = i2c;
	// Reset register_map
	memset(dev->register_map, 0, sizeof(dev->register_map));


	/*
	 * Variables needed for initialization procedure
	 */
	HAL_StatusTypeDef status = HAL_OK;
	// Stores the data we pull from the registers
	uint8_t reg_data = 0x00;
	// Register settings for initialization
	uint8_t CS4272_REGISTERS_INIT_CONFIG[CS4272_NUM_REGISTERS] = {
																	 0x28, 									   // MCLK/LRCK = 512 and SCLK/LRCK = 64, Master Mode
																	 0x80, 									   // Auto Mute
																	 0x29, 									   // Soft Ramp, ATAPI Channel Mixing (enable aL, bR)
																	 0x00,
																	 0x00,
																	 0x00,
																	 0x03, 									   // Control Port Enable, Power Down
																	 ((CS4272_CHIP_ID << 4) | CS4272_CHIP_REV)  // Device part and revision number
																 };




	/*
	 * Get access to Control Port Mode via the power up sequence (Pg 27)
	 */
	// Hold nRST low until the power supply, MCLK, and LRCK are stable
	HAL_GPIO_WritePin(CS4272_NRST_GPIO_PORT, CS4272_NRST_GPIO_PIN, GPIO_PIN_RESET);
	HAL_Delay(100);
	// Bring nRST high to turn the device on again
	HAL_GPIO_WritePin(CS4272_NRST_GPIO_PORT, CS4272_NRST_GPIO_PIN, GPIO_PIN_SET);



	uint8_t tmp_data = 0x05;
	status = CS4272_Write_Register(dev, CS4272_REG_DAC_CTRL, &tmp_data);
	status = CS4272_Read_Register(dev, CS4272_REG_DAC_CTRL, &tmp_data);



	/*
	 * Write the desired initialization register values
	 */
	for (size_t i=CS4272_NUM_REGISTERS-1; i>=0; --i)
	{
		// Write the corresponding initialization configuration value into the register
		status = CS4272_Write_Register(dev, CS4272_REGISTERS[i], &CS4272_REGISTERS_INIT_CONFIG[i]);
		if (status != HAL_OK) { return i; }

		// Read back what we just wrote and double check the contents
		status = CS4272_Read_Register(dev, CS4272_REGISTERS[i], &reg_data);
		if (status != HAL_OK || reg_data != CS4272_REGISTERS_INIT_CONFIG[i-1]) { return i; }
	}


	/*
	 * Read in and store the current state of all the registers
	 */
	status = CS4272_Read_All_Registers(dev, dev->register_map);
	if (status != HAL_OK) { return 255; }


//	/*
//	 * Once finished clear PDN bit to start the chip up with the new settings
//	 */
//	reg_data = 0x02;
//	status = CS4272_Write_Register(dev, CS4272_REG_MODE_CTRL_2, &reg_data);
//	if (status != HAL_OK) { ++error_count; }
//	// Wait a bit for the codec to power up
//	HAL_Delay(100);


	/*
	 * Return 0 to indicate everything went well)
	 */
	return 0;
}


HAL_StatusTypeDef CS4272_Read_Register(CS4272 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2c_handle, CS4272_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


HAL_StatusTypeDef CS4272_Write_Register(CS4272 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(dev->i2c_handle, CS4272_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
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



