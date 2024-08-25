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








/*
 * FUNCTIONS
 */
uint8_t CS4272_Init(CS4272 *dev, I2C_HandleTypeDef *i2c)
{
	/*
	 * Set CS4272 struct parameters
	 */
	dev->i2c_handle = i2c;



	// Set up error tracking
	uint16_t error_count = 0;
	HAL_StatusTypeDef status = HAL_OK;
	// Stores the data we pull from the registers
	uint8_t reg_data = 0x00;


	/*
	 * Check for correct Chip ID and Revision
	 */
	status = CS4272_Read_Register(dev, CS4272_REG_CHIP_ID, &reg_data);
	if (status != HAL_OK) { ++error_count; }

	if (reg_data != ((CS4272_CHIP_ID << 4) | CS4272_CHIP_REV))
	{
		// Chip ID and Revision does not match, throw error
		return 255;
	}



	/*
	 * Get access to Control Port Mode via the power up sequence (Pg 27)
	 */
	// Hold nRST low until the power supply, MCLK, and LRCK are stable
	HAL_GPIO_WritePin(CS4272_NRST_GPIO_PORT, CS4272_NRST_GPIO_PIN, GPIO_PIN_RESET);
	HAL_Delay(100);
	// Bring nRST high to turn the device on again
	HAL_GPIO_WritePin(CS4272_NRST_GPIO_PORT, CS4272_NRST_GPIO_PIN, GPIO_PIN_SET);

	// Set the Control Port Enable (CPEN) and Power Down (PDN) bits
	// Need to write 03h to register 07h within 10 ms of releasing NRST
	reg_data = 0x03;
	status = CS4272_Write_Register(dev, CS4272_REG_MODE_CTRL_2, &reg_data);
	if (status != HAL_OK) { ++error_count; }


	/*
	 * Set custom register values
	 */
	// Set clocking ratios to MCLK/LRCK = 512 and SCLK/LRCK = 64 (Pg 28)
	// Set Master mode, since default mode is Slave mode in Control Port Mode (Pg 27)
	// Final byte in register 0x01 needs to be 0b00101000 or 0x28
	reg_data = 0x28;
	status = CS4272_Write_Register(dev, CS4272_REG_MODE_CTRL_1, &reg_data);
	if (status != HAL_OK) { ++error_count; }



	/*
	 * Once finished clear PDN bit to start the chip up with the new settings
	 */
	reg_data = 0x03;
	status = CS4272_Write_Register(dev, CS4272_REG_MODE_CTRL_2, &reg_data);
	if (status != HAL_OK) { ++error_count; }




	/*
	 * Return total error count (should be 0 if everything went well)
	 */
	return error_count;
}


HAL_StatusTypeDef CS4272_Read_Register(CS4272 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2c_handle, CS4272_I2C_ADDR, reg, sizeof(reg), data, 1, HAL_MAX_DELAY);
}




HAL_StatusTypeDef CS4272_Write_Register(CS4272 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(dev->i2c_handle, CS4272_I2C_ADDR, reg, sizeof(reg), data, 1, HAL_MAX_DELAY);
}









