/*
 * CS4272.h
 *
 *	CS4272 Stereo Audio Codec Driver
 *
 *
 *  Created on: Jul 17, 2024
 *      Author: john
 */

#ifndef INC_CS4272_H_
#define INC_CS4272_H_


/*
 * INCLUDES
 */
#include "stm32h7xx_hal.h"



/*
 * DEFINES
 */
#define CS4272_I2C_ADDR				(0x10 << 1) // AD0 = 0 -> 0x10, AD0 = 1 -> 0x11, 7-bit address (Pg 36)
#define CS4272_CHIP_ID				0x0
#define CS4272_CHIP_REV				0x0

#define CS4272_NRST_GPIO_PORT 		GPIOC
#define CS4272_NRST_GPIO_PIN		GPIO_PIN_11


/*
 * REGISTERS (Pg 37-44)
 */
#define CS4272_REG_MODE_CTRL_1		 0x01
#define CS4272_REG_DAC_CTRL   		 0x02
#define CS4272_REG_DAC_VOL_MIX_CTRL  0x03
#define CS4272_REG_DAC_CH_A_VOL_CTRL 0x04
#define CS4272_REG_DAC_CH_B_VOL_CTRL 0x05
#define CS4272_REG_ADC_CTRL 	     0x06
#define CS4272_REG_MODE_CTRL_2       0x07
#define CS4272_REG_CHIP_ID     		 0x08







/*
 * CS4272 STRUCT
 */
typedef struct
{
	I2C_HandleTypeDef *i2c_handle;

} CS4272;



/*
 * FUNCTIONS
 */
uint8_t CS4272_Init(CS4272 *dev, I2C_HandleTypeDef *i2c);


HAL_StatusTypeDef CS4272_Read_Register(CS4272 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef CS4272_Write_Register(CS4272 *dev, uint8_t reg, uint8_t *data);










#endif /* INC_CS4272_H_ */
