#ifndef	_I2C_MY_H
#define _I2C_MY_H

#include "stm32f1xx_hal.h"


#define L3G4200_ADDR	0xD3
#define L3GD20_WHO_AM_I				0xD3
#define L3GD20_REG_WHO_AM_I			0x0F

void I2C1_Init_my(void);
uint16_t L3G4200_ReadReg(uint8_t reg);

#endif
