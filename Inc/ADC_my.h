#ifndef _ADC_MY_H
#define _ADC_MY_H

#include "stm32f1xx_hal.h"

#define ADC_0V_VALUE                            0
#define ADC_1V_VALUE                            1241
#define ADC_2V_VALUE                            2482
#define ADC_3V_VALUE                            3723

void ADC_Init_my(void);
void Work_with_ADC(void);

#endif
