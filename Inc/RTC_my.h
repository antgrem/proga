#ifndef _RTC_MY_H
#define _RTC_MY_H

#include "stm32f1xx_hal.h"


void RTC_my_Init(void);
void RTC_AlarmIRQ_function(void);
void RTC_IRQ_function(void);
	
#endif
