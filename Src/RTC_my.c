//File with RTC init and working function
#include "RTC_my.h"

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime_glb;

/* RTC init function */
void RTC_my_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;
  RTC_AlarmTypeDef sAlarm;

	/**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  hrtc.DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  hrtc.DateToUpdate.Month = RTC_MONTH_JANUARY;
  hrtc.DateToUpdate.Date = 1;
  hrtc.DateToUpdate.Year = 0;
  HAL_RTC_Init(&hrtc);
	
	if (HAL_RTC_GetState(&hrtc) == HAL_RTC_STATE_READY)
	{	
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

	DateToUpdate.Date =  1;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Year = 0;
  HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BCD);

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 10;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BCD);
	RTC->CRH |= RTC_CRH_SECIE;
}

	HAL_NVIC_EnableIRQ(RTC_IRQn);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

void RTC_AlarmIRQ_function(void)
{
	RTC_AlarmTypeDef sAlarm;
  	
 if(__HAL_RTC_ALARM_GET_IT_SOURCE(&hrtc, RTC_IT_ALRA))
  {
    /* Get the status of the Interrupt */
    if(__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != (uint32_t)RESET)
    {
      /* AlarmA callback */ 
      	HAL_RTC_GetAlarm(&hrtc,&sAlarm, sAlarm.Alarm, FORMAT_BCD);
				sAlarm.AlarmTime.Seconds += 10;
				HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, FORMAT_BCD);
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      
      /* Clear the Alarm interrupt pending bit */
      __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc,RTC_FLAG_ALRAF);
    }
  }
  
  /* Clear the EXTI's line Flag for RTC Alarm */
  __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

}


void RTC_IRQ_function(void)
{
	
	  if(__HAL_RTC_SECOND_GET_IT_SOURCE(&hrtc, RTC_IT_SEC))
  {
    /* Get the status of the Interrupt */
    if(__HAL_RTC_SECOND_GET_FLAG(&hrtc, RTC_FLAG_SEC))
    {
      /* Check if Overrun occurred */
      if (__HAL_RTC_SECOND_GET_FLAG(&hrtc, RTC_FLAG_OW))
      {
        /* Second error callback */ 
        HAL_RTCEx_RTCEventErrorCallback(&hrtc);
        
        /* Clear flag Second */
        __HAL_RTC_OVERFLOW_CLEAR_FLAG(&hrtc, RTC_FLAG_OW);
        
        /* Change RTC state */
        hrtc.State = HAL_RTC_STATE_ERROR; 
      }
      else 
      {
        /* Second callback */ 
     //   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
				HAL_RTC_GetTime(&hrtc,&sTime_glb, FORMAT_BCD);
        
        /* Change RTC state */
        hrtc.State = HAL_RTC_STATE_READY; 
      }
      
      /* Clear flag Second */
      __HAL_RTC_SECOND_CLEAR_FLAG(&hrtc, RTC_FLAG_SEC);
    }
  }
}

