/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <lm75.h>
#include "fatfs_sd.h"
#include "tm_stm32f4_fatfs.h"
#include "RTC_my.h"
#include "ADC_my.h"
#include "I2C_my.h"

#define LM75_ADDRESS 0x9F
#define PI 3.14159265


/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

uint16_t temp16;
	uint16_t l3g4200_temp;

osThreadId defaultTaskHandle;
osThreadId UserTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void UserDefaultTask(void const * argument);
void LM75_RW(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

  char buffer[100];

		/* Fatfs object */
	FATFS FatFs;
	/* File object */
	FIL fil, fil_Tempr;
	/* Free and total space */
	uint32_t total, free_fat;
	
	FRESULT temp_sd_res;
/* USER CODE END 0 */

int main(void)
{
GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t treg;	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	DBGMCU->CR = 0xFFFFFFFF;

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  ADC_Init_my();
  I2C1_Init_my();
  RTC_my_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
	

	
	NVIC_EnableIRQ (EXTI0_IRQn);
//	
//	HAL_PWR_EnterSLEEPMode(treg, PWR_SLEEPENTRY_WFE);
	


//  /* USER CODE BEGIN 2 */
//	if(TM_STMPE811_Init() == TM_STMPE811_State_Ok)
//		{
//			if (TM_I2C_Read(STMPE811_I2C, LM75_ADDRESS, 0x00) == 0)
//			{
//				TM_ILI9341_Puts(60, 40, "Error init LM75", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_RED);
//			}
//						
//			TM_I2C_Write(STMPE811_I2C, STMPE811_ADDRESS, STMPE811_INT_CTRL, 0x03);
//			TM_I2C_Write(STMPE811_I2C, STMPE811_ADDRESS, STMPE811_INT_STA, 0x01);
//			TM_I2C_Write(STMPE811_I2C, STMPE811_ADDRESS, STMPE811_INT_EN, 0x01);	
//		
//		};

// 
//			if (f_mount(&FatFs, "0:", 1) == FR_OK) 
//				{
//							//Get time
//						TM_RTC_GetDateTime(&datatime, TM_RTC_Format_BIN);
//				//	sprintf(buffer, "0:F%02d_%02d_%04d.txt", datatime.date, datatime.month, datatime.year);
//						sprintf(file_name_data, "0:%04d_%02d_%02d_Watt.txt", datatime.year+2000, datatime.month, datatime.date);
//					temp_sd_res = f_open(&fil, (TCHAR*) file_name_data, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
//							if (temp_sd_res != FR_OK) 
//								{
//									if (f_open(&fil, file_name_data, FA_CREATE_NEW | FA_READ | FA_WRITE) == FR_OK)
//										{//write redline
//											sprintf(buffer, "Data\t\tTime\t\tVoltage\tCurrent\tWatt\n");
//											if(f_lseek(&fil, f_size(&fil)) == FR_OK){};
//												
//												/* If we put more than 0 characters (everything OK) */
//												if (f_puts(buffer, &fil) > 0) {
//													if (TM_FATFS_DriveSize(&total, &free_fat) == FR_OK) {
//														/* Data for drive size are valid */
//														/* Close file, don't forget this! */
//														f_close(&fil);
//													}
//											}
//										}
//									}
//								else f_close(&fil);//file exists, was openned and must be closed
//									
//							sprintf(file_name_tempr, "0:%04d_%02d_%02d_Tempr_Pr.txt", datatime.year+2000, datatime.month, datatime.date);
//							temp_sd_res = f_open(&fil, (TCHAR*) file_name_tempr, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
//							if (temp_sd_res != FR_OK) 
//								{
//									if (f_open(&fil, file_name_tempr, FA_CREATE_NEW | FA_READ | FA_WRITE) == FR_OK)
//										{//write redline
//											sprintf(buffer, "Data\t\tTime\t\tTempr\tPresure\n");
//											if(f_lseek(&fil, f_size(&fil)) == FR_OK){};
//												
//												/* If we put more than 0 characters (everything OK) */
//												if (f_puts(buffer, &fil) > 0) {
//													if (TM_FATFS_DriveSize(&total, &free_fat) == FR_OK) {
//														/* Data for drive size are valid */
//														/* Close file, don't forget this! */
//														f_close(&fil);
//													}
//											}
//										}
//									}
//								else f_close(&fil);//file exists, was openned and must be closed
					
					
//					temp_sd_res = f_open(&fil, "0:Tempr.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
//					if (temp_sd_res != FR_OK) 
//						{
//							if (f_open(&fil, "0:Tempr.txt", FA_CREATE_NEW | FA_READ | FA_WRITE) == FR_OK)
//								{//write redline
//									sprintf(buffer, "Data\t\tTime\t\tVoltage\tTempr\tPresure\n");
//									if(f_lseek(&fil, f_size(&fil)) == FR_OK){};
//										
//										/* If we put more than 0 characters (everything OK) */
//										if (f_puts(buffer, &fil) > 0) {
//											if (TM_FATFS_DriveSize(&total, &free_fat) == FR_OK) {
//												/* Data for drive size are valid */
//												/* Close file, don't forget this! */
//												f_close(&fil);
//											}
//									}
//								}
//							}
//						else f_close(&fil);//file exists, was openned and must be closed
//							
//							/* Unmount drive, don't forget this! */
//							f_mount(0, "0:", 1);
//					
//				}//end mount SD
				

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(UserTask, UserDefaultTask, osPriorityNormal, 0, 256);
  UserTaskHandle = osThreadCreate(osThread(UserTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}







/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  HAL_TIM_OC_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pins : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	  /*Configure GPIO pins : PB15 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	

}

/* USER CODE BEGIN 4 */


void LM75_RW(void)
{
	char str[50];
	uint16_t temp16;
	
	temp16 = LM75_Temperature();
	
	str[0] = LM75_ReadConf();
	LM75_WriteConf(str[0] & 0xFE);
	
//	adc_Y = LM75_ReadReg(LM75_REG_CONF);
//	adc_Z = LM75_ReadReg(LM75_REG_THYS);
	
	sprintf(str, "x= %d\n", temp16);
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	uint8_t flag_buttom_press = 0;

	


  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	
  /*Configure GPIO pins : PB15 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//enable MMA7264

		
	while(1)
	{
		Work_with_ADC();
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		LM75_RW();
		osDelay(100);
		l3g4200_temp = L3G4200_ReadReg(L3GD20_REG_WHO_AM_I);
		
		
		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1) && (flag_buttom_press == 0))
			{	
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				flag_buttom_press = 1;
			}
		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0) && (flag_buttom_press == 1))
		{
			flag_buttom_press = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	}
	
//  for(;;)
//  {
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
//    osDelay(1000);
//		
//		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1) && (flag_buttom_press == 0))
//			{	
//			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//				flag_buttom_press = 1;
//			}
//		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0) && (flag_buttom_press == 1))
//			flag_buttom_press = 0;
//		
//  }
  /* USER CODE END 5 */ 
}


void UserDefaultTask(void const * argument)
{
	while(1)
	{
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		osDelay(100);
	}
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */
	
	

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
