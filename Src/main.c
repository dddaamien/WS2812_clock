#include "stm32f1xx_hal.h"
#include <stdint-gcc.h>
#include <time.h>
#include "WS2812.h"

RTC_HandleTypeDef hrtc;


void delay_cycles(int nbCycles);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);


int main(void)
{
	RTC_TimeTypeDef sTime;
//	uint8_t clockDisp[180]={0};
	struct structWs2812GRB clockDisp[60];
	struct structWs2812GRB hhColor, minColor, secColor;

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_RTC_Init();

	struct structWs2812HSV Shsv={90,127,127};
	HSVtoRGB(&hhColor,&Shsv);
	Shsv.hue=30;//={30,127,127};
	Shsv.saturation=127;
	Shsv.value=127;
	HSVtoRGB(&minColor,&Shsv);
	Shsv.hue=230;
	Shsv.saturation=127;
	Shsv.value=127;
	HSVtoRGB(&secColor,&Shsv);

	sTime.Hours = 16;
	sTime.Minutes = 15;
	sTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
	while (1)
	{
		/*HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		GPIOC->BSRR = GPIO_PIN_13;
		GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16;*/

		ws2812_sendarray(&clockDisp,180);
		HAL_Delay(500);
		for(int i=0;i<60;++i)
		{
			clockDisp[i].red=0;//rand() % 0xFF;
			clockDisp[i].green=0;
			clockDisp[i].blue=0;
		}

		HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		clockDisp[sTime.Seconds] = secColor;
		clockDisp[sTime.Minutes] = minColor;
		clockDisp[((sTime.Hours) % 12) * 5] = hhColor;
	}
}



void delay_cycles(int nbCycles)
{
	while(--nbCycles);
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_RCC_EnableCSS();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 12;
  sTime.Minutes = 10;
  sTime.Seconds = 0x0;

  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 30;
  DateToUpdate.Year = 16;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BCD);

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
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : LedR_Pin
  GPIO_InitStruct.Pin = LedR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; //GPIO_SPEED_LOW;
  HAL_GPIO_Init(LedR_GPIO_Port, &GPIO_InitStruct);*/

  /*Configure GPIO pin : PB4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
