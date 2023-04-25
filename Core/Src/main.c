/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

void SystemClock_Config(void);
void ADC_Config(void);
void GPIO_Config(void);

ADC_HandleTypeDef hadc;
GPIO_InitTypeDef GPIO_Init;

__IO uint16_t ADCValue=0;
__IO float temp_c=0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  // GPIO_Config();
  ADC_Config();

  while (1)
  {
    HAL_ADCEx_Calibration_Start(&hadc);
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 5000) == HAL_OK)
    {
      ADCValue = HAL_ADC_GetValue(&hadc);
      uint16_t TS_CAL1 = *(uint16_t*) (0x1FFFF7B8U); //1759
      int32_t TS_CAL1_TEMP = 30;
      uint16_t TS_CAL2 = 1802; 
      int32_t TS_CAL2_TEMP = 20; // room 
      temp_c = (float)(TS_CAL2_TEMP - TS_CAL1_TEMP)/(TS_CAL2 - TS_CAL1) * (ADCValue - TS_CAL1) + TS_CAL1_TEMP;
    }
    HAL_ADC_Stop(&hadc);
    HAL_Delay(5);
    
    // read temp from sensor

    // write to sd card
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // HSI at 8Mhz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB 8Mhz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB 8Mhz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void ADC_Config()
{
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // 2Mhz
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
}

void GPIO_Config()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_Init.Pin = GPIO_PIN_14;
  GPIO_Init.Mode = GPIO_MODE_ANALOG;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  // GPIO_Init.Alternate;

  HAL_GPIO_Init(GPIOA, &GPIO_Init);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
