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

ADC_HandleTypeDef hadc1;
__IO uint16_t ADCValue=0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();


  // HAL_ADC_Init(&hadc);
  HAL_ADC_Start(&hadc1);
  while (1)
  {
    if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
    {
        ADCValue = HAL_ADC_GetValue(&hadc1);
    }
    // HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
    // raw = HAL_ADC_GetValue(&hadc);
    HAL_Delay(5);
    // volatile uint32_t c = *(uint32_t *)0x40012440;
    // turn on ADC
    // my_ADC_init();
    
    // read temp from sensor

    // write to sd card
  }
}



// void my_ADC_init(void)
// {
//   // ADRDY
//   // char * p = (char *)0x40012400;
//   // *p = 0x1;

//   //ADCAL
//   p = (uint32_t *)0x40012408;
//   *p |= (1 << 31);

//   volatile int count = 0;
//   while ((*p & (1 << 31)) == (1 << 31))
//   {
//     count++;
//   }
//   c = *(uint32_t *)0x40012440;

//   // ADEN
//   p = (uint32_t *)0x40012408;
//   *p |= 1;
//   // c = *p;

//   p = (uint32_t *)0x40012400;
//   while ((*p & 1) == 0)
//   {
//   }
//   c = *p;

// }


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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
