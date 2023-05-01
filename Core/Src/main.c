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
void TMP_Config(void);
void SPI_Config(void);
void UART_Config(void);
void SD_Setup(void);
void MEM_Setup(void);

uint8_t get_pin_available(void* hal_resource);
uint8_t get_pin_locked(void* hal_resource);
void select_card(void* hal_resource);
void unselect_card(void* hal_resource);
void set_freq_high(void* hal_resource);
uint8_t sd_raw_rec_byte(void* hal_resource);
void sd_raw_send_byte(void* hal_resource, uint8_t tx_data);

ADC_HandleTypeDef hadc;
GPIO_InitTypeDef GPIO_Init;
SPI_HandleTypeDef hspi;
UART_HandleTypeDef huart;
SPI_GPIO_HAL_Resource bb_spi_dev;
SD_Device sd;
MEM_Device sd_mem_device;

volatile intptr_t n;

#define noop

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  TMP_Config();
  ADC_Config();
  SPI_Config();
  UART_Config();
  SD_Setup();
  

  sd_raw_init(&sd);
  partition_struct* part = partition_open(&sd_mem_device, 0);
  part->type = PARTITION_TYPE_FAT32;
  part->offset = 0x0;
  part->length = 229297;
  fat_fs_struct* fat_fs = fat_open(part);
  fat_dir_entry_struct file_entry;
  fat_get_dir_entry_of_path(fat_fs, "/hello_world.txt", &file_entry);
  fat_file_struct* fd = fat_open_file(fat_fs, &file_entry);
  uint8_t file_buffer[512];
  n = fat_read_file(fd, file_buffer, 512);

  while (1)
  {

    // HAL_ADCEx_Calibration_Start(&hadc);
    // HAL_ADC_Start(&hadc);
    // if (HAL_ADC_PollForConversion(&hadc, 5000) == HAL_OK)
    // {
    //   ADCValue = HAL_ADC_GetValue(&hadc);
    //   uint16_t TS_CAL1 = 600;
    //   int32_t TS_CAL1_TEMP = 30;
    //   uint16_t TS_CAL2 = 520;
    //   int32_t TS_CAL2_TEMP = 20; // room
    //   temp_c = (float)(TS_CAL2_TEMP - TS_CAL1_TEMP)/(TS_CAL2 - TS_CAL1) * (ADCValue - TS_CAL1) + TS_CAL1_TEMP;
    // }
    // HAL_ADC_Stop(&hadc);
    // HAL_Delay(5);
    
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
}

void TMP_Config()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_Init.Pin = GPIO_PIN_0;
  GPIO_Init.Mode = GPIO_MODE_ANALOG;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
}

void SPI_Config()
{
  bb_spi_dev.GPIO = GPIOB;
  bb_spi_dev.GPIO_Pin = GPIO_PIN_12;
  bb_spi_dev.hspi = &hspi;

  hspi.Instance = SPI2;
  hspi.Init.Mode = SPI_MODE_MASTER;
  hspi.Init.Direction = SPI_DIRECTION_2LINES;
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi.Init.NSS = SPI_NSS_SOFT;
  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  HAL_SPI_Init(&hspi);

  GPIO_Init.Pin = bb_spi_dev.GPIO_Pin;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(bb_spi_dev.GPIO, &GPIO_Init);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  __HAL_RCC_SPI2_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_Init.Pin = GPIO_PIN_12;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_13;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_14;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
  GPIO_Init.Pin = GPIO_PIN_15;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_Init);
}

void UART_Config()
{
  
}

void SD_Setup()
{
  /* struct which sd_raw will pass when calling hal functions */
  sd.hal_resource = &bb_spi_dev;
  /* functions sd_raw uses to talk to hal */
  sd.get_pin_available = get_pin_available;
  sd.get_pin_locked = get_pin_locked;
  sd.select_card = select_card;
  sd.unselect_card = unselect_card;
  sd.set_freq_high = set_freq_high;
  sd.sd_raw_rec_byte = sd_raw_rec_byte;
  sd.sd_raw_send_byte = sd_raw_send_byte;
  /* define underlying functions for partition manager */
  sd_mem_device.context = &sd;
  sd_mem_device.device_read = sd_raw_read;
  sd_mem_device.device_read_interval = sd_raw_read_interval;
  sd_mem_device.device_write = sd_raw_write;
  sd_mem_device.device_write_interval = sd_raw_write_interval;
}

uint8_t get_pin_available(void* hal_resource)
{
  return 0x00;
}
uint8_t get_pin_locked(void* hal_resource)
{
  return 0x01;
}
void select_card(void* hal_resource)
{
  SPI_GPIO_HAL_Resource* hr = (SPI_GPIO_HAL_Resource*) hal_resource;
  HAL_GPIO_WritePin(hr->GPIO, hr->GPIO_Pin, GPIO_PIN_RESET);
}
void unselect_card(void* hal_resource)
{
  SPI_GPIO_HAL_Resource* hr = (SPI_GPIO_HAL_Resource*) hal_resource;
  HAL_GPIO_WritePin(hr->GPIO, hr->GPIO_Pin, GPIO_PIN_SET);
}
void set_freq_high(void* hal_resource)
{

}
uint8_t sd_raw_rec_byte(void* hal_resource)
{
  SPI_GPIO_HAL_Resource* hr = (SPI_GPIO_HAL_Resource*) hal_resource;
  uint8_t tx_data = 0xff;
  uint8_t rx_data = 0x00;
  HAL_SPI_TransmitReceive(hr->hspi, &tx_data, &rx_data, 1, 5000);
  return rx_data;
}
void sd_raw_send_byte(void* hal_resource, uint8_t tx_data){
  SPI_GPIO_HAL_Resource* hr = (SPI_GPIO_HAL_Resource*) hal_resource;
  uint8_t rx_data = 0x00;
  HAL_SPI_TransmitReceive(hr->hspi, &tx_data, &rx_data, 1, 5000);
}

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
