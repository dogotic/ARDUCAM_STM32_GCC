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
#include <stdio.h>
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "ArduCAM.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char buf[2048] = {0};
int cntr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t rx, rx_prev;
  FATFS FatFs;  // Fatfs handle
  FRESULT fres; // Result after operations
  FIL image;

  /* MCU Configuration--------------------------------------------------------*/
  hw_init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(500); // a short delay is important to let the SD card settle

  // Open the file system
  fres = f_mount(&FatFs, "", 1); // 1=mount now
  if (fres != FR_OK)
  {
    sprintf(buf, "f_mount error (%i)\r\n", fres);
    HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);
    while (1)
      ;
  }

  // Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS *getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK)
  {
    sprintf(buf, "f_getfree error (%i)\r\n", fres);
    HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);
  }

  // Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  sprintf(buf, "SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);
  HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);

  uint8_t vid, pid;
  uint8_t Camera_WorkMode = 0;
  uint8_t start_shoot = 0;
  uint8_t stop = 0;

  while (1)
  {
    CS_LOW();
    uint8_t tx_data[2] = {0x80 | ARDUCHIP_TEST1, 0x55};
    uint8_t rx_data = 0x00;
    HAL_SPI_Transmit(&hspi1, tx_data, 2, 100);
    HAL_SPI_Receive(&hspi1, &rx_data, 1, 100);
    CS_HIGH();
    if (rx_data == 0x55)
      break;
  }
  HAL_UART_Transmit(&huart1, "SPI OK\r\n", strlen("SPI OK\r\n"), 100);

  while (1)
  {
    sensor_addr = 0x60;
    wrSensorReg8_8(0xff, 0x01);
    rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42)))
    {
      sprintf(buf, "ACK CMD Can't find OV2640 module!\r\n");
      // HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);
    }
    else
    {
      sensor_model = OV2640;
      sprintf(buf, "ACK CMD OV2640 detected.\r\n");
      HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);

      break;
    }
    sensor_addr = 0x78;
    rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
    rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x40))
    {
      sprintf(buf, "ACK CMD Can't find OV5640 module!\r\n");
      HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);
    }
    else
    {
      sensor_model = OV5640;
      sprintf(buf, "ACK CMD OV5640 detected.\r\n");
      HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);

      break;
    }
    sensor_addr = 0x78;
    rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42))
    {
      sprintf(buf, "ACK CMD Can't find OV5642 module!\r\n");
      HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);

      continue;
    }
    else
    {
      sensor_model = OV5642;
      sprintf(buf, "ACK CMD OV5642 detected.\r\n");
      HAL_UART_Transmit(&huart1, buf, strlen(buf), 50);

      break;
    }
  }

  bool start_writing = false;

  set_format(JPEG);
  ArduCAM_Init(sensor_model);
  clear_fifo_flag();
  write_reg(ARDUCHIP_FRAMES, 0);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    flush_fifo();
    clear_fifo_flag();
    OV2640_set_JPEG_size(OV2640_1600x1200);
    OV264_set_light_mode(LIGHT_MODE_Auto);
    start_capture();
    while (!get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      ;
    }
    int length = read_fifo_length();

    for (int i = 0; i < length; i++)
    {
      uint8_t bytes_written = 0;
      rx_prev = rx;
      rx = read_fifo();
      if ((rx_prev == 0xFF) && (rx == 0xD8))
      {
        HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_SET);
        sprintf(buf, "image_%d.jpg", cntr);
        f_open(&image, buf, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
        start_writing = true;
        cntr++;

        sprintf(buf, "START OF FILE DETECTED\r\n");
        HAL_UART_Transmit(&huart1, buf, strlen(buf), 100);

        f_write(&image, &rx_prev, 1, &bytes_written);
        f_write(&image, &rx, 1, &bytes_written);
      }
      if ((rx_prev == 0xFF) && (rx == 0xD9))
      {
        sprintf(buf, "END OF FILE DETECTED\r\n\r\n");
        HAL_UART_Transmit(&huart1, buf, strlen(buf), 100);
        f_write(&image, &rx_prev, 1, &bytes_written);
        f_write(&image, &rx, 1, &bytes_written);
        f_close(&image);
        start_writing = false;
        HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
      }

      if (start_writing)
      {
        f_write(&image, &rx, 1, &bytes_written);
      }
    }
    HAL_Delay(60000);
  }
  f_mount(NULL, "", 0);
}
