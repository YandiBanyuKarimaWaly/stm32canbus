/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define MAX_PRINT_CHAR 100
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief Send raw data to the serial
 * 
 * @param data data pointer
 * @param size size of data
 * @return true 
 * @return false 
 */
bool serial_write_dma(uint8_t *data, uint16_t size);

/**
 * @brief Print format to the serial. Configure buffer size from define
 *
 * @param format string format
 * @param ... argument for string format in the order of its occurence
 * @return true 
 * @return false 
 */
bool serial_printf_dma(const char *format, ...);

/**
 * @brief Print a CAN frame to the serial
 *
 * @param header CAN frame header
 * @param payload CAN frame payload
 * @return true 
 * @return false 
 */
bool serial_cantx_dma(CAN_TxHeaderTypeDef *header, uint8_t payload[8]);

/**
 * @brief Print a CAN frame to the serial
 *
 * @param header CAN frame header
 * @param payload CAN frame payload
 * @return true 
 * @return false 
 */
bool serial_canrx_dma(CAN_RxHeaderTypeDef *header, uint8_t payload[8]);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

