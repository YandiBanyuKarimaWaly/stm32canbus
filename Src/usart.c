/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "cobs.h"

volatile bool uart1_tx_lock = false;
void uart_tx_dma_finished(UART_HandleTypeDef *huart);
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_RegisterCallback(&huart1, HAL_UART_TX_COMPLETE_CB_ID, uart_tx_dma_finished);
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void uart_tx_dma_finished(UART_HandleTypeDef *huart)
{
  uart1_tx_lock = false;
}

bool serial_write_dma(uint8_t *data, uint16_t size)
{
  if (uart1_tx_lock == true)
  {
    return false;
  }

  return HAL_UART_Transmit_DMA(&huart1, data, size) == HAL_OK;
}

bool serial_printf_dma(const char *format, ...)
{
  //                     header, string,      null
  static uint8_t printed[1 + MAX_PRINT_CHAR + 1]; // Header
  static uint8_t encoded[COBS_ENCODE_MAX(1 + MAX_PRINT_CHAR + 1)];
  static size_t encoded_size = 0;

  bool result = false;
  if (uart1_tx_lock == true)
  {
    return result;
  }

  va_list args;
  va_start(args, format);
  int written = vsnprintf((char *)(1 + printed), MAX_PRINT_CHAR + 1, format, args);
  if (written >= 0)
  {
    const size_t packet_size = 1 + (written < MAX_PRINT_CHAR ? written : MAX_PRINT_CHAR) + 1; // Header + string + null-byte

    if (cobs_encode(printed, packet_size, encoded, sizeof(encoded), &encoded_size) == COBS_RET_SUCCESS)
    {
      result = serial_write_dma(encoded, encoded_size);
    }
  }
  va_end(args);
  return result;
}

bool serial_cantx_dma(CAN_TxHeaderTypeDef *header, uint8_t payload[8])
{
  static uint8_t packet[15];

  bool result = false;
  if (uart1_tx_lock == true)
  {
    return result;
  }

  packet[0] = COBS_INPLACE_SENTINEL_VALUE;
  packet[1] = (0b1000 << 4) + (header->IDE & 0x0F << 4) + (header->RTR & 0x0F << 4) + (header->TransmitGlobalTime << 4) + ((header->DLC & 0x0F) + 1);
  packet[2] = header->ExtId >> 24 & 0xFF;
  packet[3] = header->ExtId >> 16 & 0xFF;
  packet[4] = header->ExtId >> 8 & 0xFF;
  packet[5] = header->ExtId >> 0 & 0xFF;
  packet[6] = payload[0];
  packet[7] = payload[1];
  packet[8] = payload[2];
  packet[9] = payload[3];
  packet[10] = payload[4];
  packet[11] = payload[5];
  packet[12] = payload[6];
  packet[13] = payload[7];
  packet[14] = COBS_INPLACE_SENTINEL_VALUE;

  if (cobs_encode_inplace(packet, sizeof(packet)) == COBS_RET_SUCCESS)
  {
    result = serial_write_dma(packet, sizeof(packet));
  }

  return result;
}

bool serial_canrx_dma(CAN_RxHeaderTypeDef *header, uint8_t payload[8])
{
  static uint8_t packet[20];

  bool result = false;
  if (uart1_tx_lock == true)
  {
    return result;
  }

  packet[0] = COBS_INPLACE_SENTINEL_VALUE;
  packet[1] = (header->IDE & 0xFF << 4) + (header->RTR & 0xFF << 4) + ((header->DLC & 0xFF) + 1);
  packet[2] = header->ExtId >> 24 & 0xFF;
  packet[3] = header->ExtId >> 16 & 0xFF;
  packet[4] = header->ExtId >> 8 & 0xFF;
  packet[5] = header->ExtId >> 0 & 0xFF;
  packet[6] = payload[0];
  packet[7] = payload[1];
  packet[8] = payload[2];
  packet[9] = payload[3];
  packet[10] = payload[4];
  packet[11] = payload[5];
  packet[12] = payload[6];
  packet[13] = payload[7];
  packet[14] = header->FilterMatchIndex & 0xFF;
  packet[15] = header->Timestamp >> 24 & 0xFF;
  packet[16] = header->Timestamp >> 16 & 0xFF;
  packet[17] = header->Timestamp >> 8 & 0xFF;
  packet[18] = header->Timestamp >> 0 & 0xFF;
  packet[19] = COBS_INPLACE_SENTINEL_VALUE;

  if (cobs_encode_inplace(packet, sizeof(packet)) == COBS_RET_SUCCESS)
  {
    result = serial_write_dma(packet, sizeof(packet));
  }

  return result;
}
/* USER CODE END 1 */
