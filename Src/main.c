/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "cobs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// To make your life easier.
enum AvailableTask
{
  PrintHeartbeat,
  ReceiveSerial,
  ReceiveCAN,

  SendBatteryVoltage,
  SendBatteryCurrent,
  SendBatteryTemperature,
  SendFuelLevel,
  SendAmbientTemperature,
  SendAmbientHumidity,
  SendEngineTemperature,
  SendOilTemperature,
  SendOilPressure,
  SendCoolantTemperature,
  SendCoolantPressure,

  EOT, // DO NOT USE THIS. THIS IS FOR INDEX TRACKING. ALWAYS PUT AT END!
};

// Comment here as needed
const uint8_t EnabledTask[] = {
    PrintHeartbeat,
    // ReceiveSerial,
    // ReceiveCAN,

    SendBatteryVoltage,
    SendBatteryCurrent,
    SendBatteryTemperature,
    SendFuelLevel,
    // SendAmbientTemperature,
    // SendAmbientHumidity,
    // SendEngineTemperature,
    // SendOilTemperature,
    // SendOilPressure,
    // SendCoolantTemperature,
    // SendCoolantPressure,
};

typedef bool (*TaskFunction)();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BITS16(b1, b2) ((b1 << 8) + b2)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void receiveCAN(CAN_HandleTypeDef *hcan);

/**
 * @brief Execute a task list within interval. This is done very crudely.
 *
 * @param timekeeper object for storing timekeeping value
 * @param interval_ms interval of each task
 * @param functor list of task to be done
 * @param task index of which task to be done
 */
void do_task(uint32_t timekeeper[], uint32_t interval_ms[], TaskFunction functors[], uint8_t task);

bool printHeartbeat();
bool receiveSerial();

bool sendRandom();
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
  uint32_t timekeeper[EOT] = {0};
  uint32_t interval[EOT] = {0};
  TaskFunction tasks[EOT] = {0};

  {
    tasks[PrintHeartbeat] = printHeartbeat;
    interval[PrintHeartbeat] = 1000;

    // Both should always be run
    tasks[ReceiveSerial] = receiveSerial;
    interval[ReceiveSerial] = 0;

    tasks[ReceiveCAN] = receiveCAN;
    interval[ReceiveCAN] = 0;

    tasks[SendBatteryVoltage] = sendRandom;
    tasks[SendBatteryCurrent] = sendRandom;
    tasks[SendBatteryTemperature] = sendRandom;
    tasks[SendAmbientTemperature] = sendRandom;
    tasks[SendAmbientHumidity] = sendRandom;
    tasks[SendEngineTemperature] = sendRandom;
    tasks[SendOilTemperature] = sendRandom;
    tasks[SendOilPressure] = sendRandom;
    tasks[SendFuelLevel] = sendRandom;
    tasks[SendCoolantTemperature] = sendRandom;
    tasks[SendCoolantPressure] = sendRandom;

    interval[SendBatteryVoltage] = 251;
    interval[SendBatteryCurrent] = 237;
    interval[SendBatteryTemperature] = 272;
    interval[SendAmbientTemperature] = 228;
    interval[SendAmbientHumidity] = 293;
    interval[SendEngineTemperature] = 179;
    interval[SendOilTemperature] = 204;
    interval[SendOilPressure] = 191;
    interval[SendFuelLevel] = 335;
    interval[SendCoolantTemperature] = 372;
    interval[SendCoolantPressure] = 396;
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_RCC_GetSysClockFreq();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Application Note STM32 UM1850 #9.2 - CAN Firmware driver API description (page 92)
  HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, receiveCAN);
  CAN_FilterTypeDef filter;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan, &filter);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for (size_t i = 0; i < sizeof(EnabledTask); i++)
    {
      do_task(timekeeper, interval, tasks, EnabledTask[i]);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void do_task(uint32_t timekeeper[], uint32_t interval_ms[], TaskFunction functors[], uint8_t task)
{
  // Task is not yet defined
  if (functors[task] != 0 && functors[task] != NULL)
  {
    // USE INT TO MAKE SURE WE CAN GO BELOW 0
    const int64_t tick_ms = HAL_GetTick();
    if (timekeeper[task] - tick_ms <= 0)
    {
      if (functors[task]() == true)
      {
        timekeeper[task] = tick_ms + interval_ms[task];
      }
    }
  }
}

bool printHeartbeat()
{
  switch (HAL_CAN_GetState(&hcan))
  {
  case HAL_CAN_STATE_RESET:
    return serial_printf_dma("HAL_CAN_STATE_RESET");
  case HAL_CAN_STATE_READY:
    return serial_printf_dma("HAL_CAN_STATE_READY");
  case HAL_CAN_STATE_LISTENING:
    return serial_printf_dma("HAL_CAN_STATE_LISTENING");
  case HAL_CAN_STATE_ERROR:
    return serial_printf_dma("HAL_CAN_STATE_ERROR");
  case HAL_CAN_STATE_SLEEP_ACTIVE:
    return serial_printf_dma("HAL_CAN_STATE_SLEEP_ACTIVE");
  case HAL_CAN_STATE_SLEEP_PENDING:
    return serial_printf_dma("HAL_CAN_STATE_SLEEP_PENDING");
  default:
    return serial_printf_dma("HAL_CAN_STATE_UNKNOWN");
  }
}

bool receiveSerial()
{
  return false;
}

void receiveCAN(CAN_HandleTypeDef *hcan)
{
  static CAN_RxHeaderTypeDef header;
  static uint8_t data[8];

  uint32_t fill = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
  for (size_t j = 0; j < fill; j++)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK)
    {
      serial_canrx_dma(&header, data);
    }
  }

  fill = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);
  for (size_t j = 0; j < fill; j++)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) == HAL_OK)
    {
      serial_canrx_dma(&header, data);
    }
  }
}

bool sendCANFrame(CAN_TxHeaderTypeDef *header, uint8_t *data)
{
  uint32_t mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
  if (mailbox == 0)
  {
    return false;
  }

  // serial_cantx_dma(header, data);

  return HAL_CAN_AddTxMessage(&hcan, header, data, &mailbox) == HAL_OK;
}

// Assumes 0 <= max <= RAND_MAX
// Returns in the closed interval [0, max]
long random_at_most(long max)
{
  unsigned long
      // max <= RAND_MAX < ULONG_MAX, so this is okay.
      num_bins = (unsigned long)max + 1,
      num_rand = (unsigned long)RAND_MAX + 1,
      bin_size = num_rand / num_bins,
      defect = num_rand % num_bins;

  long x;
  do
  {
    x = random();
  }
  // This is carefully written not to overflow
  while (num_rand - defect <= (unsigned long)x);

  // Truncated division is intentional
  return x / bin_size;
}

bool sendRandom()
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];

  bool IDE = random_at_most(true);
  bool RTR = random_at_most(true);
  uint32_t StdId = random_at_most(0x07FF);
  uint32_t ExtId = random_at_most(0x1FFFFFFF);
  uint32_t DLC = random_at_most(8);

  header.DLC = DLC;
  header.ExtId = IDE == true ? ExtId : 0;
  header.IDE = IDE == true ? CAN_ID_EXT : CAN_ID_STD;
  header.RTR = RTR == true ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  header.StdId = StdId;
  header.TransmitGlobalTime = DISABLE;

  for (size_t i = 0; i < DLC; i++)
  {
    data[i] = random_at_most(255);
  }

  return sendCANFrame(&header, data);
}
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

#ifdef USE_FULL_ASSERT
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
