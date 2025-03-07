/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "e22900t22d.h"
#include "FreeRTOS.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId toggle_ledHandle;
osThreadId handle_e22Handle;
osThreadId e22_transmitHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);

/* Make sure function declarations match the implementations */
void led_toggle_task(void * argument);
void e22_handle_task(void * argument);
void e22_transmission_task(void * argument);

/* USER CODE BEGIN PFP */
static void main_e22_transceiverMode(void);
static void main_e22_configurationMode(void);
static void main_lora_packet_receive(uint8_t* dataPacket, uint8_t size);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //Initialize e22 LoRa module
  e22_lora_init(&huart2,
		  	  	HAL_UART_Transmit_DMA,
				HAL_UARTEx_ReceiveToIdle_DMA,
				main_lora_packet_receive,
				main_e22_configurationMode,
				main_e22_transceiverMode);

  /* OPTION 1: USE EITHER FreeRTOS API directly */
  /* Comment out this section if using CMSIS-RTOS API below */
  /*
  //Start FreeRTOS task creation
  xTaskCreate(led_toggle_task, "Toggle GPIO13", 128, NULL, 1, NULL);
  xTaskCreate(e22_handle_task, "E22 LoRa Handler", 128 * 4, NULL, 1, NULL);
  xTaskCreate(e22_transmission_task, "E22 LoRa Tx Task", 128 * 4, NULL, 1, NULL);
  vTaskStartScheduler();
  */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* OPTION 2: USE CMSIS-RTOS API */
  /* Create the thread(s) */
  /* definition and creation of toggle_led */
  osThreadDef(toggle_led, led_toggle_task, osPriorityNormal, 0, 128);
  toggle_ledHandle = osThreadCreate(osThread(toggle_led), NULL);

  /* definition and creation of handle_e22 */
  osThreadDef(handle_e22, e22_handle_task, osPriorityNormal, 0, 512); // Increased stack size
  handle_e22Handle = osThreadCreate(osThread(handle_e22), NULL);

  /* definition and creation of e22_transmit */
  osThreadDef(e22_transmit, e22_transmission_task, osPriorityNormal, 0, 512); // Increased stack size
  e22_transmitHandle = osThreadCreate(osThread(e22_transmit), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M0_Pin M1_Pin */
  GPIO_InitStruct.Pin = M0_Pin|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief 	LoRa module transceiver mode selection.  when this mode is active, the module configuration can't be modified.
 *
 */
static void main_e22_transceiverMode(void)
{
	HAL_GPIO_WritePin(GPIOB, M0_Pin|M1_Pin, GPIO_PIN_RESET);
}
/**
 * @brief 	LoRa module config mode selection. when this mode is active, wireless communication is inactive.
 *
 */
static void main_e22_configurationMode(void)
{
	HAL_GPIO_WritePin(GPIOB, M0_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOB, M1_Pin, GPIO_PIN_SET);
}
/**
 * @brief 	UART data transmission complete callback over DMA
 *
 * @param 	huart	:	Pointer to the UART handler
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//Change the TX line state to ready
	e22_lora_make_ready();
}
/**
 * @brief 	UART data reception complete callback over DMA
 *
 * @param 	huart	:	Pointer to the UART handler
 *
 * @param 	Size	:	Packet size received over UART
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	e22_lora_reception_complete(Size);
}
/**
 * @brief 	This function is a callback to receives a LoRa packet by copying it from the data packet buffer to the local LoRa packet buffer. It then
 * 			copies the LoRa packet to the main layer.
 *
 * @param 	dataPacket	:	dataPacket Pointer to the buffer containing the received data packet.
 *
 * @param 	size		:	size of the received data packet.
 */
static void main_lora_packet_receive(uint8_t* dataPacket, uint8_t size)
{
	uint8_t loraPacket[MAX_DATA_PACKET_SIZE] = {0};
	//Copy data to the main layer
	memcpy(&loraPacket, dataPacket, size);
	// TODO: implement main layer packet handling

	if(0 == memcmp(loraPacket, "pong", size))
	{
		HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_led_toggle_task */
/**
  * @brief  Function implementing the toggle_led thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_led_toggle_task */

void led_toggle_task(void *argument)
{
  /* USER CODE BEGIN 5 */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); // 1000ms delay

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_e22_handle_task */
/**
* @brief Function implementing the handle_e22 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_e22_handle_task */
void e22_handle_task(void *argument)
{
  /* USER CODE BEGIN e22_handle_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END e22_handle_task */
}

/* USER CODE BEGIN Header_e22_transmission_task */
/**
* @brief Function implementing the e22_transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_e22_transmission_task */
void e22_transmission_task(void *argument)
{
  /* USER CODE BEGIN e22_transmission_task */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);
    const uint8_t packetSize = 4;
    const uint8_t receiverAddress = 0x09;
    const uint8_t ComChannel = 0x12;
    uint8_t packet[5] = "ping";

  /* Infinite loop */
  for(;;)
  {
    e22_lora_transnit(packet, packetSize, receiverAddress, ComChannel);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END e22_transmission_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
