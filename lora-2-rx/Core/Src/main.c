/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LoRa Receiver for STM32F303 (example)
  ******************************************************************************
  * @attention
  * This code listens for:
  *   [AddrHigh] [AddrLow] [Channel] [Payload bytes...] [RSSI]
  * The final byte is the auto-appended RSSI by the LoRa module.
  *
  * We read the first 3 bytes for (Addr,Addr,Channel).
  * Then we read payload until a short timeout.
  * The last byte we read is considered RSSI.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "SH1106.h"
#include "fonts.h"

/* USER CODE BEGIN PV */
#define MAX_PAYLOAD_LEN  64   // Adjust as needed
uint8_t rxTemp;               // For receiving one byte at a time
uint8_t rxBuffer[MAX_PAYLOAD_LEN+1];
// We'll store the payload in rxBuffer, then interpret last as RSSI.
/* USER CODE END PV */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1; // LoRa module connected here
UART_HandleTypeDef huart2; // Debug prints via printf

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */
/**
  * Simple retarget of printf to huart2 for debug logs.
  */
int _write(int file, char *ptr, int len)
{
  if (file == 1 || file == 2) // stdout or stderr
  {
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : EIO;
  }
  errno = EBADF;
  return -1;
}

/**
  * @brief  Receives one "packet" of data from the LoRa module:
  *         [AddrHi][AddrLo][Channel][Payload...][RSSI].
  *         Returns 1 if a packet was successfully read, 0 if no data was received.
  */
int ReceiveLoRaPacket(void)
{
  // 1) Read 3 bytes for address + channel, each with e.g. 1000ms timeout
  if (HAL_UART_Receive(&huart1, &rxTemp, 1, 1000) != HAL_OK) return 0;
  uint8_t addrHi = rxTemp;

  if (HAL_UART_Receive(&huart1, &rxTemp, 1, 100) != HAL_OK) return 0;
  uint8_t addrLo = rxTemp;

  if (HAL_UART_Receive(&huart1, &rxTemp, 1, 100) != HAL_OK) return 0;
  uint8_t channel = rxTemp;

  // 2) Now read payload bytes until we time out. The last byte is the RSSI.
  //    We'll store all in rxBuffer. We'll keep reading until a short gap (e.g. 50ms).
  //    The typical pattern is: the LoRa module will quickly push all payload plus 1 RSSI byte.
  //    We'll assume the final byte we read is the RSSI.

  uint16_t payloadLen = 0;
  while (1)
  {
    // Attempt to read 1 more byte with short timeout
    if (HAL_UART_Receive(&huart1, &rxTemp, 1, 30) == HAL_OK)
    {
      // We read a new byte
      if (payloadLen < MAX_PAYLOAD_LEN)
      {
        rxBuffer[payloadLen++] = rxTemp;
      }
      else
      {
        // Overflow. We'll ignore further bytes but keep reading until the burst ends
        // or we can break.
      }
    }
    else
    {
      // Timed out => no more data in this packet
      break;
    }
  }

  // If we got no payload bytes, that means we only read the 3 header bytes.
  if (payloadLen == 0)
  {
    // Possibly just an empty message?
    // We'll still show something, but there's no RSSI or payload.
    printf("Got address=0x%02X%02X, ch=%d, but no payload.\r\n", addrHi, addrLo, channel);
    return 1;
  }

  // The last byte in our buffer is the auto-appended RSSI:
  uint8_t rssi_raw = rxBuffer[payloadLen - 1];
  payloadLen--; // Everything before that is the actual payload text

  // Print debug
  printf("Received from 0x%02X%02X (ch=%d). Payload len=%d, RSSI(raw)=0x%02X\r\n",
         addrHi, addrLo, channel, payloadLen, rssi_raw);

  // Convert RSSI raw to dBm if needed (example formula, adjust for your module).
  int rssi_dBm = -120 + (rssi_raw * 0.5f);

  // Now let's make a null-terminated payload string to show
  // But watch for bounds:
  if (payloadLen > MAX_PAYLOAD_LEN - 1)
    payloadLen = MAX_PAYLOAD_LEN - 1;
  rxBuffer[payloadLen] = '\0'; // null-terminate

  // Example debug
  printf("Payload: %s\r\n", (char*)rxBuffer);
  printf("RSSI (dBm): %d\r\n", rssi_dBm);

  // Update OLED
  SH1106_Clear();
  SH1106_GotoXY(0, 0);
  SH1106_Puts("LoRa Receiver", &Font_7x10, 1);

  // Show the payload
  SH1106_GotoXY(0, 15);
  SH1106_Puts("Msg:", &Font_7x10, 1);
  SH1106_GotoXY(35, 15);
  SH1106_Puts((char*)rxBuffer, &Font_7x10, 1);

  // Show the RSSI
  char rssiString[20];
  sprintf(rssiString, "RSSI: %d dBm", rssi_dBm);
  SH1106_GotoXY(0, 30);
  SH1106_Puts(rssiString, &Font_7x10, 1);

  SH1106_UpdateScreen();

  // Blink LED to indicate a packet was received
  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

  return 1;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  // Init OLED
  SH1106_Init();
  SH1106_Clear();
  SH1106_GotoXY(10, 10);
  SH1106_Puts("LoRa RX Start", &Font_7x10, 1);
  SH1106_UpdateScreen();

  printf("LoRa Receiver started. Waiting for packets...\r\n");

  // Turn on a LED to show we are in RX mode
  HAL_GPIO_WritePin(GPIOA, Blue_LED_Pin, GPIO_PIN_SET);

  while (1)
  {
    // Try to receive a packet
    ReceiveLoRaPacket();
    // Then loop (you might also have timeouts or other logic)
  }
}

/* Peripheral/Clock/Pin init code below -- adapt as needed */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct  = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState           = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue= RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState       = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource      = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL         = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV         = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection= RCC_PERIPHCLK_USART1
                                     |RCC_PERIPHCLK_USART2
                                     |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection= RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection= RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection  = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Example for I2C2 used by SH1106 */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance              = I2C2;
  hi2c2.Init.Timing           = 0x0010020A;
  hi2c2.Init.OwnAddress1      = 0;
  hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2      = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Example: UART1 used for LoRa data */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance                    = USART1;
  huart1.Init.BaudRate              = 9600;
  huart1.Init.WordLength            = UART_WORDLENGTH_8B;
  huart1.Init.StopBits              = UART_STOPBITS_1;
  huart1.Init.Parity                = UART_PARITY_NONE;
  huart1.Init.Mode                  = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl             = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling          = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit= UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Example: UART2 for debug (printf) */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate              = 115200;
  huart2.Init.WordLength            = UART_WORDLENGTH_8B;
  huart2.Init.StopBits              = UART_STOPBITS_1;
  huart2.Init.Parity                = UART_PARITY_NONE;
  huart2.Init.Mode                  = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl             = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling          = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit= UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Clock enable for whichever ports needed. */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Example: LED pins */
  HAL_GPIO_WritePin(GPIOA, Blue_LED_Pin|LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = Blue_LED_Pin|LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void Error_Handler(void)
{
  __disable_irq();
  while(1)
  {
  }
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
