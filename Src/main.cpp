/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmath>
#include <cstring>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "HAL_Serial_Print.h"
#include "astronomy.h"
#include "tlc592x.h"
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

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TLC592x htlc592x;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Configure_TLC592x()
{
  htlc592x.switchToSpecialMode();
  uint8_t config[28] = {
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,

    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
    0x00,0x40,
  };
  while (htlc592x.state == TLC592x_STATE_BUSY) ;
  htlc592x.shiftOut(config, 224);
  while (htlc592x.state == TLC592x_STATE_BUSY) ;
  htlc592x.switchToNormalMode();
  while (htlc592x.state == TLC592x_STATE_BUSY) ;
}
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  SFE_UBLOX_GPS myGPS;
  myGPS.begin(hi2c1);
  // myGPS.enableDebugging(huart2);
  // myGPS.getProtocolVersion();
  myGPS.setI2COutput(COM_TYPE_UBX);
  myGPS.saveConfiguration();
  HAL_Serial_Print p(huart2);
  Configure_TLC592x();
  htlc592x.disablePWM();
  //htlc592x.enablePWM(500);
  HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_RESET);
  p.println("\r\nReset");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    HAL_Serial_Print p(huart2);
    long latitude = myGPS.getLatitude();
    p.print("Lat: ");
    p.print(latitude);
    p.println(" (degrees * 10^-7)");

    long longitude = myGPS.getLongitude();
    p.print("Long: ");
    p.print(longitude);
    p.println(" (degrees * 10^-7)");

    long altitude = myGPS.getAltitudeMSL();
    p.print("Alt: ");
    p.print(altitude);
    p.println(" (mm)");

    uint8_t SIV = myGPS.getSIV();
    p.print("SIV: ");
    p.println(SIV);

    auto year = myGPS.getYear();
    auto month = myGPS.getMonth();
    auto day = myGPS.getDay();
    auto hour = myGPS.getHour();
    auto minute = myGPS.getMinute();
    auto second = myGPS.getSecond();

    p.print("Time: ");
    p.print(year); p.print("-"); p.print(month); p.print("-"); p.print(day);
    p.print(" ");
    p.print(hour); p.print(":"); p.print(minute); p.print(":"); p.println(second);

    auto jd = julian_date(year, month, day, hour, minute, second);
    auto day_fraction_s = compute_day_fraction(latitude * 1e-7f, longitude * 1e-7f, jd);

    int32_t day_fraction = roundf(112*day_fraction_s.day_fraction);
    int32_t sunset_fraction = roundf(112*day_fraction_s.daylength_fraction);

    uint8_t serial_buf[28];
    // Set bit sunset_fraction in the stream of bits
    for (uint8_t b = 0; b < 14; b++) {
      if (112 - 8*(b + 1) <= sunset_fraction && sunset_fraction < 112 - 8*b) {
        serial_buf[b] = (uint8_t)(1 << (112 - sunset_fraction - 8*b));
      } else {
        serial_buf[b] = 0;
      }
    }

    // Set day_fraction bits high starting from the left
    for (uint8_t b = 0; b < 14; b++) {
      if (day_fraction >= 112 - 8*b) {
        serial_buf[14 + b] = 0xff;
      } else if (day_fraction < 112 - 8*(b + 1)) {
        serial_buf[14 + b] = 0x00;
      } else {
        serial_buf[14 + b] = (uint8_t)(0xff << (112 - day_fraction - 8*b));
      }
    }
    htlc592x.shiftOut(serial_buf, 224);

    p.print("Day fraction: ");
    p.println(day_fraction_s.day_fraction);

    p.print("Sunset at day fraction: ");
    p.println(day_fraction_s.daylength_fraction);
    p.println();

    HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 15;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 63;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 32;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 3;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 15;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TLC592x_LE_Pin|TLC592x_SDO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TLC592x_LE_Pin TLC592x_SDO_Pin */
  GPIO_InitStruct.Pin = TLC592x_LE_Pin|TLC592x_SDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NUCLEO_LED_Pin */
  GPIO_InitStruct.Pin = NUCLEO_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NUCLEO_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14) {    // TLC592x CLK
    if (htlc592x.txCount == htlc592x.txSize) {  // Sequence finished
      HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
      HAL_TIM_Base_Stop_IT(&htim14);
      if (htlc592x.op == TLC592x_OP_SHIFT_OUT) {  // Pulse LE
        HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim17);
      }
      htlc592x.state = TLC592x_STATE_READY;
    } else {
      if (htlc592x.op == TLC592x_OP_SHIFT_OUT) {
          uint16_t pos = htlc592x.txCount / 8;
          uint8_t bit = htlc592x.txCount % 8;
          if (htlc592x.txBuffer[pos] & (1 << bit)) {
            HAL_GPIO_WritePin(TLC592x_SDO_GPIO_Port, TLC592x_SDO_Pin, GPIO_PIN_SET);
          } else {
            HAL_GPIO_WritePin(TLC592x_SDO_GPIO_Port, TLC592x_SDO_Pin, GPIO_PIN_RESET);
          }
      } else if (htlc592x.op == TLC592x_OP_SWITCH_TO_SPECIAL) {
        switch (htlc592x.txCount) {
          case 0:
            HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
            break;
          case 1:
            HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_RESET);
            break;
          case 2:
            HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_SET);
            break;
          case 3:
            HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_SET);
            break;
          case 4:
            HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
            break;
        }
      } else {
        switch (htlc592x.txCount) {
          case 0:
            HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
            break;
          case 1:
            HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_RESET);
            break;
          case 2:
            HAL_GPIO_WritePin(TLC592x_OE_GPIO_Port, TLC592x_OE_Pin, GPIO_PIN_SET);
            break;
          case 3:
            HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
            break;
          case 4:
            HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
            break;
        }
      }
      ++htlc592x.txCount;
    }
  } else if (htim->Instance == TIM17) {
    HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
    // One pulse mode will disable counter
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
