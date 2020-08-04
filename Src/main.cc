/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cc
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright 2020 James Wilson <jmw@jmw.name>. All rights reserved.
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     https://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
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
#include "Print.h"
#include "astronomy.h"
#include "tlc5926.h"
#include "ublox.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
constexpr int PWM_SCALE = 1000;
constexpr float MIN_PWM_BRIGHTNESS = 0.1f;
constexpr float CIVIL_TWILIGHT_EL = 6;

constexpr int NUM_TLC592x = 14;
constexpr size_t TLC592x_BUF_SIZE = 2 * NUM_TLC592x;
constexpr int NUM_LED = NUM_TLC592x/2 * 16; // LEDs in each row

constexpr int MAX_MISSED_FIX_COUNT = 30; // * 10 s = 5 minutes w/o GPS fix

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t tlc592xBuf[TLC592x_BUF_SIZE];
static volatile bool ubloxDataAvailable = false;

static int missedFixCount = 0;

static bool initialFix = false;
static int32_t lastLon;
static int32_t lastLat;
static int32_t lastAlt;

const Print p(huart2);

extern const char *BUILD_STRING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void ConfigureTLC5926()
{
  TLC5926_SwitchToSpecialMode();

  for (auto i = 0; i < NUM_TLC592x; i++) {
    // {CM,HC,[CC0:CC5]} = {0,1,000000}
    tlc592xBuf[2*i] = 0x00;
    tlc592xBuf[2*i + 1] = 0x40;
  }
  if (HAL_SPI_Transmit_DMA(&hspi1, tlc592xBuf, sizeof(tlc592xBuf))
      != HAL_OK) {
    Error_Handler();
  }
  while (hspi1.State != HAL_SPI_STATE_READY) { }

  TLC5926_SwitchToNormalMode();

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

static void RequireUbloxReady(uint16_t timeout)
{
  const uint32_t tickstart = HAL_GetTick();

  while (HAL_GetTick() - tickstart < timeout) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, UBLOX_I2C_ADDRESS << 1, 1, 10)
        == HAL_OK) {
      return;
    }
  }
  Error_Handler();
}

static bool ExpectACKCallback(const uint8_t cls, const uint8_t id,
                              const uint8_t *payload, const uint16_t length)
{
  return cls == UBX_CLS::UBX_ACK && id == UBX_MSG::UBX_ACK_ACK;
}

static void RequireUbloxACK()
{
  const uint32_t tickstart = HAL_GetTick();
  const uint32_t timeout = 1000;  // ACK must be sent within 1 s

  while (HAL_GetTick() - tickstart < timeout) {
    if (UBX_Receive(&hi2c1, UBLOX_I2C_ADDRESS, ExpectACKCallback, 100)) {
      return;
    }
    HAL_Delay(10);  // Don't hammer the bus
  }
  Error_Handler();
}

static void UBX_DisableUART(void)
{
  uint8_t packet[] {
    UBLOX_SYNC_1,
    UBLOX_SYNC_2,
    UBX_CLS::UBX_CFG,
    UBX_MSG::UBX_CFG_PRT,
    0, 0, // space for length

    UBX_PORT_UART, // Port ID
    0, // reserved
    UBX_INT16(0), // txReady
    UBX_INT32(0x80c0),  // mode: 8N1
    UBX_INT32(115200),  // baud rate: 115200
    UBX_INT16(0), // inProtoMask: disable all
    UBX_INT16(0), // outProtoMask: disable all
    UBX_INT16(0), // flags
    UBX_INT16(0), // reserved

    0, 0, // space for checksum
  };
  UBX_SetSizeAndChecksum(packet, sizeof(packet));
  if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDRESS << 1, packet,
                              sizeof(packet), 100) != HAL_OK) {
    Error_Handler();
  }
  RequireUbloxACK();
}

static void UBX_ConfigureNavigation(void)
{
  // Set update period to 10 s
  uint8_t packet_rate[] {
    UBLOX_SYNC_1,
    UBLOX_SYNC_2,
    UBX_CLS::UBX_CFG,
    UBX_MSG::UBX_CFG_RATE,
    0, 0, // space for length

    UBX_INT16(1000), // measRate: 1000 ms
    UBX_INT16(10), // navRate: 10
    UBX_INT16(0), // timeRef: aligned to UTC

    0, 0, // space for checksum
  };
  UBX_SetSizeAndChecksum(packet_rate, sizeof(packet_rate));
  if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDRESS << 1,
                              packet_rate, sizeof(packet_rate), 100)
      != HAL_OK) {
    Error_Handler();
  }
  RequireUbloxACK();

  // Enable periodic updates
  uint8_t packet_msg[] {
    UBLOX_SYNC_1,
    UBLOX_SYNC_2,
    UBX_CLS::UBX_CFG,
    UBX_MSG::UBX_CFG_MSG,
    0, 0, // space for length

    UBX_CLS::UBX_NAV, // msgClass
    UBX_MSG::UBX_NAV_PVT, // msgID
    1, // rate

    0, 0, // space for checksum
  };
  UBX_SetSizeAndChecksum(packet_msg, sizeof(packet_msg));
  if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDRESS << 1, packet_msg,
                              sizeof(packet_msg), 100) != HAL_OK) {
    Error_Handler();
  }
  RequireUbloxACK();
}

static void UBX_ConfigureI2C(void)
{
  uint8_t packet[] {
    UBLOX_SYNC_1,
    UBLOX_SYNC_2,
    UBX_CLS::UBX_CFG,
    UBX_MSG::UBX_CFG_PRT,
    0, 0, // space for length

    UBX_PORT_I2C, // Port ID
    0, // reserved
    UBX_INT16((6 << 2) | 1), // txReady: enabled on PIO 6 (TX)
    UBX_INT32(UBLOX_I2C_ADDRESS << 1), // mode: 0x42 slave address
    UBX_INT32(0), // reserved
    UBX_INT16(1), // inProtoMask: UBX only
    UBX_INT16(1), // outProtoMask: UBX only
    UBX_INT16(0), // flags
    UBX_INT16(0), // reserved

    0, 0, // space for checksum
  };
  UBX_SetSizeAndChecksum(packet, sizeof(packet));
  if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDRESS << 1, packet,
                              sizeof(packet), 100) != HAL_OK) {
    Error_Handler();
  }
  RequireUbloxACK();
}

static void UBX_SaveConfiguration(void)
{
  uint8_t packet[] {
    UBLOX_SYNC_1,
    UBLOX_SYNC_2,
    UBX_CLS::UBX_CFG,
    UBX_MSG::UBX_CFG_CFG,
    0, 0, // space for length

    UBX_INT32(0),  // clearMask
    UBX_INT32(0x1f1f),  // saveMask: all
    UBX_INT32(0), // loadMask

    0, 0, // space for checksum
  };
  UBX_SetSizeAndChecksum(packet, sizeof(packet));
  if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDRESS << 1, packet,
                              sizeof(packet), 100) != HAL_OK) {
    Error_Handler();
  }
  RequireUbloxACK();
}

static bool PrintVersionCallback(const uint8_t cls, const uint8_t id,
                                       const uint8_t *payload, const uint16_t length)
{
  if (cls != UBX_CLS::UBX_MON || id != UBX_MSG::UBX_MON_VER) {
    return false;
  }

  p.print("u-blox HW version ");
  p.println(reinterpret_cast<const char *>(payload + 30));
  p.print("u-blox SW version ");
  p.println(reinterpret_cast<const char *>(payload));
  p.println("u-blox extension strings:");
  for (auto offset = 40u; offset < length; offset += 30) {
    p.print("  ");
    p.println(reinterpret_cast<const char *>(payload + offset));
  }
  p.println();

  return true;
}

static void UBX_PrintVersion(void)
{
  uint8_t packet[] {
    UBLOX_SYNC_1,
    UBLOX_SYNC_2,
    UBX_CLS::UBX_MON,
    UBX_MSG::UBX_MON_VER,
    0, 0, // space for length
    0, 0, // space for checksum
  };
  UBX_SetSizeAndChecksum(packet, sizeof(packet));
  if (HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDRESS << 1, packet,
                              sizeof(packet), 100) != HAL_OK) {
    Error_Handler();
  }

  const uint32_t tickstart = HAL_GetTick();
  while (HAL_GetTick() - tickstart < 1000) {
    if (UBX_Receive(&hi2c1, UBLOX_I2C_ADDRESS, PrintVersionCallback, 100)) {
      return;
    }
    HAL_Delay(10);  // Don't hammer the bus
  }
}

static void DisplayNoFixPattern()
{
  memset(tlc592xBuf, 0, sizeof(tlc592xBuf));
  tlc592xBuf[7] = 1;
  tlc592xBuf[21] = 1;
  HAL_SPI_Transmit_DMA(&hspi1, tlc592xBuf, sizeof(tlc592xBuf));
  htim16.Instance->CCR1 = static_cast<uint32_t>(
    MIN_PWM_BRIGHTNESS*PWM_SCALE
  );
}

static bool NavigationCallback(const uint8_t cls, const uint8_t id,
                               const uint8_t *payload, const uint16_t length)
{
  if (cls != UBX_CLS::UBX_NAV || id != UBX_MSG::UBX_NAV_PVT) {
    return false;
  }

  const uint16_t year = readU2(payload, 4);
  const uint8_t month = payload[6];
  const uint8_t day = payload[7];
  const uint8_t hour = payload[8];
  const uint8_t min = payload[9];
  const uint8_t sec = payload[10];
  const uint8_t valid = payload[11];
  const uint8_t flags = payload[21];
  const uint8_t numSV = payload[23];
  int32_t lon = readU4(payload, 24);
  int32_t lat = readU4(payload, 28);
  int32_t alt_msl = readU4(payload, 36);

  const bool gnssFixOK = flags & 1;
  if (gnssFixOK) {
    initialFix = true;
    missedFixCount = 0;
    lastLon = lon;
    lastLat = lat;
    lastAlt = alt_msl;
  } else {
    const bool validDateAndTime = (valid & 3) == 3;
    if (initialFix && validDateAndTime) {
      p.println("Time fix only, reusing last position fix");
      lon = lastLon;
      lat = lastLat;
      alt_msl = lastAlt;
    } else {
      p.println("Waiting for GNSS fix...");
      if (++missedFixCount == MAX_MISSED_FIX_COUNT) {
        DisplayNoFixPattern();
      }
      return true;
    }
  }

  p.print("Position: ");
  p.printDecimal(lat, 7);
  p.print("°, ");
  p.printDecimal(lon, 7);
  p.println("°");

  p.print("Altitude (MSL): ");
  p.printDecimal(alt_msl, 3);
  p.println(" m");

  p.print("SIV: ");
  p.println(numSV);

  p.print("Time (UTC): ");
  p.print(year);
  p.print("-");
  p.print(month, Print::DEC, 2);
  p.print("-");
  p.print(day, Print::DEC, 2);
  p.print(" ");
  p.print(hour, Print::DEC, 2);
  p.print(":");
  p.print(min, Print::DEC, 2);
  p.print(":");
  p.println(sec, Print::DEC, 2);

  const auto jd = JulianDay(year, month, day, hour, min, sec);
  const auto df = DayFraction(lat * 1e-7f, lon * 1e-7f, jd);

  p.print("Solar azimuth: ");
  p.print(df.solar_azimuth);
  p.println("°");

  p.print("Solar elevation: ");
  p.print(df.solar_elevation);
  p.println("°");

  p.print("Day fraction: ");
  p.println(df.day_fraction, 3);

  p.print("Sunset at day fraction: ");
  p.println(df.daylength_fraction, 3);
  p.println();

  // Set bit sunset_fraction in the stream of bits
  if (isnanf(df.daylength_fraction)) {
    // No sunrise or sunset
    memset(tlc592xBuf, 0, TLC592x_BUF_SIZE / 2);
  } else {
    const auto sunset_fraction = static_cast<int>(
      roundf(NUM_LED*df.daylength_fraction));
    for (auto b = 0; b < TLC592x_BUF_SIZE/2; b++) {
      if (NUM_LED - 8*(b + 1) <= sunset_fraction
          && sunset_fraction < NUM_LED - 8*b) {
        tlc592xBuf[b] = 1 << (NUM_LED - sunset_fraction - 8*b);
      } else {
        tlc592xBuf[b] = 0;
      }
    }
  }

  // Set day_fraction bits high starting from the left
  const auto day_fraction = static_cast<int>(
    roundf(NUM_LED*df.day_fraction));
  for (auto b = 0; b < TLC592x_BUF_SIZE/2; b++) {
    if (day_fraction >= NUM_LED - 8*b) {
      tlc592xBuf[TLC592x_BUF_SIZE/2 + b] = 0xff;
    } else if (day_fraction < NUM_LED - 8*(b + 1)) {
      tlc592xBuf[TLC592x_BUF_SIZE/2 + b] = 0x00;
    } else {
      tlc592xBuf[TLC592x_BUF_SIZE/2 + b] =
        0xff << (NUM_LED - day_fraction - 8*b);
    }
  }
  HAL_SPI_Transmit_DMA(&hspi1, tlc592xBuf, sizeof(tlc592xBuf));

  if (df.solar_elevation > CIVIL_TWILIGHT_EL) {
    htim16.Instance->CCR1 = PWM_SCALE;
  } else if (df.solar_elevation < -CIVIL_TWILIGHT_EL) {
    htim16.Instance->CCR1 = static_cast<uint32_t>(
      MIN_PWM_BRIGHTNESS*PWM_SCALE
    );
  } else {
    // Smoothly dim the LEDs during the transition between night and day
    const float brightness = MIN_PWM_BRIGHTNESS + (1 - MIN_PWM_BRIGHTNESS)/2*(
      1 + sinf(static_cast<float>(M_PI_2) / CIVIL_TWILIGHT_EL
               * df.solar_elevation)
    );
    htim16.Instance->CCR1 = static_cast<uint32_t>(
      roundf(brightness*PWM_SCALE)
    );
  }

  return true;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ConfigureTLC5926();
  p.print("\r\nDevice Reset (build ");
  p.print(BUILD_STRING);
  p.println(")");
  RequireUbloxReady(1000);
  UBX_DisableUART();
  UBX_ConfigureI2C();
  UBX_ConfigureNavigation();
  UBX_SaveConfiguration();
  UBX_PrintVersion();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DisplayNoFixPattern();
  while (true) {
    if (ubloxDataAvailable) {
      UBX_Receive(&hi2c1, UBLOX_I2C_ADDRESS, NavigationCallback, 100);
    }
    HAL_SuspendTick();
    __WFE();
    HAL_ResumeTick();
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SAFEBOOT_N_GPIO_Port, SAFEBOOT_N_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TLC592x_LE_Pin */
  GPIO_InitStruct.Pin = TLC592x_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TLC592x_LE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAFEBOOT_N_Pin */
  GPIO_InitStruct.Pin = SAFEBOOT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SAFEBOOT_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ERROR_LED_Pin */
  GPIO_InitStruct.Pin = ERROR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ERROR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TX_READY_Pin */
  GPIO_InitStruct.Pin = TX_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TX_READY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TX_READY_Pin) {
    ubloxDataAvailable = true;
    __SEV();
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TX_READY_Pin) {
    ubloxDataAvailable = false;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    // Pulse LE
    HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC592x_LE_GPIO_Port, TLC592x_LE_Pin, GPIO_PIN_RESET);
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
  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
  while(1) { }
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
  p.print("Assertion failed at ");
  p.print(reinterpret_cast<char *>(file));
  p.print(", line ");
  p.println(line);
  HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
  while(1) { }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
