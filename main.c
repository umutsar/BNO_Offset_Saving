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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "bno055.h"
#include "debug_monitor.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int16_t ACC_OFF_X, ACC_OFF_Y, ACC_OFF_Z;
volatile int16_t MAG_OFF_X, MAG_OFF_Y, MAG_OFF_Z;
volatile int16_t GYR_OFF_X, GYR_OFF_Y, GYR_OFF_Z;
volatile int16_t ACC_RADIUS, MAG_RADIUS;
volatile uint8_t BNO_OFFSETS_VALID = 0; // 1 olduğunda değerler hazır


// Debug'ta izlemen için tek değişken:
volatile uint8_t BNO_CAL_DONE = 0;     // 1 => kalibrasyon tamam
BNO055_CalibProfile_t BNO_CAL_PROFILE; // offset paketi burada

// KULLANIM:
// 1) BNO055_Init(&hi2c1) çağrıldıktan sonra şunu çağır:
//    BNO_CalibrateQuick(60000); // 60 sn bekle (gerekirse artır)
// 2) Debug'tan BNO_CAL_DONE==1 olunca BNO_CAL_PROFILE içindeki tüm offsetleri kopyala.
uint8_t BNO_CalibrateQuick(uint32_t timeout_ms)
{
  // Ölçüm moduna geç (zaten NDOF'taysan sorun değil)
  BNO055_SetOpMode(&hi2c1, BNO055_OPMODE_NDOF);

  // Tam kalibrasyon için bekle (3-3-3-3). Sensörü her eksende yavaşça gezdir.
  if (!BNO055_WaitFullyCalibrated(&hi2c1, timeout_ms))
  {
    BNO_CAL_DONE = 0;
    return 0; // süre doldu
  }

  // Offset paketini al ve değişkene koy
  if (!BNO055_ReadCalibProfile(&hi2c1, &BNO_CAL_PROFILE))
  {
    BNO_CAL_DONE = 0;
    return 0;
  }

  // İstersen tekrar NDOF’a dokun (gerekmez ama zararı yok)
  BNO055_SetOpMode(&hi2c1, BNO055_OPMODE_NDOF);

  BNO_CAL_DONE = 1;
  return 1;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  float yaw = 0.0;
  float roll = 0.0;
  float pitch = 0.0;
  uint32_t sayac = 0;

  Servo_Init(MOTOR_1);
  Servo_Init(MOTOR_2);
  Servo_Init(MOTOR_3);
  Servo_Init(MOTOR_4);
  Servo_Init(MOTOR_5);
  Servo_Init(MOTOR_6);
  Servo_Init(MOTOR_7);
  Servo_Init(MOTOR_8);
  HAL_Delay(2);

  Servo_Calibrate(MOTOR_1);
  Servo_Calibrate(MOTOR_2);
  Servo_Calibrate(MOTOR_3);
  Servo_Calibrate(MOTOR_4);
  Servo_Calibrate(MOTOR_5);
  Servo_Calibrate(MOTOR_6);
  Servo_Calibrate(MOTOR_7);
  Servo_Calibrate(MOTOR_8);
  HAL_Delay(1000);

  if (!BNO055_Init(&hi2c1))
  {
  }

  BNO_CalibrateQuick(60000); // 60 sn
  // if (!BNO055_WaitFullyCalibrated(&hi2c1, 30000))
  // {
  //   HAL_GPIO_WritePin(GPIOC, RGB_B_Pin, SET);
  // }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // for (uint16_t i = 1000; i <= 2000; i++)
    // {
    //   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
    //   HAL_Delay(20);
    // }
    HAL_Delay(50);
    sayac += 1;
    Servo_Write(MOTOR_1, 1500);

    if (!BNO055_ReadEuler(&hi2c1, &yaw, &roll, &pitch))
    {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
