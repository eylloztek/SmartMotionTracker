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
#include "i2c.h"
#include "math.h"
#include "stdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdlib.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADXL345_ADDRESS     (0x53 << 1)  //8-bit address for STM32 HAL (shifted)
#define ADXL345_DEVID       0x00
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0      0x32

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t accX, accY, accZ;;
float gX, gY, gZ;
float roll, pitch;
uint8_t rx_data;
uint8_t rx_buffer[10];
uint8_t rx_index = 0;
float servo_angle =0;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ADXL345_Write(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDRESS, reg, 1, &value, 1, 100);
}

uint8_t ADXL345_Read(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDRESS, reg, 1, &data, 1, 100);
    return data;
}

void ADXL345_Init(void)
{
    uint8_t devid = ADXL345_Read(ADXL345_DEVID);
    if (devid != 0xE5)
    {
        printf("ADXL345 not found! ID: 0x%02X\r\n", devid);
        return;
    }
    // ±2g ve full resolution mod
    ADXL345_Write(ADXL345_DATA_FORMAT, 0x08);
    // Measurement mode
    ADXL345_Write(ADXL345_POWER_CTL, 0x08);
    printf("ADXL345 initialized successfully.\r\n");
}

void ADXL345_ReadXYZ(void)
{
    uint8_t data[6];
    HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDRESS, ADXL345_DATAX0, 1, data, 6, 100);

    accX = (int16_t)((data[1] << 8) | data[0]);
    accY = (int16_t)((data[3] << 8) | data[2]);
    accZ = (int16_t)((data[5] << 8) | data[4]);

    gX = accX * 0.0039;
    gY = accY * 0.0039;
    gZ = accZ * 0.0039;
}

void ADXL345_ComputeAngles(void)
{
    roll  = atan2(gY, gZ) * 180 / M_PI;
    pitch = atan2(-gX, sqrt(gY * gY + gZ * gZ)) * 180 / M_PI;
}

void Servo_Set_From_Angle(float roll_angle)
{
    // roll: -45° → 500 PWM, 0° → 1500 PWM, +45° → 2500 PWM
	uint16_t pwm_value = (uint16_t)((roll_angle + 90.0f) * (2000.0f / 180.0f) + 500.0f);

    if (pwm_value < 500) pwm_value = 500;
    if (pwm_value > 2500) pwm_value = 2500;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
    servo_angle = roll_angle;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_data == '\n')
        {
            rx_buffer[rx_index] = '\0';
            float angle = atof((char*)rx_buffer);

            if (angle >= 0 && angle <= 180)
            {
                servo_angle = angle;
                Servo_Set_From_Angle(servo_angle);
                printf("Servo angle set to %.2f\r\n", servo_angle);
            }
            else
            {
                printf("Invalid angle: %.2f\r\n", angle);
            }
            rx_index = 0;
        }
        else
        {
            rx_buffer[rx_index++] = rx_data;
            if (rx_index >= sizeof(rx_buffer))
                rx_index = 0;
        }
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  ADXL345_Init();
  ssd1306_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  printf("Bluetooth Servo Control Ready.\r\n");

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      ADXL345_ReadXYZ();
      ADXL345_ComputeAngles();

      Servo_Set_From_Angle(roll);
      HAL_Delay(1500);

      ssd1306_Fill(Black);

      char buf[32];
      ssd1306_SetCursor(0, 0);
      snprintf(buf, sizeof(buf), "Roll : %.2f", roll);
      ssd1306_WriteString(buf, Font_7x10, 1);

      ssd1306_SetCursor(0, 16);
      snprintf(buf, sizeof(buf), "Pitch: %.2f", pitch);
      ssd1306_WriteString(buf, Font_7x10, 1);

      ssd1306_SetCursor(0, 32);
      if (fabs(roll) > 15 || fabs(pitch) > 15)
          ssd1306_WriteString("Status: TILTED", Font_7x10, 1);
      else
          ssd1306_WriteString("Status: STABLE", Font_7x10, 1);

      ssd1306_SetCursor(0, 48);
      snprintf(buf, sizeof(buf), "Servo: %.2f", servo_angle);
      ssd1306_WriteString(buf, Font_7x10, 1);

      ssd1306_UpdateScreen();

      /*
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);   // 0 derece
      HAL_Delay(1000);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);  // 90 derece
      HAL_Delay(1000);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2500);  // 180 derece
      HAL_Delay(1000);
      */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
