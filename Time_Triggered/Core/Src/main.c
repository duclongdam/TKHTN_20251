/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h> // Thu vien cho va_list
#include "sht31.h"
#include "soil_moisture.h"
#include "simple_scheduler.h"
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    DISP_TEMP,
    DISP_HUMI,
    DISP_SOIL,
    DISP_ALL
} DisplayMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sht31_t sensor_sht;
SoilSensor sensor_soil;
uint8_t ID_SHT = 0;
uint8_t ID_SOIL = 0;
uint8_t ID_LCD = 0;
uint8_t ID_UART = 0;
float val_temp = 0.0f;
float val_hum = 0.0f;

uint8_t rx_data[RX_BUF_SIZE];
char cmd_buffer[RX_BUF_SIZE];
uint8_t rx_indx = 0;
volatile uint8_t flag_cmd = 0;
DisplayMode_t disp_mode = DISP_ALL;
char log_buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Debug_Log(const char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Debug_Log(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t *)log_buffer, strlen(log_buffer), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t received_char = rx_data[rx_indx];
        if (received_char == '\n' || received_char == '\r') {
            rx_data[rx_indx] = '\0';
            if (rx_indx > 0) {
                strcpy(cmd_buffer, (char*)rx_data);
                flag_cmd = 1;
            }
            rx_indx = 0;
        } else {
            rx_indx++;
            if (rx_indx >= RX_BUF_SIZE) rx_indx = 0;
        }
        HAL_UART_Receive_IT(&huart1, &rx_data[rx_indx], 1);
    }
}

void Task_MeasureSHT(void) {
    sht31_read(&sensor_sht, &val_temp, &val_hum);
}

void Task_MeasureSoil(void) {
    Soil_Read(&sensor_soil);
}

void Task_UpdateLCD(void) {
    char line1[17];
    char line2[17];
    memset(line1, ' ', 16); line1[16] = '\0';
    memset(line2, ' ', 16); line2[16] = '\0';
    switch (disp_mode) {
        case DISP_TEMP:
            snprintf(line1, 17, "T: %.1f C        ", val_temp);
            break;
        case DISP_HUMI:
            snprintf(line1, 17, "H: %.1f %%        ", val_hum);
            break;
        case DISP_SOIL:
            snprintf(line1, 17, "Soil: %.1f %%    ", sensor_soil.percent);
            break;
        case DISP_ALL:
            snprintf(line1, 17, "T:%.0fC H:%.0f%%     ", val_temp, val_hum);
            snprintf(line2, 17, "Soil: %.0f%%       ", sensor_soil.percent);
            break;
    }
    LCD_Put_Cur(0, 0); LCD_Send_String(line1);
    LCD_Put_Cur(1, 0); LCD_Send_String(line2);
}
void Task_Send_UART(void){
    Debug_Log("DATA: Temp=%.2f, Hum=%.2f, Soil=%.2f\r\n", val_temp, val_hum, sensor_soil.percent);
}
void Task_ProcessUART(void) {
    if (!flag_cmd) return; // Nếu không có lệnh mới thì thoát

    char cmd_type;
    int value;
    if (sscanf(cmd_buffer, "%c:%d", &cmd_type, &value) == 2) {
        switch(cmd_type) {
            // --- LỆNH THAY ĐỔI CHU KỲ (PERIOD) ---
            case 'P': 
            case 'p':
                // Giới hạn từ 100ms đến 30000ms (30 giây)
                if (value >= 100 && value <= 30000) {
                    
                    // CÔNG THỨC MỚI VỚI PSC = 35999 (0.5ms/tick)
                    // ARR = (Time_ms / 0.5) - 1  <=>  ARR = (Time_ms * 2) - 1
                    uint32_t new_arr = (value * 2) - 1;
                    
                    __HAL_TIM_SET_AUTORELOAD(&htim2, new_arr);
                    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset để áp dụng ngay
                    
                    Debug_Log("OK: Period set to %d ms (ARR=%d)\r\n", value, new_arr);
                } else {
                    Debug_Log("Err: Range 100 - 30000 ms\r\n");
                }
                break;

            // --- LỆNH THAY ĐỔI CHẾ ĐỘ HIỂN THỊ ---
            case 'D': 
            case 'd':
                if (value >= 0 && value <= 3) {
                    disp_mode = (DisplayMode_t)value;
                    Debug_Log("OK: Display Mode %d\r\n", value);
                } else {
                    Debug_Log("Err: Mode 0-3\r\n");
                }
                break;
                
            default:
                Debug_Log("Err: Cmd Error\r\n");
                break;
        }
    }
    flag_cmd = 0;
    memset(cmd_buffer, 0, RX_BUF_SIZE);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	sht31_init(&sensor_sht, &hi2c1, SHT31_ADDR_LO);
  Soil_Init(&sensor_soil, &hadc1, 0, 0);
	
	LCD_Init();
  LCD_Put_Cur(0, 0);
	LCD_Send_String("Mode: Simple");
  LCD_Put_Cur(1, 0);
	LCD_Send_String("Periodic Scheduler");
  HAL_Delay(1000);
  LCD_Clear();
	
	HAL_UART_Receive_IT(&huart1, &rx_data[rx_indx], 1);
  Debug_Log("System Ready. Waiting for Timer...\r\n");

  SCH_Init();
	SCH_Add_Task(Task_MeasureSHT); 
  SCH_Add_Task(Task_MeasureSoil);
  SCH_Add_Task(Task_UpdateLCD);
  SCH_Add_Task(Task_Send_UART);
  SCH_Set_Wakeup_Callback(Task_ProcessUART);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		SCH_Run_Cycle();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
