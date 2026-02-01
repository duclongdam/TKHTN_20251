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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdarg.h"
#include <string.h>
#include "queue_handle.h"
#include "lcd_i2c.h"
#include "sht31.h"
#include "soil_moisture.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	NO_TASK,						//Have no task
	TASK_SHT31,        	//Read SHT31
	TASK_SOIL,         	//Read soil moisture v2.0
	TASK_LCD,					 	//Write LCD
	TASK_PARSE_CMD,			//Check message from UART RX
} TaskCase_t;

typedef enum{
	DISPLAY_ALL,				//Display all
	DISPLAY_TEMP,				//Just temparature
	DISPLAY_HUMI,				//Just humidity
	DISPLAY_SOIL,				//Just soil moisture
} DisplayMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
TaskCase_t current_task = NO_TASK;
volatile DisplayMode_t current_display = DISPLAY_ALL;

sht31_t dev;
SoilSensor soilsensor;

float temp_sht31 = 0;
float humi_sht31 = 0;
float soil_moisture = 0;
char lcd_buffer[17];

volatile uint32_t period_sht31 = 500;			//ms
volatile uint32_t period_soil  = 1000;		//ms
volatile uint32_t period_lcd   = 2000;		//ms

uint8_t rx_buffer[RX_BUFFER_SIZE]; 
uint8_t rx_index = 0;             
uint8_t rx_temp_char;    
volatile uint8_t cmd_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char log_buffer[100];
//Export Debug log through UART
void Debug_Log(const char *format,...){
	va_list args ; 
	va_start(args,format);
	vsnprintf(log_buffer,sizeof(log_buffer),format,args);
	va_end(args);
	// UART
	HAL_UART_Transmit(&huart1,(uint8_t *)log_buffer,strlen(log_buffer),100);
}

void Read_SHT31(void){
	bool check = sht31_read(&dev, &temp_sht31, &humi_sht31);
	/*
		if(check){
			Debug_Log("Read SHT31 successfully: temp: %f, humi: %f",temp_sht31,humi_sht31);
		}else{
			Debug_Log("Fail to read SHT31");
		}
	*/
}

void Read_Soil(void){
		Soil_Read(&soilsensor);
		soil_moisture = soilsensor.percent;
		// Debug_Log("Read v2.0 successfully: soil: %f", soil_moisture);
}	

void Write_LCD(DisplayMode_t mode) {
	switch(mode) {
		case DISPLAY_ALL:
			sprintf(lcd_buffer, "T:%.1fC H:%.1f%% ", temp_sht31, humi_sht31);
			LCD_Put_Cur(0, 0);
			LCD_Send_String(lcd_buffer);
	
			sprintf(lcd_buffer, "Soil: %.1f %%    ", soil_moisture);
			LCD_Put_Cur(1, 0);
			LCD_Send_String(lcd_buffer);
			break;
		
		case DISPLAY_TEMP:
			LCD_Put_Cur(0, 0);
			LCD_Send_String("Temperature:    ");
			
			sprintf(lcd_buffer, "%.1f C          ", temp_sht31);
			LCD_Put_Cur(1, 0);
			LCD_Send_String(lcd_buffer);
			break;
		
		case DISPLAY_HUMI:
			LCD_Put_Cur(0, 0);
			LCD_Send_String("Humidity:       ");
			
			sprintf(lcd_buffer, "%.1f %%         ", humi_sht31);
			LCD_Put_Cur(1, 0);
			LCD_Send_String(lcd_buffer);
			break;

		case DISPLAY_SOIL:
			LCD_Put_Cur(0, 0);
			LCD_Send_String("Soil Moisture:  ");
			
			sprintf(lcd_buffer, "%.1f %%         ", soil_moisture);
			LCD_Put_Cur(1, 0);
			LCD_Send_String(lcd_buffer);
			break;
    }
}	
//UART Callback function 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_temp_char == '\n' || rx_temp_char == '\r') { //Done UART if last character is "\n" or "\r"
            if (rx_index > 0) { 
                rx_buffer[rx_index] = '\0'; 
                cmd_flag = 1;               
                Push_Queue(TASK_PARSE_CMD);
                rx_index = 0;          
            }
        } 
        else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = rx_temp_char;
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_temp_char, 1);
    }
}
//Handle received strings 
void Process_Command(void) {
    char cmd_type;
    int value;
    if (sscanf((char*)rx_buffer, "%c:%d", &cmd_type, &value) == 2) {
        
        switch(cmd_type) {
            case 'T': // SHT31 Period
            case 't':
                period_sht31 = (uint32_t)value;
                Debug_Log("OK: SHT31 Period set to %dms\r\n", period_sht31);
                break;

            case 'S': // Soil Period
            case 's':
                period_soil = (uint32_t)value;
                Debug_Log("OK: Soil Period set to %dms\r\n", period_soil);
                break;

            case 'L': // LCD Period
            case 'l':
                period_lcd = (uint32_t)value;
                Debug_Log("OK: LCD Period set to %dms\r\n", period_lcd);
                break;

						case 'D': // Display change
						case 'd':
								if (value >= 0 && value <= 3) {
										current_display = (DisplayMode_t)value;
								
										char* modes[] = {"ALL", "TEMP", "HUMI", "SOIL"};
										Debug_Log("OK: Display switched to %s\r\n", modes[value]);
										
										Push_Queue(TASK_LCD);
								} else {
										Debug_Log("Error: Invalid Display Mode (%d). Use 0-3.\r\n", value);
								}
								break;
								
            default:
                Debug_Log("Error: Unknown Command\r\n");
                break;
        }
    }
    
    cmd_flag = 0; 
}
//Count period, TIM2 interrupt each 1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        static uint32_t count_sht31 = 0;
        static uint32_t count_soil = 0;
        static uint32_t count_lcd = 0;

        count_sht31++;
        count_soil++;
        count_lcd++;

        if (count_sht31 >= period_sht31) {
            Push_Queue(TASK_SHT31);
            count_sht31 = 0;
					
        }

        if (count_soil >= period_soil) {
            Push_Queue(TASK_SOIL);
            count_soil = 0;
        }

        if (count_lcd >= period_lcd) {
            Push_Queue(TASK_LCD);
            count_lcd = 0;
        }
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	sht31_init(&dev, &hi2c1, SHT31_ADDR_LO);		//Init SHT31
	Soil_Init(&soilsensor, &hadc1, 0, 0);				//Init v2.0		
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_temp_char, 1);		//UART RX
	HAL_TIM_Base_Start_IT(&htim2);			//Start TIM2				
	
	//Init LCD
	LCD_Init();
	LCD_Put_Cur(0,0);
	LCD_Send_String("System Init...");
	HAL_Delay(1000); 
	LCD_Clear();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		current_task = Pop_Queue();
		switch(current_task){
			case TASK_SHT31:
			Read_SHT31();
			break;
			
			case TASK_SOIL:
			Read_Soil();
			break;
			
			case TASK_LCD:
			Write_LCD(current_display);
			break;
			
			case TASK_PARSE_CMD:
			Process_Command();
			break;
			
			case NO_TASK:
			//Stop Systick
			HAL_SuspendTick(); 
			//Sleep
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			//Start Systick
			HAL_ResumeTick();
			
			break;
}
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
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

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
