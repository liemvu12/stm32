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
#include "cmsis_os.h"
#include "CLCD_I2C.h"
#include <stdio.h>
#include "Setpoint_Interrupt.h"
#include "DS18B20.h"
#include "delay_timer.h"
#include "math.h"
#include "i2c-lcd.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* Definitions for Read_Sensor_Tas */
osThreadId_t Read_Sensor_TasHandle;
const osThreadAttr_t Read_Sensor_Tas_attributes = {
  .name = "Read_Sensor_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Send_Task */
osThreadId_t Send_TaskHandle;
const osThreadAttr_t Send_Task_attributes = {
  .name = "Send_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Setpoint_Task */
osThreadId_t Setpoint_TaskHandle;
const osThreadAttr_t Setpoint_Task_attributes = {
  .name = "Setpoint_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for PID_Control_Tas */
osThreadId_t PID_Control_TasHandle;
const osThreadAttr_t PID_Control_Tas_attributes = {
  .name = "PID_Control_Tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PID1 */
osMessageQueueId_t PID1Handle;
const osMessageQueueAttr_t PID1_attributes = {
  .name = "PID1"
};
/* Definitions for PID2 */
osMessageQueueId_t PID2Handle;
const osMessageQueueAttr_t PID2_attributes = {
  .name = "PID2"
};
/* Definitions for Send_Data */
osMessageQueueId_t Send_DataHandle;
const osMessageQueueAttr_t Send_Data_attributes = {
  .name = "Send_Data"
};
/* Definitions for xI2CSemaphore */
osSemaphoreId_t xI2CSemaphoreHandle;
const osSemaphoreAttr_t xI2CSemaphore_attributes = {
  .name = "xI2CSemaphore"
};
/* USER CODE BEGIN PV */
CLCD_I2C_Name LCD1;
BUTTON_Setting SETTING;

DS18B20_Name DS1;
float Temp;
float Data  = 25;
char data_send[5];

float Kp = 5;                      
float Ki = 0.005;                     
float Kd = 45;   

float prev_error = 0.0;
float integral = 0.0;

void PID_Control(float current_temperature, float setpoint_temperature) {
    float error = - setpoint_temperature + current_temperature;
    float pid_output = Kp * error + Ki * integral + Kd * (error - prev_error);

    if (pid_output > 100.0) {
        pid_output = 100.0;
    } else if (pid_output < 0.0) {
        pid_output = 0.0;
    }

    integral += error;
    prev_error = error;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(pid_output * htim2.Init.Period / 100.0));
}

void Printf_Sever(char *data_send, float Temp){
		sprintf(data_send, "Temperature: %.2f\n", Temp);
    HAL_UART_Transmit(&huart1, (uint8_t*)data_send, strlen(data_send), HAL_MAX_DELAY);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void StartRead_Sensor_Task(void *argument);
void StartSend_Task(void *argument);
void StartSetpoint_Task(void *argument);
void StartPID_Control_Task(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1);
	CLCD_I2C_Init(&LCD1,&hi2c1,0x4e,16,2);
	BUTTON_Setting_Init( &SETTING, BUTTON_SETTING_GPIO_Port, BUTTON_SETTING_Pin);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	lcd_init();
	DS18B20_Init(&DS1, &htim4, DS18B20_GPIO_Port, DS18B20_Pin);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xI2CSemaphore */
  xI2CSemaphoreHandle = osSemaphoreNew(1, 1, &xI2CSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of PID1 */
  PID1Handle = osMessageQueueNew (16, sizeof(uint16_t), &PID1_attributes);

  /* creation of PID2 */
  PID2Handle = osMessageQueueNew (16, sizeof(uint16_t), &PID2_attributes);

  /* creation of Send_Data */
  Send_DataHandle = osMessageQueueNew (16, sizeof(uint16_t), &Send_Data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Read_Sensor_Tas */
  Read_Sensor_TasHandle = osThreadNew(StartRead_Sensor_Task, NULL, &Read_Sensor_Tas_attributes);

  /* creation of Send_Task */
  Send_TaskHandle = osThreadNew(StartSend_Task, NULL, &Send_Task_attributes);

  /* creation of Setpoint_Task */
  Setpoint_TaskHandle = osThreadNew(StartSetpoint_Task, NULL, &Setpoint_Task_attributes);

  /* creation of PID_Control_Tas */
  PID_Control_TasHandle = osThreadNew(StartPID_Control_Task, NULL, &PID_Control_Tas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Trans_Pin|Task3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Task2_Pin|Task1_Pin|Task0_Pin|DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Trans_Pin Task3_Pin */
  GPIO_InitStruct.Pin = Trans_Pin|Task3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Task2_Pin Task1_Pin Task0_Pin DS18B20_Pin */
  GPIO_InitStruct.Pin = Task2_Pin|Task1_Pin|Task0_Pin|DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_SETTING_Pin */
  GPIO_InitStruct.Pin = BUTTON_SETTING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_SETTING_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRead_Sensor_Task */
/**
  * @brief  Function implementing the Read_Sensor_Tas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRead_Sensor_Task */
void StartRead_Sensor_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	__IO uint32_t tick1 = osKernelGetTickCount();
  	int i = 0; 
  /* Infinite loop */
  for(;;)
  {
		Temp = DS18B20_ReadTemp(&DS1);
		HAL_GPIO_TogglePin (Task0_GPIO_Port, Task0_Pin) ;
		osMessageQueuePut(PID1Handle , &Temp ,0 ,0);
		i ++;
		if(i ==10) {
			i = 0 ;		
			osMessageQueuePut(Send_DataHandle , &Temp, 0, 0);
		}
    	tick1 += 200;
		osDelayUntil (tick1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSend_Task */
/**
* @brief Function implementing the Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSend_Task */
void StartSend_Task(void *argument)
{
  /* USER CODE BEGIN StartSend_Task */
	__IO uint32_t tick2 = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
  osMessageQueueGet(Send_DataHandle, &Temp, 0, osWaitForever);
	HAL_GPIO_TogglePin (Task1_GPIO_Port, Task1_Pin) ;
	Printf_Sever(data_send, Temp) ;
	tick2 += 2000;
	osDelayUntil(tick2);
  }
  /* USER CODE END StartSend_Task */
}

/* USER CODE BEGIN Header_StartSetpoint_Task */
/**
* @brief Function implementing the Setpoint_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetpoint_Task */
void StartSetpoint_Task(void *argument)
{
  /* USER CODE BEGIN StartSetpoint_Task */
	__IO uint32_t tick3 = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    tick3 +=200;
		Data = HAL_ADC_GetValue(&hadc1);
		Data = mapToRange(Data, 10, 41);
	osMessageQueuePut(PID2Handle, &Data, 0, 0);
	HAL_GPIO_TogglePin (Task2_GPIO_Port, Task2_Pin) ;
	osDelayUntil(tick3);
  }
  /* USER CODE END StartSetpoint_Task */
}

/* USER CODE BEGIN Header_StartPID_Control_Task */
/**
* @brief Function implementing the PID_Control_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPID_Control_Task */
void StartPID_Control_Task(void *argument)
{
  /* USER CODE BEGIN StartPID_Control_Task */
  __IO uint32_t tick4 = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {	
  tick4 +=200 ;
	osMessageQueueGet(PID1Handle, &Temp, 0, osWaitForever);
	osMessageQueueGet(PID2Handle, &Data, 0, osWaitForever);
	PID_Control(Temp, Data);
	HAL_GPIO_TogglePin (Task3_GPIO_Port, Task3_Pin) ;
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Setpoint: ");
		sprintf(data_send,"%2.2f",Data);
		lcd_send_string(data_send);
		lcd_put_cur(1,2);
    lcd_send_string("T = ");
    lcd_put_cur(1,11);
    lcd_send_data(223);
    lcd_put_cur(1,12);
    lcd_send_string("C");    		
		sprintf(data_send,"%2.2f",Temp);
		lcd_send_cmd(0x80|0x46);
		lcd_send_string(data_send);
		if (Temp <=25 ) HAL_GPIO_WritePin(Trans_GPIO_Port,Trans_Pin,1);
		else if (Temp >=30 ) HAL_GPIO_WritePin(Trans_GPIO_Port,Trans_Pin,0);
		HAL_Delay(100);
	osDelayUntil(tick4);
  }
  /* USER CODE END StartPID_Control_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
