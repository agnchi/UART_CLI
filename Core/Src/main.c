/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdlib.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define BUFFER_SIZE 10


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
typedef struct Time
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} Time;
typedef struct TimeStr
{
	char hour[BUFFER_SIZE];
	char minute[BUFFER_SIZE];
	char second[BUFFER_SIZE];
} TimeStr;
char buffer[BUFFER_SIZE];
int buffer_index=0;
char START_MESSAGE[350]="\r\nWELCOME!\r\nAVAILABLE COMMANDS:\r\nON - turn on LED\r\nOFF- turn off LED\r\ntoggle - change LED state to oposite\r\nset_time - set current time\r\nstart_time - start the counter\r\nstop_time - stop the counter\r\nhelp - show this message again\r\n";
char NEW_LINE[8]="\r\n<<  ";
const uint8_t ENTER_CHAR = 0x0d;
const uint8_t SEMICOLON_CHAR = 0x3b;
char received;
char on_command[BUFFER_SIZE]="ON";
char off_command[BUFFER_SIZE]="OFF";
char help_command[BUFFER_SIZE]="help";
char toggle_command[BUFFER_SIZE]="toggle";
char time_command[BUFFER_SIZE]="set_time";
char start_command[BUFFER_SIZE]="start_time";
char stop_command[BUFFER_SIZE]="stop_time";
Time time={.hour=0,.minute=0,.second=0};
TimeStr time_str;
uint8_t set_timeON=0;
uint8_t interval_var=0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2,START_MESSAGE, sizeof(START_MESSAGE),100);
  HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
  Clear_Buffer();

//  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, rx_buff_size);
 // __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
  HAL_UART_Receive_IT(&huart2, &received, 1);




  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Clear_Buffer()
{
	memset(buffer, '\0', BUFFER_SIZE*sizeof(uint8_t));
}

void Update_Time(Time* time)
{
	time->second++;
	if((time->second)>59)
	{
		time->second=0;
		time->minute++;
		if(time->minute>59)
			{
				time->minute=0;
				time->hour++;
				if(time->hour>23)
				{
					time->hour=0;
				}

			}
	}

}

void TimetoString(Time *time,TimeStr*time_str)
{
	itoa(time->hour,time_str->hour,10);
	itoa(time->minute,time_str->minute,10);
	itoa(time->second,time_str->second,10);

}

void Print_Time(TimeStr *time_str)
{

	HAL_UART_Transmit(&huart2, "time: ",6,100);
	HAL_UART_Transmit(&huart2, time_str->hour,2,100);
	HAL_UART_Transmit(&huart2, ":",1,100);
	HAL_UART_Transmit(&huart2, time_str->minute,2,100);
	HAL_UART_Transmit(&huart2, ":",1,100);
	HAL_UART_Transmit(&huart2, time_str->second,2,100);
	HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
}

void StringtoTime(Time *time,TimeStr*time_str)
{
	time->hour=atoi(time_str->hour);
	time->minute=atoi(time_str->minute);
	time->second=atoi(time_str->second);
}

void Handle_Commands()
{
	if(!strcmp(buffer,on_command))
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		HAL_UART_Transmit(&huart2, "LED turned on", 13,100);
		HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE),100);
	}
	else if(!strcmp(buffer,off_command))
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		HAL_UART_Transmit(&huart2, "LED turned off ", 14,100);
		HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
	}
	else if(!strcmp(buffer,toggle_command))
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_UART_Transmit(&huart2, "LED Toggled ", 12,100);
		HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
	}
	else if(!strcmp(buffer,help_command))
	{
		HAL_UART_Transmit(&huart2, START_MESSAGE, sizeof(START_MESSAGE),100);
		HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
	}
	else if(!strcmp(buffer,start_command))
	{
		HAL_TIM_Base_Start_IT(&htim16);
		TimetoString(&time, &time_str);
		Print_Time(&time_str);
		uint8_t interval_var=0;
	}
	else if(!strcmp(buffer,stop_command))
	{
		HAL_TIM_Base_Stop_IT(&htim16);
		uint8_t interval_var=0;

	}
	else if(!strcmp(buffer,time_command))
	{

		HAL_UART_Transmit(&huart2, "Enter time in format <hh:mm:ss>; :",33,100);
		set_timeON=1;
		HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);

	}
	else
	{
		HAL_UART_Transmit(&huart2, "Unknown Command", 15,100);
		HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
	}
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	if(htim==&htim16)
	{
		Update_Time(&time);

		if(interval_var<9)
		{
			interval_var++;

		}
		else
		{
			TimetoString(&time, &time_str);
			Print_Time(&time_str);
			interval_var=0;


		}



	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART2)
	{

		if(!strcmp(received,ENTER_CHAR))
		{
			HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
			Handle_Commands();
			Clear_Buffer();
			buffer_index=0;
		}
		else if((set_timeON==1)&&(!strcmp(received,SEMICOLON_CHAR)))
		{
			HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);

			memcpy(time_str.hour,buffer,2);
			memcpy(time_str.minute,buffer+3,2);
			memcpy(time_str.second,buffer+6,2);
			set_timeON=0;
			Clear_Buffer();
			buffer_index=0;
			StringtoTime(&time,&time_str);
			TimetoString(&time, &time_str);
			Print_Time(&time_str);
			interval_var=0;
		}

		else
		{
			HAL_UART_Transmit(&huart2,&received, 1, 100);
			//memcpy(buffer+buffer_index,received,1);
			buffer[buffer_index]=received;
			buffer_index++;
		}

		HAL_UART_Receive_IT(&huart2, &received, 1);
		if(buffer_index>BUFFER_SIZE)
		{
			HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
			HAL_UART_Transmit(&huart2,"buffer size exceeded", 30, 100);
			HAL_UART_Transmit(&huart2,NEW_LINE, sizeof(NEW_LINE), 100);
			Handle_Commands();
			Clear_Buffer();
			buffer_index=0;
		}

	}

}

void HAL_USART_TxCpltCallback(UART_HandleTypeDef *huart)
{

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
