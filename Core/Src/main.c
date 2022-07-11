/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

#include "stdbool.h"
#include "queue.h"

char uart_rx_data[50] = {0};			// Main rx buffer data
char str[1] = {0};						// Buffer for one char
bool flag_command_received = false;		// Flag show status receive data (completed/not completed)
uint8_t rx_data_counter = 0;


typedef struct
{
	char TX_GSM[40];
}TX_GCM_COMAND;

TX_GCM_COMAND TX_GCM_COMAND_t;


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
 UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gsm_task */
osThreadId_t gsm_taskHandle;
const osThreadAttr_t gsm_task_attributes = {
  .name = "gsm_task",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for parsingGSMcom */
osThreadId_t parsingGSMcomHandle;
const osThreadAttr_t parsingGSMcom_attributes = {
  .name = "parsingGSMcom",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for resoursesMonito */
osThreadId_t resoursesMonitoHandle;
const osThreadAttr_t resoursesMonito_attributes = {
  .name = "resoursesMonito",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for command_from_GCM */
osMessageQueueId_t command_from_GCMHandle;
const osMessageQueueAttr_t command_from_GCM_attributes = {
  .name = "command_from_GCM"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void Start_gsm_task(void *argument);
void StartParsingGSMCom(void *argument);
void StartResoursesMonitor(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//int g = 999;   Отримання одного байту від GSM МОДУЛЯ


	if(huart == &huart1)											// Which uart generate Callback function
	{
		// '\r' -
		// '\n' - 10
		if((str[0] == 0x0D) || (rx_data_counter >= sizeof(uart_rx_data)))   		// Detect '\0' Null or  data too long
		{
			flag_command_received = true;                           // Data is ready
			rx_data_counter = 0;

			// WORK WITC RECEIVED COMMAND
			char OK_str[] = "OK";
			if((strstr(uart_rx_data, OK_str)) != NULL)		// Work
			{
				int hhdd = 888;
			}

			if((strstr(uart_rx_data, "+380931482354")) != NULL)  // Work
			{
				int hhdddd = 888;
			}





			// 1. Записати дані в чергу
//			BaseType_t xHigherPriorityTaskWoken;
//			if(xQueueSendFromISR( command_from_GCMHandle, &uart_rx_data, &xHigherPriorityTaskWoken) != pdTRUE)
//			{
//				// error
//				int hhhh = 999;
//			}
			// xQueueOverwrite( TX_GCM_COMAND_t, &uart_rx_data );
			// 2. Виставити семафор

			//memcpy(TX_GCM_COMAND_t, uart_rx_data, sizeof(uart_rx_data));

//			else
//			{
//				int gfff = 999;
//				// error
//			}


			//Передача даних через чергу в потік, де буде аналізуватися комінди
			memset(uart_rx_data, 0, sizeof(uart_rx_data));




			HAL_UART_Receive_IT(&huart1, str, 1);
		}
		else
		{
			flag_command_received = false;							// Receive data
			uart_rx_data[rx_data_counter] = str[0];					// Save data in global buffer
			HAL_UART_Receive_IT(&huart1, str, 1);					// Turn on "HAL_UART_Receive_IT"
			rx_data_counter ++;
		}
	}



		//////////////////////////////////////////////////////
//		uint8_t i = 0;
//				uint8_t sizeo_of_RX_buffer = sizeof(rx_response_from_gsm_module)-1;
//
//				do
//				{
//					HAL_UART_Receive_IT(&huart1, &rx_uart, 1);
//					rx_response_from_gsm_module[i] = rx_uart;
//					i++;
//				}while((rx_uart != 13) && (i <= sizeo_of_RX_buffer ));    // 0x0D  or 13
//				i = 0;
		////////////////////////////////////////////////////////

		// Аналіз команди яка прийшла



//		HAL_UART_Receive_IT(&huart1, &rx_uart, 1);



}
///////////////////////////////////////////////////////////////////////////////
// AT Commands


//-------------------------------------------------------------------------------
bool gsm_send_at_command(char *cmd)
{

	return true;
}
//-------------------------------------------------------------------------------
bool init_gsm_module(void)
{

	return true;
}
//-------------------------------------------------------------------------------
void meke_call_to_me(void)
{

}
//-------------------------------------------------------------------------------
void send_sme_to_me(void)
{

}
//-------------------------------------------------------------------------------
void send_sms_with_location(void)
{


}
//-------------------------------------------------------------------------------


//////////////////// MAKE test CALL
//osDelay(3000);
//
//char AT_COMAND[] = "AT\n\r"; 				// Must return OK
//char AT_COMAND_ATE0[] = "ATE0\n\r";
//char AT_COMAND_AT_CLIP[] = "AT+CLIP\n\r"; 			// AT+CLIP=1
//char AT_COMAND_TURN_OFF_ECHO[] = "ATV0n\r";
//char AT_COMAND_AT_COPS[] = "AT+COPS?\n\r"; 		// Інформація про оператора. Вертає +COPS: 0,0,»MTS-RUS»  OK
//char AT_COMAND_AT_CPAS[] = "AT+CPAS\n\r";			// Інформація про стан модуля 0 – готов к работе, 	2 – неизвестно, 3 – входящий звонок, 4 – голосовое соединение
//char AT_COMAND_AT_CSQ[] = "AT+CSQ\n\r";				// Рівень сигналу
//
//char AT_COMAND_MAKE_CALL_ON_NUMBER[] = "ATD+ 380931482354;\n\r";     // Зробити звінок по номеру
//
////HAL_UART_Transmit(&huart1, AT_COMAND, 10, 1000);
//
//HAL_UART_Transmit_IT(&huart1, AT_COMAND, sizeof(AT_COMAND));
//osDelay(100);
//HAL_UART_Transmit_IT(&huart1, AT_COMAND_ATE0, sizeof(AT_COMAND_ATE0));
//osDelay(100);
//HAL_UART_Transmit_IT(&huart1, AT_COMAND_AT_CLIP, sizeof(AT_COMAND_AT_CLIP));
//osDelay(100);
//HAL_UART_Transmit_IT(&huart1, AT_COMAND_TURN_OFF_ECHO, sizeof(AT_COMAND_TURN_OFF_ECHO));
//osDelay(3000);
//
//HAL_UART_Transmit_IT(&huart1, AT_COMAND_MAKE_CALL_ON_NUMBER, sizeof(AT_COMAND_MAKE_CALL_ON_NUMBER));			// Work !
//osDelay(100);
///////////////////////////////

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of command_from_GCM */
  command_from_GCMHandle = osMessageQueueNew (1, sizeof(TX_GCM_COMAND), &command_from_GCM_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of gsm_task */
  gsm_taskHandle = osThreadNew(Start_gsm_task, NULL, &gsm_task_attributes);

  /* creation of parsingGSMcom */
  parsingGSMcomHandle = osThreadNew(StartParsingGSMCom, NULL, &parsingGSMcom_attributes);

  /* creation of resoursesMonito */
  resoursesMonitoHandle = osThreadNew(StartResoursesMonitor, NULL, &resoursesMonito_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */



//  HAL_UART_Receive_IT(&huart1, &rx_uart, 1);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_UART_Receive_IT(&huart1, str, 1);
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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	  osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_gsm_task */
/**
* @brief Function implementing the gsm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_gsm_task */
void Start_gsm_task(void *argument)
{
  /* USER CODE BEGIN Start_gsm_task */
  /* Infinite loop */

	//HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
	//HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

	// 1. Надсилання команди АТ
	osDelay(3000);

	// Налаштування і ініціалізація
	char AT_COMAND[] = "AT\n\r"; 				// Must return OK
	char AT_COMAND_ATE0[] = "ATE0\n\r";			// Виключити ехо
	char AT_COMAND_ATE1[] = "ATE1\n\r";			// Включити ехо
	char AT_COMAND_TURN_OFF_ECHO[] = "ATV0\n\r";	// Тільки відповідь
	char AT_COMAND_TURN_ON_ECHO[] = "ATV1\n\r";	// Повна відповідь з ехо
	char AT_COMAND_АОН[] = "AT+CLIP=1\n\r";    // ??????
	//char AT_COMAND_

	// ЗВІНКИ

	char AT_COMAND_MAKE_CALL_ON_NUMBER[] = "ATD+ 380931482354;\n\r";     // Зробити звінок по номеру
	char AT_COMMAND_PICK_UP_PHONE[] = "ATA\n\r";			// Підняти трубку
	char AT_COMAND_END_CALL[] = "ATH0\n\r";				// Повісити трубку
//	char AT_COMAND_
//	char AT_COMAND_
//	char AT_COMAND_

	// SMS
//	char AT_COMAND_
//	char AT_COMAND_
//	char AT_COMAND_


	char AT_COMAND_AT_CLIP[] = "AT+CLIP=1\n\r"; 			// Включення визначника вхідного номера
	char AT_COMAND_AT_COPS[] = "AT+COPS?\n\r"; 		// Інформація про оператора. Вертає +COPS: 0,0,»MTS-RUS»  OK
	char AT_COMAND_AT_CPAS[] = "AT+CPAS\n\r";			// Інформація про стан модуля 0 – готов к работе, 	2 – неизвестно, 3 – входящий звонок, 4 – голосовое соединение
	char AT_COMAND_AT_CSQ[] = "AT+CSQ\n\r";				// Рівень сигналу



	HAL_UART_Transmit(&huart1, AT_COMAND, 10, 1000);

	HAL_UART_Transmit_IT(&huart1, AT_COMAND, sizeof(AT_COMAND));
	osDelay(100);
	HAL_UART_Transmit_IT(&huart1, AT_COMAND_ATE0, sizeof(AT_COMAND_ATE0));
	osDelay(100);
	HAL_UART_Transmit_IT(&huart1, AT_COMAND_AT_CLIP, sizeof(AT_COMAND_AT_CLIP));
	osDelay(100);
	HAL_UART_Transmit_IT(&huart1, AT_COMAND_АОН, sizeof(AT_COMAND_АОН));
	osDelay(100);
	HAL_UART_Transmit_IT(&huart1, AT_COMAND_TURN_OFF_ECHO, sizeof(AT_COMAND_TURN_OFF_ECHO));
	osDelay(3000);

//
//	HAL_UART_Transmit_IT(&huart1, AT_COMAND_MAKE_CALL_ON_NUMBER, sizeof(AT_COMAND_MAKE_CALL_ON_NUMBER));			// Work !
//	osDelay(100);

  for(;;)
  {

	  //HAL_UART_Transmit_IT(&huart1, AT_COMAND, sizeof(AT_COMAND));
	  osDelay(5000);


	 // HAL_UART_Transmit_IT(&huart1, AT_COMMAND_PICK_UP_PHONE, sizeof(AT_COMMAND_PICK_UP_PHONE));


	  //HAL_UART_Transmit(&huart1, AT_COMAND, 10, 1000);
//	  HAL_UART_Transmit_IT(&huart1, AT_COMAND_AT_COPS, sizeof(AT_COMAND_AT_COPS));
//	  osDelay(1000);
//
//	  HAL_UART_Transmit_IT(&huart1, AT_COMAND_AT_CPAS, sizeof(AT_COMAND_AT_CPAS));
//	  osDelay(1000);
//
//	  HAL_UART_Transmit_IT(&huart1, AT_COMAND_AT_CSQ, sizeof(AT_COMAND_AT_CSQ));



  }
  /* USER CODE END Start_gsm_task */
}

/* USER CODE BEGIN Header_StartParsingGSMCom */
/**
* @brief Function implementing the parsingGSMcom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParsingGSMCom */
void StartParsingGSMCom(void *argument)
{
  /* USER CODE BEGIN StartParsingGSMCom */
  /* Infinite loop */
  for(;;)
  {
	  char tx_gsm_data[50] = {0};
	  if (xQueueReceive(command_from_GCMHandle, tx_gsm_data, 0) == pdTRUE)		// Read witch button was pressed
	  {
		  // Command was received
		  int f = 1111;

		  char OK_str[] = "OK";
		  if((strstr(tx_gsm_data, OK_str)) != NULL)		// Work
		  {
			  int hhdd = 888;
		  }

		  if((strstr(tx_gsm_data, "+380931482354")) != NULL)  // Work
		  {
			  int hhdddd = 888;
		  }
	  }

    osDelay(1);
  }
  /* USER CODE END StartParsingGSMCom */
}

/* USER CODE BEGIN Header_StartResoursesMonitor */
/**
* @brief Function implementing the resoursesMonito thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartResoursesMonitor */
void StartResoursesMonitor(void *argument)
{
  /* USER CODE BEGIN StartResoursesMonitor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartResoursesMonitor */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
