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
#include "blinkers.h"
#include "motor_control.h"
#include "throuple_can.h"
#include "adc_sense.h"
#include "servo_control.h"
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


osThreadId defaultTaskHandle;
osThreadId mcuStatusHandle;
osThreadId motorControlHandle;
/* USER CODE BEGIN PV */
osThreadId canRecieveHandle;
osThreadId canTransmitHandle;
osThreadId selfTestHandle;
osThreadId steeringHandle;
static QueueHandle_t xQueueMotor;
static QueueHandle_t xQueueSteering;
static QueueHandle_t xQueueMotorState;
static QueueHandle_t xQueueCANState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void blink(void const * argument);
void motor_controller(void const * argument);
void self_test(void const * argument);
void steering_task(void const * argument);


/* USER CODE END PFP */
void can_rx_rear(void const * argument);
void can_rx_front(void const * argument);
void can_tx_rear(void const * argument);
void can_tx_front(void const * argument);
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
  HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_3_GPIO_Port, GPIO_3_Pin, GPIO_PIN_SET);
  GPIO_PinState zone_indicator = HAL_GPIO_ReadPin(GPIO_2_GPIO_Port,GPIO_2_Pin);
  /* USER CODE BEGIN 2 */
  blinkers_init();
  can_init();
  // Front Zone is set and Rear is reset (re re)
  if (zone_indicator == GPIO_PIN_RESET){ // Rear
	osThreadDef(motorControl, motor_controller, osPriorityHigh, 0, 128);
	motorControlHandle = osThreadCreate(osThread(motorControl), NULL);
	osThreadDef(canRecieve, can_rx_rear, osPriorityHigh, 0, 128);
	canRecieveHandle = osThreadCreate(osThread(canRecieve), NULL);
	osThreadDef(canTransmit, can_tx_rear, osPriorityNormal, 0, 128);
	canTransmitHandle = osThreadCreate(osThread(canTransmit), NULL);
  }
  else{ // Front
	servo_init();
	current_sense_init();
	pot_sense_init();
	osThreadDef(canRecieve, can_rx_front, osPriorityHigh, 0, 128);
	canRecieveHandle = osThreadCreate(osThread(canRecieve), NULL);
	osThreadDef(steering, steering_task, osPriorityHigh, 0, 128);
	steeringHandle = osThreadCreate(osThread(steering), NULL);
	osThreadDef(canTransmit, can_tx_front, osPriorityNormal, 0, 128);
	canTransmitHandle = osThreadCreate(osThread(canTransmit), NULL);
  }


  extern QueueHandle_t xQueueCANRx;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of mcuStatus */
//  osThreadDef(mcuStatus, blink, osPriorityNormal, 0, 128);
//  mcuStatusHandle = osThreadCreate(osThread(mcuStatus), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */




  osThreadDef(selfTest, self_test, osPriorityNormal, 0, 128);
  selfTestHandle = osThreadCreate(osThread(selfTest), NULL);



  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HAL_GPIO_TogglePin(MCU_IND_GPIO_Port,MCU_IND_Pin);
	  HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_IND_GPIO_Port, MCU_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN2_Pin|MOTOR_IN1_Pin|GPIO_1_Pin|GPIO_2_Pin
                          |GPIO_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_IND_Pin */
  GPIO_InitStruct.Pin = MCU_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_IND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_IN2_Pin MOTOR_IN1_Pin GPIO_1_Pin GPIO_2_Pin
                           GPIO_3_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN2_Pin|MOTOR_IN1_Pin|GPIO_1_Pin|GPIO_2_Pin
                          |GPIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/**
* @brief Function implementing the can receive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blink */
void can_rx_rear(void const * argument)
{
  /* USER CODE BEGIN blink */
	can_msg_t msg;
	xQueueMotor = xQueueCreate( 10,sizeof(pi_motor_command));
  /* Infinite loop */
  for(;;)
  {
	if( xQueueCANRx != NULL )
	{
	  if( xQueueReceive( xQueueCANRx,
						 &( msg ),
						 ( TickType_t ) 0 ))
	  {
		 if (msg.id == 0x100){
			 set_blinkers(msg.msg[3],msg.msg[4],msg.msg[5]);
			 pi_motor_command motor_command;
			 motor_command.brake = msg.msg[0];
			 motor_command.throttle = msg.msg[1];
			 xQueueSend(xQueueMotor, &motor_command,( TickType_t ) 10);
		 }
		 else if (msg.id == 0x446){
			 HAL_GPIO_TogglePin(MCU_IND_GPIO_Port, MCU_IND_Pin);
		 }
	  }
	}
	osDelay(10);
  }
  /* USER CODE END blink */
}


void can_rx_front(void const * argument)
{
  /* USER CODE BEGIN blink */
	can_msg_t msg;
	xQueueSteering = xQueueCreate( 10,sizeof(uint8_t));
  /* Infinite loop */
  for(;;)
  {
	if( xQueueCANRx != NULL )
	{
	  if( xQueueReceive( xQueueCANRx,
						 &( msg ),
						 ( TickType_t ) 0 ))
	  {
		 if (msg.id == 0x100){
			 set_blinkers(msg.msg[3],msg.msg[4],msg.msg[5]);
			 uint8_t steering_angle = msg.msg[2];
			 xQueueSend(xQueueSteering, &steering_angle,( TickType_t ) 10);
		 }
		 else if (msg.id == 0x446){
			 HAL_GPIO_TogglePin(MCU_IND_GPIO_Port, MCU_IND_Pin);
		 }
	  }
	}
	osDelay(10);
  }
  /* USER CODE END blink */
}


/**
* @brief Function implementing the can transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blink */
void can_tx_rear(void const * argument)
{
  /* Infinite loop */
	uint8_t data[8];
	uint16_t id = 0x300;
  zone_state_e zone_state = NORMAL;
  zone_state_e zone_state_queue = NORMAL;
  for(;;)
  {
	if (xQueueCANState != NULL){
		if (xQueueReceive(xQueueCANState, &zone_state_queue, ( TickType_t ) 0) == pdPASS){
			zone_state = zone_state_queue;
		}
	}
	switch(zone_state){
	case NORMAL:
		throuple_can_tx(id, data);
		break;
	case NORMAL_PUSHED:
		throuple_can_tx(id, data);
		break;
	case ERROR_BUTTON:
		break;
	case ERROR_BUTTON_RELEASED:
		break;
	case ERROR_HB:
		throuple_can_tx(id, data);
		break;
	}
	osDelay(20);
  }
}

void can_tx_front(void const * argument)
{
	/* Infinite loop */
	uint8_t data[8];
	uint16_t id = 0x200;
  zone_state_e zone_state = NORMAL;
  zone_state_e zone_state_queue = NORMAL;
  for(;;)
  {
	if (xQueueCANState != NULL){
		if (xQueueReceive(xQueueCANState, &zone_state_queue, ( TickType_t ) 0) == pdPASS){
			zone_state = zone_state_queue;
		}
	}
	// fill in with data
	uint16_t servo_current = current_sense_read();
	uint16_t servo_pot = pot_sense_read();
	data[0] = (uint8_t)(servo_current >> 8);
	data[1] = (uint8_t)(servo_current & 0xFF);
	data[2] = (uint8_t)(servo_pot >> 8);
	data[3] = (uint8_t)(servo_pot & 0xFF);

	switch(zone_state){
	case NORMAL:
		throuple_can_tx(id, data);
		break;
	case NORMAL_PUSHED:
		throuple_can_tx(id, data);
		break;
	case ERROR_BUTTON:
		break;
	case ERROR_BUTTON_RELEASED:
		break;
	case ERROR_HB:
		throuple_can_tx(id, data);
		break;
	}
	osDelay(20);
  }
}

void motor_control(zone_state_e state) {
	pi_motor_command motor_command;
	motor_direction dir;
	uint8_t speed;
	// Receive CAN
	if (xQueueMotor != NULL){
		if (xQueueReceive(xQueueMotor, &motor_command, ( TickType_t ) 0)){
			// Received motor message
			if (motor_command.brake!= 0){
				dir = BRAKE;
				speed = motor_command.brake;
			}
			else {
				dir = FORWARD;
				speed = motor_command.throttle;
			}
			// Adjust motor speed
			if (state == NORMAL || state == NORMAL_PUSHED){
				if (dir == FORWARD){
					set_drive_speed(speed);
				}
				else if (dir == BRAKE){
					set_brake_speed(speed);
				}
			}
		}
	}
	if (!(state == NORMAL || state == NORMAL_PUSHED)){
		set_brake_speed(100);
	}
}

// Motor Controller Task
void motor_controller(void const * argument){
	motor_init();
	zone_state_e zone_state_queue = NORMAL;
	zone_state_e zone_state = NORMAL;

	for(;;){
		// Receive State
		if (xQueueMotorState != NULL){
			if (xQueueReceive(xQueueMotorState, &zone_state_queue, ( TickType_t ) 0) == pdPASS){
				zone_state = zone_state_queue;
			}
		}
		motor_control(zone_state);

		osDelay(10);
	}
}

// Steering Angle Task
void steering_task(void const * argument){
	uint8_t steering_angle;
	uint8_t steering_angle_queue;
	for(;;){
		// Receive State
		if (xQueueSteering != NULL){
			if (xQueueReceive(xQueueSteering, &steering_angle_queue, ( TickType_t ) 0) == pdPASS){
				steering_angle = steering_angle_queue;
			}
		}
		set_servo_pos(steering_angle);
		osDelay(10);
	}
}

// Self-Test Button Task
void self_test(void const * argument){
	zone_state_e zone_state = NORMAL;
	uint8_t button_debounce = 0;
	uint8_t button_state = 0;

	xQueueMotorState = xQueueCreate( 10,sizeof(zone_state_e));
	xQueueCANState = xQueueCreate( 10,sizeof(zone_state_e));


	for (;;){
		GPIO_PinState button = HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin);
		// debounce button
		if (button == GPIO_PIN_SET){
			button_debounce = (button_debounce << 1) | 1;
		}
		else {
			button_debounce = button_debounce << 1;
		}
		if (button_debounce == 0xFF && button_state == 0){
			button_state = 1;
		}
		else if (button_debounce == 0 && button_state == 1){
			button_state = 0;
		}
		switch (zone_state) {
			case NORMAL:
				if (button_state == 1) {
					HAL_GPIO_WritePin(MCU_IND_GPIO_Port,MCU_IND_Pin,GPIO_PIN_SET);
					zone_state = ERROR_BUTTON;
				}
				break;
			case NORMAL_PUSHED:
				if (button_state == 0){
					zone_state = NORMAL;
					HAL_GPIO_WritePin(MCU_IND_GPIO_Port,MCU_IND_Pin,GPIO_PIN_RESET);
				}
				break;
			case ERROR_BUTTON:
				if (button_state == 0) {
					zone_state = ERROR_BUTTON_RELEASED;
					HAL_GPIO_WritePin(MCU_IND_GPIO_Port,MCU_IND_Pin,GPIO_PIN_SET);
				}
				break;
			case ERROR_BUTTON_RELEASED:
				if (button_state == 1){
					zone_state = NORMAL_PUSHED;
					HAL_GPIO_WritePin(MCU_IND_GPIO_Port,MCU_IND_Pin,GPIO_PIN_RESET);
				}
				break;
		}
		xQueueSend(xQueueMotorState, &zone_state,( TickType_t ) 10);
		xQueueSend(xQueueCANState, &zone_state,( TickType_t ) 10);
		osDelay(10);
	}
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
