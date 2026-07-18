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
#include <stdio.h>
#include <string.h>
#include "can_bus.hpp"
#include "message.hpp"
#include "sensor_unit.hpp"
#include "control_unit.hpp"
#include "actuator_unit.hpp"
#include "cmsis_os.h" // FreeRTOS CMSIS-V2 wrapper
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


UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim3;
// shared between EXTI0 ISR and SensorUnit
volatile uint32_t echo_rise_time = 0;   // timestamp when ECHO went HIGH
volatile uint32_t echo_fall_time = 0;   // timestamp when ECHO went LOW
volatile bool echo_ready = false;        // true when a full reading is available
// global system objects (must be global so all 3 tasks can access them)
static CANBus<SensorMessage>  sensor_bus;
static CANBus<ControlMessage> control_bus;
static SensorUnit*  sensor_ptr; //pointers bc it helps us control construction
static ControlUnit* control_ptr;
static ActuatorUnit* actuator_ptr;
// for mutex (to protect actuator against its own repeated iterations overlapping)
osMutexId_t print_mutex;
IWDG_HandleTypeDef hiwdg;  // watchdog timer or IWDG
volatile uint8_t task_checkin = 0;  // each task sets its bit when it completes a cycle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" int __io_putchar(int ch) //sends printf to UART2 and then to terminal
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

extern "C" void vApplicationStackOverflowHook(osThreadId_t xTask, char *pcTaskName)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)"STACK OVERFLOW: ", 17, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)pcTaskName, strlen(pcTaskName), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    while(1);
}

// sensor task: fires TRIG, waits for echo ISR, sends SensorMessage onto bus
void TaskSensor(void* arg)
{
    uint32_t tick = 0;
    for(;;)
    {
        uint32_t timestamp_ms = tick * 70;
        sensor_ptr->tick(timestamp_ms);  // reads HC-SR04, puts SensorMessage on bus

        /*osMutexAcquire(print_mutex, osWaitForever);
        printf("SENSOR OK tick=%lu\r\n", tick);
        osMutexRelease(print_mutex);*/
        osMutexAcquire(print_mutex, osWaitForever);
        HAL_UART_Transmit(&huart2, (uint8_t*)"SENSOR ALIVE\r\n", 14, HAL_MAX_DELAY);
        osMutexRelease(print_mutex);


        taskENTER_CRITICAL();
        task_checkin |= 0x01;            // set bit 0 (sensor completed this cycle)
        if(task_checkin == 0x07)
        {
            HAL_IWDG_Refresh(&hiwdg);
            task_checkin = 0;
        }
        taskEXIT_CRITICAL();

        tick++;
        osDelay(70);  // yield CPU to other tasks for 70ms
    }
}

// control task: reads SensorMessages, classifies distance, sends ControlMessages
void TaskControl(void* arg)
{
    for(;;)
    {
        control_ptr->tick();

        osMutexAcquire(print_mutex, osWaitForever);
        printf("CONTROL OK\r\n");
        osMutexRelease(print_mutex);

        taskENTER_CRITICAL();
        task_checkin |= 0x02;  // set bit 1 (control this completed cycle)

        // all 3 tasks checked in this cycle so system is healthy
        if(task_checkin == 0x07)  //all 3 bits are set
        {
          HAL_IWDG_Refresh(&hiwdg);  // reset the countdown timer
          task_checkin = 0;           // clear all bits for next cycle
        }
        taskEXIT_CRITICAL();

        osDelay(70);  // yield CPU
    }
}

// actuator task: reads ControlMessages, drives motor and buzzer
void TaskActuator(void* arg)
{
    for(;;)
    {
    	osMutexAcquire(print_mutex, osWaitForever);
    	actuator_ptr->tick();  // log_status() inside here calls printf
    	osMutexRelease(print_mutex);

    	taskENTER_CRITICAL();
    	task_checkin |= 0x04;       // set bit 2 (actuator completed this cycle)
    	if(task_checkin == 0x07)
    	{
    	    HAL_IWDG_Refresh(&hiwdg);
    	    task_checkin = 0;
    	}

        taskEXIT_CRITICAL();
    	osDelay(70);
    }
}
// initializes the IWDG hardware with a ~500ms timeout
void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;                      // point to the IWDG peripheral
   hiwdg.Init.Prescaler = IWDG_PRESCALER_32;   // LSI clock (~32kHz) divided by 32 = 1kHz
    //hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
   hiwdg.Init.Reload = 499;                     // counts down from 499 to 0 = 500ms timeout
   // hiwdg.Init.Reload = 3124;
    HAL_IWDG_Init(&hiwdg);                       // start the watchdog ( it's counting now )
    __HAL_DBGMCU_FREEZE_IWDG();  // pause IWDG countdown when core is halted at a breakpoint
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t reset_cause = RCC->CSR;
	__HAL_RCC_CLEAR_RESET_FLAGS();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_InitTick(TICK_INT_PRIORITY);  // reinitialize TIM6 with correct 180MHz clock
  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t hclk = HAL_RCC_GetHCLKFreq();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //MX_IWDG_Init();  // start watchdog before scheduler so if kernel init hangs we still reset
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); //blink the LED on and off so we know that firmware is responding
  HAL_Delay(200);
 // for(volatile int i = 0; i < 1000000; i++);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);

  HAL_UART_Transmit(&huart2, (uint8_t*)"BOOT\r\n", 6, HAL_MAX_DELAY);
  char reset_msg[64];
  sprintf(reset_msg, "RESET_CAUSE: 0x%08lX\r\n", reset_cause);
  HAL_UART_Transmit(&huart2, (uint8_t*)reset_msg, strlen(reset_msg), HAL_MAX_DELAY);
  //superloop arch
  /*CANBus<SensorMessage>  sensor_bus;
  CANBus<ControlMessage> control_bus;
  // pass real hardware pins into each unit
  SensorUnit  sensor(sensor_bus, TRIG_GPIO_Port, TRIG_Pin, ECHO_GPIO_Port, ECHO_Pin);

  ControlUnit control(sensor_bus, control_bus);

  ActuatorUnit actuator(control_bus, MOTOR_GPIO_Port, MOTOR_Pin, &htim3);

  uint32_t tick = 0; */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //start PWM
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //start it silent until actuator actually needs it

  // construct system objects on heap
  sensor_ptr   = new SensorUnit(sensor_bus, TRIG_GPIO_Port, TRIG_Pin, ECHO_GPIO_Port, ECHO_Pin);
  control_ptr  = new ControlUnit(sensor_bus, control_bus);
  actuator_ptr = new ActuatorUnit(control_bus, MOTOR_GPIO_Port, MOTOR_Pin, &htim3);

  MX_IWDG_Init();  // start watchdog before scheduler so if kernel init hangs we still reset

  // initialize RTOS kernel
  osKernelInitialize();

  // create mutex for printf protection
  print_mutex = osMutexNew(NULL);
  if (print_mutex == NULL) {
       HAL_UART_Transmit(&huart2, (uint8_t*)"MUTEX NULL\r\n", 12, HAL_MAX_DELAY);
   }
  // define task attributes: name, stack size, priority
  const osThreadAttr_t sensor_attr   = { .name="Sensor",   .stack_size=512, .priority=osPriorityNormal }; //all tasks have equal priority
  const osThreadAttr_t control_attr  = { .name="Control",  .stack_size=512, .priority=osPriorityNormal };
  const osThreadAttr_t actuator_attr = { .name="Actuator", .stack_size=1024, .priority=osPriorityNormal };

  // register the three tasks with the scheduler
  osThreadNew(TaskSensor,   NULL, &sensor_attr);
  osThreadNew(TaskControl,  NULL, &control_attr);
  osThreadNew(TaskActuator, NULL, &actuator_attr);

  // hand control to the RTOS scheduler — never returns
  osKernelStart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //superloop arch:
	  /*
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	 uint32_t timestamp_ms = tick * 70;
	 sensor.tick(timestamp_ms);   // reads HC-SR04, puts SensorMessage on bus
	 control.tick();               // classifies distance, puts ControlMessage on bus
	 actuator.tick();              // reads ControlMessage, drives motor + buzzer

	 printf("t=%lu ms | dist=%lu cm\r\n", timestamp_ms, (uint32_t)sensor.last_distance());
	 tick++;
	 HAL_Delay(70);
	 */
	 // osKernelStart() never returns so we never reach here

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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



  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */


  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */

  /* USER CODE BEGIN TIM3_Init 2 */
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 89;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 499;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim3);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_MspPostInit(&htim3);

	/*
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_UART_Transmit(&huart2, (uint8_t*)"BOOT\r\n", 6, HAL_MAX_DELAY);*/

  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MOTOR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Buzzer pin PA6
  /*
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // start HIGH (buzzer OFF)
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  */
  /*GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);
  */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // trigger on both edges
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  // tell the NVIC (interrupt controller) to enable this interrupt
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);  // PA0 = EXTI line 0
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
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
