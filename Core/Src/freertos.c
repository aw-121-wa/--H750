/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control/chassis/chassis.h"
#include "control/heading/heading_control.h"
#include "devices/imu/hwt101/hwt101.h"
#include "platform/time/dwt_delay.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for IMU_TASK */
osThreadId_t IMU_TASKHandle;
const osThreadAttr_t IMU_TASK_attributes = {
  .name = "IMU_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for START_TASK */
osThreadId_t START_TASKHandle;
const osThreadAttr_t START_TASK_attributes = {
  .name = "START_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MAIN_TASK */
osThreadId_t MAIN_TASKHandle;
const osThreadAttr_t MAIN_TASK_attributes = {
  .name = "MAIN_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HMI_TASK */
osThreadId_t HMI_TASKHandle;
const osThreadAttr_t HMI_TASK_attributes = {
  .name = "HMI_TASK",
  .stack_size = 324 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void IMU_Task(void *argument);
void START_Task(void *argument);
void MAIN_Task(void *argument);
void HMI_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of IMU_TASK */
  IMU_TASKHandle = osThreadNew(IMU_Task, NULL, &IMU_TASK_attributes);

  /* creation of START_TASK */
  START_TASKHandle = osThreadNew(START_Task, NULL, &START_TASK_attributes);

  /* creation of MAIN_TASK */
  MAIN_TASKHandle = osThreadNew(MAIN_Task, NULL, &MAIN_TASK_attributes);

  /* creation of HMI_TASK */
  HMI_TASKHandle = osThreadNew(HMI_Task, NULL, &HMI_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_IMU_Task */
/**
  * @brief  Function implementing the IMU_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Task */
void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  const uint32_t period_ticks = pdMS_TO_TICKS(5U);
  uint32_t next_wake;

  (void)argument;
  next_wake = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    Hwt101_Process();
    next_wake += period_ticks;
    (void)osDelayUntil(next_wake);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_START_Task */
/**
* @brief Function implementing the START_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_START_Task */
void START_Task(void *argument)
{
  /* USER CODE BEGIN START_Task */
  (void)argument;

  if (!Time_Init() ||
      Chassis_Init() != HAL_OK ||
      Heading_Init() != HAL_OK)
  {
    Error_Handler();
  }
  osThreadExit();
  /* USER CODE END START_Task */
}

/* USER CODE BEGIN Header_MAIN_Task */
/**
* @brief Function implementing the MAIN_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MAIN_Task */
void MAIN_Task(void *argument)
{
  /* USER CODE BEGIN MAIN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MAIN_Task */
}

/* USER CODE BEGIN Header_HMI_Task */
/**
* @brief Function implementing the HMI_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HMI_Task */
void HMI_Task(void *argument)
{
  /* USER CODE BEGIN HMI_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HMI_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

