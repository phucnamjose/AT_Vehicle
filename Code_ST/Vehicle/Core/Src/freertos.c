/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main_thread.h"
#include "usb_thread.h"
#include "rasberry_thread.h"
#include "arduino_thread.h"
#include "debug_cmd.h"
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
osMailQId mainTaskMailHandle;
/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId usbTaskHandle;
osThreadId rasberryTaskHandle;
osThreadId arduinoTaskHandle;
osMutexId txPiMutexHandle;
osMutexId txMegaMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartMainTask(void const * argument);
void StartUsbTask(void const * argument);
void StartRasTask(void const * argument);
void StartArduinoTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of txPiMutex */
  osMutexDef(txPiMutex);
  txPiMutexHandle = osMutexCreate(osMutex(txPiMutex));

  /* definition and creation of txMegaMutex */
  osMutexDef(txMegaMutex);
  txMegaMutexHandle = osMutexCreate(osMutex(txMegaMutex));

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
  osMailQDef(mainTaskMail, 1, mainTaskMail_t);
  mainTaskMailHandle = osMailCreate(osMailQ(mainTaskMail), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, StartMainTask, osPriorityRealtime, 0, 1024);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, StartUsbTask, osPriorityNormal, 0, 512);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* definition and creation of rasberryTask */
  osThreadDef(rasberryTask, StartRasTask, osPriorityNormal, 0, 512);
  rasberryTaskHandle = osThreadCreate(osThread(rasberryTask), NULL);

  /* definition and creation of arduinoTask */
  osThreadDef(arduinoTask, StartArduinoTask, osPriorityNormal, 0, 512);
  arduinoTaskHandle = osThreadCreate(osThread(arduinoTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartMainTask */
  setupMainThread();
  /* Infinite loop */
  for(;;)
  {
	  osSignalWait(0x01, osWaitForever); // TIM7 generate basic period
	  loopMainThread();
    osDelay(1);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartUsbTask */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbTask */
	setupUsbThread();
  /* Infinite loop */
  for(;;)
  {
	  loopUsbThread();
    osDelay(1);
  }
  /* USER CODE END StartUsbTask */
}

/* USER CODE BEGIN Header_StartRasTask */
/**
* @brief Function implementing the rasberryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRasTask */
void StartRasTask(void const * argument)
{
  /* USER CODE BEGIN StartRasTask */
	setupRasberryThread();
  /* Infinite loop */
  for(;;)
  {
	  loopRasberryThread();
    osDelay(1);
  }
  /* USER CODE END StartRasTask */
}

/* USER CODE BEGIN Header_StartArduinoTask */
/**
* @brief Function implementing the arduinoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArduinoTask */
void StartArduinoTask(void const * argument)
{
  /* USER CODE BEGIN StartArduinoTask */
	setupArduinoThread();
  /* Infinite loop */
  for(;;)
  {
	  loopArduinoThread();
    osDelay(1);
  }
  /* USER CODE END StartArduinoTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
