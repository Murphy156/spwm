/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"

extern  float freq;
extern uint8_t SinTable[256];

void SystemClock_Config(void);

/**************************** 任务句柄 ********************************/
/***
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t PROTOCOL_Task_Handle = NULL;
static TaskHandle_t KEY_Task_Handle = NULL;
static TaskHandle_t LED_Task_Handle = NULL;


void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
