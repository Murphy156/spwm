/**
  ******************************************************************************
  * @file   : main.c
  * @brief  : Main program body
  * question: 如果cmake不见了，在file->setting->cmake 中的右上角reload一下即可
  ******************************************************************************
  */
/** Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "gpio.h"
#include "bsp_DRV8303.h"
#include "bsp_led.h"
#include "bsp_key.h"
#include "bsp_motor_tim.h"
#include "bsp_usart.h"
#include "bsp_spwm.h"
#include "bsp_motor_control.h"
#include "bsp_basic_tim.h"

/** FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"

/**
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);                  /** 用于创建任务 */

static void LED_Task(void* pvParameters);         /** LED_Task任务实现 */
static void KEY_Task(void* pvParameters);         /** KEY_Task任务实现 */
static void PROTOCOL_Task(void* pvParameters);    /** PROTOCOL_Task任务实现 */
static void BSP_Init(void);                       /** 用于初始化板载相关资源 */

/**半周期正弦波，幅值256，频率为1Hz */
uint8_t SinTable[256]={// 0 ~ ¦Ð
        0x00, 0x03, 0x06, 0x09, 0x0C, 0x0F, 0x12, 0x15,
        0x18, 0x1C, 0x1F, 0x22, 0x25, 0x28, 0x2B, 0x2E,
        0x31, 0x34, 0x37, 0x3A, 0x3D, 0x40, 0x44, 0x47,
        0x4A, 0x4D, 0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E,
        0x61, 0x64, 0x67, 0x6A, 0x6D, 0x6F, 0x72, 0x75,
        0x78, 0x7A, 0x7D, 0x80, 0x83, 0x85, 0x88, 0x8B,
        0x8D, 0x90, 0x92, 0x95, 0x97, 0x9A, 0x9C, 0x9F,
        0xA1, 0xA4, 0xA6, 0xA8, 0xAB, 0xAD, 0xAF, 0xB2,
        0xB4, 0xB6, 0xB8, 0xBA, 0xBC, 0xBF, 0xC1, 0xC3,
        0xC5, 0xC7, 0xC9, 0xCA, 0xCC, 0xCE, 0xD0, 0xD2,
        0xD4, 0xD5, 0xD7, 0xD9, 0xDA, 0xDC, 0xDD, 0xDF,
        0xE0, 0xE2, 0xE3, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA,
        0xEB, 0xEC, 0xED, 0xEF, 0xF0, 0xF1, 0xF2, 0xF3,
        0xF4, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF8, 0xF9,
        0xFA, 0xFA, 0xFB, 0xFB, 0xFC, 0xFC, 0xFD, 0xFD,
        0xFD, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
        0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
        0xFD, 0xFD, 0xFD, 0xFC, 0xFC, 0xFB, 0xFB, 0xFA,
        0xFA, 0xF9, 0xF8, 0xF8, 0xF7, 0xF6, 0xF5, 0xF4,
        0xF4, 0xF3, 0xF2, 0xF1, 0xF0, 0xEF, 0xED, 0xEC,
        0xEB, 0xEA, 0xE9, 0xE7, 0xE6, 0xE5, 0xE3, 0xE2,
        0xE0, 0xDF, 0xDD, 0xDC, 0xDA, 0xD9, 0xD7, 0xD5,
        0xD4, 0xD2, 0xD0, 0xCE, 0xCC, 0xCA, 0xC9, 0xC7,
        0xC5, 0xC3, 0xC1, 0xBF, 0xBC, 0xBA, 0xB8, 0xB6,
        0xB4, 0xB2, 0xAF, 0xAD, 0xAB, 0xA8, 0xA6, 0xA4,
        0xA1, 0x9F, 0x9C, 0x9A, 0x97, 0x95, 0x92, 0x90,
        0x8D, 0x8B, 0x88, 0x85, 0x83, 0x80, 0x7D, 0x7A,
        0x78, 0x75, 0x72, 0x6F, 0x6D, 0x6A, 0x67, 0x64,
        0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52, 0x4F, 0x4D,
        0x4A, 0x47, 0x44, 0x40, 0x3D, 0x3A, 0x37, 0x34,
        0x31, 0x2E, 0x2B, 0x28, 0x25, 0x22, 0x1F, 0x1C,
        0x18, 0x15, 0x12, 0x0F, 0x0C, 0x09, 0x06, 0x03,
};
int32_t  SamplePoint = sizeof(SinTable)/sizeof(SinTable[0]); /** 标准正弦波点数 */

float freq = 5;    /** 初始频率 */

float speed = 0;   /** 初始的速度 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    BaseType_t xReturn = pdPASS;

    /** 开发板硬件初始化 */
    BSP_Init();

    /** 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                          (const char*    )"AppTaskCreate",/* 任务名字 */
                          (uint16_t       )512,  /* 任务栈大小 */
                          (void*          )NULL,/* 任务入口函数参数 */
                          (UBaseType_t    )1, /* 任务的优先级 */
                          (TaskHandle_t*  )&AppTaskCreate_Handle);
    /*** 启动任务调度 */
    if (pdPASS == xReturn)
        vTaskStartScheduler(); /*** 启动任务，开启调度 */
    else
        return -1;

    /*** 正常不会执行到这里 */
    while (1){

    };
}

/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
    BaseType_t xReturn = pdPASS; /** 定义一个创建信息返回值，默认为pdPASS */

    taskENTER_CRITICAL(); /** 进入临界区 */
    /** 数字越大，优先级越大*/
    /** 创建LED_Task任务 */
    xReturn = xTaskCreate((TaskFunction_t)  LED_Task  ,/** 任务入口函数 */
                          (const char*   ) "LED_Task" ,/** 任务名字*/
                          (uint16_t      )  512       ,/** 任务栈大小*/
                          (void*         )  NULL      ,/** 任务入口函数参数*/
                          (UBaseType_t   )  4         ,/** 任务的优先级 */
                          (TaskHandle_t* )  &LED_Task_Handle);/** 任务控制块指针*/
    if(pdPASS == xReturn){
        LED3_ON
        vTaskDelay(10);
        LED3_OFF
    }
    /** 创建KEY_Task任务 */
    xReturn =  xTaskCreate((TaskFunction_t ) KEY_Task   ,/** 任务入口函数 */
                           (const char*    ) "KEY_Task" ,/** 任务名字*/
                           (uint16_t       ) 512        ,/** 任务栈大小*/
                           (void*          ) NULL       ,/** 任务入口函数参数*/
                           (UBaseType_t    ) 5          ,/** 任务的优先级*/
                           (TaskHandle_t*  ) &KEY_Task_Handle);   /** 任务控制块指针*/
    if(pdPASS == xReturn)
    {
        LED3_ON
        vTaskDelay(10);
        LED3_OFF
    }
    /** 创建PROTOCOL_Task任务 */
    xReturn = xTaskCreate((TaskFunction_t ) PROTOCOL_Task   , /** 任务入口函数 */
                          (const char*    ) "PROTOCOL_Task" , /** 任务名字*/
                          (uint16_t       ) 1024             , /** 任务栈大小*/
                          (void*          ) NULL            , /** 任务入口函数参数*/
                          (UBaseType_t    ) 3               , /** 任务的优先级*/
                          (TaskHandle_t*  ) &PROTOCOL_Task_Handle);
    if(pdPASS == xReturn)
    {
        LED3_ON
        vTaskDelay(10);
        LED3_OFF
    }

    vTaskDelete(AppTaskCreate_Handle);
    taskEXIT_CRITICAL();
}


/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void LED_Task(void* parameter)
{
    while (1)
    {
        LED1_ON
        vTaskDelay(500);   /** 延时500个tick */
        LED1_OFF
        vTaskDelay(500);   /** 延时500个tick */
    }
}

/**********************************************************************
  * @ 函数名  ： KEY_Task
  * @ 功能说明： KEY_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void KEY_Task(void* parameter)
{
    float set_Freq = 0;
    float max =  60.0f;
    uint8_t motor1_en_flag = 0;
    config_Sinusoidal(0);
    set_MotorDir(CW);

    while (1)
    {
        if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON )
        {/** K1 被按下 */
            if(!motor1_en_flag)
            {
                LED4_ON
                set_Freq = set_bldcm_speed(1800);
                hall_motor_enable();
                set_bldcm_enable();
                while ((freq < set_Freq))
                {
                    HAL_Delay(10);
                    config_Sinusoidal( freq += Accel );
                }
            } else{
                LED4_OFF
                hall_motor_disable();
                set_bldcm_disable();
            }
            motor1_en_flag = !motor1_en_flag;

        }
        if( Key_Scan(KEY2_GPIO_PORT,KEY2_PIN) == KEY_ON )
        {/** K2 被按下 */
            set_bldcm_disable();
        }
        if( Key_Scan(KEY3_GPIO_PORT,KEY3_PIN) == KEY_ON )
        {/** K3 被按下 */
            LED4_ON
            hall_motor_enable();
        }
        if( Key_Scan(KEY4_GPIO_PORT,KEY4_PIN) == KEY_ON )
        {/** K4 被按下 */
        }
        if( Key_Scan(KEY5_GPIO_PORT,KEY5_PIN) == KEY_ON )
        {/** K5 被按下 */

        }
        vTaskDelay(500);/** 延时20个tick */
    }
}

/**********************************************************************
  * @ 函数名  ： PROTOCOL_Task
  * @ 功能说明： PROTOCOL_Task任务主体
  * @ 参数    ：
  * @ 返回值  ： 无
  ********************************************************************/
static void PROTOCOL_Task(void* pvParameters)
{
    while (1)
    {
        receiving_process();
        vTaskDelay(500);
    }
}


/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
    HAL_Init();

    /** 初始化系统时钟为168MHz */
    SystemClock_Config();

    /** 初始化SysTick */
    HAL_SYSTICK_Config( HAL_RCC_GetSysClockFreq() / configTICK_RATE_HZ );

    /** 配置优先级分组为4 */
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    /** LED 灯初始化 */
    LED_GPIO_Config();

    /** 初始化 DRV8303 */
    DRV8303_Init();

    /** 初始化按键G PIO */
    Key_GPIO_Config();

    /** 初始化USART 配置模式为 115200 8-N-1，中断接收 */
    USART_Config();

    /** 初始化电机 */
    TIMx_Configuration();

    /** 基本定时器初始化 */
    Basic_TIMx_Configuration();

    HAL_TIM_Base_Start_IT(&motor1_htimx_bldcm);

}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
    if (HAL_GetREVID() == 0x1001)
    {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }

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
/****************************END OF FILE***************************/

