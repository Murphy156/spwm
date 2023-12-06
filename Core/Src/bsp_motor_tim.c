/**
  ******************************************************************************
  * @file    bsp_motor_tim.c
  * @author  Leo
  * @version V1.0
  * @date    2023-10-22
  * @brief   电机相关定时器配置
  ******************************************************************************
  */
#include "bsp_motor_tim.h"
#include "bsp_led.h"
#include "bsp_spwm.h"
#include "main.h"

TIM_HandleTypeDef  motor1_htimx_bldcm;
TIM_OC_InitTypeDef MOTOR1_TIM_OCInitStructure;
TIM_ClockConfigTypeDef MOTOR1_ClockSourceConfig;
TIM_MasterConfigTypeDef MOTOR1_MasterConfig;
TIM_BreakDeadTimeConfigTypeDef MOTOR1_BreakDeadTimeConfig;

/** 霍尔传感器相关定时器初始出 */
TIM_HandleTypeDef motor_htimx_hall;

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIM1_GPIO_Config(void)
{
    /** 定义一个GPIO_InitTypeDef类型的结构体 */
    GPIO_InitTypeDef GPIO_InitStructure;

    /** 开启电机1定时器相关的GPIO外设时钟 */
    MOTOR1_OCPWM1_GPIO_CLK_ENABLE();
    MOTOR1_OCNPWM1_GPIO_CLK_ENABLE();
    MOTOR1_OCPWM2_GPIO_CLK_ENABLE();
    MOTOR1_OCNPWM2_GPIO_CLK_ENABLE();
    MOTOR1_OCPWM3_GPIO_CLK_ENABLE();
    MOTOR1_OCNPWM3_GPIO_CLK_ENABLE();
    /** 定时器功能引脚初始化 */
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;

    GPIO_InitStructure.Alternate = MOTOR1_OCNPWM1_AF;

    GPIO_InitStructure.Pin = MOTOR1_OCNPWM1_PIN;
    HAL_GPIO_Init(MOTOR1_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = MOTOR1_OCNPWM2_PIN;
    HAL_GPIO_Init(MOTOR1_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = MOTOR1_OCNPWM3_PIN;
    HAL_GPIO_Init(MOTOR1_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;

    /** 通道 1 */
    GPIO_InitStructure.Pin = MOTOR1_OCPWM1_PIN;
    GPIO_InitStructure.Alternate = MOTOR1_OCPWM1_AF;
    HAL_GPIO_Init(MOTOR1_OCPWM1_GPIO_PORT, &GPIO_InitStructure);

    /** 通道 2 */
    GPIO_InitStructure.Pin = MOTOR1_OCPWM2_PIN;
    GPIO_InitStructure.Alternate = MOTOR1_OCPWM2_AF;
    HAL_GPIO_Init(MOTOR1_OCPWM2_GPIO_PORT, &GPIO_InitStructure);

    /** 通道 3 */
    GPIO_InitStructure.Pin = MOTOR1_OCPWM3_PIN;
    GPIO_InitStructure.Alternate = MOTOR1_OCPWM3_AF;
    HAL_GPIO_Init(MOTOR1_OCPWM3_GPIO_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(MOTOR1_TIM_IRQn,0,0);
    HAL_NVIC_EnableIRQ(MOTOR1_TIM_IRQn);
}

/**
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM1_Mode_Config(void)
{
    /** 开启TIMx_CLK,x[1,8] */
    MOTOR1_TIM_CLK_ENABLE();

    /** 定义定时器的句柄即确定定时器寄存器的基地址 */
    motor1_htimx_bldcm.Instance = MOTOR1_TIM;

    /** 累计 TIM_Period个后产生一个更新或者中断 */
    /** 当定时器从0计数到999，即为1000次，为一个定时周期 */
    motor1_htimx_bldcm.Init.Period = MOTOR1_PWM_PERIOD_COUNT;

    /** 高级控制定时器时钟源TIMxCLK = HCLK=216MHz */
    /** 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1MHz */
    motor1_htimx_bldcm.Init.Prescaler = MOTOR1_PWM_PRESCALER_COUNT;

    /** 采样时钟分频 */
    motor1_htimx_bldcm.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;

    /** 计数方式 */
    motor1_htimx_bldcm.Init.CounterMode=TIM_COUNTERMODE_CENTERALIGNED3;

    /** 重复计数器 */
    motor1_htimx_bldcm.Init.RepetitionCounter = MOTOR1_TIM_REPETITIONCOUNTER;

    /** 初始化定时器TIMx, x[1,8] */
    HAL_TIM_PWM_Init(&motor1_htimx_bldcm);

    MOTOR1_ClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&motor1_htimx_bldcm, &MOTOR1_ClockSourceConfig);

    MOTOR1_MasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    MOTOR1_MasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&motor1_htimx_bldcm, &MOTOR1_MasterConfig);

    MOTOR1_BreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    MOTOR1_BreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    MOTOR1_BreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    MOTOR1_BreakDeadTimeConfig.DeadTime         = 0x00;
    MOTOR1_BreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    MOTOR1_BreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    MOTOR1_BreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&motor1_htimx_bldcm, &MOTOR1_BreakDeadTimeConfig);

    /** PWM模式配置 */
    /** 配置为PWM模式1 */
    MOTOR1_TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    MOTOR1_TIM_OCInitStructure.Pulse = 2100;                         /** 默认必须要初始为0 */
    MOTOR1_TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
    MOTOR1_TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    MOTOR1_TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    MOTOR1_TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&motor1_htimx_bldcm,&MOTOR1_TIM_OCInitStructure,TIM_CHANNEL_1);    /** 初始化通道 1 输出 PWM */
    HAL_TIM_PWM_ConfigChannel(&motor1_htimx_bldcm,&MOTOR1_TIM_OCInitStructure,TIM_CHANNEL_2);    /** 初始化通道 2 输出 PWM */
    HAL_TIM_PWM_ConfigChannel(&motor1_htimx_bldcm,&MOTOR1_TIM_OCInitStructure,TIM_CHANNEL_3);    /** 初始化通道 3 输出 PWM */


    /** 开启定时器通道1输出PWM */
    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&motor1_htimx_bldcm, TIM_CHANNEL_1);

    /** 开启定时器通道2输出PWM */
    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&motor1_htimx_bldcm, TIM_CHANNEL_2);

    /** 开启定时器通道3输出PWM */
    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&motor1_htimx_bldcm, TIM_CHANNEL_3);
}

/**
  * @brief  霍尔传感器引脚初始化
  * @param  无
  * @retval 无
  */
static void hall_motor_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    MOTOR_HALL_INPUTU_GPIO_CLK_ENABLE();
    MOTOR_HALL_INPUTV_GPIO_CLK_ENABLE();
    MOTOR_HALL_INPUTW_GPIO_CLK_ENABLE();

    /* 定时器通道 1 引脚初始化 */
    GPIO_InitStruct.Pin = MOTOR_HALL_INPUTU_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = MOTOR_HALL_INPUTU_AF;
    HAL_GPIO_Init(MOTOR_HALL_INPUTU_GPIO_PORT, &GPIO_InitStruct);

    /* 定时器通道 2 引脚初始化 */
    GPIO_InitStruct.Pin = MOTOR_HALL_INPUTV_PIN;
    HAL_GPIO_Init(MOTOR_HALL_INPUTV_GPIO_PORT, &GPIO_InitStruct);

    /* 定时器通道 3 引脚初始化 */
    GPIO_InitStruct.Pin = MOTOR_HALL_INPUTW_PIN;
    HAL_GPIO_Init(MOTOR_HALL_INPUTW_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  霍尔传感器定时器初始化
  * @param  无
  * @retval 无
  */
static void hall_motor_tim_init(void)
{
    TIM_HallSensor_InitTypeDef  hall_sensor_cfg;

    /** 基本定时器外设时钟使能 */
    MOTOR_HALL_TIM_CLK_ENABLE();

    /** 定时器基本功能配置 */
    motor_htimx_hall.Instance = MOTOR_HALL_TIM;
    motor_htimx_hall.Init.Prescaler = MOTOR_HALL_PRESCALER_COUNT - 1;                /** 预分频 */
    motor_htimx_hall.Init.CounterMode = TIM_COUNTERMODE_UP;                          /** 向上计数 */
    motor_htimx_hall.Init.Period = MOTOR_HALL_PERIOD_COUNT - 1;                      /** 计数周期 */
    motor_htimx_hall.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;                    /** 时钟分频 */

    hall_sensor_cfg.IC1Prescaler = TIM_ICPSC_DIV1;                                   /** 输入捕获分频 */
    hall_sensor_cfg.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;                           /** 输入捕获极性 */
    hall_sensor_cfg.IC1Filter = 10;                                                  /** 输入滤波 */
    hall_sensor_cfg.Commutation_Delay = 0U;                                          /** 不使用延迟触发 */
    HAL_TIMEx_HallSensor_Init(&motor_htimx_hall, &hall_sensor_cfg);

    HAL_NVIC_SetPriority(MOTOR_HALL_TIM_IRQn, 0, 1);    /** 设置中断优先级 */
    HAL_NVIC_EnableIRQ(MOTOR_HALL_TIM_IRQn);                                   /** 使能中断 */
}

/**
  * @brief  初始化霍尔传感器定时器
  * @param  无
  * @retval 无
  */
void hall_motor_tim_config(void)
{
    hall_motor_gpio_init();	    /** 初始化引脚 */
    hall_motor_tim_init();      /** 初始化定时器 */
}

/**
  * @brief  使能霍尔传感器
  * @param  无
  * @retval 无
  */
void hall_motor_enable(void)
{
    /** 使能霍尔传感器接口 */
    __HAL_TIM_ENABLE_IT(&motor_htimx_hall, TIM_IT_TRIGGER);
    __HAL_TIM_ENABLE_IT(&motor_htimx_hall, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Start(&motor_htimx_hall);
    HAL_TIM_TriggerCallback(&motor_htimx_hall);   /** 执行一次换相 */
}

/**
  * @brief  禁用霍尔传感器
  * @param  无
  * @retval 无
  */
void hall_motor_disable(void)
{
    /** 禁用霍尔传感器接口 */
    __HAL_TIM_DISABLE_IT(&motor_htimx_hall, TIM_IT_TRIGGER);
    __HAL_TIM_DISABLE_IT(&motor_htimx_hall, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Stop(&motor_htimx_hall);
}

/**
 * @brief  获取霍尔传感器状态
 * @param  无
 * @retval 无
 * */
uint8_t get_hall_state(void)
{
    uint8_t state = 0;

    /** 读取霍尔传感器 U 的状态 */
    if(HAL_GPIO_ReadPin(MOTOR_HALL_INPUTU_GPIO_PORT, MOTOR_HALL_INPUTU_PIN) != GPIO_PIN_RESET)
    {
        state |= 0x01U << 0;
    }

    /** 读取霍尔传感器 V 的状态 */
    if(HAL_GPIO_ReadPin(MOTOR_HALL_INPUTV_GPIO_PORT, MOTOR_HALL_INPUTV_PIN) != GPIO_PIN_RESET)
    {
        state |= 0x01U << 1;
    }

    /** 读取霍尔传感器 W 的状态 */
    if(HAL_GPIO_ReadPin(MOTOR_HALL_INPUTW_GPIO_PORT, MOTOR_HALL_INPUTW_PIN) != GPIO_PIN_RESET)
    {
        state |= 0x01U << 2;
    }
    return state;    /** 返回传感器状态 */
}


/**
 * @brief  定时器初始化
 * @param  无
 * @retval 无
 * */
void TIMx_Configuration(void)
{
    TIM1_GPIO_Config();
    TIM1_Mode_Config();
    hall_motor_tim_config();
}

/***************************** 测速部分代码 *****************************/
static uint8_t count = 0;



/**
  * @brief  定时器更新中断回调函数
  * @param  htim:定时器句柄
  * @retval 无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&motor1_htimx_bldcm)) {
        int32_t CCR1 = 0;
        int32_t CCR2 = 0;
        int32_t CCR3 = 0;
        CCR1 = map(SinTable[sin1TableIndex & 0xFF], HalfMax);
        CCR1 = HalfMax + (Sin1Dir * CCR1);
        CCR1 = (SinAmp * CCR1) / 24;

        CCR2 = map(SinTable[sin2TableIndex & 0xFF], HalfMax);
        CCR2 = HalfMax + (Sin2Dir * CCR2);
        CCR2 = (SinAmp * CCR2) / 24;

        CCR3 = map(SinTable[sin3TableIndex & 0xFF], HalfMax);
        CCR3 = HalfMax + (Sin3Dir * CCR3);
        CCR3 = (SinAmp * CCR3) / 24;

                __HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, (uint16_t) CCR1);
                __HAL_TIM_SetCompare(htim, TIM_CHANNEL_2, (uint16_t) CCR2);
                __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, (uint16_t) CCR3);

        sin1TableIndex++;
        sin2TableIndex++;
        sin3TableIndex++;

        if (sin1TableIndex >= SamplePoint) Sin1Dir = -1;
        else Sin1Dir = 1;
        if (sin2TableIndex >= SamplePoint) Sin2Dir = -1;
        else Sin2Dir = 1;
        if (sin3TableIndex >= SamplePoint) Sin3Dir = -1;
        else Sin3Dir = 1;

        if (sin1TableIndex >= 2 * SamplePoint)
            sin1TableIndex = 0;
        if (sin2TableIndex >= 2 * SamplePoint)
            sin2TableIndex = 0;
        if (sin3TableIndex >= 2 * SamplePoint)
            sin3TableIndex = 0;
    } else if(htim == (&motor_htimx_hall)){


    }
}

void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* htim_base)
{

    if(htim_base->Instance== MOTOR1_TIM )
    {
        MOTOR1_TIM_RCC_CLK_DISABLE();

    }
}
