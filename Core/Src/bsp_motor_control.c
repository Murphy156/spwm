/**
  ******************************************************************************
  * @file    bsp_motor_control.c
  * @author  Leo
  * @version V1.0
  * @date    2023-12-05
  * @brief
  ******************************************************************************
  */
#include "bsp_motor_control.h"
#include "bsp_led.h"

/** 私有变量 */
bldcm_data_t bldcm_data;

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_bldcm_enable(void)
{
    bldcm_data.is_enable = 1;

    /** 开启定时器通道1输出PWM */
    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&motor1_htimx_bldcm, TIM_CHANNEL_1);

    /** 开启定时器通道2输出PWM */
    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&motor1_htimx_bldcm, TIM_CHANNEL_2);

    /** 开启定时器通道3输出PWM */
    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&motor1_htimx_bldcm, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&motor1_htimx_bldcm,TIM_CHANNEL_4);
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_bldcm_disable(void)
{
    bldcm_data.is_enable = 0;

    /** 开启定时器通道1输出PWM */
    HAL_TIM_PWM_Stop(&motor1_htimx_bldcm,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&motor1_htimx_bldcm, TIM_CHANNEL_1);

    /** 开启定时器通道2输出PWM */
    HAL_TIM_PWM_Stop(&motor1_htimx_bldcm,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&motor1_htimx_bldcm, TIM_CHANNEL_2);

    /** 开启定时器通道3输出PWM */
    HAL_TIM_PWM_Stop(&motor1_htimx_bldcm,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&motor1_htimx_bldcm, TIM_CHANNEL_3);

    HAL_TIM_PWM_Stop(&motor1_htimx_bldcm,TIM_CHANNEL_4);
}

/**
  * @brief  设置电机速度
  * @param  v: 速度（rpm）
  * @retval 无
  */
float set_bldcm_speed(float v)
{
    float freq = v / 30;
    return freq;
}



