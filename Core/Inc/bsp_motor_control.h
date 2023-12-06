#ifndef CPROJECT_BSP_MOTOR_CONTROL_H
#define CPROJECT_BSP_MOTOR_CONTROL_H

#include "stm32f4xx.h"
#include "main.h"
#include "bsp_motor_tim.h"
#include "bsp_spwm.h"

#define SPEED_FILTER_NUM      30    /** 速度滤波次数 */

/** 电机状态 */
typedef struct
{
    int32_t timeout;                   /** 定时器更新计数 */
    MotorDir_Typedef direction;        /** 电机方向 */
    float speed;                       /** 电机转速 */
    uint8_t is_enable;                 /** 使能电机 */
    int32_t speed_group[SPEED_FILTER_NUM];
}bldcm_data_t;

extern bldcm_data_t bldcm_data;

void set_bldcm_enable(void);
void set_bldcm_disable(void);
float set_bldcm_speed(float v);

#endif
