//
// Created by uoc11 on 2023/12/4.
//

#ifndef CPROJECT_BSP_SPWM_H
#define CPROJECT_BSP_SPWM_H

#include "stm32f4xx_hal.h"
#include "bsp_motor_tim.h"

#define PI          (3.1415926f)
#define PI_X2       (2.0f*PI)

#define  MODULATED_FREQ_Hz    5

#define  CARRIER_FREQ_Hz      (float)(1.68e8/(MOTOR1_PWM_PRESCALER_COUNT+1))/(2*(MOTOR1_PWM_PERIOD_COUNT +1))//

#define  RATIO                (float)(CARRIER_FREQ_Hz/MODULATED_FREQ_Hz)

#define SIN_AMPMAX             255

typedef enum {
    CCW,
    CW,
}MotorDir_Typedef;

extern float  Volt_Freq;
extern int32_t  Accel;
extern int32_t  SinAmp;    /** 输出正弦波得幅值控制变量 */
extern int32_t  HalfMax;  /** 定时器周期一半 */
extern int16_t  Sin1Dir;       /** 正弦波1正负半轴控制 */
extern int16_t  Sin2Dir;       /** 正弦波2正负半轴控制 */
extern int16_t  Sin3Dir;      /** 正弦波3正负半轴控制 */

/** 电机方向控制变量*/
extern __IO uint16_t sin1TableIndex;
extern __IO uint16_t sin2TableIndex;  /** 相位差120°，一个正弦周期512个数据点，512/3=170.6 */
extern __IO uint16_t sin3TableIndex;  /** 相位差120°，一个正弦周期512个数据点，341=（170.6*2）*/
extern  int32_t  SamplePoint;

uint16_t map(uint16_t sinx, uint16_t out_max);
void config_Sinusoidal( float Fre);
void TuneSinAmp( uint16_t Amplitude);
void set_MotorDir(MotorDir_Typedef Dir);
#endif
