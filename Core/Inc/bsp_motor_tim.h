#ifndef CPROJECT_BSP_MOTOR_TIM_H
#define CPROJECT_BSP_MOTOR_TIM_H

#include "stm32f4xx.h"
#include "main.h"

/*****************************电机接口1宏定义*******************************************/
#define MOTOR1_TIM                        TIM1
#define MOTOR1_TIM_CLK_ENABLE()           __TIM1_CLK_ENABLE()
#define MOTOR1_TIM_RCC_CLK_DISABLE()      __HAL_RCC_TIM1_CLK_DISABLE()

#define MOTOR1_TIM_IRQn                   TIM1_CC_IRQn
#define MOTOR1_TIM_OC_IRQHANDLER          TIM1_CC_IRQHandler
extern TIM_HandleTypeDef                  motor1_htimx_bldcm;

/** 累计 TIM_Period个后产生一个更新或者中断
	当定时器从0计数到5599，即为5600次，为一个定时周期 */
#define MOTOR1_PWM_PERIOD_COUNT         (4200-1)

/** 高级控制定时器时钟源TIMxCLK = HCLK = 168MHz
	 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 15KHz*/
#define MOTOR1_PWM_PRESCALER_COUNT      (0)

#define MOTOR1_TIM_REPETITIONCOUNTER    1


#define PWM_FREQUENCY                   20000
#define PWM_PERIOD_CYCLES               (uint16_t)(168000000/PWM_FREQUENCY)

/** TIM1 通道1 输出引脚 */
#define MOTOR1_OCPWM1_PIN               GPIO_PIN_8
#define MOTOR1_OCPWM1_GPIO_PORT         GPIOA
#define MOTOR1_OCPWM1_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM1_AF                GPIO_AF1_TIM1

/** TIM1 通道2 输出引脚 */
#define MOTOR1_OCPWM2_PIN               GPIO_PIN_9
#define MOTOR1_OCPWM2_GPIO_PORT         GPIOA
#define MOTOR1_OCPWM2_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM2_AF                GPIO_AF1_TIM1

/** TIM1 通道3 输出引脚 */
#define MOTOR1_OCPWM3_PIN               GPIO_PIN_10
#define MOTOR1_OCPWM3_GPIO_PORT         GPIOA
#define MOTOR1_OCPWM3_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define MOTOR1_OCPWM3_AF			    GPIO_AF1_TIM1

/** TIM1 通道1 互补输出引脚 */
#define MOTOR1_OCNPWM1_PIN               GPIO_PIN_13
#define MOTOR1_OCNPWM1_GPIO_PORT      	 GPIOB
#define MOTOR1_OCNPWM1_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM1_AF			     GPIO_AF1_TIM1

/** TIM1 通道2 互补输出引脚 */
#define MOTOR1_OCNPWM2_PIN            	 GPIO_PIN_14
#define MOTOR1_OCNPWM2_GPIO_PORT      	 GPIOB
#define MOTOR1_OCNPWM2_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM2_AF			     GPIO_AF1_TIM1

/** TIM1 通道3 互补输出引脚 */
#define MOTOR1_OCNPWM3_PIN            	 GPIO_PIN_15
#define MOTOR1_OCNPWM3_GPIO_PORT      	 GPIOB
#define MOTOR1_OCNPWM3_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define MOTOR1_OCNPWM3_AF				 GPIO_AF1_TIM1


/***************************** 霍尔传感器定时器 *****************************/
#define MOTOR_HALL_TIM           		 TIM3
#define MOTOR_HALL_TIM_CLK_ENABLE()  	 __TIM3_CLK_ENABLE()

extern TIM_HandleTypeDef                 motor_htimx_hall;

/** 累计 TIM_Period个后产生一个更新或者中断
	当定时器从0计数到4999，即为5000次，为一个定时周期 */
#define MOTOR_HALL_PERIOD_COUNT          (0xFFFF) /** 65535 */

/** 高级控制定时器时钟源TIMxCLK = HCLK / 2 = 84MHz
	 设定定时器频率为 = TIMxCLK / (PWM_PRESCALER_COUNT)  = 1MHz*/
#define MOTOR_HALL_PRESCALER_COUNT       (84)

/** 无刷电机4对极 */
#define POLE_PAIRES                      2
/** (2*2*3) 脉冲每转 */
#define PPR                              (POLE_PAIRES*2*3)

/** 定时器计数频率 */
#define HALL_TIM_FREQ                    (84e6/MOTOR_HALL_PRESCALER_COUNT)

/** TIM3 通道 1 引脚 */
#define MOTOR_HALL_INPUTU_PIN           		    GPIO_PIN_6
#define MOTOR_HALL_INPUTU_GPIO_PORT     		    GPIOC
#define MOTOR_HALL_INPUTU_GPIO_CLK_ENABLE() 	    __GPIOC_CLK_ENABLE()
#define MOTOR_HALL_INPUTU_AF					    GPIO_AF2_TIM3

/** TIM3 通道 2 引脚 */
#define MOTOR_HALL_INPUTV_PIN           		    GPIO_PIN_7
#define MOTOR_HALL_INPUTV_GPIO_PORT     		    GPIOC
#define MOTOR_HALL_INPUTV_GPIO_CLK_ENABLE() 	    __GPIOC_CLK_ENABLE()
#define MOTOR_HALL_INPUTV_AF					    GPIO_AF2_TIM3

/** TIM3 通道 3 引脚 */
#define MOTOR_HALL_INPUTW_PIN           		    GPIO_PIN_8
#define MOTOR_HALL_INPUTW_GPIO_PORT     		    GPIOC
#define MOTOR_HALL_INPUTW_GPIO_CLK_ENABLE() 	    __GPIOC_CLK_ENABLE()
#define MOTOR_HALL_INPUTW_AF					    GPIO_AF2_TIM3

#define MOTOR_HALL_TIM_IRQn                         TIM3_IRQn
#define MOTOR_HALL_TIM_IRQHandler                   TIM3_IRQHandler

void TIMx_Configuration(void);
void hall_motor_enable(void);
void hall_motor_disable(void);
uint8_t get_hall_state(void);

#endif //CPROJECT_BSP_MOTOR_TIM_H
