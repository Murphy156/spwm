#ifndef CPROJECT_BSP_BASIC_TIM_H
#define CPROJECT_BSP_BASIC_TIM_H

#include "stm32f4xx.h"

#define BASIC_TIM           		TIM6
#define BASIC_TIM_CLK_ENABLE()   	__TIM6_CLK_ENABLE()

#define BASIC_TIM_IRQn				TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler    	TIM6_DAC_IRQHandler

/** 累计 TIM_Period个后产生一个更新或者中断*/
/** 当定时器从0计数到BASIC_PERIOD_COUNT-1，即为BASIC_PERIOD_COUNT次，为一个定时周期 */
#define BASIC_PERIOD_COUNT          (50000)

/**定时器时钟源TIMxCLK = 2 * PCLK1
				PCLK1 = HCLK / 4
				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz */
#define BASIC_PRESCALER_COUNT       (1680)

extern TIM_HandleTypeDef TIM_TimeBaseStructure;
void Basic_TIMx_Configuration(void);

#endif //CPROJECT_BSP_BASIC_TIM_H
