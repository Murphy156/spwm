#ifndef CPROJECT_BSP_ADC_H
#define CPROJECT_BSP_ADC_H

#include "stm32f4xx.h"

/** ADC定义 */
/** ADC的GPIO口的宏定义 */
#define I_ADC                       ADC3
#define I_ADC_CLK_ENABLE()          __ADC3_CLK_ENABLE()
#define Iab_ADC_GPIO_CLK_ENABLE()   __GPIOF_CLK_ENABLE()

#define MOTOR_ADC_IRQn              ADC_IRQn
#define MOTOR_ADC_IRQHandler        ADC_IRQHandler


#define Ia1_ADC_GPIO_PORT           GPIOF
#define Ia1_ADC_GPIO_PIN            GPIO_PIN_3
#define Ia1_ADC_GPIO_CLK_ENABLE()   __GPIOF_CLK_ENABLE()
#define Ia1_ADC_CHANNEL             ADC_CHANNEL_9

#define Ib1_ADC_GPIO_PORT           GPIOF
#define Ib1_ADC_GPIO_PIN            GPIO_PIN_4
#define Ib1_ADC_GPIO_CLK_ENABLE()   __GPIOF_CLK_ENABLE()
#define Ib1_ADC_CHANNEL             ADC_CHANNEL_14

#define Ia2_ADC_GPIO_PORT           GPIOF
#define Ia2_ADC_GPIO_PIN            GPIO_PIN_5
#define Ia2_ADC_GPIO_CLK_ENABLE()   __GPIOF_CLK_ENABLE()
#define Ia2_ADC_CHANNEL             ADC_CHANNEL_15

#define Ib2_ADC_GPIO_PORT           GPIOF
#define Ib2_ADC_GPIO_PIN            GPIO_PIN_6
#define Ib2_ADC_GPIO_CLK_ENABLE()   __GPIOF_CLK_ENABLE()
#define Ib2_ADC_CHANNEL             ADC_CHANNEL_4



void ADC_Init(void);


#endif
