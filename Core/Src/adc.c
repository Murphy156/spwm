#include "adc.h"
#include <math.h>
#include "bsp_led.h"
#include "bsp_svpwm.h"

ADC_HandleTypeDef hadc1;

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
static void ADC_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /** 使能 GPIO 时钟 */
    Iab_ADC_GPIO_CLK_ENABLE();
    /** 配置 IO */
    GPIO_InitStructure.Pin = Ia1_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL ; /** 不上拉不下拉 */
    HAL_GPIO_Init(Ia1_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Ib1_ADC_GPIO_PIN;
    HAL_GPIO_Init(Ib1_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Ia2_ADC_GPIO_PIN;
    HAL_GPIO_Init(Ia2_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = Ib2_ADC_GPIO_PIN;
    HAL_GPIO_Init(Ib2_ADC_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  ADC模式初始化
  * @param  无
  * @retval 无
  */
static void ADC_Mode_Config(void)
{
    I_ADC_CLK_ENABLE();

    ADC_InjectionConfTypeDef sConfigInjected;

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
       配置ADC的全局特性,包括时钟,分辨率,数据对齐方向,转换通道数量
    */
    hadc1.Instance                  = I_ADC;
    hadc1.Init.ClockPrescaler       = ADC_CLOCK_SYNC_PCLK_DIV4;     /** 时钟=84/4 = 21MHz */
    hadc1.Init.Resolution           = ADC_RESOLUTION_12B;           /** 12位分辨率(转换时间15个时钟周期)  ADC 可以将输入信号转换为 4096（2^12）个不同的数字值。*/
    hadc1.Init.ScanConvMode         = ENABLE;                       /** 扫描模式 当启用时，ADC 会按照设定的顺序连续扫描多个通道。*/
    hadc1.Init.ContinuousConvMode   = DISABLE;                      /** 连续采样 续转换模式禁用。这意味着 ADC 不会自动重复采样，而是每次需要时进行单次采样。*/
    hadc1.Init.DiscontinuousConvMode= DISABLE;                      /** 不连续采样 不连续转换模式禁用。在不连续模式下，ADC 可以在一组预定的通道之间进行采样，而不是连续采样所有通道。*/
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;/** 无外部触发 这表示没有外部触发用于开始转换。转换将通过软件命令启动。*/
    hadc1.Init.ExternalTrigConv     = ADC_SOFTWARE_START;           /** 软件启动 这个设置指定了转换的启动方式。ADC_SOFTWARE_START 表示转换将通过软件命令启动，而不是外部硬件信号。*/
    hadc1.Init.DataAlign            = ADC_DATAALIGN_LEFT;           /** 左对齐 数据对齐方式为左对齐。这决定了 ADC 转换结果在寄存器中的对齐方式。*/
    hadc1.Init.NbrOfConversion      = 2;                            /** 转换通道 2 这指定了在扫描模式下要转换的通道数量。这里设置为 2，意味着有两个通道将被连续扫描。 */
    hadc1.Init.DMAContinuousRequests= DISABLE;                      /** DMA传输请求 这意味着 DMA（直接内存访问）不会在每次 ADC 转换完成后自动传输数据。*/
    hadc1.Init.EOCSelection         = ADC_EOC_SINGLE_CONV;          /** 单次转换完成标记 这个设置指定了何时标记转换结束（EOC，End Of Conversion）。ADC_EOC_SINGLE_CONV 表示每次单个通道的转换完成后都会标记 EOC。*/
    HAL_ADC_Init(&hadc1);

    /** 配置注入通道的采样顺序和采样时间 */
    sConfigInjected.InjectedChannel               = Ia1_ADC_CHANNEL;                        /** CH9 */
    sConfigInjected.InjectedRank                  = 1;                                      /** 采样顺序 */
    sConfigInjected.InjectedNbrOfConversion       = 2;                                      /** 总的转换通道数量 */
    sConfigInjected.InjectedSamplingTime          = ADC_SAMPLETIME_3CYCLES;                 /** 采样时间,3个周期,单次转换时间是15个周期 */
    sConfigInjected.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;   /** 外部信号上升沿触发(指ADC的外部) */
    sConfigInjected.ExternalTrigInjecConv         = ADC_EXTERNALTRIGINJECCONV_T1_CC4;       /** 触发源:TIM1 CH4的比较事件 */
    sConfigInjected.AutoInjectedConv              = DISABLE;                                /** 自动注入 */
    sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;                                 /** 不连续采样 */
    sConfigInjected.InjectedOffset                = 0;                                      /** 数据偏移值 */
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    sConfigInjected.InjectedChannel               = Ib1_ADC_CHANNEL;                        /** CH14 */
    sConfigInjected.InjectedRank                  = 2;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    /** ADC_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MOTOR_ADC_IRQn , 2, 0);
    HAL_NVIC_EnableIRQ(MOTOR_ADC_IRQn );

    HAL_ADCEx_InjectedStart(&hadc1);

    __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
}

void ADC_Init(void)
{
    ADC_GPIO_Config();
    ADC_Mode_Config();
}

int32_t	ia1_buff[1000];
int32_t	ib1_buff[1000];
int i = 0;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if( hadc->Instance == I_ADC)
    {
        SVPWM_Mode();
    }
}




