/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/*
 * 硬件触发改版：TIM3 CH2 PWM2 OC2REF -> TIM3 TRGO -> ADC外部上升沿 -> DMA。
 * TIM3无比较中断；ADC非连续转换，每个100us周期CH2比较点转换一次。
 * 保留BUFFER_SIZE=1与DMA循环模式；PA12在DMA TC时短脉冲观察频率。
 * CH1 PA6 PWM保持原配置，TIM1不使用。
 * 注意：原算法块已注释，本文不恢复；旧注释中关于中断启动和16点的描述已过时。
 */

/* Includes ------------------------------------------------------------------*/
/* 工程头文件：提供 HAL、引脚/串口宏、错误处理声明等；实际芯片和外设定义以工程头文件为准。 */
#include "main.h"
/* 数学库头文件；当前正文没有调用数学库函数，保留原包含关系。 */
#include <math.h>
/* 提供 uint16_t、int32_t 等固定宽度整数类型。 */
#include <stdint.h>
/* 标准库头文件；本文件当前没有直接使用其中的函数。 */
#include <stdlib.h>
/* CMSIS DSP 接口；本文件的相关计算由自编循环完成，并未调用 DSP 函数。 */
#include "arm_math.h"
/* CMSIS DSP 常量结构头文件；当前正文没有直接使用其常量。 */
#include "arm_const_structs.h"


/* Private define ------------------------------------------------------------*/
/* 修改：删除PA2测试输出，使用PA12正常输出引脚复用为测试引脚。
 * 0：PA12保持原来的NO/NC检测输出。
 * 1：PA12在ADC启动后拉低、DMA处理结束拉高，替代原PA2测试标记；屏蔽正常检测电平写入。
 * 测试低电平宽度包含采样、中断响应和算法处理时间，不是纯ADC转换时间。
 */
#ifndef OUTPUT_TEST_ENABLE
#define OUTPUT_TEST_ENABLE      0
#endif

/* TIM3 PSC=23，按24MHz计时每格1us；ARR=69对应70us；处理DMA时TIM3不停止。 */
#define TRANSMIT_PERIOD         150     //= 单位us ，发射管周期
/* TIM3 CH1 的 CCR=90。PWM1且低有效：CNT<90 时PA6为低，CNT>=90 时为高；名义低90us、高10us。发射管何时导通取决于外部驱动电路。 */
#define TRANSMIT_PULSE          147      //= 占空比 ，pwm的CCR比较值
/* CCR2=84-1=83；保留原比较中断位置，回调直接启动ADC连续转换。 */
#define COMP_PULSE              149      //= 阈值  
/* 每轮由一次TRGO启动连续采集16个ADC点，DMA NORMAL搬运满16点后进入TC中断。 */
#define BUFFER_SIZE             3      //= adcbuffer采样点数

//= 连续模式点间隔由ADC完整转换时间和DMA读取/自动等待决定。
//= 单纯翻转io 8us 左右
//= ADC实际点间隔需结合采样保持、转换时长和触发接受规则确认。

/* 最终判断：索引0和索引2分别相对基准值计算有符号差值，独立判断偏移幅度。
 * index0_diff = 索引0滤波值 - ADC_BASELINE，必须 > 0；
 * index2_diff = 索引2滤波值 - ADC_BASELINE，必须 < 0。
 * 方向满足后，分别比较 abs(index0_diff) 与索引0阈值、abs(index2_diff) 与索引2阈值。
 * 从无效切换为有效：两个绝对偏差都必须 > 各自阈值 + 回差。
 * 从有效切换为无效：任一绝对偏差 < 各自阈值 - 回差。
 * 各自阈值上下限之间为回差区；只有两个索引都满足进入有效条件才会置有效。
 * 任一差值方向不符合要求时立即判定无效，不保留回差状态。
 */
#define ADC_BASELINE                    2040
#define ADC_INDEX0_DIFF_THRESHOLD       30
#define ADC_INDEX2_DIFF_THRESHOLD       30
#define ADC_DIFF_HYSTERESIS             10
/* 新增：最终ADC输出死区（ADC码值）。相对上一次已输出值，变化在±10以内不更新；设为0可关闭死区。 */
#define ADC_DEADBAND           5U
/* ================= ADC各级滤波独立使能开关 =================
 * 1=开启对应滤波；0=跳过，原值直接传递到下一环节。
 * 默认全部开启，与本次修改前处理效果一致。
 * 仅控制数字滤波，不影响ADC/DMA/TIM3和NO/NC配置。
 */
#define ADC_MOVING_AVG_ENABLE  0U   /* 5点滑动平均 */
#define ADC_IIR_ENABLE         1U   /* 一阶IIR */
#define ADC_TRIM_ENABLE        0U   /* 8点截尾均值 */
#define ADC_DEADBAND_ENABLE    1U   /* 最终ADC输出死区 */


#define ADC_IIR_SHIFT          3U   /* 一阶滤波alpha=1/2，较轻；改为2则alpha=1/4 */

/* 索引2仍独立进行滤波；不再单独使用索引2阈值作最终判断。
 * ADC_INDEX2_CHECK_ENABLE 保留为兼容开关：开启时处理索引2滤波，
 * 关闭时索引2直接使用原始采样值参与差值计算。
 */
#define ADC_INDEX2_CHECK_ENABLE       1U

#define ADC_INDEX2_MOVING_AVG_ENABLE  0U
#define ADC_INDEX2_IIR_ENABLE         1U
#define ADC_INDEX2_TRIM_ENABLE        0U
#define ADC_INDEX2_DEADBAND_ENABLE    1U

#define ADC_INDEX2_DEADBAND           5U
#define ADC_INDEX2_IIR_SHIFT          3U


//== 有效确认次数
#define ADC_CONFIRM_COUNT      10U
/* Private variables ---------------------------------------------------------*/
/* ADC1 的 HAL 管理句柄，包含配置、状态及关联的DMA句柄。 */
ADC_HandleTypeDef   AdcHandle;
/* DMA目标数组：16个12位ADC结果存入16位元素，不是16路ADC，也不是16位ADC精度。 */
uint16_t            adc_buffer[BUFFER_SIZE];

/* TIM3：CH1输出发射PWM，CH2提供采样启动时刻，两通道共享同一计数器。 */
TIM_HandleTypeDef  Tim3Handle;
/* TIM3通道配置结构，两次配置分别写入CH1/CH2的比较值。 */
TIM_OC_InitTypeDef sConfig;

/* 修改：TIM3先于ADC初始化启动，初始化完成前禁止其回调访问ADC/DMA。 */
static volatile uint8_t adc_ready = 0;

/* GPIO初始化结构，多次复用；每次HAL_GPIO_Init把当时字段应用到指定端口。 */
GPIO_InitTypeDef  GPIO_InitStruct;

/* DMA1通道1句柄，负责将ADC数据寄存器的结果搬入adc_buffer。 */
DMA_HandleTypeDef hdma_adc;


/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcInit(void);
static void TMR3_OC_Init(void);
static void GPIO_Init(void);
static void APP_AdcStartDMA(void);
void DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);


/* 串口调试初始化函数；main中的调用已被注释，所以当前流程不执行它。原函数及调用状态保持不变。 */
void USART_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

#if  defined(__GNUC__)
  /* GCC条件下关闭stdout缓冲，便于printf即时输出；输出重定向还依赖工程中的实现。 */
  setvbuf(stdout, NULL, _IONBF, 0 );
#endif

  /* 打开宏指定的USART时钟；确切实例、波特率和引脚以main.h中的DEBUG_USART宏为准。 */
  DEBUG_USART_CLK_ENABLE();

  /* 选择调试串口实例。 */
  DebugUartHandle.Instance          = DEBUG_USART;

  /* 设置波特率，由工程宏定义。 */
  DebugUartHandle.Init.BaudRate     = DEBUG_USART_BAUDRATE;
  /* 8位数据位。 */
  DebugUartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  /* 1个停止位。 */
  DebugUartHandle.Init.StopBits     = UART_STOPBITS_1;
  /* 无奇偶校验，合计为常用8N1格式。 */
  DebugUartHandle.Init.Parity       = UART_PARITY_NONE;
  /* 不使用RTS/CTS硬件流控。 */
  DebugUartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  /* 同时允许发送和接收。 */
  DebugUartHandle.Init.Mode         = UART_MODE_TX_RX;
  /* 串口接收采用16倍过采样；与ADC采样无关。 */
  DebugUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  /* 不启用额外串口高级功能。 */
  DebugUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  /* 应用串口配置；此处原代码没有检查返回值。 */
  HAL_UART_Init(&DebugUartHandle);

  /* 打开接收引脚所在GPIO端口时钟。 */
  DEBUG_USART_RX_GPIO_CLK_ENABLE();
  /* 打开发送引脚所在GPIO端口时钟。 */
  DEBUG_USART_TX_GPIO_CLK_ENABLE();

  /**USART1 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
  /* 选择TX引脚，复用推挽、上拉、高速；下方Alternate指定串口复用功能。 */
  GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);

  /* 复用上面的Mode/Pull/Speed字段，改为RX引脚及其复用映射。 */
  GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;

  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct);

  /* ENABLE NVIC */
  /* 配置串口中断优先级并使能NVIC；对应IRQ处理函数不在本文件中。 */
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ,0,1);
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );
}
/**
  * @brief  Main program.
  * @retval int
  */
/* 原入口顺序删除TIM1后：HAL → 时钟 → GPIO → TIM3 → ADC/DMA → 空循环。 */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  /* 初始化HAL运行基础和SysTick；随后时钟配置会通过HAL更新相关时基。 */
  HAL_Init();
  
  /* Configure the system clock */
  /* 选用24MHz HSI，AHB/APB均不分频，作为下文时序计算基础。 */
  APP_SystemClockConfig(); 
    
  /* Initialize USART */
  //USART_Config();         //= 打开这个可以启动串口1的初始化，用printf可以直接打印

  /* Initialize gpio */  
  /* 配置输出初值及PB5常开/常闭选择输入。 */
  GPIO_Init();
  
  /* TIM3在这里完成通道及TRGO配置，但暂不启动计数。 */
  TMR3_OC_Init();

  /* ADC/DMA初始化；随后先使ADC等待硬件触发，再启动TIM3。 */
  APP_AdcInit();
  /* DMA先进入待触发状态，再启动CH2和CH1；杜绝初次比较事件丢失。 */
  APP_AdcStartDMA();
  if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK) APP_ErrorHandler();
  if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK) APP_ErrorHandler();

  /* 主循环不执行采样、滤波或输出判断；这些工作集中在中断里。此文件没有喂狗或主循环任务。 */
  while (1)                 //= 代码纯中断处理
  {
         
  }
}
/**
  * @brief  IO Configuration
  * @param  None
  * @retval None
  */
/* PA5指示灯、PA12正常/测试复用输出；删除PA2测试引脚。PB5选择输出极性。 */
void GPIO_Init(void)
{
    /* 使能GPIOA时钟，允许配置和访问其引脚。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* 使能GPIOB时钟。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 修改：只配置PA5和PA12，PA2不再配置为测试输出。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12;        //= PA5绿色指示灯；PA12正常/测试输出
    /* 推挽输出可主动驱动高低电平。 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /* 启用内部上拉；输出脚实际电平仍由输出寄存器决定。 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    /* 设置引脚输出驱动速度/边沿特性，不会改变定时器或ADC频率。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    /* PB5配置为数字输入；Pull继续设为上拉，悬空时按高电平选择。 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    /* 启用内部上拉；输出脚实际电平仍由输出寄存器决定。 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    /* 设置引脚输出驱动速度/边沿特性，不会改变定时器或ADC频率。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   
    /* PA5初始置高；灯亮灭由外部接法决定，后续检测逻辑未更新PA5。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    /* PA12初始置低；正常模式由判定逻辑控制，测试模式由采样/处理标记控制。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
}

/**
  * @brief  TMR3 OC Configuration
  * @param  None
  * @retval None
  */
/* TIM3 CH1输出PA6 PWM，CH2提供原比较中断；无需CH2输出引脚。 */
static void TMR3_OC_Init(void)
{
    /* 使能TIM3外设时钟。 */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* 使能GPIOA时钟，允许配置和访问其引脚。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 选择PA6作为TIM3 CH1复用输出。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    /* 定时器复用推挽输出，电平由定时器通道控制。 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /* PA6不启用内部上拉或下拉。 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /* 设置引脚输出驱动速度/边沿特性，不会改变定时器或ADC频率。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    /* 选择TIM3复用功能；通道/引脚映射仍以目标芯片为准。 */
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 实际选择TIM3，原英文Select TIM1为复制注释错误。 */
    Tim3Handle.Instance = TIM3;                                                  /* 选择TIM3 */
    /* 自动重装载值99，向上计数0到99后更新回零，共100个计数。 */
    Tim3Handle.Init.Period            = TRANSMIT_PERIOD - 1;                     /* Auto-reload value */
    /* 分频系数PSC+1=24；24MHz输入时计数频率1MHz。 */
    Tim3Handle.Init.Prescaler         = 24 - 1;                                   /* Prescaler */
    /* 设置定时器CKD时钟分割；不是额外把计数时钟除频，计数频率主要由PSC决定。 */
    Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;                  /* No clock division */
    /* 向上计数。 */
    Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;                      /* Up counting */
    /* 填写0；对不支持重复计数器的TIM3不产生额外重复周期。 */
    Tim3Handle.Init.RepetitionCounter = 1 - 1;                                   /* No repetition counting */
    /* 启用ARR预装载；新ARR通常在更新事件生效，原英文not buffered与配置不符。 */
    Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;          /* 使能ARR预装载 */
    /* 应用TIM3时基配置；失败进入APP_ErrorHandler死循环。 */
    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)                                /* Initialize timer base */
    {
    APP_ErrorHandler();
    }
    
    /* PWM1模式，比较关系用于产生PWM；不是toggle翻转模式。 */
    sConfig.OCMode       = TIM_OCMODE_PWM1;                                   /* PWM1模式 */
    /* 低有效极性，CH1在CNT<CCR时输出低、其余时间输出高；原英文active high不符。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_LOW;                                 /* OC低有效极性 */
    /* 启用OC快速模式；不会把定时器计数周期变短。 */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;                                  /* 启用OC快速模式 */
    /* 互补输出极性字段；本程序没有启动互补输出，TIM3的支持情况取决于外设。 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* Active high output level for OCN channel */
    /* 互补输出空闲状态字段，本程序没有启用互补输出。 */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* Output low level in idle state for OC1N */
    /* 输出空闲状态配置字段；实际停表后的PA6电平不应仅凭这一字段认定。 */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* Output low level in idle state for OC1 */

    /* CH1写CCR1=90，与ARR=99共同形成90/10计数分段。 */
    sConfig.Pulse = TRANSMIT_PULSE;                                               /* CC1 value */
    /* 把结构中的输出比较参数应用到指定通道；CH1/CH2分别配置。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
    /* 恢复原版：在TIM3初始化中启动CH1输出及计数。 */
    /* CH1在ADC/DMA就绪后统一启动，避免触发信号早于ADC初始化。 */
    ///=================LEDPWM输出END=============================///
     /* CH2 PWM2的OC2REF上升沿对应CCR2比较时刻。 */
     sConfig.OCMode       = TIM_OCMODE_PWM2;                                   /* CH2 PWM2：比较匹配时OC2REF上升 */
    /* 低有效极性，CH1在CNT<CCR时输出低、其余时间输出高；原英文active high不符。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;                                 /* OC低有效极性 */
    /* 启用OC快速模式；不会把定时器计数周期变短。 */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;                                  /* 启用OC快速模式 */
    /* 互补输出极性字段；本程序没有启动互补输出，TIM3的支持情况取决于外设。 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* Active high output level for OCN channel */
    /* 互补输出空闲状态字段，本程序没有启用互补输出。 */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* Output low level in idle state for OC1N */
    /* 输出空闲状态配置字段；实际停表后的PA6电平不应仅凭这一字段认定。 */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* Output low level in idle state for OC1 */

    /* CH2 CCR2=83，在每周期约83us产生硬件触发。 */
    sConfig.Pulse = COMP_PULSE-1;                                               /* CC1 value */
    /* 把结构中的输出比较参数应用到指定通道；CH1/CH2分别配置。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
     /* CH2使用PWM2：CNT达到CCR2时OC2REF由低变高。 */
     /* OC2REF选作TIM3 TRGO，ADC选择TIM3_TRGO上升沿。无需CH2中断。 */
     TIM_MasterConfigTypeDef master = {0};
     master.MasterOutputTrigger = TIM_TRGO_OC2REF;
     master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
     if (HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &master) != HAL_OK)
     {
       APP_ErrorHandler();
     }

}

/* TIM3硬件IRQ入口，交给HAL检查中断标志、清标志并分发回调。 */
/* TIM3不再开启中断；为兼容工程中可能存在的向量定义，保留空闲入口。 */
//void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&Tim3Handle); }

/* CH2比较事件通过TRGO硬件触发ADC，不再使用TIM3比较中断回调。 */
 /**
  * @brief  ADC Configuration
  * @param  None
  * @retval None
  */
/* 单通道ADC0 + TIM3 TRGO硬件触发ADC连续转换 + DMA分组采样。 */
static void APP_AdcInit(void)
{
    /* 局部ADC通道配置结构清零，与全局TIM通道sConfig是不同对象。 */
    ADC_ChannelConfTypeDef sConfig = {0};
    /* 先将ADC置于复位状态，再释放复位并打开时钟，清除旧外设配置。 */
    __HAL_RCC_ADC_FORCE_RESET();
    /* 释放ADC复位。 */
    __HAL_RCC_ADC_RELEASE_RESET();
    /* 打开ADC外设时钟。 */
    __HAL_RCC_ADC_CLK_ENABLE();

    /* 设置ADC1实例，供校准/初始化函数访问。 */
    AdcHandle.Instance = ADC1;
    /* ADC calibration */
    /* 进行ADC自校准，减少内部偏移；本代码在HAL_ADC_Init之前校准，顺序原样保留。 */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)                 
    {
    APP_ErrorHandler();
    }

    /* 设置ADC1实例，供校准/初始化函数访问。 */
    AdcHandle.Instance = ADC1;

    /* ADC同步PCLK时钟不分频；在本时钟配置下名义ADC时钟为24MHz。 */
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;              /* Set ADC clock */
    /* 12位量化，正常结果范围0～4095。 */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;                        /* 12-bit resolution for converted data */
    /* 数据右对齐，直接作为uint16_t数值读取。 */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                        /* Right-alignment for converted data */
    /* 扫描方向为倒序；只选ADC_CHANNEL_0，因此实际仍是单通道重复采集。 */
    AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;                  /* Single sampling  */
    /* 启用自动等待数据读取；此处通常由DMA读取数据寄存器，具体行为以目标HAL/芯片实现为准。 */
    AdcHandle.Init.LowPowerAutoWait = ENABLE;                              /* Enable wait for conversion mode */

    /* [16点分组] TRGO启动连续转换，DMA one-shot满16点后终止本轮。 */
    AdcHandle.Init.ContinuousConvMode = ENABLE;  /* [16点分组] 一次触发后自动连续采样，直到DMA满16点 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;

    /* ADC启动由TIM3 TRGO上升沿硬件控制。 */
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

    /* [16点分组] ADC DMA请求采用一次模式（非DMA持续请求），配合DMA_NORMAL。 */
    AdcHandle.Init.DMAContinuousRequests = DISABLE;                       /* [16点分组] ADC DMA一次模式：16次搬运后终止本轮转换 */
    /* ADC发生数据覆盖时用新数据覆盖旧数据；本文件没有统计覆盖次数。 */
    AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                     /* Overrun handling: overwrite previous value  */

    /* 采样保持阶段3.5个ADC时钟，按24MHz约0.146us；这是采样时间，不含之后的转换周期，不能直接当完整转换间隔。 */
    AdcHandle.Init.SamplingTimeCommon=ADC_SAMPLETIME_3CYCLES_5;          /* Set sampling time */
    /* Initialize ADC */
    /* 应用ADC配置；失败进入错误处理。 */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)                                
    {
    APP_ErrorHandler();
    }

    /* 仅选择ADC0输入通道；模拟脚配置可能位于工程MSP初始化，本文未展示其具体引脚配置。 */
    sConfig.Channel = ADC_CHANNEL_0;                                       /* ADC channel selection */
    /* 按硬件通道编号参与扫描，不是手动指定第几个采样点。 */
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;                                /* Set the rank for the ADC channel order */ 
    /* Configure ADC channels */  
    /* 将ADC0加入当前转换通道配置。 */
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)            
    {
    APP_ErrorHandler();
    }

    // 配置DMA传输
    /* 按原工程接口把DMA请求映射到ADC；参数0的准确含义以所用HAL定义为准。 */
    HAL_SYSCFG_DMA_Req(0);                                      /* Set DMA1 mapping to ADC */

    /* 打开DMA时钟。 */
    __HAL_RCC_DMA_CLK_ENABLE();
    /* 选择DMA1_Channel1。 */
    hdma_adc.Instance = DMA1_Channel1;              //adc1
    /* 传输方向：ADC外设数据寄存器→RAM。 */
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    /* 外设地址固定，每次都读ADC数据寄存器。 */
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    /* 内存地址自动递增，依次填入adc_buffer[0..BUFFER_SIZE-1]。 */
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    /* 外设每次读取半字，即16位。 */
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    /* 内存每次写入半字，与uint16_t数组元素匹配。 */
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    /* [16点分组] 使用普通模式，禁止DMA循环无限覆盖数据。 */
    hdma_adc.Init.Mode = DMA_NORMAL;   /* [16点分组] 普通DMA，每组16点后完成，ISR重新装载 */
    /* DMA总线仲裁优先级中等；不同于NVIC中断优先级。 */
    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
    /* 应用DMA配置；原程序未检查返回值。 */
    HAL_DMA_Init(&hdma_adc);

    /* 建立ADC与DMA句柄关联，HAL_ADC_Start_DMA使用该DMA完成搬运。 */
    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc);                   /* Link DMA handle to ADC */

     //配置NVIC中断优先级
    /* DMA中断数值优先级1，低于TIM3的0；原配置保持。 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);  // 优先级1，子优先级0
    /* 使能DMA通道1 NVIC中断入口。 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);          // 使能DMA中断

    /* 初始化后只表示ADC配置完成；真正的DMA启动放在主函数。 */
    adc_ready = 1;
}

/* 首次上电调用，以及每轮DMA结束后重新调用：重新准备16点DMA，ADC等待TIM3 TRGO。 */
static void APP_AdcStartDMA(void)
{
    if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)
    {
      APP_ErrorHandler();
    }
    /* HAL默认开启HT，但自定义DMA IRQ只需要TC；每次启动后关闭HT，避免半传输中断反复进入。 */
    __HAL_DMA_DISABLE_IT(&hdma_adc, DMA_IT_HT);
}


/* 新增：5点滑动窗口。第一次直接填充为初值，避免开机补零导致虚假波动。 */
#define WINDOW_SIZE 5U
static uint16_t adc_window[WINDOW_SIZE];
static uint8_t adc_window_index = 0U;
static uint8_t adc_filter_initialized = 0U;
static uint32_t adc_window_sum = 0U;
static int32_t adc_iir_state = 0;

/* 为在线调试预留的观察量，DMA完成后更新。 */
/* [16点分组] 一轮16点的诊断量：每轮DMA TC后更新，可在线观察。 */
volatile uint16_t g_adc_batch_first = 0U;
volatile uint16_t g_adc_batch_last = 0U;
volatile uint16_t g_adc_batch_min = 0U;
volatile uint16_t g_adc_batch_max = 0U;
volatile uint32_t g_adc_batch_count = 0U;
volatile uint32_t g_adc_dma_rearm_error = 0U;
volatile uint16_t g_adc_raw = 0U;
volatile uint16_t g_adc_window = 0U;

volatile uint16_t g_adc_iir = 0U;      /* 新增：5点均值+IIR的结果，便于调试对比 */
volatile uint16_t g_adc_filtered = 0U;  /* 经过死区后的最终ADC值，供阈值判断 */
volatile uint16_t g_adc_filtered_before_deadband = 0U; /* 死区前的三级滤波结果，便于对比观察 */

static uint16_t adc_deadband_output = 0U;
static uint8_t adc_deadband_initialized = 0U;

volatile uint8_t g_adc_signal_state = 0U; /* 1=高于上阈值确认, 0=低于下阈值确认 */
volatile uint8_t g_adc_high_count = 0U;
volatile uint8_t g_adc_low_count = 0U;

/* 截尾窗口大小可调（必须为4的倍数）；自动去掉首尾各1/4，保留中间1/2。 */
#define TRIM_WINDOW_SIZE 8U
#if (TRIM_WINDOW_SIZE < 4U) || ((TRIM_WINDOW_SIZE % 4U) != 0U)
#error "TRIM_WINDOW_SIZE must be >= 4 and divisible by 4"
#endif
#define TRIM_REMOVE_COUNT (TRIM_WINDOW_SIZE / 4U)
#define TRIM_KEEP_COUNT   (TRIM_WINDOW_SIZE / 2U)
static uint16_t adc_trim_window[TRIM_WINDOW_SIZE];
static uint8_t adc_trim_index = 0U;
static uint8_t adc_trim_initialized = 0U;

/* 5点滑动平均 -> IIR -> 截尾均值。每一级都可以通过宏独立跳过。
 * 关闭某一级时，该级的在线观察变量记录透传值，便于比较结果。
 * 编译期选择开关，不引入额外运行时判断开销。
 */
static uint16_t adc_filter(uint16_t sample)
{
    uint16_t average = sample;
    uint16_t iir_filtered;

#if ADC_MOVING_AVG_ENABLE
    /* 第一级：原5点滑动平均，算法和初始化方式不变。 */
    if (!adc_filter_initialized)
    {
        for (uint8_t i = 0U; i < WINDOW_SIZE; i++) adc_window[i] = sample;
        adc_window_sum = (uint32_t)sample * WINDOW_SIZE;
        adc_window_index = 0U;
        adc_filter_initialized = 1U;
    }
    else
    {
        adc_window_sum -= adc_window[adc_window_index];
        adc_window[adc_window_index] = sample;
        adc_window_sum += sample;
        adc_window_index++;
        if (adc_window_index >= WINDOW_SIZE) adc_window_index = 0U;
    }
    average = (uint16_t)(adc_window_sum / WINDOW_SIZE);
#endif
    g_adc_window = average;

#if ADC_IIR_ENABLE
    /* 第二级：保持原IIR整数递推式；首次使用时从本级输入初始化。
     * 这样即使关闭前面的滑动平均，也不会引入启动时的零值拖尾。
     */
    static uint8_t adc_iir_initialized = 0U;
    if (!adc_iir_initialized)
    {
        adc_iir_state = average;
        adc_iir_initialized = 1U;
    }
    else
    {
        adc_iir_state += ((int32_t)average - adc_iir_state) / (1L << ADC_IIR_SHIFT);
    }
    iir_filtered = (uint16_t)adc_iir_state;
#else
    /* IIR关闭：直接使用前一级的输出。 */
    iir_filtered = average;
#endif
    g_adc_iir = iir_filtered;

#if ADC_TRIM_ENABLE
    /* 第三级：原8点截尾均值，丢掉最大1/4和最小1/4。 */
    if (!adc_trim_initialized)
    {
        for (uint8_t i = 0U; i < TRIM_WINDOW_SIZE; ++i)
        {
            adc_trim_window[i] = iir_filtered;
        }
        adc_trim_index = 0U;
        adc_trim_initialized = 1U;
    }
    else
    {
        adc_trim_window[adc_trim_index] = iir_filtered;
        adc_trim_index++;
        if (adc_trim_index >= TRIM_WINDOW_SIZE) adc_trim_index = 0U;
    }

    uint16_t sorted[TRIM_WINDOW_SIZE];
    for (uint8_t i = 0U; i < TRIM_WINDOW_SIZE; ++i)
    {
        sorted[i] = adc_trim_window[i];
    }
    for (uint8_t i = 1U; i < TRIM_WINDOW_SIZE; ++i)
    {
        uint16_t key = sorted[i];
        uint8_t j = i;
        while (j > 0U && sorted[j - 1U] > key)
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = key;
    }

    uint32_t sum = 0U;
    for (uint16_t i = TRIM_REMOVE_COUNT;
         i < (TRIM_WINDOW_SIZE - TRIM_REMOVE_COUNT); ++i)
    {
        sum += sorted[i];
    }
    return (uint16_t)(sum / TRIM_KEEP_COUNT);
#else
    /* 截尾均值关闭：直接进入后续死区/判定。 */
    return iir_filtered;
#endif
}

/* 新增：最终输出死区。比较本次三级滤波结果与上一次已输出值；小幅波动保持原值。
 * 使用有符号32位差值，避免uint16_t减法在下降时发生无符号下溢。
 * 第一次直接采用滤波结果，避免开机死区输出错误地保持为0。
 */
static uint16_t adc_apply_deadband(uint16_t input)
{
    if (!adc_deadband_initialized)
    {
        adc_deadband_output = input;
        adc_deadband_initialized = 1U;
        return adc_deadband_output;
    }

    const int32_t difference = (int32_t)input - (int32_t)adc_deadband_output;
    if ((difference > (int32_t)ADC_DEADBAND) ||
        (difference < -(int32_t)ADC_DEADBAND))
    {
        adc_deadband_output = input;
    }
    return adc_deadband_output;
}

/* [索引2新增] 完全独立的5点平均/IIR/8点截尾/死区滤波状态。
 * 只有索引2判定使能时才进行处理。索引0的各级宏与状态不受影响。
 */
#if ADC_INDEX2_CHECK_ENABLE
static uint16_t adc2_window[WINDOW_SIZE];
static uint32_t adc2_window_sum = 0U;
static uint8_t adc2_window_index = 0U;
static uint8_t adc2_window_initialized = 0U;
static int32_t adc2_iir_state = 0;
static uint8_t adc2_iir_initialized = 0U;
static uint16_t adc2_trim_window[TRIM_WINDOW_SIZE];
static uint8_t adc2_trim_index = 0U;
static uint8_t adc2_trim_initialized = 0U;
static uint16_t adc2_deadband_output = 0U;
static uint8_t adc2_deadband_initialized = 0U;

volatile uint16_t g_adc2_raw = 0U;
volatile uint16_t g_adc2_window = 0U;
volatile uint16_t g_adc2_iir = 0U;
volatile uint16_t g_adc2_filtered_before_deadband = 0U;
volatile uint16_t g_adc2_filtered = 0U;
volatile uint8_t g_adc2_valid = 0U;
#else
/* 关闭判定仍保留原始采样观察量，其余运算由编译器移除。 */
volatile uint16_t g_adc2_raw = 0U;
volatile uint16_t g_adc2_filtered = 0U;
volatile uint8_t g_adc2_valid = 1U;
#endif
/* 基准差值调试量：两个索引滤波值分别减去 ADC_BASELINE，保留正负号；
 * 两个绝对偏差单独提供，分别对应索引0和索引2；g_adc_diff_valid 为最终联合状态。
 */
volatile int32_t g_adc0_baseline_diff = 0;
volatile int32_t g_adc2_baseline_diff = 0;
volatile int32_t g_adc0_abs_baseline_diff = 0;
volatile int32_t g_adc2_abs_baseline_diff = 0;
volatile uint8_t g_adc_diff_valid = 0U;
volatile uint8_t g_adc0_valid = 0U; /* 兼容旧调试变量，不再单独参与最终联合判断 */

#if ADC_INDEX2_CHECK_ENABLE
static uint16_t adc_filter_index2(uint16_t sample)
{
    uint16_t average = sample;
    uint16_t iir_filtered = average;
#if ADC_INDEX2_MOVING_AVG_ENABLE
    if (!adc2_window_initialized)
    {
        for (uint8_t i = 0U; i < WINDOW_SIZE; ++i) adc2_window[i] = sample;
        adc2_window_sum = (uint32_t)sample * WINDOW_SIZE;
        adc2_window_index = 0U;
        adc2_window_initialized = 1U;
    }
    else
    {
        adc2_window_sum -= adc2_window[adc2_window_index];
        adc2_window[adc2_window_index] = sample;
        adc2_window_sum += sample;
        if (++adc2_window_index >= WINDOW_SIZE) adc2_window_index = 0U;
    }
    average = (uint16_t)(adc2_window_sum / WINDOW_SIZE);
#endif
    g_adc2_window = average;
    iir_filtered = average;
#if ADC_INDEX2_IIR_ENABLE
    if (!adc2_iir_initialized)
    {
        adc2_iir_state = average;
        adc2_iir_initialized = 1U;
    }
    else
    {
        adc2_iir_state += ((int32_t)average - adc2_iir_state) / (1L << ADC_INDEX2_IIR_SHIFT);
    }
    iir_filtered = (uint16_t)adc2_iir_state;
#endif
    g_adc2_iir = iir_filtered;
#if ADC_INDEX2_TRIM_ENABLE
    if (!adc2_trim_initialized)
    {
        for (uint8_t i = 0U; i < TRIM_WINDOW_SIZE; ++i) adc2_trim_window[i] = iir_filtered;
        adc2_trim_index = 0U;
        adc2_trim_initialized = 1U;
    }
    else
    {
        adc2_trim_window[adc2_trim_index] = iir_filtered;
        if (++adc2_trim_index >= TRIM_WINDOW_SIZE) adc2_trim_index = 0U;
    }
    uint16_t sorted[TRIM_WINDOW_SIZE];
    for (uint8_t i = 0U; i < TRIM_WINDOW_SIZE; ++i) sorted[i] = adc2_trim_window[i];
    for (uint8_t i = 1U; i < TRIM_WINDOW_SIZE; ++i)
    {
        uint16_t key = sorted[i];
        uint8_t j = i;
        while (j > 0U && sorted[j - 1U] > key)
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = key;
    }
    uint32_t sum = 0U;
    for (uint16_t i = TRIM_REMOVE_COUNT; i < TRIM_WINDOW_SIZE - TRIM_REMOVE_COUNT; ++i)
        sum += sorted[i];
    return (uint16_t)(sum / TRIM_KEEP_COUNT);
#else
    return iir_filtered;
#endif
}

static uint16_t adc_apply_deadband_index2(uint16_t input)
{
#if ADC_INDEX2_DEADBAND_ENABLE
    if (!adc2_deadband_initialized)
    {
        adc2_deadband_output = input;
        adc2_deadband_initialized = 1U;
    }
    else
    {
        const int32_t diff = (int32_t)input - (int32_t)adc2_deadband_output;
        if (diff > (int32_t)ADC_INDEX2_DEADBAND ||
            diff < -(int32_t)ADC_INDEX2_DEADBAND)
            adc2_deadband_output = input;
    }
    return adc2_deadband_output;
#else
    return input;
#endif
}
#endif

//DMA中断服务函数
/* [双索引差值判断] 每轮DMA TC复制并重装，分别处理batch[0]和batch[2]后计算滤波差值。
 * 不停止TIM3；但每轮必须停止/重新启动ADC DMA，以在下次TRGO前重新待命。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))
    {
#if OUTPUT_TEST_ENABLE
        /* 原测试模式：PA12在DMA IRQ处理开始拉低、处理完成拉高。 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
#endif
        /* [16点分组] DMA NORMAL已经完成16次搬运。先把这一组复制到局部数组，
         * 随后停止/复位ADC DMA并立即重新装载，避免后面的排序/滤波阻塞下一轮准备。
         * 此时TIM3继续运行，不停止PWM输出。
         */
        uint16_t batch[BUFFER_SIZE];
        uint16_t min16 = 0xFFFFU;
        uint16_t max16 = 0U;
        for (uint8_t i = 0U; i < BUFFER_SIZE; ++i)
        {
            batch[i] = adc_buffer[i];
        }

        /* 清除本轮传输结束标志，在重新开启DMA前先完成停止。 */
        __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1);
        if (HAL_ADC_Stop_DMA(&AdcHandle) != HAL_OK)
        {
            g_adc_dma_rearm_error++;
            APP_ErrorHandler();
        }
        APP_AdcStartDMA(); /* 新一轮16点：重新等待下一次TIM3 TRGO */

        for (uint8_t i = 0U; i < BUFFER_SIZE; ++i)
        {
            const uint16_t v = batch[i];
            if (v < min16) min16 = v;
            if (v > max16) max16 = v;
        }
        g_adc_batch_first = batch[0];
        g_adc_batch_last  = batch[BUFFER_SIZE - 1U];
        g_adc_batch_min   = min16;
        g_adc_batch_max   = max16;
        g_adc_batch_count++;

        /* [16点分组] 按要求只取本轮第0个采样值进入原四级可选滤波。
         * 保留16点DMA采集和其他15点数据供在线调试，不进行16点平均。
         * 使用batch[0]确保取到本轮已完成且复制保存的第0点。
         */
        const uint16_t sample = batch[0];
        g_adc_raw = sample;

        /* 单次调用完成5点均值、IIR和8点截尾平均。 */
        const uint16_t filtered_before_deadband = adc_filter(sample);
        g_adc_filtered_before_deadband = filtered_before_deadband;
        
        /* 第四级：死区独立开关；关闭后直接把三级滤波结果送入阈值判断。 */
#if ADC_DEADBAND_ENABLE
        const uint16_t filtered = adc_apply_deadband(filtered_before_deadband);
#else
        const uint16_t filtered = filtered_before_deadband;
#endif
        g_adc_filtered = filtered;

        /* 索引2使用本轮第三个ADC转换结果；其滤波链路与索引0相互独立。 */
        g_adc2_raw = batch[2];
#if ADC_INDEX2_CHECK_ENABLE
        const uint16_t filtered2_before = adc_filter_index2(batch[2]);
        g_adc2_filtered_before_deadband = filtered2_before;
        const uint16_t filtered2 = adc_apply_deadband_index2(filtered2_before);
        g_adc2_filtered = filtered2;
#else
        /* 关闭索引2滤波开关时，原始索引2采样值直接参与差值判断。 */
        const uint16_t filtered2 = batch[2];
        g_adc2_filtered = filtered2;
#endif

        /* 最终判断：不再把两个偏差相加，而是分别判断各自相对基准值的偏移量。
         * 索引0必须高于基准，索引2必须低于基准；任一方向不满足即立即无效。
         * 无效状态进入有效状态时，两个绝对偏差都必须大于各自阈值 + 回差。
         * 已有效时，任一绝对偏差低于各自阈值 - 回差，就清除有效状态。
         */
        const int32_t index0_diff = (int32_t)filtered - (int32_t)ADC_BASELINE;
        const int32_t index2_diff = (int32_t)filtered2 - (int32_t)ADC_BASELINE;
        const int32_t index0_abs = (index0_diff >= 0) ? index0_diff : -index0_diff;
        const int32_t index2_abs = (index2_diff >= 0) ? index2_diff : -index2_diff;
        const int32_t index0_on_threshold =
            (int32_t)ADC_INDEX0_DIFF_THRESHOLD + (int32_t)ADC_DIFF_HYSTERESIS;
        const int32_t index2_on_threshold =
            (int32_t)ADC_INDEX2_DIFF_THRESHOLD + (int32_t)ADC_DIFF_HYSTERESIS;
        const int32_t index0_off_threshold =
            (int32_t)ADC_INDEX0_DIFF_THRESHOLD - (int32_t)ADC_DIFF_HYSTERESIS;
        const int32_t index2_off_threshold =
            (int32_t)ADC_INDEX2_DIFF_THRESHOLD - (int32_t)ADC_DIFF_HYSTERESIS;

        g_adc0_baseline_diff = index0_diff;
        g_adc2_baseline_diff = index2_diff;
        g_adc0_abs_baseline_diff = index0_abs;
        g_adc2_abs_baseline_diff = index2_abs;

        if ((index0_diff <= 0) || (index2_diff >= 0))
        {
            /* 索引0必须 > 0、索引2必须 < 0；方向不符立即清除有效状态。 */
            g_adc_diff_valid = 0U;
        }
        else if (g_adc_diff_valid == 0U)
        {
            /* 进入有效状态要求两个索引分别超过各自的阈值 + 回差。 */
            if ((index0_abs > index0_on_threshold) &&
                (index2_abs > index2_on_threshold))
            {
                g_adc_diff_valid = 1U;
            }
        }
        else
        {
            /* 已有效时，任一索引低于各自的阈值 - 回差，则判定无效。 */
            if ((index0_abs < index0_off_threshold) ||
                (index2_abs < index2_off_threshold))
            {
                g_adc_diff_valid = 0U;
            }
        }

        /* 保留旧调试变量，避免外部观察窗口/调试脚本因变量被删除而失效；
         * 这两个变量不再各自按独立阈值判断，也不再用于最终判定。
         */
        g_adc0_valid = g_adc_diff_valid;
        g_adc2_valid = g_adc_diff_valid;

        /* 后续确认次数、NO/NC逻辑、PA12输出极性保持不变。 */
        const uint8_t joint_valid = g_adc_diff_valid;
        const uint8_t NONCchange = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != GPIO_PIN_RESET);
        if (joint_valid)
        {
            g_adc_low_count = 0U;
            if (g_adc_high_count < ADC_CONFIRM_COUNT) g_adc_high_count++;
            if (g_adc_high_count >= ADC_CONFIRM_COUNT)
            {
                g_adc_signal_state = 1U;
#if !OUTPUT_TEST_ENABLE
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
                                  NONCchange ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
            }
        }
        else
        {
            g_adc_high_count = 0U;
            if (g_adc_low_count < ADC_CONFIRM_COUNT) g_adc_low_count++;
            if (g_adc_low_count >= ADC_CONFIRM_COUNT)
            {
                g_adc_signal_state = 0U;
#if !OUTPUT_TEST_ENABLE
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
                                  NONCchange ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
            }
        }
        /* 回差区间内保持状态和计数，和原判定方式相同。 */

#if OUTPUT_TEST_ENABLE
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
#endif
        /* DMA TC已在本轮开始处理时清除，下一组DMA已经重新启动。 */
    }
}

/* ADC/比较器共享向量入口，此处只调用ADC的HAL处理；本文件未显式配置该向量的NVIC使能/优先级。 */
void ADC_COMP_IRQHandler(void)
{
    /* 让HAL处理ADC状态；是否实际进入该IRQ取决于工程中ADC中断和NVIC配置。 */
    HAL_ADC_IRQHandler(&AdcHandle);
  
}

    /**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
/* 系统时钟选HSI 24MHz；不使用外部晶振HSE和低速LSI；HCLK、PCLK1均等于SYSCLK。 */
void APP_SystemClockConfig(void)
{
  /* 振荡器配置结构清零。 */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  /* 总线/系统时钟配置结构清零。 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator Configuration */
  /* 把HSE、HSI、LSI都纳入本次配置，具体开关由下面各State字段决定，不表示全部开启。 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* Select oscillators HSE,HSI,LSI */
  /* 打开内部高速HSI。 */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* Enable HSI */
/* 仅在所用HAL支持HSI分频字段时编译下面的配置。 */
#if defined(RCC_HSIDIV_SUPPORT)
  /* HSI不分频。 */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI not divided */
#endif
  /* 使用HAL定义的24MHz校准档，原英文8MHz为错误注释。 */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* 使用24MHz HSI校准档 */
  /* 关闭外部高速振荡器。 */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* Disable HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  /* 关闭内部低速振荡器；当前文件未配置独立看门狗。 */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* Disable LSI */

  /* Configure oscillators */
  /* 应用振荡器设置，失败进入错误死循环。 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  /* 本次更新SYSCLK、AHB HCLK和APB1 PCLK1配置。 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Select clock types HCLK, SYSCLK, PCLK1 */
  /* HSI作为系统时钟源。 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* Select HSI as the system clock */
  /* AHB不分频，HCLK名义24MHz。 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB clock not divide */
  /* APB1不分频，PCLK1名义24MHz；上面的定时器计算按24MHz外设时钟展开。 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB clock not divided */
  /* Configure clock source */
  /* 应用时钟配置并设置FLASH等待为0；具体允许频率由目标芯片/供电条件规定，保持原配置。 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
/* 初始化出错后的无限循环；没有打印、恢复或主动复位，原程序行为保持。 */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  /* 主循环不执行采样、滤波或输出判断；这些工作集中在中断里。此文件没有喂狗或主循环任务。 */
  while (1)
  {
  }
}

/* 仅在工程启用USE_FULL_ASSERT时编译断言失败处理。 */
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
/* HAL参数断言失败入口，file/line用于定位；当前没有输出定位信息，直接无限循环。 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)  */
  /* Infinite loop */
  /* 主循环不执行采样、滤波或输出判断；这些工作集中在中断里。此文件没有喂狗或主循环任务。 */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
