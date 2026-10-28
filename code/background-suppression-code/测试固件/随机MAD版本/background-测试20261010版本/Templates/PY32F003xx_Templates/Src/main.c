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
 * 中文注释说明（本次只添加/订正注释，不修改任何可执行语句、参数和调用顺序）
 *
 * 一、完整运行链路
 *   TIM3 CH1在PA6输出PWM；TIM3 CH2比较IRQ通过HAL回调开启TIM1。
 *   TIM1更新TRGO请求ADC0转换；DMA每取得一个结果搬运一个半字。
 *   收满16项后DMA TC中断停TIM1/TIM3，计算相关和平滑、累计判断PA12输出。
 *   处理结束重启TIM3，TIM1等下一次CH2回调再启动，DMA保持循环模式。
 *   初始化阶段TIM1先于ADC启动，所以首次批次不一定由CH2回调建立起点。
 *
 * 二、按本文件24MHz配置计算的名义时序
 *   TIM3: PSC=23、ARR=99，计数1us；CH1 CCR=90，低90us/高10us。
 *   TIM3 CH2: CCR=83，在计数约83us处请求中断，实际启动TIM1还包含IRQ延迟。
 *   TIM1: PSC=0、ARR=2，更新请求间隔0.125us。ADC采样保持时间3.5周期，
 *   还需转换时间，因此8MHz是触发请求频率，不能当成实际ADC完成结果频率。
 *   DMA TC证明传输了16项，不证明16项等间隔、每次触发都被接受或波形位置固定。
 *   DMA计算期间TIM3停止，PWM和采样批次的实际重复间隔包含处理中停顿。
 *
 * 三、当前判断方式
 *   使用固定参考波形去均值相关，不使用COMP_VALUE/COMP_DIFFERENCE幅差门限。
 *   相关值经0.9/0.1平滑及IIR_Filter后，与10000/9800门限比较。
 *   高/低分支各需要10次满足条件才写PA12；回差区间保持输出和累计计数。
 *   PB5决定输出极性；PA5只有初始置高；PA2是采样启动至处理结束的测试标记。
 *
 * 四、注释描述的是现有代码，而不是采集稳定性或量产结论
 *   保留TIM1提前启动、DMA IRQ仅处理TC、IIR 16位接口转换、旧极值索引等行为。
 *   保留串口函数但main中的调用仍注释关闭；所有备用滤波函数均保留。
 *   芯片细节、HAL隐含中断使能以及模拟输入MSP配置需结合完整工程核实。
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
/* TIM3 ARR=100-1；若 TIM3 输入时钟为24MHz，PSC=24-1 后每计数1us，名义周期100us。DMA处理中会停表，因此实际相邻采样批次间隔还包含处理停顿。 */
#define TRANSMIT_PERIOD         100     //= 单位us ，发射管周期
/* TIM3 CH1 的 CCR=90。PWM1且低有效：CNT<90 时PA6为低，CNT>=90 时为高；名义低90us、高10us。发射管何时导通取决于外部驱动电路。 */
#define TRANSMIT_PULSE          90      //= 占空比 ，pwm的CCR比较值
/* TIM3 CH2 的比较位置参数，实际写入 CCR2=84-1=83；在计数约83us处请求中断，经软件延迟后开启TIM1；这是时间位置，不是ADC电压阈值。 */
#define COMP_PULSE              84      //= 阈值  
/* TIM1 ARR=3-1、PSC=0；按24MHz计算，更新间隔3/24MHz=0.125us，即8MHz触发请求。触发请求不等于完成转换，不能据此认定ADC每0.125us产生一点。 */
#define SAMPLE_PERIO            3       //= 定时器1采样周期，f=24mhz不分频率        
/* 每轮DMA传输16个半字，也就是16个ADC转换结果；参考数组也有16项，相关计算按相同索引配对。 */
#define BUFFER_SIZE             16      //= adcbuffer采样点数

//= TIM1触发间隔与ADC完整转换时间需要分开理解，不能仅比较ARR数值。
//= 单纯翻转io 8us 左右
//= ADC实际点间隔需结合采样保持、转换时长和触发接受规则确认。

/* 相关值上判定门限10000；程序比较的是经过两级平滑后的 corr_filter。 */
#define CORR_THRESHOLD   10000
/* 相关值回差200：上门限10000、下门限9800；两门限之间保持输出和两个累计计数。 */
#define CORR_HYSTERESIS  200
/* Private variables ---------------------------------------------------------*/
/* ADC1 的 HAL 管理句柄，包含配置、状态及关联的DMA句柄。 */
ADC_HandleTypeDef   AdcHandle;
/* DMA目标数组：16个12位ADC结果存入16位元素，不是16路ADC，也不是16位ADC精度。 */
uint16_t            adc_buffer[BUFFER_SIZE];
/* 固定参考波形。相关值衡量采样波形去均值后与参考波形的同向程度；参考点与实测点的时间对齐会影响结果。 */
uint16_t            adc_refbuffer[] = {2032,2027,2018,2153,2093,2048,2020,2038,2030,2034,2073,1956,1992,2016,2039,2042};

/* TIM3：CH1输出发射PWM，CH2提供采样启动时刻，两通道共享同一计数器。 */
TIM_HandleTypeDef  Tim3Handle;
/* TIM3通道配置结构，两次配置分别写入CH1/CH2的比较值。 */
TIM_OC_InitTypeDef sConfig;

/* TIM1：输出更新TRGO作为ADC外部触发源；没有配置其GPIO输出。 */
TIM_HandleTypeDef  Tim1Handle;
/* TIM1主模式配置，决定TRGO来自更新事件。 */
TIM_MasterConfigTypeDef sMasterConfig;

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
void TIM1_Init(void);
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
/* 入口顺序：HAL基础初始化 → 24MHz时钟 → GPIO → TIM1 → TIM3 → ADC/DMA → 空循环。注意TIM1/TIM3初始化函数都实际启动了定时器，不只是填写参数。 */
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
  
  /* Initialize TIM1 */
  /* 配置并立即启动TIM1；因此首次ADC启动时，TIM1已可能在连续产生TRGO。 */
  TIM1_Init();
  
  /* Initialize OC1 */ 
  /* 配置并启动TIM3 PWM/比较中断；此时ADC/DMA尚未初始化完成。 */
  TMR3_OC_Init();       
  
  /* Initialize ADC */
  /* 配置单通道ADC和循环DMA，然后使ADC进入外部触发采集状态。 */
  APP_AdcInit();     
  

                    
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
/* PA5指示灯、PA12控制输出、PA2时序测试输出；PB5上拉输入，选择输出极性。 */
void GPIO_Init(void)
{
    /* 使能GPIOA时钟，允许配置和访问其引脚。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* 使能GPIOB时钟。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 一次选择PA5、PA12、PA2，统一设置为推挽输出。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_2;        //= PA5 绿色指示灯 ;PA12 输出控制; PA2 测试用io
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
    /* PA12初始置低，直到检测累计达到条件才写入新输出。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
    /* PA2初始置低，作为示波器观察采样与处理阶段的测试标记。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
}

/**
  * @brief  TMR3 OC Configuration
  * @param  None
  * @retval None
  */
/* TIM3的CH1负责PA6波形，CH2仅提供比较中断；没有在此配置CH2输出引脚。 */
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
    
    /* 启动CH1输出及定时器计数；不使能CH1比较中断。 */
    HAL_TIM_OC_Start(&Tim3Handle,TIM_CHANNEL_1);
    ///=================LEDPWM输出END=============================///
     /* CH2同样使用PWM1配置，但本文件只使用其比较中断，不配置CH2外部输出。 */
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

    /* CH2写CCR2=83，用于在PWM高电平段开始前请求采样启动中断。 */
    sConfig.Pulse = COMP_PULSE-1;                                               /* CC1 value */
    /* 把结构中的输出比较参数应用到指定通道；CH1/CH2分别配置。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
    /* 启动CH2并开启比较中断请求。 */
    HAL_TIM_OC_Start_IT(&Tim3Handle,TIM_CHANNEL_2);     //= 启动通道2，设定比较中断
    ///===============定时触发adcEND===============================///
    /* TIM3设置数值0，DMA设置数值1；通常数值越小优先级越高，具体抢占行为取决于内核/分组。 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  
    /* NVIC允许接收TIM3中断。 */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          
                                                 
    /* 启动时基并开启更新中断；随后直接清UIE关闭更新中断，保留通道比较中断。 */
    HAL_TIM_Base_Start_IT(&Tim3Handle);
    
     /* 清除更新中断使能UIE；定时器本身继续计数，CH2比较中断仍允许。 */
     TIM3->DIER &= ~TIM_DIER_UIE;                       //= 禁用更新中断 
    
}

/* TIM3硬件IRQ入口，交给HAL检查中断标志、清标志并分发回调。 */
void TIM3_IRQHandler(void) {
  /* HAL处理当前定时器中断；比较事件到达后调用下面的OC回调。 */
  HAL_TIM_IRQHandler(&Tim3Handle);
}

/* 输出比较回调仅检查定时器实例，不检查活动通道；当前只开启了CH2比较中断。 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
      
   /* 清TIM1计数器，为本轮触发计数建立起点；这里只清CNT，没有清其他状态标志。 */
   TIM1->CNT = 0;                                           //= 触发通道2比较中断，打开定时器1轮询触发adc
   /* 置CEN开启TIM1计数，随后更新事件经TRGO请求ADC转换；启动不是硬件直连，而是TIM3中断中的软件操作。 */
   TIM1->CR1 |= (TIM_CR1_CEN); 
            
  
    /* PA2拉低标记采样启动；该动作在TIM1启动之后，存在指令和HAL函数延迟。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET) ;    //= 测试io，标记启动点
  }
}

/**
  * @brief  TMR1 Configuration
  * @param  None
  * @retval None
  */
/* TIM1作为ADC触发节拍源，使用TRGO更新事件，不使用TIM1中断。 */
void TIM1_Init(void)
{
    /* 使能TIM1时钟。 */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* 选择TIM1高级定时器。 */
    Tim1Handle.Instance = TIM1;
    /* ARR=2，每3个计数产生一次更新。 */
    Tim1Handle.Init.Period = SAMPLE_PERIO - 1;
    /* PSC=0不分频；按24MHz计数，单计数约41.67ns。 */
    Tim1Handle.Init.Prescaler              =   1 - 1;
    /* CKD不分割，不改变PSC/ARR所决定的更新周期。 */
    Tim1Handle.Init.ClockDivision          =   TIM_CLOCKDIVISION_DIV1;
    /* RCR=0，每次溢出均产生更新事件。 */
    Tim1Handle.Init.RepetitionCounter      =   1 - 1;
    /* 向上计数，0→1→2→0循环。 */
    Tim1Handle.Init.CounterMode            =   TIM_COUNTERMODE_UP;
    /* 使能ARR预装载，参数更新按更新事件装入。 */
    Tim1Handle.Init.AutoReloadPreload      =   TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    /* 配置TIM1时基，失败停在错误处理。 */
    if (HAL_TIM_Base_Init(&Tim1Handle) != HAL_OK)                                /* Initialize timer base */
    {
    APP_ErrorHandler();
    }

    /* TRGO选择更新事件，提供内部触发信号给ADC，无需外接引线。 */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;                   
    /* 不启用定时器主从同步协调模式；仍可输出TRGO，并不等于禁用ADC触发。 */
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    /* 应用TIM1的TRGO设置；原程序没有检查此调用的返回值。 */
    HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig);
        
     /*  Start TIM1 clock */
      /* 此处立即启动TIM1，而非先等TIM3 CH2中断；后续DMA完成IRQ会停TIM1，后面的批次再由TIM3回调启动。 */
      if (HAL_TIM_Base_Start(&Tim1Handle) != HAL_OK)                       
      {
        APP_ErrorHandler();
      }
}
 /**
  * @brief  ADC Configuration
  * @param  None
  * @retval None
  */
/* 单通道ADC0 + TIM1 TRGO上升沿触发 + DMA循环16点；单次转换结束后依赖后续有效触发。 */
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

    //= ADC自身连续转换关闭，由TIM1外部触发重复请求转换
    /* 关闭ADC自身连续转换模式；不能把它解释为一触发后自由连续采样。 */
    AdcHandle.Init.ContinuousConvMode = DISABLE;                           /* Single conversion mode */
    /* 关闭间断扫描模式；这里只配置一个通道。 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                        /* Disable discontinuous mode */

    //= 定时器1更新触发
    /* 转换触发源为TIM1 TRGO，不是软件触发。 */
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;           /* TIM1 TRGO外部触发 */
    /* 允许TRGO上升沿触发；ADC是否接受忙时到来的触发由芯片规定，触发频率不能直接视为结果采样率。 */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;   /* 外部触发上升沿 */

    /* 持续允许ADC转换结果发出DMA请求；这与ContinuousConvMode是两个独立选项。 */
    AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* 允许连续DMA请求 */
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
    /* 内存地址自动递增，依次填入adc_buffer[0..15]。 */
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    /* 外设每次读取半字，即16位。 */
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    /* 内存每次写入半字，与uint16_t数组元素匹配。 */
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    /* 循环模式：16项完成后DMA计数/地址循环重装；本程序通过停TIM1减少处理期间的新转换请求，并未关闭DMA或ADC。 */
    hdma_adc.Init.Mode = DMA_CIRCULAR;
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

     //启用DMA传输完成中断
    /* 显式允许DMA传输完成TC中断；HAL_ADC_Start_DMA内部还可能启用HT/TE，当前自定义IRQ只处理TC，未改动。 */
    __HAL_DMA_ENABLE_IT(&hdma_adc, DMA_IT_TC);  // 传输完成中断使能

     /* 以adc_buffer为目的地址、BUFFER_SIZE=16个传输单位启动ADC/DMA；uint32_t指针转换是HAL接口形式，真实搬运宽度由半字配置决定。 */
     if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)            
      {
        APP_ErrorHandler();
      }

}

 //= 滑动平均
/* 滑动平均窗口10项；该函数当前未被DMA判断流程调用。 */
#define WINDOW_SIZE 10
/* 全局历史窗口和写指针，静态存储默认清零，因此最初平均值包含未填满的零值。 */
uint32_t buffer[WINDOW_SIZE], aindex = 0;
/* 10项滑动平均：覆盖最旧数据、循环推进索引、重新求和后除10。 */
uint32_t adc_filter(uint32_t new_sample) {
    /* 写入本次输入，替换该位置的历史值。 */
    buffer[aindex] = new_sample;
    /* 模运算让写指针在0～9之间循环。 */
    aindex = (aindex + 1) % WINDOW_SIZE;
    uint32_t sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) sum += buffer[i];
    return sum / WINDOW_SIZE;
}

//= 二节滤波
/**
  * @brief  当前输入与两次历史输出的递归加权滤波函数（当前未调用）
  * @param  adc_value: 当前采集的16位ADC原始值
  * @param  b0/b1/b2: 当前输入、上一输出、上上输出的权重，三者之和要求为256
  * @retval 滤波后的16位结果值
  * @note   使用静态变量保存滤波状态，首次调用自动初始化
  */
/* 带两个历史输出的递归加权滤波；不是前面旧注释所写的单alpha一阶滤波，当前未调用。 */
uint16_t adc_lowpass_filter(uint16_t adc_value, uint8_t b0, uint8_t b1, uint8_t b2)
{
    /* 保存上一输出和上上输出，跨函数调用保留。 */
    static uint16_t prev = 0, prev_prev = 0;
    static uint8_t first_call = 1;
    
    // 首次调用初始化
    /* 首次以输入初始化两个历史状态并直接返回，避免从零慢慢爬升。 */
    if(first_call) {
        prev = adc_value;
        prev_prev = adc_value;
         first_call = 0;
        return adc_value;
    }
    
    // 验证系数总和（防御性编程）
    /* 三个权重之和必须为256；分别以uint8_t传入，单个权重不能表达256。无效权重返回上次输出。 */
    if(b0 + b1 + b2 != 256) {
        // 错误处理（可返回0或上次值）
        return prev;
    }

    // 二阶滤波计算（带正确四舍五入）
    /* y[n]=(b0*x[n]+b1*y[n-1]+b2*y[n-2])/256，采用32位乘加。 */
    uint32_t temp = (uint32_t)adc_value * b0 + 
                   (uint32_t)prev * b1 + 
                   (uint32_t)prev_prev * b2;
    /* 右移8位等于除256；对非负和加128是常见四舍五入处理，原注释认为应加64不符合这里的除数。 */
    temp += 128; // 除256前加128，用于非负加权和的四舍五入
    
    // 状态更新
    /* 先把旧的上一输出移到上上输出，再计算并保存新输出。 */
    prev_prev = prev;
    prev = (uint16_t)(temp >> 8);
    
    return prev;
}

/* 一阶低通，近似y+= (input-y)/8，状态从0开始；当前未调用。负数右移行为依赖C实现。 */
uint16_t fast_lowpass(uint16_t input)
{
    static int32_t y = 0;

    y = y + ((input - y) >> 3);   // α≈0.125

    return (uint16_t)y;
}


/* 两级串联一阶低通，每级系数约1/4，y1/y2从0开始；当前未调用。 */
uint16_t fast_filter(uint16_t x)
{
    static int32_t y1=0;
    static int32_t y2=0;

    /* 第一级让y1向输入x靠近四分之一误差。 */
    y1 = y1 + ((x  - y1) >> 2);
    /* 第二级让y2向第一级结果靠近四分之一误差，进一步平滑但增加延迟。 */
    y2 = y2 + ((y1 - y2) >> 2);

    return y2;
}

/* 去均值点积：sum((x[i]-mean_x)*(r[i]-mean_r))。不是归一化相关系数，也不搜索相位；len须非零且不超过两个数组长度。当前传入16。 */
int32_t calc_correlation(uint16_t *x, uint16_t *r, uint8_t len)
{
    /* 相关累计值为带符号32位，可能为正、负或零；没有取绝对值。 */
    int32_t sum = 0;
    int32_t mean_x = 0;
    int32_t mean_r = 0;

    // 计算均值
    for(int i=0;i<len;i++)
    {
        mean_x += x[i];
        mean_r += r[i];
    }

    /* 整数除法得到实测均值，舍去小数；参考均值下方同样计算。 */
    mean_x /= len;
    mean_r /= len;

    // 计算相关
    for(int i=0;i<len;i++)
    {
        /* 实测点减实测均值，去除直流偏置。 */
        int32_t dx = x[i] - mean_x;
        /* 参考点减参考均值，保留参考的波形变化。 */
        int32_t dr = r[i] - mean_r;

        /* 同向偏差乘积为正、反向为负；求和得到波形匹配强度，幅度也会影响结果。 */
        sum += dx * dr;
    }

    return sum;
}

//= 一阶iir
/* 实际调用的一阶IIR：内部32位状态累加，输出约为1/16系数低通。接口输入/返回是int16_t，因此32位相关值传入及结果返回都存在16位范围约束；本次仅注释。 */
int16_t IIR_Filter(int16_t input)
{
    static int32_t filter = 0;

    /* 状态保留约15/16，再加入新输入；返回filter/16。初始状态0会带来启动爬升。 */
    filter = filter - (filter >> 4) + input;

    return filter >> 4;
}

//DMA中断服务函数
/* DMA自定义IRQ是本程序核心：TC→停两定时器→读取PB5→极值/相关→两级平滑→累计判定→重启TIM3→清TC。未调用HAL_DMA_IRQHandler。 */
void DMA1_Channel1_IRQHandler(void)
{
  //HAL_DMA_IRQHandler(&hdma_adc);  // 标准HAL中断处理
  
  /* 仅当通道1TC置位才进入处理；HT半传输和TE错误标志未在本函数处理。TC表示搬运了16项，不能证明点间隔固定或覆盖预期脉冲位置。 */
  if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))         
    {
        /* 先停TIM1，阻止新的更新触发；不会自动中止已经开始的ADC转换，DMA也仍开启，所以不能仅凭停表证明数组绝不会再变化。 */
        TIM1->CR1 &= ~(TIM_CR1_CEN);                //= 先关闭TIM1；下面再关闭TIM3，两个操作按顺序执行
        /* TIM1计数归零，为下一次软件启动准备。 */
        TIM1->CNT = 0;
  
        /* 再停TIM3，暂停发射PWM计数及新的CH2事件；这是顺序执行的两次寄存器操作，不是同时原子停止全部外设。 */
        TIM3->CR1 &= ~(TIM_CR1_CEN); 
        /* TIM3归零；下一批从计数起点重新运行，实际周期包含本次处理中停表时间。 */
        TIM3->CNT = 0;
     
        /* 保存PB5读取后的极性选择：低电平置0，高电平置1；每次完成采样重新读取，没有按键去抖。 */
        static uint16_t NONCchange = 1;             //= 判断常开常闭输出
        
        /* PB5低电平分支：相关高时PA12低、相关低时PA12高；高电平分支反过来。NO/NC名称还取决于后级输出电路。 */
        if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
        {
           NONCchange = 0;
        }
        else
        {
          NONCchange = 1;
        }

       
       /* 本轮最大/最小ADC值，用adc_buffer[0]初始化后扫描。12位非负ADC结果可放入int16_t。极值没有参与当前输出判断。 */
       static int16_t max = 0;                       //= 查找adcbuffer最大最小值
       
       static int16_t min = 0;
       
       /* 极值索引是static，只在严格更大/更小时更新，没有每轮归零；极值位于首项或全相等时索引可能保留旧值。本次不修改。 */
       static uint8_t max_no = 0;
       
       static uint8_t min_no = 0;                    //= 最大最小值对应索引
     
       /* 以本轮首项重新初始化极值，但没有在此重新初始化极值索引。 */
       max = adc_buffer[0];
       
       min = adc_buffer[0];
       
       int i = 0;
       
       /* 遍历16项寻找极值；当前结果仅留在局部静态变量中，供调试观察，不决定输出。 */
       for(i=0;i<BUFFER_SIZE;i++)
       {
           if(adc_buffer[i]>max){max=adc_buffer[i];max_no=i;} 
            if(adc_buffer[i]<min){min=adc_buffer[i];min_no=i;} 
       }
       
       
       /* 本轮原始相关值。 */
       int32_t corr_value = 0;
       /* 跨批次保存平滑结果，初值0；每轮最终还会被下面的IIR输出重新赋值。 */
       static int32_t corr_filter = 0;   
        // ====== 计算相关 ======
        
        /* 将本轮16点与固定16点参考波形逐点计算去均值相关。 */
        corr_value = calc_correlation(adc_buffer, adc_refbuffer, BUFFER_SIZE);
        
       // ====== 简单平滑 ======
        
        /* 第一步平滑：旧corr_filter权重0.9、本轮相关权重0.1；旧值是上一轮经过下面IIR后的结果，并非独立的第一级历史变量。 */
        corr_filter = (corr_filter * 9 + corr_value) / 10;
        
        /* 第二步把第一步结果交给IIR，返回后覆盖corr_filter。传参会从int32_t转换成int16_t，相关值较大时不能按完整32位值理解此链路。 */
        corr_filter = IIR_Filter(corr_filter);

         /* 高相关分支累计计数，持久保存；低于下门限时清零。 */
         static uint16_t valid_count = 0;
         
         /* 低相关分支累计计数；高于上门限时清零。门限区间内两个计数均保持。 */
         static uint16_t valid_count1 = 0;
        
        /* 严格大于10000进入高相关判定；每次先清低相关累计。 */
        if(corr_filter > CORR_THRESHOLD)
        {
            valid_count1 = 0;

            /* 从0开始，前9次只递增，第10次进入else写输出并清计数；不是9次就切换。回差区间不会清此计数。 */
            if(valid_count < 9)
                valid_count++;
            else
            {
                valid_count = 0;

                if(!NONCchange)
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
                else
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
            }
        }
        /* 严格小于9800进入低相关判定，并清高相关累计；9800～10000（含端点）不执行任何判断分支。 */
        else if(corr_filter < CORR_THRESHOLD-CORR_HYSTERESIS)
        {
            valid_count = 0;

            /* 低相关同样第10次满足分支条件才更新输出；中间落入回差区间时累计不会被打断。 */
            if(valid_count1 < 9)
                valid_count1++;
            else
            {
                valid_count1 = 0;

                if(!NONCchange)
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
                else
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
            }
        }
      
           
        /* 重新打开TIM3后再清CNT；TIM1保持停止，直到下一个TIM3 CH2回调开启。原顺序不改。 */
        TIM3->CR1 |= (TIM_CR1_CEN);                 //= 打开定时器3，继续由3触发1
        /* TIM3归零；下一批从计数起点重新运行，实际周期包含本次处理中停表时间。 */
        TIM3->CNT = 0; 
        
        /* PA2拉高标记本轮处理接近结束；低电平宽度包含采集等待、中断响应及算法/输出处理，不是纯ADC采样时间。 */
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);   //= 测试io                            
       /* 最后清除通道1传输完成标志；没有在此清除HT/TE，也没有重新启动DMA，下一轮依赖循环模式。 */
       __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1);
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
