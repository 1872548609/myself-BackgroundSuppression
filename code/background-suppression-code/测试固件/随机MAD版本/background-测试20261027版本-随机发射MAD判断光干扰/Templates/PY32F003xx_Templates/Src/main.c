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
 * ======================== 整体设计思路（只添加注释，不改功能） ========================
 * 目标：发射光使用逐周期随机的PWM周期，ADC每个周期固定在末尾只取一个点；
 *       DMA硬件自动将10次转换结果按顺序放入adc_buffer[0..9]，满10点再统一计算。
 *
 * 硬件流水线：
 *  24MHz HSI -> TIM3分频24得到1MHz（计数器每计数一次为1us）
 *  TIM3_CH1(PA6)产生低有效PWM1，控制发射灯（LED是否点亮还依赖驱动级）
 *  TIM3_CH2内部PWM2生成OC2REF，上升沿经TRGO送入ADC外部触发输入
 *  ADC仅配置CHANNEL_0，非连续模式：每个受理的TRGO触发只完成1次转换
 *  每次EOC引发DMA请求，DMA半字搬运到adc_buffer连续地址，CPU无需逐次读取
 *  DMA循环模式装满RANDOM_BATCH_SIZE后产生TC中断，再批量截尾/IIR/阈值/输出。
 *
 * 随机周期：由xorshift产生伪随机索引，每次TIM3更新事件都抽取“下一次”周期。
 * TIM3 ARR和CCR1/CCR2全部使用预装载：更新事件(UEV)使之前排队值一同生效，
 * 更新中断进入后写入的值只是未来某次更新事件才生效，不能认为立即改变当前PWM。
 *
 * 例 period=100us：CNT 0..99，ARR=99，CCR1=97，CCR2=99。
 *   CH1 PWM1/低有效：CNT<97时OC1REF=高，但PA6实际=低；CNT>=97时PA6=高。
 *   CH2 PWM2：CNT<99时OC2REF=低；CNT>=99时OC2REF=高，形成TRGO上升沿。
 *   注意 CH2的OCPolarity配置用于外部通道输出极性，不翻转内部OC2REF；
 *        本程序未将CH2映射到GPIO，ADC看的是内部OC2REF/TRGO而非某个脚的电压。
 *   99us是ADC“启动触发点”，不是转换结果写入DMA的时刻；后者要再加转换延迟。
 *
 * 中断分工：TIM3更新中断只更新周期，DMA TC中断优先级更高且完成10点处理。
 * DMA TC入口首先停止TIM3计数，防止分析时发生下一次硬件触发并覆盖DMA缓冲。
 * 注意：仅停止TIM3不等于全局关中断；SysTick/其他高优先级中断仍可能执行。
 * 另外ADC在周期末前1us才触发：第10次转换可能跨过更新边界，必须在示波器
 * 和调试记录中核对“10个ADC结果”和“10个发射周期”是否严格一一对应。
 * ================================================================================
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

/* ===== 可调参数说明 =====
 * RANDOM_BATCH_SIZE：每批累计多少个ADC结果，DMA TC触发的间隔按“批”计。
 * RANDOM_PERIOD_COUNT：候选周期表元素个数；注意与每批数量没有必须相等的关系。
 * RANDOM_PWM_OFF_US：发射输出在周期末切换电平的提前量（CCR1=period-3）。
 * RANDOM_ADC_BEFORE_END_US：ADC触发比周期终点提前多少us（CCR2=period-1）。
 * 所有周期表值需大于以上两个偏移量，并给ADC完成/中断执行留下充足裕量。
 * 下表30..120us仅是候选集，随机取值允许重复，且固定种子每次上电序列相同。
 * TIM3输入时钟24MHz、PSC=23，所以这里数字的单位为us。
 */
#define RANDOM_BATCH_SIZE         10U  /* 多少个周期采样后统一判断 */
#define RANDOM_PERIOD_COUNT       10U  /* 下方周期数组的成员数，两者必须一致 */
#define RANDOM_PWM_OFF_US          3U  /* CCR1 = 周期 - 3 */
#define RANDOM_ADC_BEFORE_END_US   1U  /* CCR2 = 周期 - 1 */
#define BUFFER_SIZE               RANDOM_BATCH_SIZE
#if RANDOM_BATCH_SIZE < 3U
#error "RANDOM_BATCH_SIZE must be at least 3 for trimmed mean"
#endif

static const uint16_t random_period_table[RANDOM_PERIOD_COUNT] = {
    30U, 39U, 48U, 54U, 59U, 62U, 73U, 77U, 80U, 86U
};
/* 想使用6个候选周期，把 COUNT 改成6，同时表内只保留6个数字。 */
/* RANDOM_BATCH_SIZE 与 PERIOD_COUNT 独立：例如6种候选仍可随机采集10轮。 */
/* 注意：TIM3单个周期内必须留出ADC转换和IRQ裕量。 */

/* ===== ADC绝对阈值 + 迟滞（不再使用固定2040基线） =====
 * 直接使用滤波后的12位ADC码值判断，不再先减去基线。
 * 当前保持原程序的切换边界：
 *   无效 -> 有效：filtered > 2050
 *   有效 -> 无效：filtered <= 2040
 *   2041..2050：保持上一判定状态（迟滞区），避免临界抖动。
 * 如需整体调整灵敏度，修改ADC_VALID_THRESHOLD；
 * ADC_HYSTERESIS为开启和关闭判定值之间的差，须小于阈值。
 * 注意：ADC_CONFIRM_COUNT=3表示连续3个“10点批次”确认，不是3次ADC转换。
 */
#define ADC_VALID_THRESHOLD             2050U
#define ADC_HYSTERESIS                  5U
#if ADC_HYSTERESIS > ADC_VALID_THRESHOLD
#error "ADC_HYSTERESIS must not exceed ADC_VALID_THRESHOLD"
#endif
/* 原第8~10点光干扰检测不适用于随机周期的10个不同采样周期，
 * 本版暂不使用旧干扰判定；批内10点采用去掉最大/最小后的8点均值。
 */

/* ================= ADC各级滤波独立使能开关 =================
 * 1=开启对应滤波；0=跳过，原值直接传递到下一环节。
 * 保持原有各级滤波开关设置不变。
 * 仅控制数字滤波，不影响ADC/DMA/TIM3和NO/NC配置。
 */
#define ADC_MOVING_AVG_ENABLE  0U   /* 5点滑动平均 */
#define ADC_IIR_ENABLE         1U   /* 一阶IIR */
#define ADC_TRIM_ENABLE        0U   /* 8点截尾均值 */
#define ADC_DEADBAND_ENABLE    1U   /* 最终ADC输出死区 */

/* 新增：最终ADC输出死区（ADC码值）。相对上一次已输出值，变化在±5以内不更新；设为0可关闭死区。 */
#define ADC_DEADBAND           3U

#define ADC_IIR_SHIFT          1U   /* 一阶滤波3 alpha=1/8；改为2则alpha=1/4 */

//== 有效确认次数
#define ADC_CONFIRM_COUNT      3U

/* ===== 十点整体波动检测：平均绝对偏差 MAD =====
 * 目的：不用单个最大值/最小值决定是否干扰，改用全部10点的整体离散程度。
 * 计算：avg = sum(adc_buffer[i]) / RANDOM_BATCH_SIZE；
 *       MAD = sum(abs(adc_buffer[i] - avg)) / RANDOM_BATCH_SIZE。
 * MAD单位为ADC码值。数值越大表示本批10点整体波动越明显。
 * 注意：单点异常仍会抬高MAD，只是不再完全由两个极值决定；
 *       真正遮光转换过程中也可能使MAD变大，MAD不能区分所有光干扰。
 * 本版延续原干扰处理策略：MAD超阈值则丢弃本批，不更新后级滤波、
 * 阈值状态和PA12；清零连续有效/无效确认次数。
 * 阈值20仅用于开始上板观察，需按实际正常/遮光/受干扰时MAD分布调整。
 */
#define ADC_MAD_ENABLE              1U
#define ADC_MAD_THRESHOLD          50U
/* Private variables ---------------------------------------------------------*/
/* ADC1 的 HAL 管理句柄，包含配置、状态及关联的DMA句柄。 */
ADC_HandleTypeDef   AdcHandle;
/* ADC DMA目的地址：每个发射周期1个ADC值，十个周期写成adc_buffer[0..9]。
 * 与老版“同一次触发后连续采11点”不同，这里每点属于不同发射周期。
 */
uint16_t            adc_buffer[BUFFER_SIZE];

/* ===== 调试观察变量（可在Keil Watch窗口逐个观察） =====
 * volatile仅表示编译器每次都实际读/写，不表示读取多个变量/数组时具有原子快照性质。
 * 需要完整对照时建议在DMA TC处理完成后暂停调试，避免正在更新数组时读到半批数据。
 * g_random_batch_count：累计完成了多少次“10点DMA批处理”；观察是否持续递增。
 * g_random_last_period：最近一次调用QueuePeriod写入的“待生效”周期，不能当PA6当前周期。
 * g_random_active_period：软件跟踪的当前生效周期；在UEV中断后更新，存在ISR响应延迟。
 * g_random_period_history[]：正在收集的本批“已结束周期”历史，索引=记录次序。
 * g_random_period_history_count：当前历史已写个数；中断时逐步加1，批末补最后一项。
 * g_random_last_batch_periods[]：上一批完成后冻结供观察的10个周期，最值得核对示波器。
 * g_random_last_batch_period_count：上一批记录的实际元素数量，正常应=RANDOM_BATCH_SIZE。
 * g_adc_batch_value：本批ADC去掉1个最大值、1个最小值后的平均结果。
 * g_adc_mad：本批原始10点相对整批均值的平均绝对偏差（ADC码值）。
 * g_adc_interference：1表示MAD超阈值，本批不更新后级滤波和输出判断。
 * g_adc_interference_count：累计发现的干扰批次数，用于统计频率。
 * ！！周期历史基于TIM3更新事件推断，DMA TC可能早于/晚于周期更新事件：
 * ！！不能把history与ADC数据严格对应看作已验证事实，需上板测量确认。
 */
static uint32_t rng_state = 0x6D2B79F5UL; /* xorshift32内部状态，非硬件真随机；上电固定 */
static uint16_t initial_period = 100U;  /* 第一次启动的初始周期 */
volatile uint32_t g_random_batch_count = 0U;
volatile uint16_t g_random_last_period = 0U;
volatile uint16_t g_random_active_period = 0U;
volatile uint16_t g_random_period_history[RANDOM_BATCH_SIZE];
volatile uint8_t g_random_period_history_count = 0U;
volatile uint16_t g_random_last_batch_periods[RANDOM_BATCH_SIZE];
volatile uint8_t g_random_last_batch_period_count = 0U;
static uint16_t queued_period = 100U; /* 内部用：下一次UEV要切换成的周期 */
volatile uint16_t g_adc_batch_value = 0U;
/* 干扰调试观察：每完成一批都刷新（启用MAD时），与确认次数无关。 */
volatile uint16_t g_adc_mad = 0U;              /* 本批10点平均绝对偏差；每批更新 */
volatile uint8_t  g_adc_interference = 0U;     /* 1=本批MAD超过门限；0=本批正常 */
volatile uint32_t g_adc_interference_count = 0U; /* 开机以来检测到的干扰批次总数 */

/* 产生一次伪随机候选值：位移异或更新32位状态，再用取余映射到表格。
 * 该算法计算快、无需stdlib随机数库；但候选值不是不可预测的真随机数，
 * 且%表长度在表长不能整除2^32时会有非常轻微的统计偏差。
 * 每调用一次取一个值，允许连续两次选到同一个周期。
 */
static uint16_t Random_NextPeriod(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return random_period_table[rng_state % RANDOM_PERIOD_COUNT];
}

/* TIM3_QueuePeriod只写预装载值，不立即改动正在输出的周期。
 * ARR=period-1：向上计数0..ARR，共period个1us计数。
 * CCR1=period-3：在本周期末尾3us由PA6低切到高（低有效PWM1）。
 * CCR2=period-1：在周期末尾1us时OC2REF由低变高，TRGO上升沿启动ADC。
 * 依靠ARPE、OC1PE、OC2PE，三者在更新事件(UEV)同步切换；
 * 没有开启CCR预装载时，周期中途更新CCR会产生时序错位。
 */
static void TIM3_QueuePeriod(uint16_t period)
{
    TIM3->ARR = (uint32_t)(period - 1U);
    TIM3->CCR1 = (uint32_t)(period - RANDOM_PWM_OFF_US);
    TIM3->CCR2 = (uint32_t)(period - RANDOM_ADC_BEFORE_END_US);
    queued_period = period;
    g_random_last_period = period;
}

/* TIM3更新中断：UEV发生在计数器溢出回零时，不是ADC触发事件。
 * 执行顺序：1清UIF -> 2记录刚结束的周期 -> 3令原queued成为active
 *          -> 4抽取新的随机值，写ARR/CCR预装载供后续周期使用。
 * 这里ISR执行发生在UEV之后；“预装载写入”和“实际生效”之间有一个UEV边界。
 * g_random_active_period是软件跟踪值，并非从定时器影子寄存器直接读回。
 * 此ISR每周期一次，最短候选30us；需确保它能在下一次更新事件前完成。
 */
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR = (uint32_t)~TIM_SR_UIF;
        /* 在更新事件处，预装载的ARR/CCR已经成为当前生效值。
         * 上一个实际周期刚结束，将它记录到本批历史数组。
         */
        if (g_random_period_history_count < RANDOM_BATCH_SIZE)
            g_random_period_history[g_random_period_history_count++] = g_random_active_period;
        g_random_active_period = queued_period;
        /* 每次更新都独立随机抽取下一周期；允许重复抽中同一个值。 */
        TIM3_QueuePeriod(Random_NextPeriod());
    }
}


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
  /* 先启动ADC+DMA：让DMA准备好接收第一次ADC转换结果。
   * 首次配置首个周期，产生一次软件UG使ARR/CCR预装载生效；
   * UG可能设置UIF，所以立即清掉其标志，避免误当成真正经历了一个发射周期。
   * 把第二个随机周期预先排队，再开启TIM3的NVIC/更新中断。
   * 注意HAL_TIM_OC_Start启动两个通道的先后关系：第一次启动CH2时
   * 计数器可能已开始计时，CH1稍后开启；首周期波形需示波器验证。
   */
  APP_AdcStartDMA();
  
  /* 上电设首个随机周期；UG传递预装载，同时清掉初始化更新标志。 */
  initial_period = Random_NextPeriod();
  TIM3_QueuePeriod(initial_period);
  TIM3->EGR = TIM_EGR_UG;                       //== UG 是 Update Generation（软件生成更新事件）。会产生一次软件更新事件，使刚才写入的预装载值传递到实际工作的寄存器。
  TIM3->SR = (uint32_t)~TIM_SR_UIF;             //== 如果不清除，后面开启更新中断时，就可能立即进入一次TIM3中断。
  g_random_active_period = initial_period;      //== 记录当前实际使用的周期
  TIM3_QueuePeriod(Random_NextPeriod());        /* 第二个周期预先排队 */
  
  /* DIER只允许外设产生请求，还必须在NVIC开启TIM3向量。 */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  TIM3->DIER |= TIM_DIER_UIE;
  
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
/* TIM3两通道共用CNT/PSC/ARR，但有各自CCR和OCREF：
 * CH1：PWM1 + 输出极性LOW，内部OC1REF的有效高电平反转为PA6引脚低电平；
 * CH2：PWM2模式，OC2REF在CNT<CCR2时低、在CNT>=CCR2时高；
 *      TRGO源选TIM_TRGO_OC2REF，用其上升沿启动ADC。
 * CH2不需要GPIO复用引脚输出，也不使用CH2比较中断；
 * TIM3的更新中断仅用于逐周期随机装载下一组ARR/CCR。
 */
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
    /* ARR=109，计数0到109，共110us。 */
    Tim3Handle.Init.Period            = 100U - 1U;                     /* Auto-reload value */
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

    /* CH1写CCR1=107，与ARR=109共同形成107/3计数分段。 */
    sConfig.Pulse = 100U - RANDOM_PWM_OFF_US;                                               /* CC1 value */
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

    /* CH2 CCR2=108，在每周期约108us产生硬件触发。 */
    sConfig.Pulse = 100U - RANDOM_ADC_BEFORE_END_US;                                               /* CC1 value */
    /* 把结构中的输出比较参数应用到指定通道；CH1/CH2分别配置。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
     /* OC1PE/OC2PE打开后，软件写CCR1、CCR2先进入预装载寄存器；
      * UEV时硬件再将其送入实际比较寄存器。ARR同理由ARPE保证。
      * 必须确认HAL配置模式时没有清掉OCxPE，此处显式打开两位。
      */
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

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



/* CH2比较事件通过TRGO硬件触发ADC，不再使用TIM3比较中断回调。 */
 /**
  * @brief  ADC Configuration
  * @param  None
  * @retval None
  */
/* ADC为什么这样配置：
 * 1) CHANNEL_0：只是一路模拟输入，十个数据是“十个不同发射周期”的同一路结果。
 * 2) ContinuousConvMode=DISABLE：禁止某次TRGO后不受控地连续转换；
 *    目标是每个有效TRGO上升沿对应一次完整ADC转换。
 * 3) ExternalTrigConv=T3_TRGO、Edge=RISING：采样启动由TIM3硬件提供，
 *    减少软件中断启动ADC造成的采样点抖动。
 * 4) DMAContinuousRequests=ENABLE：多次转换完成后继续允许DMA请求；
 *    DMA_CIRCULAR搬满10点自动从adc_buffer[0]开始下一圈，CPU不重装DMA。
 * 5) LowPowerAutoWait=ENABLE：允许ADC等数据寄存器被读走后再继续，
 *    由DMA读取DR实现；不同HAL版本的具体行为必须以芯片手册核对。
 * 6) SamplingTimeCommon=3.5周期只代表采样保持时间，不含转换时间；
 *    CCR2在周期末前1us触发，ADC完成及DMA写入往往已经跨到下一周期。
 *    若尚未完成便来到下个TRGO，该触发可能无法受理，必须实测最短30us候选。
 */
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

    /* 硬件外部触发每次仅转换1点，下一次等待下一次TRGO上升沿。 */
    AdcHandle.Init.ContinuousConvMode = DISABLE;  /* [11点分组] 一次触发后自动连续采样，DMA满11点后由软件停止并重装 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;

    /* ADC启动由TIM3 TRGO上升沿硬件控制。 */
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

    /* 允许ADC的DMA请求跨多次单次转换持续工作，与DMA_CIRCULAR配合。 */
    AdcHandle.Init.DMAContinuousRequests = ENABLE;                       /* [11点分组] ADC DMA一次模式：11次搬运后终止本轮转换 */
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
    /* 采用DMA循环模式：10个半字搬满后产生TC并从起始地址重新准备。 */
    hdma_adc.Init.Mode = DMA_CIRCULAR;   /* [11点分组] 普通DMA，每组11点后完成，ISR重新装载 */
    /* DMA总线仲裁优先级中等；不同于NVIC中断优先级。 */
    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
    /* 应用DMA配置；原程序未检查返回值。 */
    HAL_DMA_Init(&hdma_adc);

    /* 建立ADC与DMA句柄关联，HAL_ADC_Start_DMA使用该DMA完成搬运。 */
    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc);                   /* Link DMA handle to ADC */

     //配置NVIC中断优先级
    /* DMA中断数值优先级1，低于TIM3的0；原配置保持。 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);  // 优先级1，子优先级0
    /* 使能DMA通道1 NVIC中断入口。 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);          // 使能DMA中断

    /* 初始化后只表示ADC配置完成；真正的DMA启动放在主函数。 */
    adc_ready = 1;
}

/* 初始化时只调用一次HAL_ADC_Start_DMA设置ADC与DMA：
 * DMA目标基地址=adc_buffer，搬运数=BUFFER_SIZE(默认10)，半字递增；
 * ADC每次转换完成产生一次请求，硬件DMA读ADC DR并依序写到内存。
 * 第10次搬运使DMA TC标志置位；因为CIRCULAR，会自动准备下一圈。
 * 不在每次转换后停止/重启ADC，省掉每点HAL启动/停止开销。
 * 关闭HT中断：不需要第5点半传输事件，只处理第10点TC。
 */
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

/* ===== 滤波/输出调试变量 =====
 * g_adc_raw：本版不是adc_buffer[0]，而是本批10个原始点截尾后的平均值。
 * g_adc_filtered：经过当前启用的IIR、8批截尾滤波与死区后的最终码值。
 * g_adc_threshold_delta：g_adc_filtered - ADC_VALID_THRESHOLD；正值表示超过开启阈值。
 * g_adc_diff_valid：按ADC绝对阈值及迟滞判断的即时有效标志(0/1)。
 * g_adc_signal_state：累计连续ADC_CONFIRM_COUNT“正常批次”后的有效状态(0/1)，
 *                     与PA12电平不同：PA12还取决于PB5的NO/NC极性及测试模式。
 * adc_high_count/adc_low_count：内部连续有效/无效批次计数，不对外开放Watch。
 * adc_deadband_output：最终死区保持的旧码值；干扰批次不更新该值。
 */
volatile uint16_t g_adc_raw = 0U;          /* 索引0原始值 */
volatile uint16_t g_adc_filtered = 0U;     /* 索引0最终滤波值 */
volatile int32_t g_adc_threshold_delta = 0; /* 最终ADC值 - 开启阈值，仅用于Watch */
volatile uint8_t g_adc_diff_valid = 0U;    /* 索引0回差判定结果 */
volatile uint8_t g_adc_signal_state = 0U;  /* 经过确认次数后的输出状态 */
static uint8_t adc_high_count = 0U;
static uint8_t adc_low_count = 0U;
static uint16_t adc_deadband_output = 0U;
static uint8_t adc_deadband_initialized = 0U;

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

/* ====================== DMA TC批处理与输出判断 ======================
 * DMA循环缓冲每填满10点产生TC；注意TC代表“第10次DMA搬运结束”，
 * 不意味着TIM3第10个周期的更新事件一定已发生（ADC在末尾1us才被触发）。
 * 步骤1：先停止TIM3计数并屏蔽TIM3更新中断，避免本批算法期间触发下一次采样；
 *        但不是全局禁止中断。DMA循环模式仍为启用状态，下一次触发前必须处理完成。
 * 步骤2：把10个uint16_t求和，剔除最大和最小各1个后除以8；
 *        这属于“一批内部”的截尾平均，与adc_filter()中的“8个批次历史截尾”不同。
 * 步骤3：计算10个原始ADC数据的MAD（全部10点参与），检测整组离散程度；
 *        MAD超过阈值则冻结后级滤波、阈值和输出，清零确认计数。
 *        正常批次才更新IIR、8批截尾与死区，然后比较ADC绝对阈值及迟滞。
 * 步骤4：有效/无效连续计数达ADC_CONFIRM_COUNT批，才刷新PA12；PB5决定高低极性。
 * 步骤5：将批次周期历史复制到稳定Watch数组，复位TIM3并重新装载首个/下一周期。
 * 注意：目前批末通过软件UG重启，可能产生批次间额外时间间隔/不完整周期；
 *       代码未锁住所有高优先级中断，也不能无条件保证ADC/DMA与period_history对齐。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        /* 优先停止周期计数与随机周期更新；DMA TC对应10次搬运完成。
         * ADC若刚好还在转换、其他中断若优先级更高，仍要按硬件时序验证。
         * 这一步目的在于尽早停止TRGO，给后续较长的滤波计算腾出稳定窗口。
         */
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->DIER &= ~TIM_DIER_UIE;
        DMA1->IFCR = DMA_IFCR_CGIF1;

#if OUTPUT_TEST_ENABLE
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
#endif
        uint32_t sum = 0U;
        uint16_t minval = 0xFFFFU, maxval = 0U;
        for (uint8_t i = 0U; i < RANDOM_BATCH_SIZE; ++i)
        {
            uint16_t v = adc_buffer[i];
            sum += v;
            if (v < minval) minval = v;
            if (v > maxval) maxval = v;
        }
        const uint16_t sample = (uint16_t)((sum - minval - maxval) / (RANDOM_BATCH_SIZE - 2U));
        g_adc_batch_value = sample;
        g_adc_raw = sample;
        g_random_batch_count++;

        /* 用原始10点计算整组平均绝对偏差MAD，不使用极差判断。
         * 第一遍求和已由上面的截尾平均循环完成，因此只需额外遍历一遍。
         * 使用有符号32位差值，避免低于均值时无符号减法下溢。
         * 原10点去最大/最小后的8点平均保持不变，供后级滤波使用。
         */
#if ADC_MAD_ENABLE
        const uint16_t mad_average = (uint16_t)(sum / RANDOM_BATCH_SIZE);
        uint32_t mad_deviation_sum = 0U;
        for (uint8_t i = 0U; i < RANDOM_BATCH_SIZE; ++i)
        {
            int32_t difference = (int32_t)adc_buffer[i] - (int32_t)mad_average;
            if (difference < 0) difference = -difference;
            mad_deviation_sum += (uint32_t)difference;
        }
        g_adc_mad = (uint16_t)(mad_deviation_sum / RANDOM_BATCH_SIZE);
        g_adc_interference = (g_adc_mad > ADC_MAD_THRESHOLD) ? 1U : 0U;
#else
        g_adc_mad = 0U;
        g_adc_interference = 0U;
#endif
        if (g_adc_interference)
        {
            g_adc_interference_count++;
            /* 该批不参与连续3批确认，也不能破坏现有输出/滤波历史。
             * g_adc_filtered、g_adc_threshold_delta、g_adc_diff_valid保持上次稳定值。
             * g_adc_batch_value和g_adc_raw仍显示本批实际均值，便于观察。
             */
            adc_high_count = 0U;
            adc_low_count = 0U;
        }
        else
        {
            const uint16_t filtered_before_deadband = adc_filter(sample);
    #if ADC_DEADBAND_ENABLE
            const uint16_t filtered = adc_apply_deadband(filtered_before_deadband);
    #else
            const uint16_t filtered = filtered_before_deadband;
    #endif
            g_adc_filtered = filtered;
            /* 直接使用最终ADC码值进行阈值和迟滞判断，不减去任何基线。
             * 初始无效状态：filtered > ADC_VALID_THRESHOLD 才变为有效；
             * 已经有效时：filtered <= ADC_VALID_THRESHOLD-ADC_HYSTERESIS 才变回无效。
             * 位于两阈值之间保持先前状态，避免在阈值附近频繁切换。
             */
            g_adc_threshold_delta = (int32_t)filtered - (int32_t)ADC_VALID_THRESHOLD;
            if (!g_adc_diff_valid)
            {
                if (filtered > ADC_VALID_THRESHOLD) g_adc_diff_valid = 1U;
            }
            else
            {
                if (filtered <= (ADC_VALID_THRESHOLD - ADC_HYSTERESIS))
                    g_adc_diff_valid = 0U;
            }

            const uint8_t NONCchange =
                (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) != GPIO_PIN_RESET);
            
            if (g_adc_diff_valid)
            {
                adc_low_count = 0U;
                if (adc_high_count < ADC_CONFIRM_COUNT) adc_high_count++;
                if (adc_high_count >= ADC_CONFIRM_COUNT)
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
                adc_high_count = 0U;
                if (adc_low_count < ADC_CONFIRM_COUNT) adc_low_count++;
                if (adc_low_count >= ADC_CONFIRM_COUNT)
                {
                    g_adc_signal_state = 0U;
    #if !OUTPUT_TEST_ENABLE
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,
                                      NONCchange ? GPIO_PIN_RESET : GPIO_PIN_SET);
    #endif
                }
            }
        }

        /* 定时器仍停止，重新从随机周期的起点开始下一批。DMA CIRCULAR
         * 已自动重装计数器，不重新调用HAL_ADC_Stop/Start_DMA。
         */
        /* history主要在TIM3 UEV ISR中记录“已结束周期”，但第10次ADC TC可能
         * 发生于当前发射周期更新之前，因此代码在批次末尝试补记当前周期。
         * 如果此时已经发生UEV、或存在中断延后，不能仅凭此数组证明采样严格对齐。
         */
        if (g_random_period_history_count < RANDOM_BATCH_SIZE)
            g_random_period_history[g_random_period_history_count++] = g_random_active_period;
        /* 已完成的10周期历史复制到稳定观察数组，下一批可立即覆盖工作记录。 */
        for (uint8_t i = 0U; i < RANDOM_BATCH_SIZE; ++i)
            g_random_last_batch_periods[i] = g_random_period_history[i];
        g_random_last_batch_period_count = g_random_period_history_count;
        g_random_period_history_count = 0U;

        /* 此时先写下一批第1个period到预装载，再手动UG强制统一生效。
         * 更新事件由软件发出而非周期自然到期，必须清除UIF并清理NVIC pending。
         * 然后再次排队第2个period，开UIE与CEN重启连续随机发射。
         */
        TIM3_QueuePeriod(Random_NextPeriod());
        TIM3->CNT = 0U;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->SR = (uint32_t)~TIM_SR_UIF;
        g_random_active_period = queued_period;
        TIM3_QueuePeriod(Random_NextPeriod());
        HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
        TIM3->DIER |= TIM_DIER_UIE;
        TIM3->CR1 |= TIM_CR1_CEN;
#if OUTPUT_TEST_ENABLE
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
#endif
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
