/*
 * 基础配置说明版
 *
 * 配置流程：
 * HAL初始化 -> 系统时钟 -> GPIO -> TIM3配置 -> ADC/DMA配置
 * -> DMA先启动并等待触发 -> 启动TIM3 CH2和CH1 -> DMA完成中断寄存器快速重装
 *
 * 本文件仅为基础配置展示：
 * 1. 不含滤波、阈值判定、NO/NC控制算法和测试观察变量。
 * 2. 宏控制固定周期或随机周期；随机模式才启用TIM3更新中断，ADC与DMA采集机制不变。
 * 3. 两种模式都在周期末前1us由TIM3 CH2的OC2REF上升沿发出TRGO。
 * 4. ADC仍采用连续转换，DMA仍为NORMAL模式，每轮搬运10点；中断不关闭ADC外设。
 *    因此本版并不保证每个随机发射周期只采1点；这是保留新版ADC配置的结果。
 * 5. 随机模式最短周期30us，必须上板检查TIM3更新ISR开销和DMA重装耗时。
 * 6. 需要结合main.h、HAL底层MSP及芯片参考手册确认ADC0对应引脚和DMA请求映射。
 */

/* 工程头文件：提供HAL类型、外设枚举及APP_ErrorHandler声明等。 */
#include "main.h"
/* 提供uint16_t等标准整数类型。 */
#include <stdint.h>

/*
 * 发射周期工作模式：
 * 0 = 固定周期：始终使用FIXED_TRANSMIT_PERIOD，关闭TIM3更新中断。
 * 1 = 随机周期：按random_period_table逐周期生成随机候选周期。
 * 两种模式都保留CH1 PWM和CH2 TRGO；ADC、DMA与PA12测试不变。
 */
#define TRANSMIT_RANDOM_ENABLE     1U
#define FIXED_TRANSMIT_PERIOD      110U

/* 随机发射周期：取值单位us，TIM3计数频率1MHz。 */
#define RANDOM_PERIOD_COUNT       10U
#define RANDOM_INITIAL_PERIOD     100U

#if TRANSMIT_RANDOM_ENABLE
#define TIM3_INITIAL_PERIOD RANDOM_INITIAL_PERIOD
#else
#define TIM3_INITIAL_PERIOD FIXED_TRANSMIT_PERIOD
#endif
#define RANDOM_PWM_OFF_US         3U
#define RANDOM_ADC_BEFORE_END_US  1U

#if TRANSMIT_RANDOM_ENABLE
/* 从main(4).c移植的候选表；可以随机重复同一周期。 */
static const uint16_t random_period_table[RANDOM_PERIOD_COUNT] = {
    30U, 40U, 50U, 60U, 70U, 80U, 90U, 100U, 110U, 120U
};

/* 固定种子的xorshift32是伪随机序列，每次上电重复相同序列。 */
static uint32_t rng_state = 0x6D2B79F5UL;

static uint16_t Random_NextPeriod(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return random_period_table[rng_state % RANDOM_PERIOD_COUNT];
}

/* 只写预装载值；ARR、CCR1、CCR2在下一个TIM3更新事件同时生效。 */
static void TIM3_QueuePeriod(uint16_t period)
{
    TIM3->ARR  = (uint32_t)(period - 1U);
    TIM3->CCR1 = (uint32_t)(period - RANDOM_PWM_OFF_US);
    TIM3->CCR2 = (uint32_t)(period - RANDOM_ADC_BEFORE_END_US);
}

/* 更新中断仅预先安排下一轮随机周期，不进行ADC算法或统计。 */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR = (uint32_t)~TIM_SR_UIF;
        TIM3_QueuePeriod(Random_NextPeriod());
    }
}
#endif
/* 每次DMA传输10个ADC转换结果；每个结果存入一个16位元素，并非10路ADC。 */
#define BUFFER_SIZE 10

/*
 * 输出测试宏：1=开启，0=关闭。
 * 使用现有PA12作为示波器测试输出，不额外占用GPIO。
 * 开启后，DMA TC中断真正进入处理分支时PA12拉低，完成快速重新武装后拉高。
 * 示波器测量PA12低电平宽度，可估算本轮DMA完成处理路径的耗时。
 * 注意：不包含DMA传输本身的时间，也不包含IRQ入口到标记语句前的时间；
 * GPIO函数执行及中断抢占也影响观察值，测试宏开启会增加少量执行开销。
 */
#define OUTPUT_TEST_ENABLE 1

/* ADC1句柄，保存ADC初始化参数和HAL运行状态。 */
ADC_HandleTypeDef AdcHandle;
/* ADC使用的DMA1通道1句柄。 */
DMA_HandleTypeDef hdma_adc;
/* TIM3时基及通道所关联的HAL句柄。 */
TIM_HandleTypeDef Tim3Handle;
/* CH1/CH2共用输出比较配置结构，每次配置通道时会重新赋值。 */
TIM_OC_InitTypeDef sConfig;
/* GPIO配置结构，分别用于PA5/PA12、PB5及PA6。 */
GPIO_InitTypeDef GPIO_InitStruct;
/* DMA传输目的缓冲区，存储ADC右对齐12位结果。 */
uint16_t adc_buffer[BUFFER_SIZE];

static void APP_SystemClockConfig(void);
static void GPIO_Init(void);
static void TMR3_OC_Init(void);
static void APP_AdcInit(void);
static void APP_AdcStartDMA(void);

/* 主程序入口：先配置全部外设，后启动DMA和TIM3，避免TIM3触发早于ADC准备完成。 */
int main(void)
{
    /* 初始化HAL及基础时基。 */
    HAL_Init();
    /* 选择24MHz HSI系统时钟，AHB和APB1均不分频。 */
    APP_SystemClockConfig();
    /* 设置普通输出和选择输入；PA6复用由TIM3配置函数设置。 */
    GPIO_Init();
    /* 配置TIM3 CH1 PWM、CH2 PWM2及TRGO，暂不启动计数。 */
    TMR3_OC_Init();
    /* ADC校准、通道配置、DMA映射与NVIC配置。 */
    APP_AdcInit();
    /* 先使ADC/DMA进入等待硬件触发状态。 */
    APP_AdcStartDMA();

#if TRANSMIT_RANDOM_ENABLE
    /* 随机模式：首周期UG生效，再排队第二个周期，开启TIM3更新中断。 */
    TIM3_QueuePeriod(Random_NextPeriod());
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = (uint32_t)~TIM_SR_UIF;

    TIM3_QueuePeriod(Random_NextPeriod());
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->DIER |= TIM_DIER_UIE;
#else
    /* 固定模式：ARR/CCR在TIM3初始化中设置，这里只加载预装载值。 */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = (uint32_t)~TIM_SR_UIF;
    TIM3->DIER &= ~TIM_DIER_UIE;
#endif

    /* 启动CH2通道；它的内部OC2REF用于TIM3 TRGO触发，不要求配置CH2外部引脚。 */
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
          APP_ErrorHandler();
    /* 启动CH1并从PA6输出PWM；CH1和CH2使用同一TIM3计数器。 */
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
          APP_ErrorHandler();

    /* 当前主循环无业务处理，采集链路由定时器、ADC、DMA及DMA中断完成。 */
    while (1)
    {
    }
}
/* 普通GPIO配置：PA5、PA12推挽输出；PB5上拉输入。 */
static void GPIO_Init(void)
{

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 同时选择PA5与PA12。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12;

    /* 推挽输出，GPIO可主动输出高或低电平。 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    /* 内部上拉；对主动驱动的推挽输出通常不是必要条件，保留原配置。 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 切换到GPIOB的PB5引脚。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;

    /* 普通数字输入，继承Pull=GPIO_PULLUP。 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* 复用上面的Pull=UP、Speed=HIGH，PB5作为带上拉的输入。 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA5初始化为高电平；程序中不再根据采样结果控制该引脚。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);

    /* 测试开启时，PA12平时保持高电平；DMA处理期间输出低脉冲。 */
#if OUTPUT_TEST_ENABLE
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
#else
    /* 测试关闭后保留纯净版PA12初始化状态。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
#endif
}

/* TIM3 24MHz/24=1MHz；ARR及两路CCR由随机周期函数动态预装载。 */
static void TMR3_OC_Init(void)
{

    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6设置为TIM3 CH1的复用输出。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;

    /* 使用外设复用推挽模式，不再由普通GPIO输出寄存器直接控制。 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 选择PA6上的AF1 TIM3功能，实际映射需以芯片手册为准。 */
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 选择TIM3外设实例。 */
    Tim3Handle.Instance = TIM3;

    /* 初始化占位周期；main函数启动前会随机覆盖预装载值并通过UG生效。 */
    Tim3Handle.Init.Period            = TIM3_INITIAL_PERIOD - 1U;

    /* PSC=23，24MHz/(23+1)=1MHz，因此一个CNT计数对应1us。 */
    Tim3Handle.Init.Prescaler         = 24 - 1;

    /* 定时器CKD不分频；通常不直接改变由PSC决定的计数频率。 */
    Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;

    /* 向上计数，到ARR后更新回零。 */
    Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;

    /* 重复计数值为0；TIM3若不支持重复计数器，该字段可能无实际作用。 */
    Tim3Handle.Init.RepetitionCounter = 1 - 1;

    /* 使能ARR预装载，后续修改ARR一般在更新事件生效。 */
    Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    /* CH1采用PWM1模式；逻辑有效区间通常为CNT<CCR1。 */
    sConfig.OCMode       = TIM_OCMODE_PWM1;

    /* CH1采用低有效极性，使PWM1有效段在PA6表现为低电平。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_LOW;

    /* 输出比较快速模式；保留原值，需确认芯片实际支持及用途。 */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;

    /* 以下OCN及空闲状态字段保留原值，本程序没有启用互补输出。 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;

    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* CH1低有效PWM1：周期末尾3us输出高电平，其他时间输出低电平。 */
    sConfig.Pulse = TIM3_INITIAL_PERIOD - RANDOM_PWM_OFF_US;

    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    /* CH2采用PWM2模式，其内部OC2REF在CNT达到CCR2时由低变高。 */
     sConfig.OCMode       = TIM_OCMODE_PWM2;

    /* CH2配置高有效；TRGO取自OC2REF内部信号，而非PA6物理电平。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;

    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;

    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;

    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* CH2 PWM2：在周期终点前1us产生OC2REF上升沿，作为ADC触发。 */
    sConfig.Pulse = TIM3_INITIAL_PERIOD - RANDOM_ADC_BEFORE_END_US;

    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    /* 三个预装载寄存器在更新事件统一生效，防止周期改变时CCR错位。 */
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    /* 配置TIM3主触发输出TRGO。 */
     TIM_MasterConfigTypeDef master = {0};
    /* 将CH2内部OC2REF送到TRGO，ADC通过TRGO上升沿开始转换。 */
     master.MasterOutputTrigger = TIM_TRGO_OC2REF;
    /* 不启用定时器主从同步机制。 */
     master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
     if (HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &master) != HAL_OK)
     {
         APP_ErrorHandler();
     }

}

/* ADC1及DMA配置：ADC0单通道，12位，TIM3 TRGO触发，连续转换，DMA一次搬运10个点。 */
static void APP_AdcInit(void)
{

    /* ADC通道局部配置结构，初始清零。 */
    ADC_ChannelConfTypeDef sConfig = {0};

    /* 复位ADC寄存器到初始状态。 */
    __HAL_RCC_ADC_FORCE_RESET();

    /* 释放ADC复位。 */
    __HAL_RCC_ADC_RELEASE_RESET();

    /* 使能ADC模块时钟。 */
    __HAL_RCC_ADC_CLK_ENABLE();

    AdcHandle.Instance = ADC1;

    /* 启动ADC校准；当前原代码在HAL_ADC_Init之前执行，顺序未修改。 */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    AdcHandle.Instance = ADC1;

    /* ADC时钟同步PCLK且不分频；本配置下名义为24MHz，需核对芯片允许范围。 */
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;

    /* 12位ADC：转换数据的有效数值范围为0~4095。 */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;

    /* 数据右对齐，DMA按16位半字搬运。 */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;

    /* 扫描方向向后；仅启用ADC_CHANNEL_0，因此实质上仅采集一个通道。 */
    AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;

    /* 不开启自动等待：新转换可能等待上一个数据被读取，以避免数据未取走即覆盖。 */
    AdcHandle.Init.LowPowerAutoWait = DISABLE;

    /* 开启连续转换：一次接受外部触发后尝试持续转换，而非每点都等待下一次TRGO。 */
    AdcHandle.Init.ContinuousConvMode = ENABLE;
    /* 关闭不连续扫描。 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;

    /* ADC外部触发源选TIM3 TRGO。 */
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    /* ADC仅响应所选TRGO的上升沿。 */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

    /* DMA连续请求关闭；它与ADC连续转换、NORMAL DMA的协调是需要重点审核的配置。 */
    AdcHandle.Init.DMAContinuousRequests = DISABLE;

    /* ADC结果溢出时用新数据覆盖旧数据；此处没有额外溢出统计。 */
    AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

    /* ADC采样保持时间为3.5个ADC时钟周期；不等于完整12位转换耗时。 */
    AdcHandle.Init.SamplingTimeCommon=ADC_SAMPLETIME_3CYCLES_5;

    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    /* 选择ADC输入通道0；对应实体管脚须查芯片引脚复用表和HAL MSP初始化。 */
    sConfig.Channel = ADC_CHANNEL_0;

    /* 以硬件通道编号确定转换顺序。 */
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    /* 配置DMA请求映射到ADC；参数0的含义应查当前HAL接口定义。 */
    HAL_SYSCFG_DMA_Req(0);

    /* 使能DMA外设时钟。 */
    __HAL_RCC_DMA_CLK_ENABLE();

    /* 选择DMA1通道1处理ADC结果。 */
    hdma_adc.Instance = DMA1_Channel1;

    /* 外设到内存：ADC数据寄存器 -> adc_buffer。 */
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;

    /* 外设寄存器地址固定，不递增。 */
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;

    /* 内存地址递增：依次填充数组中的10个元素。 */
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;

    /* 从ADC数据寄存器读取半字，即16位。 */
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;

    /* 内存按16位半字写入，与uint16_t缓冲区匹配。 */
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;

    /* DMA普通模式：传满10点即停止，不会自动从数组开头循环。 */
    hdma_adc.Init.Mode = DMA_NORMAL;

    /* DMA总线仲裁优先级为中，与NVIC中断优先级是不同概念。 */
    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;

    /* 将上述参数写入DMA硬件；原代码未检查此调用返回值。 */
    HAL_DMA_Init(&hdma_adc);

    /* 建立ADC与DMA句柄关联，供HAL_ADC_Start_DMA及HAL回调使用。 */
    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc);

    /* 设置DMA1_CH1中断优先级：抢占优先级1，子优先级0。 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);

    /* 启用DMA1_CH1中断；完成后的重装由对应IRQ入口处理。 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/* 启动一轮ADC DMA传输：10个结果写入数组，随后等待下一次重装。 */
static void APP_AdcStartDMA(void)
{
    /* 配置并启动ADC DMA。第三个参数10代表DMA要搬运的转换结果个数。 */
    if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)
    {
            APP_ErrorHandler();
    }

    /* 关闭DMA半传输中断，仅保留传输完成中断路径。 */
    __HAL_DMA_DISABLE_IT(&hdma_adc, DMA_IT_HT);
}

/*
 * DMA NORMAL 快速重新武装（仅由 DMA1_Channel1_IRQHandler 调用）。
 *
 * 目标：
 * 1. ADC始终保持ADEN使能，不执行ADC_Disable和HAL_ADC_Stop_DMA。
 * 2. DMA搬满10点后结束本轮连续转换。
 * 3. DMA通道重装CNDTR=10，下一次TRGO再启动下一组连续转换。
 *
 * 重要限制：
 * - 这是直接寄存器控制路径，首次启动仍由HAL_ADC_Start_DMA完成。
 * - 中断中的HAL软件状态不再逐组更新；运行期间不要混用HAL_ADC_Stop_DMA/
 *   HAL_ADC_Start_DMA或其他依赖HAL状态机的ADC操作。
 * - TIM3始终继续计数。重装窗口内的TRGO可能丢失，不保证每个PWM周期都采一组。
 * - 采用STM32F0/Puya兼容的ADC_CR_ADSTP、ADSTART、CFGR1_DMAEN寄存器定义。
 * - 若目标HAL实际寄存器定义不同，需以该芯片device header为准。
 */
#define ADC_REARM_WAIT_LIMIT  10000U

static void APP_AdcDmaFastRearm(void)
{
    uint32_t timeout;

    /* DMA NORMAL传满后一般硬件已停止。再次清EN确保CNDTR可写。 */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

//    /* 终止ADC当前连续转换，但保持ADEN=1，不关闭ADC外设。 */
//    if ((ADC1->CR & ADC_CR_ADSTART) != 0U)
//    {
//        ADC1->CR |= ADC_CR_ADSTP;
//        timeout = ADC_REARM_WAIT_LIMIT;
//        while ((ADC1->CR & ADC_CR_ADSTP) != 0U)
//        {
//            if (--timeout == 0U) APP_ErrorHandler();
//        }
//    }

    /* 转换停止后关闭DMA请求，清掉上轮可能残留的状态标志。 */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

    /* 清DMA1 CH1的GI/TC/HT/TE全部标志，避免旧标志重入。 */
    DMA1->IFCR = DMA_IFCR_CGIF1;

    /* NORMAL模式重新装载传输数量；地址和传输宽度保持初始化时的值。 */
    DMA1_Channel1->CNDTR = BUFFER_SIZE;

    /* 先让DMA就绪，再允许ADC发送DMA请求。 */
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;

    /* 清ADC溢出标志，然后置ADSTART：等待下一次外部TRGO上升沿。 */
    ADC1->ISR = ADC_ISR_OVR;

    /* 使能 ADC，不重新初始化其他配置 */
    ADC1->CR |= ADC_CR_ADEN;


    ADC1->CR |= ADC_CR_ADSTART;
}

/* DMA中断只执行ADC停止转换及DMA快速重装；原有滤波、输出判断仍不恢复。 */
void DMA1_Channel1_IRQHandler(void)
{

#if OUTPUT_TEST_ENABLE
        /* PA12低电平区间表示中断内快速重新武装耗时。 */
        GPIOA->BRR = GPIO_PIN_12;
#endif
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {

        APP_AdcDmaFastRearm();

    }
//    /* 防止其他DMA异常标志造成中断不断重复；TE后停止继续运行并进入故障处理。 */
//    else if ((DMA1->ISR & DMA_ISR_TEIF1) != 0U)
//    {
//        DMA1->IFCR = DMA_IFCR_CGIF1;
//        APP_ErrorHandler();
//    }
    
#if OUTPUT_TEST_ENABLE
        GPIOA->BSRR = GPIO_PIN_12;
#endif
}

/* 系统时钟：HSI 24MHz -> SYSCLK 24MHz -> HCLK 24MHz -> PCLK1 24MHz。 */
static void APP_SystemClockConfig(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 指定本次要配置的振荡源种类；以下分别控制各自的启停。 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;

  /* 打开内部高速振荡器HSI。 */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

#if defined(RCC_HSIDIV_SUPPORT)

  /* 若HAL支持HSI分频功能，则选择不分频。 */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
#endif

  /* 选择24MHz HSI校准档。 */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;

  /* 不使用外部高速晶振。 */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;

  /* 关闭内部低速振荡器。 */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* 配置SYSCLK/HCLK/PCLK1三个时钟域。 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;

  /* 系统时钟直接来源于HSI。 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

  /* AHB不分频：HCLK=SYSCLK=24MHz。 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  /* APB1不分频：PCLK1=HCLK=24MHz。 */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  /* 写入总线时钟参数，Flash等待周期设为0，需符合当前电压/频率限制。 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/* 统一错误处理：发生外设初始化或重启动失败后进入死循环。 */
void APP_ErrorHandler(void)
{

  while (1)
  {
  }
}
