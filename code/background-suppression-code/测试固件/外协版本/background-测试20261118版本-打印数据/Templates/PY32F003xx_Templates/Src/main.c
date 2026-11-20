
/*
 * PY32F003 背景抑制 ADC 原始采样测试版。
 *
 * 1. TIM3按候选表随机抽取周期发射并触发ADC。
 * 2. ADC + DMA每帧采集25点，完成后快速重装。
 * 3. RTT按配置打印每帧指定范围的原始ADC值。
 * 4. PA12支持DMA中断耗时测试，普通模式固定为原遮光电平。
 */

#include "main.h"
#include <stdint.h>

/* =========================== RTT打印配置 =========================== */

/* RTT总开关。 */
#define APP_RTT_ENABLE                 1U

/* 1=打印每帧原始ADC。 */
#define APP_RTT_ADC_ENABLE             1U

/* 原始ADC打印范围，下标从0开始。 */
#define APP_RTT_ADC_START              6U
#define APP_RTT_ADC_COUNT              10U

/* 两个开关均打开时才编译RTT代码。 */
#define APP_RTT_ACTIVE (APP_RTT_ENABLE && APP_RTT_ADC_ENABLE)

#if APP_RTT_ACTIVE
#include "SEGGER_RTT.h"
#include <stdio.h>
#endif


/* ========================= 采样及发射配置 ========================== */

/* DMA每帧采集25个ADC点，下标0~24。 */
#define BUFFER_SIZE                   16U

/* 随机周期候选表元素数量。 */
#define RANDOM_PERIOD_COUNT           10U

/* TIM3初始化占位周期，启动前由候选表抽出的首个周期覆盖，单位us。 */
#define TIM3_INITIAL_PERIOD          100U

/* CH1在每个周期末尾预留3us的PWM输出段。 */
#define RANDOM_PWM_OFF_US              3U

/* CH2在周期结束前7us产生ADC触发沿。 */
#define RANDOM_ADC_BEFORE_END_US       7U


/* =========================== 输出参数 ============================== */

/* 1=原遮光电平为低电平；0=原遮光电平为高电平。 */
#define OUTPUT_LIGHT_ACTIVE_HIGH       1U

/*
 * PA12硬件测试模式。
 *
 * 0：PA12固定输出原遮光电平。
 *
 * 1：进入DMA中断时PA12拉低，
 *    DMA中断处理结束后PA12拉高。
 *    使用示波器测量低电平宽度，
 *    观察中断处理耗时。
 */
#define OUTPUT_TEST_ENABLE             0U


/* =========================== 编译期检查 ============================ */

/* 开关宏检查。 */
#if (APP_RTT_ENABLE > 1U) || \
    (APP_RTT_ADC_ENABLE > 1U) || \
    (OUTPUT_LIGHT_ACTIVE_HIGH > 1U) || \
    (OUTPUT_TEST_ENABLE > 1U)

#error "Enable macros must be 0 or 1"

#endif

/* 发射周期必须大于ADC提前触发时间和PWM预留时间。 */
#if (TIM3_INITIAL_PERIOD <= RANDOM_PWM_OFF_US) || \
    (TIM3_INITIAL_PERIOD <= RANDOM_ADC_BEFORE_END_US)

#error "Invalid TIM3 initial period"

#endif

/* RTT原始ADC打印窗口检查。 */
#if APP_RTT_ACTIVE

#if (APP_RTT_ADC_COUNT < 1U) || \
    (APP_RTT_ADC_START >= BUFFER_SIZE) || \
    (APP_RTT_ADC_COUNT > \
    (BUFFER_SIZE - APP_RTT_ADC_START))

#error "Invalid RTT ADC print range"

#endif

#endif


/* ============================ 外设句柄 ============================= */

ADC_HandleTypeDef AdcHandle;
DMA_HandleTypeDef hdma_adc;
TIM_HandleTypeDef Tim3Handle;
TIM_OC_InitTypeDef sConfig;
GPIO_InitTypeDef GPIO_InitStruct;


/* ============================ ADC缓冲区 ============================ */

/*
 * DMA每帧搬运25个ADC数据。
 * 下一帧DMA会覆盖当前数据。
 */
uint16_t adc_buffer[BUFFER_SIZE];


/* ========================== Keil Watch变量 ========================= */

/* 已经处理的DMA帧总数。 */
volatile uint32_t g_signal_frames = 0U;


/* ============================ 函数声明 ============================= */

static void APP_SystemClockConfig(void);
static void GPIO_Init(void);
static void TMR3_OC_Init(void);
static void APP_AdcInit(void);
static void APP_AdcStartDMA(void);
static void APP_AdcDmaFastRearm(void);


/* ============================ RTT打印 ============================== */

#if APP_RTT_ACTIVE

/*
 * RTT总发送失败次数。
 * 缓冲区不足或者格式化异常时增加。
 */
volatile uint32_t g_rtt_write_errors = 0U;


/*
 * 非阻塞发送一整行CSV数据。
 *
 * 使用SEGGER_RTT_MODE_NO_BLOCK_SKIP模式。
 * 如果RTT缓冲区无法容纳整行数据，直接丢弃。
 */
static void APP_RTT_WriteLine(const char *line,
                              int len,
                              uint32_t size)
{
    if ((len <= 0) ||
        ((uint32_t)len >= size))
    {
        ++g_rtt_write_errors;
        return;
    }

    if (SEGGER_RTT_Write(
            0,
            line,
            (unsigned int)len) != (unsigned int)len)
    {
        ++g_rtt_write_errors;
    }
}


/*
 * 格式：
 * A,frame,adc0,adc1,...
 *
 * 每次DMA完成后打印一次。
 * 实际打印范围由APP_RTT_ADC_START和APP_RTT_ADC_COUNT决定。
 */
static void APP_RTT_SendADC(const uint16_t *samples)
{
    char line[32U + APP_RTT_ADC_COUNT * 5U];

    uint32_t i;
    uint32_t used;
    int len;

    len = snprintf(
        line,
        sizeof(line),
        "A,%lu",
        (unsigned long)g_signal_frames);

    if ((len <= 0) ||
        ((uint32_t)len >= sizeof(line)))
    {
        ++g_rtt_write_errors;
        return;
    }

    used = (uint32_t)len;

    for (i = 0U; i < APP_RTT_ADC_COUNT; ++i)
    {
        len = snprintf(
            &line[used],
            sizeof(line) - used,
            ",%u",
            (unsigned int)samples[i]);

        if ((len <= 0) ||
            ((uint32_t)len >= sizeof(line) - used))
        {
            ++g_rtt_write_errors;
            return;
        }

        used += (uint32_t)len;
    }

    len = snprintf(
        &line[used],
        sizeof(line) - used,
        "\r\n");

    if ((len <= 0) ||
        ((uint32_t)len >= sizeof(line) - used))
    {
        ++g_rtt_write_errors;
        return;
    }

    used += (uint32_t)len;

    APP_RTT_WriteLine(
        line,
        (int)used,
        sizeof(line));
}

#endif /* APP_RTT_ACTIVE */


/* ========================== TIM3随机周期 =========================== */

/* 随机周期候选表，单位us。候选值相同时，抽取结果也相同。 */
static const uint16_t random_period_table[RANDOM_PERIOD_COUNT] =
{
    50U, 50U, 50U, 50U, 50U,
    50U, 50U, 50U, 50U, 50U
};

/* Xorshift随机数状态。 */
static uint32_t rng_state = 0x6D2B79F5UL;


/* 随机抽取下一个发射周期。 */
static uint16_t Random_NextPeriod(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;

    return random_period_table[
        rng_state % RANDOM_PERIOD_COUNT];
}


/*
 * 将下一个周期写入TIM3预装载寄存器。
 * ARR和CCR在更新事件时同步生效。
 */
static void TIM3_QueuePeriod(uint16_t period)
{
    TIM3->ARR =
        (uint32_t)(period - 1U);

    TIM3->CCR1 =
        (uint32_t)(period - RANDOM_PWM_OFF_US);

    TIM3->CCR2 =
        (uint32_t)(period - RANDOM_ADC_BEFORE_END_US);
}


/* TIM3更新中断，预装载下一个抽取的周期。 */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR = (uint32_t)~TIM_SR_UIF;

        TIM3_QueuePeriod(Random_NextPeriod());
    }
}


/* ========================== DMA完成中断 ============================ */

/*
 * DMA每完成25个ADC点后进入一次中断。
 * 更新帧号，按配置打印原始ADC，然后快速重装DMA。
 * 测试模式下，PA12的低电平宽度用于测量中断耗时。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {
#if APP_RTT_ACTIVE
        uint16_t rtt_adc_snapshot[APP_RTT_ADC_COUNT];
        uint32_t i;
#endif

#if OUTPUT_TEST_ENABLE
        GPIOA->BRR = GPIO_PIN_12;
#endif

#if APP_RTT_ACTIVE
        for (i = 0U; i < APP_RTT_ADC_COUNT; ++i)
        {
            rtt_adc_snapshot[i] = adc_buffer[APP_RTT_ADC_START + i];
        }
#endif

        ++g_signal_frames;

#if APP_RTT_ACTIVE
        APP_RTT_SendADC(rtt_adc_snapshot);
#endif

        APP_AdcDmaFastRearm();

#if OUTPUT_TEST_ENABLE
        GPIOA->BSRR = GPIO_PIN_12;
#endif
    }
}


/* ============================ 主函数 ============================== */

/*
 * 初始化顺序：
 * HAL -> 时钟 -> GPIO -> TIM3 -> ADC/DMA
 * -> 准备ADC/DMA -> 启动TIM3。
 */
int main(void)
{
    HAL_Init();

    APP_SystemClockConfig();


#if APP_RTT_ACTIVE

    SEGGER_RTT_Init();

    SEGGER_RTT_SetFlagsUpBuffer(
        0,
        SEGGER_RTT_MODE_NO_BLOCK_SKIP);

#endif


    GPIO_Init();

    TMR3_OC_Init();

    APP_AdcInit();

    APP_AdcStartDMA();

    TIM3_QueuePeriod(Random_NextPeriod());

    TIM3->EGR = TIM_EGR_UG;

    TIM3->SR = (uint32_t)~TIM_SR_UIF;

    TIM3_QueuePeriod(Random_NextPeriod());

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);

    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);

    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->DIER |= TIM_DIER_UIE;


    if (HAL_TIM_OC_Start(
            &Tim3Handle,
            TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    if (HAL_TIM_OC_Start(
            &Tim3Handle,
            TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    while (1)
    {
    }
}


/* =========================== GPIO初始化 ============================ */

/*
 * PA5：普通推挽输出，默认高电平。
 * PA12：固定为原遮光电平，或用于DMA中断耗时测试。
 * PB5：上拉输入。
 */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin =
        GPIO_PIN_5 | GPIO_PIN_12;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_5;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        GPIO_PIN_SET);



#if OUTPUT_TEST_ENABLE

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_12,
        GPIO_PIN_SET);

#else

#if OUTPUT_LIGHT_ACTIVE_HIGH

    GPIOA->BRR = GPIO_PIN_12;

#else

    GPIOA->BSRR = GPIO_PIN_12;

#endif

#endif
}


/* =========================== TIM3初始化 ============================ */

/*
 * TIM3计数频率：1MHz。
 * CH1：发射PWM。
 * CH2：OC2REF经TRGO触发ADC。
 */
static void TMR3_OC_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



    Tim3Handle.Instance = TIM3;

    Tim3Handle.Init.Period =
        TIM3_INITIAL_PERIOD - 1U;

    Tim3Handle.Init.Prescaler = 24 - 1;

    Tim3Handle.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    Tim3Handle.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    Tim3Handle.Init.RepetitionCounter = 1 - 1;

    Tim3Handle.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)
    {
        APP_ErrorHandler();
    }



    sConfig.OCMode = TIM_OCMODE_PWM1;

    sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

    sConfig.OCFastMode = TIM_OCFAST_ENABLE;

    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;

    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

    sConfig.Pulse =
        TIM3_INITIAL_PERIOD - RANDOM_PWM_OFF_US;

    if (HAL_TIM_OC_ConfigChannel(
            &Tim3Handle,
            &sConfig,
            TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }



    sConfig.OCMode = TIM_OCMODE_PWM2;

    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

    sConfig.OCFastMode = TIM_OCFAST_ENABLE;

    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;

    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

    sConfig.Pulse =
        TIM3_INITIAL_PERIOD - RANDOM_ADC_BEFORE_END_US;

    if (HAL_TIM_OC_ConfigChannel(
            &Tim3Handle,
            &sConfig,
            TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    TIM3->CCMR1 |=
        TIM_CCMR1_OC1PE |
        TIM_CCMR1_OC2PE;



    {
        TIM_MasterConfigTypeDef master = {0};

        master.MasterOutputTrigger =
            TIM_TRGO_OC2REF;

        master.MasterSlaveMode =
            TIM_MASTERSLAVEMODE_DISABLE;

        if (HAL_TIMEx_MasterConfigSynchronization(
                &Tim3Handle,
                &master) != HAL_OK)
        {
            APP_ErrorHandler();
        }
    }
}


/* =========================== ADC初始化 ============================= */

/*
 * ADC通道0。
 * 12位分辨率。
 * TIM3 TRGO外部触发。
 * 连续转换模式。
 * DMA NORMAL模式，每帧25点。
 */
static void APP_AdcInit(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();
    __HAL_RCC_ADC_CLK_ENABLE();

    AdcHandle.Instance = ADC1;



    if (HAL_ADCEx_Calibration_Start(
            &AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }



    AdcHandle.Instance = ADC1;

    AdcHandle.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV1;

    AdcHandle.Init.Resolution =
        ADC_RESOLUTION_12B;

    AdcHandle.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    AdcHandle.Init.ScanConvMode =
        ADC_SCAN_DIRECTION_BACKWARD;

    AdcHandle.Init.LowPowerAutoWait = DISABLE;

    AdcHandle.Init.ContinuousConvMode = ENABLE;

    AdcHandle.Init.DiscontinuousConvMode = DISABLE;

    AdcHandle.Init.ExternalTrigConv =
        ADC_EXTERNALTRIGCONV_T3_TRGO;

    AdcHandle.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_RISING;

    AdcHandle.Init.DMAContinuousRequests = DISABLE;

    AdcHandle.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;

    AdcHandle.Init.SamplingTimeCommon =
        ADC_SAMPLETIME_3CYCLES_5;

    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }



    sConfig.Channel = ADC_CHANNEL_0;

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    if (HAL_ADC_ConfigChannel(
            &AdcHandle,
            &sConfig) != HAL_OK)
    {
        APP_ErrorHandler();
    }



    HAL_SYSCFG_DMA_Req(0);

    __HAL_RCC_DMA_CLK_ENABLE();

    hdma_adc.Instance = DMA1_Channel1;

    hdma_adc.Init.Direction =
        DMA_PERIPH_TO_MEMORY;

    hdma_adc.Init.PeriphInc =
        DMA_PINC_DISABLE;

    hdma_adc.Init.MemInc =
        DMA_MINC_ENABLE;

    hdma_adc.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_HALFWORD;

    hdma_adc.Init.MemDataAlignment =
        DMA_MDATAALIGN_HALFWORD;

    hdma_adc.Init.Mode = DMA_NORMAL;

    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;

    HAL_DMA_Init(&hdma_adc);

    __HAL_LINKDMA(
        &AdcHandle,
        DMA_Handle,
        hdma_adc);

    HAL_NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        1,
        0);

    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


/* =========================== ADC DMA启动 =========================== */

/*
 * 启动首帧DMA。
 * 等待TIM3触发ADC。
 */
static void APP_AdcStartDMA(void)
{
    if (HAL_ADC_Start_DMA(
            &AdcHandle,
            (uint32_t *)adc_buffer,
            BUFFER_SIZE) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    __HAL_DMA_DISABLE_IT(
        &hdma_adc,
        DMA_IT_HT);
}


/* =========================== DMA快速重装 =========================== */

/*
 * 保持原寄存器操作顺序不变。
 * 不在本次修改中调整ADC连续转换及DMA重装策略。
 */
static void APP_AdcDmaFastRearm(void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

    DMA1->IFCR = DMA_IFCR_CGIF1;

    DMA1_Channel1->CNDTR = BUFFER_SIZE;

    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;

    ADC1->ISR = ADC_ISR_OVR;

    ADC1->CR |= ADC_CR_ADEN;

    ADC1->CR |= ADC_CR_ADSTART;
}


/* =========================== 系统时钟配置 ========================== */

/*
 * 使用内部HSI 24MHz。
 * SYSCLK、HCLK、PCLK1均为24MHz。
 */
static void APP_SystemClockConfig(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE |
        RCC_OSCILLATORTYPE_HSI |
        RCC_OSCILLATORTYPE_LSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

#if defined(RCC_HSIDIV_SUPPORT)

    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;

#endif

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_24MHz;

    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;

    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        APP_ErrorHandler();
    }



    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0) != HAL_OK)
    {
        APP_ErrorHandler();
    }
}


/* =========================== 错误处理 ============================== */

void APP_ErrorHandler(void)
{
    while (1)
    {
    }
}
