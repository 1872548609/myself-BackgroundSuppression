/**
 ******************************************************************************
 * @file    main_raw.c
 * @brief   光电传感器 —— 纯数据采集固件(无判决 / 无标定 / 无状态机)
 *
 * 设计目标
 * ---------------------------------------------------------------------------
 *   只做一件事: 按 20kHz 发射时序采样, 每 PRINT_EVERY 帧输出一帧完整的
 *   75 点原始 ADC 值。所有判据、滤波、标定都在 PC 端离线做 —— 这样换算法
 *   不必重烧固件, 也不会因为固件里的状态机掩盖真实波形。
 *
 * 输出格式 (SEGGER RTT Terminal 0, 文本)
 * ---------------------------------------------------------------------------
 *   上电先打一行配置:
 *     #CFG,period=50,on=47,trig=35,len=75,every=100
 *   之后每 PRINT_EVERY 帧打一行:
 *     F,<frame_no>,<ms>,<v0>,<v1>,...,<v74>\r\n
 *       frame_no : 上电以来的总帧数(含未输出的帧), 用它判断有没有丢帧
 *       ms       : 输出时刻 HAL_GetTick()
 *       v0..v74  : 12bit 原始 ADC 值, 下标 0 ≈ 触发后 2.9us
 *
 * 带宽换算 (每帧约 400 字节)
 * ---------------------------------------------------------------------------
 *   PRINT_EVERY=1000 ->  20 帧/s ->   8 KB/s   仅看慢过程
 *   PRINT_EVERY= 200 -> 100 帧/s ->  40 KB/s   1MHz SWD 可用
 *   PRINT_EVERY= 100 -> 200 帧/s ->  80 KB/s   默认, 需 4MHz SWD
 *   PRINT_EVERY=  50 -> 400 帧/s -> 160 KB/s   容易丢, 必须确认 frame_no 连续
 *   若 frame_no 出现跳跃, 说明 RTT 上行带宽不够, 请加大 PRINT_EVERY 或提高 SWD 速度。
 *
 * 关发射 (测电学基线, 与"开发射"逐点对比)
 * ---------------------------------------------------------------------------
 *   EMIT_ENABLE 改成 0: CCR1=0, PWM1 永不满足 CNT<CCR, 恒输出无效电平(高),
 *   LED 常灭。采样时序、ADC、DMA 全部不变, 因此两组数据可直接对齐比较。
 *
 * 一个必须自查的隐患
 * ---------------------------------------------------------------------------
 *   ADC 连续转换 + DMA 循环, 依赖"75 点恰好 = 50.00us"。若 ADC 实际单点时间
 *   偏离 0.6667us, 波形会随 frame_no 缓慢滑移(每帧滑 75*Δ)。
 *   自查: 取第 0 帧和第 5000 帧, 看 19/24 号峰的横坐标是否移动。
 *   若移动, 说明需要改成 T3_TRGO 每周期重新触发(ContinuousConvMode=DISABLE),
 *   否则任何固定下标的窗口判据都会失效。
 ******************************************************************************
 */

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "SEGGER_RTT.h"

/* ======================= 采集配置 ======================= */

#define EMIT_ENABLE           1U        /* 0: 只采样不发射 */
#define EMIT_PERIOD_US        50U       /* 发射周期, TIM3 计数 1MHz -> 20kHz */
#define EMIT_ON_US            (EMIT_ENABLE ? 47U : 0U)
#define CH2_TRIG_US           35U       /* ADC 触发沿(TIM3-CH2) */

#define FRAME_LEN             75U       /* 50us / (16/24MHz) */
#define FRAME_LEN_2           (FRAME_LEN * 2U)

#define PRINT_EVERY           100U      /* 每 N 帧输出 1 帧, 200 帧/s, 范围 1..65535 */
#define OUT_MAX               512U      /* 单行缓冲, 75 点约 400 字节 */
#define WDT_TIMEOUT_MS        100U      /* 帧流看门狗 */

/* ======================= 运行状态 ======================= */

static uint16_t adc_frame[FRAME_LEN_2];

/* 双缓冲: 中断写 g_snap[g_wr^1] 后再翻 g_wr, 主循环读 g_snap[g_wr] 永不相撞 */
static uint16_t g_snap[2][FRAME_LEN];

static volatile uint8_t  g_wr;
static volatile uint8_t  g_new;
static volatile uint16_t g_cnt;
static volatile uint32_t g_fno;

/* ======================= 帧钩子 ======================= */

static void frame_hook(const uint16_t *src)
{
    ++g_fno;

    if (++g_cnt >= PRINT_EVERY) {
        uint8_t w = (uint8_t)(g_wr ^ 1U);

        g_cnt = 0U;
        memcpy(g_snap[w], src, sizeof(g_snap[0]));
        g_wr = w;
        g_new = 1U;
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        frame_hook(&adc_frame[0]);
    }

    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        frame_hook(&adc_frame[FRAME_LEN]);
    }
}

/* ======================= 输出 ======================= */

static char g_line[OUT_MAX];

static char *u2s(char *p, uint32_t v)
{
    char t[10];
    uint8_t n = 0U;

    do {
        t[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);

    while (n != 0U) {
        *p++ = t[--n];
    }
    return p;
}

static void dump_frame(void)
{
    uint8_t r;
    uint32_t fno;
    uint32_t i;
    char *p = g_line;

    if (g_new == 0U) {
        return;
    }

    __disable_irq();
    r = g_wr;
    fno = g_fno;
    g_new = 0U;
    __enable_irq();

    *p++ = 'F';
    *p++ = ',';
    p = u2s(p, fno);
    *p++ = ',';
    p = u2s(p, HAL_GetTick());

    for (i = 0U; i < FRAME_LEN; ++i) {
        *p++ = ',';
        p = u2s(p, g_snap[r][i]);
    }
    *p++ = '\r';
    *p++ = '\n';

    SEGGER_RTT_Write(0, g_line, (unsigned)(p - g_line));
}

/* ======================= 初始化 ======================= */

static void APP_SystemClockConfig(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;
    osc.HSEState = RCC_HSE_OFF;
    osc.LSIState = RCC_LSI_OFF;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        APP_ErrorHandler();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        APP_ErrorHandler();
    }
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Pin = GPIO_PIN_6;                 /* TIM3_CH1 发射 PWM */
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_5 | GPIO_PIN_12;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_5;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    GPIOA->BRR = GPIO_PIN_12;            /* 调试脚拉低 */
}

static void TIM3_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    TIM_HandleTypeDef h = {0};
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    h.Instance = TIM3;
    h.Init.Prescaler = 24U - 1U;
    h.Init.Period = EMIT_PERIOD_US - 1U;
    h.Init.CounterMode = TIM_COUNTERMODE_UP;
    h.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h.Init.RepetitionCounter = 0U;
    h.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&h) != HAL_OK) {
        APP_ErrorHandler();
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_LOW;
    oc.OCFastMode = TIM_OCFAST_ENABLE;
    oc.Pulse = EMIT_ON_US;

    if (HAL_TIM_OC_ConfigChannel(&h, &oc, TIM_CHANNEL_1) != HAL_OK) {
        APP_ErrorHandler();
    }

    oc.OCMode = TIM_OCMODE_PWM2;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.Pulse = CH2_TRIG_US;

    if (HAL_TIM_OC_ConfigChannel(&h, &oc, TIM_CHANNEL_2) != HAL_OK) {
        APP_ErrorHandler();
    }

    master.MasterOutputTrigger = TIM_TRGO_OC2REF;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&h, &master) != HAL_OK) {
        APP_ErrorHandler();
    }

    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    if (HAL_TIM_OC_Start(&h, TIM_CHANNEL_1) != HAL_OK) {
        APP_ErrorHandler();
    }
    if (HAL_TIM_OC_Start(&h, TIM_CHANNEL_2) != HAL_OK) {
        APP_ErrorHandler();
    }
}

static void ADC_DMA_Init(uint32_t ch)
{
    ADC_HandleTypeDef adc = {0};
    ADC_ChannelConfTypeDef cc = {0};
    DMA_HandleTypeDef hdma = {0};

    __HAL_RCC_ADC_CLK_ENABLE();

    adc.Instance = ADC1;
    if (HAL_ADCEx_Calibration_Start(&adc) != HAL_OK) {
        APP_ErrorHandler();
    }

    adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    adc.Init.Resolution = ADC_RESOLUTION_12B;
    adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    adc.Init.ContinuousConvMode = ENABLE;
    adc.Init.DiscontinuousConvMode = DISABLE;
    adc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc.Init.DMAContinuousRequests = ENABLE;
    adc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adc.Init.SamplingTimeCommon = ADC_SAMPLETIME_3CYCLES_5;

    if (HAL_ADC_Init(&adc) != HAL_OK) {
        APP_ErrorHandler();
    }

    cc.Channel = ch;
    cc.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&adc, &cc) != HAL_OK) {
        APP_ErrorHandler();
    }

    HAL_SYSCFG_DMA_Req(0);
    __HAL_RCC_DMA_CLK_ENABLE();

    hdma.Instance = DMA1_Channel1;
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma.Init.Mode = DMA_CIRCULAR;
    hdma.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma) != HAL_OK) {
        APP_ErrorHandler();
    }

    __HAL_LINKDMA(&adc, DMA_Handle, hdma);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    if (HAL_ADC_Start_DMA(&adc, (uint32_t *)adc_frame, FRAME_LEN_2) != HAL_OK) {
        APP_ErrorHandler();
    }
}

/* ======================= 主函数 ======================= */

int main(void)
{
    uint32_t last_fno = 0U;
    uint32_t last_tick;
    uint8_t stalled = 0U;
    uint32_t t0;
    char cfg[96];
    int n;

    HAL_Init();
    APP_SystemClockConfig();
    GPIO_Init();
    TIM3_Init();

    SEGGER_RTT_Init();
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    ADC_DMA_Init(ADC_CHANNEL_0);

    /* 等 ADC/DMA 真正跑起来, 否则第一行输出可能是空帧 */
    t0 = HAL_GetTick();
    while (g_fno == 0U) {
        if ((HAL_GetTick() - t0) >= 100U) {
            APP_ErrorHandler();
        }
    }

    n = snprintf(cfg, sizeof(cfg),
                 "#CFG,period=%u,on=%u,trig=%u,len=%u,every=%u\r\n",
                 (unsigned)EMIT_PERIOD_US, (unsigned)EMIT_ON_US,
                 (unsigned)CH2_TRIG_US, (unsigned)FRAME_LEN,
                 (unsigned)PRINT_EVERY);
    if (n > 0) {
        SEGGER_RTT_Write(0, cfg, (unsigned)n);
    }

    last_tick = HAL_GetTick();

    while (1) {
        dump_frame();

        /* 帧流看门狗: ADC 停摆时明确报出来, 免得对着死数据做分析 */
        if (g_fno != last_fno) {
            last_fno = g_fno;
            last_tick = HAL_GetTick();
            stalled = 0U;
        } else if (stalled == 0U && (HAL_GetTick() - last_tick) >= WDT_TIMEOUT_MS) {
            stalled = 1U;
            SEGGER_RTT_WriteString(0, "#STALL,no frame for 100ms\r\n");
        }
    }
}

void APP_ErrorHandler(void)
{
    SEGGER_RTT_WriteString(0, "#FATAL,init failed\r\n");

    while (1) {
        /* 保持可被调试器暂停 */
    }
}
