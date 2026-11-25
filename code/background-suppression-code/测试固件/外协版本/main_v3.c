#include "main.h"
#include <stdint.h>

/* ======================= 采样与发射 ======================= */

#define EMIT_PERIOD_US        50U      /* 发射周期, TIM3 计数 1MHz */
#define EMIT_ON_US            47U      /* 点亮 47us, 末尾熄灭 3us */
#define CH2_TRIG_US           35U      /* ADC 触发沿(TIM3-CH2): 帧首≈TIM3 37.9us, 峰/谷落进 PEAK/TRO 窗(数据回放选优) */

#define ADC_CONV_CYCLES       16U      /* 3.5 采样 + 12.5 转换 */
#define FRAME_LEN             75U      /* 50us / (16/24MHz) = 75, 必须整除 */
#define FRAME_LEN_2           (FRAME_LEN * 2U)

/* 峰/谷取样窗口(帧内下标)。2026-11-19 测试1.log 实测校准:
 * 有物时谷=20、峰=25, 150/240 轮零漂移; 改窗后无物 49~56, 有物 493~587 */
#define PEAK0                 24U
#define PEAK1                 25U
#define PEAK2                 26U
#define TRO0                  19U
#define TRO1                  20U
#define TRO2                  21U

/* ======================= 判决参数 ======================= */

#define MA_N                  16U      /* 帧间平均长度, 必须是 2 的幂 */
#define MA_SHIFT              4U

#define K_ON                  6        /* 置位: n > max(K_ON*sigma, THR_MIN) */
#define K_OFF                 3        /* 复位: n < max(K_OFF*sigma, REL_MIN) */

#define THR_MIN_CODES         100      /* 最小可检测对比度(码), 1 码 = 1.22mV */
#define THR_MIN               (THR_MIN_CODES * 3)   /* score 是三对三之和 */
#define REL_MIN               (THR_MIN * 2 / 3)     /* 释放门限 200: 无物 n≈126 可释放, 有物 n≈570 保持 */

#define DEBOUNCE_N            16U      /* 连续 16 帧(0.8ms)满足才翻转, 抑制边界抖动 */

/*
 * 定点技巧: base/sigma 用 Q8 存放(<<8)。
 * 若直接对码值差右移 14 位, 差值小于 16384 时增量恒为 0, 滤波器会死住;
 * 用 Q8 后增量分辨率为 1 码, 同时还能保持很长的 tau。
 */
#define QSH                   8
#define BASE_DOWN_SHIFT       6U       /* 下降快: tau ≈ 64 帧 ≈ 3ms */
#define BASE_UP_SHIFT         10U      /* 上升慢: tau ≈ 1024 帧 ≈ 50ms */
#define SIG_SHIFT             8U       /* 噪声估计速率: tau ≈ 256 帧 */
#define SIGMA_MIN             8        /* 噪声下限(码) */
#define SIGMA_MAX             400      /* 噪声上限, 防止一次扰动把阈值顶飞 */
#define QUIET_GATE            2        /* 保留: |n| < thr 才允许学习 */

#define CAL_FRAMES            256U     /* 上电标定帧数: 空场时快速收敛; 有物时拒绝学习, 标定后直接置位 */

#define STARTUP_MIN_FRAMES    8U       /* 启动自检: 认为 ADC 已跑起来的帧数门槛 */
#define STARTUP_TIMEOUT_MS    100U     /* 启动自检超时(HAL_GetTick), 超时进 ErrorHandler */

#define OUT_ACTIVE_HIGH       1U       /* 1: 有物输出高 */
#define CALIB_DUMP            1U       /* 1: 主循环每 ~200ms 打印一次平均帧轮廓(75点)+状态, 仅占主循环 */

#if CALIB_DUMP
#include "SEGGER_RTT.h"
#include <stdio.h>
#endif

/* ======================= 状态 ======================= */

static uint16_t adc_frame[FRAME_LEN_2];    /* DMA 循环写的双缓冲 */

static volatile uint32_t g_frames;
static int32_t  g_base_q8;                 /* 背景电平, Q8; 初值 0 = 峰谷相等 */
static int32_t  g_sigma_q8 = SIGMA_MIN << QSH;  /* 上电未学习时的安全初值 */
static uint8_t  g_state;                   /* 0=无物 1=有物 */
static uint8_t  g_on_cnt;                  /* 置位去抖计数 */
static uint8_t  g_off_cnt;                 /* 复位去抖计数 */

static int32_t  g_h3[3];
static uint8_t  g_h3i;
static uint8_t  g_h3_ready;                /* 0: 首帧尚未到达(避免静态 0 值污染 med3) */
static int32_t  g_ring[MA_N];
static int32_t  g_ring_sum;
static uint8_t  g_ri;
static uint8_t  g_warm;                    /* 环形缓冲填充计数 */

#if CALIB_DUMP
/*
 * 校准累加器: 每 4 帧抽 1 帧逐下标累加(中断内 +~5us, 半帧 25us 窗口内安全),
 * 主循环每 ~1000 个累计帧打印一次"平均帧轮廓"。看打印的 75 个均值,
 * 正尖峰/负尖峰对应的下标即为 PEAK/TRO 应取的位置。
 */
#define CAL_ACC_EVERY        4U           /* 每 4 帧累加 1 帧 */
#define CAL_ACC_FRAMES       1000U        /* 累计帧数达此值则打印并清零(= 4000 帧 ≈ 200ms) */
static uint32_t g_cal_acc[FRAME_LEN];
static volatile uint32_t g_cal_n;        /* 中断写/主循环轮询, 必须 volatile */
static uint32_t g_cal_snap[FRAME_LEN];    /* 主循环快照, 避免 RTT 读撕裂 */

/* 本轮(1000 个抽样帧)判决统计, 验收"无抖动/裕量"用 */
static volatile uint32_t g_cal_flips;
static volatile uint32_t g_cal_over;
static volatile int32_t  g_cal_smin = 0x7FFFFFFF;
static volatile int32_t  g_cal_smax = -0x7FFFFFFF;
#endif

/* ======================= 基础运算 ======================= */

/* 三取中: 3 次比较, 剔除孤立坏帧 */
static int32_t med3(int32_t v)
{
    int32_t a, b, c, lo, hi;

    if (g_h3_ready == 0U) {
        /* 首帧: 三槽全部填实测值, 前 2 帧不再混入静态 0 */
        g_h3[0] = v;
        g_h3[1] = v;
        g_h3[2] = v;
        g_h3_ready = 1U;
        return v;
    }

    g_h3[g_h3i] = v;
    if (++g_h3i >= 3U) {
        g_h3i = 0U;
    }

    a = g_h3[0];
    b = g_h3[1];
    c = g_h3[2];

    lo = (a < b) ? a : b;
    hi = (a < b) ? b : a;

    if (c < lo) {
        return lo;
    }
    if (c > hi) {
        return hi;
    }
    return c;
}

/* 滑动平均: 环形 + 累加和, 每帧 1 加 1 减 */
static int32_t ma(int32_t v)
{
    g_ring_sum += v - g_ring[g_ri];
    g_ring[g_ri] = v;

    if (++g_ri >= MA_N) {
        g_ri = 0U;
    }
    if (g_warm < MA_N) {
        ++g_warm;
    }

    if (g_warm >= MA_N) {
        /* 稳态 MA_N=16 是 2 的幂: 移位代替除法(Cortex-M0 无硬件除法) */
        return (g_ring_sum + (1 << (MA_SHIFT - 1))) >> MA_SHIFT;
    }

    return g_ring_sum / (int32_t)g_warm;    /* 仅前 15 帧走除法 */
}

/* 帧内同步解调: 正尖峰窗 - 负尖峰窗(三点对三点, 权重和=0) */
static int32_t frame_score(const uint16_t *f)
{
    int32_t p = (int32_t)f[PEAK0] + (int32_t)f[PEAK1] + (int32_t)f[PEAK2];
    int32_t t = (int32_t)f[TRO0] + (int32_t)f[TRO1] + (int32_t)f[TRO2];

    return p - t;
}

/* ======================= 判决 ======================= */

static void set_output(uint8_t on)
{
#if OUT_ACTIVE_HIGH
    if (on) {
        GPIOA->BSRR = GPIO_PIN_12;
    } else {
        GPIOA->BRR = GPIO_PIN_12;
    }
#else
    if (on) {
        GPIOA->BRR = GPIO_PIN_12;
    } else {
        GPIOA->BSRR = GPIO_PIN_12;
    }
#endif
}

static void calibrate(int32_t v)
{
    int32_t vq = v << QSH;
    int32_t d0 = vq - g_base_q8;
    int32_t a0 = (d0 < 0) ? -d0 : d0;

    /* 跳过环形缓冲填充期 */
    if (g_warm < MA_N) {
        return;
    }

    /*
     * 只学"像背景"的样本: 偏差 >= THR_MIN 说明上电时可能已有物体,
     * 拒绝学习, base 保持 0, 标定结束后由 decide() 直接置位。
     * 这样非空场上电不会被学成背景(弱于 THR_MIN 的目标仍属物理边界)。
     */
    if ((a0 >> QSH) >= THR_MIN) {
        return;
    }

    g_base_q8 += d0 >> 3;                        /* 快速收敛 */

    {
        int32_t d = vq - g_base_q8;
        if (d < 0) {
            d = -d;
        }
        g_sigma_q8 += (d - g_sigma_q8) >> 2;
        if (g_sigma_q8 < (SIGMA_MIN << QSH)) {
            g_sigma_q8 = SIGMA_MIN << QSH;
        }
        if (g_sigma_q8 > (SIGMA_MAX << QSH)) {     /* 与 decide() 一致, 标定期强扰动不顶飞阈值 */
            g_sigma_q8 = SIGMA_MAX << QSH;
        }
    }
}

static void decide(int32_t v)
{
    int32_t vq = v << QSH;
    int32_t n = (vq - g_base_q8) >> QSH;
    int32_t a = (n < 0) ? -n : n;
    int32_t sigma = g_sigma_q8 >> QSH;
    int32_t thr = (int32_t)K_ON * sigma;

    if (thr < THR_MIN) {
        thr = THR_MIN;
    }

    if (g_state == 0U) {
        g_off_cnt = 0U;

        if (n > thr) {
            if (++g_on_cnt >= DEBOUNCE_N) {
                g_on_cnt = 0U;
                g_state = 1U;
                set_output(1U);
#if CALIB_DUMP
                ++g_cal_flips;
#endif
                return;
            }
        } else {
            g_on_cnt = 0U;
        }

        if (a < thr) {
            int32_t d = vq - g_base_q8;

            if (d < 0) {
                g_base_q8 += d >> BASE_DOWN_SHIFT;
            } else {
                g_base_q8 += d >> BASE_UP_SHIFT;
            }

            g_sigma_q8 += ((a << QSH) - g_sigma_q8) >> SIG_SHIFT;

            if (g_sigma_q8 < (SIGMA_MIN << QSH)) {
                g_sigma_q8 = SIGMA_MIN << QSH;
            }
            if (g_sigma_q8 > (SIGMA_MAX << QSH)) {
                g_sigma_q8 = SIGMA_MAX << QSH;
            }
        }
    } else {
        int32_t rel = (int32_t)K_OFF * sigma;

        if (rel < REL_MIN) {
            rel = REL_MIN;
        }

        g_on_cnt = 0U;

        if (n < rel) {
            if (++g_off_cnt >= DEBOUNCE_N) {
                g_off_cnt = 0U;
                g_state = 0U;
                set_output(0U);
#if CALIB_DUMP
                ++g_cal_flips;
#endif
            }
        } else {
            g_off_cnt = 0U;
        }
    }
}

/* 一帧: 加权 -> 中值 -> 平均 -> 判决 */
static void process_frame(const uint16_t *f)
{
    int32_t s = frame_score(f);

#if CALIB_DUMP
    if ((g_frames % CAL_ACC_EVERY) == 0U) {
        uint32_t i;
        for (i = 0; i < FRAME_LEN; ++i) {
            g_cal_acc[i] += f[i];
        }
        ++g_cal_n;
    }
#endif

    s = med3(s);
    s = ma(s);

#if CALIB_DUMP
    if ((g_frames % CAL_ACC_EVERY) == 0U) {
        if (s < g_cal_smin) {
            g_cal_smin = s;
        }
        if (s > g_cal_smax) {
            g_cal_smax = s;
        }
        if (s > THR_MIN) {
            ++g_cal_over;
        }
    }
#endif

    if (g_frames < CAL_FRAMES) {
        calibrate(s);
    } else {
        decide(s);
    }

    ++g_frames;
}

/* ======================= DMA 中断 ======================= */

/*
 * 半满: adc_frame[0..74] 完整; 全满: adc_frame[75..149] 完整。
 * 循环模式下 DMA 自己绕回, 中断里只处理数据, 不做任何重装/打印。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        process_frame(&adc_frame[0]);
    }

    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        process_frame(&adc_frame[FRAME_LEN]);
    }
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

    g.Pin = GPIO_PIN_5 | GPIO_PIN_12;   /* PA5 预置高 + PA12 检测输出(同 V1) */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_5;                 /* PB5 上拉输入(同 V1) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    set_output(0U);
}

/* TIM3: 1MHz 计数, ARR=49 -> 50us;
 * CH1: PWM1+低有效, 高 47us(点亮) + 低 3us(熄灭, 周期末尾);
 * CH2: PWM2, CCR2=CH2_TRIG_US -> OC2REF 上升沿在 t=CH2_TRIG_US, 经 TRGO 硬件触发 ADC。 */
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
    oc.OCPolarity = TIM_OCPOLARITY_LOW;      /* 低电平点亮 */
    oc.OCFastMode = TIM_OCFAST_ENABLE;
    oc.Pulse = EMIT_ON_US;

    if (HAL_TIM_OC_ConfigChannel(&h, &oc, TIM_CHANNEL_1) != HAL_OK) {
        APP_ErrorHandler();
    }

    oc.OCMode = TIM_OCMODE_PWM2;             /* CNT>=CCR 时 OC2REF 为高 -> 上升沿在 CCR2 */
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

    if (HAL_TIM_OC_Start(&h, TIM_CHANNEL_2) != HAL_OK) {   /* 无引脚, 仅内部 TRGO */
        APP_ErrorHandler();
    }
}

/* ADC 由 TIM3-TRGO 硬件触发启动, 之后自由连续 1.5MS/s + DMA 循环双缓冲
 * (相位由 CH2 硬锁定; 触发->首样本延迟 ~2.9us, 首样本 ≈ TIM3 t≈37.9us) */
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

    adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;   /* 24MHz */
    adc.Init.Resolution = ADC_RESOLUTION_12B;
    adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    adc.Init.ContinuousConvMode = ENABLE;                 /* 触发一次后自由连续 */
    adc.Init.DiscontinuousConvMode = DISABLE;
    adc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;   /* V1 做法: 硬件锁相 */
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc.Init.DMAContinuousRequests = ENABLE;
    adc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adc.Init.SamplingTimeCommon = ADC_SAMPLETIME_3CYCLES_5;  /* 3.5+12.5=16 周期 */

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
    hdma.Init.Mode = DMA_CIRCULAR;                        /* 关键: 循环 */
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
    HAL_Init();
    APP_SystemClockConfig();
    GPIO_Init();
    TIM3_Init();

#if CALIB_DUMP
    SEGGER_RTT_Init();
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif

    /*
     * 相位由硬件锁定: TIM3-CH2(TRGO) 触发 ADC 启动, 之后自由连续 + 循环 DMA。
     * HAL_ADC_Start_DMA 内部使能 ADC 后, 下一个 CH2 沿(t=CH2_TRIG_US)自动开始转换,
     * 不再依赖"等 UIF 后软件启动"的不确定延迟。
     */
    ADC_DMA_Init(ADC_CHANNEL_0);

    /*
     * 启动自检(替代 V2 早期 "while(UIF==0){}" 的无超时死等):
     * CH2 / TRGO / ADC 触发链任一环节异常时, ADC 不启动、DMA 无数据、判决永不运行。
     * 用 HAL_GetTick() 限 100ms, 正常 ~0.4ms(8 帧)即通过; 超时进 ErrorHandler 便于定位。
     */
    {
        uint32_t t0 = HAL_GetTick();
        uint32_t now;

        while (g_frames < STARTUP_MIN_FRAMES) {
            now = HAL_GetTick();
            if ((now - t0) >= STARTUP_TIMEOUT_MS) {
                APP_ErrorHandler();
            }
        }
    }

    {
        uint32_t last_frames = 0U;
        uint32_t last_tick = HAL_GetTick();

        while (1) {
            /* 帧流看门狗: 100ms 无新帧说明 ADC/DMA/触发链停摆, 输出安全态 */
            if (g_frames != last_frames) {
                last_frames = g_frames;
                last_tick = HAL_GetTick();
            } else if ((HAL_GetTick() - last_tick) >= 100U) {
                g_state = 0U;
                g_on_cnt = 0U;
                g_off_cnt = 0U;
                set_output(0U);
                last_tick = HAL_GetTick();
            }

#if CALIB_DUMP
            /*
             * 校准打印(只在主循环, 不进中断, 不影响判决):
             *   D,frame,ms,base,sigma,state,accn,flips,smin,smax,over —— 状态行
             *   P,idx,mean                      —— 75 个下标的平均码值
             * 尖峰下标与 PEAK/TRO 不吻合时, 微调 CH2_TRIG_US(整 us)或宏后重烧。
             */
            if (g_cal_n >= CAL_ACC_FRAMES) {
                char line[64];
                uint32_t i, n;
                uint32_t flips, over;
                int32_t smin, smax;
                int len;

                __disable_irq();
                for (i = 0; i < FRAME_LEN; ++i) {
                    g_cal_snap[i] = g_cal_acc[i];
                    g_cal_acc[i] = 0U;
                }
                n = g_cal_n;
                g_cal_n = 0U;
                flips = g_cal_flips;
                g_cal_flips = 0U;
                over = g_cal_over;
                g_cal_over = 0U;
                smin = g_cal_smin;
                g_cal_smin = 0x7FFFFFFF;
                smax = g_cal_smax;
                g_cal_smax = -0x7FFFFFFF;
                __enable_irq();

                len = snprintf(line, sizeof(line),
                               "D,%lu,%lu,%ld,%ld,%u,%lu,%lu,%ld,%ld,%lu\r\n",
                               (unsigned long)g_frames,
                               (unsigned long)HAL_GetTick(),   /* ms: SysTick 毫秒, 无除法开销 */
                               (long)(g_base_q8 >> QSH), (long)(g_sigma_q8 >> QSH),
                               (unsigned)g_state, (unsigned long)n,
                               (unsigned long)flips, (long)smin, (long)smax,
                               (unsigned long)over);
                if (len > 0) {
                    SEGGER_RTT_Write(0, line, (unsigned)len);
                }
                for (i = 0; i < FRAME_LEN; ++i) {
                    len = snprintf(line, sizeof(line), "P,%lu,%lu\r\n",
                                   (unsigned long)i,
                                   (unsigned long)(n ? (g_cal_snap[i] / n) : 0U));
                    if (len > 0) {
                        SEGGER_RTT_Write(0, line, (unsigned)len);
                    }
                }
            }
#endif
        }
    }
}

void APP_ErrorHandler(void)
{
    while (1) {
    }
}
