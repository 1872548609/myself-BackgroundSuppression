/*
 * PY32F003 背景抑制：18点DMA + 单点10帧分组MAD + 随机发射 + 组末单帧相关积分。
 * 前N-1帧仅累计指定索引；第N帧计算MAD、更新干扰状态，非冻结时
 * 只对当前帧计算正负窗口积分 -> 8点截尾平均 -> IIR -> 死区 -> 输出判断。
 * 强干扰冻结全部后级，MAD继续累计；保留随机发射、ADC/DMA初始化及快速重装时序。
 */
#include "main.h"
#include <stdint.h>

/* ========================================================================== */
/* 01. 参数配置 */
/* ========================================================================== */

/* 1.1 采样与随机发射 */
#define BUFFER_SIZE                      18U          /* DMA每帧采集18个ADC点，下标0~17。 */
#define RANDOM_INITIAL_PERIOD            100U         /* TIM3初始化时的占位周期，单位us。 */
#define RANDOM_PWM_OFF_US                3U           /* CH1在每个周期末尾预留3us的PWM输出段。 */
#define RANDOM_ADC_BEFORE_END_US         2U           /* CH2在周期结束前2us产生ADC触发沿。 */

/* 修改候选表即可调整随机周期；当前候选值均为40us，实际周期仍为40us。 */
static const uint16_t random_period_table[] =
{
    40U, 40U, 40U, 40U, 40U,
    40U, 40U, 40U, 40U, 40U
};
#define RANDOM_PERIOD_COUNT              (sizeof(random_period_table) / sizeof(random_period_table[0]))

/* 1.2 单点分组MAD */
#define SIGNAL_PRECHECK_EFFECT_ENABLE    1U           /* 1=干扰状态影响后级；0=仅观察MAD，仍按组末处理。 */
#define SIGNAL_ADC_INDEX                 1U           /* 每个DMA帧只采集此索引用于MAD。 */
#define SIGNAL_MAD_WINDOW_SIZE           10U          /* 非重叠分组帧数，可设2~255。 */
#define SIGNAL_MAD_WEAK_ENTER            50U          /* MAD >= 50进入弱干扰。 */
#define SIGNAL_MAD_WEAK_RELEASE          10U          /* MAD <= 10解除弱干扰。 */
#define SIGNAL_MAD_STRONG_ENTER          100U         /* MAD > 100累计强干扰确认。 */
#define SIGNAL_MAD_STRONG_RELEASE        90U          /* MAD <= 90累计恢复确认。 */
#define SIGNAL_MAD_STRONG_CONFIRM        5U           /* 连续5组确认强干扰。 */
#define SIGNAL_MAD_RECOVER_CONFIRM       5U           /* 连续5组确认恢复。 */

/* 1.3 正负窗口相关积分
 * 下标区间首尾均包含，正窗口+1、负窗口-1；两窗口必须等长且不重叠。
 * MAD使用指定索引跨帧检测；相关积分仅使用MAD组末当前帧。
 * 当前积分=ADC[0..4]之和-ADC[5..9]之和。
 */
#define CORR_POS_START                   0U           /* 正窗口起点。 */
#define CORR_POS_END                     4U           /* 正窗口终点。 */
#define CORR_NEG_START                   5U           /* 负窗口起点。 */
#define CORR_NEG_END                     9U           /* 负窗口终点。 */
#define CORR_POS_COUNT                   (CORR_POS_END - CORR_POS_START + 1U)
#define CORR_NEG_COUNT                   (CORR_NEG_END - CORR_NEG_START + 1U)

/* 1.4 后级滤波与输出（保留各功能开关，便于调试） */
#define SIGNAL_FILTER_Q_BITS             8U           /* IIR定点小数精度，内部状态=ADC码值*256。 */
#define SIGNAL_FILTER_NORMAL_SHIFT       1U           /* 正常IIR=1/2，仅在有效组末积分结果时更新。 */
#define SIGNAL_FILTER_WEAK_SHIFT         2U           /* 弱干扰IIR=1/4，仅在有效组末积分结果时更新。 */
#define SIGNAL_TRIMMED_MEAN_ENABLE       1U           /* 1=开启8个有效组末积分值的截尾平均；0=直接使用单帧积分。 */
#define SIGNAL_TRIM_WINDOW_SIZE          8U           /* 截尾历史数量；当前固定8组，去掉两端各2组。 */
#define OUTPUT_DEADBAND_ENABLE           1U           /* 1=开启IIR后输出死区；0=跳过死区，直接使用IIR结果。 */
#define OUTPUT_DEADBAND                  10U          /* 积分结果死区：相对上次保持值，变化绝对值<=10不更新。 */
/* 后级输出阈值使用正负窗口相关积分的数值，与MAD独立。 */
#define OUTPUT_OFF_THRESHOLD             -1          /* 积分滤波值<=-10连续确认遮光。 */
#define OUTPUT_HYSTERESIS                280          /* 有光阈值=OUTPUT_OFF_THRESHOLD+OUTPUT_HYSTERESIS=270。 */
#define OUTPUT_WEAK_OFF_THRESHOLD        -1          /* 弱干扰时遮光阈值。 */
#define OUTPUT_WEAK_HYSTERESIS           400          /* 弱干扰有光阈值=-10+200=190。 */
#define OUTPUT_LIGHT_CONFIRM_COUNT       1U           /* 连续3次有效后级结果达到ON阈值才确认有光。 */
#define OUTPUT_DARK_CONFIRM_COUNT        1U           /* 连续3次有效后级结果不高于OFF阈值才确认遮光。 */
#define OUTPUT_LIGHT_ACTIVE_HIGH         1U           /* 1=有光输出高电平；0=有光输出低电平。 */
#define OUTPUT_TEST_ENABLE               0U           /* 1=PA12测DMA中断耗时；0=PA12输出有光状态。 */

/* ========================================================================== */
/* 02. 编译期参数检查 */
/* ========================================================================== */

#if (SIGNAL_PRECHECK_EFFECT_ENABLE != 0U) && (SIGNAL_PRECHECK_EFFECT_ENABLE != 1U)
#error "SIGNAL_PRECHECK_EFFECT_ENABLE must be 0 or 1"
#endif
#if (CORR_POS_START > CORR_POS_END) || (CORR_NEG_START > CORR_NEG_END)
#error "Correlation window start must be <= end"
#endif
#if (CORR_POS_END >= BUFFER_SIZE) || (CORR_NEG_END >= BUFFER_SIZE)
#error "Correlation index exceeds DMA buffer"
#endif
#if !((CORR_POS_END < CORR_NEG_START) || (CORR_NEG_END < CORR_POS_START))
#error "Correlation windows must not overlap"
#endif
#if (CORR_POS_COUNT != CORR_NEG_COUNT)
#error "Correlation positive and negative windows must be equal length"
#endif
#if ((2UL * CORR_POS_COUNT * 4095UL) > (2147483647UL >> SIGNAL_FILTER_Q_BITS))
#error "Correlation Q-format may overflow int32_t"
#endif

#if (SIGNAL_ADC_INDEX >= BUFFER_SIZE)
#error "MAD ADC index exceeds DMA buffer"
#endif
#if (SIGNAL_MAD_WINDOW_SIZE < 2U) || (SIGNAL_MAD_WINDOW_SIZE > 255U)
#error "MAD window must be within 2..255"
#endif
#if (SIGNAL_MAD_WEAK_RELEASE >= SIGNAL_MAD_WEAK_ENTER) || \
    (SIGNAL_MAD_WEAK_ENTER >= SIGNAL_MAD_STRONG_ENTER) || \
    (SIGNAL_MAD_STRONG_RELEASE >= SIGNAL_MAD_STRONG_ENTER)
#error "Invalid MAD thresholds or hysteresis"
#endif
#if (SIGNAL_MAD_STRONG_CONFIRM < 1U) || (SIGNAL_MAD_STRONG_CONFIRM > 255U) || \
    (SIGNAL_MAD_RECOVER_CONFIRM < 1U) || (SIGNAL_MAD_RECOVER_CONFIRM > 255U)
#error "MAD confirmation counts must be within 1..255"
#endif
#if (SIGNAL_FILTER_NORMAL_SHIFT > 15U) || (SIGNAL_FILTER_WEAK_SHIFT > 15U) || \
    (SIGNAL_FILTER_WEAK_SHIFT <= SIGNAL_FILTER_NORMAL_SHIFT)
#error "Invalid IIR shift values"
#endif
#if (SIGNAL_TRIM_WINDOW_SIZE != 8U)
#error "Current trimmed mean implementation requires 8 samples"
#endif
#if (OUTPUT_DEADBAND_ENABLE > 1U)
#error "OUTPUT_DEADBAND_ENABLE must be 0 or 1"
#endif
#if (SIGNAL_TRIMMED_MEAN_ENABLE > 1U) || (OUTPUT_TEST_ENABLE > 1U) || \
    (OUTPUT_LIGHT_ACTIVE_HIGH > 1U)
#error "Feature switches must be 0 or 1"
#endif
#if (OUTPUT_LIGHT_CONFIRM_COUNT < 1U) || (OUTPUT_LIGHT_CONFIRM_COUNT > 255U) || \
    (OUTPUT_DARK_CONFIRM_COUNT < 1U) || (OUTPUT_DARK_CONFIRM_COUNT > 255U)
#error "Output confirmation counts must be within 1..255"
#endif
#if (OUTPUT_DEADBAND > (CORR_POS_COUNT * 4095UL))
#error "OUTPUT_DEADBAND exceeds correlation magnitude bound"
#endif

/* ========================================================================== */
/* 03. 外设句柄、调试变量与算法状态 */
/* ========================================================================== */

ADC_HandleTypeDef AdcHandle;
DMA_HandleTypeDef hdma_adc;
TIM_HandleTypeDef Tim3Handle;
TIM_OC_InitTypeDef sConfig;
GPIO_InitTypeDef GPIO_InitStruct;
/* ==================== Keil Watch 调试变量 ====================
 * DMA 每完成18个ADC采样，才会执行一次本帧算法。
 * volatile 全局量可以直接加到 Keil Watch；数值随中断不断刷新。
 * 若需观察同一帧的对应关系，建议在 DMA1_Channel1_IRQHandler 内设置断点。
 */
uint16_t adc_buffer[BUFFER_SIZE];            /* 18点原始ADC数据，索引0~17；下一次DMA会覆盖，12位取值0~4095。 */

/* A. 单点MAD分组：每帧仅保存一个ADC值，满组后才计算MAD。 */
volatile uint32_t g_signal_frames = 0U;         /* DMA完成帧总数。 */
volatile uint16_t g_signal_mad_adc_mean = 0U;   /* 最近一组ADC均值。 */
volatile uint32_t g_signal_mad_value = 0U;      /* 最近一组平均绝对偏差，整数截断。 */
volatile uint16_t g_signal_mad_count = 0U;      /* 当前组累计帧数。 */
volatile uint8_t  g_signal_mad_ready = 0U;       /* 本帧是否完成MAD分组。 */
volatile uint32_t g_signal_mad_groups = 0U;     /* 完成的MAD组数。 */
volatile uint8_t  g_signal_noise_level = 0U;    /* 0正常，1弱干扰，2强干扰。 */
volatile uint8_t  g_signal_weak_active = 0U;
volatile uint8_t  g_signal_strong_active = 0U;
volatile uint8_t  g_signal_strong_count = 0U;
volatile uint8_t  g_signal_recover_count = 0U;
static uint16_t g_signal_mad_history[SIGNAL_MAD_WINDOW_SIZE];
static uint32_t g_signal_mad_sum = 0U;

/* B. 每个有效MAD组末仅计算一次当前帧积分，不做跨帧积分平均。 */
volatile int32_t  g_match_correlation = 0;
volatile uint32_t g_match_valid_frames = 0U;    /* 实际计算积分的帧数。 */
volatile uint32_t g_match_discarded_groups = 0U; /* 强干扰冻结而跳过的组数。 */
volatile int32_t  g_match_filtered_integral = 0;

/* C. 截尾与输出：每个有效单帧积分执行8点截尾 -> IIR -> 死区 -> 输出判断。 */
volatile int32_t  g_output_trimmed_integral = 0; /* 最近8个有效组末积分值的截尾平均，IIR输入。 */
volatile int32_t  g_output_deadband_value = 0;   /* 最新死区输出值：偏差<=OUTPUT_DEADBAND时保持上次结果；用于输出迟滞判断。 */
static uint8_t g_output_deadband_initialized = 0U; /* 死区首次处理标记：首次直接采用当前滤波值。 */
volatile uint8_t  g_output_light_state = 0U;    /* 当前逻辑光状态：0=遮光，1=有光；OUTPUT_TEST_ENABLE=1时不会映射到PA12。 */
volatile uint8_t  g_output_light_confirm = 0U;  /* 遮光状态下，连续满足ON门槛的后级分组结果数；达到设定次数切有光。 */
volatile uint8_t  g_output_dark_confirm = 0U;   /* 有光状态下，连续满足OFF门槛的后级分组结果数；达到设定次数切遮光。 */

/* D. 后级滤波内部状态；强干扰冻结时保持不变。 */
static int32_t g_signal_filter_q8 = 0;          /* IIR的Q8定点状态，约等于滤波ADC码值×256。 */
#if SIGNAL_TRIMMED_MEAN_ENABLE
static int32_t g_output_trim_history[SIGNAL_TRIM_WINDOW_SIZE] = {0}; /* 最近8个有效组末的单帧积分值，不含IIR。 */
static uint8_t g_output_trim_write = 0U;        /* 下一次将写入的历史数组位置，循环0~7。 */
static uint8_t g_output_trim_count = 0U;        /* 已收集的积分值数，上限8；满8组前不做截尾输出。 */
#endif

/* ========================================================================== */
/* 04. 函数声明与随机发射 */
/* ========================================================================== */

void APP_ErrorHandler(void);

static void APP_SystemClockConfig(void);
static void GPIO_Init(void);
static void TMR3_OC_Init(void);
static void APP_AdcInit(void);
static void APP_AdcStartDMA(void);
static void APP_AdcDmaFastRearm(void);

static uint32_t rng_state = 0x6D2B79F5UL;

static uint16_t Random_NextPeriod(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return random_period_table[rng_state % RANDOM_PERIOD_COUNT];
}

static void TIM3_QueuePeriod(uint16_t period)
{
    TIM3->ARR  = (uint32_t)(period - 1U);
    TIM3->CCR1 = (uint32_t)(period - RANDOM_PWM_OFF_US);
    TIM3->CCR2 = (uint32_t)(period - RANDOM_ADC_BEFORE_END_US);
}

void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR = (uint32_t)~TIM_SR_UIF;
        TIM3_QueuePeriod(Random_NextPeriod());
    }
}

/* ========================================================================== */
/* 05. 信号处理算法 */
/* ========================================================================== */

/* 5.1 分组MAD：每帧累计一个原始ADC值。第N帧才计算MAD，不使用固定直流基准。
 * MAD = sum(abs(x[i]*N - sum(x))) / (N*N)，避免均值先截断。
 * 12位ADC、N<=255时，全部中间变量均在32位范围内。
 */
static uint8_t APP_SignalMadUpdate(uint16_t sample)
{
    uint32_t i;
    uint32_t abs_sum = 0U;
    int32_t diff;
    const uint32_t n = SIGNAL_MAD_WINDOW_SIZE;

    g_signal_mad_history[g_signal_mad_count] = sample;
    g_signal_mad_sum += sample;
    ++g_signal_mad_count;
    if (g_signal_mad_count < n)
    {
        return 0U;
    }

    g_signal_mad_adc_mean = (uint16_t)(g_signal_mad_sum / n);
    for (i = 0U; i < n; ++i)
    {
        diff = (int32_t)g_signal_mad_history[i] * (int32_t)n
            - (int32_t)g_signal_mad_sum;
        abs_sum += (uint32_t)((diff < 0) ? -diff : diff);
    }

    g_signal_mad_value = abs_sum / (n * n);
    g_signal_mad_count = 0U;
    g_signal_mad_sum = 0U;
    ++g_signal_mad_groups;
    return 1U;
}

/* 5.2 干扰状态：每个MAD组末更新一次强弱干扰状态，强干扰优先。 */
static void APP_SignalInterferenceUpdate(void)
{
    /* 弱干扰双阈值迟滞，切换IIR系数及输出阈值。 */
    if (!g_signal_weak_active)
    {
        if (g_signal_mad_value >= SIGNAL_MAD_WEAK_ENTER)
        {
            g_signal_weak_active = 1U;
        }
    }
    else if (g_signal_mad_value <= SIGNAL_MAD_WEAK_RELEASE)
    {
        g_signal_weak_active = 0U;
    }

    if (!g_signal_strong_active)
    {
        g_signal_recover_count = 0U;
        if (g_signal_mad_value > SIGNAL_MAD_STRONG_ENTER)
        {
            if (g_signal_strong_count < SIGNAL_MAD_STRONG_CONFIRM)
            {
                ++g_signal_strong_count;
            }
            if (g_signal_strong_count >= SIGNAL_MAD_STRONG_CONFIRM)
            {
                g_signal_strong_active = 1U;
                g_signal_strong_count = 0U;
            }
        }
        else
        {
            g_signal_strong_count = 0U;
        }
    }
    else
    {
        g_signal_strong_count = 0U;
        if (g_signal_mad_value <= SIGNAL_MAD_STRONG_RELEASE)
        {
            if (g_signal_recover_count < SIGNAL_MAD_RECOVER_CONFIRM)
            {
                ++g_signal_recover_count;
            }
            if (g_signal_recover_count >= SIGNAL_MAD_RECOVER_CONFIRM)
            {
                g_signal_strong_active = 0U;
                g_signal_recover_count = 0U;
            }
        }
        else
        {
            g_signal_recover_count = 0U;
        }
    }

    g_signal_noise_level = g_signal_strong_active ? 2U : (g_signal_weak_active ? 1U : 0U);
}

/* 5.3 相关积分：正负窗口求和差分。直流基线因正负窗口点数相等自动抵消。 */
static int32_t APP_SignalCalculateCorrelation(const uint16_t *samples)
{
    uint32_t i;
    int32_t pos = 0, neg = 0;
    for (i = CORR_POS_START; i <= CORR_POS_END; ++i)
    {
        pos += samples[i];
    }
    for (i = CORR_NEG_START; i <= CORR_NEG_END; ++i)
    {
        neg += samples[i];
    }
    return pos - neg;
}

/* 5.4 截尾平均：8个组末单帧积分值去掉最大2个、最小2个，再求中间4组平均。
 * 排序仅对8个积分值操作，不会修改18点原始DMA数据。
 */
static uint8_t APP_OutputTrimUpdate(int32_t value)
{
#if SIGNAL_TRIMMED_MEAN_ENABLE
    int32_t ordered[SIGNAL_TRIM_WINDOW_SIZE];
    uint32_t i, j;
    int32_t sum = 0;

    g_output_trim_history[g_output_trim_write] = value;
    g_output_trim_write = (uint8_t)((g_output_trim_write + 1U) % SIGNAL_TRIM_WINDOW_SIZE);
    if (g_output_trim_count < SIGNAL_TRIM_WINDOW_SIZE)
    {
        ++g_output_trim_count;
    }
    if (g_output_trim_count < SIGNAL_TRIM_WINDOW_SIZE)
    {
        return 0U;
    }

    for (i = 0U; i < SIGNAL_TRIM_WINDOW_SIZE; ++i)
    {
        int32_t current = g_output_trim_history[i];
        j = i;
        while (j > 0U && ordered[j - 1U] > current)
        {
            ordered[j] = ordered[j - 1U];
            --j;
        }
        ordered[j] = current;
    }

    for (i = 2U; i < 6U; ++i)
    {
        sum += ordered[i];
    }

    g_output_trimmed_integral = sum / 4;
#else
    g_output_trimmed_integral = value;
#endif
    return 1U;
}

/* 5.5 IIR：前置影响开启时由MAD选择1/2或1/4；关闭时固定正常IIR。 */
static void APP_SignalFilterUpdate(int32_t sample)
{
#if SIGNAL_PRECHECK_EFFECT_ENABLE
    uint32_t shift = g_signal_weak_active ? SIGNAL_FILTER_WEAK_SHIFT : SIGNAL_FILTER_NORMAL_SHIFT;
#else
    uint32_t shift = SIGNAL_FILTER_NORMAL_SHIFT;
#endif
    int32_t target_q8 = sample * (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
    g_signal_filter_q8 += (target_q8 - g_signal_filter_q8) / (int32_t)(1U << shift);
    g_match_filtered_integral = g_signal_filter_q8 / (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
}

/* 5.6 输出死区：比较当前滤波值与上次死区保持值。
 * 首次直接使用滤波值，之后变化幅度<=OUTPUT_DEADBAND时不更新。
 * 使用int32_t有符号差值，正确处理相关积分为负时的变化幅度。
 */
static int32_t APP_OutputApplyDeadband(int32_t filtered)
{
#if OUTPUT_DEADBAND_ENABLE
    if (g_output_deadband_initialized == 0U)
    {
        g_output_deadband_value = filtered;
        g_output_deadband_initialized = 1U;
    }
    else
    {
        int32_t difference = filtered - g_output_deadband_value;
        if ((difference > (int32_t)OUTPUT_DEADBAND) ||
            (difference < -(int32_t)OUTPUT_DEADBAND))
        {
            g_output_deadband_value = filtered;
        }
    }
#else
    g_output_deadband_value = filtered;
#endif
    return g_output_deadband_value;
}

/* 5.7 输出状态与迟滞确认 */
static void APP_OutputSetLight(uint8_t light)
{
    g_output_light_state = light;
#if !OUTPUT_TEST_ENABLE
#if OUTPUT_LIGHT_ACTIVE_HIGH
    if (light)
    {
        GPIOA->BSRR = GPIO_PIN_12;
    }
    else
    {
        GPIOA->BRR  = GPIO_PIN_12;
    }
#else
    if (light)
    {
        GPIOA->BRR  = GPIO_PIN_12;
    }
    else
    {
        GPIOA->BSRR = GPIO_PIN_12;
    }
#endif
#endif
}

static void APP_OutputUpdateDecision(int32_t value)
{
    static uint8_t previous_level = 0U;
    uint8_t level = 0U;
    int32_t off = OUTPUT_OFF_THRESHOLD;
    int32_t hyst = OUTPUT_HYSTERESIS;
#if SIGNAL_PRECHECK_EFFECT_ENABLE
    if (g_signal_weak_active != 0U)
    {
        level = 1U;
        off = OUTPUT_WEAK_OFF_THRESHOLD;
        hyst = OUTPUT_WEAK_HYSTERESIS;
    }
#endif

    if (level != previous_level)
    {
        g_output_light_confirm = 0U;
        g_output_dark_confirm = 0U;
        previous_level = level;
    }

    if (g_output_light_state == 0U)
    {
        g_output_dark_confirm = 0U;
        if (value >= off + hyst)
        {
            if (++g_output_light_confirm >= OUTPUT_LIGHT_CONFIRM_COUNT)
            {
                g_output_light_confirm = 0U;
                APP_OutputSetLight(1U);
            }
        }
        else
        {
            g_output_light_confirm = 0U;
        }
    }
    else
    {
        g_output_light_confirm = 0U;
        if (value <= off)
        {
            if (++g_output_dark_confirm >= OUTPUT_DARK_CONFIRM_COUNT)
            {
                g_output_dark_confirm = 0U;
                APP_OutputSetLight(0U);
            }
        }
        else
        {
            g_output_dark_confirm = 0U;
        }
    }
}

/* 5.8 完整MAD组处理：先更新干扰状态，再决定是否计算当前帧积分。 */
static void APP_ProcessMadGroup(void)
{
    int32_t decision_value;

    APP_SignalInterferenceUpdate();

#if SIGNAL_PRECHECK_EFFECT_ENABLE
    if (g_signal_strong_active != 0U)
    {
        ++g_match_discarded_groups;
        g_output_light_confirm = 0U;
        g_output_dark_confirm = 0U;
        return;
    }
#endif

    /* 只计算当前MAD组的最后一帧，不累计积分。 */
    g_match_correlation = APP_SignalCalculateCorrelation(adc_buffer);
    ++g_match_valid_frames;

    if (APP_OutputTrimUpdate(g_match_correlation) == 0U)
    {
        return;
    }

    APP_SignalFilterUpdate(g_output_trimmed_integral);
    decision_value = APP_OutputApplyDeadband(g_match_filtered_integral);
    APP_OutputUpdateDecision(decision_value);
}

/* ========================================================================== */
/* 06. DMA中断：采集一帧 -> MAD分组 -> 组末处理 -> DMA重装                     */
/* ========================================================================== */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) == 0U)
    {
        return;
    }

#if OUTPUT_TEST_ENABLE
    GPIOA->BRR = GPIO_PIN_12;
#endif

    ++g_signal_frames;
    g_signal_mad_ready = APP_SignalMadUpdate(adc_buffer[SIGNAL_ADC_INDEX]);

    if (g_signal_mad_ready != 0U)
    {
        APP_ProcessMadGroup();
    }

    /* 无论是否满组、是否冻结，均必须重装DMA。 */
    APP_AdcDmaFastRearm();

#if OUTPUT_TEST_ENABLE
    GPIOA->BSRR = GPIO_PIN_12;
#endif
}

/* ========================================================================== */
/* 07. 主函数与启动流程 */
/* ========================================================================== */

/* 仅保留随机周期启动流程；不改变外设配置与DMA重装时序。 */
/* 初始化顺序：HAL→时钟→GPIO→TIM3→ADC/DMA→启动TIM3；避免先触发再启动DMA。 */
int main(void)
{
    HAL_Init(); /* 初始化HAL底层与系统滴答时基。 */
    APP_SystemClockConfig(); /* 选择HSI 24MHz作为系统时钟。 */
    GPIO_Init(); /* 配置PA5/PA12输出、PB5输入。 */
    TMR3_OC_Init(); /* 配置TIM3 CH1发射PWM、CH2 ADC触发时序。 */
    APP_AdcInit(); /* 配置ADC单通道、DMA NORMAL和中断。 */
    APP_AdcStartDMA(); /* 先准备ADC和DMA，等待TIM3触发。 */
    /* 首周期立即生效；下一周期写入预装载，之后由更新中断持续更新。 */

    TIM3_QueuePeriod(Random_NextPeriod());
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = (uint32_t)~TIM_SR_UIF;

    TIM3_QueuePeriod(Random_NextPeriod());

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->DIER |= TIM_DIER_UIE;

    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    while (1)
    {
    }
}

/* ========================================================================== */
/* 08. 硬件初始化与DMA快速重装 */
/* ========================================================================== */

/* GPIO：PA5/PA12推挽输出；PB5上拉输入。PA12根据测试模式选择初始电平。 */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟，以配置PA5、PA6和PA12。 */
    __HAL_RCC_GPIOB_CLK_ENABLE(); /* 使能GPIOB时钟，供PB5输入使用。 */
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_12; /* 选择PA5和PA12一起配置为普通输出。 */
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP; /* 推挽输出，MCU可主动拉高或拉低。 */
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_5; /* 改为选择PB5。 */
    GPIO_InitStruct.Mode      = GPIO_MODE_INPUT; /* 将PB5配置为数字输入。 */
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); /* PA5默认输出高电平。 */
#if OUTPUT_TEST_ENABLE
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); /* 测试模式PA12默认为高电平，中断期间拉低。 */
#else
#if OUTPUT_LIGHT_ACTIVE_HIGH
    GPIOA->BRR = GPIO_PIN_12; /* 有光高有效时，上电遮光状态输出低。 */
#else
    GPIOA->BSRR = GPIO_PIN_12; /* 有光低有效时，上电遮光状态输出高。 */
#endif
#endif
}

/* TIM3：1MHz计数，CH1产生发射PWM，CH2的OC2REF经TRGO触发ADC。 */
static void TMR3_OC_Init(void)
{
    TIM_MasterConfigTypeDef master = {0};

    __HAL_RCC_TIM3_CLK_ENABLE(); /* 使能TIM3外设时钟。 */
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟，以配置PA5、PA6和PA12。 */
    GPIO_InitStruct.Pin       = GPIO_PIN_6; /* PA6作为TIM3 CH1 PWM输出引脚。 */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP; /* 配置为外设复用推挽输出。 */
    GPIO_InitStruct.Pull      = GPIO_NOPULL; /* PWM输出不使用内部上下拉。 */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3; /* 将PA6连接至TIM3的AF1复用功能。 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Tim3Handle.Instance = TIM3;
    Tim3Handle.Init.Period            = RANDOM_INITIAL_PERIOD - 1U; /* 周期ARR初值；计数从0开始所以减1。 */
    Tim3Handle.Init.Prescaler         = 24 - 1; /* 24MHz/(23+1)=1MHz，计数分辨率1us。 */
    Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; /* 定时器时钟不额外分频。 */
    Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP; /* 计数器递增计数。 */
    Tim3Handle.Init.RepetitionCounter = 1 - 1; /* 重复计数为0；保留原初始化参数。 */
    Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* ARR新值在更新事件生效，防止修改中途改变周期。 */
    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    sConfig.OCMode       = TIM_OCMODE_PWM1; /* CH1使用PWM1模式产生发射脉冲。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_LOW; /* CH1输出极性为低有效。 */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE; /* 保留原PWM快速模式设置。 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH; /* 保留互补输出极性字段，本程序不使用互补输出。 */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfig.Pulse        = RANDOM_INITIAL_PERIOD - RANDOM_PWM_OFF_US; /* CH1在周期末3us切换输出。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    sConfig.OCMode       = TIM_OCMODE_PWM2; /* CH2采用PWM2，内部OC2REF用于ADC触发。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH; /* CH2内部比较输出为高有效。 */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE; /* 保留原PWM快速模式设置。 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH; /* 保留互补输出极性字段，本程序不使用互补输出。 */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfig.Pulse        = RANDOM_INITIAL_PERIOD - RANDOM_ADC_BEFORE_END_US; /* CH2在周期结束前2us产生OC2REF上升沿。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; /* 开启两路CCR预装载，同步在       更新事件生效。 */

    master.MasterOutputTrigger = TIM_TRGO_OC2REF; /* 将CH2内部OC2REF作为TIM3 TRGO触发源。 */
    master.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE; /* 不启用定时器主从同步。 */
    if (HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &master) != HAL_OK)
    {
        APP_ErrorHandler();
    }
}

/* ADC/DMA：ADC1的通道0，单通道12位连续转换；DMA1_CH1 NORMAL按18点搬运。 */
static void APP_AdcInit(void)
{
    ADC_ChannelConfTypeDef channel_config = {0}; /* 初始化ADC通道配置结构。 */
    __HAL_RCC_ADC_FORCE_RESET(); /* 复位ADC外设寄存器。 */
    __HAL_RCC_ADC_RELEASE_RESET(); /* 释放ADC外设复位。 */
    __HAL_RCC_ADC_CLK_ENABLE(); /* 使能ADC外设时钟。 */
    AdcHandle.Instance = ADC1;
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1; /* ADC同步PCLK不分频，当前名义24MHz。 */
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B; /* ADC 12位分辨率，输出0~4095。 */
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT; /* 采样结果右对齐到16位数据中。 */
    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_BACKWARD; /* 设置扫描方向；只选通道0，实际单通道。 */
    AdcHandle.Init.LowPowerAutoWait      = DISABLE; /* 关闭自动等待模式。 */
    AdcHandle.Init.ContinuousConvMode    = ENABLE; /* 接受一次外部触发后连续转换。 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE; /* 关闭不连续扫描。 */
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO; /* 选择TIM3 TRGO作为ADC触发源。 */
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING; /* 仅响应TRGO上升沿。 */
    AdcHandle.Init.DMAContinuousRequests = DISABLE; /* 保留原DMA连续请求关闭设置；与连续转换的配合需实测。 */
    AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN; /* ADC溢出时新数据覆盖旧数据。 */
    AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_3CYCLES_5; /* 采样保持时间3.5个ADC时钟，不含转换阶段。 */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    channel_config.Channel = ADC_CHANNEL_0; /* 只采集ADC通道0。 */
    channel_config.Rank    = ADC_RANK_CHANNEL_NUMBER; /* 转换排序使用硬件通道编号。 */
    if (HAL_ADC_ConfigChannel(&AdcHandle, &channel_config) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    HAL_SYSCFG_DMA_Req(0); /* 沿用原代码的DMA请求映射选择，具体参数意义依赖HAL。 */
    __HAL_RCC_DMA_CLK_ENABLE(); /* 使能DMA时钟。 */

    hdma_adc.Instance = DMA1_Channel1; /* 使用DMA1 Channel1搬运ADC结果。 */
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY; /* 从ADC数据寄存器搬运到内存。 */
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE; /* ADC数据寄存器地址保持不变。 */
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE; /* 内存地址自增，逐个写入adc_buffer。 */
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* ADC外设侧每次读取16位。 */
    hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD; /* 内存侧每次写入16位uint16_t元素。 */
    hdma_adc.Init.Mode                = DMA_NORMAL; /* NORMAL模式，搬满18点后停止并触发TC中断。 */
    hdma_adc.Init.Priority            = DMA_PRIORITY_MEDIUM; /* DMA总线仲裁优先级设为中等。 */

    HAL_DMA_Init(&hdma_adc); /* 将DMA参数配置到硬件。 */
    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc); /* 建立ADC句柄与DMA句柄关联。 */

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0); /* DMA通道1的NVIC中断优先级。 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* 允许DMA搬运完成中断。 */
}

/* 启动首帧DMA并准备ADC外部触发，关闭不需要的半传输中断。 */
static void APP_AdcStartDMA(void)
{
    if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)adc_buffer, BUFFER_SIZE) != HAL_OK)
    {
        APP_ErrorHandler();
    }
    __HAL_DMA_DISABLE_IT(&hdma_adc, DMA_IT_HT); /* 关闭半传输中断，仅使用全传输完成中断。 */
}

/* DMA TC后快速重装，暂不改变原来寄存器顺序；需留意连续ADC转换时DMA重启时序。 */
static void APP_AdcDmaFastRearm(void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_EN; /* 先停止DMA通道，以允许改写CNDTR。 */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* 临时关ADC对DMA的请求。 */
    DMA1->IFCR = DMA_IFCR_CGIF1; /* 清DMA Channel1所有相关中断标志。 */
    DMA1_Channel1->CNDTR = BUFFER_SIZE; /* 重新装载18点传输长度。 */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* 重新打开DMA通道。 */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN; /* 重新允许ADC DMA请求。 */
    ADC1->ISR = ADC_ISR_OVR; /* 清ADC溢出标志。 */
    ADC1->CR |= ADC_CR_ADEN; /* 保持ADC使能，沿用现有快速重装策略。 */
    ADC1->CR |= ADC_CR_ADSTART; /* 设置ADC启动，等待下一次TRGO触发。 */
}

/* 时钟：使用内部24MHz HSI，SYSCLK/HCLK/PCLK1均为24MHz，不使用HSE/LSI。 */
static void APP_SystemClockConfig(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; /* 准备振荡器参数结构。 */
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; /* 准备系统时钟分频参数结构。 */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* 声明将配置HSI、HSE、LSI三个振荡源。 */
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON; /* 打开内部高速HSI。 */
#if defined(RCC_HSIDIV_SUPPORT)
    RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1; /* HSI不分频。 */
#endif
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz; /* 选择HSI 24MHz校准档。 */
    RCC_OscInitStruct.HSEState            = RCC_HSE_OFF; /* 关闭外部高速晶振。 */
    RCC_OscInitStruct.LSIState            = RCC_LSI_OFF; /* 关闭内部低速振荡器。 */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* 配置SYSCLK、HCLK、PCLK1时钟域。 */
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI; /* 系统时钟直接使用HSI。 */
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1; /* AHB不分频，HCLK=24MHz。 */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; /* APB1不分频，PCLK1=24MHz。 */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        APP_ErrorHandler();
    }
}

void APP_ErrorHandler(void)
{
    while (1)
    {
    }
}

