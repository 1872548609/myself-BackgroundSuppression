/*
 * PY32F003 背景抑制：25点DMA + ADC[7]减固定基线 + 5帧MAD + 干扰冻结
 * 每帧读取ADC[7]；MAD基于原始值计算（减固定基线后MAD数值不变）。
 * 检测信号=ADC[7]-SIGNAL_ADC_BASELINE，有符号IIR -> 截尾平均 -> 10码死区 -> 输出判断。
 * 保持TIM3、ADC/DMA初始化与快速重装寄存器操作不变。
 */
#include "main.h"
#include <stdint.h>
/* ============================= 采样及发射配置 ============================= */
#define BUFFER_SIZE                   25U  /* DMA每帧采集25个ADC点，下标0~24。 */
#define TRANSMIT_RANDOM_ENABLE         1U  /* 1=候选表随机周期；0=固定周期。 */
#define FIXED_TRANSMIT_PERIOD        110U  /* 固定周期模式的TIM3周期，单位us。 */
#define RANDOM_PERIOD_COUNT           10U  /* 随机周期候选表元素数量。 */
#define RANDOM_INITIAL_PERIOD        100U  /* TIM3初始化时的占位周期，单位us。 */
#define RANDOM_PWM_OFF_US              3U  /* CH1在每个周期末尾预留3us的PWM输出段。 */
#define RANDOM_ADC_BEFORE_END_US       7U  /* CH2在周期结束前7us产生ADC触发沿。 */
#if TRANSMIT_RANDOM_ENABLE
#define TIM3_INITIAL_PERIOD          RANDOM_INITIAL_PERIOD /* 随机模式采用随机初始周期。 */
#else
#define TIM3_INITIAL_PERIOD          FIXED_TRANSMIT_PERIOD /* 固定模式采用固定周期。 */
#endif

/* ============================ 单点与MAD参数 ============================= */
#define SIGNAL_ADC_INDEX                7U  /* 单点检测位置：直接使用adc_buffer[7]。 */
#define SIGNAL_ADC_BASELINE          2050U  /* ADC[7]直流基线；送往IIR前减去该值，结果允许为负。 */
#define SIGNAL_MAD_WINDOW_SIZE          5U  /* 连续5帧ADC[7]构成滑动窗口；改此宏可调整MAD点数。 */
/* MAD为窗口内各采样值相对窗口均值的平均绝对偏差，不使用固定2030基准。 */
#define SIGNAL_MAD_WEAK_ENTER          10U  /* MAD >= 30进入弱干扰，使用弱光IIR。 */
#define SIGNAL_MAD_WEAK_RELEASE         7U  /* MAD <= 20退出弱干扰，恢复正常IIR。 */
#define SIGNAL_MAD_STRONG_ENTER        20U  /* MAD > 100视为强干扰候选；连续确认后冻结。 */
#define SIGNAL_MAD_STRONG_RELEASE      15U  /* MAD <= 85累计恢复确认；与进入阈值构成迟滞。 */
#define SIGNAL_MAD_STRONG_CONFIRM       5U  /* 连续5帧满足强干扰阈值才进入冻结。 */
#define SIGNAL_MAD_RECOVER_CONFIRM      5U  /* 冻结后连续5帧满足解除阈值才恢复检测。 */

/* ============================= 滤波和输出参数 =========================== */
#define SIGNAL_FILTER_Q_BITS           8U  /* IIR定点小数精度，内部状态=ADC码值*256。 */
#define SIGNAL_FILTER_NORMAL_SHIFT     1U  /* 正常状态IIR系数1/(2^1)=1/2，每个DMA帧更新。 */
#define SIGNAL_FILTER_WEAK_SHIFT       3U  /* 弱干扰IIR系数1/(2^3)=1/8；比正常模式更平滑。 */
#define SIGNAL_TRIMMED_MEAN_ENABLE     1U  /* 1=开启8帧IIR截尾平均；0=直接使用IIR结果。 */
#define SIGNAL_TRIM_WINDOW_SIZE        8U  /* 截尾历史数量；当前固定8组，去掉两端各2组。 */
#define OUTPUT_DEADBAND_ENABLE         1U  /* 1=开启滤波后输出死区；0=跳过死区，直接使用截尾平均结果。 */
#define OUTPUT_DEADBAND                10U  /* 输出死区宽度：相对上次死区输出，变化绝对值<=10时保持原值。 */
/* 输出阈值单位为ADC[7]-SIGNAL_ADC_BASELINE的差值（可为负），需实测标定。 */
#define OUTPUT_OFF_THRESHOLD           10  /* 有光状态下，滤波ADC值低于该值连续确认后切换为遮光；需实测重标定。 */
#define OUTPUT_HYSTERESIS              5  /* 遮光状态下，ON阈值=OFF阈值+迟滞量。 */
#define OUTPUT_LIGHT_CONFIRM_COUNT     5U /* 连续5帧后级滤波值达到ON阈值才确认有光。 */
#define OUTPUT_DARK_CONFIRM_COUNT      5U /* 连续5帧后级滤波值低于OFF阈值才确认遮光。 */
#define OUTPUT_LIGHT_ACTIVE_HIGH       1U /* 1=有光输出高电平；0=有光输出低电平。 */
#define OUTPUT_TEST_ENABLE             0U /* 1=PA12测DMA中断耗时；0=PA12输出有光状态。 */

/* 编译期配置检查，避免MAD窗口越界或确认计数无效。 */
#if (SIGNAL_ADC_BASELINE > 4095U)
#error "SIGNAL_ADC_BASELINE must be within 0..4095"
#endif
#if (SIGNAL_ADC_INDEX >= BUFFER_SIZE)
#error "SIGNAL_ADC_INDEX exceeds BUFFER_SIZE"
#endif
#if (SIGNAL_MAD_WINDOW_SIZE < 2U) || (SIGNAL_MAD_WINDOW_SIZE > 255U)
#error "SIGNAL_MAD_WINDOW_SIZE must be within 2..255"
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
#if (OUTPUT_DEADBAND > 4095U)
#error "OUTPUT_DEADBAND must be within 0..4095"
#endif

ADC_HandleTypeDef AdcHandle;
DMA_HandleTypeDef hdma_adc;
TIM_HandleTypeDef Tim3Handle;
TIM_OC_InitTypeDef sConfig;
GPIO_InitTypeDef GPIO_InitStruct;
/* ==================== Keil Watch 调试变量 ====================
 * DMA 每完成25个ADC采样，才会执行一次本帧算法。
 * volatile 全局量可以直接加到 Keil Watch；数值随中断不断刷新。
 * 若需观察同一帧的对应关系，建议在 DMA1_Channel1_IRQHandler 内设置断点。
 */
uint16_t adc_buffer[BUFFER_SIZE];            /* 25点原始ADC数据，索引0~24；下一次DMA会覆盖，12位取值0~4095。 */

/* A. 直接采样与MAD强/弱干扰状态，可在Keil Watch观察。 */
volatile uint16_t g_signal_raw_adc = 0U;         /* 最新DMA帧的原始ADC[7]（0..4095）。 */
volatile int32_t  g_signal_offset_adc = 0;       /* 最新信号=原始ADC[7]-SIGNAL_ADC_BASELINE，可为负数；IIR输入。 */
volatile uint32_t g_signal_frames = 0U;          /* 已处理DMA帧总数。 */
volatile uint16_t g_signal_mad_mean = 0U;        /* 最近N帧单点ADC值的窗口算术均值。 */
volatile uint16_t g_signal_mad = 0U;             /* 最近N帧ADC[7]相对均值的平均绝对偏差。 */
volatile uint8_t  g_signal_mad_ready = 0U;       /* 1=已收齐首次MAD窗口，0=尚在启动阶段。 */
volatile uint8_t  g_signal_noise_level = 0U;     /* 0=正常IIR、1=弱干扰IIR、2=强干扰冻结。 */
volatile uint8_t  g_signal_weak_active = 0U;     /* 弱干扰迟滞状态（0/1），冻结时仍更新。 */
volatile uint8_t  g_signal_strong_active = 0U;   /* 强干扰冻结状态（0/1），确认5次才切换。 */
volatile uint8_t  g_signal_strong_count = 0U;    /* 连续满足强干扰进入条件的帧数。 */
volatile uint8_t  g_signal_recover_count = 0U;   /* 冻结后连续满足退出条件的帧数。 */
volatile uint32_t g_signal_freeze_frames = 0U;  /* 冻结期间未更新IIR和输出的累计帧数。 */
volatile uint32_t g_signal_freeze_events = 0U;  /* 累计进入强干扰冻结的次数。 */
volatile int32_t  g_match_filtered_integral = 0; /* 沿用Watch变量名，实际为ADC[7]减基线后经IIR滤波的有符号值。 */
static uint16_t g_signal_mad_history[SIGNAL_MAD_WINDOW_SIZE]; /* 最近N帧单点ADC值环形缓存。 */
static uint8_t g_signal_mad_write = 0U;         /* 下次覆盖的MAD历史位置。 */
static uint8_t g_signal_mad_count = 0U;         /* 当前已有的MAD历史点数。 */

/* B. 截尾与输出：非冻结帧IIR -> 8帧截尾 -> 死区 -> 输出迟滞判断。 */
volatile int32_t  g_output_trimmed_integral = 0; /* 最新8帧截尾平均后的有符号值，死区处理的输入。 */
volatile int32_t  g_output_deadband_value = 0;   /* 最新死区输出值：偏差<=OUTPUT_DEADBAND时保持上次结果；用于输出迟滞判断。 */
static uint8_t g_output_deadband_initialized = 0U; /* 死区首次处理标记：首次直接采用当前滤波值。 */
volatile uint8_t  g_output_ready = 0U;          /* 0=尚未收齐首批8个IIR结果；1=已有可用输出。 */
volatile uint8_t  g_output_light_state = 0U;    /* 当前逻辑光状态：0=遮光，1=有光；OUTPUT_TEST_ENABLE=1时不会映射到PA12。 */
volatile uint8_t  g_output_light_confirm = 0U;  /* 遮光状态下，连续满足ON门槛的后级分组结果数；达到设定次数切有光。 */
volatile uint8_t  g_output_dark_confirm = 0U;   /* 有光状态下，连续满足OFF门槛的后级分组结果数；达到设定次数切遮光。 */
volatile uint32_t g_output_state_changes = 0U; /* 上电至今逻辑光状态真正发生0/1切换的累计次数。 */

/* C. 后级滤波内部状态；强干扰冻结时停止更新。 */
static int32_t g_signal_filter_q8 = 0;          /* IIR的Q8定点状态，约等于滤波ADC码值×256。 */
#if SIGNAL_TRIMMED_MEAN_ENABLE
static int32_t g_output_trim_history[SIGNAL_TRIM_WINDOW_SIZE] = {0}; /* 最近8个非冻结帧的IIR历史，环形缓冲区。 */
static uint8_t g_output_trim_write = 0U;        /* 下一次将写入的历史数组位置，循环0~7。 */
static uint8_t g_output_trim_count = 0U;        /* 已收集的IIR结果数，上限8；满8帧前不做截尾输出。 */
#endif

static void APP_SystemClockConfig(void);
static void GPIO_Init(void);
static void TMR3_OC_Init(void);
static void APP_AdcInit(void);
static void APP_AdcStartDMA(void);
static void APP_AdcDmaFastRearm(void);
#if TRANSMIT_RANDOM_ENABLE
static const uint16_t random_period_table[RANDOM_PERIOD_COUNT] = {
    100U, 100U, 100U, 100U, 100U, 100U, 100U, 100U, 100U, 100U
}; /* 当前10个候选值均为100us，虽运行随机抽取但周期实际固定。 */
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
#endif /* TRANSMIT_RANDOM_ENABLE */
/* 最近SIGNAL_MAD_WINDOW_SIZE个原始ADC[7]的均值中心MAD。
 * 所有点减去同一基线不改变MAD，故无需对MAD窗口再作减法。 */
static void APP_SignalMadUpdate(uint16_t adc)
{
    uint32_t sum = 0U, deviations = 0U, i;
    uint32_t mean;

    g_signal_mad_history[g_signal_mad_write] = adc;
    if (++g_signal_mad_write >= SIGNAL_MAD_WINDOW_SIZE) g_signal_mad_write = 0U;
    if (g_signal_mad_count < SIGNAL_MAD_WINDOW_SIZE) ++g_signal_mad_count;
    if (g_signal_mad_count < SIGNAL_MAD_WINDOW_SIZE)
    {
        g_signal_mad_ready = 0U;
        return;
    }

    g_signal_mad_ready = 1U;
    for (i = 0U; i < SIGNAL_MAD_WINDOW_SIZE; ++i) sum += g_signal_mad_history[i];
    mean = sum / SIGNAL_MAD_WINDOW_SIZE;
    for (i = 0U; i < SIGNAL_MAD_WINDOW_SIZE; ++i)
    {
        uint32_t v = g_signal_mad_history[i];
        deviations += (v >= mean) ? (v - mean) : (mean - v);
    }
    g_signal_mad_mean = (uint16_t)mean;
    g_signal_mad = (uint16_t)(deviations / SIGNAL_MAD_WINDOW_SIZE);
}

/* 强干扰优先于弱干扰。进入冻结的第5帧立即冻结；解除的第5帧仍保持冻结。
 * 返回1表示本帧不再更新后级。MAD始终更新，用于监测恢复。
 */
static uint8_t APP_SignalInterferenceUpdate(void)
{
    uint8_t was_frozen = g_signal_strong_active;
    if (!g_signal_mad_ready)
    {
        g_signal_noise_level = 0U;
        return 0U;
    }

    /* 弱干扰双阈值迟滞，切换IIR系数但不中断检测。 */
    if (!g_signal_weak_active)
    {
        if (g_signal_mad >= SIGNAL_MAD_WEAK_ENTER) g_signal_weak_active = 1U;
    }
    else if (g_signal_mad <= SIGNAL_MAD_WEAK_RELEASE)
    {
        g_signal_weak_active = 0U;
    }

    if (!g_signal_strong_active)
    {
        g_signal_recover_count = 0U;
        if (g_signal_mad > SIGNAL_MAD_STRONG_ENTER)
        {
            if (g_signal_strong_count < SIGNAL_MAD_STRONG_CONFIRM)
                ++g_signal_strong_count;
            if (g_signal_strong_count >= SIGNAL_MAD_STRONG_CONFIRM)
            {
                g_signal_strong_active = 1U;
                g_signal_strong_count = 0U;
                g_output_light_confirm = 0U;
                g_output_dark_confirm = 0U;
                ++g_signal_freeze_events;
            }
        }
        else g_signal_strong_count = 0U;
    }
    else
    {
        g_signal_strong_count = 0U;
        if (g_signal_mad <= SIGNAL_MAD_STRONG_RELEASE)
        {
            if (g_signal_recover_count < SIGNAL_MAD_RECOVER_CONFIRM)
                ++g_signal_recover_count;
            if (g_signal_recover_count >= SIGNAL_MAD_RECOVER_CONFIRM)
            {
                g_signal_strong_active = 0U;
                g_signal_recover_count = 0U;
                g_output_light_confirm = 0U;
                g_output_dark_confirm = 0U;
            }
        }
        else g_signal_recover_count = 0U;
    }

    g_signal_noise_level = g_signal_strong_active ? 2U : (g_signal_weak_active ? 1U : 0U);
    return (uint8_t)(was_frozen || g_signal_strong_active);
}

/* 根据弱干扰状态选择1/2或1/8 IIR系数；输入为减SIGNAL_ADC_BASELINE后的有符号值。 */
static void APP_SignalFilterUpdate(int32_t sample)
{
    uint32_t shift = g_signal_weak_active ? SIGNAL_FILTER_WEAK_SHIFT : SIGNAL_FILTER_NORMAL_SHIFT;
    int32_t target_q8 = sample * (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
    g_signal_filter_q8 += (target_q8 - g_signal_filter_q8) / (int32_t)(1U << shift);
    g_match_filtered_integral = g_signal_filter_q8 / (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
}
/* 8帧IIR结果去掉最大2帧、最小2帧，再求中间4帧平均。
 * 排序仅对8个滤波值操作，不会修改25点原始DMA数据。
 */
static uint8_t APP_OutputTrimUpdate(int32_t value)
{
#if SIGNAL_TRIMMED_MEAN_ENABLE
    int32_t ordered[SIGNAL_TRIM_WINDOW_SIZE];
    uint32_t i, j;
    int32_t sum = 0;
    g_output_trim_history[g_output_trim_write] = value;
    g_output_trim_write = (uint8_t)((g_output_trim_write + 1U) % SIGNAL_TRIM_WINDOW_SIZE);
    if (g_output_trim_count < SIGNAL_TRIM_WINDOW_SIZE) ++g_output_trim_count;
    if (g_output_trim_count < SIGNAL_TRIM_WINDOW_SIZE) return 0U;
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
    for (i = 2U; i < 6U; ++i) sum += ordered[i];
    g_output_trimmed_integral = sum / 4;
#else
    g_output_trimmed_integral = value;
#endif
    g_output_ready = 1U;
    return 1U;
}
/* 滤波完成后的输出死区：比较当前滤波值与上次死区保持值。
 * 首次直接使用滤波值，之后变化幅度<=OUTPUT_DEADBAND时不更新。
 * 使用int32_t有符号差值，正确处理ADC减基线后可能出现的负数。
 * 强干扰冻结时不调用本函数，因此死区保持值也一起冻结。
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

static void APP_OutputSetLight(uint8_t light)
{
    g_output_light_state = light;
#if !OUTPUT_TEST_ENABLE
#if OUTPUT_LIGHT_ACTIVE_HIGH
    if (light) GPIOA->BSRR = GPIO_PIN_12;
    else       GPIOA->BRR  = GPIO_PIN_12;
#else
    if (light) GPIOA->BRR  = GPIO_PIN_12;
    else       GPIOA->BSRR = GPIO_PIN_12;
#endif
#endif
}
static void APP_OutputUpdateDecision(int32_t value)
{
    int32_t off = OUTPUT_OFF_THRESHOLD;
    int32_t hyst = OUTPUT_HYSTERESIS;
    if (g_output_light_state == 0U)
    {
        g_output_dark_confirm = 0U;
        if (value >= off + hyst)
        {
            if (++g_output_light_confirm >= OUTPUT_LIGHT_CONFIRM_COUNT)
            {
                g_output_light_confirm = 0U;
                APP_OutputSetLight(1U);
                ++g_output_state_changes;
            }
        }
        else g_output_light_confirm = 0U;
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
                ++g_output_state_changes;
            }
        }
        else g_output_dark_confirm = 0U;
    }
}
/* DMA每完成25点直接读取索引7、更新MAD并判定是否冻结。
 * 无强干扰时每帧更新IIR、截尾平均及原输出判定；始终完成DMA快速重装。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {
#if OUTPUT_TEST_ENABLE
        GPIOA->BRR = GPIO_PIN_12;
#endif
        uint16_t sample = adc_buffer[SIGNAL_ADC_INDEX];
        g_signal_raw_adc = sample;
        g_signal_offset_adc = (int32_t)sample - (int32_t)SIGNAL_ADC_BASELINE;
        ++g_signal_frames;
        APP_SignalMadUpdate(sample);

        if (APP_SignalInterferenceUpdate() != 0U)
        {
            ++g_signal_freeze_frames;
        }
        else
        {
            APP_SignalFilterUpdate(g_signal_offset_adc);
            if (APP_OutputTrimUpdate(g_match_filtered_integral) != 0U)
            {
                /* 8帧截尾平均 -> 10码死区 -> 原阈值迟滞/连续确认。 */
                int32_t decision_value = APP_OutputApplyDeadband(g_output_trimmed_integral);
                APP_OutputUpdateDecision(decision_value);
            }
        }

        APP_AdcDmaFastRearm();
#if OUTPUT_TEST_ENABLE
        GPIOA->BSRR = GPIO_PIN_12;
#endif
    }
}

/* 原硬件配置保留，不改变定时器、ADC触发和DMA重装时序。 */
/* 初始化顺序：HAL→时钟→GPIO→TIM3→ADC/DMA→启动TIM3；避免先触发再启动DMA。 */
int main(void)
{
    HAL_Init(); /* 初始化HAL底层与系统滴答时基。 */
    APP_SystemClockConfig(); /* 选择HSI 24MHz作为系统时钟。 */
    GPIO_Init(); /* 配置PA5/PA12输出、PB5输入。 */
    TMR3_OC_Init(); /* 配置TIM3 CH1发射PWM、CH2 ADC触发时序。 */
    APP_AdcInit(); /* 配置ADC单通道、DMA NORMAL和中断。 */
    APP_AdcStartDMA(); /* 先准备ADC和DMA，等待TIM3触发。 */
#if TRANSMIT_RANDOM_ENABLE
    TIM3_QueuePeriod(Random_NextPeriod()); /* 把抽取的周期写入ARR/CCR预装载寄存器。 */
    TIM3->EGR = TIM_EGR_UG; /* 软件更新使首周期配置立即生效。 */
    TIM3->SR = (uint32_t)~TIM_SR_UIF; /* 清除软件更新产生的UIF标志。 */
    TIM3_QueuePeriod(Random_NextPeriod()); /* 把抽取的周期写入ARR/CCR预装载寄存器。 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0); /* 设置TIM3更新中断优先级。 */
    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn); /* 清除TIM3待处理中断。 */
    HAL_NVIC_EnableIRQ(TIM3_IRQn); /* 启用随机周期的TIM3更新中断。 */
    TIM3->DIER |= TIM_DIER_UIE; /* 允许TIM3更新事件产生中断。 */
#else
    TIM3->EGR = TIM_EGR_UG; /* 软件更新使首周期配置立即生效。 */
    TIM3->SR = (uint32_t)~TIM_SR_UIF; /* 清除软件更新产生的UIF标志。 */
    TIM3->DIER &= ~TIM_DIER_UIE; /* 固定周期模式不需要TIM3更新中断。 */
#endif
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
          APP_ErrorHandler();
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
          APP_ErrorHandler();
    while (1)
    {
    }
}
/* GPIO：PA5/PA12推挽输出；PB5上拉输入。PA12根据测试模式选择初始电平。 */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟，以配置PA5、PA6和PA12。 */
    __HAL_RCC_GPIOB_CLK_ENABLE(); /* 使能GPIOB时钟，供PB5输入使用。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12; /* 选择PA5和PA12一起配置为普通输出。 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; /* 推挽输出，MCU可主动拉高或拉低。 */
    GPIO_InitStruct.Pull = GPIO_PULLUP; /* 保留原内部上拉配置。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* 选择较高GPIO边沿速度。 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); /* 将上述设置应用于GPIOA。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5; /* 改为选择PB5。 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /* 将PB5配置为数字输入。 */
    GPIO_InitStruct.Pull = GPIO_PULLUP; /* 保留原内部上拉配置。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* 选择较高GPIO边沿速度。 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); /* 将PB5输入及上拉配置写入硬件。 */
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); /* PA5默认输出高电平。 */
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
    __HAL_RCC_TIM3_CLK_ENABLE(); /* 使能TIM3外设时钟。 */
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 使能GPIOA时钟，以配置PA5、PA6和PA12。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6; /* PA6作为TIM3 CH1 PWM输出引脚。 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; /* 配置为外设复用推挽输出。 */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* PWM输出不使用内部上下拉。 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; /* 选择较高GPIO边沿速度。 */
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3; /* 将PA6连接至TIM3的AF1复用功能。 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); /* 将上述设置应用于GPIOA。 */
    Tim3Handle.Instance = TIM3; /* 选择TIM3实例。 */
    Tim3Handle.Init.Period            = TIM3_INITIAL_PERIOD - 1U; /* 周期ARR初值；计数从0开始所以减1。 */
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
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET; /* 保留互补通道空闲状态配置。 */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET; /* 输出空闲状态为复位电平。 */
    sConfig.Pulse = TIM3_INITIAL_PERIOD - RANDOM_PWM_OFF_US; /* CH1在周期末3us切换输出。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    {
          APP_ErrorHandler();
    }
     sConfig.OCMode       = TIM_OCMODE_PWM2; /* CH2采用PWM2，内部OC2REF用于ADC触发。 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH; /* CH2内部比较输出为高有效。 */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE; /* 保留原PWM快速模式设置。 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH; /* 保留互补输出极性字段，本程序不使用互补输出。 */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET; /* 保留互补通道空闲状态配置。 */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET; /* 输出空闲状态为复位电平。 */
    sConfig.Pulse = TIM3_INITIAL_PERIOD - RANDOM_ADC_BEFORE_END_US; /* CH2在周期结束前7us产生OC2REF上升沿。 */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    {
          APP_ErrorHandler();
    }
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; /* 开启两路CCR预装载，同步在更新事件生效。 */
     TIM_MasterConfigTypeDef master = {0}; /* 准备TIM3主触发配置结构。 */
     master.MasterOutputTrigger = TIM_TRGO_OC2REF; /* 将CH2内部OC2REF作为TIM3 TRGO触发源。 */
     master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; /* 不启用定时器主从同步。 */
     if (HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &master) != HAL_OK)
     {
         APP_ErrorHandler();
     }
}
/* ADC/DMA：ADC0单通道12位连续转换；DMA1_CH1 NORMAL按25点搬运。 */
static void APP_AdcInit(void)
{
    ADC_ChannelConfTypeDef sConfig = {0}; /* 初始化ADC通道配置结构。 */
    __HAL_RCC_ADC_FORCE_RESET(); /* 复位ADC外设寄存器。 */
    __HAL_RCC_ADC_RELEASE_RESET(); /* 释放ADC外设复位。 */
    __HAL_RCC_ADC_CLK_ENABLE(); /* 使能ADC外设时钟。 */
    AdcHandle.Instance = ADC1; /* 使用ADC1实例。 */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
    {
          APP_ErrorHandler();
    }
    AdcHandle.Instance = ADC1; /* 使用ADC1实例。 */
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1; /* ADC同步PCLK不分频，当前名义24MHz。 */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B; /* ADC 12位分辨率，输出0~4095。 */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT; /* 采样结果右对齐到16位数据中。 */
    AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD; /* 设置扫描方向；只选通道0，实际单通道。 */
    AdcHandle.Init.LowPowerAutoWait = DISABLE; /* 关闭自动等待模式。 */
    AdcHandle.Init.ContinuousConvMode = ENABLE; /* 接受一次外部触发后连续转换。 */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE; /* 关闭不连续扫描。 */
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; /* 选择TIM3 TRGO作为ADC触发源。 */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; /* 仅响应TRGO上升沿。 */
    AdcHandle.Init.DMAContinuousRequests = DISABLE; /* 保留原DMA连续请求关闭设置；与连续转换的配合需实测。 */
    AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN; /* ADC溢出时新数据覆盖旧数据。 */
    AdcHandle.Init.SamplingTimeCommon=ADC_SAMPLETIME_3CYCLES_5; /* 采样保持时间3.5个ADC时钟，不含转换阶段。 */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
          APP_ErrorHandler();
    }
    sConfig.Channel = ADC_CHANNEL_0; /* 只采集ADC通道0。 */
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER; /* 转换排序使用硬件通道编号。 */
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
    {
          APP_ErrorHandler();
    }
    HAL_SYSCFG_DMA_Req(0); /* 沿用原代码的DMA请求映射选择，具体参数意义依赖HAL。 */
    __HAL_RCC_DMA_CLK_ENABLE(); /* 使能DMA时钟。 */
    hdma_adc.Instance = DMA1_Channel1; /* 使用DMA1 Channel1搬运ADC结果。 */
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY; /* 从ADC数据寄存器搬运到内存。 */
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE; /* ADC数据寄存器地址保持不变。 */
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE; /* 内存地址自增，逐个写入adc_buffer。 */
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* ADC外设侧每次读取16位。 */
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD; /* 内存侧每次写入16位uint16_t元素。 */
    hdma_adc.Init.Mode = DMA_NORMAL; /* NORMAL模式，搬满25点后停止并触发TC中断。 */
    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM; /* DMA总线仲裁优先级设为中等。 */
    HAL_DMA_Init(&hdma_adc); /* 将DMA参数配置到硬件。 */
    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc); /* 建立ADC句柄与DMA句柄关联。 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0); /* DMA通道1的NVIC中断优先级。 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* 允许DMA搬运完成中断。 */
}
/* 启动首帧DMA并准备ADC外部触发，关闭不需要的半传输中断。 */
static void APP_AdcStartDMA(void)
{
    if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)
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
    DMA1_Channel1->CNDTR = BUFFER_SIZE; /* 重新装载25点传输长度。 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* 声明将配置HSI、HSE、LSI三个振荡源。 */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON; /* 打开内部高速HSI。 */
#if defined(RCC_HSIDIV_SUPPORT)
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; /* HSI不分频。 */
#endif
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz; /* 选择HSI 24MHz校准档。 */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF; /* 关闭外部高速晶振。 */
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF; /* 关闭内部低速振荡器。 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* 配置SYSCLK、HCLK、PCLK1时钟域。 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* 系统时钟直接使用HSI。 */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* AHB不分频，HCLK=24MHz。 */
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
