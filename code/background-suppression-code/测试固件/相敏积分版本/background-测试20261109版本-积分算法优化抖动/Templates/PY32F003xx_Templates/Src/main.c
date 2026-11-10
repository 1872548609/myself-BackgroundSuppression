/*
 * PY32F003 背景抑制 - 25点DMA / 10点正负窗口相关积分 / 有光干扰冻结
 * 默认采样索引 6~10: +1; 11~15: -1; 其余不参与相关积分（可由宏修改）。
 * 默认积分 = sum(ADC[6..10]) - sum(ADC[11..15])。
 * 每帧：背景偏差连续确认 -> 有光干扰冻结输出 -> 正负积分 -> 固定IIR -> 8帧截尾 -> 输出迟滞/确认。
 * 保留原TIM3、ADC、DMA及PA12测试时序和寄存器重装实现。
 */
#include "main.h"
#include <stdint.h>
/* ============================= 采样及发射配置 ============================= */
#define BUFFER_SIZE                   25U  /* DMA每帧采集25个ADC点，下标0~24。 */
#define TRANSMIT_RANDOM_ENABLE        1U  /* 1=候选表随机周期；0=固定周期。 */
#define FIXED_TRANSMIT_PERIOD        110U  /* 固定周期模式的TIM3周期，单位us。 */
#define RANDOM_PERIOD_COUNT          10U  /* 随机周期候选表元素数量。 */
#define RANDOM_INITIAL_PERIOD        100U  /* TIM3初始化时的占位周期，单位us。 */
#define RANDOM_PWM_OFF_US              3U  /* CH1在每个周期末尾预留3us的PWM输出段。 */
#define RANDOM_ADC_BEFORE_END_US       7U  /* CH2在周期结束前7us产生ADC触发沿。 */
#if TRANSMIT_RANDOM_ENABLE
#define TIM3_INITIAL_PERIOD          RANDOM_INITIAL_PERIOD /* 随机模式采用随机初始周期。 */
#else
#define TIM3_INITIAL_PERIOD          FIXED_TRANSMIT_PERIOD /* 固定模式采用固定周期。 */
#endif

/* ========================== 有光干扰冻结参数 ============================ */
#define LIGHT_INTERFERENCE_BASELINE  2030U /* 前6个背景点的固定参考值，仅用于计算背景偏差指标。 */
#define LIGHT_INTERFERENCE_THRESHOLD 150U  /* 有光干扰进入阈值：连续达到该值才进入冻结。 */
#define LIGHT_INTERFERENCE_RELEASE_THRESHOLD 120U /* 冻结解除阈值，低于进入阈值形成迟滞。 */
#define LIGHT_INTERFERENCE_CONFIRM_COUNT 5U /* 连续5帧达到进入阈值，确认有光干扰并冻结输出。 */
#define LIGHT_INTERFERENCE_RELEASE_COUNT 5U /* 连续5帧低于解除阈值，解除冻结；恢复确认帧仍冻结。 */

/* =========================== 相关积分窗口参数 ===========================
 * 只需修改以下4个宏即可调整原始ADC数组的正、负积分范围（起止索引均包含）。
 * 正窗口的每个ADC值系数+1，负窗口的每个ADC值系数-1。
 * 两段必须不重叠、点数相等，且索引都在0~BUFFER_SIZE-1。
 * 两段点数相等时，固定直流基线项自动抵消，无需逐点减2030。
 * 例如正6~10、负11~15：积分 = sum(ADC[6..10])-sum(ADC[11..15])。
 */
#define CORR_POS_START                 6U  /* 正积分窗口起始下标，包含该点。 */
#define CORR_POS_END                  10U  /* 正积分窗口结束下标，包含该点。 */
#define CORR_NEG_START                11U  /* 负积分窗口起始下标，包含该点。 */
#define CORR_NEG_END                  15U  /* 负积分窗口结束下标，包含该点。 */
#define CORR_POS_COUNT (CORR_POS_END - CORR_POS_START + 1U) /* 正窗口点数。 */
#define CORR_NEG_COUNT (CORR_NEG_END - CORR_NEG_START + 1U) /* 负窗口点数。 */

/* ============================= 滤波和输出参数 =========================== */
#define SIGNAL_FILTER_Q_BITS           8U  /* IIR定点小数精度，内部状态=积分值*256。 */
#define SIGNAL_FILTER_NORMAL_SHIFT     1U  /* 正常帧IIR系数1/(2^1)=1/2。 */
#define SIGNAL_TRIMMED_MEAN_ENABLE     1U  /* 1=开启8组IIR截尾平均；0=直接使用IIR结果。 */
#define SIGNAL_TRIM_WINDOW_SIZE        8U  /* 截尾历史数量；当前固定8组，去掉两端各2组。 */
/* 下列阈值单位均为正负窗口相关积分值，需根据实测数据重新标定。 */
#define OUTPUT_OFF_THRESHOLD          200  /* 有光状态下，积分低于该值连续确认后切换为遮光。 */
#define OUTPUT_HYSTERESIS             100  /* 遮光状态下，ON阈值=OFF阈值+迟滞量。 */
#define OUTPUT_LIGHT_CONFIRM_COUNT     5U /* 连续5次达到ON阈值才确认有光。 */
#define OUTPUT_DARK_CONFIRM_COUNT      5U /* 连续5次低于OFF阈值才确认遮光。 */
#define OUTPUT_LIGHT_ACTIVE_HIGH       1U /* 1=有光输出高电平；0=有光输出低电平。 */
#define OUTPUT_TEST_ENABLE             0U /* 1=PA12测DMA中断耗时；0=PA12输出有光状态。 */

/* 编译期参数检查：防止修改积分范围时越界、反向或重叠。 */
#if (CORR_POS_START > CORR_POS_END) || (CORR_NEG_START > CORR_NEG_END)
#error "Correlation window start must be <= end"
#endif
#if (CORR_POS_END >= BUFFER_SIZE) || (CORR_NEG_END >= BUFFER_SIZE)
#error "Correlation window index exceeds ADC buffer"
#endif
#if !((CORR_POS_END < CORR_NEG_START) || (CORR_NEG_END < CORR_POS_START))
#error "Positive and negative correlation windows must not overlap"
#endif
#if (CORR_POS_COUNT != CORR_NEG_COUNT)
#error "Positive and negative windows must have equal sample counts"
#endif
#if (LIGHT_INTERFERENCE_RELEASE_THRESHOLD >= LIGHT_INTERFERENCE_THRESHOLD)
#error "Interference release threshold must be lower than enter threshold"
#endif
#if (LIGHT_INTERFERENCE_CONFIRM_COUNT == 0U) || (LIGHT_INTERFERENCE_RELEASE_COUNT == 0U)
#error "Interference confirmation counts must be nonzero"
#endif
#if (SIGNAL_TRIM_WINDOW_SIZE != 8U)
#error "Current trimmed mean implementation requires 8 samples"
#endif
/* 12位ADC每个采样最大4095，保证Q8目标量在int32_t范围内。 */
#if ((CORR_POS_COUNT * 4095UL) > (2147483647UL >> SIGNAL_FILTER_Q_BITS))
#error "Correlation magnitude can overflow int32 Q8 state"
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

/* A. 有光干扰监测：不再划分正常/弱干扰，只判断是否进入冻结状态。 */
volatile uint32_t g_noise_processed_frames = 0U; /* 已完成背景偏差分析的总帧数。 */
volatile uint32_t g_noise_metric = 0U;           /* 索引0~5相对固定参考值的平均绝对偏差指标；不是统计学MAD。 */
volatile uint32_t g_baseline_average = 0U;       /* 索引0~5的ADC平均值，仅用于观察背景基线漂移。 */
volatile uint8_t  g_light_interference_state = 0U; /* 有光干扰状态：0=正常检测，1=强干扰冻结输出。 */
volatile uint8_t  g_light_interference_confirm = 0U; /* 连续达到进入阈值的帧数，达到5帧后进入冻结。 */
volatile uint8_t  g_light_interference_release_confirm = 0U; /* 冻结期间连续低于解除阈值的帧数。 */
volatile uint32_t g_light_interference_enter_count = 0U; /* 累计进入有光干扰冻结状态的次数。 */
volatile uint32_t g_light_interference_release_count = 0U; /* 累计解除有光干扰冻结状态的次数。 */

/* B. 正负窗口相关积分：冻结期间保持所有积分及滤波结果不变。 */
volatile int32_t  g_match_correlation = 0;       /* 最近一帧正常处理时的正负窗口积分。 */
volatile int32_t  g_match_filtered_integral = 0; /* 固定IIR系数1/2后的积分结果，不再按干扰等级切换。 */
volatile uint32_t g_match_valid_frames = 0U;     /* 正常参与相关积分的累计帧数。 */
volatile uint32_t g_match_frozen_frames = 0U;    /* 处于有光干扰冻结状态、未更新检测算法的帧数。 */
volatile uint8_t  g_match_frame_valid = 0U;      /* 最近一帧是否参与积分：1=参与，0=冻结未处理。 */

/* C. 截尾与输出：每个正常帧先IIR，再将最近8个IIR结果去高低各2个，平均剩余4个；冻结时不更新。 */
volatile int32_t  g_output_trimmed_integral = 0; /* 最新可用的截尾平均积分；输出迟滞判断实际使用的数值。 */
volatile uint8_t  g_output_ready = 0U;          /* 0=尚未收齐首批8个有效IIR值；1=已有可用输出值，置1后持续保持。 */
volatile uint8_t  g_output_light_state = 0U;    /* 当前逻辑光状态：0=遮光，1=有光；OUTPUT_TEST_ENABLE=1时不会映射到PA12。 */
volatile uint8_t  g_output_light_confirm = 0U;  /* 遮光状态下，连续满足有光ON门槛的有效帧计数；达到5次才切为有光。 */
volatile uint8_t  g_output_dark_confirm = 0U;   /* 有光状态下，连续满足遮光OFF门槛的有效帧计数；达到5次才切为遮光。 */
volatile uint32_t g_output_state_changes = 0U; /* 上电至今逻辑光状态真正发生0/1切换的累计次数。 */

/* D. 算法内部状态：通常无需加入Watch；这里保留注释方便定位。 */
static int32_t g_signal_filter_q8 = 0;          /* IIR的Q8定点状态：内部数值约为滤波积分×256，不是ADC原始值。 */
#if SIGNAL_TRIMMED_MEAN_ENABLE
static int32_t g_output_trim_history[SIGNAL_TRIM_WINDOW_SIZE] = {0}; /* 最近8次有效帧的IIR积分历史，环形缓冲区。 */
static uint8_t g_output_trim_write = 0U;        /* 下一次将写入的历史数组位置，循环0~7。 */
static uint8_t g_output_trim_count = 0U;        /* 当前已收集的有效IIR值个数，上限8；满8前不做截尾输出。 */
#endif

static void APP_SystemClockConfig(void);
static void GPIO_Init(void);
static void TMR3_OC_Init(void);
static void APP_AdcInit(void);
static void APP_AdcStartDMA(void);
static void APP_AdcDmaFastRearm(void);
#if TRANSMIT_RANDOM_ENABLE
static const uint16_t random_period_table[RANDOM_PERIOD_COUNT] = {
    140U, 140U, 140U, 140U, 140U, 140U, 140U, 140U, 140U, 140U
}; /* 当前10个候选值均为140us，虽运行随机抽取但周期实际固定。 */
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
/*
 * 更新有光干扰状态。前6点相对固定基准的平均绝对偏差仅作为异常指标，
 * 不再解释为光强等级，也不再根据它切换IIR参数。
 * 返回1表示本帧冻结，不更新积分、滤波及输出；返回0表示正常处理。
 */
static uint8_t APP_LightInterferenceUpdate(const uint16_t *samples)
{
    uint32_t i, sum = 0U, deviations = 0U;
    uint8_t was_frozen = g_light_interference_state;

    for (i = 0U; i < 6U; ++i)
    {
        uint32_t value = samples[i];
        sum += value;
        deviations += (value >= LIGHT_INTERFERENCE_BASELINE)
                    ? value - LIGHT_INTERFERENCE_BASELINE
                    : LIGHT_INTERFERENCE_BASELINE - value;
    }

    g_baseline_average = sum / 6U;
    g_noise_metric = deviations / 6U;
    ++g_noise_processed_frames;

    if (g_light_interference_state == 0U)
    {
        g_light_interference_release_confirm = 0U;
        if (g_noise_metric >= LIGHT_INTERFERENCE_THRESHOLD)
        {
            if (g_light_interference_confirm < LIGHT_INTERFERENCE_CONFIRM_COUNT)
                ++g_light_interference_confirm;
            if (g_light_interference_confirm >= LIGHT_INTERFERENCE_CONFIRM_COUNT)
            {
                g_light_interference_state = 1U;
                g_light_interference_confirm = 0U;
                g_output_light_confirm = 0U;
                g_output_dark_confirm = 0U;
                ++g_light_interference_enter_count;
                return 1U;
            }
        }
        else
        {
            g_light_interference_confirm = 0U;
        }
        return 0U;
    }

    /* 已冻结：只有连续多帧低于解除阈值，才允许恢复检测。 */
    g_light_interference_confirm = 0U;
    if (g_noise_metric <= LIGHT_INTERFERENCE_RELEASE_THRESHOLD)
    {
        if (g_light_interference_release_confirm < LIGHT_INTERFERENCE_RELEASE_COUNT)
            ++g_light_interference_release_confirm;
        if (g_light_interference_release_confirm >= LIGHT_INTERFERENCE_RELEASE_COUNT)
        {
            g_light_interference_state = 0U;
            g_light_interference_release_confirm = 0U;
            g_output_light_confirm = 0U;
            g_output_dark_confirm = 0U;
            ++g_light_interference_release_count;
        }
    }
    else
    {
        g_light_interference_release_confirm = 0U;
    }

    /* 本帧开始时已冻结，即使本帧刚完成解除确认，也从下一帧恢复处理。 */
    return was_frozen;
}

/* 正负窗口积分，固定中心值因两区点数相同而完全抵消。
 * 原始ADC顺序，不使用按值排序后的索引。
 */
static int32_t APP_SignalCalculateCorrelation(const uint16_t *samples)
{
    uint32_t i;
    int32_t positive = 0, negative = 0;
    for (i = CORR_POS_START; i <= CORR_POS_END; ++i) positive += samples[i];
    for (i = CORR_NEG_START; i <= CORR_NEG_END; ++i) negative += samples[i];
    return positive - negative;
}
static void APP_SignalFilterUpdate(int32_t integral)
{
    int32_t target_q8 = integral * (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
    g_signal_filter_q8 += (target_q8 - g_signal_filter_q8) / (int32_t)(1U << SIGNAL_FILTER_NORMAL_SHIFT);
    g_match_filtered_integral = g_signal_filter_q8 / (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
}
/* 8个IIR结果去掉最大2个、最小2个，再求中间4个平均。
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
/* 中断中完成本帧运算，再重装DMA；有光干扰冻结时只更新监测状态和DMA。 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {
#if OUTPUT_TEST_ENABLE
        GPIOA->BRR = GPIO_PIN_12;
#endif
        /* 连续5帧达到有光干扰阈值后冻结；冻结期间保持输出及全部滤波状态。 */
        if (APP_LightInterferenceUpdate(adc_buffer) != 0U)
        {
            g_match_frame_valid = 0U;
            ++g_match_frozen_frames;
        }
        else
        {
            int32_t correlation = APP_SignalCalculateCorrelation(adc_buffer);
            g_match_correlation = correlation;
            g_match_frame_valid = 1U;
            ++g_match_valid_frames;

            /* 所有正常帧使用同一组IIR参数，不再区分正常/弱干扰。 */
            APP_SignalFilterUpdate(correlation);
            if (APP_OutputTrimUpdate(g_match_filtered_integral) != 0U)
                APP_OutputUpdateDecision(g_output_trimmed_integral);
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
