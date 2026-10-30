/*
 * 基础配置说明版
 *
 * 配置流程：
 * HAL初始化 -> 系统时钟 -> GPIO -> TIM3配置 -> ADC/DMA配置
 * -> DMA先启动并等待触发 -> 启动TIM3 CH2和CH1 -> DMA完成中断寄存器快速重装
 *
 * 本文件仅为基础配置展示：
 * 1. 追加波形分区与固定基线平均绝对偏差(MAD)逐帧分类，暂不加入输出或模板积分。
 * 2. 宏控制固定周期或随机周期；随机模式才启用TIM3更新中断，ADC与DMA采集机制不变。
 * 3. 每帧只计算相关积分，累计10个有效帧求均值，再计算匹配幅度并更新两档IIR。
 * 4. 两种模式都在周期末前7us由TIM3 CH2的OC2REF上升沿发出TRGO。
 * 5. ADC仍采用连续转换，DMA仍为NORMAL模式，每轮搬运14点；保留原快速重装操作。
 *    因此本版并不保证每个随机发射周期只采1点；这是保留新版ADC配置的结果。
 * 6. 当前候选表为140us（10项相同），修改随机表后须重新检查中断与DMA重装时序。
 * 7. 需要结合main.h、HAL底层MSP及芯片参考手册确认ADC0对应引脚和DMA请求映射。
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
#define RANDOM_ADC_BEFORE_END_US  7U

#if TRANSMIT_RANDOM_ENABLE
/* 从main(4).c移植的候选表；可以随机重复同一周期。 */
static const uint16_t random_period_table[RANDOM_PERIOD_COUNT] = {
    140U, 140U, 140U, 140U, 140U, 140U, 140U, 140U, 140U, 140U
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
/* 每次DMA传输BUFFER_SIZE个ADC转换结果；每个结果存入一个16位元素，并非多路ADC。 */
#define BUFFER_SIZE 14

/* ==================== 波形分区配置（含首尾索引） ====================
 * 只调整下面四个索引，就能修改各自区域的起止位置。
 * 当前分区：索引0~5作为背景区，6~13作为有效光信号区。
 * 重要：默认划分只是示例！需根据ADC实测脉冲位置重新调整。
 * 两个区域必须在[0, BUFFER_SIZE-1]中，不允许重叠。
 */
#define ADC_BASELINE_START_INDEX  0U
#define ADC_BASELINE_END_INDEX    5U
#define ADC_SIGNAL_START_INDEX    6U
#define ADC_SIGNAL_END_INDEX      13U

/* 固定背景参考ADC码：仅用于计算绝对偏差，不随当前帧均值变化。 */
#define NOISE_BASELINE_REFERENCE    2040U

#if (NOISE_BASELINE_REFERENCE > 4095U)
#error "NOISE_BASELINE_REFERENCE must be within 0..4095 for 12-bit ADC"
#endif

#if (BUFFER_SIZE < 2)
#error "BUFFER_SIZE must be >= 2"
#endif
#if (ADC_BASELINE_START_INDEX > ADC_BASELINE_END_INDEX) || \
    (ADC_SIGNAL_START_INDEX > ADC_SIGNAL_END_INDEX) || \
    (ADC_BASELINE_END_INDEX >= BUFFER_SIZE) || \
    (ADC_SIGNAL_END_INDEX >= BUFFER_SIZE)
#error "ADC area indexes out of range or inverted"
#endif
#if !((ADC_BASELINE_END_INDEX < ADC_SIGNAL_START_INDEX) || \
      (ADC_SIGNAL_END_INDEX < ADC_BASELINE_START_INDEX))
#error "ADC baseline and signal areas must not overlap"
#endif

/* ==================== 当前帧干扰分类阈值 ====================
 * 每次DMA完成只判断当前帧，不保留历史状态、不迟滞、不进行连续确认。
 * MAD < WEAK_THRESHOLD              : 0 正常
 * WEAK_THRESHOLD <= MAD < SEVERE_THRESHOLD : 1 弱干扰
 * MAD >= SEVERE_THRESHOLD          : 2 强干扰
 * 两个阈值为调试初值，需按实际采样分布调整。
 */
#define NOISE_MAD_WEAK_THRESHOLD     15U
#define NOISE_MAD_SEVERE_THRESHOLD   60U

#if (NOISE_MAD_WEAK_THRESHOLD >= NOISE_MAD_SEVERE_THRESHOLD)
#error "Noise thresholds must satisfy WEAK_THRESHOLD < SEVERE_THRESHOLD"
#endif

/* ==================== 信号模板、10周期积分平均及两档IIR ====================
 * 信号区索引6~13，共8点；每个模板元素依次对应ADC索引6、7...13。
 * 模板元素是【原始ADC码 - 固定基线2040】后的有符号差值。
 * 例如ADC读数为2020，模板元素填-20；ADC读数为2100，则填60。
 * 以下波形仅供算法测试，绝非实际测定模板；上板后请逐个替换。
 * 注意模板必须与PWM触发和ADC采样时序对齐，且非全零。
 */
#define SIGNAL_TEMPLATE_0    0
#define SIGNAL_TEMPLATE_1  -20
#define SIGNAL_TEMPLATE_2  -80
#define SIGNAL_TEMPLATE_3 -200
#define SIGNAL_TEMPLATE_4 -270
#define SIGNAL_TEMPLATE_5 -190
#define SIGNAL_TEMPLATE_6  -60
#define SIGNAL_TEMPLATE_7  -10

/* 匹配幅度用1000表示与模板等幅；比如500表示50%。
 * IIR以放大256倍(Q8)的有符号状态保存，避免整数逐帧除法造成小差值不更新。
 * 仅允许按2的整数次幂设定K，运行时由位移实现低开销滤波。
 */
#define SIGNAL_ACCUMULATE_CYCLES 10U /* 满10个有效DMA帧才计算一次积分均值、匹配幅度和IIR */
#define SIGNAL_AMPLITUDE_SCALE 1000L
#define SIGNAL_FILTER_Q_BITS  8U
#define SIGNAL_FILTER_NORMAL_SHIFT 1U /* 正常: K=2, alpha=1/2 */
#define SIGNAL_FILTER_WEAK_SHIFT   3U /* 弱干扰: K=8, alpha=1/8 */

#if ((ADC_SIGNAL_END_INDEX - ADC_SIGNAL_START_INDEX + 1U) != 8U)
#error "Signal template has exactly 8 points: check ADC signal area indexes"
#endif
#if (SIGNAL_ACCUMULATE_CYCLES < 1U)
#error "SIGNAL_ACCUMULATE_CYCLES must be >= 1"
#endif
#if (SIGNAL_FILTER_NORMAL_SHIFT > 7U) || (SIGNAL_FILTER_WEAK_SHIFT > 7U)
#error "IIR shifts must be <= 7"
#endif

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

/* DMA完成后直接分析adc_buffer；必须在重新武装DMA之前完成分析。 */
volatile uint32_t g_noise_processed_frames = 0U;

/* Keil Watch观测：仅代表最近一帧的干扰分类，不是保持状态。 */
volatile uint8_t  g_noise_level = 0U;     /* 0正常/1弱干扰/2强干扰 */
volatile uint32_t g_noise_metric = 0U;    /* 当前帧固定2040基线平均绝对偏差 */
volatile uint32_t g_baseline_average = 0U;/* 实测背景均值，只用于调试 */
volatile uint32_t g_signal_average = 0U;  /* 信号区均值，保留供后续算法使用 */

typedef enum {
    NOISE_LEVEL_NORMAL = 0U,
    NOISE_LEVEL_WEAK   = 1U,
    NOISE_LEVEL_SEVERE = 2U
} NoiseLevel_t;

/* 固定模板，不在中断内采集和更新；实验确认后替换上面的8个宏。 */
static const int16_t g_signal_template[8] = {
    SIGNAL_TEMPLATE_0, SIGNAL_TEMPLATE_1, SIGNAL_TEMPLATE_2, SIGNAL_TEMPLATE_3,
    SIGNAL_TEMPLATE_4, SIGNAL_TEMPLATE_5, SIGNAL_TEMPLATE_6, SIGNAL_TEMPLATE_7
};

/* 仅在启动时计算一次模板能量ΣT²，避免每帧重复计算。 */
static uint32_t g_signal_template_energy = 0U;
static int32_t g_signal_filter_q8 = 0;

/* 调试变量：单帧只有相关积分；匹配幅度与IIR仅在满10个有效帧时更新。 */
volatile int32_t  g_match_correlation = 0;          /* 最新有效单帧的 S[n] */
volatile int64_t  g_match_correlation_sum10 = 0;    /* 最近一组10帧相关积分总和 */
volatile int32_t  g_match_correlation_avg10 = 0;    /* 最近一组10帧相关积分均值 */
volatile int32_t  g_match_amplitude_avg10_x1000 = 0; /* 10帧积分均值匹配幅度 */
volatile int32_t  g_match_filtered_x1000 = 0;     /* 以上述10帧幅度为输入的IIR */
volatile uint32_t g_match_valid_frames = 0U;
volatile uint32_t g_match_rejected_frames = 0U;
volatile uint8_t  g_match_frame_valid = 0U;
volatile uint32_t g_match_accumulate_pending_count = 0U;
volatile uint32_t g_match_accumulate_groups = 0U;

/* 只保留累加器和本组弱干扰标志，不保存原始波形或单帧幅度数组。 */
static int64_t g_match_sum_accumulator = 0;
static uint8_t g_match_group_has_weak = 0U;

static void APP_SignalTemplateInit(void);
static int32_t APP_SignalCalculateCorrelation(const uint16_t *samples);
static void APP_SignalFilterUpdate(int32_t amplitude, uint8_t noise_level);
static void APP_SignalAccumulateAndFilter(int32_t correlation, uint8_t noise_level);
static void APP_SignalProcessFrame(const uint16_t *samples);

/* DMA中断只调度函数；全部计算直接使用adc_buffer，不复制快照。 */
static uint32_t APP_NoiseCalculateMAD(const uint16_t *samples, uint32_t *baseline_mean);
static uint8_t APP_NoiseClassifyFrame(uint32_t mad);
static void APP_NoiseAnalyzeFrame(const uint16_t *samples);
static void APP_AdcDmaFastRearm(void);

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
    /* 模板能量只计算一次；首次ADC硬件触发之前完成准备。 */
    APP_SignalTemplateInit();
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

    /* 业务处理全部由中断触发，主循环保持空闲。 */
    while (1)
    {
    }
}
/* ====================== 波形分区与背景MAD分析 ======================
 * DMA NORMAL完成后，直接读取原始adc_buffer；无需复制快照。
 * 注意：MAD分析完成前不能重装DMA，否则下一帧可能覆盖当前数据。
 */
/* 阶段2：计算背景区相对于固定基线NOISE_BASELINE_REFERENCE的平均绝对偏差。
 * 公式：metric = Σ |samples[i] - NOISE_BASELINE_REFERENCE| / count。
 * 与传统“围绕本帧均值的MAD”不同，固定基线指标同时反映背景直流偏移
 * 与随机波动；这正是本版的预期行为。
 * 只遍历背景区一次，不排序、不去极值、不复制快照。
 * 另外记录背景区真实均值供Keil Watch观察，但不参与偏差计算。
 */
static uint32_t APP_NoiseCalculateMAD(const uint16_t *samples,
                                     uint32_t *baseline_mean)
{
    uint32_t i;
    uint32_t sum = 0U;
    uint32_t deviations = 0U;
    const uint32_t count = ADC_BASELINE_END_INDEX - ADC_BASELINE_START_INDEX + 1U;

    for (i = ADC_BASELINE_START_INDEX; i <= ADC_BASELINE_END_INDEX; ++i)
    {
        uint32_t value = samples[i];
        sum += value;
        deviations += (value >= NOISE_BASELINE_REFERENCE)
                    ? (value - NOISE_BASELINE_REFERENCE)
                    : (NOISE_BASELINE_REFERENCE - value);
    }

    *baseline_mean = sum / count;  /* 仅保留用于观察当前背景实际均值 */
    return deviations / count;   /* 固定基线平均绝对偏差，整数ADC码 */
}

/* 阶段3：依据本帧MAD直接分类，不依赖此前任何帧的判断结果。 */
static uint8_t APP_NoiseClassifyFrame(uint32_t mad)
{
    if (mad >= NOISE_MAD_SEVERE_THRESHOLD)
        return (uint8_t)NOISE_LEVEL_SEVERE;

    if (mad >= NOISE_MAD_WEAK_THRESHOLD)
        return (uint8_t)NOISE_LEVEL_WEAK;

    return (uint8_t)NOISE_LEVEL_NORMAL;
}

/* 阶段4：更新当前帧平均绝对偏差和分类，信号区仍只计算均值。 */
static void APP_NoiseAnalyzeFrame(const uint16_t *samples)
{
    uint32_t i;
    uint32_t signal_sum = 0U;
    uint32_t baseline_mean = 0U;
    const uint32_t signal_count = ADC_SIGNAL_END_INDEX - ADC_SIGNAL_START_INDEX + 1U;
    const uint32_t mad = APP_NoiseCalculateMAD(samples, &baseline_mean);

    for (i = ADC_SIGNAL_START_INDEX; i <= ADC_SIGNAL_END_INDEX; ++i)
        signal_sum += samples[i];

    g_baseline_average = baseline_mean;
    g_signal_average = signal_sum / signal_count;
    g_noise_metric = mad;
    g_noise_level = APP_NoiseClassifyFrame(mad);
    ++g_noise_processed_frames;
}

/* ==================== 相干积分 → 10有效周期平均 → 匹配幅度 → IIR ====================
 * 一次ADC DMA传输对应一个完整波形。强干扰帧被丢弃，不增加累计次数。
 * 正常/弱干扰帧只进行8点相关乘加；满10帧后才进行一次均值除法、
 * 幅度定标除法和IIR更新。这是单条串联处理链，而非原来的两条并行路径。
 */
static void APP_SignalTemplateInit(void)
{
    uint32_t i;
    uint32_t energy = 0U;
    for (i = 0U; i < 8U; ++i)
    {
        int32_t v = (int32_t)g_signal_template[i];
        energy += (uint32_t)(v * v);
    }
    if (energy == 0U) APP_ErrorHandler();
    g_signal_template_energy = energy;
}

/* 阶段1：每个有效周期做8点相关积分，暂不进行幅度除法。 */
static int32_t APP_SignalCalculateCorrelation(const uint16_t *samples)
{
    uint32_t i;
    int32_t correlation = 0;
    for (i = 0U; i < 8U; ++i)
    {
        int32_t x = (int32_t)samples[ADC_SIGNAL_START_INDEX + i]
                  - (int32_t)NOISE_BASELINE_REFERENCE;
        correlation += x * (int32_t)g_signal_template[i];
    }
    return correlation;
}

/* 阶段4：对10帧均值求出的匹配幅度做IIR，不再对每个单帧做IIR。
 * 弱干扰档K=8，正常档K=2。分组中只要出现弱干扰有效帧，
 * 整组使用较深的弱干扰档；强干扰帧根本不进入该组。
 */
static void APP_SignalFilterUpdate(int32_t amplitude, uint8_t noise_level)
{
    uint32_t shift = (noise_level == (uint8_t)NOISE_LEVEL_WEAK)
                   ? SIGNAL_FILTER_WEAK_SHIFT : SIGNAL_FILTER_NORMAL_SHIFT;
    int32_t target_q8 = amplitude * (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
    int32_t difference = target_q8 - g_signal_filter_q8;
    g_signal_filter_q8 += difference / (int32_t)(1U << shift);
    g_match_filtered_x1000 = g_signal_filter_q8 / (int32_t)(1U << SIGNAL_FILTER_Q_BITS);
}

/* 阶段2和3：先累计10帧S[n]，取积分均值，再算相对模板的幅度。
 * amplitude_x1000 = (mean(S) * 1000) / sum(T[i]^2)。
 * 平均与归一化只在第10个有效帧进行，避免每帧两次除法。
 * 强干扰帧不计数、不会重置已经积累的有效帧。
 */
static void APP_SignalAccumulateAndFilter(int32_t correlation, uint8_t noise_level)
{
    g_match_sum_accumulator += (int64_t)correlation;
    if (noise_level == (uint8_t)NOISE_LEVEL_WEAK)
        g_match_group_has_weak = 1U;

    ++g_match_accumulate_pending_count;
    if (g_match_accumulate_pending_count >= SIGNAL_ACCUMULATE_CYCLES)
    {
        int64_t total = g_match_sum_accumulator;
        int32_t average = (int32_t)(total / SIGNAL_ACCUMULATE_CYCLES);
        int32_t amplitude = (int32_t)(((int64_t)average * SIGNAL_AMPLITUDE_SCALE)
                            / (int64_t)g_signal_template_energy);
        uint8_t group_level = g_match_group_has_weak
                            ? (uint8_t)NOISE_LEVEL_WEAK : (uint8_t)NOISE_LEVEL_NORMAL;

        g_match_correlation_sum10 = total;
        g_match_correlation_avg10 = average;
        g_match_amplitude_avg10_x1000 = amplitude;
        APP_SignalFilterUpdate(amplitude, group_level);
        ++g_match_accumulate_groups;

        g_match_sum_accumulator = 0;
        g_match_accumulate_pending_count = 0U;
        g_match_group_has_weak = 0U;
    }
}

/* 强干扰拒绝本帧，其余帧计算一次相关积分并加入10周期累加器。 */
static void APP_SignalProcessFrame(const uint16_t *samples)
{
    int32_t correlation;
    if (g_noise_level == (uint8_t)NOISE_LEVEL_SEVERE)
    {
        g_match_frame_valid = 0U;
        ++g_match_rejected_frames;
        return;
    }

    correlation = APP_SignalCalculateCorrelation(samples);
    g_match_correlation = correlation;
    APP_SignalAccumulateAndFilter(correlation, g_noise_level);
    g_match_frame_valid = 1U;
    ++g_match_valid_frames;
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

    /* CH2 PWM2：在周期终点前RANDOM_ADC_BEFORE_END_US产生OC2REF上升沿。 */
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

/* ADC1及DMA配置：ADC0单通道，12位，TIM3 TRGO触发，连续转换，DMA一次搬运BUFFER_SIZE个点。 */
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

/* 启动一轮ADC DMA传输：BUFFER_SIZE个结果写入数组，随后等待下一次重装。 */
static void APP_AdcStartDMA(void)
{
    /* 配置并启动ADC DMA。第三个参数BUFFER_SIZE代表DMA要搬运的转换结果个数。 */
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
 * 1. 不调用ADC_Disable和HAL_ADC_Stop_DMA；保持原版ADEN重启步骤。
 * 2. DMA搬满BUFFER_SIZE点后结束本轮连续转换。
 * 3. DMA通道重装CNDTR=BUFFER_SIZE，下一次TRGO再启动下一组连续转换。
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

/* DMA TC中断：当前帧MAD判断 -> 模板相关/10帧累加/IIR -> 快速重新武装DMA。
 * 不使用快照、不在主循环处理数据；各阶段分别由函数负责。
 * 代价：新增8次乘加和一次除法，推迟下次TRGO前的重新武装，需实测PA12确认。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {
#if OUTPUT_TEST_ENABLE
        GPIOA->BRR = GPIO_PIN_12;
#endif
        APP_NoiseAnalyzeFrame(adc_buffer);
        APP_SignalProcessFrame(adc_buffer);
        APP_AdcDmaFastRearm();
#if OUTPUT_TEST_ENABLE
        GPIOA->BSRR = GPIO_PIN_12;
#endif
    }
    /* 保持原版DMA TE错误处理策略不变。 */
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
