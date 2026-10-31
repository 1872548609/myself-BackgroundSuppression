/*
 * 基础配置说明版
 *
 * 配置流程：
 * HAL初始化 -> 系统时钟 -> GPIO -> TIM3配置 -> ADC/DMA配置
 * -> DMA先启动并等待触发 -> 启动TIM3 CH2和CH1 -> DMA完成中断寄存器快速重装
 *
 * 本文件仅为基础配置展示：
 * 1. 波形分区与固定基线MAD逐帧分类；10有效帧相关积分均值→幅度→IIR→可选8点截尾平均→输出判断。
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

/**
 * @brief 生成下一次发射周期候选值。
 * @return uint16_t：从 random_period_table 中选出的周期，单位 µs。
 * @note 更新 rng_state；只在随机模式下由 TIM3 更新中断调用。
 */
static uint16_t Random_NextPeriod(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return random_period_table[rng_state % RANDOM_PERIOD_COUNT];
}

/* 只写预装载值；ARR、CCR1、CCR2在下一个TIM3更新事件同时生效。 */
/**
 * @brief 设置下一周期 TIM3 的 ARR、CCR1 与 CCR2 预装载值。
 * @param[in] period：下一轮发射周期，单位 µs；须大于 PWM/ADC 提前量。
 * @return 无（void）。
 * @note 仅排队寄存器预装载值，实际生效依赖下一次 TIM3 更新事件；不启动 ADC。
 */
static void TIM3_QueuePeriod(uint16_t period)
{
    TIM3->ARR  = (uint32_t)(period - 1U);
    TIM3->CCR1 = (uint32_t)(period - RANDOM_PWM_OFF_US);
    TIM3->CCR2 = (uint32_t)(period - RANDOM_ADC_BEFORE_END_US);
}

/* 更新中断仅预先安排下一轮随机周期，不进行ADC算法或统计。 */
/**
 * @brief TIM3 更新事件中断：准备下一个随机周期。
 * @return 无（void）。
 * @note 只处理 UIF 并调用 TIM3_QueuePeriod；不负责 ADC 数据处理。
 */
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
#define NOISE_BASELINE_REFERENCE    2030U

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
 * 模板元素应为【原始ADC码 - NOISE_BASELINE_REFERENCE】后的有符号差值。
 * 例如ADC读数为2020，模板元素填-20；ADC读数为2100，则填60。
 * 以下波形仅供算法测试，绝非实际测定模板；上板后请逐个替换。
 * 注意模板必须与PWM触发和ADC采样时序对齐，且非全零。
 */

#define SIGNAL_TEMPLATE_0    167
#define SIGNAL_TEMPLATE_1    278
#define SIGNAL_TEMPLATE_2    240
#define SIGNAL_TEMPLATE_3    183
#define SIGNAL_TEMPLATE_4    120
#define SIGNAL_TEMPLATE_5   -211
#define SIGNAL_TEMPLATE_6   -256
#define SIGNAL_TEMPLATE_7   -202


/* 匹配幅度用1000表示与模板等幅；比如500表示50%。
 * IIR以放大256倍(Q8)的有符号状态保存，避免整数逐帧除法造成小差值不更新。
 * 仅允许按2的整数次幂设定K，运行时由位移实现低开销滤波。
 */
#define SIGNAL_ACCUMULATE_CYCLES 10U /* 满10个有效DMA帧才计算一次积分均值、匹配幅度和IIR */
#define SIGNAL_AMPLITUDE_SCALE 1000L
#define SIGNAL_FILTER_Q_BITS  8U
#define SIGNAL_FILTER_NORMAL_SHIFT 1U /* 正常: K=2, alpha=1/2 */
#define SIGNAL_FILTER_WEAK_SHIFT   3U /* 弱干扰: K=8, alpha=1/8 */

/* ============== IIR后8组截尾平均（按 IIR 更新次数计数） ==============
 * 8个IIR输出排序后剔除最低2个、最高2个，中间4个取平均。
 * 这是两端各截去1/4，并非两端各截去1/2（那样无法保留数据）。
 * 开启时，首次收满8次IIR结果才允许产生输出判断。
 */
#define SIGNAL_TRIMMED_MEAN_ENABLE  1U
#define SIGNAL_TRIM_WINDOW_SIZE    8U
#define SIGNAL_TRIM_DROP_EACH_SIDE 2U

/* 输出幅度阈值，以 x1000 为单位：
 * 正常背景：>60才有光确认，迟滞10 -> ON=70, OFF=60。
 * 弱干扰：>100才有光确认，迟滞10 -> ON=110, OFF=100。
 * 等于上阈值视为满足进入；等于下阈值视为满足退出。
 */
#define OUTPUT_NORMAL_OFF_THRESHOLD  100
#define OUTPUT_NORMAL_HYSTERESIS     10
#define OUTPUT_WEAK_OFF_THRESHOLD    200
#define OUTPUT_WEAK_HYSTERESIS       20
#define OUTPUT_LIGHT_CONFIRM_COUNT   5U
#define OUTPUT_DARK_CONFIRM_COUNT    5U
/* 测试关时PA12输出：1=有光输出高电平，0=有光输出低电平 */
#define OUTPUT_LIGHT_ACTIVE_HIGH     1U

#if (SIGNAL_TRIM_WINDOW_SIZE != 8U) || (SIGNAL_TRIM_DROP_EACH_SIDE != 2U)
#error "Trimmed average is fixed to 8 values, discard 2 low and 2 high"
#endif
#if (OUTPUT_LIGHT_CONFIRM_COUNT < 1U) || (OUTPUT_DARK_CONFIRM_COUNT < 1U)
#error "Output confirmation count must be positive"
#endif


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
#define OUTPUT_TEST_ENABLE 0
#if (OUTPUT_TEST_ENABLE != 0U) && (OUTPUT_TEST_ENABLE != 1U)
#error "OUTPUT_TEST_ENABLE must be 0 or 1"
#endif

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
/* ========================= Keil Watch 调试变量说明 =========================
 * 更新频率：
 * ① 每次 DMA TC（14点完整一帧）：背景MAD、干扰等级及有效帧相关积分。
 * ② 每累计 SIGNAL_ACCUMULATE_CYCLES（默认10）个有效帧：相关均值、匹配幅度、IIR。
 * ③ 每得到一次新IIR结果：截尾窗口更新；窗口就绪才进行输出迟滞/确认。
 * ④ 强干扰帧（等级2）不会参与10帧累计，也不会更新积分/滤波/输出判断结果。
 * 注意：Watch暂停时，单帧变量与10帧变量可能来自不同组，这是正常现象。
 *       “x1000”表示将无单位的相对模板幅度放大1000倍，并非ADC原始值。
 * ======================================================================== */

/* ADC原始波形：每完成一次DMA，依次装入14个12位ADC码值（0~4095）。
 * adc_buffer[0..5]为背景区；adc_buffer[6..13]为有效信号区。
 * DMA重装后下一帧会覆盖此数组；本程序不复制波形快照。 */
uint16_t adc_buffer[BUFFER_SIZE];

/* -------- A. 背景干扰判断：每次14点DMA传输完成后更新 -------- */
volatile uint32_t g_noise_processed_frames = 0U; /* 已完成MAD分析的总帧数；每次DMA TC +1。 */
volatile uint8_t  g_noise_level = 0U; /* 当前最新一帧干扰级别：0=正常(<15)，1=弱干扰(15~59)，2=强干扰(>=60)；无历史迟滞。 */
volatile uint32_t g_noise_metric = 0U; /* 最新一帧背景区0..5相对固定基线2030的平均绝对偏差；单位ADC码，数值越大偏离越多。 */
volatile uint32_t g_baseline_average = 0U; /* 最新一帧背景0..5的实际平均ADC码；仅用于观察基线漂移，不参与当前MAD的中心值计算。 */
volatile uint32_t g_signal_average = 0U; /* 最新一帧有效区6..13的原始ADC算术均值；仅调试，不是模板匹配幅度。 */

typedef enum {
    NOISE_LEVEL_NORMAL = 0U,
    NOISE_LEVEL_WEAK   = 1U,
    NOISE_LEVEL_SEVERE = 2U
} NoiseLevel_t;

/* 固定8点参考模板，单位为“ADC码 - 固定基线2030”，可正可负。
 * 由SIGNAL_TEMPLATE_0..7配置；与adc_buffer[6..13]逐点对应。 */
static const int16_t g_signal_template[8] = {
    SIGNAL_TEMPLATE_0, SIGNAL_TEMPLATE_1, SIGNAL_TEMPLATE_2, SIGNAL_TEMPLATE_3,
    SIGNAL_TEMPLATE_4, SIGNAL_TEMPLATE_5, SIGNAL_TEMPLATE_6, SIGNAL_TEMPLATE_7
};

/* 模板能量Σ(T[i]^2)，启动时计算一次。作为幅度定标的分母；非Watch输出用变量。 */
static uint32_t g_signal_template_energy = 0U;
/* IIR内部Q8定点状态：真实幅度(x1000数值)再乘256保存，避免小幅变化被整数截断。 */
static int32_t g_signal_filter_q8 = 0;

/* -------- B. 每帧相关积分 / 10个有效帧的分组平均 -------- */
volatile int32_t  g_match_correlation = 0; /* 最新一次“有效帧”的8点相关积分S=Σ[(ADC-2030)*模板]；强干扰帧保持旧值。可为负。 */
volatile int64_t  g_match_correlation_sum10 = 0; /* 最近【已完成】一组10有效帧的相关积分总和ΣS；只在组满时刷新，不是正在累计的和。 */
volatile int32_t  g_match_correlation_avg10 = 0; /* 最近已完成10有效帧的相关积分均值=sum10/10；仅组满时更新。 */
volatile int32_t  g_match_amplitude_avg10_x1000 = 0; /* 最近已完成10帧的相对模板幅度=(平均相关积分*1000)/模板能量；1000≈模板等幅；可>1000或为负。 */
volatile int32_t  g_match_filtered_x1000 = 0; /* 将上一变量送入IIR得到的滤波幅度；每10有效帧更新1次；正常K=2、组内有弱干扰则K=8。 */
volatile uint32_t g_match_valid_frames = 0U; /* 累计通过干扰判定并参与相关积分的DMA帧总数（等级0或1）。 */
volatile uint32_t g_match_rejected_frames = 0U; /* 累计因强干扰（等级2）被跳过的DMA帧数；不会加入10帧组。 */
volatile uint8_t  g_match_frame_valid = 0U; /* 最近一帧是否被接受：1=正常/弱干扰并计算相关，0=强干扰拒绝；每帧更新。 */
volatile uint32_t g_match_accumulate_pending_count = 0U; /* 正在累计的本组有效帧数：通常0..9；第10个有效帧完成后清0。 */
volatile uint32_t g_match_accumulate_groups = 0U; /* 累计完成的10有效帧分组数；每组完成+1，可用来确认幅度/IIR是否持续刷新。 */

/* 内部临时累加器：本组尚未完成时的ΣS，组满后清0；与上面的sum10不同。 */
static int64_t g_match_sum_accumulator = 0;
/* 本组是否曾出现弱干扰帧：0=全正常、1=至少一次弱干扰；决定本组IIR和输出阈值档位。 */
static uint8_t g_match_group_has_weak = 0U;

/* -------- C. IIR后的可选8点截尾平均（每点=一次10帧分组的IIR结果） -------- */
#if SIGNAL_TRIMMED_MEAN_ENABLE
static int32_t g_output_trim_history[SIGNAL_TRIM_WINDOW_SIZE] = {0}; /* 最近8个IIR输出的循环缓存；不是14点ADC数据。 */
static uint8_t g_output_trim_write = 0U; /* 下次写入缓存的位置，循环0..7。 */
static uint8_t g_output_trim_count = 0U; /* 已积累的IIR历史数量，最大8；不足8时不启动输出判断。 */
#endif
volatile int32_t g_output_trimmed_x1000 = 0; /* 最终输出判定输入：开截尾时丢最高2、最低2后中间4值平均；关截尾时直接等于IIR。 */

/* -------- D. 最终有光/遮光输出：只在有新可用结果时判定 -------- */
volatile uint8_t g_output_light_state = 0U; /* 当前逻辑输出：0=遮光，1=有光；上电默认0。测试宏关闭时映射到PA12。 */
volatile uint8_t g_output_light_confirm = 0U; /* 当前处于遮光时，连续满足有光进入门槛的次数；达到5次切有光，之后清0。 */
volatile uint8_t g_output_dark_confirm = 0U; /* 当前处于有光时，连续满足遮光退出门槛的次数；达到5次切遮光，之后清0。 */
volatile uint8_t g_output_ready = 0U; /* 1=已得到可用于输出判断的截尾结果（或宏关闭时已有IIR）；一旦置1不会每组自动清0。 */
volatile uint32_t g_output_state_changes = 0U; /* 上电以来真实输出状态切换的次数；仅0↔1时+1，可检查抖动。 */
volatile uint8_t g_output_last_group_noise = 0U; /* 最近完成的10有效帧组的综合干扰档：0=全正常，1=本组至少1帧弱干扰；用于选IIR/输出阈值。 */

static void APP_SignalTemplateInit(void);
static int32_t APP_SignalCalculateCorrelation(const uint16_t *samples);
static void APP_SignalFilterUpdate(int32_t amplitude, uint8_t noise_level);
static uint8_t APP_SignalFrameCorrelationStage(const uint16_t *samples, int32_t *correlation);
static uint8_t APP_SignalAccumulateStage(int32_t correlation, uint8_t noise_level);
static void APP_SignalAmplitudeStage(void);
static void APP_OutputSetLight(uint8_t is_light);
static void APP_OutputUpdateDecision(int32_t value, uint8_t group_noise);
static uint8_t APP_OutputTrimUpdate(int32_t value);


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
/**
 * @brief 应用程序入口：按时钟、GPIO、TIM3、ADC/DMA、模板能量的顺序初始化。
 * @return int：正常运行不返回。
 * @note 先启动 ADC/DMA 等待触发，后启动 TIM3；while(1) 不执行算法。
 */
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
/**
 * @brief 对指定背景区计算相对于固定 ADC 基线的平均绝对偏差。
 * @param[in] samples：完整 ADC 帧指针，至少 BUFFER_SIZE 个 uint16_t
 * @param[out] baseline_mean：接收背景区实际算术均值的非空指针。
 * @return uint32_t：背景区相对 NOISE_BASELINE_REFERENCE 的平均绝对偏差，单位 ADC 码。
 * @note 遍历 ADC_BASELINE_START_INDEX..ADC_BASELINE_END_INDEX；返回值是固定中心绝对偏差，不是以该帧均值为中心的经典 MAD。
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
/**
 * @brief 将当前帧背景偏差直接分成正常、弱干扰或强干扰。
 * @param[in] mad：当前帧固定基线平均绝对偏差（ADC 码）。
 * @return uint8_t：0 正常；1 弱干扰；2 强干扰。
 * @note 依次比较强干扰门槛与弱干扰门槛；没有跨帧状态、迟滞或确认计数。
 */
static uint8_t APP_NoiseClassifyFrame(uint32_t mad)
{
    if (mad >= NOISE_MAD_SEVERE_THRESHOLD)
        return (uint8_t)NOISE_LEVEL_SEVERE;

    if (mad >= NOISE_MAD_WEAK_THRESHOLD)
        return (uint8_t)NOISE_LEVEL_WEAK;

    return (uint8_t)NOISE_LEVEL_NORMAL;
}

/* 阶段4：更新当前帧平均绝对偏差和分类，信号区仍只计算均值。 */
/**
 * @brief 计算一帧的背景指标、信号区均值并更新干扰调试输出。
 * @param[in] samples：完整14点 ADC 帧指针。
 * @return 无（void）；结果写入 g_baseline_average、g_signal_average、g_noise_metric、g_noise_level、g_noise_processed_frames。
 * @note 每次 DMA TC 调用一次；必须先于 APP_SignalFrameCorrelationStage 执行，以提供本帧噪声等级。
 */
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
/**
 * @brief 初始化固定参考模板的平方能量。
 * @return 无（void）；写入 g_signal_template_energy。
 * @note 计算 ΣT[i]²；若模板全零则进入 APP_ErrorHandler；仅在系统启动时调用一次。
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
/**
 * @brief 计算当前帧8点有效信号与固定模板的离散相关积分。
 * @param[in] samples：完整 ADC 帧，信号区为索引 ADC_SIGNAL_START_INDEX..ADC_SIGNAL_END_INDEX。
 * @return int32_t：Σ[(ADC[i]－NOISE_BASELINE_REFERENCE)×template[i]]，带符号。
 * @note 仅完成单帧乘加，不计算匹配幅度、不更新 IIR；要求模板与采样时序对齐。
 */
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

/* 输出驱动：PA12测试宏开启时绝不改变PA12输出状态；关闭时用于实际判定输出。
 * 只在输出状态真正变化时写GPIO，减小中断额外开销。
 */
/**
 * @brief 更新软件有光状态并按配置驱动 PA12。
 * @param[in] is_light：0 遮光、非0 有光。
 * @return 无（void）；写入 g_output_light_state；测试模式关闭时写 PA12 GPIO。
 * @note OUTPUT_TEST_ENABLE=1 时仅更新逻辑状态，不修改 PA12；电平极性由 OUTPUT_LIGHT_ACTIVE_HIGH 决定。
 */
static void APP_OutputSetLight(uint8_t is_light)
{
    g_output_light_state = is_light;
#if !OUTPUT_TEST_ENABLE
#if OUTPUT_LIGHT_ACTIVE_HIGH
    if (is_light) GPIOA->BSRR = GPIO_PIN_12;
    else          GPIOA->BRR  = GPIO_PIN_12;
#else
    if (is_light) GPIOA->BRR  = GPIO_PIN_12;
    else          GPIOA->BSRR = GPIO_PIN_12;
#endif
#endif
}

/* 每产生一次新IIR结果才确认一次，不按每次DMA中断计数。
 * 正常: OFF<=60、ON>=70; 弱干扰: OFF<=100、ON>=110。
 * 迟滞带内维持原输出并清除连续确认次数。
 */
/**
 * @brief 根据当前组滤波结果执行双迟滞及连续确认输出。
 * @param[in] value：最终判断幅度，单位为模板幅度×1000；[in] group_noise：当前10有效帧组噪声级别（0正常、1弱）。
 * @return 无（void）；可修改确认计数、g_output_light_state、g_output_state_changes 以及 PA12。
 * @note 上电默认遮光；有光按上限持续满足 OUTPUT_LIGHT_CONFIRM_COUNT 次确认，遮光按下限持续满足 OUTPUT_DARK_CONFIRM_COUNT 次确认。只在新10周期组且截尾结果已就绪时调用。
 */
static void APP_OutputUpdateDecision(int32_t value, uint8_t group_noise)
{
    const int32_t off_threshold = (group_noise == (uint8_t)NOISE_LEVEL_WEAK)
                                    ? OUTPUT_WEAK_OFF_THRESHOLD
                                    : OUTPUT_NORMAL_OFF_THRESHOLD;
    const int32_t hysteresis = (group_noise == (uint8_t)NOISE_LEVEL_WEAK)
                                    ? OUTPUT_WEAK_HYSTERESIS
                                    : OUTPUT_NORMAL_HYSTERESIS;
    const int32_t on_threshold = off_threshold + hysteresis;
    g_output_last_group_noise = group_noise;

    if (g_output_light_state == 0U)
    {
        g_output_dark_confirm = 0U;
        if (value >= on_threshold)
        {
            if (g_output_light_confirm < OUTPUT_LIGHT_CONFIRM_COUNT)
                ++g_output_light_confirm;
            if (g_output_light_confirm >= OUTPUT_LIGHT_CONFIRM_COUNT)
            {
                APP_OutputSetLight(1U);
                ++g_output_state_changes;
                g_output_light_confirm = 0U;
            }
        }
        else g_output_light_confirm = 0U;
    }
    else
    {
        g_output_light_confirm = 0U;
        if (value <= off_threshold)
        {
            if (g_output_dark_confirm < OUTPUT_DARK_CONFIRM_COUNT)
                ++g_output_dark_confirm;
            if (g_output_dark_confirm >= OUTPUT_DARK_CONFIRM_COUNT)
            {
                APP_OutputSetLight(0U);
                ++g_output_state_changes;
                g_output_dark_confirm = 0U;
            }
        }
        else g_output_dark_confirm = 0U;
    }
}

/* 8点截尾平均放在IIR之后。缓存满8组时，每次新IIR结果滚动更新。
 * 使用局部8元素数组插入排序，不改变历史数组或adc_buffer。
 */
/**
 * @brief 对新一组 IIR 幅度更新可选8值截尾滑动平均。
 * @param[in] value：最新 IIR 结果，单位为模板幅度×1000。
 * @return uint8_t：1 表示本次结果可用于输出判断；0 表示开启截尾后尚未积满8个 IIR 输出。
 * @note 开启时存入8值环形缓存、排序、去掉最低2值和最高2值、平均中间4值；关闭时直接传递 value。写入 g_output_trimmed_x1000、g_output_ready。
 */
static uint8_t APP_OutputTrimUpdate(int32_t value)
{
#if SIGNAL_TRIMMED_MEAN_ENABLE
    int32_t ordered[SIGNAL_TRIM_WINDOW_SIZE];
    int64_t sum = 0;
    uint32_t i, j;
    int32_t current;

    g_output_trim_history[g_output_trim_write] = value;
    g_output_trim_write = (uint8_t)((g_output_trim_write + 1U) % SIGNAL_TRIM_WINDOW_SIZE);
    if (g_output_trim_count < SIGNAL_TRIM_WINDOW_SIZE)
        ++g_output_trim_count;
    if (g_output_trim_count < SIGNAL_TRIM_WINDOW_SIZE)
        return 0U; /* 未满8组，不能拿初始化的0参与统计 */

    for (i = 0U; i < SIGNAL_TRIM_WINDOW_SIZE; ++i)
    {
        current = g_output_trim_history[i];
        j = i;
        while ((j > 0U) && (ordered[j - 1U] > current))
        {
            ordered[j] = ordered[j - 1U];
            --j;
        }
        ordered[j] = current;
    }
    for (i = SIGNAL_TRIM_DROP_EACH_SIDE;
         i < SIGNAL_TRIM_WINDOW_SIZE - SIGNAL_TRIM_DROP_EACH_SIDE; ++i)
        sum += ordered[i];
    g_output_trimmed_x1000 = (int32_t)(sum / (int32_t)(SIGNAL_TRIM_WINDOW_SIZE - 2U * SIGNAL_TRIM_DROP_EACH_SIDE));
#else
    g_output_trimmed_x1000 = value;
#endif
    g_output_ready = 1U;
    return 1U; /* 本次可以执行输出判断 */
}

/* 阶段5：仅对完成10帧后的匹配幅度做IIR，不再对每个单帧做IIR。
 * 弱干扰档K=8，正常档K=2。分组中只要出现弱干扰有效帧，
 * 整组使用较深的弱干扰档；强干扰帧根本不进入该组。
 */
/**
 * @brief 对10周期平均匹配幅度执行一阶定点 IIR 滤波。
 * @param[in] amplitude：10周期平均匹配幅度×1000；[in] noise_level：组干扰等级（0正常、1弱）。
 * @return 无（void）；更新内部 g_signal_filter_q8 与可观察的 g_match_filtered_x1000。
 * @note 正常系数 1/2，弱干扰系数 1/8；仅在10有效帧组结束后调用一次，并非每个 DMA 帧执行。
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

/* 阶段2：检查本帧是否可用，正常/弱干扰才计算相关积分。
 * 强干扰帧不修改原来的积分值，也不增加10帧组计数。
 */
/**
 * @brief 判断本帧是否可参与相干积累，并提取本帧相关积分。
 * @param[in] samples：完整 ADC 帧指针
 * @param[out] correlation：有效帧相关积分输出指针。
 * @return uint8_t：1 有效帧且已写入 *correlation；0 强干扰拒绝，此时 *correlation 不保证被修改。
 * @note 依赖 APP_NoiseAnalyzeFrame 已更新 g_noise_level；同时更新 g_match_frame_valid、g_match_valid_frames 或 g_match_rejected_frames。
 */
static uint8_t APP_SignalFrameCorrelationStage(const uint16_t *samples,
                                                int32_t *correlation)
{
    if (g_noise_level == (uint8_t)NOISE_LEVEL_SEVERE)
    {
        g_match_frame_valid = 0U;
        ++g_match_rejected_frames;
        return 0U;
    }

    *correlation = APP_SignalCalculateCorrelation(samples);
    g_match_correlation = *correlation;
    g_match_frame_valid = 1U;
    ++g_match_valid_frames;
    return 1U;
}

/* 阶段3：仅累计每个有效周期的相关积分，满10次才返回1。
 * 本阶段不计算匹配幅度、不调用IIR，也不触碰输出。
 * 标记本组是否出现弱干扰，供后续滤波和输出阈值选择。
 */
/**
 * @brief 累计有效帧相关积分并在满10帧时产生分组结果。
 * @param[in] correlation：本有效帧相关积分；[in] noise_level：本帧干扰等级（0正常、1弱）。
 * @return uint8_t：1 当前组已满，可以进行幅度/IIR/输出处理；0 尚未累计满。
 * @note 写入 g_match_correlation_sum10、g_match_correlation_avg10、g_output_last_group_noise、g_match_accumulate_pending_count、g_match_accumulate_groups；强干扰帧应在调用前排除。
 */
static uint8_t APP_SignalAccumulateStage(int32_t correlation, uint8_t noise_level)
{
    g_match_sum_accumulator += (int64_t)correlation;
    if (noise_level == (uint8_t)NOISE_LEVEL_WEAK)
        g_match_group_has_weak = 1U;

    ++g_match_accumulate_pending_count;
    if (g_match_accumulate_pending_count < SIGNAL_ACCUMULATE_CYCLES)
        return 0U;

    g_match_correlation_sum10 = g_match_sum_accumulator;
    g_match_correlation_avg10 = (int32_t)(g_match_sum_accumulator / SIGNAL_ACCUMULATE_CYCLES);
    g_output_last_group_noise = g_match_group_has_weak
                              ? (uint8_t)NOISE_LEVEL_WEAK : (uint8_t)NOISE_LEVEL_NORMAL;
    ++g_match_accumulate_groups;

    g_match_sum_accumulator = 0;
    g_match_accumulate_pending_count = 0U;
    g_match_group_has_weak = 0U;
    return 1U;
}

/* 阶段4：10帧完整时，积分均值除以固定模板能量，得到幅度x1000。 */
/**
 * @brief 将最近10帧相关积分均值换算为模板相对匹配幅度。
 * @param 无；隐式输入 g_match_correlation_avg10、g_signal_template_energy。
 * @return 无（void）；写入 g_match_amplitude_avg10_x1000。
 * @note 幅度 = 均值积分×1000/模板能量；只有 APP_SignalAccumulateStage 返回1后调用。幅度可大于1000或为负。
 */
static void APP_SignalAmplitudeStage(void)
{
    g_match_amplitude_avg10_x1000 = (int32_t)(
        ((int64_t)g_match_correlation_avg10 * SIGNAL_AMPLITUDE_SCALE)
        / (int64_t)g_signal_template_energy);
}

/* 普通GPIO配置：PA5、PA12推挽输出；PB5上拉输入。 */
/**
 * @brief 配置 PA5/PA12 输出和 PB5 上拉输入，并初始化输出电平。
 * @return 无（void）；配置 GPIO 寄存器。
 * @note PA12在测试模式下默认高电平；非测试模式按照 OUTPUT_LIGHT_ACTIVE_HIGH 设置初始遮光电平。
 */
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
    /* 测试关闭时PA12作为光状态输出；上电初始遮光。 */
#if OUTPUT_LIGHT_ACTIVE_HIGH
    GPIOA->BRR = GPIO_PIN_12;
#else
    GPIOA->BSRR = GPIO_PIN_12;
#endif
#endif
}

/* TIM3 24MHz/24=1MHz；ARR及两路CCR由随机周期函数动态预装载。 */
/**
 * @brief 初始化 TIM3 时基、PA6发射PWM、CH2比较触发和 TRGO。
 * @return 无（void）；配置 TIM3/GPIO。
 * @note TIM3 计数基准为 1 MHz；配置阶段不启动通道；CH1负责发射，CH2内部 OC2REF 驱动 ADC 触发。
 */
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
/**
 * @brief 配置 ADC1 单通道采样和 DMA1 Channel1 NORMAL 搬运。
 * @return 无（void）；初始化 ADC/DMA 句柄、外设和 DMA NVIC。
 * @note 本函数只配置，不开始采集；ADC外部触发源 TIM3 TRGO，每轮 DMA 采集 BUFFER_SIZE 个结果。
 */
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
/**
 * @brief 启动首轮 ADC/DMA 并等待 TIM3 硬件触发。
 * @return 无（void）；使 ADC/DMA 处于待触发状态。
 * @note 第一次使用 HAL_ADC_Start_DMA；关闭半传输中断，后续帧由 APP_AdcDmaFastRearm 寄存器方式重装。
 */
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

/**
 * @brief DMA NORMAL 本轮完成后快速重新装载 ADC/DMA。
 * @return 无（void）；写 DMA CCR/CNDTR/IFCR 及 ADC CFGR1/ISR/CR。
 * @note 由 DMA TC 中断调用；不复制 adc_buffer；调用前必须完成所有原始缓冲区读取。下一次 TRGO 前必须重新武装完成。
 */
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

/* DMA TC中断：明确列出各个算法阶段。
 * 1 背景区MAD逐帧分类；2 当前有效信号区相关积分；3 累计10个有效帧；
 * 4 计算积分均值对应的匹配幅度；5 两档IIR；6 可选8点截尾平均；
 * 7 两档迟滞及5次输出确认；8 DMA快速重装。
 * 无快照、主循环为空。阶段4～7仅在新的10周期组完成时执行。
 * 若截尾窗口尚未积满8组，则阶段7暂不执行。
 * 注意：先完成算法再重新武装DMA，须测量最坏中断耗时。
 */
/**
 * @brief ADC DMA完成中断：按显式阶段执行干扰检测、相干积分、滤波和输出。
 * @return 无（void）；更新背景/相关/累积/IIR/截尾/输出等全局结果并重装 DMA。
 * @note 每帧执行背景分类、有效性判断和必要的相关积分；每满10有效帧才执行幅度、IIR、截尾与判定；处理期间 adc_buffer 不会被复制。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {
        int32_t correlation = 0;
        uint8_t frame_valid;
        uint8_t group_ready = 0U;
        uint8_t output_ready;

#if OUTPUT_TEST_ENABLE
        GPIOA->BRR = GPIO_PIN_12;
#endif
        /* ① 每帧：背景区相对固定基线的MAD，更新干扰等级。 */
        APP_NoiseAnalyzeFrame(adc_buffer);

        /* ② 每帧：强干扰则拒绝；否则提取8点模板相关积分。 */
        frame_valid = APP_SignalFrameCorrelationStage(adc_buffer, &correlation);

        if (frame_valid != 0U)
        {
            /* ③ 每有效帧：累计相关积分；只有第10次返回完成标志。 */
            group_ready = APP_SignalAccumulateStage(correlation, g_noise_level);
        }

        if (group_ready != 0U)
        {
            /* ④ 每10有效帧：积分均值 -> 匹配幅度x1000。 */
            APP_SignalAmplitudeStage();

            /* ⑤ 每10有效帧：依据本组干扰等级执行K=2或K=8的IIR。 */
            APP_SignalFilterUpdate(g_match_amplitude_avg10_x1000,
                                   g_output_last_group_noise);

            /* ⑥ 每10有效帧：可选8点截尾平均（关闭时直接使用IIR）。 */
            output_ready = APP_OutputTrimUpdate(g_match_filtered_x1000);

            if (output_ready != 0U)
            {
                /* ⑦ 每个新结果：双迟滞+有光/无光各5次确认，控制PA12。 */
                APP_OutputUpdateDecision(g_output_trimmed_x1000,
                                         g_output_last_group_noise);
            }
        }

        /* ⑧ 每帧：全部算法结束后重新武装ADC/DMA等待下次TRGO。 */
        APP_AdcDmaFastRearm();
#if OUTPUT_TEST_ENABLE
        GPIOA->BSRR = GPIO_PIN_12;
#endif
    }
    /* 保持原版DMA TE错误处理策略不变。 */
}

/* 系统时钟：HSI 24MHz -> SYSCLK 24MHz -> HCLK 24MHz -> PCLK1 24MHz。 */
/**
 * @brief 配置 HSI 系统时钟和 AHB/APB1 分频。
 * @return 无（void）；更新 RCC 和 FLASH 相关配置。
 * @note 按原项目设定目标系统时钟24 MHz；HAL初始化失败进入 APP_ErrorHandler。
 */
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
/**
 * @brief 统一处理不可恢复的外设初始化或运行错误。
 * @return 无（void）；不会返回。
 * @note 进入无限循环，停止正常软件流程；未实现故障码或自动恢复。
 */
void APP_ErrorHandler(void)
{

  while (1)
  {
  }
}
