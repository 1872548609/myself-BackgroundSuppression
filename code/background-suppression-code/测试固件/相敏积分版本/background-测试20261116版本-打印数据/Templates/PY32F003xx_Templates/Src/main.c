
/*
 * PY32F003 背景抑制：相关积分独立测试版。
 *
 * 功能：
 * 1. TIM3随机/固定周期发射。
 * 2. ADC + DMA每帧采集25点。
 * 3. 正负窗口相关积分。
 * 4. N帧累计积分平均。
 * 5. 输出迟滞及连续确认。
 * 6. PA12输出极性切换。
 * 7. PA12 DMA中断耗时测试。
 * 8. RTT独立打印原始ADC、单帧积分和分组积分平均。
 *
 * 删除：
 * 前置面积、干扰判断、截尾平均、IIR及死区滤波。
 *
 * 保留原TIM3、ADC、DMA初始化和快速重装逻辑。
 */

#include "main.h"
#include <stdint.h>

/* =========================== RTT打印配置 =========================== */

/* RTT总开关。 */
#define APP_RTT_ENABLE                 1U

/* 三种打印功能独立控制。 */
#define APP_RTT_ADC_ENABLE             1U  /* 1=打印每帧原始ADC。 */
#define APP_RTT_CORR_ENABLE            0U  /* 1=打印每帧相关积分。 */
#define APP_RTT_AVG_ENABLE             0U  /* 1=打印N帧累计积分平均。 */

/* 原始ADC打印范围，打印正负积分原始值 */
#define APP_RTT_ADC_START              6U
#define APP_RTT_ADC_COUNT              10U

/* 总开关打开且至少开启一种打印功能时，才编译RTT代码。 */
#define APP_RTT_ACTIVE \
    (APP_RTT_ENABLE && \
    (APP_RTT_ADC_ENABLE || \
     APP_RTT_CORR_ENABLE || \
     APP_RTT_AVG_ENABLE))

#if APP_RTT_ACTIVE
#include "SEGGER_RTT.h"
#include <stdio.h>
#endif


/* ========================= 采样及发射配置 ========================== */

/* DMA每帧采集25个ADC点，下标0~24。 */
#define BUFFER_SIZE                   25U

/* 1=候选表随机周期；0=固定周期。 */
#define TRANSMIT_RANDOM_ENABLE         1U

/* 固定周期模式的TIM3周期，单位us。 */
#define FIXED_TRANSMIT_PERIOD        110U

/* 随机周期候选表元素数量。 */
#define RANDOM_PERIOD_COUNT           10U

/* TIM3初始化时的占位周期，单位us。 */
#define RANDOM_INITIAL_PERIOD        100U

/* CH1在每个周期末尾预留3us的PWM输出段。 */
#define RANDOM_PWM_OFF_US              3U

/* CH2在周期结束前7us产生ADC触发沿。 */
#define RANDOM_ADC_BEFORE_END_US       7U

#if TRANSMIT_RANDOM_ENABLE
#define TIM3_INITIAL_PERIOD          RANDOM_INITIAL_PERIOD
#else
#define TIM3_INITIAL_PERIOD          FIXED_TRANSMIT_PERIOD
#endif


/* ======================= 正负窗口相关积分参数 ====================== */

/*
 * 下标区间首尾均包含。
 * 正窗口权重+1，负窗口权重-1。
 * 两个窗口必须等长且不重叠。
 *
 * correlation =
 * ADC[6..10]之和 - ADC[11..15]之和。
 */

#define CORR_POS_START                 6U
#define CORR_POS_END                  10U

#define CORR_NEG_START                11U
#define CORR_NEG_END                  15U

#define CORR_POS_COUNT \
    (CORR_POS_END - CORR_POS_START + 1U)

#define CORR_NEG_COUNT \
    (CORR_NEG_END - CORR_NEG_START + 1U)

/* 1=开启N帧累计平均；0=直接使用单帧相关积分。 */
#define CORR_AVG_ENABLE                1U

/* 开启平均时，每组累计DMA帧数。 */
#define CORR_AVG_FRAME_COUNT           4U

/*
 * 编译期确定右移位数。
 * 使用移位代替整数除法。
 */

#if CORR_AVG_ENABLE

#if CORR_AVG_FRAME_COUNT == 2U

#define CORR_AVG_SHIFT                 1U

#elif CORR_AVG_FRAME_COUNT == 4U

#define CORR_AVG_SHIFT                 2U

#elif CORR_AVG_FRAME_COUNT == 8U

#define CORR_AVG_SHIFT                 3U

#elif CORR_AVG_FRAME_COUNT == 16U

#define CORR_AVG_SHIFT                 4U

#elif CORR_AVG_FRAME_COUNT == 32U

#define CORR_AVG_SHIFT                 5U

#elif CORR_AVG_FRAME_COUNT == 64U

#define CORR_AVG_SHIFT                 6U

#elif CORR_AVG_FRAME_COUNT == 128U

#define CORR_AVG_SHIFT                 7U

#elif CORR_AVG_FRAME_COUNT == 256U

#define CORR_AVG_SHIFT                 8U

#elif CORR_AVG_FRAME_COUNT == 512U

#define CORR_AVG_SHIFT                 9U

#elif CORR_AVG_FRAME_COUNT == 1024U

#define CORR_AVG_SHIFT                10U

#else

#error "CORR_AVG_FRAME_COUNT must be power of 2 (2..1024)"

#endif

#endif /* CORR_AVG_ENABLE */


/* =========================== 输出参数 ============================== */

/* 遮光阈值：积分平均值<=-10。 */
#define OUTPUT_OFF_THRESHOLD          (-10)

/* 迟滞量：有光阈值=-10+50=40。 */
#define OUTPUT_HYSTERESIS              50

#define OUTPUT_ON_THRESHOLD \
    (OUTPUT_OFF_THRESHOLD + OUTPUT_HYSTERESIS)

/* 连续3次满足有光阈值才确认有光。 */
#define OUTPUT_LIGHT_CONFIRM_COUNT     3U

/* 连续3次满足遮光阈值才确认遮光。 */
#define OUTPUT_DARK_CONFIRM_COUNT      3U

/* 1=有光输出高电平；0=有光输出低电平。 */
#define OUTPUT_LIGHT_ACTIVE_HIGH       1U

/*
 * PA12硬件测试模式。
 *
 * 0：PA12正常输出有光/遮光状态。
 *
 * 1：进入DMA中断时PA12拉低，
 *    DMA中断处理结束后PA12拉高。
 *    使用示波器测量低电平宽度，
 *    观察中断处理耗时。
 */
#define OUTPUT_TEST_ENABLE             0U


/* =========================== 编译期检查 ============================ */

/* 开关宏检查。 */
#if (CORR_AVG_ENABLE != 0U) && (CORR_AVG_ENABLE != 1U)
#error "CORR_AVG_ENABLE must be 0 or 1"
#endif

#if (APP_RTT_ENABLE > 1U) || \
    (APP_RTT_ADC_ENABLE > 1U) || \
    (APP_RTT_CORR_ENABLE > 1U) || \
    (APP_RTT_AVG_ENABLE > 1U) || \
    (TRANSMIT_RANDOM_ENABLE > 1U) || \
    (OUTPUT_LIGHT_ACTIVE_HIGH > 1U) || \
    (OUTPUT_TEST_ENABLE > 1U)

#error "Enable macros must be 0 or 1"

#endif

/* 相关积分窗口起止位置检查。 */
#if (CORR_POS_START > CORR_POS_END) || \
    (CORR_NEG_START > CORR_NEG_END)

#error "Correlation window start must be <= end"

#endif

/* 相关积分窗口不能超过DMA缓冲区。 */
#if (CORR_POS_END >= BUFFER_SIZE) || \
    (CORR_NEG_END >= BUFFER_SIZE)

#error "Correlation index exceeds DMA buffer"

#endif

/* 正负窗口不能重叠。 */
#if !((CORR_POS_END < CORR_NEG_START) || \
      (CORR_NEG_END < CORR_POS_START))

#error "Correlation windows must not overlap"

#endif

/* 正负窗口点数必须一致。 */
#if (CORR_POS_COUNT != CORR_NEG_COUNT)

#error "Correlation windows must be equal length"

#endif

#if CORR_AVG_ENABLE
/* 防止积分累计值超过int32_t范围。 */
#if ((CORR_POS_COUNT * 4095ULL * \
      CORR_AVG_FRAME_COUNT) > 2147483647ULL)

#error "Correlation accumulation exceeds int32_t"

#endif
#endif /* CORR_AVG_ENABLE */

/* 输出迟滞必须大于0。 */
#if (OUTPUT_HYSTERESIS <= 0)

#error "OUTPUT_HYSTERESIS must be positive"

#endif

/* 连续确认次数必须在uint8_t范围内。 */
#if (OUTPUT_LIGHT_CONFIRM_COUNT < 1U) || \
    (OUTPUT_LIGHT_CONFIRM_COUNT > 255U) || \
    (OUTPUT_DARK_CONFIRM_COUNT < 1U) || \
    (OUTPUT_DARK_CONFIRM_COUNT > 255U)

#error "Output confirmation count must be 1..255"

#endif

/* 发射周期必须大于ADC提前触发时间和PWM预留时间。 */
#if (TIM3_INITIAL_PERIOD <= RANDOM_PWM_OFF_US) || \
    (TIM3_INITIAL_PERIOD <= RANDOM_ADC_BEFORE_END_US)

#error "Invalid TIM3 initial period"

#endif

/* RTT原始ADC打印窗口检查。 */
#if APP_RTT_ACTIVE && APP_RTT_ADC_ENABLE

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

/* 当前帧的原始相关积分值，可为负数。 */
volatile int32_t g_match_correlation = 0;

/* 最近一次有效积分结果：N帧平均或单帧积分。 */
volatile int32_t g_match_average_integral = 0;

/* 当前分组已经累计的帧数。 */
volatile uint16_t g_match_average_count = 0U;

/* 已经处理的有效相关积分帧数。 */
volatile uint32_t g_match_valid_frames = 0U;

/* 已经产生的有效积分结果总数。 */
volatile uint32_t g_match_average_groups = 0U;

/* 本次DMA中断是否产生新的有效积分结果。 */
volatile uint8_t g_match_average_ready = 0U;

/* 当前分组的积分累计和，满组后清零。 */
#if CORR_AVG_ENABLE
static int32_t g_match_average_sum = 0;
#endif


/* ============================ 输出状态 ============================= */

/* 当前逻辑光状态：0=遮光，1=有光。 */
volatile uint8_t g_output_light_state = 0U;

/* 遮光状态下，连续满足有光阈值的次数。 */
volatile uint8_t g_output_light_confirm = 0U;

/* 有光状态下，连续满足遮光阈值的次数。 */
volatile uint8_t g_output_dark_confirm = 0U;

/* 上电至今输出状态发生切换的总次数。 */
volatile uint32_t g_output_state_changes = 0U;


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


/* ------------------------ 原始ADC打印 ------------------------ */

#if APP_RTT_ADC_ENABLE

/*
 * 格式：
 * A,frame,adc0,adc1,...,adc7
 *
 * 每次DMA完成后打印一次。
 * 实际打印范围由APP_RTT_ADC_START和COUNT决定。
 */
static void APP_RTT_SendADC(const uint16_t *samples)
{
    /*
     * 每个12位ADC最多4位数字，加一个逗号。
     * 额外32字节用于标识、帧号及换行。
     */
    char line[32U + APP_RTT_ADC_COUNT * 5U];

    uint32_t i;
    uint32_t used;
    int len;

    /* 写入数据类型标识和帧号。 */
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

    /* 依次写入原始ADC值。 */
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

    /* 添加换行符。 */
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

#endif /* APP_RTT_ADC_ENABLE */


/* ------------------------ 单帧积分打印 ------------------------ */

#if APP_RTT_CORR_ENABLE

/*
 * 格式：
 * F,frame,corr_raw
 *
 * 每次DMA完成后打印一次。
 */
static void APP_RTT_SendCorrelation(void)
{
    char line[48];

    int len = snprintf(
        line,
        sizeof(line),
        "F,%lu,%ld\r\n",
        (unsigned long)g_signal_frames,
        (long)g_match_correlation);

    APP_RTT_WriteLine(
        line,
        len,
        sizeof(line));
}

#endif /* APP_RTT_CORR_ENABLE */


/* ------------------------ 分组积分打印 ------------------------ */

#if APP_RTT_AVG_ENABLE

/*
 * 格式：
 * G,group,corr_avg,light
 *
 * 开启平均每N帧打印一次；关闭平均每帧打印一次。
 * light为本次输出判断后的逻辑状态。
 */
static void APP_RTT_SendAverage(void)
{
    char line[48];

    int len = snprintf(
        line,
        sizeof(line),
        "G,%lu,%ld,%u\r\n",
        (unsigned long)g_match_average_groups,
        (long)g_match_average_integral,
        (unsigned int)g_output_light_state);

    APP_RTT_WriteLine(
        line,
        len,
        sizeof(line));
}

#endif /* APP_RTT_AVG_ENABLE */

#endif /* APP_RTT_ACTIVE */


/* ========================== TIM3随机周期 =========================== */

#if TRANSMIT_RANDOM_ENABLE

/*
 * 随机周期候选表。
 * 当前候选值全部为100us，因此实际发射周期固定为100us。
 * 保留随机抽取功能，方便后续修改候选表。
 */
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


/* TIM3更新中断，仅随机周期模式使用。 */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        /* 清除更新中断标志。 */
        TIM3->SR = (uint32_t)~TIM_SR_UIF;

        /* 准备下一个发射周期。 */
        TIM3_QueuePeriod(Random_NextPeriod());
    }
}

#endif /* TRANSMIT_RANDOM_ENABLE */


/* ========================== 单帧相关积分 =========================== */

/*
 * 正负窗口求和差分。
 *
 * 正窗口点数与负窗口点数一致，
 * 因此共同直流基线可以自动抵消。
 *
 * 当前配置：
 * ADC[6..10]求和 - ADC[11..15]求和。
 */
static int32_t APP_SignalCalculateCorrelation(
    const uint16_t *samples)
{
    uint32_t i;

    int32_t pos = 0;
    int32_t neg = 0;

    /* 正窗口累加。 */
    for (i = CORR_POS_START;
         i <= CORR_POS_END;
         ++i)
    {
        pos += samples[i];
    }

    /* 负窗口累加。 */
    for (i = CORR_NEG_START;
         i <= CORR_NEG_END;
         ++i)
    {
        neg += samples[i];
    }

    /* 正积分减负积分。 */
    return pos - neg;
}


/* ========================== 带符号移位平均 ========================= */

/*
 * 正数直接右移。
 *
 * 负数先转为正数，右移后恢复符号，
 * 保证平均值按照向零截断。
 *
 * 例如：
 *  7 / 4 =  1
 * -7 / 4 = -1
 */
#if CORR_AVG_ENABLE
static int32_t APP_CorrelationShiftAverage(int32_t sum)
{
    if (sum < 0)
    {
        return -((-sum) >> CORR_AVG_SHIFT);
    }

    return sum >> CORR_AVG_SHIFT;
}
#endif /* CORR_AVG_ENABLE */


/* ========================== N帧积分累计平均 ======================= */

/*
 * 每次DMA完成调用一次。
 * 关闭平均时直接输出单帧积分并返回1。
 *
 * 开启平均且未满N帧：
 * 只累计相关积分，返回0。
 *
 * 满N帧：
 * 计算积分平均值；
 * 清除累计和及帧计数；
 * 分组编号加1；
 * 返回1，通知输出判断。
 */
static uint8_t APP_CorrelationAverageUpdate(
    int32_t correlation)
{
#if CORR_AVG_ENABLE
    /* 累计当前帧相关积分。 */
    g_match_average_sum += correlation;

    ++g_match_average_count;

    /* 未满N帧，不产生新的平均结果。 */
    if (g_match_average_count < CORR_AVG_FRAME_COUNT)
    {
        return 0U;
    }

    /* 满N帧，计算相关积分平均值。 */
    g_match_average_integral =
        APP_CorrelationShiftAverage(
            g_match_average_sum);

    /* 清除本组累计状态。 */
    g_match_average_sum = 0;
    g_match_average_count = 0U;

#else
    /* 关闭平均：每帧直接产生有效积分结果。 */
    g_match_average_integral = correlation;
    g_match_average_count = 0U;
#endif

    /* 开启平均每N帧加1；关闭平均每帧加1。 */
    ++g_match_average_groups;

    return 1U;
}


/* ========================== 输出电平控制 =========================== */

/*
 * 保存逻辑光状态。
 *
 * 正常模式：
 * 根据OUTPUT_LIGHT_ACTIVE_HIGH配置PA12输出极性。
 *
 * 测试模式：
 * 只更新逻辑光状态，不修改PA12。
 * PA12由DMA中断测试代码控制。
 */
static void APP_OutputSetLight(uint8_t light)
{
    g_output_light_state = light;

#if !OUTPUT_TEST_ENABLE

#if OUTPUT_LIGHT_ACTIVE_HIGH

    /* 有光高有效。 */
    if (light != 0U)
    {
        GPIOA->BSRR = GPIO_PIN_12;
    }
    else
    {
        GPIOA->BRR = GPIO_PIN_12;
    }

#else

    /* 有光低有效。 */
    if (light != 0U)
    {
        GPIOA->BRR = GPIO_PIN_12;
    }
    else
    {
        GPIOA->BSRR = GPIO_PIN_12;
    }

#endif

#endif
}


/* ========================== 输出状态判断 =========================== */

/*
 * 使用有效积分结果进行判断（N帧平均或单帧积分）。
 *
 * 当前遮光：
 * 连续3组 >= OUTPUT_ON_THRESHOLD，确认有光。
 *
 * 当前有光：
 * 连续3组 <= OUTPUT_OFF_THRESHOLD，确认遮光。
 *
 * 位于迟滞区间时保持原来的输出状态。
 */
static void APP_OutputUpdateDecision(int32_t value)
{
    if (g_output_light_state == 0U)
    {
        /* 当前为遮光状态。 */
        g_output_dark_confirm = 0U;

        /* 判断是否满足有光阈值。 */
        if (value >= OUTPUT_ON_THRESHOLD)
        {
            ++g_output_light_confirm;

            /* 连续确认次数达到要求。 */
            if (g_output_light_confirm >=
                OUTPUT_LIGHT_CONFIRM_COUNT)
            {
                g_output_light_confirm = 0U;

                APP_OutputSetLight(1U);

                ++g_output_state_changes;
            }
        }
        else
        {
            /* 未连续满足，清除确认计数。 */
            g_output_light_confirm = 0U;
        }
    }
    else
    {
        /* 当前为有光状态。 */
        g_output_light_confirm = 0U;

        /* 判断是否满足遮光阈值。 */
        if (value <= OUTPUT_OFF_THRESHOLD)
        {
            ++g_output_dark_confirm;

            /* 连续确认次数达到要求。 */
            if (g_output_dark_confirm >=
                OUTPUT_DARK_CONFIRM_COUNT)
            {
                g_output_dark_confirm = 0U;

                APP_OutputSetLight(0U);

                ++g_output_state_changes;
            }
        }
        else
        {
            /* 未连续满足，清除确认计数。 */
            g_output_dark_confirm = 0U;
        }
    }
}


/* ========================== DMA完成中断 ============================ */

/*
 * DMA每完成25个ADC点后进入一次中断。
 *
 * 执行顺序：
 *
 * 1. PA12测试引脚拉低（测试模式）。
 * 2. 保存需要打印的原始ADC。
 * 3. 计算单帧相关积分。
 * 4. 累计N帧积分并求平均。
 * 5. 完整分组进行输出判断。
 * 6. 根据宏执行RTT打印。
 * 7. DMA快速重装。
 * 8. PA12测试引脚拉高（测试模式）。
 */
void DMA1_Channel1_IRQHandler(void)
{
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)
    {
        int32_t correlation;

#if APP_RTT_ACTIVE && APP_RTT_ADC_ENABLE

        uint16_t rtt_adc_snapshot[APP_RTT_ADC_COUNT];
        uint32_t i;

#endif


        /* ---------------- PA12中断耗时测试 ---------------- */

#if OUTPUT_TEST_ENABLE

        /* 进入DMA处理时拉低。 */
        GPIOA->BRR = GPIO_PIN_12;

#endif


        /* ---------------- 原始ADC快照 --------------------- */

#if APP_RTT_ACTIVE && APP_RTT_ADC_ENABLE

        /*
         * 必须在DMA重装之前复制原始ADC数据。
         * 防止后续打印时缓冲区被下一帧覆盖。
         */
        for (i = 0U; i < APP_RTT_ADC_COUNT; ++i)
        {
            rtt_adc_snapshot[i] =
                adc_buffer[APP_RTT_ADC_START + i];
        }

#endif


        /* ---------------- 单帧相关积分 --------------------- */

        ++g_signal_frames;

        correlation =
            APP_SignalCalculateCorrelation(adc_buffer);

        g_match_correlation = correlation;

        ++g_match_valid_frames;


        /* ---------------- 积分结果更新 --------------------- */

        g_match_average_ready =
            APP_CorrelationAverageUpdate(correlation);


        /* ---------------- 输出状态判断 --------------------- */

        if (g_match_average_ready != 0U)
        {
            /*
             * 不再经过截尾平均、IIR及死区。
             * 直接使用有效积分结果判断输出。
             */
            APP_OutputUpdateDecision(
                g_match_average_integral);
        }

  /* ---------------- RTT打印 ------------------------- */

#if APP_RTT_ACTIVE

#if APP_RTT_ADC_ENABLE

        /* 每帧打印原始ADC快照。 */
        APP_RTT_SendADC(rtt_adc_snapshot);

#endif

#if APP_RTT_CORR_ENABLE

        /* 每帧打印原始相关积分。 */
        APP_RTT_SendCorrelation();

#endif

#if APP_RTT_AVG_ENABLE

        /* 开启平均每N帧打印；关闭平均每帧打印。 */
        if (g_match_average_ready != 0U)
        {
            APP_RTT_SendAverage();
        }

#endif

#endif /* APP_RTT_ACTIVE */

        /* ---------------- DMA快速重装 --------------------- */

        APP_AdcDmaFastRearm();


      
        /* ---------------- PA12中断耗时测试 ---------------- */

#if OUTPUT_TEST_ENABLE

        /* 所有DMA处理及RTT打印结束后拉高。 */
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

    /* 系统时钟配置为24MHz。 */
    APP_SystemClockConfig();


#if APP_RTT_ACTIVE

    /* 初始化SEGGER RTT。 */
    SEGGER_RTT_Init();

    /* 缓冲区不足时直接丢弃，不阻塞等待。 */
    SEGGER_RTT_SetFlagsUpBuffer(
        0,
        SEGGER_RTT_MODE_NO_BLOCK_SKIP);

#endif


    /* GPIO初始化。 */
    GPIO_Init();

    /* TIM3 CH1 PWM及CH2 ADC触发初始化。 */
    TMR3_OC_Init();

    /* ADC及DMA初始化。 */
    APP_AdcInit();

    /* 先准备ADC和DMA，再启动定时器。 */
    APP_AdcStartDMA();


    /* ---------------- 随机/固定周期启动 ---------------- */

#if TRANSMIT_RANDOM_ENABLE

    /* 装载第一个随机周期。 */
    TIM3_QueuePeriod(Random_NextPeriod());

    /* 软件更新，使首周期配置立即生效。 */
    TIM3->EGR = TIM_EGR_UG;

    /* 清除软件更新产生的UIF。 */
    TIM3->SR = (uint32_t)~TIM_SR_UIF;

    /* 准备下一周期配置。 */
    TIM3_QueuePeriod(Random_NextPeriod());

    /* 配置TIM3更新中断。 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);

    HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);

    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    /* 允许TIM3产生更新中断。 */
    TIM3->DIER |= TIM_DIER_UIE;

#else

    /* 固定周期模式。 */
    TIM3->EGR = TIM_EGR_UG;

    TIM3->SR = (uint32_t)~TIM_SR_UIF;

    /* 固定周期不需要TIM3更新中断。 */
    TIM3->DIER &= ~TIM_DIER_UIE;

#endif


    /* 先启动CH2 ADC触发通道。 */
    if (HAL_TIM_OC_Start(
            &Tim3Handle,
            TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    /* 再启动CH1发射PWM通道。 */
    if (HAL_TIM_OC_Start(
            &Tim3Handle,
            TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    while (1)
    {
        /*
         * 保留原设计：
         * 采样、积分、输出及RTT均在中断中处理。
         */
    }
}


/* =========================== GPIO初始化 ============================ */

/*
 * PA5：普通推挽输出，默认高电平。
 * PA12：有光/遮光输出或中断耗时测试。
 * PB5：上拉输入。
 */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA5、PA12配置为普通推挽输出。 */
    GPIO_InitStruct.Pin =
        GPIO_PIN_5 | GPIO_PIN_12;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* PB5配置为上拉输入。 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /* PA5默认输出高电平。 */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        GPIO_PIN_SET);


    /* ---------------- PA12初始电平 ---------------- */

#if OUTPUT_TEST_ENABLE

    /*
     * 测试模式：
     * PA12初始高电平，中断处理期间拉低。
     */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_12,
        GPIO_PIN_SET);

#else

#if OUTPUT_LIGHT_ACTIVE_HIGH

    /* 有光高有效，上电遮光状态为低电平。 */
    GPIOA->BRR = GPIO_PIN_12;

#else

    /* 有光低有效，上电遮光状态为高电平。 */
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

    /* PA6配置为TIM3 CH1 PWM输出。 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* ---------------- TIM3基本配置 ---------------- */

    Tim3Handle.Instance = TIM3;

    /* ARR初始值，计数从0开始。 */
    Tim3Handle.Init.Period =
        TIM3_INITIAL_PERIOD - 1U;

    /* 24MHz / (23+1) = 1MHz。 */
    Tim3Handle.Init.Prescaler = 24 - 1;

    Tim3Handle.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    Tim3Handle.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    Tim3Handle.Init.RepetitionCounter = 1 - 1;

    /* ARR预装载开启。 */
    Tim3Handle.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* ---------------- CH1发射PWM ---------------- */

    sConfig.OCMode = TIM_OCMODE_PWM1;

    sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

    sConfig.OCFastMode = TIM_OCFAST_ENABLE;

    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;

    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

    /* 周期末尾预留3us发射段。 */
    sConfig.Pulse =
        TIM3_INITIAL_PERIOD - RANDOM_PWM_OFF_US;

    if (HAL_TIM_OC_ConfigChannel(
            &Tim3Handle,
            &sConfig,
            TIM_CHANNEL_1) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* ---------------- CH2 ADC触发 ---------------- */

    sConfig.OCMode = TIM_OCMODE_PWM2;

    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

    sConfig.OCFastMode = TIM_OCFAST_ENABLE;

    sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;

    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

    /* 周期结束前7us产生ADC触发沿。 */
    sConfig.Pulse =
        TIM3_INITIAL_PERIOD - RANDOM_ADC_BEFORE_END_US;

    if (HAL_TIM_OC_ConfigChannel(
            &Tim3Handle,
            &sConfig,
            TIM_CHANNEL_2) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* CCR1、CCR2开启预装载。 */
    TIM3->CCMR1 |=
        TIM_CCMR1_OC1PE |
        TIM_CCMR1_OC2PE;


    /* ---------------- TIM3 TRGO配置 ---------------- */

    {
        TIM_MasterConfigTypeDef master = {0};

        /* CH2内部OC2REF作为TRGO触发源。 */
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


    /* ---------------- ADC校准 ---------------- */

    if (HAL_ADCEx_Calibration_Start(
            &AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* ---------------- ADC基本配置 ---------------- */

    AdcHandle.Instance = ADC1;

    /* ADC同步PCLK，不分频。 */
    AdcHandle.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV1;

    /* 12位ADC，范围0~4095。 */
    AdcHandle.Init.Resolution =
        ADC_RESOLUTION_12B;

    /* 数据右对齐。 */
    AdcHandle.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    /* 单通道，保留原扫描方向。 */
    AdcHandle.Init.ScanConvMode =
        ADC_SCAN_DIRECTION_BACKWARD;

    /* 关闭自动等待。 */
    AdcHandle.Init.LowPowerAutoWait = DISABLE;

    /* 外部触发后连续转换。 */
    AdcHandle.Init.ContinuousConvMode = ENABLE;

    AdcHandle.Init.DiscontinuousConvMode = DISABLE;

    /* TIM3 TRGO触发。 */
    AdcHandle.Init.ExternalTrigConv =
        ADC_EXTERNALTRIGCONV_T3_TRGO;

    /* 上升沿触发。 */
    AdcHandle.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_RISING;

    /* 保留原DMA请求配置。 */
    AdcHandle.Init.DMAContinuousRequests = DISABLE;

    /* ADC溢出时使用新数据覆盖旧数据。 */
    AdcHandle.Init.Overrun =
        ADC_OVR_DATA_OVERWRITTEN;

    /* 采样保持时间3.5个ADC时钟周期。 */
    AdcHandle.Init.SamplingTimeCommon =
        ADC_SAMPLETIME_3CYCLES_5;

    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* ---------------- ADC通道配置 ---------------- */

    sConfig.Channel = ADC_CHANNEL_0;

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

    if (HAL_ADC_ConfigChannel(
            &AdcHandle,
            &sConfig) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* ---------------- DMA配置 ---------------- */

    /* 保留原DMA请求映射。 */
    HAL_SYSCFG_DMA_Req(0);

    __HAL_RCC_DMA_CLK_ENABLE();

    hdma_adc.Instance = DMA1_Channel1;

    /* ADC外设到内存。 */
    hdma_adc.Init.Direction =
        DMA_PERIPH_TO_MEMORY;

    /* 外设地址固定。 */
    hdma_adc.Init.PeriphInc =
        DMA_PINC_DISABLE;

    /* 内存地址自动递增。 */
    hdma_adc.Init.MemInc =
        DMA_MINC_ENABLE;

    /* 外设数据宽度16位。 */
    hdma_adc.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_HALFWORD;

    /* 内存数据宽度16位。 */
    hdma_adc.Init.MemDataAlignment =
        DMA_MDATAALIGN_HALFWORD;

    /* DMA普通模式，满25点后完成。 */
    hdma_adc.Init.Mode = DMA_NORMAL;

    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;

    HAL_DMA_Init(&hdma_adc);

    /* 建立ADC和DMA句柄关联。 */
    __HAL_LINKDMA(
        &AdcHandle,
        DMA_Handle,
        hdma_adc);

    /* DMA完成中断配置。 */
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

    /* 关闭半传输中断，只使用完成中断。 */
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
    /* 1. 停止DMA通道。 */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    /* 2. 临时关闭ADC DMA请求。 */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

    /* 3. 清除DMA通道1中断标志。 */
    DMA1->IFCR = DMA_IFCR_CGIF1;

    /* 4. 重新装载25点传输长度。 */
    DMA1_Channel1->CNDTR = BUFFER_SIZE;

    /* 5. 重新开启DMA通道。 */
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    /* 6. 重新允许ADC DMA请求。 */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;

    /* 7. 清除ADC溢出标志。 */
    ADC1->ISR = ADC_ISR_OVR;

    /* 8. 保持ADC使能。 */
    ADC1->CR |= ADC_CR_ADEN;

    /* 9. 沿用原ADC启动策略。 */
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

    /* 开启HSI。 */
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

#if defined(RCC_HSIDIV_SUPPORT)

    /* HSI不分频。 */
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;

#endif

    /* HSI选择24MHz校准档。 */
    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_24MHz;

    /* 关闭HSE和LSI。 */
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;

    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        APP_ErrorHandler();
    }


    /* ---------------- 系统时钟分频 ---------------- */

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1;

    /* SYSCLK直接使用HSI。 */
    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    /* AHB不分频。 */
    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    /* APB1不分频。 */
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
