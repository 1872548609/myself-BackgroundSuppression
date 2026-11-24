/* B20 背景抑制检测程序 · V2 功能与新相位合并版
 * 先看 main() 的启动顺序，再看 DMA1_Channel1_IRQHandler() 如何交付一帧。
 * 算法主线在 process_frame()：同步取点 → 峰谷差 → 三帧中值 → 十六帧滑动平均 → 标定或判决 → PA12。
 * 主循环负责调试输出。检测计算由 DMA 中断驱动，即使主循环尚在等待启动自检，中断也会处理帧。 */
#include "main.h"
#include <stdint.h>

/* ======================= 采样与发射 ======================= */

/* 这三个数描述同一轮 TIM3 计数。周期 50 µs，CH1 比较点为 47，CH2 比较点为 35。
 * CH2 把触发事件送到芯片内部的 ADC；触发沿与第一个采样结果之间还存在延迟。 */
#define EMIT_PERIOD_US        50U      /* 发射周期, TIM3 计数 1MHz */
#define EMIT_ON_US            47U      /* CH1 比较值，单位 µs；对应 PA6 的有效电平持续时间 */
#define CH2_TRIG_US           35U      /* CH2 触发位置为 35 µs；原作者估计首样本约在 37.9 µs */

/* 按作者配置推算，16 个 ADC 时钟产生一点，24 MHz 下约为 0.667 µs/点。
 * FRAME_LEN 手工设为 75；修改 ADC_CONV_CYCLES 不会自动重算帧长或配置硬件。 */
#define ADC_CONV_CYCLES       16U      /* 3.5 采样 + 12.5 转换 */
#define FRAME_LEN             75U      /* 50us / (16/24MHz) = 75, 必须整除 */
#define FRAME_LEN_2           (FRAME_LEN * 2U)

/* 峰/谷取样窗口(帧内下标)。V2 实测新相位：谷中心 20、峰中心 25。
 * 下标从 0 开始。代码直接读取这六个固定位置，选窗是否准确可通过 RTT 的 P 行核对。 */
#define PEAK0                 24U
#define PEAK1                 25U
#define PEAK2                 26U
#define TRO0                  19U
#define TRO1                  20U
#define TRO2                  21U

/* ======================= 判决参数 ======================= */

#define MA_N                  16U      /* 滑动窗口长度，必须保持为 2 的幂 */
#define MA_SHIFT              4U       /* 稳态时用右移 4 位代替除以 16 */

#define K_ON                  6        /* 置位: n > max(K_ON*sigma, THR_MIN) */
#define K_OFF                 3        /* 复位: n < max(K_OFF*sigma, REL_MIN) */

#define THR_MIN_CODES         100      /* 单对峰谷的对比度参考值；12 位、参考电压 5 V 时约 1.22 mV/码 */
/* 三个峰值分别比三个谷值平均高 100 码时，差分总分为 300。电压换算取决于实际参考电压。 */
#define THR_MIN               (THR_MIN_CODES * 3)   /* score 是三对三之和 */
#define REL_MIN               (THR_MIN * 2 / 3)     /* 最小释放门限，当前为 200 */

#define DEBOUNCE_N            16U      /* 连续 16 帧满足条件后才切换状态 */

/*
 * 定点技巧: base/sigma 用 Q8 存放(<<8)。
 * 内部以分数的 1/256 为一个存储单位，便于逐帧累计小变化。
 * 例如背景 80 分存为 20480，读回整数分数时右移 8 位。
 */
#define QSH                   8
#define BASE_DOWN_SHIFT       6U       /* 下降快: tau ≈ 64 帧 ≈ 3ms */
#define BASE_UP_SHIFT         10U      /* 上升慢: tau ≈ 1024 帧 ≈ 50ms */
#define SIG_SHIFT             8U       /* 噪声估计速率: tau ≈ 256 帧 */
#define SIGMA_MIN             8        /* 噪声下限(码) */
#define SIGMA_MAX             400      /* 噪声上限, 防止一次扰动把阈值顶飞 */
/* 当前判决直接比较 a < thr；调整 QUIET_GATE 的值不会改变这一条件。 */
#define QUIET_GATE            2        /* 保留: |n| < thr 才允许学习 */

#define CAL_FRAMES            256U     /* 上电标定帧数；疑似有物的样本不写入背景 */

#define STARTUP_MIN_FRAMES    8U       /* 启动自检: 认为 ADC 已跑起来的帧数门槛 */
#define STARTUP_TIMEOUT_MS    100U     /* 启动自检超时(HAL_GetTick), 超时进 ErrorHandler */

#define OUT_ACTIVE_HIGH       1U       /* 1: 有物输出高 */
#define CALIB_DUMP            1U       /* 启用平均轮廓统计及主循环 RTT 打印；中断负责抽帧累加 */

/* 预处理开关为 0 时，省去 RTT 头文件、波形累加器和打印代码。判决仍由 DMA 中断执行。 */
#if CALIB_DUMP
#include "SEGGER_RTT.h"
#include <stdio.h>
#endif

/* ======================= 状态 ======================= */

static uint16_t adc_frame[FRAME_LEN_2];    /* DMA 循环写的双缓冲 */

/* 这些静态变量上电初始化为零。g_frames 在每次处理完一帧后递增；主程序用它判断采样是否启动。 */
static volatile uint32_t g_frames;
static int32_t  g_base_q8;                 /* 背景电平, Q8 */
static int32_t  g_sigma_q8 = SIGMA_MIN << QSH; /* 噪声估计，上电先使用安全下限 */
static uint8_t  g_state;                   /* 0=无物 1=有物 */
static uint8_t  g_on_cnt;                  /* 连续满足有物条件的帧数 */
static uint8_t  g_off_cnt;                 /* 连续满足释放条件的帧数 */

/* 三个历史槽保存最近的帧分数；g_h3i 指向下一个要替换的槽，g_h3_ready 标记首次填充完成。 */
static int32_t  g_h3[3];
static uint8_t  g_h3i;
static uint8_t  g_h3_ready;                /* 0: 首帧尚未到达(避免静态 0 值污染 med3) */
/* g_ring 存中值滤波后的结果，g_ring_sum 保存窗口总和；g_ri 是下一写入位置，g_warm 是有效项数。 */
static int32_t  g_ring[MA_N];
static int32_t  g_ring_sum;
static uint8_t  g_ri;
static uint8_t  g_warm;                    /* 环形缓冲填充计数 */

#if CALIB_DUMP
/*
 * 每 4 帧抽取 1 帧，逐下标累加。按设计速率，相邻半满/全满事件间隔约 50 µs。
 * 主循环每 ~1000 个累计帧打印一次"平均帧轮廓"。看打印的 75 个均值,
 * 正尖峰/负尖峰对应的下标即为 PEAK/TRO 应取的位置。
 */
#define CAL_ACC_EVERY        4U           /* 每 4 帧累加 1 帧 */
#define CAL_ACC_FRAMES       1000U        /* 累计帧数达此值则打印并清零(= 4000 帧 ≈ 200ms) */
/* g_cal_acc[i] 累加多帧的同一下标 i。g_cal_n 统计实际抽取帧数。主循环取快照后再做除法。 */
static uint32_t g_cal_acc[FRAME_LEN];
static volatile uint32_t g_cal_n;        /* 中断写/主循环轮询, 必须 volatile */
static uint32_t g_cal_snap[FRAME_LEN];    /* 主循环快照, 避免 RTT 读撕裂 */
/* 每轮打印周期内的判决统计，用于检查门限裕量和输出抖动。 */
static volatile uint32_t g_cal_flips;
static volatile uint32_t g_cal_over;
static volatile int32_t  g_cal_smin = 0x7FFFFFFF;
static volatile int32_t  g_cal_smax = -0x7FFFFFFF;
#endif

/* ======================= 基础运算 ======================= */

/* 三取中: 3 次比较, 剔除孤立坏帧 */
/* 输入是一帧的峰谷差，输出是最近三个分数的中间值。例如 80、900、82 返回 82。 */
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

    /* 用当前值覆盖最旧槽；下次依次写槽 0、1、2，再绕回槽 0。 */
    g_h3[g_h3i] = v;
    if (++g_h3i >= 3U) {
        g_h3i = 0U;
    }

    a = g_h3[0];
    b = g_h3[1];
    c = g_h3[2];

    /* 先确定 a、b 的小值和大值。c 在区间外时取最近端点；在区间内时，c 自身就是中值。 */
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
    /* 加新值、减即将被覆盖的旧值，一次更新窗口总和。初始槽为零，因此填充期也适用。 */
    g_ring_sum += v - g_ring[g_ri];
    g_ring[g_ri] = v;

    if (++g_ri >= MA_N) {
        g_ri = 0U;
    }
    /* 有效项数从 1 增至 16 后封顶。窗口满后，每帧丢掉一个最旧值并输出一次平均。 */
    if (g_warm < MA_N) {
        ++g_warm;
    }

    if (g_warm >= MA_N) {
        /* 窗口填满后固定除以 16；加 8 后右移 4 位，实现四舍五入并避免稳态软件除法。 */
        return (g_ring_sum + (1 << (MA_SHIFT - 1))) >> MA_SHIFT;
    }

    return g_ring_sum / (int32_t)g_warm;    /* 只有前 15 帧按实际点数做除法 */
}

/* 帧内同步解调: 正尖峰窗 - 负尖峰窗(三点对三点, 权重和=0) */
static int32_t frame_score(const uint16_t *f)
{
    /* f 指向本次 DMA 已写完的半区。即使传入 &adc_frame[75]，f[13] 也仍表示本帧第 14 个采样点。 */
    int32_t p = (int32_t)f[PEAK0] + (int32_t)f[PEAK1] + (int32_t)f[PEAK2];
    int32_t t = (int32_t)f[TRO0] + (int32_t)f[TRO1] + (int32_t)f[TRO2];

    /* 两组各三点，共同直流偏移会相减抵消。结果带符号，范围为 −12285 至 +12285（12 位 ADC）。 */
    return p - t;
}

/* ======================= 判决 ======================= */

/* 把逻辑状态 on 映射为 PA12 电平。写 BSRR 对应位置高，写 BRR 对应位清低；宏为 0 时有物输出低。 */
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

/* 上电空场标定。v 已经经过中值与滑动平均；标定的是空场峰谷分数和它的波动幅度。 */
static void calibrate(int32_t v)
{
    /* 意图是把 v 放大 256 倍以保留小数。原式在 v 为负时存在 C 有符号左移未定义行为。 */
    int32_t vq = v << QSH;
    int32_t d0 = vq - g_base_q8;
    int32_t a0 = (d0 < 0) ? -d0 : d0;

    /* 上电空场标定: 用较快的速率收敛, 跳过环形缓冲填充期 */
    if (g_warm < MA_N) {
        return;
    }

    /* 偏差达到检测下限时，认为上电时可能已有物体，不把该帧学习成背景。 */
    if ((a0 >> QSH) >= THR_MIN) {
        return;
    }

    /* 背景向当前分数移动约 1/8。例如背景为 80、当前为 96，更新后约为 82。 */
    g_base_q8 += d0 >> 3;                         /* 快速收敛 */

    {
        /* 先用更新后的背景计算偏差，再取绝对值。噪声估计向该幅度移动约 1/4，最后限制到 8～400 分。 */
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

/* 运行阶段的两状态迟滞判断。相同的残差落在上下门限之间时，已有状态决定输出。 */
static void decide(int32_t v)
{
    /* 这里也把 v 转成 Q8，负分数左移有同样的 C 语言问题。n 为有符号残差，a 为其绝对值。 */
    int32_t vq = v << QSH;
    int32_t n = (vq - g_base_q8) >> QSH;
    int32_t a = (n < 0) ? -n : n;
    int32_t sigma = g_sigma_q8 >> QSH;
    /* 进入门限取 6 倍噪声与 300 中的大值。例如噪声为 60，进入门限为 360。 */
    int32_t thr = (int32_t)K_ON * sigma;

    if (thr < THR_MIN) {
        thr = THR_MIN;
    }

    if (g_state == 0U) {
        g_off_cnt = 0U;

        /* 正向残差连续超限 DEBOUNCE_N 帧后才置位，避免边界附近来回跳变。 */
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

        /* 只有无物且残差幅度小于进入门限才学习；大的负向扰动既不会置位，也不会更新背景。 */
        if (a < thr) {
            int32_t d = vq - g_base_q8;

            /* 背景下降每帧跟进约 1/64，上升约 1/1024，给正在增大的目标信号留下越过门限的时间。 */
            if (d < 0) {
                g_base_q8 += d >> BASE_DOWN_SHIFT;
            } else {
                g_base_q8 += d >> BASE_UP_SHIFT;
            }

            /* 噪声向本帧绝对偏差移动约 1/256。此处估算的是平均绝对偏差，并限制上下界。 */
            g_sigma_q8 += ((a << QSH) - g_sigma_q8) >> SIG_SHIFT;

            if (g_sigma_q8 < (SIGMA_MIN << QSH)) {
                g_sigma_q8 = SIGMA_MIN << QSH;
            }
            if (g_sigma_q8 > (SIGMA_MAX << QSH)) {
                g_sigma_q8 = SIGMA_MAX << QSH;
            }
        }
    } else {
        /* 有物期间冻结背景与噪声估计，使用独立的释放门限和连续确认计数。 */
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
/* DMA 每交付一个半区就调用一次。检测使用六个点；调试波形按原始 75 个点分别统计，两条计算各有用途。 */
static void process_frame(const uint16_t *f)
{
    int32_t s = frame_score(f);

#if CALIB_DUMP
    /* 帧号尚未递增，因此第 0、4、8……号帧被抽取。每次将一帧各点累加后，帧数只加一次。 */
    if ((g_frames % CAL_ACC_EVERY) == 0U) {
        uint32_t i;
        for (i = 0; i < FRAME_LEN; ++i) {
            g_cal_acc[i] += f[i];
        }
        ++g_cal_n;
    }
#endif

    /* 先在连续三帧间取中值，再平均近期的中值结果。一个很大的孤立尖刺通常在第一步就被去掉。 */
    s = med3(s);
    s = ma(s);

#if CALIB_DUMP
    /* 记录抽样帧的分数范围，以及高于最低进入门限的次数。只用于调试打印，不参与判决。 */
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

    /* 帧号 0～255 走标定，其中最初 15 帧只填满滤波窗口。第 257 帧（帧号 256）开始判决。 */
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
    /* HTIF1 表示前半区写满。向清除寄存器写对应位后把前半区首地址交给算法；此时 DMA 可继续写后半区。 */
    if ((DMA1->ISR & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        process_frame(&adc_frame[0]);
    }

    /* TCIF1 表示后半区也写满。循环 DMA 自动绕回前半区；两个标志分别检查，CPU 须在数据被覆盖前处理。 */
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        process_frame(&adc_frame[FRAME_LEN]);
    }
}

/* ======================= 初始化 ======================= */

/* 系统时钟决定定时器计数速度和同步 ADC 时钟。初始化失败统一转入错误处理函数。 */
static void APP_SystemClockConfig(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* 选择内部高速振荡器 HSI，配置 24 MHz 校准值。此处选择的时钟源是 HSI。 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;
    osc.HSEState = RCC_HSE_OFF;
    osc.LSIState = RCC_LSI_OFF;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* SYSCLK 取 HSI，AHB 和 APB1 均不分频，按本配置计算为 24 MHz。 */
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        APP_ErrorHandler();
    }
}

/* 配置引脚复用、方向和初始输出。相同的引脚号配不同端口，代表不同的物理引脚。 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

//    g.Pin = GPIO_PIN_6;                 /* TIM3_CH1 发射 PWM */
//    g.Mode = GPIO_MODE_AF_PP;
//    g.Pull = GPIO_NOPULL;
//    g.Speed = GPIO_SPEED_FREQ_HIGH;
//    g.Alternate = GPIO_AF1_TIM3;
//    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_5 | GPIO_PIN_12;   /* PA5 预置高 + PA12 检测输出(同 V1) */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* PB5 配为上拉输入。本文件没有读取 PB5 的逻辑。 */
    g.Pin = GPIO_PIN_5;                 /* PB5 上拉输入(同 V1) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    /* 单独置高 PA5，再按 OUT_ACTIVE_HIGH 的定义将 PA12 设成无物状态。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    set_output(0U);
}

/* TIM3: 1MHz 计数, ARR=49 -> 50us;
 * CH1 使用 PWM1 和低有效极性，PA6 前 47 µs 为低，末尾 3 µs 为高。
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

    /* PWM1 在 CNT < 47 的时间段有效；低极性将该段映射为 PA6 低电平。实际发光相位取决于外部驱动。 */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_LOW;      /* CH1 有效时输出低电平 */
    oc.OCFastMode = TIM_OCFAST_ENABLE;
    oc.Pulse = EMIT_ON_US;

    if (HAL_TIM_OC_ConfigChannel(&h, &oc, TIM_CHANNEL_1) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* PWM2 的内部 OC2REF 在 CNT 达到 35 时转高。ADC 使用这个内部上升沿，CH2 无需接线。 */
    oc.OCMode = TIM_OCMODE_PWM2;             /* CNT>=CCR 时 OC2REF 为高 -> 上升沿在 CCR2 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.Pulse = CH2_TRIG_US;

    if (HAL_TIM_OC_ConfigChannel(&h, &oc, TIM_CHANNEL_2) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* 将 OC2REF 选作 TIM3 主触发输出 TRGO，建立 TIM3 到 ADC 的内部触发通路。 */
    master.MasterOutputTrigger = TIM_TRGO_OC2REF;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&h, &master) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* 打开两个比较寄存器的预装载，新的比较值在更新事件生效。下面依次启动发射通道和触发通道。 */
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    if (HAL_TIM_OC_Start(&h, TIM_CHANNEL_1) != HAL_OK) {
        APP_ErrorHandler();
    }

    if (HAL_TIM_OC_Start(&h, TIM_CHANNEL_2) != HAL_OK) {   /* 无引脚, 仅内部 TRGO */
        APP_ErrorHandler();
    }
}

/* ADC 由 TIM3-TRGO 硬件触发启动, 之后自由连续 1.5MS/s + DMA 循环双缓冲
 * 首次触发决定采样起点；原作者给出的约 2.9 µs 延迟需以对应硬件为准。 */
/* 输入 ch 是 ADC 通道号，main() 传入通道 0。ADC1 负责转换，DMA1 通道 1 把结果搬进静态数组。 */
static void ADC_DMA_Init(uint32_t ch)
{
    ADC_HandleTypeDef adc = {0};
    ADC_ChannelConfTypeDef cc = {0};
    DMA_HandleTypeDef hdma = {0};
    GPIO_InitTypeDef analog_input = {0};

    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();
    __HAL_RCC_ADC_CLK_ENABLE();

    adc.Instance = ADC1;
    /* ADC 外设自身的校准用于转换硬件；后面的 calibrate() 学习的是现场空场信号。 */
    if (HAL_ADCEx_Calibration_Start(&adc) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* 同步时钟配置为 PCLK/1。按本程序为 24 MHz；项目数据手册 V2.7 表 5-22 在 2.3～5.5 V 档给出的上限为 12 MHz。
     * 故 75 点/50 µs 是当前参数推算；调整分频后需重新核算帧长和峰谷位置。 */
    adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;   /* 24MHz */
    /* 12 位结果右对齐，单点数值为 0～4095。此处只选择一个通道。 */
    adc.Init.Resolution = ADC_RESOLUTION_12B;
    adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;
    /* 首次有效 TRGO 上升沿启动后，ADC 自动连续转换。以后每 75 点对应一帧，依靠采样速率与发射周期匹配。 */
    adc.Init.ContinuousConvMode = ENABLE;                 /* 触发一次后自由连续 */
    adc.Init.DiscontinuousConvMode = DISABLE;
    adc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;   /* TIM3 的 TRGO 提供启动沿 */
    adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    /* 连续发 DMA 请求；若数据寄存器发生溢出，允许用新数据覆盖旧数据。采样时间选择 3.5 个 ADC 时钟。 */
    adc.Init.DMAContinuousRequests = ENABLE;
    adc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adc.Init.SamplingTimeCommon = ADC_SAMPLETIME_3CYCLES_5;  /* 3.5+12.5=16 周期 */

    if (HAL_ADC_Init(&adc) != HAL_OK) {
        APP_ErrorHandler();
    }
    
     /* 本板 ADC_CHANNEL_0 对应 PA0。显式配置模拟输入，避免依赖 MSP 的实现。 */
    if (ch == ADC_CHANNEL_0) {
        analog_input.Pin = GPIO_PIN_0;
        analog_input.Mode = GPIO_MODE_ANALOG;
        analog_input.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &analog_input);
    }

    /* 把函数参数指定的通道加入转换序列，再交给 HAL 应用配置。 */
    cc.Channel = ch;
    cc.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&adc, &cc) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* 配置外设 DMA 请求映射并开启 DMA 时钟，为接下来的 DMA1 通道 1 配置做准备。 */
    HAL_SYSCFG_DMA_Req(0);
    __HAL_RCC_DMA_CLK_ENABLE();

    /* 源地址固定在 ADC 数据寄存器，目的地址逐项递增。两端都按半字（16 位）传输，正好匹配 uint16_t 数组。 */
    hdma.Instance = DMA1_Channel1;
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    /* 循环模式写完 150 项会绕回起点。DMA_PRIORITY_HIGH 是 DMA 仲裁优先级，下面 NVIC 设置的是 CPU 中断优先级。 */
    hdma.Init.Mode = DMA_CIRCULAR;                        /* 关键: 循环 */
    hdma.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma) != HAL_OK) {
        APP_ErrorHandler();
    }

    /* 把 ADC 与 DMA 的 HAL 句柄关联以启动传输。这里句柄为局部变量；本文件后续 IRQ 直接访问寄存器和静态数组。 */
    __HAL_LINKDMA(&adc, DMA_Handle, hdma);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* HAL 接口接收地址指针；实际传输宽度由 DMA 半字配置决定，长度 150 表示 150 个采样值。 */
    if (HAL_ADC_Start_DMA(&adc, (uint32_t *)adc_frame, FRAME_LEN_2) != HAL_OK) {
        APP_ErrorHandler();
    }
}

/* ======================= 主函数 ======================= */

/* 程序入口。TIM3 先运行；ADC/DMA 就绪后等待下一次 CH2 上升沿，随后中断持续执行检测。 */
int main(void)
{
    HAL_Init();
    APP_SystemClockConfig();
    GPIO_Init();
    TIM3_Init();

/* 只有启用调试输出时才初始化 RTT。非阻塞跳过模式下，上行缓冲空间不足时可能丢失打印数据。 */
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

        /* 主程序轮询 g_frames；中断仍能执行。8 帧自检通过仅说明帧处理已启动，空场标定另需累计到 256 帧。 */
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

        /* 主循环监测帧流并输出调试数据；检测判决仍由 DMA 中断驱动。 */
        while (1) {
            /* 连续 100 ms 没有新帧时，强制回到无物安全状态，并清除两侧去抖计数。 */
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
         * 中断内抽帧累加，主循环取快照并打印。复制快照时有短暂的中断屏蔽。
         *   D,frame,ms,base,sigma,state,accn,flips,smin,smax,over —— 状态行
         *   P,idx,mean                      —— 75 个下标的平均码值
         * 尖峰下标与 PEAK/TRO 不吻合时, 微调 CH2_TRIG_US(整 us)或宏后重烧。
         */
        /* 至少积满 1000 个抽取帧才输出。主循环读取稍晚时 n 可以大于 1000，所以均值要用实际 n。 */
        if (g_cal_n >= CAL_ACC_FRAMES) {
            char line[64];
            uint32_t i, n;
            uint32_t flips, over;
            int32_t smin, smax;
            int len;

            /* 暂时屏蔽中断，使数组和计数属于同一批数据。DMA 硬件仍能写 adc_frame；这里复制的是软件累加数组。 */
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
            /* 恢复中断后即可积累下一批。下面只用快照算波形，字符串格式化与传输均在主循环完成。 */
            __enable_irq();

            /* D 行同时给出翻转次数、分数范围和超过最低门限的抽样次数。 */
            len = snprintf(line, sizeof(line),
                           "D,%lu,%lu,%ld,%ld,%u,%lu,%lu,%ld,%ld,%lu\r\n",
                           (unsigned long)g_frames,
                           (unsigned long)HAL_GetTick(),
                           (long)(g_base_q8 >> QSH), (long)(g_sigma_q8 >> QSH),
                           (unsigned)g_state, (unsigned long)n,
                           (unsigned long)flips, (long)smin, (long)smax,
                           (unsigned long)over);
            if (len > 0) {
                SEGGER_RTT_Write(0, line, (unsigned)len);
            }
            /* 每条 P 行对应一个帧内位置 i，值为该位置跨 n 个抽取帧的平均值。可用这 75 个均值检查取点窗口。 */
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

/* 初始化失败或启动超时会停留在此循环，便于调试器定位；函数内没有复位或重试逻辑。 */
void APP_ErrorHandler(void)
{
    while (1) {
    }
}
