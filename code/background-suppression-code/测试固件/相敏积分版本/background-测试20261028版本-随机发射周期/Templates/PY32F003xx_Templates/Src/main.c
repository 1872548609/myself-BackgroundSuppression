/*
 * 基础配置说明版
 *
 * 配置流程：
 * HAL初始化 -> 系统时钟 -> GPIO -> TIM3配置 -> ADC/DMA配置
 * -> DMA先启动并等待触发 -> 启动TIM3 CH2和CH1 -> DMA完成中断重新武装
 *
 * 本文件仅为基础配置展示：
 * 1. 不含滤波、阈值判定、NO/NC控制算法和测试观察变量。
 * 2. 保持纯净版所有有效语句及参数不变，仅增加中文说明。
 * 3. 原设计使用TIM3 CH2的OC2REF作为TRGO，在指定计数点触发ADC。
 * 4. ADC采用连续转换，DMA为NORMAL模式，每轮目标搬运10个16位结果。
 * 5. 本文件未验证在实际芯片上能否每轮可靠停止、重装DMA并接受下一次触发。
 * 6. 需要结合main.h、HAL底层MSP及芯片参考手册确认ADC0对应引脚和DMA请求映射。
 */

/* 工程头文件：提供HAL类型、外设枚举及APP_ErrorHandler声明等。 */
#include "main.h"
/* 提供uint16_t等标准整数类型。 */
#include <stdint.h>

/* TIM3计数周期，当前计数步长按24MHz/24=1MHz计算，每格1us。ARR=109，因此周期为110us。 */
#define TRANSMIT_PERIOD 110
/* CH1比较值CCR1=107。PWM1+低有效时，计数0~106对应低电平，107~109对应高电平。 */
#define TRANSMIT_PULSE 107
/* CH2实际比较值为COMP_PULSE-1=108，在CNT=108附近产生OC2REF上升沿。 */
#define COMP_PULSE 109
/* 每次DMA传输10个ADC转换结果；每个结果存入一个16位元素，并非10路ADC。 */
#define BUFFER_SIZE 7

/*
 * 输出测试宏：1=开启，0=关闭。
 * 使用现有PA12作为示波器测试输出，不额外占用GPIO。
 * 开启后，DMA TC中断真正进入处理分支时PA12拉低，完成DMA停止及重新启动后拉高。
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
    /* 先使ADC/DMA进入等待硬件触发状态。 */
    APP_AdcStartDMA();

    /* 启动CH2通道；它的内部OC2REF用于TIM3 TRGO触发，不要求配置CH2外部引脚。 */
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_2) != HAL_OK)
          APP_ErrorHandler();
    /* 启动CH1并从PA6输出PWM；CH1和CH2使用同一TIM3计数器。 */
    if (HAL_TIM_OC_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
          APP_ErrorHandler();

    /* 当前主循环无业务处理，采集链路由定时器、ADC、DMA及DMA中断完成。 */
    while (1)
    {
    }
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

/* TIM3：24MHz时钟、PSC=23、ARR=109，计数周期110us；CH1 PWM，CH2提供ADC TRGO。 */
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

    /* ARR=110-1=109，向上计数0~109，合计110个计数周期。 */
    Tim3Handle.Init.Period            = TRANSMIT_PERIOD - 1;

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

    /* CCR1=107，110计数中名义低107us、高3us；发射管导通极性取决于外围驱动。 */
    sConfig.Pulse = TRANSMIT_PULSE;

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

    /* CCR2=108，约在本周期第108个计数点形成OC2REF上升沿。 */
    sConfig.Pulse = COMP_PULSE-1;

    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    {
          APP_ErrorHandler();
    }

    /* 配置TIM3主触发输出TRGO；此配置不启动TIM3中断。 */
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

/* ADC1及DMA配置：ADC0单通道，12位，TIM3 TRGO触发，连续转换，DMA一次搬运10个点。 */
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

    /* 开启自动等待：新转换可能等待上一个数据被读取，以避免数据未取走即覆盖。 */
    AdcHandle.Init.LowPowerAutoWait = ENABLE;

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

/* 启动一轮ADC DMA传输：10个结果写入数组，随后等待下一次重装。 */
static void APP_AdcStartDMA(void)
{
    /* 配置并启动ADC DMA。第三个参数10代表DMA要搬运的转换结果个数。 */
    if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)
    {
            APP_ErrorHandler();
    }

    /* 关闭DMA半传输中断，仅保留传输完成中断路径。 */
    __HAL_DMA_DISABLE_IT(&hdma_adc, DMA_IT_HT);
}

/* DMA通道1 IRQ：只做必要的完成标志清除、停止和再次启动，不做信号算法。 */
void DMA1_Channel1_IRQHandler(void)
{
    /* 仅处理通道1传输完成TC标志；HT和TE未在此函数中处理。 */
    if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))
    {
#if OUTPUT_TEST_ENABLE
        /* 中断处理计时开始，拉低PA12。 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
#endif
        /* 清除本轮DMA传输完成标志，防止该标志持续触发IRQ。 */
        __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1);
        /* 停止当前ADC/DMA，使HAL状态可重新进入下一轮启动。 */
        if (HAL_ADC_Stop_DMA(&AdcHandle) != HAL_OK)
        {
              APP_ErrorHandler();
        }
        /* 重新装载10点DMA并等待下一次TIM3 TRGO；TIM3计数器没有停止。 */
        APP_AdcStartDMA();
#if OUTPUT_TEST_ENABLE
        /* 中断处理计时结束，恢复PA12高电平。 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
#endif
    }
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
