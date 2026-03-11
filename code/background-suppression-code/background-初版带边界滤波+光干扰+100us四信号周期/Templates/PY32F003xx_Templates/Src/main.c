/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "arm_math.h"
#include "arm_const_structs.h"


/* Private define ------------------------------------------------------------*/
#define TRANSMIT_PERIOD         25      //= 单位us ，发射管周期
#define TRANSMIT_PULSE          20      //= 占空比 ，pwm的CCR比较值
#define COMP_PULSE              20      //= 阈值         
#define COMP_PULSE1             25
#define BUFFER_SIZE             4       //= adcbuffer采样点数

//= 定时器1采样周期频率不能小于adc采样最短时间 >2
//= 单纯翻转io 8us 左右
//= adc读取在0.5us左右 3.5t时

#define CORR_THRESHOLD   10
#define CORR_HYSTERESIS  30
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef   AdcHandle;
int16_t adc_buffer[BUFFER_SIZE];
int16_t adc_buffer1[BUFFER_SIZE];
int16_t aADCxConvertedData;

TIM_HandleTypeDef  Tim3Handle;
TIM_OC_InitTypeDef sConfig;

TIM_HandleTypeDef  Tim1Handle;

GPIO_InitTypeDef  GPIO_InitStruct;

DMA_HandleTypeDef hdma_adc;


/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcInit(void);
static void TMR3_OC_Init(void);
static void GPIO_Init(void);
void DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);


void USART_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

#if  defined(__GNUC__)
  setvbuf(stdout, NULL, _IONBF, 0 );
#endif

  DEBUG_USART_CLK_ENABLE();

  DebugUartHandle.Instance          = DEBUG_USART;

  DebugUartHandle.Init.BaudRate     = DEBUG_USART_BAUDRATE;
  DebugUartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  DebugUartHandle.Init.StopBits     = UART_STOPBITS_1;
  DebugUartHandle.Init.Parity       = UART_PARITY_NONE;
  DebugUartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  DebugUartHandle.Init.Mode         = UART_MODE_TX_RX;
  DebugUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  DebugUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  HAL_UART_Init(&DebugUartHandle);

  DEBUG_USART_RX_GPIO_CLK_ENABLE();
  DEBUG_USART_TX_GPIO_CLK_ENABLE();

  /**USART1 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
  GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
  HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
  GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;

  HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct);

  /* ENABLE NVIC */
  HAL_NVIC_SetPriority(DEBUG_USART_IRQ,0,1);
  HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );
}
/**
  * @brief  Main program.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  APP_SystemClockConfig(); 
    
  /* Initialize USART */
  //USART_Config();         //= 打开这个可以启动串口1的初始化，用printf可以直接打印

  /* Initialize gpio */  
  GPIO_Init();
  
  /* Initialize OC1 */ 
  TMR3_OC_Init();       
  
  /* Initialize ADC */
  APP_AdcInit();     
  

                    
  while (1)                 //= 代码纯中断处理
  {
         
  }
}
/**
  * @brief  IO Configuration
  * @param  None
  * @retval None
  */
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_2;        //= PA5 绿色指示灯 ;PA12 输出控制; PA2 测试用io PB5 外部切换
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
   
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
}

/**
  * @brief  TMR3 OC Configuration
  * @param  None
  * @retval None
  */
static void TMR3_OC_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    
    TIM_MasterConfigTypeDef sMasterConfig;

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Tim3Handle.Instance = TIM3;                                                  /* Select TIM1 */
    Tim3Handle.Init.Period            = TRANSMIT_PERIOD - 1;                     /* Auto-reload value */
    Tim3Handle.Init.Prescaler         = 24 - 1;                                   /* Prescaler */
    Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;                  /* No clock division */
    Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;                      /* Up counting */
    Tim3Handle.Init.RepetitionCounter = 1 - 1;                                   /* No repetition counting */
    Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;          /* Auto-reload register not buffered */
    if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)                                /* Initialize timer base */
    {
    APP_ErrorHandler();
    }
    
    sConfig.OCMode       = TIM_OCMODE_PWM1;                                   /* Output configured in toggle mode */
    sConfig.OCPolarity   = TIM_OCPOLARITY_LOW;                                 /* Active high output level for OC channel */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;                                  /* Disable fast output mode */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* Active high output level for OCN channel */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* Output low level in idle state for OC1N */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* Output low level in idle state for OC1 */

    sConfig.Pulse = TRANSMIT_PULSE;                                               /* CC1 value */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
    HAL_TIM_OC_Start(&Tim3Handle,TIM_CHANNEL_1);
    ///=================LEDPWM输出END=============================///
     sConfig.OCMode       = TIM_OCMODE_PWM1;                                   /* Output configured in toggle mode */
    sConfig.OCPolarity   = TIM_OCPOLARITY_LOW;                                 /* Active high output level for OC channel */
    sConfig.OCFastMode   = TIM_OCFAST_ENABLE;                                  /* Disable fast output mode */
    
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* Active high output level for OCN channel */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* Output low level in idle state for OC1N */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* Output low level in idle state for OC1 */

    sConfig.Pulse = COMP_PULSE-1;                                               /* CC1 value */
    if (HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
      
    HAL_TIM_OC_Start(&Tim3Handle,TIM_CHANNEL_2);     //= 启动通道2
    
    #if 0
    sConfig.Pulse  = COMP_PULSE1;
     if (HAL_TIM_OC_ConfigChannel(&Tim3Handle,&sConfig,TIM_CHANNEL_3) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
    
    HAL_TIM_OC_Start(&Tim3Handle,TIM_CHANNEL_3);
    #endif 
    
  
    ///===============定时触发adcEND===============================///
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &sMasterConfig);
     
    #if 1
    HAL_TIM_Base_Start(&Tim3Handle);
    #endif
}


 /**
  * @brief  ADC Configuration
  * @param  None
  * @retval None
  */
static void APP_AdcInit(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();
    __HAL_RCC_ADC_CLK_ENABLE();

    AdcHandle.Instance = ADC1;
    /* ADC calibration */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)                 
    {
    APP_ErrorHandler();
    }

    AdcHandle.Instance = ADC1;

    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;              /* Set ADC clock */
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;                        /* 12-bit resolution for converted data */
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                        /* Right-alignment for converted data */
    AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;                  /* Single sampling  */
    AdcHandle.Init.LowPowerAutoWait = ENABLE;                              /* Enable wait for conversion mode */

    //= 连续转换
    AdcHandle.Init.ContinuousConvMode = DISABLE;                           /* Single conversion mode */
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                        /* Disable discontinuous mode */

    //= 定时器3触发
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;           /* Software triggering */
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;   /* No external trigger edge */

    AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* isable DMA */
    AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                     /* Overrun handling: overwrite previous value  */
    //= 采样周期 3cyclk = 0.5us
    AdcHandle.Init.SamplingTimeCommon=ADC_SAMPLETIME_41CYCLES_5;          /* Set sampling time */
    /* Initialize ADC */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)                                
    {
    APP_ErrorHandler();
    }
    //= 通道配置
    sConfig.Channel = ADC_CHANNEL_0;                                       /* ADC channel selection */
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;                                /* Set the rank for the ADC channel order */ 
    /* Configure ADC channels */  
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)            
    {
    APP_ErrorHandler();
    }

    // 配置DMA传输
    HAL_SYSCFG_DMA_Req(0);                                      /* Set DMA1 mapping to ADC */

    __HAL_RCC_DMA_CLK_ENABLE();
    hdma_adc.Instance = DMA1_Channel1;              //adc1
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&hdma_adc);

    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc);                   /* Link DMA handle to ADC */

     //配置NVIC中断优先级
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);  // 优先级1，子优先级0
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);          // 使能DMA中断

     //启用DMA传输完成中断
    __HAL_DMA_ENABLE_IT(&hdma_adc, DMA_IT_TC);  // 传输完成中断使能

    #if 1
    if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)            
      {
        APP_ErrorHandler();
      }
    #endif 
}

 //= 滑动平均
#define WINDOW_SIZE 10
uint32_t buffer[WINDOW_SIZE], aindex = 0;
uint32_t adc_filter(uint32_t new_sample) {
    buffer[aindex] = new_sample;
    aindex = (aindex + 1) % WINDOW_SIZE;
    uint32_t sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) sum += buffer[i];
    return sum / WINDOW_SIZE;
}

//= 一阶iir
int16_t IIR_Filter(int16_t input)
{
    static int32_t filter = 0;

    filter = filter - (filter >> 3) + input;

    return filter >> 3;
}
//= 二节滤波
/**
  * @brief  16位ADC值的一阶低通滤波函数
  * @param  adc_value: 当前采集的16位ADC原始值
  * @param  alpha_int: 滤波系数（0-256对应0.0-1.0，建议4-64）
  * @retval 滤波后的16位结果值
  * @note   使用静态变量保存滤波状态，首次调用自动初始化
  */
uint16_t adc_lowpass_filter(uint16_t adc_value, uint8_t b0, uint8_t b1, uint8_t b2)
{
    static uint16_t prev = 0, prev_prev = 0;
    static uint8_t first_call = 1;
    
    // 首次调用初始化
    if(first_call) {
        prev = adc_value;
        prev_prev = adc_value;
         first_call = 0;
        return adc_value;
    }
    
    // 验证系数总和（防御性编程）
    if(b0 + b1 + b2 != 256) {
        // 错误处理（可返回0或上次值）
        return prev;
    }

    // 二阶滤波计算（带正确四舍五入）
    uint32_t temp = (uint32_t)adc_value * b0 + 
                   (uint32_t)prev * b1 + 
                   (uint32_t)prev_prev * b2;
    temp += 128; // 实际四舍五入偏移量应为64，此处保持与原逻辑一致需修正
    
    // 状态更新
    prev_prev = prev;
    prev = (uint16_t)(temp >> 8);
    
    return prev;
}



//= adcbuffer求均值
int16_t ADC_Buffer_Average(int16_t *buf, uint16_t len)
{
    int32_t sum = 0;

    for(uint16_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }

    return (int16_t)(sum / len);
}

//= 死区控制
int32_t Deadband_Filter(int32_t input, int32_t deadband)
{
    static int32_t last_output = 0;

    if(abs(input - last_output) > deadband)
    {
        last_output = input;
    }

    return last_output;
}

//= 滞回输出
void Sensor_Output_Control(int32_t value)
{
        if(value > CORR_THRESHOLD + CORR_HYSTERESIS)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        }
        else if(value < CORR_THRESHOLD - CORR_HYSTERESIS)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        }
}
//= 各相平均
typedef struct {
    int32_t cmp_avg;   // 奇数下标平均值（比较触发点）
    int32_t upd_avg;   // 偶数下标平均值（更新事件点）
} ADC_AvgResult_t;

ADC_AvgResult_t ADC_Buffer_EvenOdd_Average(int16_t *buf, uint16_t len)
{
    ADC_AvgResult_t result = {0, 0};
    int32_t sum_cmp = 0;   // 奇数下标总和
    int32_t sum_upd = 0;   // 偶数下标总和
    uint16_t count_cmp = 0, count_upd = 0;

    for(uint16_t i = 0; i < len; i++)
    {
        if(i % 2 == 0) // 偶数下标 → 更新事件
        {
            sum_upd += buf[i];
            count_upd++;
        }
        else          // 奇数下标 → 比较触发点
        {
            sum_cmp += buf[i];
            count_cmp++;
        }
    }

    result.cmp_avg = (count_cmp != 0) ? (sum_cmp / count_cmp) : 0;
    result.upd_avg = (count_upd != 0) ? (sum_upd / count_upd) : 0;

    return result;
}



//= 参考
int32_t ref_wave[BUFFER_SIZE] =
{
  -1, 1, -1, 1
};

int32_t optical_sync_detect(int16_t *buf)
{
    int32_t mean = 0;

    for(int i=0;i<BUFFER_SIZE;i++)
        mean += buf[i];

    mean /= BUFFER_SIZE;

    int32_t corr = 0;
    
    int32_t s = 0;

    for(int i=0;i<BUFFER_SIZE;i++)
    {
        s = buf[i] - mean;

        corr += s * ref_wave[i];
    }

    return corr;
}

//= 判决
void optical_decision(int32_t value)
{
#if 0
    static int32_t detect_cnt = 0;
    
    if(value > (CORR_THRESHOLD + CORR_HYSTERESIS))
    {
        if(detect_cnt < 5)
            detect_cnt++;
        else
        {
             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
        }
    }
    else if(value < CORR_THRESHOLD)
    {
        if(detect_cnt>0)
           detect_cnt --;
        else 
        {
             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
        }
    }
#endif 
#if 1

    if(value > CORR_THRESHOLD)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    }
    else if(value < -CORR_THRESHOLD)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    }

#endif 
}
//DMA中断服务函数
void DMA1_Channel1_IRQHandler(void)
{
  //HAL_DMA_IRQHandler(&hdma_adc);  // 标准HAL中断处理
  
  if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))         
    {
       #if 1
       
       static ADC_AvgResult_t result = {0};
       
       result = ADC_Buffer_EvenOdd_Average(adc_buffer,BUFFER_SIZE);
       
       static int32_t about_value = 0;
       about_value = optical_sync_detect(adc_buffer);
       about_value = IIR_Filter(about_value);
        
       static int32_t filter_value = 0;
       filter_value = Deadband_Filter(about_value,8);
       
       optical_decision(filter_value);

//        if(filter_value > CORR_THRESHOLD + CORR_HYSTERESIS)
//        {
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
//        }
//        else if(filter_value < CORR_THRESHOLD)
//        {
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//        }
       #elif
//       static int16_t avg = 0;
//       avg = ADC_Buffer_Average(adc_buffer, BUFFER_SIZE);
      #endif    
      
      
      
       __HAL_DMA_CLEAR_FLAG(&hdma_adc, DMA_IFCR_CTCIF1);
    }                                  
}

void ADC_COMP_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&AdcHandle);
  
}

    /**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator Configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI; /* Select oscillators HSE,HSI,LSI */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* Enable HSI */
#if defined(RCC_HSIDIV_SUPPORT)
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI not divided */
#endif
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_24MHz;  /* Configure HSI clock as 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;                         /* Disable HSE */
  /*RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;*/
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* Disable LSI */

  /* Configure oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Clock source configuration */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1; /* Select clock types HCLK, SYSCLK, PCLK1 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; /* Select HSI as the system clock */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     /* AHB clock not divide */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* APB clock not divided */
  /* Configure clock source */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)  */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
