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
#define TRANSMIT_PERIOD 100  //= 单位us ，发射管周期
#define TRANSMIT_PULSE 90   //= 占空比 ，占周期多少  ,也就是pwm的CCR比较值


#define COMP_PULSE 85      // 比较点  
#define COMP_VALUE 100    // 最大最小值的比较差


#define COMP_DIFFERENCE 50  // 回差=COMP_DIFFERENCE/2

#define SAMPLE_PERIO 3      // 定时器1采样周期        
#define BUFFER_SIZE 15        // 采样点数

//= 定时器1采样周期频率不能小于adc采样最短时间 >2
//= 单纯翻转io 8us 左右
//= adc读取在0.5us左右 3.5t时
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef   AdcHandle;
uint16_t            adc_buffer[BUFFER_SIZE];
uint16_t            AD_value_max = COMP_VALUE;
uint16_t            AD_value_min = COMP_VALUE;
uint16_t            aADCxConvertedData;

// 预计算系数表（避免运行时计算三角函数）
float coeff = 0;

TIM_HandleTypeDef  Tim3Handle;
TIM_OC_InitTypeDef sConfig;

TIM_HandleTypeDef  Tim1Handle;
TIM_MasterConfigTypeDef sMasterConfig;

GPIO_InitTypeDef  GPIO_InitStruct;

DMA_HandleTypeDef hdma_adc;
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcInit(void);
static void TMR3_OC_Init(void);
static void GPIO_Init(void);
void TIM1_Init(void);
void DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);

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
  //DEBUG_USART_Config();    

  /* Initialize gpio */  
  GPIO_Init();
  
  /* Initialize TIM1 */
  TIM1_Init();
  
  /* Initialize OC1 */ 
  TMR3_OC_Init();       
  
  /* Initialize ADC */
  APP_AdcInit();     
  
  
                    
  while (1)
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
    
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
//    GPIO_InitStruct.Pin = GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    
//    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
//    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    
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
   
    //HAL_TIM_OC_Start(&Tim3Handle,TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&Tim3Handle,TIM_CHANNEL_2);
    ///===============定时触发adcEND===============================///
//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
//    HAL_TIMEx_MasterConfigSynchronization(&Tim3Handle, &sMasterConfig);
    
//    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
//    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
//    HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig);
    
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          
    
                                                  
    HAL_TIM_Base_Start_IT(&Tim3Handle);
    //HAL_TIM_Base_Start(&Tim3Handle);
    
    
     TIM3->DIER &= ~TIM_DIER_UIE; // 禁用更新中断 
    
}

void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&Tim3Handle);  // 处理中断
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
      
      TIM1->CNT = 0;
   TIM1->CR1 |= (TIM_CR1_CEN); 
            
  
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET) ;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
     
     
//     if(__HAL_TIM_GET_IT_SOURCE(&Tim3Handle, TIM_IT_UPDATE))
//     {
//         __HAL_TIM_CLEAR_IT(&Tim3Handle, TIM_IT_UPDATE);  
           
//            TIM3->CR1 &= ~(TIM_CR1_CEN); 
//            TIM3->CNT = 0;

//            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET) ;
//     }
//     
          
    // HAL_TIM_Base_Stop_IT(&Tim3Handle);
//    HAL_TIM_OC_Stop(&Tim3Handle,TIM_CHANNEL_1);                             
    
      
    
  }
}


/**
  * @brief  TMR1 Configuration
  * @param  None
  * @retval None
  */
void TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    
    Tim1Handle.Instance = TIM1;
    Tim1Handle.Init.Period = SAMPLE_PERIO - 1;
    Tim1Handle.Init.Prescaler              =   1 - 1;
    Tim1Handle.Init.ClockDivision          =   TIM_CLOCKDIVISION_DIV1;
    Tim1Handle.Init.RepetitionCounter      =   1 - 1;
    Tim1Handle.Init.CounterMode            =   TIM_COUNTERMODE_UP;
    Tim1Handle.Init.AutoReloadPreload      =   TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_Base_Init(&Tim1Handle) != HAL_OK)                                /* Initialize timer base */
    {
    APP_ErrorHandler();
    }
    
//    // 配置从模式：外部时钟模式1，触发源为TIM3_TRGO(ITR1)  
//    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
//    sSlaveConfig.InputTrigger = TIM_TS_ITR2;  //ITR1对应TIM3_TRGO
//    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;        
//    HAL_TIM_SlaveConfigSynchro(&Tim1Handle, &sSlaveConfig); 
    
    //= 主模式
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig);
        
     /*  Start TIM1 clock */
      if (HAL_TIM_Base_Start(&Tim1Handle) != HAL_OK)                       
      {
        APP_ErrorHandler();
      }
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
  
  //= 定时器3比较触发
  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;           /* Software triggering */
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;   /* No external trigger edge */
  
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* isable DMA */
  AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                     /* Overrun handling: overwrite previous value  */
  
  AdcHandle.Init.SamplingTimeCommon=ADC_SAMPLETIME_3CYCLES_5;          /* Set sampling time */
  /* Initialize ADC */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)                                
  {
    APP_ErrorHandler();
  }

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

     if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK)            
      {
        APP_ErrorHandler();
      }

}

 //= 滑动平均
#define WINDOW_SIZE 2
uint32_t buffer[WINDOW_SIZE], aindex = 0;
uint32_t adc_filter(uint32_t new_sample) {
    buffer[aindex] = new_sample;
    aindex = (aindex + 1) % WINDOW_SIZE;
    uint32_t sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) sum += buffer[i];
    return sum / WINDOW_SIZE;
}

//= 一节滤波
/**
  * @brief  16位ADC值的一阶低通滤波函数
  * @param  adc_value: 当前采集的16位ADC原始值
  * @param  alpha_int: 滤波系数（0-256对应0.0-1.0，建议4-64）
  * @retval 滤波后的16位结果值
  * @note   使用静态变量保存滤波状态，首次调用自动初始化
  */
uint16_t adc_lowpass_filter(uint16_t adc_value, uint8_t alpha_int)
{
    // 状态保持变量（静态存储）
    static uint16_t previous_value = 0;    // 上次滤波结果
    static uint8_t first_call = 1;        // 首次调用标志
    
    // 首次调用处理（直接初始化状态）
    if (first_call) {
        previous_value = adc_value;
        first_call = 0;
        return adc_value;
    }
    
    // 整数运算实现滤波公式：y = y_prev*(256-α)/256 + x*α/256
    // 使用32位临时变量防止溢出
    uint32_t temp = (uint32_t)previous_value * (256 - alpha_int);
    temp += (uint32_t)adc_value * alpha_int;
    temp += 128;  // 四舍五入偏移量（+0.5）
    
    // 更新状态并返回结果（8位分数位右移）
    previous_value = (uint16_t)(temp >> 8);
    return previous_value;
}

//DMA中断服务函数
void DMA1_Channel1_IRQHandler(void)
{
  //HAL_DMA_IRQHandler(&hdma_adc);  // 标准HAL中断处理
  
  if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))         
    {
        

        TIM1->CR1 &= ~(TIM_CR1_CEN); 
        TIM1->CNT = 0;
        
        //aADCxConvertedData = adc_filter(aADCxConvertedData);
        
       // aADCxConvertedData = adc_lowpass_filter(aADCxConvertedData,64);
       

        
        static uint16_t NONCchange = 1;
        
        if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
        {
           NONCchange = 0;
        }
        else
        {
          NONCchange = 1;
        }
              
       
       static uint16_t max = 0;
       
       static uint16_t min = 0;
       
       max = adc_buffer[0];
       
       min = adc_buffer[0];
       
       int i = 0;
       
       for(i=0;i<BUFFER_SIZE;i++)
       {
           if(adc_buffer[i]>max){max=adc_buffer[i];} 
            if(adc_buffer[i]<min){min=adc_buffer[i];} 
       }
       
       static uint16_t valid_count = 0;    
       
       static uint16_t difvalue = 0;
       
       difvalue = max-min;
       
       difvalue = adc_lowpass_filter(difvalue,32);
       
        if(difvalue>=COMP_VALUE+COMP_DIFFERENCE) 
        {
               
            if(valid_count<9)
            {     
                valid_count ++;
              
            }
            else
            {
                if(!NONCchange)
                {
                   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
                }
                
            }
              
        } 
        else if(difvalue<COMP_VALUE)
        {
            if(valid_count)
            {
                valid_count--;
            }
            else
            {
                if(!NONCchange)
                {
                   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
                }
                else
                {
                    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET) ;
                }
               
            }
        }
        
                                                          

           

//         static int16_t  difvalue =0;
//         
//         difvalue = (int16_t)aADCxConvertedData-2040 ; 
//         
//         if(difvalue<0)
//         {
//              difvalue = abs(difvalue);
//         
//                 if(difvalue>=COMP_VALUE+COMP_DIFFERENCE) 
//                {
//                       
//                    if(valid_count<9)
//                    {     
//                        valid_count ++;
//                      
//                    }
//                    else
//                    {
//                        if(!NONCchange)
//                        {
//                           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
//                        }
//                        else
//                        {
//                            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
//                        }
//                        
//                    }
//                      
//                } 
//                else if(difvalue<COMP_VALUE)
//                {
//                    if(valid_count)
//                    {
//                        valid_count--;
//                    }
//                    else
//                    {
//                        if(!NONCchange)
//                        {
//                           HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
//                        }
//                        else
//                        {
//                            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET) ;
//                        }
//                       
//                    }
//                }
//         }
         
        
       
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);                               
       __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1);
    }                                  
}

void ADC_COMP_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&AdcHandle);
  
}

  




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//    if(hadc->Instance == ADC1) {

//        if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))         
//        {
//             __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1); 

//             __HAL_RCC_TIM1_CLK_DISABLE();
//             
//            static uint16_t min_val = 0;
//            static uint16_t max_val = 0;

//            min_val = adc_buffer[0];
//            max_val = adc_buffer[0];

//           
//            int i = 0;
//            for(i=0;i<BUFFER_SIZE;i++)
//            {
//                 uint16_t sample = adc_buffer[i];
//                 
//                if (sample < min_val)
//                    min_val = sample;
//                if (sample > max_val) 
//                    max_val = sample;
//            }
//            
//            
//            if(max_val-min_val>100) {
//                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//            } else {
//                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//            }
//        
//            min_val = 0;
//            max_val = 0;
//        }
//    }  
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
