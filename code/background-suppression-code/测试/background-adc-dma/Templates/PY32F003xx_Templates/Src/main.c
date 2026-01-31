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

/* Private define ------------------------------------------------------------*/
#define TRANSMIT_PERIOD 100  //= 单位us ，发射管周期
#define TRANSMIT_PULSE 90   //= 占空比 ，占周期多少  ,也就是pwm的CCR比较值

#define COMP_VALUE 1860          
#define BUFFER_SIZE 10
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef   AdcHandle;
uint16_t            adc_buffer[BUFFER_SIZE];
uint16_t            AD_value_max = COMP_VALUE;
uint16_t            AD_value_min = COMP_VALUE;
uint16_t            aADCxConvertedData;


TIM_HandleTypeDef  TimHandle;
TIM_OC_InitTypeDef sConfig;

TIM_HandleTypeDef  Tim1Handle;
TIM_MasterConfigTypeDef sMasterConfig;

GPIO_InitTypeDef  GPIO_InitStruct;


/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_AdcInit(void);
static void TMR3_OC_Init(void);
static void GPIO_Init(void);
void TIM1_Init(void);

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

  /* Initialize OC1 */ 
  TMR3_OC_Init();       
  
  /* Initialize ADC */
  APP_AdcInit();     
  
  /* Initialize TIM1 */
  TIM1_Init();
                            
  while (1)
  {
 /* DMA channel 1 transfer completed */
    if (__HAL_DMA_GET_FLAG(DMA1, DMA_ISR_TCIF1))         
    {
      /* Clear DMA channel 1 transfer complete flag */
      __HAL_DMA_CLEAR_FLAG(DMA1, DMA_IFCR_CTCIF1);       
    
    }   
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
    
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //GPIO_InitStruct.Pin = GPIO_PIN_6;
    //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
}

/**
  * @brief  TMR1 Configuration
  * @param  None
  * @retval None
  */
void TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    Tim1Handle.Instance = TIM1;
    Tim1Handle.Init.Period = 60 - 1;
    Tim1Handle.Init.Prescaler              =   1 - 1;
    Tim1Handle.Init.ClockDivision          =   TIM_CLOCKDIVISION_DIV1;
    Tim1Handle.Init.RepetitionCounter      =   1 - 1;
    Tim1Handle.Init.CounterMode            =   TIM_COUNTERMODE_UP;
    Tim1Handle.Init.AutoReloadPreload      =   TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_Base_Init(&Tim1Handle) != HAL_OK)                                /* Initialize timer base */
    {
    APP_ErrorHandler();
    }
    
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

    TimHandle.Instance = TIM3;                                                  /* Select TIM1 */
    TimHandle.Init.Period            = TRANSMIT_PERIOD - 1;                     /* Auto-reload value */
    TimHandle.Init.Prescaler         = 24 - 1;                                   /* Prescaler */
    TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;                  /* No clock division */
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;                      /* Up counting */
    TimHandle.Init.RepetitionCounter = 1 - 1;                                   /* No repetition counting */
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;          /* Auto-reload register not buffered */
    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)                                /* Initialize timer base */
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
    if (HAL_TIM_OC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)/* Configure OC1 channel */
    {
    APP_ErrorHandler();
    }
                                                       
    HAL_TIM_Base_Start(&TimHandle);
    HAL_TIM_OC_Start(&TimHandle,TIM_CHANNEL_1);
//    HAL_TIM_OC_Start_IT(&TimHandle,TIM_CHANNEL_1);
//    
//    HAL_NVIC_SetPriority((IRQn_Type)(TIM3_IRQn), 0, 0);
//    HAL_NVIC_EnableIRQ((IRQn_Type)(TIM3_IRQn));
//    
//    HAL_TIM_Base_Start_IT(&TimHandle);
}

void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&TimHandle);  // 处理中断
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
  
//    HAL_TIM_Base_Stop_IT(&TimHandle);
//    
//    
//    HAL_ADC_Start_IT(&AdcHandle);      

//    HAL_TIM_Base_Start(&TimHandle);
//    HAL_TIM_OC_Start(&TimHandle,TIM_CHANNEL_1);

//    HAL_TIM_Base_Start_IT(&TimHandle);
  }
}
/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
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
  AdcHandle.Init.ContinuousConvMode = DISABLE;                           /* Single conversion mode */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                        /* Disable discontinuous mode */
  
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
  
 
//  //= ADC中断读取 
//  HAL_NVIC_SetPriority((IRQn_Type)ADC_COMP_IRQn, 0, 0);  // 设置优先级（优先级组根据项目调整）
//  HAL_NVIC_EnableIRQ((IRQn_Type)ADC_COMP_IRQn);          // 使能ADC中断
  
   // 配置DMA传输
    HAL_SYSCFG_DMA_Req(0);                                      /* Set DMA1 mapping to ADC */
   
    DMA_HandleTypeDef hdma_adc;
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

//    
//      if (HAL_ADC_Start_DMA(&AdcHandle, &aADCxConvertedData, 1) != HAL_OK)            
//      {
//        APP_ErrorHandler();
//      }
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)adc_buffer , BUFFER_SIZE);
                                    
//  /* Enable ADC，and enable interrupt */
//  if (HAL_ADC_Start_IT(&AdcHandle) != HAL_OK)                                     
//  {
//    APP_ErrorHandler();
//  }
}


void ADC_COMP_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&AdcHandle);
}

  
#define WINDOW_SIZE 2
uint32_t buffer[WINDOW_SIZE], aindex = 0;
uint32_t adc_filter(uint32_t new_sample) {
    buffer[aindex] = new_sample;
    aindex = (aindex + 1) % WINDOW_SIZE;
    uint32_t sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) sum += buffer[i];
    return sum / WINDOW_SIZE;
}
  
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//    static uint8_t count = 0;

//    aADCxConvertedData = HAL_ADC_GetValue(hadc);
//      
//    aADCxConvertedData = adc_filter(aADCxConvertedData);
//  
//    count++;
//    if(count > 3)
//    {
//        count = 0;
//        if( (AD_value_max > AD_value_min)  && ((AD_value_max - AD_value_min) > 260))  
//        {
//            
//               
//                AD_value_max = COMP_VALUE;        
//                AD_value_min = COMP_VALUE;
//            
//                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
//                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
//            
//        }
//        else
//        {
//            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
//        }
//    }
// 
//    if (aADCxConvertedData > AD_value_max)
//    {
//        AD_value_max = aADCxConvertedData ;
//    }
//    if (aADCxConvertedData < AD_value_min)
//    {     
//        AD_value_min = aADCxConvertedData ;
//    }

//  HAL_ADC_Start_IT(&AdcHandle);
  //printf("ADC: %d\r\n", aADCxConvertedData);
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
