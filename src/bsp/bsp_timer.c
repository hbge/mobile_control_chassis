#include "bsp_timer.h"

#include "stm32f4xx_tim.h"

#include "bsp_gpio.h"

EventGroupHandle_t xEventGroup = NULL;

int16_t bsp_timer_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
//  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFE;
  TIM_TimeBaseStructure.TIM_Prescaler = 90-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_SetCounter(TIM2, 0);
  
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//  
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = 1000-1;
//  TIM_TimeBaseStructure.TIM_Prescaler = 90-1;
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//  
//  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//  TIM_ARRPreloadConfig(TIM3, ENABLE);
//  TIM_SetCounter(TIM3, 0);
//  
//  /* TIM3 enable counter */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
//	NVIC_Init(&NVIC_InitStructure);  
  
  xEventGroup = xEventGroupCreate();
  if(xEventGroup == NULL)
  {
    return -1;
  }
  
  xEventGroupClearBits(xEventGroup, EV_SEND);
  
  TIM_Cmd(TIM2, ENABLE);
  
  return 0;
}

uint32_t bsp_timer_get(void)
{
  return TIM_GetCounter(TIM2);
}

//延时函数，单位us
void bsp_timer_delayus(uint32_t us)
{
  uint32_t time_now = 0;
  if(us == 0)
  {
    return;
  }
  else
  {
    time_now = bsp_timer_get();
    while((bsp_timer_get() - time_now) < us);
  }
}

//延时函数，单位ms
void bsp_timer_delayms(uint32_t ms)
{
  uint32_t time_now = 0;
  if(ms == 0)
  {
    return;
  }
  else
  {
    time_now = bsp_timer_get();
    while(((bsp_timer_get() - time_now))*1000 < ms);
  }
}

//void TIM3_IRQHandler(void)   
//{
//  BaseType_t pxHigherPriorityTaskWoken;
//    
//	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  
//  {
//    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//    xEventGroupSetBitsFromISR(xEventGroup, EV_SEND, &pxHigherPriorityTaskWoken);
//    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
//  }
//}