#ifndef __BPS_TIMER_H_
#define __BSP_TIMER_H_

#include "stm32f4xx_tim.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#ifdef __cplusplus
extern "C"{
#endif
  
#define EV_SEND	(1<< 0)
  
extern EventGroupHandle_t xEventGroup;
  
int16_t bsp_timer_init(void);
uint32_t bsp_timer_get(void);
  
#ifdef __cplusplus
}
#endif

#endif