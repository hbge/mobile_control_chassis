#ifndef __BSP_GPIO_H_
#define __BSP_GPIO_H_

#include "stm32f4xx_gpio.h"

#ifdef __cplusplus
extern "C"{
#endif

#define TRI_IN_PORT  GPIOA
#define TRI_OUT_PORT GPIOA
  
#define TRI_IN_PIN  GPIO_Pin_9
#define TRO_OUT_PIN GPIO_Pin_10
  
int16_t bsp_gpio_init(void);

void bsp_intrig_set_high(void);
void bsp_intrig_set_low(void);
void bsp_outtrig_set_high(void);
void bsp_outtrig_set_low(void);
  
#ifdef __cplusplus
}
#endif
  
#endif