#ifndef __BSP_UART_H_
#define __BSP_UART_H_

#include "stm32f4xx_conf.h"

#ifdef __cplusplus
extern "C"{
#endif
  
extern unsigned char isGetTime;
  
int16_t bsp_uart_init(void);
void timesync_write(uint64_t time);
  
#ifdef __cplusplus
}
#endif

#endif