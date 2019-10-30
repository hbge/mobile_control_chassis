#include "bsp_gpio.h"

int16_t bsp_gpio_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOG Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  
  GPIO_ResetBits(GPIOG, GPIO_Pin_14 | GPIO_Pin_13);

  /* Configure PG6 and PG8 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  return 0;
}

/* 使用GPIOG.13进行输入触发 */
inline void bsp_intrig_set_high(void)
{
  GPIO_SetBits(GPIOG, GPIO_Pin_13);
}

/* 使用GPIOG.13进行输入触发 */
inline void bsp_intrig_set_low(void)
{
  GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}

/* 使用GPIOG.14进行输出触发 */
inline void bsp_outtrig_set_high(void)
{
  GPIO_SetBits(GPIOG, GPIO_Pin_14);
}

/* 使用GPIOG.14进行输出触发 */
inline void bsp_outtrig_set_low(void)
{
  GPIO_ResetBits(GPIOG, GPIO_Pin_14);
}