#include "bsp_uart.h"
#include "bsp_timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

unsigned char isGetTime = 0;

int16_t bsp_uart_init(void)
{
  /* HMI串口 */
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1,&USART_InitStructure);
  USART_Cmd(USART1 ,ENABLE);
  
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  
  NVIC_Init(&NVIC_InitStructure);
  
  /* 对时串口 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART3,&USART_InitStructure);
  USART_Cmd(USART3 ,ENABLE);
  
  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  
  NVIC_Init(&NVIC_InitStructure);

  return 0;
}

void timesync_write(uint64_t time)
{
  uint8_t tmp[8] = {0};
  uint8_t i;
  
  tmp[0] = (uint8_t)time;
  tmp[1] = (uint8_t)(time>>8);
  tmp[2] = (uint8_t)(time>>16);
  tmp[3] = (uint8_t)(time>>24);
  
  tmp[4] = (uint8_t)(time>>32);
  tmp[5] = (uint8_t)(time>>40);
  tmp[6] = (uint8_t)(time>>48);
  tmp[7] = (uint8_t)(time>>56);
  
  for(i=0; i<8; i++)
  {
    USART_SendData(USART3, (uint8_t)tmp[i]);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
  }
}

int fputc(int ch, FILE* f)
{
  USART_SendData(USART1, (uint8_t)ch);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  return (ch);
}

void USART3_IRQHandler(void)
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE))
  {
    USART_ClearFlag(USART3, USART_IT_RXNE);
  }
}

//uint8_t data[14] = {0};
//uint64_t time_get = 0;
//uint32_t time_get_u32 = 0;
//
//uint32_t time_local = 0;
//int32_t time_diff = 0;

void USART1_IRQHandler(void)
{
//  uint8_t res;
//  static uint8_t i = 0;
  //
  if(USART_GetITStatus(USART1, USART_IT_RXNE))
  {
//    //
//    res=USART_ReceiveData(USART1);
//    if(i==0 && res == 0x0A)
//    {
//      data[0] = 0x0A;
//      i++;
//    }
//    else if(i==1 && res == 0x0D)
//    {
//      data[1] = 0x0D;
//      i++;
//    }
//    else if(i==2 && res == 0x01)
//    {
//      data[2] = 0x01;
//      i++;
//    }
//    else if(i==3 && res == 0x08)
//    {
//      data[3] = 0x08;
//      i++;
//    }
//    else if(i>3 && i<14)
//    {
//      data[i] = res;
//      i++;
//    }
//    else if(i>=14)
//    {
//      i = 0;
//      
//      if(isGetTime == 1)
//      {
//        time_get = (uint64_t)data[11] | ((uint64_t)data[10]<<8) |\
//                   ((uint64_t)data[9]<<16) | ((uint64_t)data[8]<<24) |\
//                   ((uint64_t)data[7]<<32) | ((uint64_t)data[6]<<40) |\
//                   ((uint64_t)data[5]<<48) | ((uint64_t)data[4]<<56);
//        
//        time_get_u32 = (uint32_t)time_get;
//        
//        time_local = bsp_timer_get();
//        
//        time_diff = time_get - time_local;
//        
//        isGetTime = 0;
//        
//        memset(data, 0 ,sizeof(data));
//      }
//    }
  }
}

