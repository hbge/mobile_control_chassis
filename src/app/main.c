#include "bsp_gpio.h"
#include "bsp_timer.h"
#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx_can.h"

#include <stdio.h>
#include <string.h>

#define TASK_SYNC_SEND_PRIO (configMAX_PRIORITIES - 30) 
#define TASK_SYNC_SEND_STK_SIZE 512
TaskHandle_t handle_sync_send;                          
void task_sync_send(void *pvParameters);

#define TASK_SYNC_RECV_PRIO (configMAX_PRIORITIES - 29) 
#define TASK_SYNC_RECV_STK_SIZE 512
TaskHandle_t handle_sync_recv;                          
void task_sync_recv(void *pvParameters);

#define TASK_START_PRIO (configMAX_PRIORITIES - 29) 
#define TASK_START_STK_SIZE 512
TaskHandle_t handle_start;                          
void task_start(void *pvParameters);

#define CANx                       CAN1
#define CAN_CLK                    RCC_APB1Periph_CAN1
#define CAN_RX_PIN                 GPIO_Pin_11
#define CAN_TX_PIN                 GPIO_Pin_12
#define CAN_GPIO_PORT              GPIOA
#define CAN_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define CAN_AF_PORT                GPIO_AF_CAN1
#define CAN_RX_SOURCE              GPIO_PinSource0
#define CAN_TX_SOURCE              GPIO_PinSource1 

void main(void)
{
//  __disable_irq();
//  
//  bsp_gpio_init();
//  bsp_timer_init();
//  bsp_uart_init();
//  
//  xTaskCreate(task_start, 
//              (const char *)"task_start",
//              (uint16_t)TASK_START_STK_SIZE,
//              (void *)NULL,
//              (UBaseType_t)TASK_START_PRIO,
//              (TaskHandle_t *)&handle_start);
//  
//  __disable_irq();
//  
//  vTaskStartScheduler();
  CanTxMsg TxMessage;
  GPIO_InitTypeDef  GPIO_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  
  SystemCoreClockUpdate();
  
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CANx);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 2;
  CAN_Init(CANx, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* Transmit Structure preparation */
  TxMessage.StdId = 0x321;
  TxMessage.ExtId = 0x01;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.DLC = 1;
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
  
  NVIC_InitTypeDef  NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  while(1)
  {
    CAN_Transmit(CANx, &TxMessage);
    /* Wait until one of the mailboxes is empty */
    while((CAN_GetFlagStatus(CANx, CAN_FLAG_RQCP0) !=RESET) || \
          (CAN_GetFlagStatus(CANx, CAN_FLAG_RQCP1) !=RESET) || \
          (CAN_GetFlagStatus(CANx, CAN_FLAG_RQCP2) !=RESET));
  }
  
}

void task_start(void *pvParameters)
{
  taskENTER_CRITICAL();
  
  xTaskCreate(task_sync_send, 
              (const char *)"task_sync_send",
              (uint16_t)TASK_SYNC_SEND_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)TASK_SYNC_SEND_PRIO,
              (TaskHandle_t *)&handle_sync_send);
  
  xTaskCreate(task_sync_recv, 
              (const char *)"task_sync_recv",
              (uint16_t)TASK_SYNC_RECV_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)TASK_SYNC_RECV_PRIO,
              (TaskHandle_t *)&handle_sync_recv);
  
  taskEXIT_CRITICAL();
  
  vTaskDelete(NULL);
}

void task_sync_send(void *pvParameters)
{
  uint32_t time_now = 0;
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS( 1000 );
  for(;;)
  {
    xLastWakeTime = xTaskGetTickCount();
    //接收到定时中断
    bsp_intrig_set_low();
    vTaskDelay(10);
    bsp_intrig_set_high();
    time_now = bsp_timer_get();
    vTaskDelay(100);
    bsp_intrig_set_low();
    vTaskDelayUntil( &xLastWakeTime, xPeriod);
    
    timesync_write(time_now);
    
    printf("%u\r\n", time_now);
  }
}

void task_sync_recv(void *pvParameters)
{
  for(;;)
  {
    vTaskDelay(1000);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  printf("assert failed file is %s\r\n", file);
  printf("assert failed file is %d\r\n", line);
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif