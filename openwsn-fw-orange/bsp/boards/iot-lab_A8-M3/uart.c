/**
\brief iot-lab_A8-M3 definition of the "uart" bsp module (based on openmoteSTM32 code).

\author Chang Tengfei <tengfei.chang@gmail.com>,  July 2012.
\author Alaeddine Weslati <alaeddine.weslati@inria.fr>,  January 2014.
\author Quentin Lampin <quentin.lampin@orange.com>, January 2015.
*/
#ifdef SENSORLAB
#include "opendefs.h"
#endif
#include "stm32f10x_lib.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "uart.h"
#ifndef SENSORLAB
#include "leds.h"
#endif

#include "rcc.h"
#include "nvic.h"

//=========================== defines =========================================

//=========================== variables =======================================
#ifdef SENSORLAB
typedef struct {
  uint16_t tx_buffer_size;
  uint16_t rx_buffer_size;
  uart_tx_completed_cbt dma_tx_complete_cb;
  uart_rx_cbt uart_rx_cb;
  DMA_InitTypeDef DMA_InitStructure;
} uart_vars_t;

static uart_vars_t uart_vars;
#else
typedef struct {
   uart_tx_cbt txCb;
   uart_rx_cbt rxCb;
} uart_vars_t;

uart_vars_t uart_vars;
#endif

//=========================== prototypes ======================================

//=========================== public ==========================================
#ifdef SENSORLAB
bool uart_DMA_available(void)
{
  return TRUE;
}

void uart_init()
{
  /* initialize the UART in FULL DUPLEX TX/RX */
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* reset local variables */
  memset(&uart_vars,0,sizeof(uart_vars_t));

  /* Enable UART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed           = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure USART DMA Interruptions
  NVIC_uart();
  NVIC_dma();

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate              = 500000;
  USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits              = USART_StopBits_1;
  USART_InitStructure.USART_Parity                = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  /* configure DMA TX on DMA1 channel 4 */
  DMA_StructInit(&(uart_vars.DMA_InitStructure)); // setup configuration for memory to UART1
  uart_vars.DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
  uart_vars.DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  uart_vars.DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  uart_vars.DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  uart_vars.DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  uart_vars.DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  uart_vars.DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  uart_vars.DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  uart_vars.DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  /* Enable RX interrupt */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  /* Disable TX interrupt */
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  /* Enable USART1 DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
}

void uart_DMA_write(void* buffer, uint16_t buffer_size, uart_tx_completed_cbt callback)
{
  uart_vars.tx_buffer_size = buffer_size;
  uart_vars.dma_tx_complete_cb = callback;

  uart_vars.DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer;
  uart_vars.DMA_InitStructure.DMA_BufferSize = buffer_size;

  DMA_DeInit(DMA1_Channel4); // reset DMA configuration to default values
  DMA_Init(DMA1_Channel4, &(uart_vars.DMA_InitStructure));

  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE); // setup Transfert complete interrupt
  DMA_Cmd(DMA1_Channel4, ENABLE);
}

uint8_t uart_readByte(){
  uint16_t temp;
  temp = USART_ReceiveData(USART1);
  return (uint8_t)temp;
}
#else
void uart_init() 
{
  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* reset local variables */
  memset(&uart_vars,0,sizeof(uart_vars_t));

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed           = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate              = 500000;
  USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits              = USART_StopBits_1;
  USART_InitStructure.USART_Parity                = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // make sure no interrupts fire as we enable the UART
  uart_clearTxInterrupts();
  uart_clearRxInterrupts();
   
  // enable USART1
  USART_Cmd(USART1, ENABLE);
    
  // enable NVIC uart
  NVIC_uart();
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) 
{
  uart_vars.txCb = txCb;
  uart_vars.rxCb = rxCb;

}

void uart_enableInterrupts()
{
  USART_ITConfig(USART1, USART_IT_TC,   ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void uart_disableInterrupts()
{
  USART_ITConfig(USART1, USART_IT_TC,   DISABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
}

void uart_clearRxInterrupts()
{
  USART_ClearFlag(USART1, USART_FLAG_RXNE);
}

void uart_clearTxInterrupts()
{
  USART_ClearFlag(USART1, USART_FLAG_TC);
}

void uart_writeByte(uint8_t byteToWrite)
{ 
  USART_SendData(USART1, byteToWrite);
}

uint8_t uart_readByte()
{
  uint16_t temp;
  temp = USART_ReceiveData(USART1);
  return (uint8_t)temp;
}
#endif
//=========================== interrupt handlers ==============================
#ifdef SENSORLAB
kick_scheduler_t uart_dma_tx_complete_isr(void)
{
  uart_vars.dma_tx_complete_cb(uart_vars.tx_buffer_size);
  return DO_NOT_KICK_SCHEDULER;
}
kick_scheduler_t uart_rx_isr()
{
  uart_vars.uart_rx_cb();
  return DO_NOT_KICK_SCHEDULER;
}
#else
kick_scheduler_t uart_tx_isr() 
{
  uart_clearTxInterrupts();
  uart_vars.txCb();
  return DO_NOT_KICK_SCHEDULER;
}

kick_scheduler_t uart_rx_isr() 
{
  uart_clearTxInterrupts();
  uart_vars.rxCb();
  return DO_NOT_KICK_SCHEDULER;
}
#endif

#ifdef SENSORLAB
//=========================== callback setters =================================
void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb)
{
  //txCb ignored in this implementation (we're using the DMA)
  uart_vars.uart_rx_cb = rxCb;
}

//=========================== legacy handlers ==================================
void uart_enableInterrupts(){}
void uart_disableInterrupts(){}
void uart_clearRxInterrupts(){}
void uart_clearTxInterrupts(){}
void uart_writeByte(uint8_t byteToWrite){}

kick_scheduler_t uart_tx_isr() { return DO_NOT_KICK_SCHEDULER;}
#endif

