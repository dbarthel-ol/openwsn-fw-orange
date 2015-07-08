#ifndef __UART_H
#define __UART_H

/**
\addtogroup BSP
\{
\addtogroup uart
\{

\brief Cross-platform declaration "uart" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
\author Quentin Lampin <quentin.lampin@orange.com>, February 2015.
*/

#include "stdint.h"
#include "board.h"
 
//=========================== define ==========================================

//=========================== typedef =========================================

typedef enum {
   UART_EVENT_THRES,
   UART_EVENT_OVERFLOW,
} uart_event_t;

typedef void (*uart_tx_cbt)(void);
typedef void (*uart_rx_cbt)(void);
#ifdef SENSORLAB
typedef void (*uart_tx_completed_cbt)(uint16_t bytes_written);
#endif

//=========================== variables =======================================

//=========================== prototypes ======================================

void    uart_init(void);
void    uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb);
void    uart_enableInterrupts(void);
void    uart_disableInterrupts(void);
void    uart_clearRxInterrupts(void);
void    uart_clearTxInterrupts(void);
void    uart_writeByte(uint8_t byteToWrite);
#ifdef FASTSIM
void    uart_writeCircularBuffer_FASTSIM(uint8_t* buffer, uint8_t* outputBufIdxR, uint8_t* outputBufIdxW);
#endif
uint8_t uart_readByte(void);

// interrupt handlers
kick_scheduler_t uart_tx_isr(void);
kick_scheduler_t uart_rx_isr(void);
#ifdef SENSORLAB
kick_scheduler_t uart_dma_tx_complete_isr(void);


// DMA related handlers
bool uart_DMA_available(void);
void uart_DMA_write(void* buffer, uint16_t buffer_size, uart_tx_completed_cbt callback);
#endif

/**
\}
\}
*/

#endif
