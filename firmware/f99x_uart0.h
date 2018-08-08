#ifndef _UART0_H_
#define _UART0_H_

void uart0_bauding(unsigned long baudrate);
void uart0_init();
void uart0_init_boot();

#include "c8051f990.h"
#include "fifo.h"

FIFO_SIZE_T uart0_write(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart0_read(char *buf, FIFO_SIZE_T size);
FIFO_SIZE_T uart0_tx_margin();
FIFO_SIZE_T uart0_rx_size();

void interrupt_uart0() __interrupt (INTERRUPT_UART0);

#define uart0_tx_active() (TB80 == 1)

// For stdio.h
char getchar();
void putchar(char c);

#endif
