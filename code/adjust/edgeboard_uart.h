#ifndef _edgeboard_uart_h
#define _edgeboard_uart_h

#include "global.h"

extern uint8 buff;

void USART_Receive_Call_Back();
void UART_indata(void);
void UART_send_int16(int16 send_int16);
void UART_send_data(void);
void UART_PID_adjust(void);
void UART_send_car_start(void);
#endif
