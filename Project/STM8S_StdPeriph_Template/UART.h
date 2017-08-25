#ifndef __UART_H
#define __UART_H

void USART_Initialization();
void UART1_Init_Fast();
void UART1_DeInit_Fast();
inline void UART1_ENABLE();
void USART_SendString(char * value);
void USART_SendChar(char value);


#endif