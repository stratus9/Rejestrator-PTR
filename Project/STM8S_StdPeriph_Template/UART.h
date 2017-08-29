#ifndef __UART_H
#define __UART_H

void USART_Initialization();
void UART1_Init_Fast();
void UART1_DeInit_Fast();
inline void UART1_ENABLE();
void USART_SendString(uint8_t * value);
void USART_SendChar(uint8_t value);


#endif