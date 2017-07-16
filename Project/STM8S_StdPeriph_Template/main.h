#ifndef __MAIN_H
#define __MAIN_H

/* Private function prototypes -----------------------------------------------*/
void Delay(uint32_t);
void USART_Initialization(void);
void UART1_Init_Fast();
void UART1_DeInit_Fast();
void UART1_ENABLE();
void USART_SendString(char *);
void USART_SendChar(char);

void GPIO_Initialization(void);
void GPIO_Init_Fast();

void Timer1_Init();
void Timer2_Init();
void Timer2_ISR();



#endif /* __MAIN_H */