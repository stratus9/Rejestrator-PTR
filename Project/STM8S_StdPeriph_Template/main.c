/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   Main program body
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/


void main(void)
{
  //----------------Select fCPU = 16MHz--------------------------//
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);      //118 B
  
  //----------------------Init UART------------------------------//
  //USART_Initialization();       //1182 B -> 85 B
  
  //------------------------Init I2C-----------------------------//
  //I2C_Initialization();
  
  
  //-----------------------Init GPIO-----------------------------//
  //GPIO_Initialization();        //263 B -> 57 B
   
  //----------------------Init Timers---------------------------//
  //Delay(10000);
  //Timer1_Init();                //716 B
  //Timer2_Init();                //516 B
  //enableInterrupts();
  
  //----------------------Init Buzzer---------------------------//
  // Beep_Start();
  // Beep_Stop();
  Beep_Initialization(); 

  while (1){
  }
}

/**
  * @brief  Delay.
  * @param  nCount
  * @retval None
  */
void Delay(uint32_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

//======================================================================================
//                              GPIO
//======================================================================================

void GPIO_Initialization(void){
  /*
  GPIO_Init(GPIOD, (GPIO_Pin_TypeDef)GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);   //Si4463 CS   //13 B
  GPIO_WriteHigh (GPIOD, GPIO_PIN_2);         //9 B
  
  GPIO_Init(GPIOC, (GPIO_Pin_TypeDef)GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);   //13 B
  
  GPIO_Init(GPIOD, (GPIO_Pin_TypeDef)GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST);   //13 B
  GPIO_WriteHigh (GPIOD, GPIO_PIN_4);           //9 B
  */
  GPIO_Init_Fast();   //57 B
}

void GPIO_Init_Fast(){
  //PD2 - SI4463 CS pin
  GPIOD->CR2 &= ~0x04;  //slow slope 2MHz
  GPIOD->ODR |= 0x04;   //Init High state
  GPIOD->DDR |= 0x04;   //Output
  GPIOD->CR1 |= 0x04;   //Push-Pull
  GPIOD->ODR |= 0x04;   //Init High state
  
  //PC3 - ??
  GPIOC->CR2 &= ~0x08;  //slow slope 2MHz
  GPIOC->DDR |= 0x08;   //Output
  GPIOC->CR1 |= 0x08;   //Push-Pull
  
  //PD4 - ??
  GPIOD->CR2 &= ~0x10;  //slow slope 2MHz
  GPIOD->ODR |= 0x10;   //Init High state
  GPIOD->DDR |= 0x10;   //Output
  GPIOD->CR1 |= 0x10;   //Push-Pull
  GPIOD->ODR |= 0x10;   //Init High state
}

//=======================================================================================
//                              UART
//=======================================================================================
void USART_Initialization(void){
  UART1_DeInit_Fast();     //47 B
  
  //UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE); //1163 B
  UART1_Init_Fast();    //29 B
  UART1_ENABLE();       //19 B -> 9 B
}

void UART1_Init_Fast(){
  #define UART1_baud 115200
  #define UART1_BBR1 0x08
  #define UART1_BBR2 0x0B

  UART1->CR1 = 0x00;           //set 8-bit word length, no parity, UART enabled (change for low power)
  UART1->CR3 = 0x00;           //set 1 bit Stop

  UART1->BRR1 = UART1_BBR1; 
  UART1->BRR2 = UART1_BBR2;       
   
  UART1->CR2 |= 0x08 | 0x04;    //RX i TX enabled
}

void UART1_DeInit_Fast(){
  /* Clear the Idle Line Detected bit in the status register by a read
  to the UART1_SR register followed by a Read to the UART1_DR register */
  
  (void)UART1->SR;    //3B
  (void)UART1->DR;    //3B
  
  UART1->BRR2 = 0x00;  // Set UART1_BRR2 to reset value 0x00    //4B
  UART1->BRR1 = 0x00;  // Set UART1_BRR1 to reset value 0x00    //4B
  
  UART1->CR1 = 0x00;  // Set UART1_CR1 to reset value 0x00    //4B
  UART1->CR2 = 0x00;  // Set UART1_CR2 to reset value 0x00    //4B
  UART1->CR3 = 0x00;  // Set UART1_CR3 to reset value 0x00    //4B
  UART1->CR4 = 0x00;  // Set UART1_CR4 to reset value 0x00    //4B
  UART1->CR5 = 0x00;  // Set UART1_CR5 to reset value 0x00    //4B
  
  UART1->GTR = 0x00;    //4B
  UART1->PSCR = 0x00;   //4B
}

void UART1_ENABLE(){
    /* UART1 Enable */
    UART1->CR1 &= (~0x20); 
}

void USART_SendString(char * value){
  while(*value){
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET){}
    UART1_SendData8(*value++);
  }
}

void USART_SendChar(char value){
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET){}
  UART1_SendData8(value);
}

//==========================================================================================================
//                              Timer
//==========================================================================================================

void Timer1_Init(){
  TIM1_DeInit();
  /* Time Base configuration */
  /*
  TIM1_Period = 4095
  TIM1_Prescaler = 0
  TIM1_CounterMode = TIM1_COUNTERMODE_UP
  TIM1_RepetitionCounter = 0
  */
  TIM1_TimeBaseInit(1032, TIM1_COUNTERMODE_UP, 6, 0);
  
  /*
  TIM1_OCMode = TIM1_OCMODE_PWM2
  TIM1_OutputState = TIM1_OUTPUTSTATE_ENABLE
  TIM1_OutputNState = TIM1_OUTPUTNSTATE_ENABLE
  TIM1_Pulse = CCR1_Val
  TIM1_OCPolarity = TIM1_OCPOLARITY_LOW
  TIM1_OCNPolarity = TIM1_OCNPOLARITY_HIGH
  TIM1_OCIdleState = TIM1_OCIDLESTATE_SET
  TIM1_OCNIdleState = TIM1_OCIDLESTATE_RESET
  */
  TIM1_OC3Init(TIM1_OCMODE_PWM1, 
               TIM1_OUTPUTSTATE_ENABLE, 
               TIM1_OUTPUTNSTATE_ENABLE,
               3,    //by³o 3
               TIM1_OCPOLARITY_LOW, 
               TIM1_OCNPOLARITY_HIGH, 
               TIM1_OCIDLESTATE_SET,
               TIM1_OCNIDLESTATE_RESET); 
  
  //TIM1_CCPreloadControl(ENABLE);
  TIM1_ITConfig(TIM1_IT_COM, DISABLE);

  /* TIM1 counter enable */
  TIM1_Cmd(ENABLE);
  
  /* TIM1 Main Output Enable */
  TIM1_CtrlPWMOutputs(ENABLE);
}

void Timer2_Init(){
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_2, 6630);    //1200 = X/(2*6630) Fclk = 15.912.000 ~fcpu
  
  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);

  /* TIM1 counter enable */
  TIM2_Cmd(ENABLE);
}

void Timer2_ISR(){
}

void Beep_Initialization(void){
  BEEP_DeInit();
  BEEP->CSR &= 0xE0;
  BEEP->CSR |= 10;
  
  BEEP_Init(BEEP_FREQUENCY_2KHZ);       //BEEP_FREQUENCY_1KHZ, BEEP_FREQUENCY_2KHZ, BEEP_FREQUENCY_4KHZ
  BEEP_Cmd(DISABLE);
}

inline void Beep_Start(void){
  BEEP_Cmd(ENABLE);
}

inline void Beep_Stop(void){
  BEEP_Cmd(DISABLE);
}




#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
