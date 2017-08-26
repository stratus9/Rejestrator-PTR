/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include <stdlib.h>
#include "Timers.h"

void Delay(uint32_t nCount){
    /* Decrement nCount value */
    while (nCount != 0){
        nCount--;
    }
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
  //TIM2_DeInit();
  //TIM2_TimeBaseInit(TIM2_PRESCALER_16384, 97);    //10 = X/(1024*6630) Fclk = 15.912.000 ~fcpu
  //TIM2_TimeBaseInit(TIM2_PRESCALER_16, 9945);    //~100Hz
  
  TIM2->PSCR = 0x0E;
  TIM2->ARRH = 0x00;
  TIM2->ARRL = 0x61;
  
  //TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  TIM2->IER |= TIM2_IT_UPDATE;

  /* TIM1 counter enable */
  TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
}

//==========================================================================================================
//                              ISR
//==========================================================================================================
void Timer2_ISR(){    
  if(beep_trigger){
    beep++;
    if(beep > 8) beep = 0;      //bylo 20
    if(beep == 4) Beep_Start();
    else Beep_Stop();
  }
  else Beep_Stop();
  
  if((state_d.button < 250) && (!(GPIOD->IDR & 0x02))) state_d.button++;
  else state_d.button = 0;
}

//==========================================================================================================
//                              Buzzer
//==========================================================================================================

void Beep_Initialization(){
  BEEP->CSR = (10) | 0x40;
  
  //BEEP_Init(BEEP_FREQUENCY_2KHZ);       //BEEP_FREQUENCY_1KHZ, BEEP_FREQUENCY_2KHZ, BEEP_FREQUENCY_4KHZ
}

inline void Beep_Start(){
  /* Enable the BEEP peripheral */
    BEEP->CSR |= BEEP_CSR_BEEPEN;
}

inline void Beep_Stop(){
  BEEP->CSR &= (~BEEP_CSR_BEEPEN);
}