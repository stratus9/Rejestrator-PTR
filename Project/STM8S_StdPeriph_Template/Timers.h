#ifndef __TIMERS_H
#define __TIMERS_H

void Delay(uint32_t nCount);
void Timer1_Init();
void Timer2_Init();
void Timer2_ISR();
void Beep_Initialization();
inline void Beep_Start();
inline void Beep_Stop();

#endif