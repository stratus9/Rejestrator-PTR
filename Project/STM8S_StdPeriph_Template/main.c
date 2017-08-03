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
#include "fonts.h"

/* Private defines -----------------------------------------------------------*/

/* Private var ---------------------------------------------------------------*/
OLED_t OLED_buffer;


/* Private functions ---------------------------------------------------------*/


void main(void)
{
  //----------------Select fCPU = 16MHz--------------------------//
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);      //118 B
  
  //----------------------Init UART------------------------------//
  //USART_Initialization();       //1182 B -> 85 B
  
  //------------------------Init I2C-----------------------------//
  I2C_Initialization();
  
  
  //-----------------------Init GPIO-----------------------------//
  // LED_BLUE(1/0);
  // LED_GREEN(1/0);
  GPIO_Initialization();        //263 B -> 57 B
   
  //----------------------Init Timers---------------------------//
  //Delay(10000);
  //Timer1_Init();                //716 B
  Timer2_Init();                //516 B
  enableInterrupts();
  
  //----------------------Init Buzzer---------------------------//
  // Beep_Start();
  // Beep_Stop();
  Beep_Initialization(); 
  
  //0x3A - ADXL345 (0x1D<<1); 0xEE - BMP280 (0x77<<1);  0x3C<<1 - OLED
  OLED_Reset();
  OLED_SendCommand(0xAE);//wy³¹cz panel OLED
  //OLED_SendCommand(0x00);//adres kolumny LOW
  //OLED_SendCommand(0x10);//adres kolumny HIGH
  //OLED_SendCommand(0x40);//adres startu linii 
  
  OLED_SendCommand(0x20);//tryb adresowania strony 
  OLED_SendCommand(0x02);
  
  OLED_SendCommand(0x81);//ustaw kontrast
  OLED_SendCommand(0xCF);
  
  OLED_SendCommand(0xA0);//ustaw remapowanie    //by³o A1
  OLED_SendCommand(0xC8);//kierunek skanowania //by³o C0
  
  OLED_SendCommand(0xA8);//ustaw multiplex ratio 
  OLED_SendCommand(31);//1/64
  
  OLED_SendCommand(0xA6);//wyœwietlanie bez inwersji 
  OLED_SendCommand(0xD3);//ustaw display offset 
  OLED_SendCommand(0x00);//bez offsetu
  OLED_SendCommand(0xD5);//ustaw divide ratio/czêstotliwoœæ oscylatora
  OLED_SendCommand(0x80);//100ramek/sec
  OLED_SendCommand(0xD9);//ustaw okres pre charge
  OLED_SendCommand(0xF1);//pre charge 15 cykli, discharge 1 cykl
  OLED_SendCommand(0xDA);//konfiguracja wyprowadzeñ sterownika
  OLED_SendCommand(0x12);
  OLED_SendCommand(0xDB);//ustawienie vcomh
  OLED_SendCommand(0x40);
  OLED_SendCommand(0x8D);//ustawienie Charge Pump
  OLED_SendCommand(0x14);
  OLED_SendCommand(0xA4);//"pod³¹czenie" zawartoœci RAM do panelu OLED
  OLED_SendCommand(0xA6);//wy³¹czenie inwersji wyœwietlania
  OLED_SendCommand(0xAF);//w³¹cza wyœwietlacz
  OLED_setContrast(1);
  Delay(100000);
  LED_GREEN(0);
  
  OLED_Clear(&OLED_buffer, 0);
  //OLED_dispTxt(&OLED_buffer, 0,0, "Vmax: 123   m/s",12,1);
  //OLED_dispTxt(&OLED_buffer, 0,10,"Amax: 12.1  m/s^",12,1);
  //OLED_dispTxt(&OLED_buffer, 0,21,"Hmax: 32154 m",12,1);
  OLED_drawPlotTemplate(&OLED_buffer, 'h');
  
  OLED_RefreshRAM(&OLED_buffer);

  while (1){
    LED_BLUE(1);
    Delay(300000);
    LED_BLUE(0);
    Delay(100000);
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
  //PC3 - LED1
  GPIOC->CR2 &= ~0x08;  //slow slope 2MHz
  GPIOC->DDR &= ~0x08;  //Input
  
  //PA1 - LED2
  GPIOA->CR2 &= ~0x02;  //slow slope 2MHz
  GPIOA->DDR &= ~0x02;  //Intput
  
  //PA2 - LED3  - wada w PCB v1.0; W wersji v1.0 zworka z +3V3
  //GPIOA->CR2 &= ~0x04;  //slow slope 2MHz
  //GPIOA->DDR |= 0x04;   //Output
  //GPIOA->CR1 |= 0x04;   //Push-Pull
  
  //PC4 - I2C Pull-up
  GPIOC->CR2 &= ~0x10;  //slow slope 2MHz
  GPIOC->DDR |= 0x10;   //Output
  GPIOC->CR1 |= 0x10;   //Push-Pull
  GPIOC->ODR |= 0x10;   //Enable I2C Pull-up
  
  //PD2 - LCD RES#
  GPIOD->CR2 &= ~0x04;  //slow slope 2MHz
  GPIOD->DDR |= 0x04;   //Output
  GPIOD->CR1 |= 0x04;   //Push-Pull
  GPIOD->ODR |= 0x04;   //Enable I2C Pull-up
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
  TIM2_TimeBaseInit(TIM2_PRESCALER_16384, 975);    //1 = X/(1024*6630) Fclk = 15.912.000 ~fcpu
  //TIM2_TimeBaseInit(TIM2_PRESCALER_16, 9945);    //~100Hz
  
  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);

  /* TIM1 counter enable */
  TIM2_Cmd(ENABLE);
}

void Timer2_ISR(){
  
}

//==========================================================================================================
//                              Buzzer
//==========================================================================================================

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


//==========================================================================================================
//                              RGB LEDs
//==========================================================================================================

/*
  //PC3 - LED1
  GPIOC->CR2 &= ~0x08;  //slow slope 2MHz
  GPIOC->DDR |= 0x08;   //Output
  GPIOC->CR1 |= 0x08;   //Push-Pull
  
  //PA1 - LED2
  GPIOA->CR2 &= ~0x02;  //slow slope 2MHz
  GPIOA->DDR |= 0x02;   //Output
  GPIOA->CR1 |= 0x02;   //Push-Pull

*/

void LED_BLUE(uint8_t value){
  if(value){
    GPIOC->DDR |= 0x08;   //Output
    GPIOC->CR1 |= 0x08;   //Push-Pull
    GPIOC->ODR &= ~0x08;
  }
  else{
     GPIOC->DDR &= ~0x08;   //Input
     GPIOC->ODR |= 0x08;    //Pull-up
  }
}

void LED_GREEN(uint8_t value){
  if(value){
    GPIOA->DDR |= 0x02;   //Output
    GPIOA->CR1 |= 0x02;   //Push-Pull
    GPIOA->ODR &= ~0x02;
  }
  else{
     GPIOA->DDR &= ~0x02;   //Input
     GPIOA->ODR |= 0x02;    //Pull-up
  }
}

//==========================================================================================================
//                              I2C
//==========================================================================================================

void I2C_Initialization(void){
  I2C_DeInit();
  I2C_Init(300000, 0xAA, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);
  I2C_Cmd(ENABLE);
  
  
}

void I2C_SendOneByte(uint8_t address, uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C_GenerateSTART(ENABLE);
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = address | 0x00;                        // Transmit address+E
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C_GenerateSTOP(ENABLE);                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void OLED_Reset(){
  GPIOD->ODR &= ~0x04;   //Enable I2C Pull-down
  Delay(100000);
  GPIOD->ODR |= 0x04;   //Enable I2C Pull-up
  Delay(100000);
}

void OLED_SendCommand(uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C_GenerateSTART(ENABLE);
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = 0x3C<<1;                            // Transmit address+W
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = 0x00;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C_GenerateSTOP(ENABLE);                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void OLED_SendData(uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C_GenerateSTART(ENABLE);
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = 0x3C<<1;                            // Transmit address+W
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = 0x40;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C_GenerateSTOP(ENABLE);                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void OLED_RefreshRAM(OLED_t * OLED){
  uint8_t i, j;
  
  for (i = 0; i < 4; i ++) { 
    OLED_SendCommand(0xB0 + i);
    OLED_SetColStart();   
    for (j = 0; j < 96; j ++) {
      OLED_SendData(OLED->OLED_dispBuff[j][i]); 
    }
  }  
}

void OLED_Clear(OLED_t * OLED, uint8_t fill){ 
  uint8_t i, j;
  
  for (i = 0; i < 4; i ++) {
    for (j = 0; j < 96; j ++) {
      OLED->OLED_dispBuff[j][i] = fill;
    }
  }
  
  OLED_RefreshRAM(OLED);//zawartoœæ bufora do RAM obrazu
} 

void OLED_SetColStart(void){ 
    OLED_SendCommand(0x00); //low
    OLED_SendCommand(0x12); //high
}


void OLED_DrawPoint(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t p){
  uint8_t chPos, chBx, chTemp = 0;
  
  if (x > 95 || y > 31) {
    return;
  }
  chPos = y / 8; 
  chBx = y % 8;
  chTemp = 1 << (chBx);
  
  if (p) {
    OLED->OLED_dispBuff[x][chPos] |= chTemp;
    
  } else {
    OLED->OLED_dispBuff[x][chPos] &= ~chTemp;
  }
}

//wyœwietlenie jednego znaku 
//argumenty:
//x,y - wspó³rzêdne na ekranie
//Chr - kod ASCII znaku 
//size - rozmiar 12, lub 16
//mode=1 znak wyœwietlany normalnie, mode=0 znak wyœwietlany w negatywie
//*******************************************************************************
void OLED_displayChar(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t Chr, uint8_t size, uint8_t mode){    
  uint8_t i, j;
  uint8_t chTemp, chYpos0 = y;
  
  Chr = Chr - ' ' + 1;          
  for (i = 0; i < size; i ++) {  
    if (size == 12) {
      if (mode) {
        chTemp = c_chFont1206[Chr][i];
      } else {
        chTemp = ~c_chFont1206[Chr][i];
      }
    } else {
      if (mode) {
        chTemp = c_chFont1608[Chr][i];
      } else {
        chTemp = ~c_chFont1608[Chr][i];
      }
    }
    
    for (j = 0; j < 8; j ++) {
      if (chTemp & 0x80) {
        OLED_DrawPoint(OLED, x, y, 1);
      } else {
        OLED_DrawPoint(OLED, x, y, 0);
      }
      chTemp <<= 1;
      y ++;
      
      if ((y - chYpos0) == size) {
        y = chYpos0;
        x ++;
        break;
      }
    }  
  }
}

//*********************************************************************************
//wyœwietlenie ³añcucha znaków - napisu 
//argumenty
//x,y - wspó³rzêdne na ekranie
//*txt - wskaŸnik na pocz¹tek bufora zwieraj¹cego ³añcuch znaków ASCII do wyœwietlenia
//size - wysokoœæ znaków 12, lub 16 pikseli 
//mode=1 znaki wyœwietlane normalnie, mode=0 znaki wyœwietlane w negatywie
//*********************************************************************************
void OLED_dispTxt(OLED_t * OLED, uint8_t x, uint8_t y, const uint8_t *txt, uint8_t size, uint8_t mode){
  while (*txt != '\0') {    
    if (x > (96 - size / 2)) {
      x = 0;
      y += size;
      if (y > (32 - size)) {
        y = x = 0;
        //DisplayCls(0x00);
      }
    }
    
    OLED_displayChar(OLED, x, y, *txt, size, mode);
    x += size / 2;
    txt ++;
  }
}

void OLED_setContrast(uint8_t value){
  OLED_SendCommand(0x81);//ustaw kontrast
  OLED_SendCommand(value);
}

void OLED_drawLine(OLED_t * OLED, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode){
  uint8_t tmp;
  uint8_t x,y;
  uint8_t dx, dy;
  int8_t err;
  int8_t ystep;
  
  uint8_t swapxy = 0;
  
  if ( x1 > x2 ) dx = x1-x2; else dx = x2-x1;
  if ( y1 > y2 ) dy = y1-y2; else dy = y2-y1;
  
  if ( dy > dx ) 
  {
    swapxy = 1;
    tmp = dx; dx =dy; dy = tmp;
    tmp = x1; x1 =y1; y1 = tmp;
    tmp = x2; x2 =y2; y2 = tmp;
  }
  if ( x1 > x2 ) 
  {
    tmp = x1; x1 =x2; x2 = tmp;
    tmp = y1; y1 =y2; y2 = tmp;
  }
  err = dx >> 1;
  if ( y2 > y1 ) ystep = 1; else ystep = -1;
  y = y1;
  
  if ( x2 == 255 ) x2--;
  
  for( x = x1; x <= x2; x++ ){
    if ( swapxy == 0 ) 
      OLED_DrawPoint(OLED, x, y, mode); 
    else 
      OLED_DrawPoint(OLED, y, x, mode); 
    err -= (uint8_t)dy;
    if ( err < 0 ){
      y += (uint8_t)ystep;
      err += (uint8_t)dx;
    }
  }
}

void OLED_drawPlotTemplate(OLED_t * OLED, char type){
  //----  X axis  ---------
  OLED_drawLine(OLED,  0, 29, 90, 29, 1);
  OLED_drawLine(OLED, 90, 29, 88, 27, 1);
  OLED_drawLine(OLED, 90, 29, 88, 31, 1);
  OLED_displayChar(OLED, 91, 20, 't', 12, 1);
    
  //---- Y axis  ----------
  OLED_drawLine(OLED, 2, 6, 2, 31, 1);
  OLED_drawLine(OLED, 2, 6, 0,  8, 1);
  OLED_drawLine(OLED, 2, 6, 4,  8, 1);
  OLED_displayChar(OLED, 4, 0, type, 12, 1);
}

void OLED_drawPlotData(OLED_t * OLED, dataset_t * data){
  
}

void datasetPrepare(dataset_t * data){
  
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
