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
//#include <math.h>
#include <stdlib.h>
//#include <stdint.h>

/* Private defines -----------------------------------------------------------*/

/* Private var ---------------------------------------------------------------*/
OLED_t OLED_buffer;
sensors_t Sensors;
bmp_t BMP;
volatile uint8_t beep = 0;
volatile uint8_t beep_trigger = 0;
//FLASH_pageStruct_t FLASH_pageStruct_d;

/* Private functions ---------------------------------------------------------*/


void main(void)
{
  //while(1){}
  //----------------Select fCPU = 16MHz--------------------------//
  CLK->CKDIVR = 0x00;    //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);      //118 B
  
  //----------------------Init UART------------------------------//
  USART_Initialization();       //1182 B -> 85 B
  
  //------------------------Init I2C-----------------------------//
  I2C_Initialization();       //605 B
  
  //-----------------------Init GPIO-----------------------------//
  GPIO_Initialization();        //263 B -> 57 B
  
  //-----------------------Init SPI------------------------------//
  SPI_Initialization();        //
   
  //----------------------Init Timers---------------------------//
  //Delay(10000);
  //Timer1_Init();              //716 B
  Timer2_Init();                //209 B
  enableInterrupts();           
  
  //----------------------Init Buzzer---------------------------//
  // Beep_Start();
  // Beep_Stop();
  Beep_Initialization();      //130 B
  
  //0x3A - ADXL345 (0x1D<<1); 0xEE - BMP280 (0x77<<1);  0x3C<<1 - OLED
  OLED_Init();                  //169 B
  ADXL_init();                  //37 B
  BMP_init(&BMP);               //241 B
  LED_GREEN(0);
  
  OLED_Clear(&OLED_buffer, 0);  //64B 
  OLED_paramTemplate(&OLED_buffer);
  OLED_RefreshRAM(&OLED_buffer);        //6 B
  
  
  FLASH_PowerUp();
  
  BMP_read(&BMP);       //430 B
  BMP.max_pressure = BMP.press_f;
  BMP.min_pressure = BMP.press_f;
  BMP.max_altitude = 0;
  
  uint8_t framecount = 0;
  while (1){
   ADXL_read(&Sensors); //87 B
   BMP_read(&BMP);       //430 B
   LED_BLUE(1);
   if(BMP.max_pressure < BMP.press_f) BMP.max_pressure = BMP.press_f;
   if(BMP.min_pressure > BMP.press_f) BMP.min_pressure = BMP.press_f;
   
   BMP.altitude = BMP_altitude(BMP.max_pressure, BMP.min_pressure);
   if(BMP.max_altitude < BMP.altitude) BMP.max_altitude = BMP.altitude;
   
   BMP.diff_pressure = BMP.max_pressure - BMP.min_pressure;
   
   //if((BMP.max_altitude > 50) && (BMP.altitude < 20)) beep_trigger = 1;
   //else beep_trigger = 0;
   
   if(Sensors.max_acc < Sensors.accY) Sensors.max_acc = Sensors.accY;
   
   framecount++;
   if(framecount > 4){
     framecount = 0;
     //OLED_dispVelocity(&OLED_buffer, BMP.press);
     //OLED_dispAcceleration(&OLED_buffer, (uint32_t)BMP.diff_pressure);
     //OLED_dispAcceleration(&OLED_buffer, labs((int32_t)Sensors.max_acc*4));
     //OLED_dispAltitude(&OLED_buffer, labs(BMP.max_altitude));
    
     //OLED_RefreshRAM(&OLED_buffer);
   }

   Delay(1000);       //11 B
   LED_BLUE(0);
   Delay(2000);    //3 B
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
//                              SPI
//==========================================================================================================
void SPI_Initialization(){
  SPI_DeInit();       //25 B
  SPI->CR1 = 0x28 | 0x04;      //MSB first, x64 prescaler, polarity Low, Master
  SPI->CR2 = 0x02 | 0x01;      //Full duplex, Software slave management enabled, Master
  /* SPI Enable */
  SPI->CR1 |= SPI_CR1_SPE; /* Enable the SPI peripheral*/
}

inline void SPI_wait(){
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET){}  
}

void SPI_sendByte(uint8_t value){
  //SPI_wait();
  while (!(SPI->SR & SPI_FLAG_TXE)){} 
  SPI->DR = value;
}

uint8_t SPI_ReadByte(){
  uint8_t Data = 0;

  /* Wait until the transmit buffer is empty */
  while (!(SPI->SR & SPI_FLAG_TXE));
  /* Send the byte */
  SPI->DR = 0x00;

  /* Wait until a data is received */
  while ((SPI->SR & SPI_FLAG_BSY));
  while (!(SPI->SR & SPI_FLAG_RXNE));
  /* Get the received data */
  Data = SPI->DR;

  /* Return the shifted data */
  return Data;
}

uint8_t SPI_RWByte(uint8_t value){
  uint8_t Data = 0;

  /* Wait until the transmit buffer is empty */
  while (!(SPI->SR & SPI_FLAG_TXE)){}
  /* Send the byte */
  SPI->DR = value;
  while (!(SPI->SR & SPI_FLAG_RXNE));
  Data = SPI->DR;
  
  SPI_ClearRXBuffer();
  /* Return the shifted data */
  return Data;
}

void SPI_ClearRXBuffer(){
  while ((SPI->SR & SPI_FLAG_RXNE)){
    SPI_ReceiveData();
  }
}

void FLASH_PowerUp(){
  FLASH_CS(1);
  SPI_RWByte(0xAB);
  FLASH_CS(0);
}

uint16_t FLASH_ReadID(){
	FLASH_CS(1);
	SPI_RWByte(0x90);
	SPI_RWByte(0x00);
	SPI_RWByte(0x00);
	SPI_RWByte(0x00);
	uint16_t tmp = SPI_RWByte(0) << 8;
	tmp |= SPI_RWByte(0);
	FLASH_CS(0);
	return tmp;
}

void FLASH_CS(uint8_t value) {
  if(value){
    GPIOA->DDR |= 0x08;   //Output
    GPIOA->CR1 |= 0x08;   //Push-Pull
    GPIOA->ODR &= ~0x08;
  }
  else{
     GPIOA->DDR |= 0x08;   //Output
    GPIOA->CR1 |= 0x08;   //Push-Pull
    GPIOA->ODR |= 0x08;
  }
}

void FLASH_1byteCommand(uint8_t value){
	FLASH_CS(1);
	SPI_RWByte(value);
	FLASH_CS(0);
}

void FLASH_WriteEnable(uint8_t value){
	if(value) FLASH_1byteCommand(0x06);
	else FLASH_1byteCommand(0x04);
}

void FLASH_arrayRead(uint32_t address, uint8_t * array, uint32_t length){
	FLASH_waitForReady();
	FLASH_CS(1);
	SPI_RWByte(0x03);	//READ
	SPI_RWByte((address>>16) & 0xFF);
	SPI_RWByte((address>>8) & 0xFF);
	SPI_RWByte(address & 0xFF);
	for (uint32_t i=0; i<length; i++){
		array[i] = SPI_RWByte(0);
	}
	FLASH_CS(0);
}

void FLASH_waitForReady(void){
	while(FLASH_status() & 0x01) Delay(1000);
}

uint8_t FLASH_status(void){
	FLASH_CS(1);
	SPI_RWByte(0x05);	//READ
	uint8_t stat = SPI_RWByte(0);
	FLASH_CS(0);
	return stat;
}

void FLASH_pageWrite(uint32_t page, uint8_t * array, uint16_t length){  //strona ma 256B
	uint32_t address = page<<8;
	FLASH_waitForReady();
	FLASH_WriteEnable(1);
	FLASH_CS(1);
	SPI_RWByte(0x02);	//Page program
	SPI_RWByte((address>>16) & 0xFF);
	SPI_RWByte((address>>8) & 0xFF);
	SPI_RWByte(address & 0xFF);
	for(uint16_t i = 0; i < length; i++){
		SPI_RWByte(*array++);
	}
	FLASH_CS(0);
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
  TIM2_TimeBaseInit(TIM2_PRESCALER_16384, 97);    //10 = X/(1024*6630) Fclk = 15.912.000 ~fcpu
  //TIM2_TimeBaseInit(TIM2_PRESCALER_16, 9945);    //~100Hz
  
  //TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  TIM2->IER |= TIM2_IT_UPDATE;

  /* TIM1 counter enable */
  TIM2->CR1 |= (uint8_t)TIM2_CR1_CEN;
}

void Timer2_ISR(){    
  if(beep_trigger){
    beep++;
    if(beep > 20) beep = 0;
    if(beep == 4) Beep_Start();
    else Beep_Stop();
  }
  else Beep_Stop();
}

//==========================================================================================================
//                              Buzzer
//==========================================================================================================

void Beep_Initialization(void){
  BEEP->CSR = (10) | 0x40;
  
  //BEEP_Init(BEEP_FREQUENCY_2KHZ);       //BEEP_FREQUENCY_1KHZ, BEEP_FREQUENCY_2KHZ, BEEP_FREQUENCY_4KHZ
  
  /* Enable the BEEP peripheral */
    BEEP->CSR |= BEEP_CSR_BEEPEN;
}

inline void Beep_Start(void){
  /* Enable the BEEP peripheral */
    BEEP->CSR |= BEEP_CSR_BEEPEN;
}

inline void Beep_Stop(void){
  BEEP->CSR &= (~BEEP_CSR_BEEPEN);
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
  //I2C_DeInit();
  //I2C_Init(300000, 0xAA, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);
  I2C_InitFast();
  I2C_Cmd(ENABLE);
}

void I2C_InitFast(){
  /*------------------------- I2C FREQ Configuration ------------------------*/
  I2C->FREQR = 16;

  /*--------------------------- I2C CCR Configuration ------------------------*/
  I2C->CR1 &= (uint8_t)(~0x01); // Disable I2C to configure TRISER

  I2C->TRISER = 6; // Set Maximum Rise Time: 300ns max in Fast Mode


  /* Write CCR with new calculated value */
  I2C->CCRL = 8;
  I2C->CCRH = 0x80;

  I2C->CR1 |= 0x01; // Enable I2C 

  /* Configure I2C acknowledgement */
  //I2C_AcknowledgeConfig(I2C_ACK_CURR);
  I2C->CR2 |= I2C_CR2_ACK;
  I2C->CR2 &= (uint8_t)(~I2C_CR2_POS);

  /*--------------------------- I2C OAR Configuration ------------------------*/
  I2C->OARL = 0xAA;
  I2C->OARH = 0x40;
}

void I2C_SendOneByte(uint8_t address, uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;    //I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = address | 0x00;                        // Transmit address+E
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->CR2 |= I2C_CR2_STOP;     //I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void I2C_SendTwoBytes(uint8_t address, uint8_t data1, uint8_t data2){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;    //I2C->CR2 |= I2C_CR2_START;
  while(!(I2C->SR1 & I2C_SR1_SB)){}
  
  I2C->SR1; // Read SR1 to clear SB bit
  I2C->DR = address | 0x00;                        // Transmit address+E
  while(!(I2C->SR1 & I2C_SR1_TXE));
  while(!(I2C->SR1 & I2C_SR1_ADDR)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data1;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->SR3;
  I2C->DR = data2;
  while(!(I2C->SR1 & I2C_SR1_TXE)){}           // Wait until address transmission is finished
  
  I2C->CR2 |= I2C_CR2_STOP;     //I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

uint8_t I2C_ReadOneByte(uint8_t address, uint8_t reg){
  uint8_t data;
  I2C_SendOneByte(address, reg);
  
  uint8_t res = 1;
    volatile uint32_t timeout;
  
    //Wait for the bus to be ready
    timeout = 0x0FFF;
    while(I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
    if (!timeout) {
      //Error
      res = 0;
      return res;
    }
    
    //Send the start condition
    I2C->CR2 |= I2C_CR2_START;  //I2C->CR2 |= I2C_CR2_START;
    timeout = 0x0FFF;
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    
    //Send the slave address
    //I2C_Send7bitAddress(address, I2C_DIRECTION_RX);
    I2C->DR = (uint8_t)(address | I2C_DIRECTION_RX);
    timeout = 0x0FFF;
    while(!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    else if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
        I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
        I2C->CR2 |= I2C_CR2_STOP;       //I2C->CR2 |= I2C_CR2_STOP;
        res = 0;
        goto stop;
    }
    
    I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
    
  stop:
    //Send the stop condition
    I2C->CR2 |= I2C_CR2_STOP;
    
    if (res) {
      //Wait for the data to be received
      timeout = 0x0FFF;
      while(!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
      if (!timeout) {
        //Error
        res = 0;
      }
      else {
        //Get the data
        data = I2C->DR;
      }
    }
  return data;
}

uint8_t I2C_ReadNByte(uint8_t address, uint8_t reg, uint8_t * data, uint8_t length){
  I2C_SendOneByte(address, reg);
  I2C->CR2 |= I2C_CR2_ACK;
  I2C->CR2 &= (uint8_t)(~I2C_CR2_POS);
  uint8_t res = 1;
    volatile uint32_t timeout;
    uint16_t i;
  
    //Wait for the bus to be ready
    timeout = 0x0FFF;
    while(timeout-- && I2C_GetFlagStatus(I2C_FLAG_BUSBUSY));
    if (!timeout) {
      //Error
      res = 0;
      return res;
    }
    
    //Send the start condition
    I2C->CR2 |= I2C_CR2_START;
    timeout = 0x0FFF;
    while(timeout-- && !I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    
    //Send the slave address
    //I2C_Send7bitAddress(address, I2C_DIRECTION_RX);
    I2C->DR = (uint8_t)(address | I2C_DIRECTION_RX);
    timeout = 0x0FFF;
    while(timeout-- && !I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    if (!timeout) {
      //Error
      res = 0;
      goto stop;
    }
    else if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
        I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
        I2C->CR2 |= I2C_CR2_STOP;
        res = 0;
        goto stop;
    }
    
    for (i=0; i<length; i++) {
      if (i == length - 1) {
        I2C->CR2 &= (uint8_t)(~I2C_CR2_ACK);
        //Send the stop condition
        I2C->CR2 |= I2C_CR2_STOP;
      }

      //Wait for the data to be received
      timeout = 0x0FFF;
      while(timeout-- && !I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED));
      if (!timeout) {
        //Error
        res = 0;
      }
      else {
        //Get the data
        *data = I2C->DR;
        data++;
      }
    }
    
   
  stop:
    //If something bad happened, send the stop condition
    if (!res)
      I2C->CR2 |= I2C_CR2_STOP;
    
    return res;
}

//==========================================================================================================
//                              OLED
//==========================================================================================================
void OLED_Init(){
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
}

void OLED_Reset(){
  GPIOD->ODR &= ~0x04;   //Enable I2C Pull-down
  Delay(100000);
  GPIOD->ODR |= 0x04;   //Enable I2C Pull-up
  Delay(100000);
}

void OLED_SendCommand(uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;
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
  
  I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
  while(!(I2C->CR2 & I2C_CR2_STOP)){}
}

void OLED_SendData(uint8_t data){
  while(I2C->SR3 & I2C_SR3_BUSY); //wait for not busy
  
  I2C->CR2 |= I2C_CR2_START;
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
  
  I2C->CR2 |= I2C_CR2_STOP;                    //Generate STOP
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

void OLED_SetColStart(){ 
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
void OLED_displayChar(OLED_t * OLED, uint8_t x, uint8_t y, uint8_t Chr){    
  uint8_t i, j;
  uint8_t chTemp, chYpos0 = y;
  
  //Chr = Chr - ' ' + 1;    
  Chr = OLED_charDecoder(Chr);
  for (i = 0; i < 12; i ++) {  
        chTemp = c_chFont1206_lite[Chr][i];

    for (j = 0; j < 8; j ++) {
      if (chTemp & 0x80) {
        OLED_DrawPoint(OLED, x, y, 1);
      } else {
        OLED_DrawPoint(OLED, x, y, 0);
      }
      chTemp <<= 1;
      y ++;
      
      if ((y - chYpos0) == 12) {
        y = chYpos0;
        x ++;
        break;
      }
    }  
  }
}

uint8_t OLED_charDecoder(char znak){
  if(znak == ' ') return 0;
  if((znak >= '0') && (znak <= '9')) return znak-41;
  if(znak == '!') return 1;
  if(znak == '%') return 2;
  if(znak == '+') return 3;
  if(znak == '-') return 4;
  if(znak == '.') return 5;
  if(znak == '/') return 6;
  if(znak == ':') return 17;
  if(znak == 'A') return 18;
  if(znak == 'H') return 19;
  if(znak == 'P') return 20;
  if(znak == 'V') return 21;
  if(znak == '^') return 22;
  if(znak == 'a') return 23;
  if(znak == 'm') return 24;
  if(znak == 'x') return 25;
  if(znak == 's') return 26;
  return 0;
}

//*********************************************************************************
//wyœwietlenie ³añcucha znaków - napisu 
//argumenty
//x,y - wspó³rzêdne na ekranie
//*txt - wskaŸnik na pocz¹tek bufora zwieraj¹cego ³añcuch znaków ASCII do wyœwietlenia
//size - wysokoœæ znaków 12, lub 16 pikseli 
//mode=1 znaki wyœwietlane normalnie, mode=0 znaki wyœwietlane w negatywie
//*********************************************************************************
void OLED_dispTxt(OLED_t * OLED, uint8_t x, uint8_t y,  uint8_t *txt){
  while (*txt != '\0') {    
    
    OLED_displayChar(OLED, x, y, *txt);
    x += 6;
    txt ++;
  }
}

void OLED_setContrast(uint8_t value){
  OLED_SendCommand(0x81);//ustaw kontrast
  OLED_SendCommand(value);
}

void OLED_int2string(char * string, uint32_t number){
  string[0] = (number>99999)?( number/100000 %10 + '0'):(' ');
  string[1] = (number>9999 )?((number/10000)%10 + '0'):(' ');
  string[2] = (number>999  )?((number/1000 )%10 + '0'):(' ');
  string[3] = (number/100  )%10 + '0';
  string[4] = '.';
  string[5] = (number/10   )%10 + '0';
  string[6] = '\0';
}

void OLED_paramTemplate(OLED_t * OLED){
  OLED_dispTxt(OLED, 0, 0,"Vmax:       m/s");
  OLED_dispTxt(OLED, 0,10,"Amax:       m/s^");
  OLED_dispTxt(OLED, 0,21,"Hmax:       m");
}

void OLED_dispInt(OLED_t * OLED, uint8_t x, uint8_t y, int32_t value){
  char bufor[7];
  OLED_int2string(bufor, value);
  
  OLED_dispTxt(OLED,x,y,bufor);
}

inline void OLED_dispVelocity(OLED_t * OLED, uint32_t value){
  OLED_dispInt(OLED, 32, 0, value);
}

inline void OLED_dispAcceleration(OLED_t * OLED, uint32_t value){
  OLED_dispInt(OLED, 32, 10, value);
}

inline void OLED_dispAltitude(OLED_t * OLED, uint32_t value){
  OLED_dispInt(OLED, 32, 21, value);
}

/*
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
*/

/*
void OLED_drawPlotTemplate(OLED_t * OLED, char type){
  //----  X axis  ---------
  OLED_drawLine(OLED,  0, 29, 90, 29, 1);
  OLED_drawLine(OLED, 90, 29, 88, 27, 1);
  OLED_drawLine(OLED, 90, 29, 88, 31, 1);
  OLED_displayChar(OLED, 91, 20, 't');
    
  //---- Y axis  ----------
  OLED_drawLine(OLED, 2, 6, 2, 31, 1);
  OLED_drawLine(OLED, 2, 6, 0,  8, 1);
  OLED_drawLine(OLED, 2, 6, 4,  8, 1);
  OLED_displayChar(OLED, 4, 0, type);
}
*/

void OLED_drawPlotData(OLED_t * OLED, dataset_t * data){
  
}

void datasetPrepare(dataset_t * data){
  
}

//==========================================================================================================
//                              Dev
//==========================================================================================================
/*
void dev_CheckSensors(){
  OLED_Clear(&OLED_buffer, 0);
  
  if(I2C_ReadOneByte(0xEE, 0xD0) == 0x58) OLED_dispTxt(&OLED_buffer,0,0,"BMP  OK");
  else OLED_dispTxt(&OLED_buffer,0,0,"BMP  NOPE");
  
  if(I2C_ReadOneByte(0x3A, 0x00) == 0xE5) OLED_dispTxt(&OLED_buffer,0,12,"ADXL OK");
  else OLED_dispTxt(&OLED_buffer,0,12,"ADXL  NOPE");
  
  OLED_RefreshRAM(&OLED_buffer);
}
*/
//==========================================================================================================
//                              BMP280
//==========================================================================================================
void BMP_init(bmp_t * BMP) {
  uint8_t buffer[25];
  I2C_ReadNByte(0xEE, 0x88, buffer, 24);
  BMP->T1 = buffer[0] | (buffer[1] << 8);
  BMP->T2 = buffer[2] | (buffer[3] << 8);
  BMP->T3 = buffer[4] | (buffer[5] << 8);
  BMP->P1 = buffer[6] | (buffer[7] << 8);
  BMP->P2 = buffer[8] | (buffer[9] << 8);
  BMP->P3 = buffer[10] |(buffer[11] << 8);
  BMP->P4 = buffer[12] | (buffer[13] << 8);
  BMP->P5 = buffer[14] | (buffer[15] << 8);
  BMP->P6 = buffer[16] | (buffer[17] << 8);
  BMP->P7 = buffer[18] | (buffer[19] << 8);
  BMP->P8 = buffer[20] | (buffer[21] << 8);
  BMP->P9 = buffer[22] | (buffer[23] << 8);
  
  /*
  BMP->T1 = 27504;      //buffer[0] | (buffer[1] << 8);
  BMP->T2 = 26435;      //buffer[2] | (buffer[3] << 8);
  BMP->T3 = -1000;      //buffer[4] | (buffer[5] << 8);
  BMP->P1 = 36477;      //buffer[6] | (buffer[7] << 8);
  BMP->P2 = -10685;     //buffer[8] | (buffer[9] << 8);
  BMP->P3 = 3024;       //buffer[10] |(buffer[11] << 8);
  BMP->P4 = 2855;       //buffer[12] | (buffer[13] << 8);
  BMP->P5 = 140;        //buffer[14] | (buffer[15] << 8);
  BMP->P6 = -7;         //buffer[16] | (buffer[17] << 8);
  BMP->P7 = 15500;      //buffer[18] | (buffer[19] << 8);
  BMP->P8 = -14600;     //buffer[20] | (buffer[21] << 8);
  BMP->P9 = 6000;       //buffer[22] | (buffer[23] << 8);
  */
  
  I2C_SendTwoBytes(0xEE, 0xF4, 0x3F);
}

void BMP_read(bmp_t * BMP) {
  uint8_t buffer[7];
  I2C_ReadNByte(0xEE, 0xF7, buffer, 6);
  
  BMP->UP = buffer[0];
  BMP->UP = BMP->UP << 8 | buffer[1];
  BMP->UP = BMP->UP << 8 | buffer[2];
  BMP->UP >>= 4;
  
  BMP->UT = buffer[3];
  BMP->UT = BMP->UT << 8 | buffer[4];
  BMP->UT = BMP->UT << 8 | buffer[5];
  BMP->UT >>= 4;
  
  //--------- Temp -------------
  int32_t var1, var2, tfine;
  var1  = ((((BMP->UT>>3) - ((int32_t)BMP->T1 <<1))) * ((int32_t)BMP->T2)) >> 11;
  var2  = (((((BMP->UT>>4) - ((int32_t)BMP->T1)) * ((BMP->UT>>4) - ((int32_t)BMP->T1))) >> 12) * ((int32_t)BMP->T3)) >> 14;
  
  tfine = var1 + var2;
  BMP->temp = (tfine)>>9;
  
  //--------- Press -----------
  float fvar1=0.0, fvar2=0.0, p=0.0;
  fvar1 = (float)tfine/2.0 - 64000.0;
  fvar2 = fvar1 * fvar1*((double)BMP->P6)/32768.0;
  fvar2 = fvar2 + fvar1*((float)BMP->P5)*2.0;
  fvar2 = (fvar2/4.0) + ((float)BMP->P4)*65536.0;
  fvar1 = (((float)BMP->P3) * fvar1 * fvar1/524288.0 + ((float)BMP->P2) * fvar1)/524288.0;
  fvar1 = (1.0 + fvar1/32768.0)*((float)BMP->P1);
  p = 1048576.0-((float)BMP->UP);
  p = (p-(fvar2/4096.0))*6250.0 / fvar1;
  fvar1 = ((float)BMP->P9)*p*p/2147483648.0;
  fvar2 = p*((float)BMP->P8)/32768.0;
  p = p + (fvar1 + fvar2+((float)BMP->P7))/16.0;
  BMP->press = (uint32_t)p;
  BMP->press_f = p;
}

float BMP_altitude(float startPress, float currPress){
   float x1, x2, x3, x4;
   //return (1.0-powf(currPress/startPress, 0.1902632365))*43538.0;
   x1 = (float)currPress/(float)startPress;
   x2 = -4863.0*x1*x1*x1;
   x3 =  16830.0*x1*x1;
   x4 = -27490.0*x1;
   return (x2 + x3 + x4 + 15520.0)*100.0;
}

//==========================================================================================================
//                              ADXL345
//==========================================================================================================
void ADXL_init(){
  I2C_SendTwoBytes(0x3A, 0x2D, 0x00);
  I2C_SendTwoBytes(0x3A, 0x2D, 0x10);
  I2C_SendTwoBytes(0x3A, 0x2D, 0x08);
  I2C_SendTwoBytes(0x3A, 0x31, 0x0B);   // +/- 16g Full ress
}

void ADXL_read(sensors_t * sensor){
  uint8_t buffer[7];
  I2C_ReadNByte(0x3A, 0x32, buffer, 6);
  sensor->accX = buffer[1]<<8 | buffer[0];
  sensor->accY = buffer[3]<<8 | buffer[2];
  sensor->accZ = buffer[5]<<8 | buffer[4];
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
