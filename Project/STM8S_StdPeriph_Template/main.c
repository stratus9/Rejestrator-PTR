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
#include "BMP.h"
#include "ADXL.h"
#include "SPI.h"
//#include <math.h>
#include <stdlib.h>
#include "UART.h"
#include "I2C.h"
#include "Timers.h"
#include "OLED.h"
#include "FLASH.h"
//#include <stdint.h>

/* Private defines -----------------------------------------------------------*/

/* Private var ---------------------------------------------------------------*/
OLED_t OLED_buffer;
sensors_t Sensors;
bmp_t BMP;
volatile uint8_t beep = 0;
volatile uint8_t beep_trigger = 0;
state_t state_d;
FLASH_pageStruct_t FLASH_pageStruct_d;

/* Private functions ---------------------------------------------------------*/


void main(void){
  //while(1){}
  //__halt();
  //----------------Select fCPU = 16MHz--------------------------//
  CLK->CKDIVR = 0x00;    //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);      //118 B
  
  //----------------------Init UART------------------------------//
  USART_Initialization();       //1182 B -> 85 B
  
  //------------------------Init I2C-----------------------------//
  I2C_Initialization();       //605 B -> 69 B
  
  //-----------------------Init GPIO-----------------------------//
  GPIO_Init_Fast();        //263 B -> 57 B
  
  //-----------------------Init SPI------------------------------//
  SPI_Initialization();        //
   
  //----------------------Init Timers---------------------------//
  //Delay(10000);
  Timer2_Init();                //209 B -> 36 B           
  
  //----------------------Init Buzzer---------------------------//
  Beep_Initialization();      //130 B -> 12 B
  
  ISR_init();
  
  //0x3A - ADXL345 (0x1D<<1); 0xEE - BMP280 (0x77<<1);  0x3C<<1 - OLED
  OLED_Init();                  //169 B
  ADXL_init();                  //37 B
  BMP_init(&BMP);               //241 B
  FLASH_PowerUp();
  
  OLED_Clear(&OLED_buffer, 0);  //64B 
  OLED_paramTemplate(&OLED_buffer);
  //OLED_RefreshRAM(&OLED_buffer);        //6 B
  
  
  BMP_read(&BMP);       //430 B
  BMP.press = BMP.press_raw;    //inicjalizacji filtra cisnienia
  BMP.max_pressure = BMP.press;
  BMP.max_altitude = 0;
  BMP.start_altitude = 0;
  
  
  
  
   state_d.tmp = FLASH_status();
   //beep_trigger = 1;
   //FLASH_chipErase();
   //FLASH_waitForReady();
   //beep_trigger = 0;
  
  
  
  uint8_t framecount = 0;
  
  while (1){
   StateMachine();
   if(UART1->SR & 0x20){
     uint8_t tmp = UART1->DR;
     if(tmp == 0xAA) FLASH_ReadOut();
     else if(tmp == 0xEE) {
       beep_trigger = 1;
       FLASH_chipErase();
       FLASH_waitForReady();
       beep_trigger = 0;
     }
   }
   
   if(state_d.devState > 0){
     ADXL_read(&Sensors); //87 B
     BMP_read(&BMP);       //430 B
     
     //Buforowanie najwiekszego cisnienie (na starcie)
     if(state_d.devState == 1) BMP.max_pressure = BMP.press;
     
     //Wyznaczenie wzglednej wysokosci
     BMP.old_altitude = BMP.altitude;
     BMP.altitude = BMP_altitude(BMP.max_pressure, BMP.press);
     
     //Wyznaczenie predkosci lotu
     BMP_velo(&BMP);
     if(BMP.max_velocity < BMP.velocity) BMP.max_velocity = BMP.velocity;
     
     //Wyznacznie rzeczywistej wysokosci lotu
     if(state_d.devState == 1) BMP.start_altitude = BMP.altitude;
     BMP.real_altitude = (uint32_t)labs(BMP.altitude - BMP.start_altitude);
     
     //Buforowanie najwyzszego punku lotu
     if(BMP.max_altitude < BMP.real_altitude) BMP.max_altitude = BMP.real_altitude;
     
     //Buforowanie najwiekszego przyspieszenia
     if(Sensors.max_acc < labs(Sensors.accY)) Sensors.max_acc = labs(Sensors.accY);
   }
   
   //Obsluga OLED
   framecount++;
   if((framecount > 10) && (state_d.devState >= 4)){
     framecount = 0;
     OLED_dispVelocity(&OLED_buffer, 0);
     OLED_dispAcceleration(&OLED_buffer, (Sensors.max_acc > 255)?((((int32_t)Sensors.max_acc*1000)>>8)-1000):0);
     OLED_dispAltitude(&OLED_buffer, BMP.max_altitude);
    
     OLED_RefreshRAM(&OLED_buffer);
   }

   Delay(10000);       //11 B
   if(state_d.devState) LED_GREEN(1);
   //LED_GREEN(1);
   Delay(10000);    //3 B
   LED_GREEN(0); 
  }
}

//======================================================================================
//                              State machine
//======================================================================================
void StateMachine(){
  switch(state_d.devState) {
        //--------case 0 sleep-------------------------------------------
        case 0: 
        //wylacz buzzer i ledy
        //uspij
          if(state_d.button > 20){
            state_d.devState = 1;
            beep_trigger = 2;
            while(!(GPIOD->IDR & 0x02)) {}
            state_d.button = 0;
          }
        break;
                    
        //-------case 1 wait for start-----------------------------------------
        case 1: 
        //miganie zielonej diodki jako gotowosc
        LED_BLUE(1);
          //FLASH_saveData();     // <------------- wywalic tylko DEV
        if(Sensors.acc_sum_smooth > 700UL) {
          state_d.devState = 2;
          beep_trigger = 2;
        }
        break;
                
        //-------case 2 wait for landing-----------------------------------------
        case 2:
          LED_BLUE(0);
          FLASH_saveData();
        //lecimy czyli zapis parametrów do FLASH
          if((Sensors.acc_sum_smooth < 500) && (labs(BMP.real_altitude) < 2000) && (BMP.max_altitude > 1000)) {
            state_d.devState = 3;
            beep_trigger = 2;
          }
        break;
        
        //-------case 3 wait for pickup-----------------------------------------
        case 3:
        LED_BLUE(1);
        //wylacz zapis flash
        //wylacz ledy i ekran
        FLASH_saveData();
        beep_trigger = 1;
        if(state_d.button > 5) {
            state_d.devState = 4;
            beep_trigger = 2;
            while(!(GPIOD->IDR & 0x02)) {}
            state_d.button = 0;
        }
        break;
  
        //-------case 4 displ values-----------------------------------------
        case 4:
        //wypisz wynik
        beep_trigger = 0;
        if(state_d.button > 20) {
            state_d.devState = 0;
            beep_trigger = 2;
            OLED_Clear(&OLED_buffer, 0);  //64B 
            OLED_RefreshRAM(&OLED_buffer);        //6 B
            LED_GREEN(0);
            LED_BLUE(0);
            while(!(GPIOD->IDR & 0x02)) {}
            state_d.button = 0;
        }
        break;
    }
}

//======================================================================================
//                              Button ISR
//======================================================================================
void ButtonISR(){
  LED_GREEN(1);
  Delay(200000);    //3 B
  LED_GREEN(0);
  Delay(200000);    //3 B
}

void ISR_init(){
  EXTI_DeInit();
  //EXTI->CR1 = 0x80;
  //EXTI->CR2 = 0x00;
  enableInterrupts();
}

//======================================================================================
//                              GPIO
//======================================================================================
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
  
  //PD1 - switch
  GPIOD->DDR &= 0x02;  //Intput
  GPIOD->CR1 |= 0x02;   //pull-up
  GPIOD->CR2 &= 0x02;  //slow slope 2MHz
  
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


void FLASH_saveData(){
	FLASH_dataStruct_t FLASH_struct_d;
	FLASH_struct_d.marker = 0xAA;
	
	FLASH_struct_d.pressure = BMP.press;
        FLASH_struct_d.pressure_raw = BMP.press_raw;
	FLASH_struct_d.state = state_d.devState;
	FLASH_struct_d.altitude = BMP.altitude;
	FLASH_struct_d.temperature = BMP.temp;
	FLASH_struct_d.Vbat = 317;
	
	FLASH_struct_d.accX = Sensors.accX;
	FLASH_struct_d.accY = Sensors.accY;
	FLASH_struct_d.accZ = Sensors.accZ;
	FLASH_struct_d.velocity = BMP.velocity;
	
	uint8_t pagePosition = FLASH_pageStruct_d.position;
	if (pagePosition < 8){
		FLASH_pageStruct_d.FLASH_dataStruct[pagePosition] = FLASH_struct_d;
		FLASH_pageStruct_d.position++;
	}
	if(pagePosition >=8){
		FLASH_pageWrite(FLASH_pageStruct_d.pageNo, FLASH_pageStruct_d.data, 256);
		FLASH_pageStruct_d.pageNo++;
		FLASH_pageStruct_d.position = 0;
	}
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
