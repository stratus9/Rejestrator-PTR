/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include <stdlib.h>
#include "SPI.h"
#include "FLASH.h"
#include "Timers.h"
#include "OLED.h"
#include "UART.h"

//==========================================================================================================
//                              FLASH
//==========================================================================================================

void FLASH_PowerUp(){
  FLASH_CS(1);
  SPI_RWByte(0xAB);
  FLASH_CS(0);
  
  FLASH_WriteEnable(1);
  
  FLASH_CS(1);
  SPI_RWByte(0x01);
  SPI_RWByte(0x00);
  SPI_RWByte(0x00);
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

void FLASH_waitForReady(){
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
void FLASH_chipErase(){ // okolo 50s na wyczyszczenie
        FLASH_waitForReady();
	FLASH_WriteEnable(1);
        
        FLASH_CS(1);
	SPI_RWByte(0xC7);	//CHIP ERASE
	FLASH_CS(0);
}

void FLASH_blockErase(uint8_t block){   // okolo 1s na wyczyszczenie
        FLASH_waitForReady();
	FLASH_WriteEnable(1);
        
        FLASH_CS(1);
	SPI_RWByte(0xD8);	//Block erase
	SPI_RWByte(block);
        SPI_RWByte(0x00);
        SPI_RWByte(0x00);
	FLASH_CS(0);
}


void FLASH_program1byte(uint32_t address, uint8_t val){
        FLASH_waitForReady();
	FLASH_WriteEnable(1);
        
        FLASH_CS(1);
	SPI_RWByte(0x02);	//Page program
	SPI_RWByte((address>>16) & 0xFF);
	SPI_RWByte((address>>8) & 0xFF);
	SPI_RWByte(address & 0xFF);
	SPI_RWByte(val);
	FLASH_CS(0);
}

uint8_t FLASH_Read1byte(uint32_t address){
	FLASH_waitForReady();
	FLASH_CS(1);
	SPI_RWByte(0x03);	//READ
	SPI_RWByte((address>>16) & 0xFF);
	SPI_RWByte((address>>8) & 0xFF);
	SPI_RWByte(address & 0xFF);
	uint8_t tmp = SPI_RWByte(0);
	FLASH_CS(0);
        
        return tmp;
}

void FLASH_ReadOut(){
  FLASH_dataStruct_t FLASH_dataStruct;
  uint8_t string[12];
  uint32_t position = 0;
  USART_SendString("No,Press,PressRaw,State,Alti,Temp,Vbat,AccX,AccY,AccZ,Velo!\n");
  do{
    FLASH_arrayRead(position, FLASH_dataStruct.array, 32);
    if(FLASH_dataStruct.array[0] != 0xAA) break;	//je?li brak zapisanych danych, zakoñcz przepisywanie
    position += 32;
    
    FLASH_int2string(string, (position>>5));
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.pressure);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.pressure_raw);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.state);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.altitude);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.temperature);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.Vbat);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.accX*100/256);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.accY*100/256);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.accZ*100/256);
    USART_SendString(string);
    USART_SendChar(',');
    
    FLASH_int2string(string, FLASH_dataStruct.velocity);
    USART_SendString(string);
    USART_SendChar('\n');
  } while((position < 0x200000) );
  USART_SendString("Readout end!\n");
}

void FLASH_int2string(uint8_t * string, int32_t number){
  if(number < 0){
    string[0] = '-';
    number = -number;
  }
  else string[0] = '+';
  string[1] = (number/10000000 )%10 + '0';
  string[2] = (number/1000000  )%10 + '0';
  string[3] = (number/100000   )%10 + '0';
  string[4] = (number/10000    )%10 + '0';
  string[5] = (number/1000 )%10    + '0';
  string[6] = (number/100  )%10    + '0';
  string[7] = (number/10   )%10    + '0';
  string[8] = (number      )%10    + '0';
  string[9] = '\0';
}